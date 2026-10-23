#include "config.h"
#include "sim.h"
#include "devices.h"
#include "mmu.h"
#include "platform.h"
#include "processor.h"
#include "trap.h"
#include "disasm.h"
#include <cosim/target/Api.h>

#include <atomic>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#if defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#endif
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {
[[gnu::used]] volatile const char* force_disassembler_link = fpr_name[0];

constexpr cosim_region_spec_t regions[] = {
    {0x80000000ULL, 0x7fff000ULL, "ram_cached", "ram", "ram", 7, 0, 1},
    {0x87fff000ULL, 0x1000ULL, "ram_uncached", "ram", "ram", 7, 0, 0},
    {0x12002000ULL, 0x1000ULL, "uart", "uart", "dw_apb_uart", 3, 1, 0},
};
constexpr cosim_platform_spec_t platform = {
    regions, sizeof(regions) / sizeof(regions[0]), 0x80000000ULL};
constexpr cosim_clock_domain_desc_t clocks[] = {{"cpu", 1000000000ULL}};
constexpr cosim_tl_master_port_desc_t masters[] = {
    {"cpu", 0, COSIM_TL_ADDRESS_BITS, COSIM_TL_DATA_BITS, COSIM_TL_SOURCE_BITS,
     COSIM_TL_REQUIRED_CAPABILITIES},
};
constexpr cosim_simulation_description_t description = {
    &platform, clocks, 1, masters, 1, COSIM_TL_REQUIRED_CAPABILITIES};

class HostMemory final : public abstract_mem_t {
    reg_t length;
public:
    explicit HostMemory(reg_t length) : length(length) {}
    bool load(reg_t, size_t, uint8_t*) override { return false; }
    bool store(reg_t, size_t, const uint8_t*) override { return false; }
    reg_t size() override { return length; }
    char* contents(reg_t) override { return nullptr; }
    void dump(std::ostream&) override {}
};

using Transfer = std::function<cosim_tl_dbeat_t(const cosim_tl_request_t&)>;

class HostSim final : public sim_t {
    Transfer transfer_;
    static uint64_t unpack(const uint8_t* bytes, size_t size) {
        uint64_t value = 0;
        std::memcpy(&value, bytes, size);
        return value;
    }
    bool read(reg_t address, size_t size, bool fetch, uint8_t* bytes) const {
        if (!size || size > sizeof(uint64_t) || (size & (size - 1))) return false;
        const auto width = uint64_t(size);
        const uint64_t aligned = address & ~(width - 1);
        cosim_tl_request_t request{};
        request.address = aligned;
        request.opcode = COSIM_TL_A_GET;
        request.param = fetch ? COSIM_TL_A_PARAM_FETCH : 0;
        request.size = static_cast<uint8_t>(__builtin_ctzll(size));
        request.source = 0;
        request.burstLast = 1;
        const auto response = transfer_(request);
        if (response.denied) return false;
        const uint64_t value = response.data >> ((aligned & 7) * 8);
        std::memcpy(bytes, &value, size);
        return true;
    }
public:
    HostSim(const cfg_t* cfg, std::vector<std::pair<reg_t, abstract_mem_t*>> mems,
            Transfer transfer)
        : sim_t(cfg, false, std::move(mems), {}, false, {"none"}, debug_module_config_t{}, "/dev/null",
                false, nullptr, false, nullptr, std::nullopt), transfer_(std::move(transfer)) {}

    char* addr_to_mem(reg_t) override { return nullptr; }
    bool mmio_fetch(reg_t address, size_t size, uint8_t* bytes) override {
        return read(address, size, true, bytes);
    }
    bool mmio_load(reg_t address, size_t size, uint8_t* bytes) override {
        return read(address, size, false, bytes);
    }
    bool mmio_store(reg_t address, size_t size, const uint8_t* bytes) override {
        if (!size || size > sizeof(uint64_t) || (size & (size - 1))) return false;
        const unsigned lane = unsigned(address & 7);
        if (lane + size > sizeof(uint64_t)) return false;
        cosim_tl_request_t request{};
        request.address = address;
        // Spike always writes the full access size, so the mask is contiguous.
        request.opcode = COSIM_TL_A_PUT_FULL_DATA;
        request.size = static_cast<uint8_t>(__builtin_ctzll(size));
        request.source = 0;
        request.data = unpack(bytes, size) << (lane * 8);
        request.mask = uint8_t(((uint64_t(1) << size) - 1) << lane);
        request.burstLast = 1;
        return !transfer_(request).denied;
    }
};

class SpikeSimulation {
    enum class State : uint8_t { Idle, Running, Blocked, Completed, Failed, Stopped };

    // The main thread and the single worker form an SPSC pair: every handshake
    // field is atomic and the retired stream is a bounded lock-free SPSC ring,
    // so the adapter needs no mutex or condition variable. Waits spin with
    // pause; futex latency would dominate at one wake per TL beat.
    static constexpr size_t RetiredCapacity = 4096;

    std::unique_ptr<cfg_t> cfg;
    std::vector<std::unique_ptr<HostMemory>> storage;
    std::vector<std::pair<reg_t, abstract_mem_t*>> mems;
    std::unique_ptr<HostSim> sim;
    cosim::TlMaster master;
    std::thread worker;
    std::atomic<State> state{State::Idle};
    std::atomic<bool> taskRequested{false};
    std::atomic<bool> stopRequested{false};
    // Written by the worker before publishing State::Failed; read after acquire.
    std::string failure;
    cosim_retired_instruction_t retired[RetiredCapacity];
    std::atomic<size_t> retiredHead{0};
    std::atomic<size_t> retiredTail{0};
    // TL handle the worker is parked on; read by onClockEdge to skip futile wakeups.
    std::atomic<cosim_request_handle_t> blockedHandle{0};

    static void pause() {
#if defined(__x86_64__) || defined(__i386__)
        _mm_pause();
#endif
    }

    // Every predicate below is published by the peer thread, so spinning never
    // blocks the progress it waits for. Handoff waits resolve within a few
    // hundred microseconds (one DUT edge per TL beat), while a sched_yield on
    // this class of host costs tens of microseconds of kernel time and invites
    // the peer to fall off its own spin window, ping-ponging into a yield
    // storm. Spin with pause for a budget two orders of magnitude past the
    // worst normal wait (~1M pauses = ~14ms at ~14ns/pause); yield only as an
    // escape hatch for a permanently stalled peer.
    static constexpr unsigned SpinBudget = 1u << 20;
    template <typename Pred>
    void waitFor(Pred pred) {
        for (unsigned spin = 0;; ++spin) {
            if (pred()) return;
            if (spin < SpinBudget) pause();
            else std::this_thread::yield();
        }
    }

    void setFailed(const std::string& reason) {
        failure = reason;
        state.store(State::Failed, std::memory_order_release);
    }

    cosim_tl_dbeat_t transfer(const cosim_tl_request_t& request) {
        const cosim::RequestHandle handle = master.submit(request);
        if (!handle) throw std::runtime_error("Spike TL request was rejected");
        while (true) {
            cosim_tl_dbeat_t beat{};
            if (master.peek(handle, beat)) {
                master.consume(handle);
                return beat;
            }
            blockedHandle.store(handle.value(), std::memory_order_release);
            state.store(State::Blocked, std::memory_order_release);
            waitFor([&] {
                return state.load(std::memory_order_acquire) == State::Running ||
                    stopRequested.load(std::memory_order_acquire);
            });
            blockedHandle.store(0, std::memory_order_release);
            if (stopRequested.load(std::memory_order_acquire)) {
                master.cancel(handle);
                throw std::runtime_error("Spike simulation stopped while waiting for TL");
            }
        }
    }

    cosim_retired_instruction_t retiredInstruction(
        processor_t* core, reg_t pc, insn_t instruction) const {
        auto* coreState = core->get_state();
        cosim_retired_instruction_t event{};
        event.pc = pc;
        event.instruction = static_cast<unsigned>(instruction.bits());
        event.rawInstruction = static_cast<unsigned>(instruction.bits());
        event.compressed = instruction.length() == 2;
        for (const auto& item : coreState->log_reg_write) {
            if ((item.first & 0xf) == 0 && (item.first >> 4) != 0) {
                event.destinationValid = 1;
                event.destination = static_cast<unsigned>(item.first >> 4);
                event.data = item.second.v[0];
                break;
            }
        }
        const auto& reads = coreState->log_mem_read;
        const auto& writes = coreState->log_mem_write;
        if (!writes.empty()) {
            event.memoryValid = 1;
            event.memoryWrite = 1;
            event.memoryAddress = std::get<0>(writes.front());
            event.memoryData = std::get<1>(writes.front());
            event.memorySize = static_cast<unsigned>(__builtin_ctz(std::get<2>(writes.front())));
            event.memoryMask = uint8_t((1u << (1u << event.memorySize)) - 1u);
        } else if (!reads.empty()) {
            event.memoryValid = 1;
            event.memoryAddress = std::get<0>(reads.front());
            event.memorySize = static_cast<unsigned>(__builtin_ctz(std::get<2>(reads.front())));
            event.memoryMask = uint8_t((1u << (1u << event.memorySize)) - 1u);
        }
        return event;
    }

    void runInstruction() {
        try {
            auto* core = sim->get_core(0);
            auto* coreState = core->get_state();
            const reg_t pc = coreState->pc;
            const auto fetch = core->get_mmu()->load_insn(pc);
            const auto before = coreState->minstret->read();
            core->step(1);
            if (coreState->minstret->read() != before) {
                auto event = retiredInstruction(core, pc, fetch.insn);
                const size_t tail = retiredTail.load(std::memory_order_relaxed);
                // The runtime drains after every edge, so a full ring means the
                // DUT stopped pairing; wait for space rather than dropping events.
                waitFor([&] {
                    return retiredHead.load(std::memory_order_acquire) + RetiredCapacity != tail ||
                        stopRequested.load(std::memory_order_acquire);
                });
                if (retiredHead.load(std::memory_order_acquire) + RetiredCapacity != tail) {
                    retired[tail % RetiredCapacity] = event;
                    retiredTail.store(tail + 1, std::memory_order_release);
                }
            }
            state.store(State::Completed, std::memory_order_release);
        } catch (const std::exception& error) {
            setFailed(error.what());
        }
    }

    void workerMain() {
        while (true) {
            waitFor([&] {
                return state.load(std::memory_order_acquire) == State::Running ||
                    stopRequested.load(std::memory_order_acquire);
            });
            if (stopRequested.load(std::memory_order_acquire)) break;
            if (taskRequested.exchange(false, std::memory_order_acq_rel)) runInstruction();
        }
    }

public:
    explicit SpikeSimulation(const cosim_simulation_config_t* input) : cfg(new cfg_t), master(nullptr) {
        if (!input || !input->platform || !input->masters || input->masterCount != 1 ||
            !input->transports || input->transportCount != 1)
            throw std::invalid_argument("invalid Spike cosim configuration");
        if (input->platform->regionCount != sizeof(regions) / sizeof(regions[0]))
            throw std::invalid_argument("Spike platform region count does not match");
        master = cosim::TlMaster(&input->transports[0]);
        cfg->isa = DEFAULT_ISA;
        cfg->priv = DEFAULT_PRIV;
        cfg->start_pc.set_global(input->platform->resetPC);
        cfg->hartids = {0};
        cfg->explicit_hartids = false;
        cfg->mem_layout.clear();
        for (size_t i = 0; i < input->platform->regionCount; ++i) {
            const auto& region = input->platform->regions[i];
            if (region.device) continue;
            cfg->mem_layout.emplace_back(region.base, region.size);
            storage.push_back(std::make_unique<HostMemory>(region.size));
            mems.emplace_back(region.base, storage.back().get());
        }
        sim = std::make_unique<HostSim>(cfg.get(), mems,
            [this](const cosim_tl_request_t& request) { return transfer(request); });
        sim->configure_log(false, true);
        worker = std::thread([this] { workerMain(); });
    }

    ~SpikeSimulation() {
        stopRequested.store(true, std::memory_order_release);
        state.store(State::Running, std::memory_order_release);
        if (worker.joinable()) worker.join();
    }

    int reset(unsigned) {
        const State current = state.load(std::memory_order_acquire);
        if (current != State::Idle && current != State::Completed && current != State::Failed)
            return 1;
        retiredHead.store(0, std::memory_order_release);
        retiredTail.store(0, std::memory_order_release);
        taskRequested.store(false, std::memory_order_release);
        sim->get_core(0)->reset();
        return 0;
    }

    int onClockEdge(unsigned domain) {
        if (domain != 0) return 1;
        const State current = state.load(std::memory_order_acquire);
        if (current == State::Failed || current == State::Stopped) return 1;
        if (current == State::Blocked) {
            // The worker cannot make progress until this exact D beat lands, so
            // peek from here and leave it parked while the beat is not ready.
            const cosim_request_handle_t handle = blockedHandle.load(std::memory_order_acquire);
            if (!handle) return 0;
            cosim_tl_dbeat_t beat{};
            if (!master.peek(cosim::RequestHandle(handle), beat)) return 0;
        } else if (current == State::Idle || current == State::Completed) {
            taskRequested.store(true, std::memory_order_release);
        }
        state.store(State::Running, std::memory_order_release);
        waitFor([&] {
            const State next = state.load(std::memory_order_acquire);
            return next == State::Blocked || next == State::Completed ||
                next == State::Failed || stopRequested.load(std::memory_order_acquire);
        });
        return state.load(std::memory_order_acquire) == State::Failed ? 1 : 0;
    }

    int pollEvent(cosim_simulation_event_t* event) {
        if (!event) return -1;
        if (state.load(std::memory_order_acquire) == State::Failed) {
            event->type = COSIM_EVENT_FAILED;
            std::snprintf(event->reason, sizeof(event->reason), "%s", failure.c_str());
            return 1;
        }
        const size_t head = retiredHead.load(std::memory_order_relaxed);
        if (head != retiredTail.load(std::memory_order_acquire)) {
            event->type = COSIM_EVENT_RETIRED;
            event->instruction = retired[head % RetiredCapacity];
            retiredHead.store(head + 1, std::memory_order_release);
            return 1;
        }
        return 0;
    }

    int finished() const {
        return state.load(std::memory_order_acquire) == State::Failed;
    }
};

COSIM_EXTERN_C_LOCAL int describe(cosim_simulation_description_t* output) {
    if (!output || !fpr_name) return 1;
    *output = description;
    return 0;
}

COSIM_EXTERN_C_LOCAL void* create(const cosim_simulation_config_t* config,
                                  const cosim_target_run_options_t*) {
    try {
        return new SpikeSimulation(config);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "[spike-cosim] create failed: %s\n", error.what());
        return nullptr;
    } catch (...) {
        std::fprintf(stderr, "[spike-cosim] create failed: unknown exception\n");
        return nullptr;
    }
}

COSIM_EXTERN_C_LOCAL int reset(void* simulation, unsigned edges) {
    return static_cast<SpikeSimulation*>(simulation)->reset(edges);
}

COSIM_EXTERN_C_LOCAL int onClockEdge(void* simulation, unsigned domain, cosim_sim_time_t) {
    return static_cast<SpikeSimulation*>(simulation)->onClockEdge(domain);
}

COSIM_EXTERN_C_LOCAL int pollEvent(void* simulation, cosim_simulation_event_t* event) {
    return static_cast<SpikeSimulation*>(simulation)->pollEvent(event);
}

COSIM_EXTERN_C_LOCAL int finished(void* simulation) {
    return static_cast<SpikeSimulation*>(simulation)->finished();
}

COSIM_EXTERN_C_LOCAL void destroy(void* simulation) {
    delete static_cast<SpikeSimulation*>(simulation);
}
}

COSIM_EXTERN_C const cosim_simulation_api_t cosim_simulation_api = {
    COSIM_SIMULATION_API_VERSION, "spike", describe, create, reset, onClockEdge,
    pollEvent, finished, destroy};
