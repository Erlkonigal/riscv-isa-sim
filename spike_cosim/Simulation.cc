// Spike reference adapter for cosim ABI v9. Spike runs synchronously on the
// runner thread: step() executes until exactly one instruction retires. All RAM
// traffic goes to the runtime-owned golden memory and all device traffic is
// replayed against the DUT MMIO trace; Spike never sees the TL bus.
#include "config.h"
#include "sim.h"
#include "devices.h"
#include "mmu.h"
#include "platform.h"
#include "processor.h"
#include "trap.h"
#include "disasm.h"
#include <cosim/target/Api.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
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
constexpr cosim_ref_description_t description = {&platform};

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

class HostSim final : public sim_t {
public:
    HostSim(const cfg_t* cfg, std::vector<std::pair<reg_t, abstract_mem_t*>> mems,
            cosim_ref_memory_ops_t memory, cosim_ref_mmio_ops_t mmio,
            const cosim_platform_spec_t* platform)
        : sim_t(cfg, false, std::move(mems), {}, false, {"none"}, debug_module_config_t{}, "/dev/null",
                false, nullptr, false, nullptr, std::nullopt),
          memory_(memory), mmio_(mmio) {
        for (size_t i = 0; i < platform->regionCount; ++i) {
            const auto& region = platform->regions[i];
            if (!region.size) continue;
            (region.device ? devices_ : ram_).emplace_back(region.base, region.base + region.size);
        }
    }

    char* addr_to_mem(reg_t) override { return nullptr; }
    bool mmio_fetch(reg_t address, size_t size, uint8_t* bytes) override {
        // Instruction fetches hit golden memory only; a fetch into a device
        // region raises a fetch access fault, mirroring the DUT bus denial.
        return route(address, size) == Route::Golden && readGolden(address, size, bytes);
    }
    bool mmio_load(reg_t address, size_t size, uint8_t* bytes) override {
        const Route target = route(address, size);
        if (target == Route::Golden) return readGolden(address, size, bytes);
        if (target != Route::Mmio || !validSize(size)) return false;
        uint64_t value = 0;
        if (mmio_.load(mmio_.opaque, address,
                       static_cast<unsigned>(__builtin_ctzll(size)), &value) != 1)
            return false;
        std::memcpy(bytes, &value, size);
        return true;
    }
    bool mmio_store(reg_t address, size_t size, const uint8_t* bytes) override {
        if (!validSize(size)) return false;
        const Route target = route(address, size);
        if (target == Route::Golden)
            return memory_.write(memory_.opaque, address, size, bytes) == 0;
        if (target != Route::Mmio) return false;
        uint64_t value = 0;
        std::memcpy(&value, bytes, size);
        return mmio_.store(mmio_.opaque, address,
                           static_cast<unsigned>(__builtin_ctzll(size)), value,
                           uint8_t((uint64_t(1) << size) - 1u)) == 1;
    }

private:
    enum class Route : uint8_t { Golden, Mmio, Fault };

    static bool validSize(size_t size) {
        return size && size <= sizeof(uint64_t) && !(size & (size - 1));
    }
    Route route(reg_t address, size_t size) const {
        if (!size) return Route::Fault;
        for (const auto& span : ram_)
            if (address >= span.first && address <= span.second && size <= span.second - address)
                return Route::Golden;
        for (const auto& span : devices_)
            if (address >= span.first && address <= span.second && size <= span.second - address)
                return Route::Mmio;
        return Route::Fault;
    }
    bool readGolden(reg_t address, size_t size, uint8_t* bytes) const {
        if (!validSize(size)) return false;
        return memory_.read(memory_.opaque, address, size, bytes) == 0;
    }

    cosim_ref_memory_ops_t memory_;
    cosim_ref_mmio_ops_t mmio_;
    std::vector<std::pair<uint64_t, uint64_t>> ram_;
    std::vector<std::pair<uint64_t, uint64_t>> devices_;
};

class SpikeSimulation {
    // step() must always make progress; bound the executions between
    // retirements so a pathological trap loop surfaces as a failure.
    static constexpr unsigned MaxExecutionsPerStep = 1u << 20;

    std::unique_ptr<cfg_t> cfg;
    std::vector<std::unique_ptr<HostMemory>> storage;
    std::vector<std::pair<reg_t, abstract_mem_t*>> mems;
    std::unique_ptr<HostSim> sim;
    bool failed = false;
    std::string failure;

    static cosim_retired_instruction_t retiredInstruction(
        processor_t* core, reg_t pc, insn_t instruction) {
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

public:
    explicit SpikeSimulation(const cosim_ref_config_t* input) : cfg(new cfg_t) {
        if (!input || !input->platform || !input->memory.read || !input->memory.write ||
            !input->mmio.load || !input->mmio.store)
            throw std::invalid_argument("invalid Spike cosim configuration");
        if (input->platform->regionCount != sizeof(regions) / sizeof(regions[0]))
            throw std::invalid_argument("Spike platform region count does not match");
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
        sim = std::make_unique<HostSim>(cfg.get(), mems, input->memory, input->mmio,
                                        input->platform);
        sim->configure_log(false, true);
    }

    int reset() {
        failed = false;
        failure.clear();
        sim->get_core(0)->reset();
        return 0;
    }

    int step(cosim_ref_step_result_t* result) {
        if (!result) return -1;
        if (failed) {
            result->status = COSIM_REF_STEP_FAILED;
            std::snprintf(result->reason, sizeof(result->reason), "%s", failure.c_str());
            return 0;
        }
        try {
            auto* core = sim->get_core(0);
            auto* coreState = core->get_state();
            for (unsigned executions = 0; executions < MaxExecutionsPerStep; ++executions) {
                const reg_t pc = coreState->pc;
                const auto fetch = core->get_mmu()->load_insn(pc);
                const auto before = coreState->minstret->read();
                core->step(1);
                if (coreState->minstret->read() == before) continue;
                result->status = COSIM_REF_STEP_RETIRED;
                result->instruction = retiredInstruction(core, pc, fetch.insn);
                return 0;
            }
            failure = "reference executed without retiring";
        } catch (const std::exception& error) {
            failure = error.what();
        } catch (...) {
            failure = "unknown reference exception";
        }
        failed = true;
        result->status = COSIM_REF_STEP_FAILED;
        std::snprintf(result->reason, sizeof(result->reason), "%s", failure.c_str());
        return 0;
    }

    int finished() const { return failed ? 1 : 0; }
};

COSIM_EXTERN_C_LOCAL int describe(cosim_ref_description_t* output) {
    if (!output || !fpr_name) return 1;
    *output = description;
    return 0;
}

COSIM_EXTERN_C_LOCAL void* create(const cosim_ref_config_t* config) {
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

COSIM_EXTERN_C_LOCAL int reset(void* simulation) {
    return static_cast<SpikeSimulation*>(simulation)->reset();
}

COSIM_EXTERN_C_LOCAL int step(void* simulation, cosim_ref_step_result_t* result) {
    return static_cast<SpikeSimulation*>(simulation)->step(result);
}

COSIM_EXTERN_C_LOCAL int finished(void* simulation) {
    return static_cast<SpikeSimulation*>(simulation)->finished();
}

COSIM_EXTERN_C_LOCAL void destroy(void* simulation) {
    delete static_cast<SpikeSimulation*>(simulation);
}
}

COSIM_EXTERN_C const cosim_ref_api_t cosim_ref_api = {
    COSIM_REF_API_VERSION, "spike", describe, create, reset, step, finished, destroy};
