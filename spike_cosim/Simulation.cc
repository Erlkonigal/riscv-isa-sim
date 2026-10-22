#include "config.h"
#include "sim.h"
#include "devices.h"
#include "mmu.h"
#include "platform.h"
#include "processor.h"
#include "trap.h"
#include "disasm.h"
#include <cosim/target/Api.h>

#include <cstdio>
#include <cstring>
#include <memory>
#include <sstream>
#include <stdexcept>
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
    const cosim_simulation_host_t* host;
    static uint64_t unpack(const uint8_t* bytes, size_t size) {
        uint64_t value = 0;
        std::memcpy(&value, bytes, size);
        return value;
    }
    bool read(reg_t address, size_t size, bool fetch, uint8_t* bytes) const {
        uint64_t value = 0;
        const int status = host->read(host->opaque, address, __builtin_ctzll(size), fetch, &value);
        if (status > 1) return false;
        std::memcpy(bytes, &value, size);
        return true;
    }
public:
    HostSim(const cfg_t* cfg, std::vector<std::pair<reg_t, abstract_mem_t*>> mems,
            const cosim_simulation_host_t* host, const std::string& elf)
        : sim_t(cfg, false, std::move(mems), {}, false, {elf}, debug_module_config_t{}, "/dev/null",
                false, nullptr, false, nullptr, std::nullopt), host(host) {}

    char* addr_to_mem(reg_t) override { return nullptr; }
    bool mmio_fetch(reg_t address, size_t size, uint8_t* bytes) override { return read(address, size, true, bytes); }
    bool mmio_load(reg_t address, size_t size, uint8_t* bytes) override { return read(address, size, false, bytes); }
    bool mmio_store(reg_t address, size_t size, const uint8_t* bytes) override {
        const unsigned lane = address & 7;
        if (!size || size > sizeof(uint64_t) || lane + size > sizeof(uint64_t)) return false;
        const int shift = __builtin_ctzll(size);
        const uint8_t localMask = size == 8 ? 0xff : uint8_t((1u << size) - 1u);
        const uint8_t mask = uint8_t(localMask << lane);
        const uint64_t data = unpack(bytes, size) << (lane * 8);
        return host->write(host->opaque, address, shift, data, mask) <= 1;
    }
};

class SpikeSimulation {
    cosim_simulation_config_t config{};
    std::unique_ptr<cfg_t> cfg;
    std::vector<std::unique_ptr<HostMemory>> storage;
    std::vector<std::pair<reg_t, abstract_mem_t*>> mems;
    std::unique_ptr<HostSim> sim;
    bool failed = false;

    void report(const std::string& message) { failed = true; config.host->fail(config.host->opaque, message.c_str()); }

    cosim_retired_instruction_t retiredInstruction(processor_t* core, reg_t pc, insn_t instruction) {
        auto* state = core->get_state();
        cosim_retired_instruction_t event{};
        event.pc = pc;
        event.instruction = static_cast<unsigned>(instruction.bits());
        event.rawInstruction = static_cast<unsigned>(instruction.bits());
        event.compressed = instruction.length() == 2;
        for (const auto& item : state->log_reg_write) {
            if ((item.first & 0xf) == 0 && (item.first >> 4) != 0) {
                event.destinationValid = 1;
                event.destination = static_cast<unsigned>(item.first >> 4);
                event.data = item.second.v[0];
                break;
            }
        }
        const auto& reads = state->log_mem_read;
        const auto& writes = state->log_mem_write;
        if (!writes.empty()) {
            event.memoryValid = 1; event.memoryWrite = 1;
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
    explicit SpikeSimulation(const cosim_simulation_config_t* input) : cfg(new cfg_t) {
        if (!input || !input->host || !input->platform || input->platform->regionCount != sizeof(regions) / sizeof(regions[0]))
            throw std::invalid_argument("invalid Spike cosim configuration");
        config = *input;
        cfg->isa = DEFAULT_ISA; cfg->priv = DEFAULT_PRIV; cfg->start_pc.set_global(input->platform->resetPC);
        cfg->hartids = {0}; cfg->explicit_hartids = false; cfg->mem_layout.clear();
        for (size_t i = 0; i < input->platform->regionCount; ++i) {
            const auto& region = input->platform->regions[i];
            if (region.device) continue;
            cfg->mem_layout.emplace_back(region.base, region.size);
            storage.push_back(std::make_unique<HostMemory>(region.size));
            mems.emplace_back(region.base, storage.back().get());
        }
        sim = std::make_unique<HostSim>(cfg.get(), mems, input->host, input->elf ? input->elf : "none");
        sim->configure_log(false, true);
    }

    int reset(unsigned) {
        sim->get_core(0)->reset();
        return 0;
    }

    int step(cosim_step_result_t* result) {
        if (!result) return 1;
        *result = {};
        try {
            auto* core = sim->get_core(0);
            auto* state = core->get_state();
            const reg_t pc = state->pc;
            const auto fetch = core->get_mmu()->load_insn(pc);
            const auto before = state->minstret->read();
            core->step(1);
            result->elapsedCycles = 1;
            if (state->minstret->read() != before) {
                result->status = COSIM_STEP_RETIRED;
                result->instruction = retiredInstruction(core, pc, fetch.insn);
            } else {
                result->status = COSIM_STEP_NO_RETIRE;
            }
            if (failed) {
                result->status = COSIM_STEP_FAILED;
                return 1;
            }
            return 0;
        } catch (const std::exception& error) {
            report(error.what());
            result->status = COSIM_STEP_FAILED;
            return 1;
        }
    }
    int finished() const { return failed; }
};

COSIM_EXTERN_C_LOCAL int describe(cosim_platform_spec_t* output) {
    if (!output || !fpr_name) return 1;
    *output = platform;
    return 0;
}
COSIM_EXTERN_C_LOCAL void* create(const cosim_simulation_config_t* config) {
    try { return new SpikeSimulation(config); }
    catch (const std::exception& error) {
        if (config && config->host && config->host->fail) config->host->fail(config->host->opaque, error.what());
        return nullptr;
    }
}
COSIM_EXTERN_C_LOCAL int reset(void* simulation, unsigned cycles) {
    return static_cast<SpikeSimulation*>(simulation)->reset(cycles);
}
COSIM_EXTERN_C_LOCAL int step(void* simulation, cosim_step_result_t* result) {
    return static_cast<SpikeSimulation*>(simulation)->step(result);
}
COSIM_EXTERN_C_LOCAL int finished(void* simulation) {
    return static_cast<SpikeSimulation*>(simulation)->finished();
}
COSIM_EXTERN_C_LOCAL void destroy(void* simulation) {
    delete static_cast<SpikeSimulation*>(simulation);
}
}

COSIM_EXTERN_C const cosim_simulation_api_t cosim_simulation_api = {
    COSIM_SIMULATION_API_VERSION, "spike", describe, create, reset, step, finished,
    destroy};
