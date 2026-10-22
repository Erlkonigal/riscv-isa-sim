# Spike cosim adapter

This directory builds Spike's implementation of the independent cosim
simulation ABI. The adapter is part of the Spike repository and does not link
against the cosim runtime. It uses the host callbacks for all memory and MMIO
accesses, so the runtime can execute it beside an RTL DUT.

After configuring Spike, build the shared library with the cosim headers:

```sh
make COSIM_INCLUDE_DIR=/path/to/cosim/include libspike_cosim.so
```

The generated shared-library rule links Spike's static subprojects as a group,
because the ISA, disassembler, and SoftFloat archives have cyclic references.

The library exports `cosim_simulation_api`. It does not export
`cosim_signal_trace_api` because Spike has no SignalTrace schemas. Its platform
description matches the default Jeucore simulation map: RAM at `0x80000000`, an
uncached RAM page, and the UART at `0x12002000`.

Each `step` returns either one retired instruction or an idle result through the
common API v6 result structure and reports one elapsed CPU cycle for every core
step. Pass and fail PCs are configured by the cosim runtime rather than embedded
in this platform description.

The adapter accepts the common waveform configuration in the cosim API for
ABI compatibility. Spike has no RTL waveform backend, so it ignores waveform
settings; they can be enabled for a Verilator DUT while Spike runs as a REF.
