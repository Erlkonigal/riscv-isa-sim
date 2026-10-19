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

The library exports `cosim_simulation_api`. Its platform description matches
the default Jeucore simulation map: RAM at `0x80000000`, an uncached RAM page,
and the UART at `0x12002000`.
