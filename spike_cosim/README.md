# Spike cosim adapter

This directory builds Spike's reference shared library (cosim REF API v8). It
links Spike as a static library bundle and uses the cosim headers from the
sibling checkout. It does not link the cosim runtime.

```sh
make COSIM_INCLUDE_DIR=/path/to/cosim/include libspike_cosim.so
```

The library exports `cosim_ref_api` and runs synchronously on the runner thread:
each `step` executes until exactly one instruction retires and returns its
retirement record. Spike never sees the TL bus or simulation time. Instruction
fetches and RAM accesses are served by the runtime-owned golden memory through
the `memory` callbacks; device accesses are replayed against the DUT MMIO trace
through the `mmio` callbacks, so external effects execute exactly once on the
DUT side. Accesses outside the platform regions raise the matching access fault,
mirroring the DUT bus denial.

The platform description matches Jeucore's RAM, uncached RAM page, and UART
regions; the runtime installs the ELF into the DUT devices and the golden
memory. The adapter does not export a SignalTrace API.
