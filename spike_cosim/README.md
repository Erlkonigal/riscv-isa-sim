# Spike cosim adapter

This directory builds Spike's API-v7 target shared library. It links Spike as a
static library bundle and uses the cosim headers from the sibling checkout. It
does not link the cosim runtime.

```sh
make COSIM_INCLUDE_DIR=/path/to/cosim/include libspike_cosim.so
```

Spike exposes one 64-bit TL master and one CPU clock domain. All instruction and
data accesses go through the runtime-owned TL transport. The adapter keeps
Spike's synchronous memory callbacks compatible with asynchronous TL service by
running one instruction in a worker state machine: when a request has no D beat,
the worker yields to the runtime, which advances the global scheduler and resumes
the worker after response availability. The adapter never advances simulation
time itself.

The library exports `cosim_simulation_api` and reports retired instructions via
`poll_event`. It does not export a SignalTrace API. The platform description
matches Jeucore's RAM, uncached RAM page, and UART regions; the runner installs
the ELF into both targets' device sessions.
