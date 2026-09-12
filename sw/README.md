# Software

[CoreMark](coremark/README.md) and [Embench-IoT](embench/) run as bare-metal
simulation workloads. Each has `upstream/` sources and an `umbra/` platform port.
Assembly programs and memory images support the directed tests and Spike comparison.

Run from the repository root:

```sh
make coremark MODE=performance ITERATIONS=1
make coremark MODE=validation ITERATIONS=1
make embench BENCH=crc32
make spike
```

Use `CROSS=` to select a RISC-V compiler prefix. `BENCH=all` runs the included
Embench suite. At the default scale factor, `xgboost`'s self-check does not
meaningfully validate its numerical results.

The full one-cycle-memory Embench rerun completed all 19 programs and scored
**1.217/MHz**. See [benchmark results](../docs/IMPLEMENTATION.md#fpga-and-benchmark-results).
The small Makefile targets use zero-latency memory by default, so they do not
reproduce that delayed-memory score without a separately configured simulator.

See [running checks](../docs/IMPLEMENTATION.md#running-checks) for dependencies and
[third-party notices](../THIRD_PARTY_NOTICES.md) for credits and licenses.
