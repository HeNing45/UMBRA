# Software

[CoreMark](coremark/README.md) and [Embench-IoT](embench/) run as bare-metal
simulation workloads. Each has `upstream/` sources and an `umbra/` platform port.
Assembly programs and memory images support the directed tests and Spike comparison.
The `rv32i_*_smoke` and `rv32i_smoke_start` files are small instruction tests;
the `_rars` variant uses data labels for execution in RARS.

Embench's upstream sources retain their GPLv3 and per-file licence notices.
See [third-party notices](../THIRD_PARTY_NOTICES.md); the repository's Apache
licence does not replace the licences of bundled benchmarks.

Run from the repository root:

```sh
make coremark MEMORY=delayed PROFILE=max MODE=performance ITERATIONS=16
make coremark MEMORY=delayed PROFILE=max MODE=validation ITERATIONS=1
make embench MEMORY=delayed PROFILE=max BENCH=all
make spike
```

Use `CROSS=` to select a RISC-V compiler prefix. `BENCH=all` runs the included
Embench suite. At the default scale factor, `xgboost`'s self-check does not
meaningfully validate its numerical results.

The Embench rerun with one-cycle memory responses completed all 19 programs and scored
**1.488/MHz** with `max`, versus **1.217/MHz** at `-O2`.
See [benchmark results](../docs/IMPLEMENTATION.md#fpga-and-benchmark-results).
`MEMORY=delayed` selects the measured configuration: one-cycle pipelined
instruction responses, one-cycle data responses and two outstanding reads.
`MEMORY=zero` remains the default. The profiles use separate simulator builds.

The CoreMark `max` sample on the corrected RTL measures **3.2029 iterations per million
timed cycles** over 16 iterations, versus **2.7743** at `-O2`. CRC checks pass,
but the short run is not a qualifying ten-second CoreMark score or an
on-board measurement.

`PROFILE=o2` is the default. `PROFILE=max` selects `-O3`, full loop unrolling,
an inline limit of 1000, and 8-byte function, jump and loop alignment.
Each memory/compiler combination has separate benchmark output files.

See [running checks](../docs/IMPLEMENTATION.md#running-checks) for dependencies and
[third-party notices](../THIRD_PARTY_NOTICES.md) for credits and licenses.
