<!--
SPDX-FileCopyrightText: Copyright 2026 He Ning
SPDX-License-Identifier: Apache-2.0
-->

# Software

[CoreMark®-derived CRC checks](coremark/README.md) and [Embench-IoT](embench/) run as bare-metal
simulation workloads. Each has `upstream/` sources and an `umbra/` platform port.
Assembly programs and memory images support the directed tests and Spike comparison.
The `rv32i_*_smoke` and `rv32i_smoke_start` files are small instruction tests;
the `_rars` variant uses data labels for execution in RARS.

CoreMark is a trademark of EEMBC. This is a mixed-license repository:
original UMBRA tools and the CoreMark platform port are Apache-2.0,
Copyright 2026 He Ning. CoreMark software is Apache-2.0 plus the COREMARK®
Acceptable Use Agreement for the mark. Embench's upstream suite and UMBRA
port are GPL-3.0-or-later, with per-file notices. Redistributing the Embench
workload or binaries built from it requires GPLv3 compliance, including
corresponding source for binaries; this does not relicense the CPU RTL.
See [third-party notices](../THIRD_PARTY_NOTICES.md).

Run from the repository root:

```sh
make coremark MEMORY=delayed PROFILE=max MODE=performance ITERATIONS=16
make coremark MEMORY=delayed PROFILE=max MODE=validation ITERATIONS=1
make embench MEMORY=delayed PROFILE=max BENCH=all
make spike
make spike-extended
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

The CoreMark-derived `max` CRC/cycle sample on the corrected RTL measures
**3.2029 iterations per million timed RTL cycles** over 16 iterations,
versus **2.7743** at `-O2`. CRC checks pass, but the run is shorter than the
required ten seconds. It is **not an official CoreMark score** or an on-board
measurement.

`PROFILE=o2` is the default. `PROFILE=max` selects `-O3`, full loop unrolling,
an inline limit of 1000, and 8-byte function, jump and loop alignment.
Each memory/compiler combination has separate benchmark output files.

See [running checks](../docs/IMPLEMENTATION.md#running-checks) for dependencies and
[third-party notices](../THIRD_PARTY_NOTICES.md) for credits and licenses.
