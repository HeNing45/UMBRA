<!--
SPDX-FileCopyrightText: Copyright 2026 He Ning
SPDX-License-Identifier: Apache-2.0
-->

# Implementation

## Four processor generations

| Directory | Design | Entry point | Filelist |
| --- | --- | --- | --- |
| [rtl/](../rtl/) | Single-cycle RV32I | `rv32i_single_cycle_core.sv` | `filelist.f` |
| [rtl_p/](../rtl_p/) | Five-stage RV32IM pipeline | `rv32i_pipeline_core.sv` | `filelist_pipeline.f` |
| [rtl_ooo/](../rtl_ooo/) | Scalar RV32IM out-of-order | `rv32i_ooo_frontend.sv` + `rv32i_ooo_core.sv` | `filelist_ooo.f` |
| [rtl_superscalar/](../rtl_superscalar/) | 2-wide RV32IM out-of-order | `umbra_ss_cpu_top.sv` | `filelist_superscalar.f` |

Filelists live in their corresponding directories; source paths are relative
to the repository root. Shared packages and leaf modules remain in `rtl/`
and `rtl_p/`. The testbenches included here target the superscalar core.

## Superscalar core

The core has 64 physical registers, a 32-entry reorder buffer, a unified
16-entry issue queue, 8 branch checkpoints and 8-entry load/store
queues. Two ALUs, a multiply/divide unit and an address generation unit feed
two writeback lanes. Retirement remains ordered, up to two instructions per cycle.
Registered selection and operand boundaries separate scheduling from execution.

`umbra_ss_cpu_top` exposes instruction/data request-response ports and commit
traces. Scratchpad modules model memory timing for simulation; they are not
caches, board SRAM macros or a DDR controller. Memory settings must match the
testbench being run. The pipeline and OoO cores include partial machine-mode
CSR/trap support.

## Verification

- **Spike:** the 16-instruction ALU smoke and extended seeded RV32IM differential pass. The extended gate matched 36,864 commit records and accepted store effects across three memory-timing modes; this is bounded stress, not exhaustive ISA proof.
- **CoreMark®-derived CRC check:** performance and validation seed sets passed all expected CRC checks.
- **Embench:** all 19 programs completed with successful return values; the
  unchanged XGBoost self-check is weak at the default scale factor.

CoreMark is a trademark of EEMBC. These are RTL simulation checks, not official
CoreMark scores. The directed superscalar suite passed 60 of 61 tests;
`tb_rv32i_ss_ras_bench` still has a cycle-count mismatch (403 versus 374 expected).

## Physical layout

[Download the previous GDS](https://github.com/HeNing45/UMBRA/releases/download/physical-osu45-20ns-corrected/umbra_syn_island.gds)
and [KLayout layer file](../physical/umbra_syn_island.lyp).
The GDS is distributed through the [physical release](https://github.com/HeNing45/UMBRA/releases/tag/physical-osu45-20ns-corrected),
which also includes the license and third-party notices.
The newly verified layout has not yet been uploaded; these links retain the previous release.

In KLayout, open the GDS, select `umbra_syn_island`, and load the layer file.

The corrected implementation meets 20 ns / 50 MHz with extracted setup slack
+3.181066 ns, hold slack +0.000109 ns and zero hold violators. Calibre and IC
Validator each report zero results across 167 DRC checks; both LVS comparisons
pass. Their shared VSS label diagnostic describes seven intentionally grounded
output ports, not an unintended short. This is an academic model result,
not foundry signoff; the hold margin is only 109 fs. ICV LVS retains native
exit 30 for that diagnostic, despite a passing comparison. Positive-limit
capacitance violations are zero; 101,843 zero-limit library entries remain.

## FPGA and benchmark results

The corrected KU5P core meets **100 MHz** in routed, out-of-context Vivado timing
(setup +0.474 ns, hold +0.009 ns). External memory, full clock network integration
and on-board execution are not included.

The routed run has zero failing setup/hold endpoints and zero routing errors.
Vivado retains 17 warnings: 16 DSP pipelining recommendations and one
no-routable-load warning. OOC boundary ports have no physical pin locations;
this result does not qualify external I/O timing or a board implementation.

Both compiler profiles were rerun on the corrected frontend RTL.

The Embench rerun scores **1.488/MHz** with `max`, versus **1.217/MHz** at
`-O2`, across the same 19 programs. Both use GCC 15.2.0, one-cycle
instruction/data responses, pipelined instruction fetch and two outstanding
data reads. The `max` combined timed sections would take **560.7 ms at 100 MHz**
(652.8 ms at `-O2`), assuming the same cycle counts and memory behavior.
This is a simulation projection, not an FPGA benchmark measurement.

The corrected RTL passed the CoreMark-derived performance and validation CRC checks.
The 16-iteration `max` sample takes 4,995,442 timed cycles, or **3.2029 iterations
per million timed RTL cycles**. The `-O2` comparison takes 5,767,311 timed cycles
and measures **2.7743 iterations per million timed RTL cycles**. These are
CRC/cycle checks, not an official CoreMark score. The run is shorter than
the required ten seconds. These results use the delayed-memory configuration
described under [Running checks](#running-checks).

## Running checks

Use Make, Python 3, Icarus Verilog, Verilator and a C++ compiler. Workloads
require RISC-V GCC/binutils; Spike is installed separately. Run from the repo root:

```sh
make lint
make test TB=tb_rv32i_ss_iq
make spike
make spike-extended
make coremark MEMORY=delayed PROFILE=max MODE=performance ITERATIONS=16
make coremark MEMORY=delayed PROFILE=max MODE=validation ITERATIONS=1
make embench MEMORY=delayed PROFILE=max BENCH=all
```

`make test` runs one selected testbench. Use `CROSS=` to override the
`riscv-none-elf-` toolchain prefix and `TIMEOUT=` to change the simulation limit.
Benchmark builds default to `MEMORY=zero`; `MEMORY=delayed` selects one-cycle
pipelined instruction responses and one-cycle data responses with two
outstanding reads. Each profile has a separate simulator build directory.

`make spike-extended` defaults to seeds 1, 7 and 42, with 4,096 commits per
seed under always-ready, one-cycle-response and stalled/four-cycle-response
D-memory modes. It requires exact commit traces, exact accepted-store address,
enabled data, byte mask and commit order, and nonzero load/store/branch/M/recovery
coverage. `SS_SPIKE_SEEDS`, `SS_SPIKE_COMMITS` and `SS_SPIKE_RUN_DIR` override
the defaults.

`PROFILE=o2` is the default compiler setting. `PROFILE=max` uses `-O3`,
`-funroll-all-loops`, `-finline-limit=1000`, and 8-byte function, jump and loop
alignment. Benchmark outputs are separated by memory and compiler profile.

## Licences

This is a mixed-license repository. Original UMBRA RTL, testbenches,
verification tools and documentation are Copyright 2026 He Ning,
[Apache-2.0](../LICENSE). Embench's upstream sources and UMBRA port are
GPL-3.0-or-later, with per-file notices; compiled Embench workloads are GPL
combined works, not a relicensing of the CPU RTL. CoreMark software is
Apache-2.0 plus the COREMARK® Acceptable Use Agreement for the mark.
The release-only GDS includes Apache-2.0 OSU/FreePDK45 cell geometry.
See [NOTICE](../NOTICE) and [third-party notices](../THIRD_PARTY_NOTICES.md).
