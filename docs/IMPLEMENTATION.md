# Implementation

## Four processor generations

| Directory | Design | Entry point | Filelist |
| --- | --- | --- | --- |
| [rtl/](../rtl/) | Single-cycle RV32I | `rv32i_single_cycle_core.sv` | `filelist.f` |
| [rtl_p/](../rtl_p/) | Five-stage RV32IM pipeline | `rv32i_pipeline_core.sv` | `filelist_pipeline.f` |
| [rtl_ooo/](../rtl_ooo/) | Scalar RV32IM out-of-order | `rv32i_ooo_frontend.sv` + `rv32i_ooo_core.sv` | `filelist_ooo.f` |
| [rtl_superscalar/](../rtl_superscalar/) | Two-wide RV32IM out-of-order | `umbra_ss_cpu_top.sv` | `filelist_superscalar.f` |

Filelists live in their corresponding directories; source paths are relative
to the repository root. Shared packages and leaf modules remain in `rtl/`
and `rtl_p/`. The testbenches included here target the superscalar core.

## Superscalar core

The core has 64 physical registers, a 32-entry reorder buffer, a unified
16-entry issue queue, eight branch checkpoints and eight-entry load/store
queues. Two ALUs, a multiply/divide unit and an address-generation unit feed
two writeback lanes. Retirement remains ordered, up to two instructions per cycle.
Registered selection and operand boundaries separate scheduling from execution.

`umbra_ss_cpu_top` exposes instruction/data request-response ports and commit
traces. Scratchpad modules model memory timing for simulation; they are not
caches, board SRAM macros or a DDR controller. Memory settings must match the
testbench being run. The pipeline and OoO cores include partial machine-mode
CSR/trap support.

## Verification

- **Spike:** passed the 16-instruction ALU commit-trace comparison.
- **CoreMark:** performance and validation seed sets passed all expected CRC checks.
- **Embench:** all 19 programs completed with successful return values; the
  unchanged XGBoost self-check is weak at the default scale factor.

These are RTL simulation checks. The short CoreMark runs are not qualifying
benchmark scores. The directed superscalar suite passed 60 of 61 tests;
`tb_rv32i_ss_ras_bench` still has a cycle-count mismatch (403 versus 374 expected).

## Physical layout

[Download the GDS](https://github.com/HeNing45/UMBRA/releases/download/physical-osu45-20ns-corrected/umbra_syn_island.gds)
and [KLayout layer file](../physical/umbra_syn_island.lyp).
The GDS is distributed through the [physical release](https://github.com/HeNing45/UMBRA/releases/tag/physical-osu45-20ns-corrected),
which also includes the license and third-party notices.

In KLayout, open the GDS, select `umbra_syn_island`, and load the layer file.

The corrected implementation meets 20 ns / 50 MHz with extracted setup slack
+3.943214 ns, hold slack +0.000031 ns and zero hold violators. Calibre and IC
Validator each report zero results across 167 DRC checks; both LVS comparisons
pass. Their shared VSS-label diagnostic describes seven intentionally grounded
output ports, not an unintended short. This is an academic-model result,
not foundry signoff; the hold margin is only 31 fs.

## FPGA and benchmark results

The KU5P core meets **100 MHz** in routed, out-of-context Vivado timing
(setup +0.446 ns, hold +0.012 ns). External memory, full clock-network integration
and on-board execution are not included.

The routed run has zero failing setup/hold endpoints and zero routing errors.
Vivado retains 17 warnings: 16 DSP pipelining recommendations and one net
without routable loads. OOC boundary ports have no physical pin locations;
this result does not qualify external I/O timing or a board implementation.

The Embench rerun scores **1.488/MHz** with `max`, versus **1.217/MHz** at
`-O2`, across the same 19 programs. Both use GCC 15.2.0, one-cycle
instruction/data responses, pipelined instruction fetch and two outstanding
data reads. The `max` combined timed sections would take **560.7 ms at 100 MHz**
(652.8 ms at `-O2`), assuming the same cycle counts and memory behavior.
This is a simulation projection, not an FPGA benchmark measurement.

The corrected RTL also passes CoreMark performance and validation CRC checks.
The 16-iteration `max` sample takes 4,995,442 timed cycles, or **3.2029 iterations
per million timed cycles**. The `-O2` comparison takes 5,767,311 timed cycles
and scores **2.7743**. The run is shorter than ten seconds and is not a
qualifying CoreMark score. These results use the delayed-memory profile below.

## Running checks

Use Make, Python 3, Icarus Verilog, Verilator and a C++ compiler. Workloads
require RISC-V GCC/binutils; Spike is installed separately. Run from the repo root:

```sh
make lint
make test TB=tb_rv32i_ss_iq
make spike
make coremark MEMORY=delayed PROFILE=max MODE=performance ITERATIONS=16
make coremark MEMORY=delayed PROFILE=max MODE=validation ITERATIONS=1
make embench MEMORY=delayed PROFILE=max BENCH=all
```

`make test` runs one selected testbench. Use `CROSS=` to override the
`riscv-none-elf-` toolchain prefix and `TIMEOUT=` to change the simulation limit.
Benchmark builds default to `MEMORY=zero`; `MEMORY=delayed` selects one-cycle
pipelined instruction responses and one-cycle data responses with two
outstanding reads. Each profile has a separate simulator build directory.

`PROFILE=o2` is the default compiler setting. `PROFILE=max` uses `-O3`,
`-funroll-all-loops`, `-finline-limit=1000`, and 8-byte function, jump and loop
alignment. Benchmark outputs are separated by memory and compiler profile.
