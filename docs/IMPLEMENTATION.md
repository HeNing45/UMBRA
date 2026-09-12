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

## FPGA and benchmark results

The KU5P core meets **100 MHz** in routed, out-of-context Vivado timing
(setup +0.601 ns, hold +0.012 ns). External memory, full clock-network integration
and on-board execution are not included.

The Embench rerun scores **1.217/MHz** across 19 programs, using GCC 15.2.0
at `-O2`, one-cycle instruction/data responses, pipelined instruction fetch
and two outstanding data reads. Its combined timed sections would take
**652.8 ms at 100 MHz**, assuming the same cycle counts and memory behavior.
This is a simulation projection, not an FPGA benchmark measurement.

## Running checks

Use Make, Python 3, Icarus Verilog, Verilator and a C++ compiler. Workloads
require RISC-V GCC/binutils; Spike is installed separately. Run from the repo root:

```sh
make lint
make test TB=tb_rv32i_ss_iq
make spike
make coremark MODE=performance ITERATIONS=1
make coremark MODE=validation ITERATIONS=1
make embench BENCH=crc32
```

`make test` runs one selected testbench. Use `CROSS=` to override the
`riscv-none-elf-` toolchain prefix and `TIMEOUT=` to change the simulation limit.
