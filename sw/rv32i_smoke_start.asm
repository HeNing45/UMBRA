# SPDX-FileCopyrightText: Copyright 2026 He Ning
# SPDX-License-Identifier: Apache-2.0

# Minimal RV32I single-cycle smoke program.
#
# Local Verilator testbench assumptions:
# - PC starts at 0.
# - dmem[0] is reached by address 0(x0).
# - dmem[1] is reached by address 4(x0).
#
# Expected result after execution:
# - dmem[0] = 12
# - dmem[1] = 12
#
# Focused smoke coverage, not a complete instruction-set test.

    addi x1, x0, 5          # x1 = 5
    addi x2, x0, 7          # x2 = 7
    add  x3, x1, x2         # x3 = 12
    sub x4, x3, x2          # x4 = 5
    sw   x3, 0(x0)          # dmem[0] = 12
    lw   x4, 0(x0)          # x4 = dmem[0]
    beq  x4, x3, pass       # should be taken
    addi x5, x0, 1          # should be skipped
pass:
    or   x6, x4, x0         # x6 = 12
    sw   x6, 4(x0)          # dmem[1] = 12

halt:
    jal  x0, halt           # stop here forever

sub
and / andi
or / ori
lui
auipc
sll / srl / sra
