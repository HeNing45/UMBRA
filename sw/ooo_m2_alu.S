/*
 * SPDX-FileCopyrightText: Copyright 2026 He Ning
 * SPDX-License-Identifier: Apache-2.0
 */

.section .text
.globl _start
# Spike-diff program: straight-line, ALU-only.
# No loads/stores, no branches/jumps (/), no auipc/PC-relative
# results (the OoO core fetches PC-0-based while Spike runs at 0x80000000, so a
# PC-relative result would differ), no ecall. 16 instructions; run with
# +MAX_COMMITS=16.
_start:
    addi x1,  x0, 5
    addi x2,  x0, 7
    add  x3,  x1, x2
    sub  x4,  x2, x1
    and  x5,  x3, x4
    or   x6,  x3, x4
    xor  x7,  x3, x2
    sll  x8,  x1, x2
    srl  x9,  x8, x1
    slt  x10, x4, x3
    sltu x11, x3, x4
    lui  x12, 0x12345
    addi x13, x12, -1
    xori x14, x7, -1
    andi x15, x13, 0xff
    sub  x16, x12, x1
