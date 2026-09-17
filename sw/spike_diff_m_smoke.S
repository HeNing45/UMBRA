/*
 * SPDX-FileCopyrightText: Copyright 2026 He Ning
 * SPDX-License-Identifier: Apache-2.0
 */

.section .text
.globl _start
_start:
    # Signed multiply family: -3 * 7 = -21.
    addi x1, x0, -3
    addi x2, x0, 7
    mul    x10, x1, x2      # low 32 bits: 0xffffffeb
    mulh   x11, x1, x2      # signed high: 0xffffffff
    mulhsu x12, x1, x2      # signed*unsigned high: 0xffffffff

    # Unsigned high multiply: 0xffffffff * 2 = 0x00000001_fffffffe.
    addi x3, x0, -1
    addi x4, x0, 2
    mulhu  x13, x3, x4      # unsigned high: 0x00000001

    # Signed divide/remainder: -21 / 7 = -3, rem 0.
    addi x5, x0, -21
    addi x6, x0, 7
    div  x14, x5, x6
    rem  x15, x5, x6

    # Unsigned divide/remainder: 0xffffffff / 2.
    divu x16, x3, x4        # 0x7fffffff
    remu x17, x3, x4        # 0x00000001

    # Divide-by-zero corner cases from the RV32M spec.
    div  x18, x5, x0        # quotient = all 1s
    rem  x19, x5, x0        # remainder = dividend
    divu x20, x3, x0        # quotient = all 1s
    remu x21, x3, x0        # remainder = dividend

    # Signed overflow corner case: INT_MIN / -1.
    lui  x22, 0x80000       # x22 = 0x80000000
    addi x23, x0, -1
    div  x24, x22, x23      # quotient = INT_MIN, no trap
    rem  x25, x22, x23      # remainder = 0

    ecall
