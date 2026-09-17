# SPDX-FileCopyrightText: Copyright 2026 He Ning
# SPDX-License-Identifier: Apache-2.0

# Directed sub-word load smoke program for the single-cycle RTL testbench.
#
# Local memory model is little-endian word-backed:
# dmem[0] = 0xabcd0123
# byte offset 0 -> 0x23
# byte offset 1 -> 0x01
# byte offset 2 -> 0xcd
# byte offset 3 -> 0xab
#
# Expected result after execution:
# dmem[1] = 0x00000023 lb byte0 signed
# dmem[2] = 0x00000001 lb byte1 signed
# dmem[3] = 0xffffffcd lb byte2 signed
# dmem[4] = 0xffffffab lb byte3 signed
# dmem[5] = 0x000000cd lbu byte2 unsigned
# dmem[6] = 0x000000ab lbu byte3 unsigned
# dmem[7] = 0x00000123 lh low half signed
# dmem[8] = 0xffffabcd lh high half signed
# dmem[9] = 0x0000abcd lhu high half unsigned
# dmem[10] = 0xabcd0123 lw full word unchanged

    lui  x1, 0xabcd0
    addi x1, x1, 0x123
    sw   x1, 0(x0)

    lb   x2,  0(x0)
    lb   x3,  1(x0)
    lb   x4,  2(x0)
    lb   x5,  3(x0)
    lbu  x6,  2(x0)
    lbu  x7,  3(x0)
    lh   x8,  0(x0)
    lh   x9,  2(x0)
    lhu  x10, 2(x0)
    lw   x11, 0(x0)

    sw   x2,  4(x0)
    sw   x3,  8(x0)
    sw   x4,  12(x0)
    sw   x5,  16(x0)
    sw   x6,  20(x0)
    sw   x7,  24(x0)
    sw   x8,  28(x0)
    sw   x9,  32(x0)
    sw   x10, 36(x0)
    sw   x11, 40(x0)

halt:
    jal  x0, halt
