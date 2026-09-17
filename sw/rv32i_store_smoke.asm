# SPDX-FileCopyrightText: Copyright 2026 He Ning
# SPDX-License-Identifier: Apache-2.0

# Directed sub-word store smoke program for the single-cycle RTL testbench.
#
# The testbench preloads:
# dmem[1] = 0xaabbccdd
# dmem[2] = 0xdeadbeef
# dmem[3] = 0xcafef00d
#
# Expected result after execution:
# dmem[0] = 0x11223344 sw full word
# dmem[1] = 0x44556677 sb writes byte lanes 0,1,2,3
# dmem[2] = 0xdead1234 sh writes low half
# dmem[3] = 0x5678f00d sh writes high half
# dmem[4] = 0x13579753 sw full word

    lui  x1, 0x11223
    addi x1, x1, 0x344
    sw   x1, 0(x0)

    addi x2, x0, 0x77
    addi x3, x0, 0x66
    addi x4, x0, 0x55
    addi x5, x0, 0x44
    sb   x2, 4(x0)
    sb   x3, 5(x0)
    sb   x4, 6(x0)
    sb   x5, 7(x0)

    lui  x6, 0x1
    addi x6, x6, 0x234
    sh   x6, 8(x0)

    lui  x7, 0x5
    addi x7, x7, 0x678
    sh   x7, 14(x0)

    lui  x8, 0x13579
    addi x8, x8, 0x753
    sw   x8, 16(x0)

halt:
    jal  x0, halt
