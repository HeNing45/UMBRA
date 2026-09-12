# Branch-focused RV32I smoke program for the local RTL testbench.
#
# Local RTL testbench assumptions:
# - PC starts at 0.
# - dmem[0] is reached by address 0(x0).
# - dmem[1] is reached by address 4(x0).
# - dmem[2] is reached by address 8(x0).
# - dmem[n] is reached by address 4*n(x0).
#
# Expected result after execution:
# - dmem[0] = 11 beq taken
# - dmem[1] = 22 beq not taken
# - dmem[2] = 33 bne taken
# - dmem[3] = 44 bne not taken
# - dmem[4] = 12 add
# - dmem[5] = 2 sub
# - dmem[6] = 8 and
# - dmem[7] = 8 andi
# - dmem[8] = 14 or
# - dmem[9] = 14 ori
# - dmem[10] = 8 sll
# - dmem[11] = 8 srl
# - dmem[12] = -8 sra
# - dmem[13] = 0x12345000 lui
# - dmem[14] = 204 auipc x26, 0 at PC 0x000000cc
# - dmem[15] = 0x12345000 lw

    addi x10, x0, 99         # fail marker if a wrong path stores
    addi x1,  x0, 5
    addi x2,  x0, 5
    addi x3,  x0, 7

    beq  x1, x2, beq_taken_ok
    sw   x10, 0(x0)          # wrong path
    jal  x0, after_beq_taken
beq_taken_ok:
    addi x4, x0, 11
    sw   x4, 0(x0)
after_beq_taken:

    beq  x1, x3, beq_not_taken_bad
    addi x5, x0, 22
    sw   x5, 4(x0)
    jal  x0, after_beq_not_taken
beq_not_taken_bad:
    sw   x10, 4(x0)          # wrong path
after_beq_not_taken:

    bne  x1, x3, bne_taken_ok
    sw   x10, 8(x0)          # wrong path
    jal  x0, after_bne_taken
bne_taken_ok:
    addi x6, x0, 33
    sw   x6, 8(x0)
after_bne_taken:

    bne  x1, x2, bne_not_taken_bad
    addi x7, x0, 44
    sw   x7, 12(x0)
    jal  x0, after_bne_not_taken
bne_not_taken_bad:
    sw   x10, 12(x0)         # wrong path
after_bne_not_taken:

    add  x8,  x1,  x3         # x8 = 12
    sw   x8,  16(x0)

    sub  x9,  x3,  x1         # x9 = 2
    sw   x9,  20(x0)

    addi x11, x0, 10
    addi x12, x0, 12
    and  x13, x11, x12        # x13 = 8
    sw   x13, 24(x0)

    andi x14, x12, 10         # x14 = 8
    sw   x14, 28(x0)

    or   x15, x11, x12        # x15 = 14
    sw   x15, 32(x0)

    ori  x16, x11, 12         # x16 = 14
    sw   x16, 36(x0)

    addi x17, x0, 1
    addi x18, x0, 3
    sll  x19, x17, x18        # x19 = 8
    sw   x19, 40(x0)

    addi x20, x0, 64
    srl  x21, x20, x18        # x21 = 8
    sw   x21, 44(x0)

    addi x22, x0, -32
    addi x23, x0, 2
    sra  x24, x22, x23        # x24 = -8
    sw   x24, 48(x0)

    lui  x25, 0x12345         # x25 = 0x12345000
    sw   x25, 52(x0)

    auipc x26, 0              # x26 = current PC = 0x000000cc
    sw   x26, 56(x0)

    lw   x27, 52(x0)          # x27 = dmem[13]
    sw   x27, 60(x0)

halt:
    jal  x0, halt
