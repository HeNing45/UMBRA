    .section .text
    .globl _start
_start:
    addi x1, x0, 5
    addi x2, x0, 5
    beq  x1, x2, branch_target
    addi x3, x0, 99

branch_target:
    addi x4, x0, 2
    jal  x5, jal_target
    addi x6, x0, 99

jal_target:
    addi  x7, x0, 7
    auipc x8, 0
    addi  x8, x8, 17
    jalr  x9, 0(x8)
    addi  x10, x0, 99

jalr_target:
    addi x11, x0, 11
    addi x12, x0, 12
