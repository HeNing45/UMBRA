.section .text
.globl _start
_start:
    addi x1, x0, 5
    addi x2, x0, 7
    add  x3, x1, x2

    lui  x5, 0x80001       # x5 = 0x80001000, inside Spike memory map
    sw   x3, 0(x5)
    lw   x4, 0(x5)

    beq  x4, x3, pass
    addi x6, x0, 99        # wrong path if branch is broken

pass:
    addi x7, x0, 1
    ecall
