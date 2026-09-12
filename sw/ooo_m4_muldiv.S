    .section .text
    .globl _start
# Spike-diff program: straight-line RV32M through the OoO frontend/core.
# No loads/stores, no traps, no PC-relative results. Run with MAX_COMMITS=16.
_start:
    addi x1,  x0, 6
    addi x2,  x0, 7
    mul  x3,  x1, x2
    mulh x4,  x1, x2
    addi x5,  x0, -9
    mul  x6,  x5, x2
    div  x7,  x6, x2
    rem  x8,  x6, x2
    divu x9,  x6, x2
    remu x10, x6, x2
    addi x11, x0, -1
    div  x12, x1, x0
    rem  x13, x1, x0
    mulhu  x14, x11, x2
    mulhsu x15, x5, x2
    mulh   x16, x5, x2
