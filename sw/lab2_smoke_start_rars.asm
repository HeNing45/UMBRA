# RARS-safe version of lab2_smoke_start.asm.
#
# The local RTL smoke test intentionally uses address 0(x0) for dmem[0].
# RARS treats address 0 as out of range, so this version uses .data labels.

.data
mem0: .word 0
mem1: .word 0

.text
main:
    addi x1, x0, 5          # x1 = 5
    addi x2, x0, 7          # x2 = 7
    add  x3, x1, x2         # x3 = 12

    la   x10, mem0
    sw   x3, 0(x10)         # mem0 = 12
    lw   x4, 0(x10)         # x4 = mem0

    beq  x4, x3, pass       # should be taken
    addi x5, x0, 1          # should be skipped

pass:
    or   x6, x4, x0         # x6 = 12

    la   x10, mem1
    sw   x6, 0(x10)         # mem1 = 12

halt:
    jal  x0, halt           # stop here forever
