    # ooo_m5_mispredict.S -- mispredict-heavy Spike-diff stimulus.
    #
    # Stresses branch speculation + recovery end-to-end vs Spike:
    # * a countdown loop whose backward BNE is TAKEN 3x (a mispredict + recover
    # each iteration under predict-not-taken) then NOT taken on exit;
    # * a forward BLT that is TAKEN (mispredict -> recover, skips x5);
    # * a MUL in flight next to the branches (muldiv recovery interaction);
    # * a JAL (serialized jump, skips x9);
    # * a self-loop tail so the RTL and Spike commit streams stay identical
    # past the program body (robust MAX_COMMITS).
    .section .text
    .globl _start
_start:
    addi x1, x0, 4        # loop counter
    addi x2, x0, 0        # accumulator
    addi x3, x0, 3        # addend
loop:
    add  x2, x2, x3       # acc += 3
    addi x1, x1, -1       # counter--
    bne  x1, x0, loop     # taken 3x (mispredict), not-taken on exit -> x2 = 12
    addi x4, x0, 10
    blt  x4, x2, skip     # 10 < 12 -> taken (mispredict), skips next
    addi x5, x0, 99       # wrong-path: speculatively dispatched, then flushed
skip:
    mul  x6, x2, x3       # 12 * 3 = 36
    addi x7, x0, 7
    jal  x8, jtgt         # serialized jump, link in x8; skips next
    addi x9, x0, 99       # never dispatched (frontend redirected)
jtgt:
    addi x10, x0, 10
    addi x11, x0, 11
    addi x12, x0, 12
done:
    jal  x0, done         # self-loop tail (identical in RTL & Spike)
