# ooo_m7_mem.S — memory Spike-diff program (loads + stores, lockstep).
#
# Self-contained: builds its own data via stores (no .data section), aligned
# accesses only (misalign traps are ), no ecall (Spike bare-metal halts on
# traps), ends in a spin. Compare the first 22 commits (through addi x18).
#
# Exercises: SW/SB/SH lane placement + byte-enable merge, LW/LB/LBU/LH/LHU
# extraction, RAW through memory, loaded-value arithmetic, a taken branch
# skipping a store (RTL executes it speculatively; it must never write).

.section .text
.globl _start
_start:
    auipc x1, 0x1            # x1 = pc+0x1000: data region above text, POSITION-
                             # DEPENDENT so Spike (0x80001000) stays inside its
                             # RAM window; the normalizer value-rebases x1
    addi  x9, x0, -1         # x9 = FFFFFFFF (every lane consequential)
    sw    x9, 0(x1)          # mem[1000] = FFFFFFFF
    lw    x5, 0(x1)          # RAW: x5 = FFFFFFFF
    addi  x22, x0, 0x4D2
    sb    x22, 5(x1)         # mem[1004] lane1 <- D2 => 0000D200
    sh    x22, 6(x1)         # mem[1004] high <- 04D2 => 04D2D200
    lw    x6, 4(x1)          # x6 = 04D2D200
    lb    x7, 5(x1)          # x7 = FFFFFFD2 (sign)
    lbu   x8, 5(x1)          # x8 = 000000D2 (zero)
    lh    x10, 6(x1)         # x10 = 000004D2
    lhu   x11, 6(x1)         # x11 = 000004D2
    add   x12, x5, x8        # x12 = 000000D1
    sw    x12, 8(x1)
    lw    x13, 8(x1)         # x13 = 000000D1
    mul   x14, x22, x22      # x14 = 0x4D2^2 = 00173C44
    sw    x14, 12(x1)
    lhu   x15, 12(x1)        # x15 = 00003C44
    lb    x16, 14(x1)        # x16 = 00000017 (byte lane 2 of mem[100C])
    beq   x5, x9, 1f         # TAKEN: skips the store below
    sw    x9, 16(x1)         # architecturally skipped; RTL may execute it
                             # speculatively but must never write it
1:  lw    x17, 16(x1)        # x17 = 0 (the skipped store left memory alone)
    addi  x18, x0, 0x55      # commit #22: comparison endpoint
spin:
    jal   x0, spin
