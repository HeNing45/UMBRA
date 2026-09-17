/*
 * SPDX-FileCopyrightText: Copyright 2026 He Ning
 * SPDX-License-Identifier: Apache-2.0
 */

# Full-overlap store-to-load forwarding stimulus.
#
# Pattern (x8): an older store whose DATA is late (muldiv), followed by a
# younger load of the SAME word — a full overlap.
#
# The youngest older full-overlap store supplies the load value once its
# data is ready. The load must neither read stale memory nor use an older
# matching store's value. Architectural results are independent of latency.
# Commit count: 3 prelude + 8*4 body + 1 tail = 36 (MAX_COMMITS=36)
.section .text
.globl _start
_start:
    auipc x28, 0x1          # data base = pc + 0x1000
    addi  x1, x0, 100
    addi  x2, x0, 7

.macro REP
    mul   x26, x1, x2       # late store data
    sw    x26, 8(x28)       # older store to slot 8
    lw    x5, 8(x28)        # younger load, SAME word: full overlap
    add   x1, x1, x5        # consume
.endm
    REP
    REP
    REP
    REP
    REP
    REP
    REP
    REP

    addi  x31, x0, 0x55     # tail marker
spin:
    jal   x0, spin
