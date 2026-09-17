/*
 * SPDX-FileCopyrightText: Copyright 2026 He Ning
 * SPDX-License-Identifier: Apache-2.0
 */

# Load past an independent older store.
#
# Pattern (x8): an older store whose DATA is late (muldiv result) but whose
# ADDRESS is known early, followed by a younger load from a provably
# independent slot, whose result feeds the next iteration (critical path).
#
# With every older store address known and disjoint, the load may launch
# without waiting for store data. Its value must match in-order execution.
# Commit count: 5 prelude + 8*4 body + 1 tail = 38 (MAX_COMMITS=38)
.section .text
.globl _start
_start:
    auipc x28, 0x1          # data base = pc + 0x1000
    addi  x1, x0, 100
    addi  x2, x0, 7
    addi  x3, x0, 13
    sw    x3, 32(x28)       # init the independent load slot

.macro REP
    mul   x26, x1, x2       # late store data
    sw    x26, 0(x28)       # older store: addr known early, data late
    lw    x5, 32(x28)       # younger INDEPENDENT load (slot 32 vs slot 0)
    add   x1, x1, x5        # consume the load -> win lands on the chain
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
