# Load blocked by an unresolved older store address.
#
# Pattern (x8): an older store whose ADDRESS is late (muldiv -> masked to an
# aligned pool offset), followed by a younger load that is in fact
# independent (slot 36, outside the 0x1c mask range 0..28) — but the LSQ
# cannot prove that until the store's address resolves.
#
# Without memory-dependence speculation, the load must wait until the older
# store address is known. The eventual disjoint address does not justify
# issuing the load before that information is available.
# Commit count: 5 prelude + 8*5 body + 1 tail = 46 (MAX_COMMITS=46)
.section .text
.globl _start
_start:
    auipc x28, 0x1          # data base = pc + 0x1000
    addi  x1, x0, 100
    addi  x2, x0, 7
    addi  x3, x0, 13
    sw    x3, 36(x28)       # init the trailing load's slot

.macro REP
    mul   x26, x1, x2       # late address seed
    andi  x26, x26, 0x1c    # aligned pool offset 0..28
    add   x26, x26, x28     # older store's address resolves only now
    sw    x3, 0(x26)        # older store: UNKNOWN address until here
    lw    x5, 36(x28)       # younger load: independent, but unprovably so
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
