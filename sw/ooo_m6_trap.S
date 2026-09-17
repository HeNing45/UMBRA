/*
 * SPDX-FileCopyrightText: Copyright 2026 He Ning
 * SPDX-License-Identifier: Apache-2.0
 */

# Register-only machine-mode trap round trip: ECALL enters the mtvec handler,
# the handler advances mepc, and MRET resumes after ECALL.
.option arch, +zicsr
.section .text
.globl _start
_start:
    la    t0, handler
    csrw  mtvec, t0
    addi  a0, x0, 5
    ecall                 # trap: mepc <- this PC, redirect to mtvec
    addi  a1, x0, 0x111    # mret returns here (mepc+4)
    addi  a2, x0, 0x222
done:
    jal   x0, done

handler:
    csrr  t1, mcause       # expect 11 (ecall-from-M)
    csrr  t2, mepc
    addi  t2, t2, 4        # skip the ecall on return
    csrw  mepc, t2
    mret
