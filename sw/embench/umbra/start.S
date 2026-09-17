/*
 * SPDX-FileCopyrightText: Copyright 2026 He Ning
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/* Minimal RV32IM reset/startup path for UMBRA Embench-IoT images.
 *
 * Same shape as the CoreMark crt0, with one deliberate difference: Embench's
 * upstream main returns 0 when verify_benchmark passed. The done MMIO
 * value is derived from that return, so the simulation harness (which
 * requires done_value == 1) enforces self-verification for free:
 * main == 0 -> done = 1 (verified pass)
 * main != 0 -> done = 2 (verification FAILED; harness errors out)
 */

    .section .text.init
    .globl _start
    .type _start, @function
_start:
    la      sp, __stack_top

    .option push
    .option norelax
    la      gp, __global_pointer$
    .option pop

    la      t0, __bss_start
    la      t1, __bss_end
.Lzero_bss:
    bgeu    t0, t1, .Lcall_main
    sw      zero, 0(t0)
    addi    t0, t0, 4
    j       .Lzero_bss

.Lcall_main:
    call    main

    li      t0, 0x0003ff08
    seqz    t1, a0            /* 1 when main returned 0, else 0 */
    xori    t2, t1, 1         /* 1 only on failure */
    add     t1, t1, t2        /* pass -> 1, fail -> 1 */
    add     t1, t1, t2        /* pass -> 1, fail -> 2 */
    sw      t1, 0(t0)
.Lhalt:
    j       .Lhalt

    .size _start, .-_start
