/*
 * SPDX-FileCopyrightText: Copyright 2026 He Ning
 * SPDX-License-Identifier: Apache-2.0
 */

/* Minimal RV32IM reset/startup path for the UMBRA CoreMark image. */

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

    /* portable_fini normally signals 1 before main returns. */
    li      t0, 0x0003ff08
    li      t1, 0xdead0001
    sw      t1, 0(t0)
.Lhalt:
    j       .Lhalt

    .size _start, .-_start
