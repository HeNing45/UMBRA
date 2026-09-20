#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright 2026 He Ning
# SPDX-License-Identifier: Apache-2.0

"""Emit a deterministic, non-trapping RV32IM stress program to stdout.

This is a bounded seeded instruction template, not an ISA coverage claim.
Keep code under the trace testbench's 4 KiB instruction image. Data is
initialized explicitly; Spike and RTL use identical high addresses.
"""

import argparse
import random


def program(seed):
    rng = random.Random(seed)
    lines = [
        ".section .text",
        ".option norvc",
        ".option norelax",
        ".globl _start",
        "_start:",
        "lui x1,0x80001",
        "addi x2,x0,0",
        "addi x3,x0,1000",
        "addi x4,x0,-1",
        "addi x5,x0,0",
    ]
    for reg in range(6, 25):
        lines.append(f"addi x{reg},x0,{rng.randint(-2048, 2047)}")
    for offset in range(0, 68, 4):
        lines.append(f"sw x0,{offset}(x1)")
    lines.append("loop:")
    for block in range(8):
        a, b, c = rng.sample(range(6, 25), 3)
        alu = rng.choice(["add", "sub", "xor", "or", "and", "slt", "sltu"])
        md = ["mul", "mulh", "mulhsu", "mulhu", "div", "divu", "rem", "remu"][block]
        offset = block * 8
        lines += [
            f"{alu} x{a},x{b},x{c}",
            f"{md} x{b},x{a},x4",
            f"sw x{b},{offset}(x1)",
            f"lw x{c},{offset}(x1)",
            f"add x5,x5,x{c}",
            f"sb x{a},{offset+1}(x1)",
            f"sh x{c},{offset+2}(x1)",
            f"lb x{a},{offset+1}(x1)",
            f"lbu x{b},{offset+1}(x1)",
            f"lh x{c},{offset+2}(x1)",
            f"lhu x{b},{offset+2}(x1)",
            f"slli x{a},x{a},{rng.randrange(32)}",
            f"srli x{b},x{b},{rng.randrange(32)}",
            f"sra x{c},x{a},x2",
        ]
    lines += [
        "div x24,x5,x0",
        "rem x23,x5,x0",
        "lui x22,0x80000",
        "div x21,x22,x4",
        "rem x20,x22,x4",
        "andi x25,x2,1",
        "beq x25,x0,even",
        "addi x5,x5,3",
        "jal x0,join",
        "even:",
        "addi x5,x5,-5",
        "join:",
        "beq x24,x4,skip_store",
        "sw x4,64(x1)",
        "skip_store:",
        "lw x26,64(x1)",
        "add x5,x5,x26",
        "jal x31,subroutine",
    ]
    for offset in range(0, 64, 4):
        lines += [f"lw x27,{offset}(x1)", "xor x5,x5,x27"]
    lines += [
        "addi x2,x2,1",
        "blt x2,x3,loop",
        "spin:",
        "jal x0,spin",
        "subroutine:",
        "xor x5,x5,x2",
        "jalr x0,0(x31)",
    ]
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, required=True)
    print(program(parser.parse_args().seed), end="")
