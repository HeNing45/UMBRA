#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright 2026 He Ning
# SPDX-License-Identifier: Apache-2.0

"""Normalize Spike --log-commits output to the repo's COMMIT trace format.

Trace fields:
  - Emits one COMMIT line per retired Spike instruction.
  - Extracts rd/wdata when Spike reports a GPR write.
  - Emits rd=0/wdata=0 for stores, branches, jumps to x0, and traps.
  - Optionally subtracts a PC base so high-linked Spike ELFs compare with
    RTL-local PC traces.
"""

from __future__ import annotations

import argparse
import re
import sys


COMMIT_RE = re.compile(
    r"core\s+\d+:\s+(?:\d+\s+)?0x(?P<pc>[0-9a-fA-F]+)\s+\(0x(?P<instr>[0-9a-fA-F]+)\)"
)
GPR_RE = re.compile(r"\b(?:x|[a-z][a-z0-9]*)\s*(?P<rd>\d+)\s+0x(?P<wdata>[0-9a-fA-F]+)")
ALT_GPR_RE = re.compile(r"\b(?:x(?P<rdx>\d+)|[a-z][a-z0-9]*)=0x(?P<wdata>[0-9a-fA-F]+)")


def parse_int(text: str) -> int:
    return int(text, 0)


def normalize(
    path: str,
    pc_base: int,
    skip_before: int | None,
    value_base: int | None = None,
    value_size: int = 0x20000,
) -> int:
    count = 0
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            match = COMMIT_RE.search(line)
            if not match:
                continue

            raw_pc = int(match.group("pc"), 16)
            if skip_before is not None and raw_pc < skip_before:
                continue

            pc = raw_pc - pc_base
            instr = int(match.group("instr"), 16)
            rd = 0
            wdata = 0

            gpr = GPR_RE.search(line)
            if gpr:
                rd = int(gpr.group("rd"), 10)
                wdata = int(gpr.group("wdata"), 16)
            else:
                alt = ALT_GPR_RE.search(line)
                if alt and alt.group("rdx") is not None:
                    rd = int(alt.group("rdx"), 10)
                    wdata = int(alt.group("wdata"), 16)

            wdata &= 0xFFFFFFFF
            # Opt-in: registers holding MEMORY ADDRESSES differ between
            # the high-linked Spike run and the rebased RTL run (e.g. auipc
            # results). Rebase wdata values that fall inside the Spike memory
            # window so address-bearing commits compare. Data values must be
            # chosen outside [value_base, value_base+value_size) by the test
            # program. Disabled by default because ordinary data must not be rebased.
            if value_base is not None and value_base <= wdata < value_base + value_size:
                wdata -= value_base
            print(f"COMMIT pc={pc & 0xFFFFFFFF:08x} instr={instr:08x} rd={rd} wdata={wdata:08x}")
            count += 1
    return count


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("spike_log", help="raw Spike --log-commits log")
    parser.add_argument(
        "--pc-base",
        default="0x80000000",
        type=parse_int,
        help="base address to subtract from Spike PCs before printing",
    )
    parser.add_argument(
        "--keep-before-base",
        action="store_true",
        help="keep Spike boot/setup commits below pc-base instead of dropping them",
    )
    parser.add_argument(
        "--skip-before",
        default=None,
        type=parse_int,
        help="drop Spike commits below this raw PC; defaults to pc-base unless --keep-before-base is set",
    )
    parser.add_argument(
        "--value-base",
        default=None,
        type=parse_int,
        help="opt-in: subtract this base from wdata values inside the Spike memory window (address-bearing registers)",
    )
    parser.add_argument(
        "--value-size",
        default=0x20000,
        type=parse_int,
        help="size of the Spike memory window used with --value-base",
    )
    args = parser.parse_args()

    skip_before = None if args.keep_before_base else (
        args.skip_before if args.skip_before is not None else args.pc_base
    )
    count = normalize(args.spike_log, args.pc_base, skip_before,
                      args.value_base, args.value_size)
    if count == 0:
        print(f"no Spike commit lines found in {args.spike_log}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
