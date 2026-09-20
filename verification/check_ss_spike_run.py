#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright 2026 He Ning
# SPDX-License-Identifier: Apache-2.0

"""Reject truncated traces and unentered memory/ISA states in a superscalar run."""

from pathlib import Path
import re
import sys


out = Path(sys.argv[1])
stem = sys.argv[2]
count, stall, latency = map(int, sys.argv[3:6])
traces = []
for suffix in ("rtl.commit.log", "spike.commit.head.log"):
    lines = (out / f"{stem}.{suffix}").read_text().splitlines()
    if len(lines) != count or any(not line.startswith("COMMIT ") for line in lines):
        raise SystemExit(f"FAIL {suffix}: expected exactly {count} commits, got {len(lines)}")
    traces.append(lines)
if traces[0] != traces[1]:
    raise SystemExit("FAIL Spike/RTL commit traces differ")

raw = (out / f"{stem}.rtl.raw.log").read_text()
if "FATAL:" in raw or "ERROR:" in raw or f"TRACE DONE commits={count} " not in raw:
    raise SystemExit("FAIL RTL error or missing completion witness")
match = re.search(
    r"SPIKE_COVER loads=(\d+) stores=(\d+) branches=(\d+) muldiv=(\d+) "
    r"waits=(\d+) outstanding=(\d+) recoveries=(\d+)",
    raw,
)
if not match:
    raise SystemExit("FAIL missing coverage witness")
loads, stores, branches, muldiv, waits, outstanding, recoveries = map(int, match.groups())
if min(loads, stores, branches, muldiv, recoveries) == 0:
    raise SystemExit(f"FAIL unentered ISA/recovery class: {match.group()}")
if (stall and not waits) or (latency and not outstanding):
    raise SystemExit(f"FAIL unentered memory delay: {match.group()}")

spike_commits = []
for line in (out / f"{stem}.spike.raw.log").read_text().splitlines():
    item = re.search(
        r"core\s+\d+:\s+\d+\s+0x([0-9a-fA-F]+)\s+\(0x([0-9a-fA-F]+)\)",
        line,
    )
    if item and int(item[1], 16) >= 0x80000000:
        spike_commits.append((int(item[2], 16), line))

expected = []
for order, (instruction, line) in enumerate(spike_commits[:count]):
    if instruction & 0x7F != 0x23:
        continue
    effect = re.search(r"\bmem 0x([0-9a-fA-F]+) 0x([0-9a-fA-F]+)", line)
    if not effect:
        raise SystemExit("FAIL Spike store lacks memory effect")
    address, value = (int(value, 16) for value in effect.groups())
    size = 1 << ((instruction >> 12) & 7)
    if size > 4 or address % size:
        raise SystemExit("FAIL unsupported/misaligned Spike store")
    byte_enable = ((1 << size) - 1) << (address & 3)
    data = (value & ((1 << (8 * size)) - 1)) << (8 * (address & 3))
    expected.append((order, address, byte_enable, data))

actual = []
for line in raw.splitlines():
    if not line.startswith("STORE"):
        continue
    effect = re.fullmatch(
        r"STORE\s+addr=([0-9a-fA-F]+) data=([0-9a-fA-F]+) "
        r"wmask=([01]{4}) commit_order=([0-9a-fA-F]+)",
        line,
    )
    if not effect:
        raise SystemExit("FAIL malformed RTL store trace")
    address, data = int(effect[1], 16), int(effect[2], 16)
    byte_enable, order = int(effect[3], 2), int(effect[4], 16)
    mask = sum(0xFF << (8 * byte) for byte in range(4) if byte_enable & (1 << byte))
    actual.append((order, address, byte_enable, data & mask))
if actual != expected:
    raise SystemExit(
        f"FAIL Spike/RTL store effects differ ({len(expected)} expected, {len(actual)} actual)"
    )

print(f"PASS {out.name} commits={count} {match.group()}")
