#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright 2026 He Ning
# SPDX-License-Identifier: Apache-2.0

"""Bound local simulations and check their architectural result markers."""

import argparse
import difflib
import os
from pathlib import Path
import re
import signal
import subprocess
import sys


def run(command, log, timeout):
    log = Path(log)
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("w") as output:
        process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            status = process.wait(timeout=timeout)
        except (subprocess.TimeoutExpired, KeyboardInterrupt) as error:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()
            reason = "Interrupted" if isinstance(error, KeyboardInterrupt) else f"Timeout after {timeout}s"
            raise RuntimeError(f"{reason}; see {log}") from None
    text = log.read_text(errors="replace")
    if status:
        print(text[-4000:], end="")
        raise RuntimeError(f"Command exited {status}; see {log}")
    return text


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=("test", "coremark", "embench", "spike"))
    parser.add_argument("--mode", choices=("performance", "validation"), default="performance")
    parser.add_argument("--log", default="build/run.log")
    parser.add_argument("--timeout", type=int, default=300)
    args, command = parser.parse_known_args()
    require(args.timeout > 0, "Timeout must be positive")
    if args.kind == "spike":
        require(not command, "Spike smoke test takes no additional commands")
        run(["spike", "--isa=rv32im", "-m0x80000000:0x20000", "--log-commits",
             "--instructions=80", "build/spike/alu.elf"], "build/spike/spike.log", args.timeout)
        normalized = run([sys.executable, "verification/normalize_spike_trace.py",
                          "build/spike/spike.log"], "build/spike/normalized.log", args.timeout)
        rtl = run(["vvp", "build/spike/rtl.vvp", "+TRACE", "+IMEM=build/spike/alu.mem",
                   "+MAX_COMMITS=16"], "build/spike/rtl.log", args.timeout)
        expected = [x for x in normalized.splitlines() if x.startswith("COMMIT ")][:16]
        actual = [x for x in rtl.splitlines() if x.startswith("COMMIT ")]
        require(len(expected) == len(actual) == 16, "Expected exactly 16 commits from each model")
        if expected != actual:
            print("\n".join(difflib.unified_diff(expected, actual, fromfile="Spike", tofile="RTL")))
            raise RuntimeError("Spike/RTL commit trace mismatch")
        print("PASS: 16 committed instructions match Spike")
        return

    require(bool(command), "Missing simulator command")
    text = run(command, args.log, args.timeout)
    if args.kind == "test":
        counted = re.search(r"PASS checks=[1-9]\d*", text)
        latency_test = any("tb_rv32i_ss_dmem_scratchpad_latk" in arg for arg in command)
        latency_pass = latency_test and "SCRATCHPAD LATK CONFORMANCE PASS" in text
        require(counted or latency_pass, "Missing non-empty testbench PASS result")
    elif args.kind == "coremark":
        checks = ("e9f5", "e714", "1fd7", "8e3a") if args.mode == "performance" else ("18f2", "e3c1", "0747", "8d84")
        for label, value in zip(("seedcrc", "[0]crclist", "[0]crcmatrix", "[0]crcstate"), checks):
            require(re.search(re.escape(label) + r"\s*:\s*0x" + value + r"\b", text), f"Wrong or missing {label}")
        require("COREMARK_TARGET_DONE cycles=" in text, "Missing architectural completion")
        errors = [line for line in text.splitlines() if "ERROR!" in line and
                  "Must execute for at least 10 secs for a valid result" not in line]
        require(not errors, "Unexpected CoreMark error: " + "; ".join(errors))
    else:
        require(re.search(r"EMBENCH_TICKS\s+[1-9]\d*", text), "Missing Embench cycle count")
        require("COREMARK_TARGET_DONE cycles=" in text, "Missing architectural completion")
    print(text, end="")
    print(f"PASS: {args.kind}; log: {args.log}")
    if args.kind != "test":
        print("RTL functional/cycle measurement only; not a qualifying hardware benchmark result.")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        sys.exit(1)
