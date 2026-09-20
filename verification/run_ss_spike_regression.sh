#!/bin/bash
# SPDX-FileCopyrightText: Copyright 2026 He Ning
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail
cd "$(dirname "$0")/.."

RUN_DIR="${SS_SPIKE_RUN_DIR:-build/ss_spike_$(date +%Y%m%d_%H%M%S)}"
COMMITS="${SS_SPIKE_COMMITS:-4096}"
SEEDS="${SS_SPIKE_SEEDS:-1 7 42}"
if ! [[ "$COMMITS" =~ ^[0-9]+$ ]] || (( COMMITS < 1024 || COMMITS > 8192 )); then
  echo 'SS_SPIKE_COMMITS must be 1024..8192' >&2
  exit 2
fi
if [ -e "$RUN_DIR" ]; then
  echo "Refusing to overwrite existing run directory: $RUN_DIR" >&2
  exit 2
fi
mkdir -p "$RUN_DIR"

{
  git rev-parse HEAD
  git status --short
  printf 'commits=%s seeds=%s\n' "$COMMITS" "$SEEDS"
  shasum -a 256 rtl_superscalar/*.sv rtl_superscalar/filelist_superscalar.f \
    tb/tb_rv32i_ss_core_spike_diff.sv tb/ooo_dmem_model.sv \
    verification/run_ss_spike_regression.sh verification/gen_ss_spike_program.py \
    verification/check_ss_spike_run.py verification/run_spike_diff.sh \
    verification/build_mem.sh verification/run.py \
    verification/normalize_spike_trace.py
  while IFS= read -r source; do
    [[ -z "$source" || "$source" == \#* ]] && continue
    shasum -a 256 "$source"
  done < rtl_superscalar/filelist_superscalar.f
  command -v spike iverilog
  shasum -a 256 "$(command -v spike)" "$(command -v iverilog)"
} > "$RUN_DIR/manifest.txt"

cases=0
for seed in $SEEDS; do
  [[ "$seed" =~ ^[0-9]+$ ]] || { echo "Invalid seed: $seed" >&2; exit 2; }
  src="$RUN_DIR/ss_seed_${seed}.S"
  python3 verification/gen_ss_spike_program.py --seed "$seed" > "$src"
  for timing in '0 0' '0 1' '3 4'; do
    read -r stall latency <<< "$timing"
    out="$RUN_DIR/seed_${seed}_s${stall}_l${latency}"
    mkdir -p "$out"
    if ! MAX_COMMITS="$COMMITS" OUT_DIR="$out" SIM_TIMEOUT_SEC=120 \
      RTL_RESET_PC=2147483648 SPIKE_TEXT_BASE=0x80000000 RTL_REBASE_DELTA=-0x80000000 \
      NORMALIZE_PC_BASE=0 NORMALIZE_VALUE_BASE= NORMALIZE_SKIP_BEFORE=2147483648 \
      RTL_TB_TOP=tb_rv32i_ss_core_spike_diff \
      RTL_FILELIST=rtl_superscalar/filelist_superscalar.f \
      SS_EXTRA_DEFINES="-DUMBRA_M3_IMEM_LATENCY=1 -DUMBRA_M3_IMEM_PIPELINED=1 -DSS_SPIKE_READY_STALL=$stall -DSS_SPIKE_RESP_LATENCY=$latency" \
      bash verification/run_spike_diff.sh "$src" > "$out/run.log" 2>&1; then
      echo "FAIL seed=$seed stall=$stall latency=$latency: $out/run.log"
      exit 1
    fi
    python3 verification/check_ss_spike_run.py \
      "$out" "ss_seed_${seed}" "$COMMITS" "$stall" "$latency"
    cases=$((cases + 1))
  done
done

(( cases > 0 )) || { echo 'No seeds executed' >&2; exit 1; }
printf 'SS SPIKE REGRESSION PASS cases=%s commits_per_case=%s total=%s\n' \
  "$cases" "$COMMITS" "$((cases * COMMITS))"
