#!/bin/bash
# SPDX-FileCopyrightText: Copyright 2026 He Ning
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

SRC="${1:-sw/ooo_m2_alu.S}"
OUT_DIR="${OUT_DIR:-build/spike_diff}"
SIM_TIMEOUT_SEC="${SIM_TIMEOUT_SEC:-30}"
SPIKE_TEXT_BASE="${SPIKE_TEXT_BASE:-0x80000000}"
SPIKE_MEM_SIZE="${SPIKE_MEM_SIZE:-0x20000}"
MAX_COMMITS="${MAX_COMMITS:-16}"
RTL_RESET_PC="${RTL_RESET_PC:-0}"
NORMALIZE_PC_BASE="${NORMALIZE_PC_BASE:-$SPIKE_TEXT_BASE}"
NORMALIZE_SKIP_BEFORE="${NORMALIZE_SKIP_BEFORE:-}"
RTL_TB_TOP="${RTL_TB_TOP:-tb_rv32i_ss_core_spike_diff}"
RTL_FILELIST="${RTL_FILELIST:-rtl_superscalar/filelist_superscalar.f}"

mkdir -p "$OUT_DIR"
bash verification/build_mem.sh "$SRC" "$OUT_DIR"

STEM="$(basename "$SRC")"
STEM="${STEM%.*}"
ELF="$OUT_DIR/$STEM.elf"
MEM="$OUT_DIR/$STEM.mem"
SPIKE_LOG="$OUT_DIR/$STEM.spike.raw.log"
SPIKE_TRACE="$OUT_DIR/$STEM.spike.commit.log"
SPIKE_TRACE_HEAD="$OUT_DIR/$STEM.spike.commit.head.log"
RTL_BIN="$OUT_DIR/$STEM.$RTL_TB_TOP.vvp"
RTL_BUILD_LOG="$OUT_DIR/$STEM.tb.build.log"
RTL_RAW_LOG="$OUT_DIR/$STEM.rtl.raw.log"
RTL_TRACE="$OUT_DIR/$STEM.rtl.commit.log"

python3 verification/run.py raw --timeout "$SIM_TIMEOUT_SEC" --log "$SPIKE_LOG" \
  spike --isa=rv32im -m"${SPIKE_TEXT_BASE}:${SPIKE_MEM_SIZE}" \
  --log-commits --instructions="$((MAX_COMMITS + 50))" "$ELF"

NORMALIZE_ARGS=(--pc-base "$NORMALIZE_PC_BASE")
if [ -n "${NORMALIZE_VALUE_BASE:-}" ]; then
  NORMALIZE_ARGS+=(--value-base "$NORMALIZE_VALUE_BASE" --value-size "$SPIKE_MEM_SIZE")
fi
if [ -n "$NORMALIZE_SKIP_BEFORE" ]; then
  NORMALIZE_ARGS+=(--skip-before "$NORMALIZE_SKIP_BEFORE")
fi
python3 verification/normalize_spike_trace.py "${NORMALIZE_ARGS[@]}" "$SPIKE_LOG" > "$SPIKE_TRACE"
head -n "$MAX_COMMITS" "$SPIKE_TRACE" > "$SPIKE_TRACE_HEAD"

EXTRA_DEFINES="${SS_EXTRA_DEFINES:-}"
python3 verification/run.py raw --timeout "$SIM_TIMEOUT_SEC" --log "$RTL_BUILD_LOG" \
  iverilog -g2012 -o "$RTL_BIN" -s "$RTL_TB_TOP" \
  -DOOO_SPIKE_RESET_PC="$RTL_RESET_PC" $EXTRA_DEFINES \
  -f "$RTL_FILELIST" "tb/$RTL_TB_TOP.sv" tb/ooo_dmem_model.sv

python3 verification/run.py raw --timeout "$SIM_TIMEOUT_SEC" --log "$RTL_RAW_LOG" \
  vvp "$RTL_BIN" +TRACE "+IMEM=$MEM" "+MAX_COMMITS=$MAX_COMMITS"
grep '^COMMIT ' "$RTL_RAW_LOG" > "$RTL_TRACE" || true

diff -u "$SPIKE_TRACE_HEAD" "$RTL_TRACE"
printf 'SPIKE DIFF PASS commits=%s source=%s\n' "$MAX_COMMITS" "$SRC"
