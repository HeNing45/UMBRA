#!/bin/bash
# SPDX-FileCopyrightText: Copyright 2026 He Ning
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SRC="${1:?usage: verification/build_mem.sh <source.S|source.c> [out_dir]}"
OUT_DIR="${2:-$ROOT_DIR/build/spike_diff}"

SPIKE_TEXT_BASE="${SPIKE_TEXT_BASE:-0x80000000}"
RTL_REBASE_DELTA="${RTL_REBASE_DELTA:--0x80000000}"
MARCH="${MARCH:-rv32im}"
MABI="${MABI:-ilp32}"
CROSS="${CROSS:-riscv-none-elf-}"

XPACK_ROOT="$HOME/Library/xPacks/@xpack-dev-tools/riscv-none-elf-gcc"
XPACK_BIN=""
if [ -d "$XPACK_ROOT" ]; then
  XPACK_BIN="$(find "$XPACK_ROOT" -mindepth 3 -maxdepth 3 -type d -path '*/.content/bin' 2>/dev/null | sort | tail -n 1 || true)"
fi

if command -v "${CROSS}gcc" >/dev/null 2>&1; then
  GCC="${CROSS}gcc"
  OBJCOPY="${CROSS}objcopy"
  OBJDUMP="${CROSS}objdump"
elif [ "$CROSS" = "riscv-none-elf-" ] && [ -n "$XPACK_BIN" ] && [ -x "$XPACK_BIN/riscv-none-elf-gcc" ]; then
  GCC="$XPACK_BIN/riscv-none-elf-gcc"
  OBJCOPY="$XPACK_BIN/riscv-none-elf-objcopy"
  OBJDUMP="$XPACK_BIN/riscv-none-elf-objdump"
else
  echo "${CROSS}gcc not found on PATH or in the default xPack install path" >&2
  exit 1
fi

mkdir -p "$OUT_DIR"

BASE="$(basename "$SRC")"
STEM="${BASE%.*}"
ELF="$OUT_DIR/$STEM.elf"
MEM="$OUT_DIR/$STEM.mem"
DUMP="$OUT_DIR/$STEM.dump"

"$GCC" -march="$MARCH" -mabi="$MABI" -nostdlib -nostartfiles \
  -Wl,-N "-Ttext=$SPIKE_TEXT_BASE" \
  -o "$ELF" "$SRC"

"$OBJCOPY" --change-addresses="$RTL_REBASE_DELTA" \
  -O verilog --verilog-data-width=4 \
  "$ELF" "$MEM"

"$OBJDUMP" -d "$ELF" > "$DUMP"

printf 'ELF  %s\nMEM  %s\nDUMP %s\n' "$ELF" "$MEM" "$DUMP"
