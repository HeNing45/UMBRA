// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

// Shared text trace format for ROB and Spike-lockstep testbenches.
// Include this from simulation-only code; these text records are not a
// synthesizable interface contract.
`ifndef RV32I_OOO_TRACE_FORMAT_SVH
`define RV32I_OOO_TRACE_FORMAT_SVH

`define RV32I_OOO_SEQ_NONE 64'hffff_ffff_ffff_ffff

typedef struct packed {
  logic [31:0] pc;
  logic [31:0] inst;
  logic        rd_wen;
  logic [4:0]  rd;
  logic [31:0] wdata;
  logic        trap_valid;
  logic [31:0] trap_cause;
  logic [31:0] trap_tval;
  logic        is_mret;
  logic [63:0] commit_order;
} rv32i_ooo_commit_trace_t;

typedef struct packed {
  logic [31:0] addr;
  logic [31:0] data;
  logic [3:0]  wmask;
  logic [63:0] commit_order;
} rv32i_ooo_store_trace_t;

typedef struct packed {
  logic [63:0] uop_seq;
  logic [63:0] dispatch_seq;
  logic [63:0] issue_seq;
  logic [63:0] execute_seq;
  logic [63:0] writeback_seq;
  logic [63:0] commit_seq;
  logic [4:0]  rob_idx;
  logic [5:0]  pdst;
  logic [5:0]  prs1;
  logic [5:0]  prs2;
  logic [31:0] pc;
  logic [31:0] inst;
} rv32i_ooo_debug_trace_t;

`define RV32I_OOO_COMMIT_TRACE_FMT \
  "COMMIT pc=%08h inst=%08h rd_wen=%0d rd=%0d wdata=%08h trap_valid=%0d trap_cause=%08h trap_tval=%08h is_mret=%0d commit_order=%016h"

`define RV32I_OOO_COMMIT_TRACE_ARGS(t_) \
  (t_).pc, (t_).inst, (t_).rd_wen, (t_).rd, (t_).wdata, \
  (t_).trap_valid, (t_).trap_cause, (t_).trap_tval, (t_).is_mret, \
  (t_).commit_order

`define RV32I_OOO_STORE_TRACE_FMT \
  "STORE  addr=%08h data=%08h wmask=%04b commit_order=%016h"

`define RV32I_OOO_STORE_TRACE_ARGS(t_) \
  (t_).addr, (t_).data, (t_).wmask, (t_).commit_order

`define RV32I_OOO_DEBUG_TRACE_FMT \
  "OOO_DBG uop_seq=%016h dispatch_seq=%016h issue_seq=%016h execute_seq=%016h writeback_seq=%016h commit_seq=%016h rob_idx=%02h pdst=%02h prs1=%02h prs2=%02h pc=%08h inst=%08h"

`define RV32I_OOO_DEBUG_TRACE_ARGS(t_) \
  (t_).uop_seq, (t_).dispatch_seq, (t_).issue_seq, (t_).execute_seq, \
  (t_).writeback_seq, (t_).commit_seq, (t_).rob_idx, (t_).pdst, \
  (t_).prs1, (t_).prs2, (t_).pc, (t_).inst

`define RV32I_OOO_TRACE_EMIT_COMMIT(t_) \
  $display(`RV32I_OOO_COMMIT_TRACE_FMT, `RV32I_OOO_COMMIT_TRACE_ARGS(t_))

`define RV32I_OOO_TRACE_EMIT_STORE(t_) \
  $display(`RV32I_OOO_STORE_TRACE_FMT, `RV32I_OOO_STORE_TRACE_ARGS(t_))

`define RV32I_OOO_TRACE_EMIT_DEBUG(t_) \
  $display(`RV32I_OOO_DEBUG_TRACE_FMT, `RV32I_OOO_DEBUG_TRACE_ARGS(t_))

`endif
