// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps
package rv32i_ss_pkg;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;

  parameter int OOO_XLEN         = 32;
  // ISA-FIXED (RV32I x0..x31): NOT a configuration knob. Decode's 5-bit
  // register fields, x0 semantics, and the rename map all assume exactly 32
  // architectural registers; rv32i_ss_rename guards this at elaboration.
  parameter int OOO_ARCH_REGS    = 32;
  parameter int OOO_PHYS_REGS    = 64;
  parameter int OOO_ROB_DEPTH    = 32;
  parameter int OOO_IQ_DEPTH     = 16;
  parameter int OOO_BRANCH_CKPTS = 8;
  parameter int SS_LQ_DEPTH      = 8;
  parameter int SS_SQ_DEPTH      = 8;

  // Index widths derive from capacity parameters to avoid truncation and
  // aliasing when capacity changes. The ROB and LSQ separately enforce the
  // power-of-two queue geometry required by their ring arithmetic.
  localparam int OOO_ARCH_BITS = $clog2(OOO_ARCH_REGS);
  localparam int OOO_PHYS_BITS = $clog2(OOO_PHYS_REGS);
  localparam int OOO_ROB_BITS  = $clog2(OOO_ROB_DEPTH);
  localparam int OOO_IQ_BITS   = $clog2(OOO_IQ_DEPTH);
  localparam int OOO_CKPT_BITS = $clog2(OOO_BRANCH_CKPTS);
  localparam int SS_LQ_BITS    = $clog2(SS_LQ_DEPTH);
  localparam int SS_SQ_BITS    = $clog2(SS_SQ_DEPTH);

  typedef logic [OOO_XLEN-1:0]         word_t;
  typedef logic [OOO_ARCH_BITS-1:0]    arch_reg_t;
  typedef logic [OOO_PHYS_BITS-1:0]    phys_reg_t;
  typedef logic [OOO_ROB_BITS-1:0]     rob_idx_t;
  typedef logic [SS_LQ_BITS-1:0]       lq_idx_t;
  typedef logic [SS_SQ_BITS-1:0]       sq_idx_t;
  typedef logic [OOO_CKPT_BITS-1:0]    ckpt_idx_t;
  typedef logic [OOO_BRANCH_CKPTS-1:0] branch_mask_t;
  typedef logic [63:0]                 commit_order_t;
  typedef logic [63:0]                 rob_seq_t;
  typedef logic [11:0]                 csr_addr_t;
  typedef logic [4:0]                  csr_zimm_t;

  // M-mode synchronous trap cause codes used by this core (mcause values).
  parameter word_t OOO_CAUSE_IADDR_MISALIGN = 32'd0;
  parameter word_t OOO_CAUSE_ILLEGAL        = 32'd2;
  parameter word_t OOO_CAUSE_EBREAK         = 32'd3;
  parameter word_t OOO_CAUSE_LOAD_MISALIGN  = 32'd4;
  parameter word_t OOO_CAUSE_STORE_MISALIGN = 32'd6;
  parameter word_t OOO_CAUSE_ECALL_M        = 32'd11;

  typedef enum logic [1:0] {
    OOO_OP_ALU,
    OOO_OP_BRANCH,
    OOO_OP_JUMP
  } ooo_op_class_e;

  typedef enum logic [1:0] {
    OOO_FU_ALU,
    OOO_FU_MULDIV,
    OOO_FU_LSU
  } ooo_fu_class_e;

  typedef enum logic [1:0] {
    ISSUE_UNIT_ALU0,
    ISSUE_UNIT_ALU1,
    ISSUE_UNIT_MULDIV,
    ISSUE_UNIT_AGEN
  } issue_unit_e;

  typedef enum logic [1:0] {
    OOO_SRC_REG,
    OOO_SRC_IMM,
    OOO_SRC_PC,
    OOO_SRC_ZERO
  } ooo_src_sel_e;

  typedef struct packed {
    word_t         pc;
    rob_idx_t      rob_idx;
    rob_seq_t      rob_seq;
    ooo_op_class_e op_class;
    ooo_fu_class_e fu_class;
    alu_op_e       alu_op;
    br_type_e      branch_op;
    muldiv_op_e    muldiv_op;
    phys_reg_t     prs1;
    phys_reg_t     prs2;
    phys_reg_t     pdst;
    ooo_src_sel_e  src1_sel;
    ooo_src_sel_e  src2_sel;
    word_t         imm;
    logic          rd_wen;
    branch_mask_t  branch_mask;
    ckpt_idx_t     checkpoint_id;
    csr_op_e       csr_op;
    csr_addr_t     csr_addr;
    csr_zimm_t     csr_zimm;
    logic          is_load;
    logic          is_store;
    mem_size_e     mem_size;
    logic          mem_unsigned;
    lq_idx_t       lq_idx;
    sq_idx_t       sq_idx;
    // Fetch-time branch prediction carried to resolution. A zero taken bit
    // selects not-taken; a taken prediction carries its target for validation.
    logic          pred_taken;
    word_t         pred_target;

  } iq_entry_t;

  // Registered execute input. Fixed banks; the
  // core executes the oldest surviving occupant and never shifts payload.
  typedef struct packed {
    logic        valid;
    iq_entry_t   uop;
    word_t       op_a;
    word_t       op_b;
    logic        store_data_valid;
    logic        store_data_pending;
    phys_reg_t   store_prs2;
    word_t       store_data;
  } exec_input_slot_t;

  typedef struct packed {
    logic      valid;
    rob_idx_t  rob_idx;
    logic      addr_valid;
    word_t     addr;
    mem_size_e mem_size;
    logic      mem_unsigned;
    phys_reg_t pdst;
    logic      rd_wen;
    rob_seq_t  rob_seq;
    logic      executed;
    logic      inert;
  } lq_entry_t;

  typedef struct packed {
    logic       valid;
    rob_idx_t   rob_idx;
    rob_seq_t   rob_seq;
    phys_reg_t  data_preg;
    logic       addr_valid;
    logic       data_valid;
    logic       deferred_pending;
    word_t      addr;
    word_t      data;
    logic [3:0] be;
    logic       inert;
  } sq_entry_t;

  typedef struct packed {
    logic      resolve_valid;
    logic      recover_valid;
    logic      correct_valid;
    ckpt_idx_t checkpoint_id;
    rob_idx_t  rob_idx;
    word_t     target;
  } branch_candidate_t;

  typedef struct packed {
    logic      valid;     // a completion is offered this cycle (= FU done)
    rob_idx_t  rob_idx;
    rob_seq_t  rob_seq;
    phys_reg_t pdst;
    logic      rd_wen;
    word_t     result;
    logic      trap_valid;
    word_t     trap_cause;
    word_t     trap_tval;
    logic      csr_we;
    word_t     csr_wdata;
  } completion_packet_t;

  // Per-ALU registered result capacity. `older` is production order among the
  // two banks of one ALU: the CDB client offers the earlier packet; both live
  // packets remain bypass-eligible.
  typedef struct packed {
    logic               valid;
    logic               live;
    logic               older;
    completion_packet_t pkt;
  } exec_result_slot_t;

  // decode-detected trap, carried frontend -> core -> ROB.
  typedef struct packed {
    logic  valid;     // a trap was decoded (ecall / ebreak / illegal)
    word_t cause;     // mcause code (2 illegal / 3 ebreak / 11 ecall)
    word_t tval;      // mtval
    logic  is_mret;   // mret = trap-return (NOT a trap; valid stays 0)
  } decoded_trap_t;

endpackage
