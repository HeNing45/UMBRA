// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// Pipeline-stage types and the wider control struct carried across F/D/E/M/W.
// All "first" enum members are the safe/neutral value so that a packed-struct
// '0 reset produces a benign bubble (no reg_write, no mem_write, no branch).
package rv32i_pipeline_pkg;
  import fyp_cpu_pkg::*;

  // ----- conditional-branch type (jumps handled separately) ---------------
  typedef enum logic [2:0] {
    BR_NONE,
    BR_BEQ,
    BR_BNE,
    BR_BLT,
    BR_BGE,
    BR_BLTU,
    BR_BGEU
  } br_type_e;


  // ----- M-extension operation ------------------------------------------------
  // Signed and unsigned multiply, divide and remainder operations.
  typedef enum logic [3:0] {
    MD_NONE,
    MD_MULH,
    MD_MULHU,
    MD_DIV,
    MD_REM,
    MD_REMU,
    MD_MUL,
    MD_DIVU,
    MD_MULHSU
  } muldiv_op_e;

  typedef enum logic [2:0] {
    CSR_NONE,
    CSR_RW,
    CSR_RS,
    CSR_RC,
    CSR_RWI,
    CSR_RSI,
    CSR_RCI
  } csr_op_e;

  typedef enum logic [2:0] {
    TRAP_NONE,
    TRAP_ILLEGAL,
    TRAP_ECALL,
    TRAP_EBREAK,
    TRAP_MRET,
    TRAP_IADDR_MISALIGN,
    TRAP_LOAD_MISALIGN,
    TRAP_STORE_MISALIGN
  } trap_op_e;
  // ----- forwarding source for an EX-stage operand --------------------------
  typedef enum logic [1:0] {
    FWD_NONE,    // use register file value (rs_data_e)
    FWD_FROM_M,  // bypass EX/MEM result_m
    FWD_FROM_W   // bypass MEM/WB result_w
  } fwd_sel_e;

  // Memory-access size is defined in fyp_cpu_pkg and shared with the
  // single-cycle path.

  // ----- W-stage writeback source ------------------------------------------
  typedef enum logic [2:0] {
    RES_ALU,
    RES_MEM,
    RES_PC4,
    RES_MULDIV,
    RES_CSR
  } result_src_e;

  // ----- pipeline control word carried D -> E -> M -> W ---------------------
  // Reset/bubble = '0 : reg_write=0, mem_write=0, mem_read=0, alu_op=ALU_ADD,
  // branch_op=BR_NONE, is_jump/is_jalr/is_muldiv=0 -> safe nop.
  typedef struct packed {
    logic        valid;
    logic        reg_write;
    logic        mem_write;
    logic        mem_read;     // load (needed for load-use stall detection)
    mem_size_e   mem_size;
    logic        mem_unsigned; // 1 for lbu/lhu, 0 otherwise
    alu_op_e     alu_op;
    alu_a_sel_e  alu_a_sel;
    alu_b_sel_e  alu_b_sel;
    imm_sel_e    imm_sel;
    result_src_e result_src;
    br_type_e    branch_op;
    logic        is_jump;      // jal or jalr
    logic        is_jalr;      // jalr base is rs1, not pc
    logic        is_muldiv;
    muldiv_op_e  muldiv_op;
    logic        sim_halt;
    logic        illegal;
    csr_op_e     csr;
    trap_op_e    trap_op;
  } pipe_ctrl_t;

  // ----- decoded instruction payload carried through the pipe ---------------
  typedef struct packed {
    word_t       pc;
    word_t       pc_plus4;
    logic [31:0] instr;
    word_t       imm;
    word_t       rs1_data;
    word_t       rs2_data;
    reg_addr_t   rs1_addr;
    reg_addr_t   rs2_addr;
    reg_addr_t   rd_addr;
    logic        bp_pred_taken;
    word_t       bp_pred_target;
    pipe_ctrl_t  ctrl;
    logic [11:0] csr_addr;
    logic [4:0]  csr_zimm;
    word_t       trap_tval;
  } uop_t;

  function automatic uop_t uop_bubble();
    uop_bubble       = '0;
    uop_bubble.instr = 32'h00000013;
  endfunction

endpackage
