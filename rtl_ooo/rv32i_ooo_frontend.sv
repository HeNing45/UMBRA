// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// rv32i_ooo_frontend - in-order fetch -> OoO decoded packet.
//
//   PC -> imem -> rv32i_ooo_decode -> rv32i_imm_gen -> OoO packet
//
// Decode lives in rv32i_ooo_decode (a wrapper over the pipeline
// decoder). This module is pure fetch + packet wiring: it forwards the decoder's
// OoO fields, runs imm-gen off the decoder's imm_sel, and packs the
// decode-detected trap (ecall/ebreak/illegal/mret) into decoded_trap using the
// mtval convention. CSR fields are decoded and forwarded to the backend.

module rv32i_ooo_frontend
  import rv32i_ooo_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import fyp_cpu_pkg::imm_sel_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;
  import rv32i_pipeline_pkg::trap_op_e;

#(
  parameter word_t RESET_PC = 32'h0000_0000
)(
  input  logic clk,
  input  logic rst_n,

  // instruction memory (combinational read)
  output word_t imem_addr,
  input  word_t imem_rdata,

  // redirect (branch recovery / jump / trap)
  input  logic  redirect_valid,
  input  word_t redirect_target,

  // decoded packet -> core
  output logic          decoded_valid,
  input  logic          decoded_ready,
  output word_t         decoded_pc,
  output word_t         decoded_instr,
  output arch_reg_t     decoded_rs1,
  output arch_reg_t     decoded_rs2,
  output arch_reg_t     decoded_rd,
  output logic          decoded_rd_we,
  output logic          decoded_needs_checkpoint,
  output ooo_op_class_e decoded_op_class,
  output ooo_fu_class_e decoded_fu_class,
  output muldiv_op_e    decoded_muldiv_op,
  output alu_op_e       decoded_alu_op,
  output br_type_e      decoded_branch_op,
  output ooo_src_sel_e  decoded_src1_sel,
  output ooo_src_sel_e  decoded_src2_sel,
  output word_t         decoded_imm,
  output decoded_trap_t decoded_trap,     // decode-detected trap
  output csr_op_e       decoded_csr_op,
  output csr_addr_t     decoded_csr_addr,
  output csr_zimm_t     decoded_csr_zimm,

  output logic           decoded_is_load,
  output logic           decoded_is_store,
  output mem_size_e      decoded_mem_size,
  output logic           decoded_mem_unsigned
);

  word_t       pc_q;
  logic        fetch_fire;
  logic [31:0] dec_imm;

  // ---- decoder outputs ----
  arch_reg_t     dec_rs1, dec_rs2, dec_rd;
  logic          dec_rd_we;
  ooo_op_class_e dec_op_class;
  ooo_fu_class_e dec_fu_class;
  alu_op_e       dec_alu_op;
  muldiv_op_e    dec_muldiv_op;
  br_type_e      dec_branch_op;
  ooo_src_sel_e  dec_src1_sel, dec_src2_sel;
  imm_sel_e      dec_imm_sel;
  trap_op_e      dec_trap_op;
  logic          dec_is_mret;
  logic          dec_is_csr;     // decoder CSR classification
  logic          dec_is_mem;     // decoder memory classification
  // Illegal instructions use TRAP_ILLEGAL; CSR operations travel to the backend.
  /* verilator lint_off UNUSEDSIGNAL */
  logic    dec_illegal;
  csr_op_e dec_csr_op;
  csr_addr_t dec_csr_addr;
  csr_zimm_t dec_csr_zimm;
  /* verilator lint_on UNUSEDSIGNAL */

  logic           dec_is_load;
  logic           dec_is_store;
  mem_size_e      dec_mem_size;
  logic           dec_mem_unsigned;

  // ---- fetch ----
  assign imem_addr  = pc_q;
  assign fetch_fire = decoded_valid & decoded_ready;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)              pc_q <= RESET_PC;
    else if (redirect_valid) pc_q <= redirect_target;
    else if (fetch_fire)     pc_q <= pc_q + 32'd4;
  end

  // ---- decode (wrapper over the pipeline decoder) ----
  rv32i_ooo_decode u_dec (
    .instr     (imem_rdata),
    .rs1       (dec_rs1),
    .rs2       (dec_rs2),
    .rd        (dec_rd),
    .rd_we     (dec_rd_we),
    .illegal   (dec_illegal),
    .op_class  (dec_op_class),
    .fu_class  (dec_fu_class),
    .alu_op    (dec_alu_op),
    .muldiv_op (dec_muldiv_op),
    .branch_op (dec_branch_op),
    .src1_sel  (dec_src1_sel),
    .src2_sel  (dec_src2_sel),
    .imm_sel   (dec_imm_sel),
    .is_csr    (dec_is_csr),
    .csr_op    (dec_csr_op),
    .is_mem    (dec_is_mem),
    .trap_op   (dec_trap_op),
    .is_mret   (dec_is_mret),
    .csr_addr  (dec_csr_addr),
    .csr_zimm  (dec_csr_zimm),
    .is_load    (dec_is_load),
    .is_store   (dec_is_store),
    .mem_size   (dec_mem_size),
    .mem_unsigned (dec_mem_unsigned)
  );

  rv32i_imm_gen u_imm (
    .instr   (imem_rdata),
    .imm_sel (dec_imm_sel),
    .imm     (dec_imm)
  );

  // ---- decoded packet wiring ----
  assign decoded_valid            = 1'b1;            // always fetching
  assign decoded_pc               = pc_q;
  assign decoded_instr            = imem_rdata;
  assign decoded_rs1              = dec_rs1;
  assign decoded_rs2              = dec_rs2;
  assign decoded_rd               = dec_rd;
  assign decoded_rd_we            = dec_rd_we;
  assign decoded_needs_checkpoint = (dec_op_class == OOO_OP_BRANCH);
  assign decoded_op_class         = dec_op_class;
  assign decoded_fu_class         = dec_fu_class;
  assign decoded_muldiv_op        = dec_muldiv_op;
  assign decoded_alu_op           = dec_alu_op;
  assign decoded_branch_op        = dec_branch_op;
  assign decoded_src1_sel         = dec_src1_sel;
  assign decoded_src2_sel         = dec_src2_sel;
  assign decoded_imm              = word_t'(dec_imm);
  assign decoded_csr_op           = dec_csr_op;
  assign decoded_csr_addr         = dec_csr_addr;
  assign decoded_csr_zimm         = dec_csr_zimm;
  assign decoded_is_load          = dec_is_load;
  assign decoded_is_store         = dec_is_store;
  assign decoded_mem_size         = dec_mem_size;
  assign decoded_mem_unsigned     = dec_mem_unsigned;

  // ---- decode-detected trap -> direct-cause form ----
  always_comb begin
    decoded_trap         = '0;
    decoded_trap.is_mret = dec_is_mret;
    unique case (dec_trap_op)
      rv32i_pipeline_pkg::TRAP_ILLEGAL: begin
        decoded_trap.valid = 1'b1; decoded_trap.cause = OOO_CAUSE_ILLEGAL;  decoded_trap.tval = imem_rdata;
      end
      rv32i_pipeline_pkg::TRAP_ECALL: begin
        decoded_trap.valid = 1'b1; decoded_trap.cause = OOO_CAUSE_ECALL_M;
      end
      rv32i_pipeline_pkg::TRAP_EBREAK: begin
        decoded_trap.valid = 1'b1; decoded_trap.cause = OOO_CAUSE_EBREAK;  decoded_trap.tval = pc_q;
      end
      default: ;  // TRAP_NONE / TRAP_MRET (via is_mret) / execute-detected misalign
    endcase
  end

endmodule
