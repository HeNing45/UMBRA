`timescale 1ns/1ps

// umbra_ss_cpu_top - wiring-only CPU top: rv32i_ss_frontend + rv32i_ss_core.
//
// ZERO logic by contract: no state, no control, no expression on any
// connection. Every value crossing this module keeps ONE name on both sides
// of every boundary and travels through a plain net.
// All integration semantics -- the decoded_* packet, ready backpressure, and
// the redirect loop -- live in the two children; this module exists so one
// instantiation gives the whole CPU: clk/rst_n, the imem request/response
// channel, the dmem handshake, and the commit-trace channel.
//
// The commit_* ports are re-exposed unchanged; the observation-only contract
// and the commit_fire exception are documented at the core's port header.

module umbra_ss_cpu_top
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;
#(
  parameter word_t RESET_PC = 32'h0000_0000
)(
  input  logic clk,
  input  logic rst_n,

  // instruction request/response channel
  output logic        imem_req_valid,
  input  logic        imem_req_ready,
  output word_t       imem_req_addr,
  input  logic        imem_resp_valid,
  output logic        imem_resp_ready,
  input  word_t [1:0] imem_resp_data,

  // commit-trace channel (Spike-diff / parity harnesses)
  output logic [1:0]      commit_fire,
  output commit_order_t   commit_order,
  output word_t [1:0]     commit_pc,
  output word_t [1:0]     commit_inst,
  output arch_reg_t [1:0] commit_rd,
  output logic [1:0]      commit_rd_wen,
  output word_t [1:0]     commit_wdata,

  // dmem handshake
  output logic        dmem_valid,
  output logic        dmem_we,
  output logic  [3:0] dmem_be,
  output word_t       dmem_addr,
  output word_t       dmem_wdata,
  input  logic        dmem_ready,
  input  logic        dmem_rvalid,
  input  word_t       dmem_rdata
);

  // frontend -> core decoded packet; core -> frontend ready + redirect
  logic          decoded_valid;
  logic [1:0]    decoded_slot_valid;
  logic          decoded_ready;
  word_t         [1:0] decoded_pc;
  word_t         [1:0] decoded_instr;
  arch_reg_t     [1:0] decoded_rs1;
  arch_reg_t     [1:0] decoded_rs2;
  arch_reg_t     [1:0] decoded_rd;
  logic          [1:0] decoded_rd_we;
  logic          [1:0] decoded_needs_checkpoint;
  ooo_op_class_e [1:0] decoded_op_class;
  ooo_fu_class_e [1:0] decoded_fu_class;
  muldiv_op_e    [1:0] decoded_muldiv_op;
  alu_op_e       [1:0] decoded_alu_op;
  br_type_e      [1:0] decoded_branch_op;
  ooo_src_sel_e  [1:0] decoded_src1_sel;
  ooo_src_sel_e  [1:0] decoded_src2_sel;
  word_t         [1:0] decoded_imm;
  decoded_trap_t decoded_trap;
  csr_op_e       decoded_csr_op;
  csr_addr_t     decoded_csr_addr;
  csr_zimm_t     decoded_csr_zimm;
  logic      [1:0] decoded_is_load;
  logic      [1:0] decoded_is_store;
  mem_size_e [1:0] decoded_mem_size;
  logic      [1:0] decoded_mem_unsigned;
  logic      [1:0] decoded_pred_taken;
  word_t           decoded_pred_target;

  logic  redirect_valid;
  word_t redirect_target;

  // core -> frontend predictor training
  logic  bp_update_valid;
  word_t bp_update_pc;
  logic  bp_update_taken;
  word_t bp_update_target;
  logic  bp_return_update_valid;
  word_t bp_return_update_pc;
  logic  ras_fetch_valid;
  word_t ras_fetch_target;

  rv32i_ss_frontend #(
    .RESET_PC (RESET_PC)
  ) u_fe (
    .clk                      (clk),
    .rst_n                    (rst_n),
    .imem_req_valid           (imem_req_valid),
    .imem_req_ready           (imem_req_ready),
    .imem_req_addr            (imem_req_addr),
    .imem_resp_valid          (imem_resp_valid),
    .imem_resp_ready          (imem_resp_ready),
    .imem_resp_data           (imem_resp_data),
    .redirect_valid           (redirect_valid),
    .redirect_target          (redirect_target),
    .bp_update_valid          (bp_update_valid),
    .bp_update_pc             (bp_update_pc),
    .bp_update_taken          (bp_update_taken),
    .bp_update_target         (bp_update_target),
    .bp_return_update_valid   (bp_return_update_valid),
    .bp_return_update_pc      (bp_return_update_pc),
    .ras_fetch_valid          (ras_fetch_valid),
    .ras_fetch_target         (ras_fetch_target),
    .decoded_valid            (decoded_valid),
    .decoded_slot_valid       (decoded_slot_valid),
    .decoded_ready            (decoded_ready),
    .decoded_pc               (decoded_pc),
    .decoded_instr            (decoded_instr),
    .decoded_rs1              (decoded_rs1),
    .decoded_rs2              (decoded_rs2),
    .decoded_rd               (decoded_rd),
    .decoded_rd_we            (decoded_rd_we),
    .decoded_needs_checkpoint (decoded_needs_checkpoint),
    .decoded_op_class         (decoded_op_class),
    .decoded_fu_class         (decoded_fu_class),
    .decoded_muldiv_op        (decoded_muldiv_op),
    .decoded_alu_op           (decoded_alu_op),
    .decoded_branch_op        (decoded_branch_op),
    .decoded_src1_sel         (decoded_src1_sel),
    .decoded_src2_sel         (decoded_src2_sel),
    .decoded_imm              (decoded_imm),
    .decoded_trap             (decoded_trap),
    .decoded_csr_op           (decoded_csr_op),
    .decoded_csr_addr         (decoded_csr_addr),
    .decoded_csr_zimm         (decoded_csr_zimm),
    .decoded_is_load          (decoded_is_load),
    .decoded_is_store         (decoded_is_store),
    .decoded_mem_size         (decoded_mem_size),
    .decoded_mem_unsigned     (decoded_mem_unsigned),
    .decoded_pred_taken       (decoded_pred_taken),
    .decoded_pred_target      (decoded_pred_target)
  );

  rv32i_ss_core u_core (
    .clk                      (clk),
    .rst_n                    (rst_n),
    .decoded_valid            (decoded_valid),
    .decoded_slot_valid       (decoded_slot_valid),
    .decoded_ready            (decoded_ready),
    .decoded_pc               (decoded_pc),
    .decoded_instr            (decoded_instr),
    .decoded_rs1              (decoded_rs1),
    .decoded_rs2              (decoded_rs2),
    .decoded_rd               (decoded_rd),
    .decoded_rd_we            (decoded_rd_we),
    .decoded_needs_checkpoint (decoded_needs_checkpoint),
    .decoded_op_class         (decoded_op_class),
    .decoded_fu_class         (decoded_fu_class),
    .decoded_alu_op           (decoded_alu_op),
    .decoded_muldiv_op        (decoded_muldiv_op),
    .decoded_branch_op        (decoded_branch_op),
    .decoded_src1_sel         (decoded_src1_sel),
    .decoded_src2_sel         (decoded_src2_sel),
    .decoded_imm              (decoded_imm),
    .decoded_trap             (decoded_trap),
    .decoded_csr_op           (decoded_csr_op),
    .decoded_csr_addr         (decoded_csr_addr),
    .decoded_csr_zimm         (decoded_csr_zimm),
    .decoded_is_load          (decoded_is_load),
    .decoded_is_store         (decoded_is_store),
    .decoded_mem_size         (decoded_mem_size),
    .decoded_mem_unsigned     (decoded_mem_unsigned),
    .decoded_pred_taken       (decoded_pred_taken),
    .decoded_pred_target      (decoded_pred_target),
    .redirect_valid           (redirect_valid),
    .redirect_target          (redirect_target),
    .bp_update_valid          (bp_update_valid),
    .bp_update_pc             (bp_update_pc),
    .bp_update_taken          (bp_update_taken),
    .bp_update_target         (bp_update_target),
    .bp_return_update_valid   (bp_return_update_valid),
    .bp_return_update_pc      (bp_return_update_pc),
    .ras_fetch_valid          (ras_fetch_valid),
    .ras_fetch_target         (ras_fetch_target),
    .commit_fire              (commit_fire),
    .commit_order             (commit_order),
    .commit_pc                (commit_pc),
    .commit_inst              (commit_inst),
    .commit_rd                (commit_rd),
    .commit_rd_wen            (commit_rd_wen),
    .commit_wdata             (commit_wdata),
    .dmem_valid               (dmem_valid),
    .dmem_we                  (dmem_we),
    .dmem_be                  (dmem_be),
    .dmem_addr                (dmem_addr),
    .dmem_wdata               (dmem_wdata),
    .dmem_ready               (dmem_ready),
    .dmem_rvalid              (dmem_rvalid),
    .dmem_rdata               (dmem_rdata)
  );

endmodule
