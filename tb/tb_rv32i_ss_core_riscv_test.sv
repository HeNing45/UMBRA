// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// tb_rv32i_ss_core_riscv_test — strict riscv-tests tohost harness for the
// superscalar OoO core.
//
// OoO twin of tb_rv32i_pipeline_riscv_test.sv: loads the same rebased .mem into
// imem AND dmem (split-memory Harvard trick — self-modifying code / fence_i is
// an accepted cut, same as the in-order harness), then judges STRICTLY by
// stores to the riscv-tests tohost word:
//   tohost <- 1        => RISCV_TEST PASS
//   tohost <- nonzero  => RISCV_TEST FAIL (+ fatal)
// The tohost watch lives in ooo_dmem_model (one-cycle pulse after the
// byte-enable merge); this TB only judges.
//
// Standing known-cut expectations (mirror the in-order documented cuts):
//   rv32ui-p-fence_i  — split-memory TB limitation
//   rv32ui-p-ma_data  — expects hardware misaligned-data emulation; this core
//                       traps instead


// The instruction environment is rv32i_ss_imem_scratchpad. Both defaults
// are 0, reproducing rv32i_ss_imem_zero_latency_adapter.
// The pipelined-correctness run passes +define+UMBRA_M3_IMEM_LATENCY=1 and
// +define+UMBRA_M3_IMEM_PIPELINED=1. Both are parameters, fixed at elaboration.
`ifndef UMBRA_M3_IMEM_LATENCY
`define UMBRA_M3_IMEM_LATENCY 0
`endif
`ifndef UMBRA_M3_IMEM_PIPELINED
`define UMBRA_M3_IMEM_PIPELINED 0
`endif

module tb_rv32i_ss_core_riscv_test;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;

  localparam int MEM_WORDS = 65536;   // 256 KiB, [17:2]
  localparam int MEM_MSB   = 17;

  logic clk;
  logic rst_n;

  logic          dec_valid, dec_ready;
  logic [1:0] dec_slot_valid;
  word_t [1:0] dec_pc, dec_instr;
  arch_reg_t [1:0] dec_rs1, dec_rs2, dec_rd;
  logic [1:0] dec_rd_we, dec_needs_checkpoint;
  ooo_op_class_e [1:0] dec_op_class;
  ooo_fu_class_e [1:0] dec_fu_class;
  muldiv_op_e [1:0]    dec_muldiv_op;
  alu_op_e [1:0]       dec_alu_op;
  br_type_e [1:0]      dec_branch_op;
  ooo_src_sel_e [1:0] dec_src1_sel, dec_src2_sel;
  word_t [1:0]         dec_imm;
  decoded_trap_t dec_trap;
  csr_op_e       dec_csr_op;
  csr_addr_t     dec_csr_addr;
  csr_zimm_t     dec_csr_zimm;
  logic [1:0] dec_is_load, dec_is_store;
  mem_size_e [1:0]     dec_mem_size;
  logic [1:0]          dec_mem_unsigned;

  logic [1:0]          commit_fire;
  commit_order_t commit_order;
  word_t [1:0]         commit_pc, commit_inst, commit_wdata;
  arch_reg_t [1:0]     commit_rd;
  logic [1:0]          commit_rd_wen;

  logic          redirect_valid;
  word_t         redirect_target;

  logic        dmem_valid, dmem_we;
  logic [3:0]  dmem_be;
  word_t       dmem_addr, dmem_wdata, dmem_rdata;

  word_t       imem_addr;
  word_t [1:0]       imem_rdata;
  logic imem_req_valid, imem_req_ready;
  word_t imem_req_addr;
  logic imem_resp_valid, imem_resp_ready;
  word_t [1:0] imem_resp_data;
  logic [31:0] imem [0:MEM_WORDS-1];
  assign imem_rdata = !$isunknown(imem_addr[MEM_MSB:3])
      ? {imem[{imem_addr[MEM_MSB:3], 1'b1}], imem[{imem_addr[MEM_MSB:3], 1'b0}]}
      : {32'h0000_0013, 32'h0000_0013};

  logic        tohost_we;
  word_t       tohost_val;
  word_t       tohost_addr, tohost_full_addr;

  // prediction seam + training loop (LIVE: the real machine)
  logic [1:0] dec_pred_taken;
  word_t      dec_pred_target;
  logic       bp_update_valid, bp_update_taken;
  word_t      bp_update_pc, bp_update_target;

  rv32i_ss_imem_scratchpad #(.LATENCY(`UMBRA_M3_IMEM_LATENCY),
                             .PIPELINED(`UMBRA_M3_IMEM_PIPELINED)) u_imem_adapter (
    .clk(clk), .rst_n(rst_n),
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr), .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready), .imem_resp_data(imem_resp_data),
    .line_addr(imem_addr), .line_data(imem_rdata)
  );

  rv32i_ss_frontend #(.RESET_PC(32'h0)) u_fe (
    .clk(clk), .rst_n(rst_n),
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr), .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready), .imem_resp_data(imem_resp_data),
    .redirect_valid(redirect_valid), .redirect_target(redirect_target),
    .bp_update_valid(bp_update_valid), .bp_update_pc(bp_update_pc),
    .bp_update_taken(bp_update_taken), .bp_update_target(bp_update_target),
    .decoded_valid(dec_valid), .decoded_slot_valid(dec_slot_valid), .decoded_ready(dec_ready),
    .decoded_pc(dec_pc), .decoded_instr(dec_instr),
    .decoded_rs1(dec_rs1), .decoded_rs2(dec_rs2), .decoded_rd(dec_rd),
    .decoded_rd_we(dec_rd_we), .decoded_needs_checkpoint(dec_needs_checkpoint),
    .decoded_op_class(dec_op_class), .decoded_fu_class(dec_fu_class),
    .decoded_muldiv_op(dec_muldiv_op), .decoded_alu_op(dec_alu_op),
    .decoded_branch_op(dec_branch_op),
    .decoded_src1_sel(dec_src1_sel), .decoded_src2_sel(dec_src2_sel),
    .decoded_imm(dec_imm),
    .decoded_trap(dec_trap),
    .decoded_csr_op(dec_csr_op),
    .decoded_csr_addr(dec_csr_addr),
    .decoded_csr_zimm(dec_csr_zimm),
    .decoded_is_load(dec_is_load),
    .decoded_is_store(dec_is_store),
    .decoded_mem_size(dec_mem_size),
    .decoded_mem_unsigned(dec_mem_unsigned),
    .decoded_pred_taken(dec_pred_taken),
    .decoded_pred_target(dec_pred_target)
  );

  rv32i_ss_core u_core (
    .clk(clk), .rst_n(rst_n),
    .decoded_valid(dec_valid), .decoded_slot_valid(dec_slot_valid), .decoded_ready(dec_ready),
    .decoded_pc(dec_pc), .decoded_instr(dec_instr),
    .decoded_rs1(dec_rs1), .decoded_rs2(dec_rs2), .decoded_rd(dec_rd),
    .decoded_rd_we(dec_rd_we), .decoded_needs_checkpoint(dec_needs_checkpoint),
    .decoded_op_class(dec_op_class), .decoded_fu_class(dec_fu_class),
    .decoded_muldiv_op(dec_muldiv_op), .decoded_alu_op(dec_alu_op),
    .decoded_branch_op(dec_branch_op),
    .decoded_src1_sel(dec_src1_sel), .decoded_src2_sel(dec_src2_sel),
    .decoded_imm(dec_imm),
    .decoded_trap(dec_trap),
    .decoded_csr_op(dec_csr_op),
    .decoded_csr_addr(dec_csr_addr),
    .decoded_csr_zimm(dec_csr_zimm),
    .decoded_is_load(dec_is_load),
    .decoded_is_store(dec_is_store),
    .decoded_mem_size(dec_mem_size),
    .decoded_mem_unsigned(dec_mem_unsigned),
    .decoded_pred_taken(dec_pred_taken),
    .decoded_pred_target(dec_pred_target),
    .redirect_valid(redirect_valid), .redirect_target(redirect_target),
    .bp_update_valid(bp_update_valid), .bp_update_pc(bp_update_pc),
    .bp_update_taken(bp_update_taken), .bp_update_target(bp_update_target),
    .commit_fire(commit_fire), .commit_order(commit_order),
    .commit_pc(commit_pc), .commit_inst(commit_inst),
    .commit_rd(commit_rd), .commit_rd_wen(commit_rd_wen),
    .commit_wdata(commit_wdata),
    .dmem_valid(dmem_valid), .dmem_we(dmem_we), .dmem_be(dmem_be),
    .dmem_addr(dmem_addr), .dmem_wdata(dmem_wdata),
    .dmem_ready(1'b1), .dmem_rvalid(1'b1),
    .dmem_rdata(dmem_rdata)
  );

  ooo_dmem_model #(.MEM_WORDS(MEM_WORDS), .MEM_MSB(MEM_MSB)) u_dmem (
    .clk(clk), .rst_n(rst_n),
    .addr(dmem_addr), .rdata(dmem_rdata),
    .we(dmem_we), .be(dmem_be), .wdata(dmem_wdata),
    .tohost_addr(tohost_addr), .tohost_full_addr(tohost_full_addr),
    .tohost_we(tohost_we), .tohost_val(tohost_val)
  );

  initial clk = 1'b0;
  always #5 clk = ~clk;

  string test_name;
  string imem_path;

  // strict tohost oracle (judgement here; the watch lives in the model)
  always @(posedge clk) begin
    if (rst_n && tohost_we) begin
      if (tohost_val == 32'h0000_0001) begin
        $display("RISCV_TEST PASS test=%s tohost=%08h", test_name, tohost_val);
        $finish;
      end else if (tohost_val != 32'h0000_0000) begin
        $display("RISCV_TEST FAIL test=%s tohost=%08h", test_name, tohost_val);
        $fatal(1, "riscv-test failed");
      end
    end
  end

  // watchdog: generous — OoO with serialized jumps is slower than the pipeline
  localparam int WATCHDOG_CYCLES = 400000;
  int wdog;
  always @(posedge clk) begin
    if (rst_n) begin
      wdog <= wdog + 1;
      if (wdog == WATCHDOG_CYCLES)
        $fatal(1, "WATCHDOG TIMEOUT test=%s after %0d cycles (last commit pc=%08h order=%0d)",
               test_name, WATCHDOG_CYCLES, commit_pc[0], commit_order);
    end
  end

  integer i;
  initial begin
    wdog = 0;
    test_name = "unknown";
    for (i = 0; i < MEM_WORDS; i++) imem[i] = 32'h00000013;

    if (!$value$plusargs("IMEM=%s", imem_path))
      $fatal(1, "missing +IMEM=<path>");
    if (!$value$plusargs("TOHOST=%h", tohost_addr))
      $fatal(1, "missing +TOHOST=<hexaddr>");
    if (!$value$plusargs("TOHOST_FULL=%h", tohost_full_addr))
      tohost_full_addr = tohost_addr;
    void'($value$plusargs("TESTNAME=%s", test_name));

    $readmemh(imem_path, imem);
    #1;  // model zero-fill first, then mirror the image (text + data) into dmem
    for (i = 0; i < MEM_WORDS; i++) u_dmem.mem[i] = imem[i];

    rst_n = 1'b0;
    repeat (4) @(posedge clk);
    rst_n = 1'b1;

    // Judged only by tohost (ecall/ebreak are instructions under test).
    forever @(posedge clk);
  end

endmodule
