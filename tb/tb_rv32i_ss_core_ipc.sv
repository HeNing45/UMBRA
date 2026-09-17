// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// tb_rv32i_ss_core_ipc — measured IPC gate.
//
// Runs a straight-line steady-state program and measures committed IPC over a
// FIXED WINDOW, excluding warm-up. Warm-up matters: the first commits are
// paid for by an empty pipeline (fetch, rename, issue, first writebacks), so
// counting from reset understates steady-state commit bandwidth.
//
// The window opens once IPC_WARMUP instructions have retired and closes after
// a further IPC_WINDOW retirements. Cycles are counted only while the window
// is open, so the ratio is a true steady-state figure.
//
// IPC is committed instructions / cycles, so it counts BOTH commit slots.
//
//   +IMEM=<path>       program image (also mirrored into dmem)
//   +IPC_WARMUP=N      retirements to discard  (default 64)
//   +IPC_WINDOW=N      retirements to measure  (default 512)
//
// Emits one machine-readable line for the runner to threshold:
//   IPC RESULT commits=<N> cycles=<M> ipc=<x.xxx>

module tb_rv32i_ss_core_ipc;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;

  logic clk;
  logic rst_n;

  logic          dec_valid, dec_ready;
  logic [1:0]    dec_slot_valid;
  word_t [1:0]   dec_pc, dec_instr;
  arch_reg_t [1:0] dec_rs1, dec_rs2, dec_rd;
  logic [1:0]    dec_rd_we, dec_needs_checkpoint;
  ooo_op_class_e [1:0] dec_op_class;
  ooo_fu_class_e [1:0] dec_fu_class;
  muldiv_op_e [1:0]    dec_muldiv_op;
  alu_op_e [1:0]       dec_alu_op;
  br_type_e [1:0]      dec_branch_op;
  ooo_src_sel_e [1:0]  dec_src1_sel, dec_src2_sel;
  word_t [1:0]         dec_imm;
  decoded_trap_t dec_trap;
  csr_op_e       dec_csr_op;
  csr_addr_t     dec_csr_addr;
  csr_zimm_t     dec_csr_zimm;
  logic [1:0]    dec_is_load, dec_is_store;
  mem_size_e [1:0] dec_mem_size;
  logic [1:0]    dec_mem_unsigned;

  logic [1:0]      commit_fire;
  commit_order_t   commit_order;
  word_t [1:0]     commit_pc, commit_inst, commit_wdata;
  arch_reg_t [1:0] commit_rd;
  logic [1:0]      commit_rd_wen;

  logic  redirect_valid;
  word_t redirect_target;

  logic        dmem_valid, dmem_we;
  logic [3:0]  dmem_be;
  word_t       dmem_addr, dmem_wdata, dmem_rdata;

  word_t       imem_addr;
  word_t [1:0] imem_rdata;
  logic imem_req_valid, imem_req_ready;
  word_t imem_req_addr;
  logic imem_resp_valid, imem_resp_ready;
  word_t [1:0] imem_resp_data;
  logic [31:0] imem [1024];
  assign imem_rdata = !$isunknown(imem_addr[11:3])
      ? {imem[{imem_addr[11:3], 1'b1}], imem[{imem_addr[11:3], 1'b0}]}
      : {32'h0000_0013, 32'h0000_0013};

  ooo_dmem_model #(.MEM_WORDS(65536), .MEM_MSB(17)) u_dmem (
    .clk(clk), .rst_n(rst_n),
    .addr(dmem_addr), .rdata(dmem_rdata),
    .we(dmem_we), .be(dmem_be), .wdata(dmem_wdata),
    .tohost_addr(32'hFFFF_FFFC), .tohost_full_addr(32'hFFFF_FFFC),
    .tohost_we(), .tohost_val()
  );

  // prediction seam + training loop (LIVE: the real machine)
  logic [1:0] dec_pred_taken;
  word_t      dec_pred_target;
  logic       bp_update_valid, bp_update_taken;
  word_t      bp_update_pc, bp_update_target;

  rv32i_ss_imem_zero_latency_adapter u_imem_adapter (
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
    .decoded_is_load(dec_is_load), .decoded_is_store(dec_is_store),
    .decoded_mem_size(dec_mem_size), .decoded_mem_unsigned(dec_mem_unsigned),
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
    .decoded_is_load(dec_is_load), .decoded_is_store(dec_is_store),
    .decoded_mem_size(dec_mem_size), .decoded_mem_unsigned(dec_mem_unsigned),
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

  initial clk = 1'b0;
  always #5 clk = ~clk;

  string imem_path;
  int    warmup, window;
  int    retired;        // total retirements since reset
  int    win_commits;    // retirements inside the window
  int    win_cycles;     // cycles the window has been open
  bit    win_open, win_done;
  int    this_cycle;
  int    counted_this_cycle;
  int    dual_this_cycle;
  int    dual_cycles;    // cycles that retired TWO -- the existence proof
  int    wdog;

  localparam int WATCHDOG_CYCLES = 200_000;

  always @(posedge clk) begin
    if (rst_n && !win_done) begin
      wdog <= wdog + 1;
      if (wdog == WATCHDOG_CYCLES)
        $fatal(1, "IPC_WATCHDOG: window never closed (retired=%0d)", retired);

      // count this edge's retirements: BOTH slots
      this_cycle = {31'b0, commit_fire[0]} + {31'b0, commit_fire[1]};
      dual_this_cycle = (commit_fire == 2'b11) ? 1 : 0;

      if (win_open) begin
        // Cap only the measurement numerator on the closing edge. Hardware
        // may retire two when one record remains in an odd-sized window; the
        // fixed-window result must still report exactly IPC_WINDOW records.
        counted_this_cycle = this_cycle;
        if (counted_this_cycle > (window - win_commits))
          counted_this_cycle = window - win_commits;
        win_cycles <= win_cycles + 1;
        dual_cycles <= dual_cycles + dual_this_cycle;
        win_commits <= win_commits + counted_this_cycle;
        if ((win_commits + counted_this_cycle) >= window) begin
          win_done <= 1'b1;
          $display("IPC RESULT commits=%0d cycles=%0d dual_cycles=%0d",
                   win_commits + counted_this_cycle, win_cycles + 1,
                   dual_cycles + dual_this_cycle);
          $finish;
        end
      end else if ((retired + this_cycle) >= warmup) begin
        // warm-up satisfied; the window opens on the NEXT edge so no partially
        // counted cycle is attributed to the measurement
        win_open <= 1'b1;
      end

      retired <= retired + this_cycle;
    end
  end

  integer i;
  initial begin
    wdog = 0; retired = 0; win_commits = 0; win_cycles = 0;
    win_open = 1'b0; win_done = 1'b0; dual_cycles = 0;
    if (!$value$plusargs("IPC_WARMUP=%d", warmup)) warmup = 64;
    if (!$value$plusargs("IPC_WINDOW=%d", window)) window = 512;
    if ((warmup < 0) || (window <= 0))
      $fatal(1, "IPC arguments must satisfy warmup>=0 and window>0");
    for (i = 0; i < 1024; i++) imem[i] = 32'h00000013;
    if (!$value$plusargs("IMEM=%s", imem_path))
      $fatal(1, "missing +IMEM=<path>");
    $readmemh(imem_path, imem);
    for (i = 0; i < 1024; i++) u_dmem.mem[i] = imem[i];
    rst_n = 1'b0;
    repeat (4) @(posedge clk);
    rst_n = 1'b1;
  end

endmodule
