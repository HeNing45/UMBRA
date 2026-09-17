// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// Store address/data decoupling — end-to-end consequential proof.
//
// The DIV is older than the store and starts while the store's base is ready.
// The store must issue AGEN before DIV produces x12, leaving an addr-valid /
// data-invalid SQ row.  A younger same-address load must wait on that exact
// full-cover row, then receive 12 only after the accepted DIV CDB beat fills
// the store data.  The store completes through the deferred SQ seam and writes
// memory only when it later commits.
// A second run holds AGEN execution until the input slot has snooped the
// accepted DIV result, then releases it after the broadcast is gone. The
// same committed store/load/consumer values prove pre-AGEN data retention.
//
//   0x00 addi x1,  x0, 0x100
//   0x04 addi x2,  x0, 84
//   0x08 addi x3,  x0, 7
//   0x0c div  x12, x2, x3       # x12 = 12, deliberately late
//   0x10 sw   x12, 0(x1)        # address can issue before x12
//   0x14 lw   x13, 0(x1)        # must wait, then forward 12
//   0x18 addi x14, x13, 1       # consequential consumer = 13
//   0x1c addi x15, x0, 0x55     # completion marker
//   0x20 jal  x0, 0

module tb_rv32i_ss_store_data_decouple;
  import rv32i_ss_pkg::*;

  localparam time CLK_PERIOD = 10ns;

  logic clk;
  logic rst_n;

  word_t       imem_addr;
  word_t [1:0] imem_rdata;
  logic        imem_req_valid;
  logic        imem_req_ready;
  word_t       imem_req_addr;
  logic        imem_resp_valid;
  logic        imem_resp_ready;
  word_t [1:0] imem_resp_data;
  logic [31:0] imem [0:255];

  logic [1:0]      commit_fire;
  commit_order_t   commit_order;
  word_t [1:0]     commit_pc;
  word_t [1:0]     commit_inst;
  arch_reg_t [1:0] commit_rd;
  logic [1:0]      commit_rd_wen;
  word_t [1:0]     commit_wdata;

  logic       dmem_valid;
  logic       dmem_we;
  logic [3:0] dmem_be;
  word_t      dmem_addr;
  word_t      dmem_wdata;
  word_t      dmem_rdata;

  int checks;
  int errors;
  int cycles;
  int slot;
  int i;

  logic entered_early_store_agen;
  logic entered_pending_full_cover;
  logic entered_late_cdb_capture;
  logic entered_post_capture_forward;
  logic entered_deferred_completion;
  logic marker_committed;
  logic store_committed;
  int   store_writes;
  word_t div_value;
  word_t load_value;
  word_t consumer_value;
  int scenario;
  logic entered_input_pending;
  logic entered_input_capture;

  assign imem_rdata = !$isunknown(imem_addr[9:3])
      ? {imem[{imem_addr[9:3], 1'b1}],
         imem[{imem_addr[9:3], 1'b0}]}
      : {32'h0000_0013, 32'h0000_0013};

  rv32i_ss_imem_zero_latency_adapter u_imem_adapter (
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr), .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready), .imem_resp_data(imem_resp_data),
    .line_addr(imem_addr), .line_data(imem_rdata)
  );

  umbra_ss_cpu_top #(.RESET_PC(32'h0000_0000)) u_cpu (
    .clk          (clk),
    .rst_n        (rst_n),
    .imem_req_valid  (imem_req_valid),
    .imem_req_ready  (imem_req_ready),
    .imem_req_addr   (imem_req_addr),
    .imem_resp_valid (imem_resp_valid),
    .imem_resp_ready (imem_resp_ready),
    .imem_resp_data  (imem_resp_data),
    .commit_fire  (commit_fire),
    .commit_order (commit_order),
    .commit_pc    (commit_pc),
    .commit_inst  (commit_inst),
    .commit_rd    (commit_rd),
    .commit_rd_wen(commit_rd_wen),
    .commit_wdata (commit_wdata),
    .dmem_valid   (dmem_valid),
    .dmem_we      (dmem_we),
    .dmem_be      (dmem_be),
    .dmem_addr    (dmem_addr),
    .dmem_wdata   (dmem_wdata),
    .dmem_ready   (1'b1),
    .dmem_rvalid  (1'b1),
    .dmem_rdata   (dmem_rdata)
  );

  ooo_dmem_model #(.MEM_WORDS(256), .MEM_MSB(9)) u_dmem (
    .clk             (clk),
    .rst_n           (rst_n),
    .addr            (dmem_addr),
    .rdata           (dmem_rdata),
    .we              (dmem_we),
    .be              (dmem_be),
    .wdata           (dmem_wdata),
    .tohost_addr     (32'hffff_fffc),
    .tohost_full_addr(32'hffff_fffc),
    .tohost_we       (),
    .tohost_val      ()
  );

  initial clk = 1'b0;
  always #(CLK_PERIOD / 2) clk = ~clk;

  task automatic check(input string name, input logic condition);
    checks++;
    if (!condition) begin
      errors++;
      $display("  FAIL [%0d] %s", checks, name);
    end
  endtask

  // Entry proof and architectural consequence are observed independently.
  always @(posedge clk) begin
    if (rst_n) begin
      cycles <= cycles + 1;
      for (int bank = 0; bank < 2; bank++) begin
        if (u_cpu.u_core.agen_in_q[bank].valid &&
            u_cpu.u_core.agen_in_q[bank].uop.pc == 32'h10) begin
          if (u_cpu.u_core.agen_in_q[bank].store_data_pending)
            entered_input_pending <= 1'b1;
          if (entered_input_pending && !u_cpu.u_core.agen_exec_fire &&
              u_cpu.u_core.agen_in_q[bank].store_data_valid &&
              !u_cpu.u_core.agen_in_q[bank].store_data_pending &&
              u_cpu.u_core.agen_in_q[bank].store_data == 32'd12)
            entered_input_capture <= 1'b1;
        end
      end

      if (u_cpu.u_core.store_agen_fire &&
          (u_cpu.u_core.agen_issue_entry.pc == 32'h0000_0010) &&
          !u_cpu.u_core.store_data_ready_at_issue)
        entered_early_store_agen <= 1'b1;

      if (u_cpu.u_core.u_lsq.lq_select_valid &&
          u_cpu.u_core.u_lsq.sq_overlap_winner_valid &&
          !u_cpu.u_core.u_lsq.sq_overlap_winner_data_valid &&
          ((u_cpu.u_core.u_lsq.sq_overlap_winner_be &
            u_cpu.u_core.u_lsq.load_be) == u_cpu.u_core.u_lsq.load_be)) begin
        entered_pending_full_cover <= 1'b1;
        if (u_cpu.u_core.u_lsq.lq_mem_req_fire ||
            u_cpu.u_core.u_lsq.lq_forward_fire)
          $fatal(1, "pending-data full-cover store allowed younger load progress");
      end

      if ((|u_cpu.u_core.u_lsq.sq_data_wb_match[0]) ||
          (|u_cpu.u_core.u_lsq.sq_data_wb_match[1]))
        entered_late_cdb_capture <= 1'b1;

      if (u_cpu.u_core.u_lsq.lq_forward_fire &&
          u_cpu.u_core.u_lsq.sq_overlap_winner_data_valid)
        entered_post_capture_forward <= 1'b1;

      if (u_cpu.u_core.sq_complete_accept)
        entered_deferred_completion <= 1'b1;

      if (dmem_we) begin
        store_writes <= store_writes + 1;
        if (!(|commit_fire))
          $fatal(1, "store write occurred outside commit");
      end

      for (slot = 0; slot < 2; slot = slot + 1) begin
        if (commit_fire[slot]) begin
          if (commit_pc[slot] == 32'h0000_000c)
            div_value <= commit_wdata[slot];
          if (commit_pc[slot] == 32'h0000_0010)
            store_committed <= 1'b1;
          if (commit_pc[slot] == 32'h0000_0014)
            load_value <= commit_wdata[slot];
          if (commit_pc[slot] == 32'h0000_0018)
            consumer_value <= commit_wdata[slot];
          if (commit_pc[slot] == 32'h0000_001c)
            marker_committed <= 1'b1;
        end
      end
    end
  end

  initial begin
    repeat (2000) @(posedge clk);
    $fatal(1, "WATCHDOG: store-data decoupling test exceeded 2000 cycles");
  end

  initial begin
    checks = 0;
    for (scenario = 0; scenario < 2; scenario++) begin
    @(negedge clk);
    rst_n = 1'b0;
    errors = 0;
    cycles = 0;
    entered_early_store_agen = 1'b0;
    entered_pending_full_cover = 1'b0;
    entered_late_cdb_capture = 1'b0;
    entered_post_capture_forward = 1'b0;
    entered_deferred_completion = 1'b0;
    marker_committed = 1'b0;
    store_committed = 1'b0;
    store_writes = 0;
    div_value = '0;
    load_value = '0;
    consumer_value = '0;
    entered_input_pending = 1'b0;
    entered_input_capture = 1'b0;
    if (scenario == 1)
      force u_cpu.u_core.agen_exec_fire = 1'b0;

    for (i = 0; i < 256; i = i + 1)
      imem[i] = 32'h0000_0013;

    imem[0] = 32'h1000_0093;  // addi x1,  x0, 0x100
    imem[1] = 32'h0540_0113;  // addi x2,  x0, 84
    imem[2] = 32'h0070_0193;  // addi x3,  x0, 7
    imem[3] = 32'h0231_4633;  // div  x12, x2, x3
    imem[4] = 32'h00c0_a023;  // sw   x12, 0(x1)
    imem[5] = 32'h0000_a683;  // lw   x13, 0(x1)
    imem[6] = 32'h0016_8713;  // addi x14, x13, 1
    imem[7] = 32'h0550_0793;  // addi x15, x0, 0x55
    imem[8] = 32'h0000_006f;  // jal  x0, 0

    rst_n = 1'b0;
    repeat (4) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;

    if (scenario == 1) begin
      while (!entered_input_capture && cycles < 300) @(negedge clk);
      check("pending store entered registered input", entered_input_pending);
      check("waiting input captured producer data", entered_input_capture);
      repeat (3) @(negedge clk);
      check("producer broadcast has drained before AGEN release",
            !(|u_cpu.u_core.sq_data_wb_fire));
      release u_cpu.u_core.agen_exec_fire;
    end

    while (!marker_committed && (cycles < 1500))
      @(posedge clk);
    repeat (4) @(posedge clk);

    @(negedge clk);
    check("marker committed", marker_committed);
    if (scenario == 0) begin
      check("store AGEN issued before data became ready",
            entered_early_store_agen);
      check("younger load observed a pending full-cover winner",
            entered_pending_full_cover);
      check("accepted producer CDB beat captured late store data",
            entered_late_cdb_capture);
      check("load forwarded only after store data capture",
            entered_post_capture_forward);
      check("store completed through deferred SQ seam",
            entered_deferred_completion);
    end
    check("DIV architectural result is 12", div_value == 32'd12);
    check("younger load architectural result is 12", load_value == 32'd12);
    check("dependent consumer architectural result is 13",
          consumer_value == 32'd13);
    check("store committed", store_committed);
    check("exactly one committed memory write", store_writes == 1);
    check("memory contains the late producer value",
          u_dmem.mem[32'h100 >> 2] == 32'd12);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_store_data_decouple] scenario=%0d checks=%0d cycles=%0d",
               scenario, checks, cycles);
    end else begin
      $fatal(1,
             "[tb_rv32i_ss_store_data_decouple] FAIL errors=%0d checks=%0d",
             errors, checks);
    end
    end
    $display("[tb_rv32i_ss_store_data_decouple] PASS checks=%0d scenarios=2", checks);
    $finish;
  end

endmodule
