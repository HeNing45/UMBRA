// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// backend-priority and killed predicted-stream request proof.
//
// A forced, exact BTB entry predicts the upper-word branch at 0x0c taken to
// 0x30. The target request is accepted and held while the branch later
// resolves not-taken. Recovery to 0x10 must win, mark the accepted predicted-
// stream request killed, accept/discard its poison response, and only then
// request the fall-through. Admitting the killed line commits x10/x11 before
// the real response is released, so the control error is consequential.
module tb_rv32i_ss_fetch_steer_recovery;
  import rv32i_ss_pkg::*;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  logic  imem_req_valid;
  logic  imem_req_ready;
  word_t imem_req_addr;
  logic  imem_resp_valid;
  logic  imem_resp_ready;
  word_t [1:0] imem_resp_data;

  logic [1:0] commit_fire;
  commit_order_t commit_order;
  word_t [1:0] commit_pc, commit_inst, commit_wdata;
  arch_reg_t [1:0] commit_rd;
  logic [1:0] commit_rd_wen;

  umbra_ss_cpu_top u_cpu (
    .clk(clk), .rst_n(rst_n),
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr), .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready), .imem_resp_data(imem_resp_data),
    .commit_fire(commit_fire), .commit_order(commit_order),
    .commit_pc(commit_pc), .commit_inst(commit_inst),
    .commit_rd(commit_rd), .commit_rd_wen(commit_rd_wen),
    .commit_wdata(commit_wdata),
    .dmem_ready(1'b1), .dmem_rvalid(1'b0), .dmem_rdata('0)
  );

  `define FE   u_cpu.u_fe
  `define CORE u_cpu.u_core
  `define BP   u_cpu.u_fe.u_bp

  localparam word_t ADDI_X2_2    = 32'h0020_0113;
  localparam word_t ADDI_X3_3    = 32'h0030_0193;
  localparam word_t ADDI_X1_0    = 32'h0000_0093;
  localparam word_t BNE_X1_30    = 32'h0200_9263; // 0x0c -> 0x30, actually NT
  localparam word_t POISON_X10   = 32'h0630_0513;
  localparam word_t POISON_X11   = 32'h0580_0593;
  localparam word_t FALL_X12     = 32'h00c0_0613;
  localparam word_t MARKER_X14   = 32'h00e0_0713;

  int checks = 0;
  int errors = 0;
  int waited;
  bit pred_slot1_dispatch_seen;
  bit recovery_with_target_request_seen;
  bit killed_identity_seen;
  bit killed_response_seen;
  bit poison_commit_seen;
  bit marker_seen;

  task automatic check(input string name, input logic condition);
    checks++;
    if (!condition) begin
      $error("[%s]", name);
      errors++;
    end
  endtask

  // pipelined legality: at most one accepted transaction, none accepted while
  // a response is pending  — the frontend now presents
  // its next offer during flight and relies on this environment rule.
  logic env_outstanding;
  assign imem_req_ready = !env_outstanding;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) env_outstanding <= 1'b0;
    else if (imem_req_valid && imem_req_ready &&
             !(imem_resp_valid && imem_resp_ready)) env_outstanding <= 1'b1;
    else if (imem_resp_valid && imem_resp_ready) env_outstanding <= 1'b0;
  end

  task automatic wait_request(input word_t expected_addr);
    waited = 0;
    while (!(imem_req_valid && (imem_req_addr == expected_addr))) begin
      @(negedge clk);
      waited++;
      if (waited > 100)
        $fatal(1, "request %08h did not appear: valid=%0b addr=%08h",
               expected_addr, imem_req_valid, imem_req_addr);
    end
  endtask

  task automatic respond_same_cycle(
    input word_t expected_addr,
    input word_t lower,
    input word_t upper
  );
    wait_request(expected_addr);
    imem_resp_data = {upper, lower};
    imem_resp_valid = 1'b1;
    #1;
    check("same-cycle response has reserved receiver", imem_resp_ready);
    @(posedge clk);
    @(negedge clk);
    imem_resp_valid = 1'b0;
    imem_resp_data = '0;
  endtask

  task automatic respond_delayed(input word_t lower, input word_t upper);
    @(negedge clk);
    imem_resp_data = {upper, lower};
    imem_resp_valid = 1'b1;
    check("delayed response has accepted identity", imem_resp_ready);
    @(posedge clk);
    @(negedge clk);
    imem_resp_valid = 1'b0;
    imem_resp_data = '0;
  endtask

  always @(posedge clk) begin
    if (rst_n === 1'b1) begin
      if (`CORE.bundle_fire && (`CORE.decoded_slot_valid == 2'b11) &&
          `CORE.decoded_pred_taken[1])
        pred_slot1_dispatch_seen <= 1'b1;
      for (int lane = 0; lane < 2; lane++) begin
        if (commit_fire[lane]) begin
          if (commit_rd_wen[lane] &&
              ((commit_rd[lane] == arch_reg_t'(10)) ||
               (commit_rd[lane] == arch_reg_t'(11))))
            poison_commit_seen <= 1'b1;
          if (commit_rd_wen[lane] &&
              (commit_rd[lane] == arch_reg_t'(14)) &&
              (commit_wdata[lane] == 32'd14))
            marker_seen <= 1'b1;
        end
      end
    end
  end

  initial begin
    repeat (1800) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_fetch_steer_recovery exceeded 1800 cycles");
  end

  initial begin
    $display("[tb_rv32i_ss_fetch_steer_recovery] starting");
    imem_resp_valid = 1'b0;
    imem_resp_data = '0;
    pred_slot1_dispatch_seen = 1'b0;
    recovery_with_target_request_seen = 1'b0;
    killed_identity_seen = 1'b0;
    killed_response_seen = 1'b0;
    poison_commit_seen = 1'b0;
    marker_seen = 1'b0;

    repeat (4) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;

    // Force the precise prediction state rather than relying on a warmup
    // loop. PC 0x0c has idx=3/tag=0 at the ratified BTB geometry.
    `BP.btb_valid_q[3] = 1'b1;
    `BP.btb_tag_q[3] = '0;
    `BP.btb_target_q[3] = 30'(32'h0000_0030 >> 2);
    `BP.btb_state_q[3] = 2'b11;
    `BP.btb_return_q[3] = 1'b0;

    respond_same_cycle(32'h0000_0000, ADDI_X2_2, ADDI_X3_3);
    respond_same_cycle(32'h0000_0008, ADDI_X1_0, BNE_X1_30);

    // Request-time upper-half steering must offer and accept 0x30 before the
    // not-taken verdict exists.
    wait_request(32'h0000_0030);
    @(posedge clk);
    @(negedge clk);
    check("predicted target request accepted",
          `FE.req_inflight_q && (`FE.req_inflight_addr_q == 32'h30));

    waited = 0;
    while (!`CORE.branch_recover_req) begin
      @(negedge clk);
      waited++;
      if (waited > 100) $fatal(1, "predicted-taken branch never recovered");
    end
    check("fall-through recovery sees accepted predicted target request",
          `FE.req_inflight_q && (`FE.req_inflight_addr_q == 32'h30) &&
          (`CORE.recover_q_target == 32'h10));
    recovery_with_target_request_seen = 1'b1;
    @(posedge clk);
    @(negedge clk);
    check("backend redirect retained a killed request identity",
          `FE.req_inflight_q && `FE.req_inflight_killed_q &&
          (`FE.req_inflight_addr_q == 32'h30));
    killed_identity_seen = 1'b1;

    respond_delayed(POISON_X10, POISON_X11);
    killed_response_seen = 1'b1;
    check("killed predicted-target response did not fill queue",
          (`FE.fq_count_q == 0) && !`FE.decoded_valid);

    // A killed prefetch offer created at the predicted target's
    // acceptance may be producer-held through the recovery; it fires as
    // soon as the poison response drains and must itself be drained before
    // the fall-through target can be accepted.
    if (imem_req_valid && (imem_req_addr != 32'h0000_0010)) begin
      respond_delayed(POISON_X10, POISON_X11);
      check("killed prefetch offer drained without refill",
            (`FE.fq_count_q == 0) && !`FE.decoded_valid);
    end

    wait_request(32'h0000_0010);
    @(posedge clk);
    @(negedge clk);
    check("fall-through request accepted after killed response drained",
          `FE.req_inflight_q && (`FE.req_inflight_addr_q == 32'h10));
    repeat (12) begin
      @(posedge clk);
      @(negedge clk);
    end
    check("poison target never committed while fall-through withheld",
          !poison_commit_seen);

    respond_delayed(FALL_X12, MARKER_X14);
    waited = 0;
    while (!marker_seen) begin
      @(posedge clk);
      waited++;
      if (waited > 120) $fatal(1, "fall-through marker never committed");
    end
    repeat (2) @(posedge clk);

    check("upper-half fetch steering entered", `FE.d015_req_branch_upper_count > 0);
    check("predicted upper branch reached dispatch", pred_slot1_dispatch_seen);
    check("recovery/request overlap entered", recovery_with_target_request_seen);
    check("killed identity entered", killed_identity_seen);
    check("killed response entered", killed_response_seen);
    check("poison never committed", !poison_commit_seen);
    check("fall-through marker committed", marker_seen);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_fetch_steer_recovery] PASS checks=%0d", checks);
      $finish;
    end
    $fatal(1,
           "[tb_rv32i_ss_fetch_steer_recovery] FAIL errors=%0d checks=%0d",
           errors, checks);
  end
endmodule
