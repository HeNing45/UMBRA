`timescale 1ns/1ps

// consequential recovery test.
//
// A fall-through line request is accepted before an older taken branch
// resolves. Recovery marks that accepted request killed. Its delayed response
// is still consumed at the memory boundary, but must never become a decoded or
// committed instruction. The target response is deliberately withheld long
// enough that weakening the killed-response gate lets the stale addi pair
// execute and commit, making the control error architecturally visible.
module tb_rv32i_ss_fetch_queue_recovery;
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

  // The frontend presents its next offer while a
  // request is outstanding and relies on the environment's
  // acceptance rule — at most one accepted transaction, none accepted while
  // a response is pending. Constant ready would allow a second acceptance
  // into the single in-flight record.
  logic env_outstanding;
  assign imem_req_ready = !env_outstanding;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) env_outstanding <= 1'b0;
    else if (imem_req_valid && imem_req_ready &&
             !(imem_resp_valid && imem_resp_ready)) env_outstanding <= 1'b1;
    else if (imem_resp_valid && imem_resp_ready) env_outstanding <= 1'b0;
  end

  logic [1:0] commit_fire;
  commit_order_t commit_order;
  word_t [1:0] commit_pc, commit_inst, commit_wdata;
  arch_reg_t [1:0] commit_rd;
  logic [1:0] commit_rd_wen;

  logic dmem_valid, dmem_we;
  logic [3:0] dmem_be;
  word_t dmem_addr, dmem_wdata;

  umbra_ss_cpu_top u_cpu (
    .clk             (clk),
    .rst_n           (rst_n),
    .imem_req_valid  (imem_req_valid),
    .imem_req_ready  (imem_req_ready),
    .imem_req_addr   (imem_req_addr),
    .imem_resp_valid (imem_resp_valid),
    .imem_resp_ready (imem_resp_ready),
    .imem_resp_data  (imem_resp_data),
    .commit_fire     (commit_fire),
    .commit_order    (commit_order),
    .commit_pc       (commit_pc),
    .commit_inst     (commit_inst),
    .commit_rd       (commit_rd),
    .commit_rd_wen   (commit_rd_wen),
    .commit_wdata    (commit_wdata),
    .dmem_valid      (dmem_valid),
    .dmem_we         (dmem_we),
    .dmem_be         (dmem_be),
    .dmem_addr       (dmem_addr),
    .dmem_wdata      (dmem_wdata),
    .dmem_ready      (1'b1),
    .dmem_rvalid     (1'b0),
    .dmem_rdata      ('0)
  );

  `define FE   u_cpu.u_fe
  `define CORE u_cpu.u_core

  localparam word_t ADDI_X2_2    = 32'h0020_0113;
  localparam word_t ADDI_X3_3    = 32'h0030_0193;
  localparam word_t ADDI_X1_1    = 32'h0010_0093;
  localparam word_t BNE_X1_30    = 32'h0200_9263; // bne x1,x0,+36 -> 0x30
  localparam word_t STALE_X10_99 = 32'h0630_0513;
  localparam word_t STALE_X11_88 = 32'h0580_0593;
  localparam word_t TARGET_X12   = 32'h00c0_0613;
  localparam word_t MARKER_X14   = 32'h00e0_0713;

  int checks = 0;
  int errors = 0;
  int waited;

  bit fallthrough_request_accepted;
  bit recovery_with_request_seen;
  bit killed_identity_seen;
  bit killed_response_accepted;
  bit stale_dispatch_seen;
  bit stale_commit_seen;
  bit target_commit_seen;
  bit marker_commit_seen;

  task automatic check(input string name, input logic condition);
    checks++;
    if (!condition) begin
      $error("[%s]", name);
      errors++;
    end
  endtask

  task automatic wait_request(input word_t expected_addr);
    waited = 0;
    while (!(imem_req_valid && (imem_req_addr == expected_addr))) begin
      @(negedge clk);
      waited++;
      if (waited > 80) begin
        $fatal(1, "request %08h did not appear: valid=%0b addr=%08h",
               expected_addr, imem_req_valid, imem_req_addr);
      end
    end
  endtask

  task automatic respond_same_cycle(
    input word_t expected_addr,
    input word_t lower,
    input word_t upper
  );
    wait_request(expected_addr);
    imem_resp_data  = {upper, lower};
    imem_resp_valid = 1'b1;
    #1;
    check("same-cycle response has a reserved receiver", imem_resp_ready);
    @(posedge clk);
    @(negedge clk);
    imem_resp_valid = 1'b0;
    imem_resp_data  = '0;
  endtask

  task automatic respond_delayed(input word_t lower, input word_t upper);
    @(negedge clk);
    imem_resp_data  = {upper, lower};
    imem_resp_valid = 1'b1;
    check("delayed response has accepted identity", imem_resp_ready);
    @(posedge clk);
    @(negedge clk);
    imem_resp_valid = 1'b0;
    imem_resp_data  = '0;
  endtask

  always @(posedge clk) begin
    if (rst_n === 1'b1) begin
      if (`FE.decoded_valid &&
          ((`FE.decoded_pc[0] == 32'h0000_0010) ||
           (`FE.decoded_pc[0] == 32'h0000_0014))) begin
        stale_dispatch_seen <= 1'b1;
      end

      for (int lane = 0; lane < 2; lane++) begin
        if (commit_fire[lane]) begin
          if ((commit_pc[lane] == 32'h0000_0010) ||
              (commit_pc[lane] == 32'h0000_0014)) begin
            stale_commit_seen <= 1'b1;
          end
          if (commit_rd_wen[lane] && (commit_rd[lane] == arch_reg_t'(12)) &&
              (commit_wdata[lane] == 32'd12)) begin
            target_commit_seen <= 1'b1;
          end
          if (commit_rd_wen[lane] && (commit_rd[lane] == arch_reg_t'(14)) &&
              (commit_wdata[lane] == 32'd14)) begin
            marker_commit_seen <= 1'b1;
          end
        end
      end
    end
  end

  initial begin
    repeat (1500) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_fetch_queue_recovery exceeded 1500 cycles");
  end

  initial begin
    $display("[tb_rv32i_ss_fetch_queue_recovery] starting");
    imem_resp_valid = 1'b0;
    imem_resp_data  = '0;
    fallthrough_request_accepted = 1'b0;
    recovery_with_request_seen = 1'b0;
    killed_identity_seen = 1'b0;
    killed_response_accepted = 1'b0;
    stale_dispatch_seen = 1'b0;
    stale_commit_seen = 1'b0;
    target_commit_seen = 1'b0;
    marker_commit_seen = 1'b0;

    repeat (4) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;

    // Seed a harmless first line, then return the branch line in queue slot 1.
    // Its sequential successor reserves slot 0, so a kill-gate mutation makes
    // the dead response become the post-flush head rather than merely leaving
    // an unreachable non-head entry.
    respond_same_cycle(32'h0000_0000, ADDI_X2_2, ADDI_X3_3);
    respond_same_cycle(32'h0000_0008, ADDI_X1_1, BNE_X1_30);

    // The sequential line is accepted but intentionally left outstanding.
    wait_request(32'h0000_0010);
    @(posedge clk);
    @(negedge clk);
    check("fall-through request accepted", `FE.req_inflight_q &&
          (`FE.req_inflight_addr_q == 32'h0000_0010));
    fallthrough_request_accepted = 1'b1;

    // Wait for the real execute-time branch recovery. Do not return the old
    // line on that edge: the following state must carry a killed identity.
    waited = 0;
    while (!`CORE.branch_recover_req) begin
      @(negedge clk);
      waited++;
      if (waited > 80) $fatal(1, "taken branch never recovered");
    end
    check("recovery occurred with accepted fall-through request",
          `FE.req_inflight_q && (`FE.req_inflight_addr_q == 32'h10));
    recovery_with_request_seen = 1'b1;
    @(posedge clk);
    @(negedge clk);
    check("recovery retained killed response identity",
          `FE.req_inflight_q && `FE.req_inflight_killed_q &&
          (`FE.req_inflight_addr_q == 32'h10));
    killed_identity_seen = 1'b1;

    // Return the dead line after recovery. It contains visible register
    // writes, so any accidental refill becomes an architectural failure.
    respond_delayed(STALE_X10_99, STALE_X11_88);
    killed_response_accepted = 1'b1;
    check("dead response left queue empty", (`FE.fq_count_q == 0) &&
          !`FE.decoded_valid);

    // A killed prefetch offer created at 0x10's acceptance may be
    // producer-held through the recovery; it fires as soon as the dead
    // response drains and must itself be drained before the target request
    // can be accepted.
    if (imem_req_valid && (imem_req_addr != 32'h0000_0030)) begin
      respond_delayed(STALE_X10_99, STALE_X11_88);
      check("killed prefetch offer drained without refill",
            (`FE.fq_count_q == 0) && !`FE.decoded_valid);
    end

    // The target request is now accepted, but its response is held back. A
    // mutation that admits the dead line gets twelve unconstrained cycles to
    // dispatch and commit it before the real target appears.
    wait_request(32'h0000_0030);
    @(posedge clk);
    @(negedge clk);
    check("target request accepted after dead response drained",
          `FE.req_inflight_q && (`FE.req_inflight_addr_q == 32'h30));
    repeat (12) begin
      @(posedge clk);
      @(negedge clk);
    end
    check("dead response never reached decode", !stale_dispatch_seen);
    check("dead response never reached commit", !stale_commit_seen);

    respond_delayed(TARGET_X12, MARKER_X14);
    waited = 0;
    while (!marker_commit_seen) begin
      @(posedge clk);
      waited++;
      if (waited > 120) $fatal(1, "target marker never committed");
    end
    repeat (2) @(posedge clk);

    check("fall-through request state entered", fallthrough_request_accepted);
    check("recovery-with-request state entered", recovery_with_request_seen);
    check("killed identity state entered", killed_identity_seen);
    check("killed response acceptance entered", killed_response_accepted);
    check("target instruction committed", target_commit_seen);
    check("marker instruction committed", marker_commit_seen);
    check("no stale PC committed", !stale_commit_seen);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_fetch_queue_recovery] PASS checks=%0d", checks);
      $finish;
    end
    $fatal(1,
           "[tb_rv32i_ss_fetch_queue_recovery] FAIL errors=%0d checks=%0d stale_dispatch=%0b stale_commit=%0b",
           errors, checks, stale_dispatch_seen, stale_commit_seen);
  end
endmodule
