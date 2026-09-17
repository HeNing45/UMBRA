// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// tb_rv32i_ss_issue_wakeup_recovery -- forced legal window for the
// issue-driven ALU holder-live recovery gate.
//
// The program creates the recovery, tag reuse, new producer, and dependent
// consumer naturally. One test-only force suppresses ALU1's CDB selection for
// exactly the first post-recovery cycle. That models the otherwise-unseen
// third-ranked-holder window without fabricating holder contents, liveness,
// physical tags, free-list state, or architectural data:
//
//   1. load A releases an older taken branch and its younger ALU victim
//      together. Age order binds the branch to ALU0 and the victim to ALU1.
//   2. load B returns on that issue edge. On the registered recovery cycle,
//      load B and the branch consume both CDB lanes, so the killed ALU1 packet
//      remains physically resident while recovery clears only its live bit.
//   3. the test suppresses the killed holder's CDB eligibility until the
//      contested correct-path consumer issues. This is a test-only way to
//      force a legal run of cycles in which other completion clients keep the
//      killed packet resident; no packet contents, tags, liveness, free-list
//      state, architectural data, or bypass result is fabricated.
//   4. the first target word consumes the older free register. Recovery's
//      reclaimed register is the second allocation of that target bundle, so
//      the correct-path x9 producer in slot 1 reclaims the victim's physical
//      register and issues into ALU0 while killed ALU1 remains resident.
//      The dependent x11 consumer then sees both holders naming one pdst.
//
// Baseline consequence: only the live ALU0 holder may bypass, so x11 = 6.
// Mutation consequence: omit the ALU1 live clear on recovery and, with the
// intended one-value-one-place pin removed for the counterfactual, stale ALU1
// priority overwrites the ALU0 value and x11 becomes 100. This is a
// provably-entered + architecturally-consequential test for the new state.
module tb_rv32i_ss_issue_wakeup_recovery;
  import rv32i_ss_pkg::*;

  localparam time CLK_PERIOD = 10ns;

  logic clk;
  logic rst_n;

  word_t imem_addr;
  word_t [1:0] imem_rdata;
  logic imem_req_valid, imem_req_ready;
  word_t imem_req_addr;
  logic imem_resp_valid, imem_resp_ready;
  word_t [1:0] imem_resp_data;
  word_t imem [0:63];
  assign imem_rdata = {
    imem[{imem_addr[7:3], 1'b1}],
    imem[{imem_addr[7:3], 1'b0}]
  };

  rv32i_ss_imem_zero_latency_adapter u_imem_adapter (
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr), .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready), .imem_resp_data(imem_resp_data),
    .line_addr(imem_addr), .line_data(imem_rdata)
  );

  logic       dmem_valid;
  logic       dmem_we;
  logic [3:0] dmem_be;
  word_t      dmem_addr;
  word_t      dmem_wdata;
  logic       dmem_rvalid;
  word_t      dmem_rdata;

  umbra_ss_cpu_top u_cpu (
    .clk         (clk),
    .rst_n       (rst_n),
    .imem_req_valid  (imem_req_valid),
    .imem_req_ready  (imem_req_ready),
    .imem_req_addr   (imem_req_addr),
    .imem_resp_valid (imem_resp_valid),
    .imem_resp_ready (imem_resp_ready),
    .imem_resp_data  (imem_resp_data),
    .dmem_valid  (dmem_valid),
    .dmem_we     (dmem_we),
    .dmem_be     (dmem_be),
    .dmem_addr   (dmem_addr),
    .dmem_wdata  (dmem_wdata),
    .dmem_ready  (1'b1),
    .dmem_rvalid (dmem_rvalid),
    .dmem_rdata  (dmem_rdata)
  );

  `define CORE u_cpu.u_core
  `define RN   u_cpu.u_core.u_rename
  `define PRF  u_cpu.u_core.u_prf
  `define LSQ  u_cpu.u_core.u_lsq

  int errors;
  int checks;
  int waited;
  int cycle_count;

  bit victim_issue_seen;
  bit recovery_window_seen;
  bit forced_post_recovery_hold_seen;
  bit forced_contested_window_seen;
  bit force_hold_active;
  bit survivor_refill_seen;
  bit victim_pdst_reallocated_seen;
  bit true_producer_alu0_seen;
  bit two_holders_one_pdst_seen;
  bit live_holder_consumer_seen;
  bit marker_seen;

  phys_reg_t victim_pdst;
  word_t victim_result;

  task automatic check_bit(input string name, input logic got, input logic exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got=%0b exp=%0b", name, got, exp);
      errors++;
    end
  endtask

  task automatic check_word(input string name, input word_t got, input word_t exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got=%08h exp=%08h", name, got, exp);
      errors++;
    end
  endtask

  task automatic check_arch(input string name, input int regno, input word_t exp);
    phys_reg_t p;
    p = `RN.committed_map_q[regno];
    checks++;
    if (`PRF.regs_q[p] !== exp) begin
      $error("[%s] x%0d=%08h via p%0d exp=%08h",
             name, regno, `PRF.regs_q[p], p, exp);
      errors++;
    end
  endtask

  task automatic pulse_load_response(input word_t data);
    dmem_rdata = data;
    dmem_rvalid = 1'b1;
    @(posedge clk);
    @(negedge clk);
    dmem_rvalid = 1'b0;
    dmem_rdata = '0;
  endtask

  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;

  initial begin
    repeat (3000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_issue_wakeup_recovery exceeded 3000 cycles");
  end

  always @(posedge clk) begin
    if (rst_n === 1'b1) begin
      cycle_count <= cycle_count + 1;
      if (victim_issue_seen && !marker_seen) begin
        $display("TRACE IWREC c=%0d recover=%0b issue={%0b:%08h,%0b:%08h} grant={a0:%0b a1:%0b ag:%0b lq:%0b} holders=%b live={%0b,%0b} pdst={%0d,%0d} bundle=%0b pc0=%08h alloc0=%0d",
                 cycle_count, `CORE.branch_recover_req,
                 `CORE.alu0_issue_fire, `CORE.alu0_issue_entry.pc,
                 `CORE.alu1_issue_fire, `CORE.alu1_issue_entry.pc,
                 `CORE.cdb_grant_alu0, `CORE.cdb_grant_alu1,
                 `CORE.cdb_grant_agen, `CORE.cdb_grant_lq,
                 `CORE.holder_valid, `CORE.alu0_holder_live_q,
                 `CORE.alu1_holder_live_q, `CORE.alu0_complete.pdst,
                 `CORE.alu1_complete.pdst, `CORE.bundle_fire,
                 `CORE.decoded_pc[0], `CORE.preg_alloc_reg[0]);
      end
      // Branch (pc 0x10) must be grant position 0 / ALU0; the younger victim
      // (pc 0x14) must occupy ALU1 on the same issue edge.
      if (`CORE.alu0_exec_fire &&
          (`CORE.alu0_exec_entry.pc == 32'h0000_0010) &&
          (`CORE.alu0_exec_entry.op_class == OOO_OP_BRANCH) &&
          `CORE.alu1_exec_fire &&
          (`CORE.alu1_exec_entry.pc == 32'h0000_0014)) begin
        victim_issue_seen <= 1'b1;
        victim_pdst <= `CORE.alu1_exec_entry.pdst;
        victim_result <= `CORE.exec_result[1];
      end

      // Recovery broadcasts one cycle after branch issue. The older load-B
      // completion and branch take both CDB lanes, leaving ALU1 resident.
      if (`CORE.branch_recover_req && `CORE.alu1_complete.valid &&
          (`CORE.alu1_complete.pdst == victim_pdst) &&
          !`CORE.cdb_grant_alu1 && `CORE.cdb_grant_alu0 &&
          `CORE.cdb_grant_lq) begin
        recovery_window_seen <= 1'b1;
      end

      // These two older-than-branch survivors wake from load B after the
      // recovery edge and refill two different completion clients.
      if (`CORE.alu0_issue_fire &&
          (`CORE.alu0_issue_entry.pc == 32'h0000_0008) &&
          `CORE.agen_issue_fire &&
          (`CORE.agen_enq_entry.pc == 32'h0000_000c)) begin
        survivor_refill_seen <= 1'b1;
      end

      // The first target allocation consumes the older free row; recovery's
      // reclaimed victim row must become slot 1's producer destination.
      if (`CORE.bundle_fire && (`CORE.decoded_pc[0] == 32'h0000_0020) &&
          `CORE.preg_slot_need[1] &&
          (`CORE.preg_alloc_reg[1] == victim_pdst)) begin
        victim_pdst_reallocated_seen <= 1'b1;
      end

      // ALU1 is physically occupied by the killed packet, so the new owner
      // must issue through ALU0 while the stale packet remains resident.
      if ((`CORE.alu0_issue_fire || `CORE.alu0_exec_fire) &&
          ((`CORE.alu0_issue_entry.pc == 32'h0000_0024) ||
           (`CORE.alu0_exec_entry.pc == 32'h0000_0024)) &&
          `CORE.alu1_complete.valid &&
          (`CORE.alu1_complete.pdst == victim_pdst) &&
          !`CORE.alu1_holder_live_q) begin
        true_producer_alu0_seen <= 1'b1;
      end

      if (`CORE.alu0_holder_live_q && `CORE.alu0_complete.valid &&
          (`CORE.alu0_complete.pdst == victim_pdst) &&
          !`CORE.alu1_holder_live_q && `CORE.alu1_complete.valid &&
          (`CORE.alu1_complete.pdst == victim_pdst)) begin
        two_holders_one_pdst_seen <= 1'b1;
      end

      // Occupancy-only: the consumer cannot issue into ALU0 while the
      // producer holder is still valid. Architectural consequence is
      // unchanged: operand_a must be 5 (live producer / PRF), not 99
      // (killed ALU1 packet), and that issue happens while the killed
      // packet is still physically resident.
      if ((`CORE.alu0_issue_fire &&
           (`CORE.alu0_issue_entry.pc == 32'h0000_0028) &&
           (`CORE.alu0_operand_a == 32'd5) &&
           `CORE.alu1_complete.valid &&
           (`CORE.alu1_complete.pdst == victim_pdst) &&
           !`CORE.alu1_holder_live_q) ||
          (`CORE.alu1_issue_fire &&
           (`CORE.alu1_issue_entry.pc == 32'h0000_0028) &&
           (`CORE.alu1_operand_a == 32'd5) &&
           `CORE.alu1_complete.valid &&
           (`CORE.alu1_complete.pdst == victim_pdst) &&
           !`CORE.alu1_holder_live_q)) begin
        live_holder_consumer_seen <= 1'b1;
        if (force_hold_active) forced_contested_window_seen <= 1'b1;
      end

      if ((`CORE.commit_fire[0] && `CORE.commit_rd_wen[0] &&
           (`CORE.commit_rd[0] == arch_reg_t'(14)) &&
           (`CORE.commit_wdata[0] == 32'd14)) ||
          (`CORE.commit_fire[1] && `CORE.commit_rd_wen[1] &&
           (`CORE.commit_rd[1] == arch_reg_t'(14)) &&
           (`CORE.commit_wdata[1] == 32'd14))) begin
        marker_seen <= 1'b1;
      end
    end
  end

  initial begin
    $display("[tb_rv32i_ss_issue_wakeup_recovery] starting");
    errors = 0;
    checks = 0;
    cycle_count = 0;
    victim_issue_seen = 1'b0;
    recovery_window_seen = 1'b0;
    forced_post_recovery_hold_seen = 1'b0;
    forced_contested_window_seen = 1'b0;
    force_hold_active = 1'b0;
    survivor_refill_seen = 1'b0;
    victim_pdst_reallocated_seen = 1'b0;
    true_producer_alu0_seen = 1'b0;
    two_holders_one_pdst_seen = 1'b0;
    live_holder_consumer_seen = 1'b0;
    marker_seen = 1'b0;
    victim_pdst = '0;
    victim_result = '0;
    dmem_rvalid = 1'b0;
    dmem_rdata = '0;

    for (int i = 0; i < 64; i++) imem[i] = 32'h0000_0013;

    imem[ 0] = 32'h0000_2083; // 0x00 lw   x1,0(x0)       -- load A
    imem[ 1] = 32'h0040_2103; // 0x04 lw   x2,4(x0)       -- load B
    imem[ 2] = 32'h00a1_0193; // 0x08 addi x3,x2,10       -- survivor ALU
    imem[ 3] = 32'h0001_2023; // 0x0c sw   x0,0(x2)       -- survivor AGEN
    imem[ 4] = 32'h0000_9863; // 0x10 bne  x1,x0,+16      -> 0x20
    imem[ 5] = 32'h0620_8493; // 0x14 addi x9,x1,98       -- ALU1 victim = 99
    imem[ 6] = 32'h0014_8513; // 0x18 addi x10,x9,1       -- wrong path
    imem[ 7] = 32'h0000_0013; // 0x1c nop
    imem[ 8] = 32'h00c0_0613; // 0x20 addi x12,x0,12      -- older free row
    imem[ 9] = 32'h0050_0493; // 0x24 addi x9,x0,5        -- reuses victim pdst
    imem[10] = 32'h0014_8593; // 0x28 addi x11,x9,1       -- must consume 5
    imem[11] = 32'h00d0_0693; // 0x2c addi x13,x0,13      -- filler
    imem[12] = 32'h00e0_0713; // 0x30 addi x14,x0,14      -- marker
    imem[13] = 32'h0000_006f; // 0x34 jal  x0,0

    rst_n = 1'b0;
    repeat (4) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;

    // Respond to load A first. Its value releases the branch and victim.
    waited = 0;
    while ((`LSQ.lq_out_count_q == 2'd0)) begin
      @(negedge clk);
      waited++;
      if (waited > 200) $fatal(1, "load A never became outstanding");
    end
    check_bit("load A is ROB row 0",
              `LSQ.lq_out_head_entry.rob_idx == rob_idx_t'(0), 1'b1);
    pulse_load_response(32'd1);

    // Wait until branch+victim issue together. Load B must already be the
    // outstanding request; return it on this exact edge so its completion is
    // resident during the next cycle's registered recovery broadcast.
    waited = 0;
    while (!(`CORE.alu0_exec_fire &&
             (`CORE.alu0_exec_entry.pc == 32'h0000_0010) &&
             `CORE.alu1_exec_fire &&
             (`CORE.alu1_exec_entry.pc == 32'h0000_0014))) begin
      @(negedge clk);
      waited++;
      if (waited > 300) begin
        $fatal(1, "branch/victim dual issue not reached: issue=%b pc={%08h,%08h}",
               `CORE.issue_fire, `CORE.alu1_issue_entry.pc,
               `CORE.alu0_issue_entry.pc);
      end
    end
    check_bit("load B outstanding on victim issue",
              (`LSQ.lq_out_count_q != 2'd0), 1'b1);
    check_bit("load B is ROB row 1",
              `LSQ.lq_out_head_entry.rob_idx == rob_idx_t'(1), 1'b1);
    pulse_load_response(32'h0000_0100);

    // The recovery edge clears ALU1 liveness. Suppress only the killed
    // packet's CDB eligibility until the correct-path dependent consumer has
    // sampled the contested bypass, including the refill interval before
    // the reclaimed physical register is renamed. No packet/tag/data/free-list
    // state or architectural result is forced.
    while (!`CORE.branch_recover_req) @(negedge clk);
    @(posedge clk); #1;
    check_bit("recovery cleared victim liveness",
              !`CORE.alu1_holder_live_q, 1'b1);
    check_bit("killed victim packet remains resident",
              `CORE.alu1_complete.valid &&
              (`CORE.alu1_complete.pdst == victim_pdst), 1'b1);
    force_hold_active = 1'b1;
    force `CORE.holder_valid[1] = 1'b0;
    forced_post_recovery_hold_seen = 1'b1;
    waited = 0;
    while (!live_holder_consumer_seen) begin
      @(negedge clk);
      waited++;
      if (waited > 40) begin
        $fatal(1, "contested consumer did not issue while killed holder was retained");
      end
    end
    release `CORE.holder_valid[1];
    force_hold_active = 1'b0;

    waited = 0;
    while (!marker_seen) begin
      @(posedge clk);
      waited++;
      if (waited > 500) $fatal(1, "marker never committed");
    end
    repeat (4) @(posedge clk);

    // Provably-entered pins, from initial victim placement through contested
    // consumer operand. These prevent a later timing retune from going green
    // after drifting off the load-bearing window.
    check_bit("victim issued in ALU1 beside recovering branch",
              victim_issue_seen, 1'b1);
    check_bit("recovery left ALU1 holder resident and ungranted",
              recovery_window_seen, 1'b1);
    check_bit("post-recovery killed-holder CDB exclusion forced",
              forced_post_recovery_hold_seen, 1'b1);
    check_bit("forced exclusion remained active through contested consumer",
              forced_contested_window_seen, 1'b1);
    check_bit("older ALU plus AGEN survivors refilled after recovery",
              survivor_refill_seen, 1'b1);
    check_bit("correct-path producer reclaimed victim pdst",
              victim_pdst_reallocated_seen, 1'b1);
    check_bit("killed ALU1 packet stayed resident through consumer",
              forced_contested_window_seen, 1'b1);
    check_bit("consumer took the live ALU0 holder value",
              live_holder_consumer_seen, 1'b1);

    check_word("captured killed result", victim_result, 32'd99);
    check_arch("older load-B consumer", 3, 32'h0000_010a);
    check_arch("correct-path producer", 9, 32'd5);
    check_arch("correct-path dependent consumer", 11, 32'd6);
    check_arch("wrong-path dependent did not commit", 10, 32'd0);
    check_arch("marker", 14, 32'd14);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_issue_wakeup_recovery] PASS checks=%0d victim_pdst=%0d",
               checks, victim_pdst);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_issue_wakeup_recovery] FAIL errors=%0d checks=%0d",
             errors, checks);
    end
  end
endmodule
