// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// =============================================================================
// tb_rv32i_ss_combiner_sweep -- exhaustive combiner unit sweep: all 32 heads x
// all 992 distinct ROB-index combinations x all 16 outcome combinations (507,904
// vectors) against an INDEPENDENT host-integer age oracle, with 8-family
// reachability accounting and a return-to-idle check.
//
// =============================================================================
module tb_rv32i_ss_combiner_sweep;
  import rv32i_ss_pkg::*;

  // The DUT's tripwires are clocked. Each vector holds #1 from an
  // integer time; posedges at +0.5 sample every one of the 507,904 legal
  // vectors against the pins, settled, none may fatal.
  logic clk = 1'b0;
  always #0.5 clk = ~clk;

  branch_candidate_t [1:0] branch_candidate;
  rob_idx_t                rob_head_idx;
  branch_candidate_t       selected_recovery;
  branch_mask_t            checkpoint_release_mask;

  longint unsigned vectors;
  int family_seen [0:7];

  rv32i_ss_branch_combiner dut (
    .clk                     (clk),
    .branch_candidate        (branch_candidate),
    .rob_head_idx            (rob_head_idx),
    .selected_recovery       (selected_recovery),
    .checkpoint_release_mask (checkpoint_release_mask)
  );

  // outcome: 0=idle, 1=correct, 2=recover, 3=resolve-only/misaligned.
  function automatic branch_candidate_t make_candidate(
      input int outcome, input int checkpoint_id, input int rob_idx,
      input word_t target);
    branch_candidate_t c;
    c = '0;
    c.resolve_valid = (outcome != 0);
    c.correct_valid = (outcome == 1);
    c.recover_valid = (outcome == 2);
    c.checkpoint_id = ckpt_idx_t'(checkpoint_id);
    c.rob_idx       = rob_idx_t'(rob_idx);
    c.target        = target;
    return c;
  endfunction

  task automatic apply_and_check(
      input int head, input int idx0, input int idx1,
      input int outcome0, input int outcome1);
    branch_candidate_t c0, c1, exp_sel;
    branch_candidate_t [1:0] next_stim;
    branch_mask_t exp_release;
    int age0, age1, recovery_age;
    bit have_recovery;

    c0 = make_candidate(outcome0, 0, idx0,
                        32'hA000_0000 | word_t'(idx0));
    c1 = make_candidate(outcome1, 1, idx1,
                        32'hB000_0000 | word_t'(idx1));
    age0 = (idx0 - head) & 31;
    age1 = (idx1 - head) & 31;

    // Contract oracle: select by ROB ring distance, then qualify every
    // correct result against the selected recovery. This is deliberately
    // expressed as host-style integer age arithmetic, not the DUT equation.
    exp_sel       = '0;
    exp_release   = '0;
    have_recovery = 1'b0;
    recovery_age  = 0;
    if ((outcome0 == 2) && ((outcome1 != 2) || (age0 < age1))) begin
      exp_sel       = c0;
      have_recovery = 1'b1;
      recovery_age  = age0;
    end else if (outcome1 == 2) begin
      exp_sel       = c1;
      have_recovery = 1'b1;
      recovery_age  = age1;
    end

    if ((outcome0 == 1) && (!have_recovery || (age0 < recovery_age)))
      exp_release[0] = 1'b1;
    if ((outcome1 == 1) && (!have_recovery || (age1 < recovery_age)))
      exp_release[1] = 1'b1;

    next_stim[0] = c0;
    next_stim[1] = c1;
    rob_head_idx = rob_idx_t'(head);
    branch_candidate = next_stim;
    #1;

    vectors++;
    if ((selected_recovery !== exp_sel) ||
        (checkpoint_release_mask !== exp_release)) begin
      $fatal(1,
        "COMBINER mismatch vector=%0d head=%0d idx={%0d,%0d} outcome={%0d,%0d} age={%0d,%0d} got_sel=%h exp_sel=%h got_rel=%b exp_rel=%b",
        vectors, head, idx0, idx1, outcome0, outcome1, age0, age1,
        selected_recovery, exp_sel, checkpoint_release_mask, exp_release);
    end

    // Eight consequential contract families, counted once each below.
    if ((outcome0 == 0) && (outcome1 == 0)) family_seen[0] = 1;
    if ((outcome0 == 1) && (outcome1 == 1)) family_seen[1] = 1;
    if (((outcome0 == 2) ^ (outcome1 == 2)) &&
        !((outcome0 == 1) || (outcome1 == 1))) family_seen[2] = 1;
    if ((outcome0 == 2) && (outcome1 == 2) && (age0 < age1)) family_seen[3] = 1;
    if ((outcome0 == 2) && (outcome1 == 2) && (age1 < age0)) family_seen[4] = 1;
    if ((outcome0 == 1) && (outcome1 == 2) && (age0 < age1) ||
        (outcome1 == 1) && (outcome0 == 2) && (age1 < age0)) family_seen[5] = 1;
    if ((outcome0 == 1) && (outcome1 == 2) && (age0 > age1) ||
        (outcome1 == 1) && (outcome0 == 2) && (age1 > age0)) family_seen[6] = 1;
    if ((outcome0 == 3) || (outcome1 == 3)) family_seen[7] = 1;
  endtask

  initial begin : exhaustive_campaign
    int head, idx0, idx1, outcome0, outcome1, families;
    branch_candidate_t [1:0] idle;

    vectors = 0;
    for (int f = 0; f < 8; f++) family_seen[f] = 0;
    branch_candidate = '0;
    rob_head_idx = '0;
    #1;

    // All heads, all distinct physical ROB-index combinations, and every legal
    // outcome combination. Distinct indices match the integrated machine pin.
    for (head = 0; head < OOO_ROB_DEPTH; head++) begin
      for (idx0 = 0; idx0 < OOO_ROB_DEPTH; idx0++) begin
        for (idx1 = 0; idx1 < OOO_ROB_DEPTH; idx1++) begin
          if (idx0 != idx1) begin
            for (outcome0 = 0; outcome0 < 4; outcome0++) begin
              for (outcome1 = 0; outcome1 < 4; outcome1++) begin
                apply_and_check(head, idx0, idx1, outcome0, outcome1);
              end
            end
          end
        end
      end
    end

    idle = '0;
    branch_candidate = idle;
    rob_head_idx = '0;
    #1;
    if ((selected_recovery !== '0) || (checkpoint_release_mask !== '0))
      $fatal(1, "COMBINER outputs failed to return to zero");

    families = 0;
    for (int f = 0; f < 8; f++) families += family_seen[f];
    if (families != 8)
      $fatal(1, "COMBINER family reachability incomplete: %0d/8", families);

    $display("[tb_rv32i_ss_combiner_sweep] PASS checks=%0d (families=%0d)", vectors, families);
    $finish;
  end
endmodule
