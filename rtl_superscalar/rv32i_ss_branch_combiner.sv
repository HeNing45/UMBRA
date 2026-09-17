// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

module rv32i_ss_branch_combiner
  import rv32i_ss_pkg::*;
(
  // clk feeds ONLY the sim-only tripwires below: sampling the input contract
  // at the consuming edge is what lets the pins stay armed in every build
  // (including Verilator --no-timing). The datapath remains pure comb.
  input  logic                    clk,
  input  branch_candidate_t [1:0] branch_candidate,
  input  rob_idx_t                rob_head_idx,

  output branch_candidate_t       selected_recovery,
  output branch_mask_t            checkpoint_release_mask
);

  rob_idx_t [1:0] candidate_age;
  rob_idx_t       recovery_age;

  always_comb begin
    candidate_age[0] = branch_candidate[0].rob_idx - rob_head_idx;
    candidate_age[1] = branch_candidate[1].rob_idx - rob_head_idx;

    selected_recovery       = '0;
    checkpoint_release_mask = '0;
    recovery_age            = '0;

    // Select the oldest recovery candidate, then derive the release set.
    if (branch_candidate[0].recover_valid &&
        (!branch_candidate[1].recover_valid ||
         (candidate_age[0] < candidate_age[1]))) begin
      selected_recovery = branch_candidate[0];
    end else if (branch_candidate[1].recover_valid) begin
      selected_recovery = branch_candidate[1];
    end
    recovery_age = selected_recovery.rob_idx - rob_head_idx;

    if (branch_candidate[0].correct_valid &&
        (!selected_recovery.recover_valid ||
         (candidate_age[0] < recovery_age))) begin
      checkpoint_release_mask |= branch_mask_t'(1'b1)
                                 << branch_candidate[0].checkpoint_id;
    end

    if (branch_candidate[1].correct_valid &&
        (!selected_recovery.recover_valid ||
         (candidate_age[1] < recovery_age))) begin
      checkpoint_release_mask |= branch_mask_t'(1'b1)
                                 << branch_candidate[1].checkpoint_id;
    end
  end

`ifndef SYNTHESIS
  // Input-contract tripwires. These are environment invariants, not
  // arbitration cases: two ALUs can never resolve the same ROB entry in
  // one cycle, and one candidate can never be simultaneously correct and
  // recovering. Equality in the age compares is therefore unreachable.
  //
  // Sample checks at the consuming clock edge. Combinational settling can
  // temporarily violate relationships that hold at every real cycle boundary.
  // Clocked checks observe the values downstream registers consume and remain
  // active in simulators without event-delay support. Separate blocks ensure
  // each independent invariant is evaluated.
  always @(posedge clk) begin
    if (branch_candidate[0].recover_valid && branch_candidate[1].recover_valid &&
        (branch_candidate[0].rob_idx == branch_candidate[1].rob_idx)) begin
      $fatal(1, "rv32i_ss_branch_combiner: dual recovery with equal rob_idx");
    end
  end
  always @(posedge clk) begin
    if (branch_candidate[0].correct_valid && branch_candidate[0].recover_valid) begin
      $fatal(1, "rv32i_ss_branch_combiner: candidate 0 correct and recover co-asserted");
    end
  end
  always @(posedge clk) begin
    if (branch_candidate[1].correct_valid && branch_candidate[1].recover_valid) begin
      $fatal(1, "rv32i_ss_branch_combiner: candidate 1 correct and recover co-asserted");
    end
  end
`endif

endmodule
