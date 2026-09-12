`timescale 1ns/1ps

// tb_rv32i_ss_branch_combiner — standalone unit battery for the
// release/recovery combiner (pure comb datapath, drive/settle/check).
//
// The DUT's tripwires are clocked: stimulus lands at integer times and
// holds #1, the free-running clock below puts one posedge at each +0.5, so
// every applied vector is sampled by the pins exactly once, settled,
// mid-hold. The datapath checks themselves stay clockless.
//
// Green run:            all-legal stimulus, prints "PASS checks=N".
// Tripwire RED runs:    +red_equal_age / +red_correct_recover drive one
//                       illegal vector each and EXPECT the DUT $fatal at the
//                       first posedge inside the hold; surviving it prints
//                       TB_RED_FAIL (no PASS line).

module tb_rv32i_ss_branch_combiner;
  import rv32i_ss_pkg::*;

  logic clk = 1'b0;
  always #0.5 clk = ~clk;   // posedges at 0.5, 1.5, ... — mid-hold sampling

  branch_candidate_t [1:0] branch_candidate;
  rob_idx_t                rob_head_idx;
  branch_candidate_t       selected_recovery;
  branch_mask_t            checkpoint_release_mask;

  integer checks = 0;
  integer errors = 0;

  rv32i_ss_branch_combiner dut (
    .clk                     (clk),
    .branch_candidate        (branch_candidate),
    .rob_head_idx            (rob_head_idx),
    .selected_recovery       (selected_recovery),
    .checkpoint_release_mask (checkpoint_release_mask)
  );

  function automatic branch_candidate_t mk(
      input logic      resolve,
      input logic      recover,
      input logic      correct,
      input ckpt_idx_t ck,
      input rob_idx_t  idx,
      input word_t     tgt);
    branch_candidate_t c;
    c = '0;
    c.resolve_valid = resolve;
    c.recover_valid = recover;
    c.correct_valid = correct;
    c.checkpoint_id = ck;
    c.rob_idx       = idx;
    c.target        = tgt;
    return c;
  endfunction

  // Single atomic assignment of the whole candidate array so the DUT
  // tripwires never see a half-updated (transiently illegal) input.
  task automatic apply(input branch_candidate_t c0,
                       input branch_candidate_t c1,
                       input rob_idx_t          head);
    branch_candidate_t [1:0] stim;
    stim[0] = c0;
    stim[1] = c1;
    rob_head_idx     = head;
    branch_candidate = stim;
    #1;
  endtask

  task automatic check_sel(input string name, input branch_candidate_t exp);
    checks = checks + 1;
    if (selected_recovery !== exp) begin
      errors = errors + 1;
      $error("[%s] selected_recovery=%h expected=%h", name,
             selected_recovery, exp);
    end
  endtask

  task automatic check_rel(input string name, input branch_mask_t exp);
    checks = checks + 1;
    if (checkpoint_release_mask !== exp) begin
      errors = errors + 1;
      $error("[%s] release_mask=%b expected=%b", name,
             checkpoint_release_mask, exp);
    end
  endtask

  branch_candidate_t c0, c1;

  initial begin
    branch_candidate = '0;
    rob_head_idx     = '0;
    #1;

    // ---- tripwire RED modes: one illegal vector, expect DUT $fatal ----
    if ($test$plusargs("red_equal_age")) begin
      apply(mk(1'b1, 1'b1, 1'b0, ckpt_idx_t'(0), rob_idx_t'(7), 32'h0),
            mk(1'b1, 1'b1, 1'b0, ckpt_idx_t'(1), rob_idx_t'(7), 32'h4),
            rob_idx_t'(0));
      $display("TB_RED_FAIL: equal-idx dual recovery did not trip");
      $finish;
    end
    if ($test$plusargs("red_correct_recover")) begin
      apply(mk(1'b1, 1'b1, 1'b1, ckpt_idx_t'(0), rob_idx_t'(7), 32'h0),
            mk(1'b0, 1'b0, 1'b0, ckpt_idx_t'(0), rob_idx_t'(0), 32'h0),
            rob_idx_t'(0));
      $display("TB_RED_FAIL: correct&&recover candidate did not trip");
      $finish;
    end

    // ---- neither candidate valid ----
    apply('0, '0, rob_idx_t'(0));
    check_sel("T1 idle sel", '0);
    check_rel("T1 idle rel", '0);

    // ---- candidate 0 correct alone ----
    c0 = mk(1'b1, 1'b0, 1'b1, ckpt_idx_t'(2), rob_idx_t'(4), 32'h0);
    apply(c0, '0, rob_idx_t'(0));
    check_sel("T2 c0-correct sel", '0);
    check_rel("T2 c0-correct rel", branch_mask_t'(4'b0100));

    // ---- candidate 1 correct alone ----
    c1 = mk(1'b1, 1'b0, 1'b1, ckpt_idx_t'(1), rob_idx_t'(6), 32'h0);
    apply('0, c1, rob_idx_t'(0));
    check_sel("T3 c1-correct sel", '0);
    check_rel("T3 c1-correct rel", branch_mask_t'(4'b0010));

    // ---- both correct -> two release bits ----
    c0 = mk(1'b1, 1'b0, 1'b1, ckpt_idx_t'(2), rob_idx_t'(4), 32'h0);
    c1 = mk(1'b1, 1'b0, 1'b1, ckpt_idx_t'(1), rob_idx_t'(6), 32'h0);
    apply(c0, c1, rob_idx_t'(0));
    check_sel("T4 both-correct sel", '0);
    check_rel("T4 both-correct rel", branch_mask_t'(4'b0110));

    // ---- candidate 0 recovery alone ----
    c0 = mk(1'b1, 1'b1, 1'b0, ckpt_idx_t'(3), rob_idx_t'(5), 32'hA000_0010);
    apply(c0, '0, rob_idx_t'(0));
    check_sel("T5 c0-recover sel", c0);
    check_rel("T5 c0-recover rel", '0);

    // ---- candidate 1 recovery alone ----
    c1 = mk(1'b1, 1'b1, 1'b0, ckpt_idx_t'(0), rob_idx_t'(9), 32'hB000_0020);
    apply('0, c1, rob_idx_t'(0));
    check_sel("T6 c1-recover sel", c1);
    check_rel("T6 c1-recover rel", '0);

    // ---- dual recovery, candidate 0 older ----
    c0 = mk(1'b1, 1'b1, 1'b0, ckpt_idx_t'(1), rob_idx_t'(3), 32'hA000_0030);
    c1 = mk(1'b1, 1'b1, 1'b0, ckpt_idx_t'(2), rob_idx_t'(9), 32'hB000_0040);
    apply(c0, c1, rob_idx_t'(0));
    check_sel("T7 dual c0-older sel", c0);
    check_rel("T7 dual c0-older rel", '0);

    // ---- dual recovery, candidate 1 older ----
    c0 = mk(1'b1, 1'b1, 1'b0, ckpt_idx_t'(1), rob_idx_t'(9), 32'hA000_0050);
    c1 = mk(1'b1, 1'b1, 1'b0, ckpt_idx_t'(2), rob_idx_t'(3), 32'hB000_0060);
    apply(c0, c1, rob_idx_t'(0));
    check_sel("T8 dual c1-older sel", c1);
    check_rel("T8 dual c1-older rel", '0);

    // ---- dual recovery across the ROB wrap ----
    // head=30: c0 idx=1 is ring-age 3, c1 idx=31 is ring-age 1 -> c1 is
    // older. A raw index compare (1 < 31) would wrongly pick c0.
    c0 = mk(1'b1, 1'b1, 1'b0, ckpt_idx_t'(1), rob_idx_t'(1),  32'hA000_0070);
    c1 = mk(1'b1, 1'b1, 1'b0, ckpt_idx_t'(2), rob_idx_t'(31), 32'hB000_0080);
    apply(c0, c1, rob_idx_t'(30));
    check_sel("T9 wrap c1-older sel", c1);
    check_rel("T9 wrap c1-older rel", '0);

    // ---- older-correct + younger-recovery -> the correct releases ----
    c0 = mk(1'b1, 1'b0, 1'b1, ckpt_idx_t'(0), rob_idx_t'(12), 32'h0);
    c1 = mk(1'b1, 1'b1, 1'b0, ckpt_idx_t'(3), rob_idx_t'(15), 32'hB000_0090);
    apply(c0, c1, rob_idx_t'(10));
    check_sel("T10 sel is younger recovery", c1);
    check_rel("T10 older correct releases", branch_mask_t'(4'b0001));

    // ---- older-recovery + younger-correct -> no release ----
    c0 = mk(1'b1, 1'b1, 1'b0, ckpt_idx_t'(0), rob_idx_t'(12), 32'hA000_00A0);
    c1 = mk(1'b1, 1'b0, 1'b1, ckpt_idx_t'(3), rob_idx_t'(15), 32'h0);
    apply(c0, c1, rob_idx_t'(10));
    check_sel("T11 sel is older recovery", c0);
    check_rel("T11 younger correct held", '0);

    // ---- resolve-only (misaligned taken) -> neither output ----
    c0 = mk(1'b1, 1'b0, 1'b0, ckpt_idx_t'(1), rob_idx_t'(4), 32'hC000_00B0);
    apply(c0, '0, rob_idx_t'(0));
    check_sel("T12 misalign sel", '0);
    check_rel("T12 misalign rel", '0);

    // ---- misaligned resolve-only + other correct ----
    c0 = mk(1'b1, 1'b0, 1'b0, ckpt_idx_t'(1), rob_idx_t'(4), 32'hC000_00C0);
    c1 = mk(1'b1, 1'b0, 1'b1, ckpt_idx_t'(2), rob_idx_t'(6), 32'h0);
    apply(c0, c1, rob_idx_t'(0));
    check_sel("T13 misalign+correct sel", '0);
    check_rel("T13 misalign+correct rel", branch_mask_t'(4'b0100));

    // ---- release qualification across the ROB wrap ----
    // head=30: recovery at idx=0 is ring-age 2, correct at idx=31 is
    // ring-age 1 -> strictly older -> releases. Raw compare (31 < 0
    // false) would wrongly hold it.
    c0 = mk(1'b1, 1'b1, 1'b0, ckpt_idx_t'(1), rob_idx_t'(0),  32'hA000_00D0);
    c1 = mk(1'b1, 1'b0, 1'b1, ckpt_idx_t'(2), rob_idx_t'(31), 32'h0);
    apply(c0, c1, rob_idx_t'(30));
    check_sel("T14 wrap sel", c0);
    check_rel("T14 wrap older-correct releases", branch_mask_t'(4'b0100));

    // ---- return to idle: outputs must drop with the inputs ----
    apply('0, '0, rob_idx_t'(0));
    check_sel("T15 back-to-idle sel", '0);
    check_rel("T15 back-to-idle rel", '0);

    if (errors == 0)
      $display("[tb_rv32i_ss_branch_combiner] PASS checks=%0d", checks);
    else
      $display("[tb_rv32i_ss_branch_combiner] FAIL checks=%0d errors=%0d",
               checks, errors);
    $finish;
  end

endmodule
