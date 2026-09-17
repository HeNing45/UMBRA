// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// tb_rv32i_ss_core_stale_wb — natural writeback behavior across recovery.
// Real programs enter the core through the frontend. The only timing control
// is the external load-response release cycle; no completion or recovery
// packet is forced. A release-cycle sweep surrounds a measured broadcast
// cycle, and observation counters require every targeted window to occur.
//
// The stale_a program checks that an older survivor load is accepted during
// the broadcast. A wrong-path load occupies the real LQ mailbox during that
// broadcast, drains after its ROB index has been reused, and must be rejected
// by the generation check against the replacement entry. Accepting it would
// make the replacement instruction commit 0xFEEDFACE.
//
// The stale_b program places a wrong-path divide-by-zero in S_DONE on the
// broadcast. The idle unit does not kill that completion; it drains through
// a real CDB grant and is rejected because its ROB entry is invalid.

`define T36_TB_NAME "tb_rv32i_ss_core_stale_wb"
`define T36_WATCHDOG_CYCLES 12000

module tb_rv32i_ss_core_stale_wb;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;

  `include "tb/t36_harness.svh"
  `include "tb/t36_progs.svh"

  completion_packet_t lqmb, mdc, cdb0, cdb1;
  assign lqmb = `LSQ.lq_complete;
  assign mdc  = `CORE.muldiv_complete;
  assign cdb0 = `CORE.cdb_q[0];
  assign cdb1 = `CORE.cdb_q[1];

  localparam int SURV_IDX  = 2;   // stale_a survivor load rob idx
  localparam int KA_IDX    = 7;   // stale_a wrong-path / refill rob idx
  localparam int KB_IDX    = 4;   // stale_b wrong-path div / refill rob idx

  // per-iteration observation state
  bit  mon_on;
  int  ev_surv_accept_on_bcast;   //
  int  ev_holder_at_bcast;        // provably-entered
  int  ev_stale_seq_reject;       // seq-arm rejection observed
  int  ev_stale_any_reject;       // stale beat rejected (either arm)
  int  ev_three_holders;          //
  int  ev_sdone_spared;           //
  int  ev_md_reject;              // rejection observed
  int  n_bcast;
  int  bcast_cyc_seen;
  completion_packet_t held_at_bcast_pkt, held_at_grant_pkt;
  bit  held_seen, held_grant_seen;
  bit  fresh_done_early_err;

  int  holders_now;

  always @(posedge clk) begin
    if (rst_n === 1'b1 && mon_on) begin
      holders_now = $countones({`CORE.alu0_complete.valid,
                                `CORE.alu1_complete.valid,
                                mdc.valid,
                                `CORE.agen_complete.valid,
                                lqmb.valid});
      if (holders_now >= 3) ev_three_holders++;

      if (`CORE.branch_recover_req === 1'b1) begin
        n_bcast++;
        if (bcast_cyc_seen < 0) bcast_cyc_seen = cyc;
        // survivor beat accepted ON the broadcast cycle.
        if (cdb0.valid === 1'b1 && cdb0.rob_idx === rob_idx_t'(SURV_IDX) &&
            `CORE.rob_wb_accept[0] === 1'b1)
          ev_surv_accept_on_bcast++;
        if (cdb1.valid === 1'b1 && cdb1.rob_idx === rob_idx_t'(SURV_IDX) &&
            `CORE.rob_wb_accept[1] === 1'b1)
          ev_surv_accept_on_bcast++;
        // wrong-path packet held in the REAL LQ mailbox on the broadcast.
        if (lqmb.valid === 1'b1 && lqmb.rob_idx === rob_idx_t'(KA_IDX)) begin
          ev_holder_at_bcast++;
          held_seen        = 1'b1;
          held_at_bcast_pkt = lqmb;
        end
        // wrong-path muldiv S_DONE spared on the broadcast.
        if (mdc.valid === 1'b1 && mdc.rob_idx === rob_idx_t'(KB_IDX) &&
            `CORE.muldiv_busy === 1'b0)
          ev_sdone_spared++;
      end

      // Bit-exact hold: capture the held packet again at its grant.
      if (held_seen && !held_grant_seen && `CORE.cdb_grant_lq === 1'b1) begin
        held_grant_seen   = 1'b1;
        held_at_grant_pkt = lqmb;
      end

      // Stale-beat rejection observation (post-broadcast beats carrying the
      // wrong-path generation). Seq-arm: the ROB row is LIVE (refilled) yet
      // the beat is refused -> the seq term did the adjudication.
      if (n_bcast > 0) begin
        if (cdb0.valid === 1'b1 && cdb0.rob_idx === rob_idx_t'(KA_IDX) &&
            cdb0.result === 32'hFEED_FACE) begin
          check_bit("stale lane-0 beat rejected", `CORE.rob_wb_accept[0], 1'b0);
          ev_stale_any_reject++;
          if (`ROB.valid_q[KA_IDX] === 1'b1) ev_stale_seq_reject++;
          // K=2 restatement: under K=1 the whole refill stream was
          // time-gated behind the stale response (the one outstanding
          // window), so the refilled row could never be done when the stale
          // beat arrived — an accidental schedule property this flag used to
          // assert. Under K=2 the refill runs immediately and the row being
          // done at the stale beat is the NORMAL late-release order. The
          // architectural invariant is that the stale beat is never
          // ACCEPTED into the refilled row, done or not.
          if ((`ROB.done_q[KA_IDX] === 1'b1) &&
              (`CORE.rob_wb_accept[0] === 1'b1)) fresh_done_early_err = 1'b1;
        end
        if (cdb1.valid === 1'b1 && cdb1.rob_idx === rob_idx_t'(KA_IDX) &&
            cdb1.result === 32'hFEED_FACE) begin
          check_bit("stale lane-1 beat rejected", `CORE.rob_wb_accept[1], 1'b0);
          ev_stale_any_reject++;
          if (`ROB.valid_q[KA_IDX] === 1'b1) ev_stale_seq_reject++;
          if ((`ROB.done_q[KA_IDX] === 1'b1) &&
              (`CORE.rob_wb_accept[1] === 1'b1)) fresh_done_early_err = 1'b1;
        end
        // Part B: the spared div's beat (result FFFFFFFF at KB) must be refused.
        if (cdb0.valid === 1'b1 && cdb0.rob_idx === rob_idx_t'(KB_IDX) &&
            cdb0.result === 32'hFFFF_FFFF) begin
          check_bit("spared-div lane-0 beat rejected", `CORE.rob_wb_accept[0], 1'b0);
          ev_md_reject++;
        end
        if (cdb1.valid === 1'b1 && cdb1.rob_idx === rob_idx_t'(KB_IDX) &&
            cdb1.result === 32'hFFFF_FFFF) begin
          check_bit("spared-div lane-1 beat rejected", `CORE.rob_wb_accept[1], 1'b0);
          ev_md_reject++;
        end
      end
    end
  end

  task automatic iter_reset();
    ev_surv_accept_on_bcast = 0;
    ev_holder_at_bcast      = 0;
    ev_stale_seq_reject     = 0;
    ev_stale_any_reject     = 0;
    ev_sdone_spared         = 0;
    ev_md_reject            = 0;
    n_bcast                 = 0;
    bcast_cyc_seen          = -1;
    held_seen               = 1'b0;
    held_grant_seen         = 1'b0;
    fresh_done_early_err    = 1'b0;
    mon_on = 1'b1;
  endtask

  // One Part-A iteration at a given response-release cycle.
  task automatic run_a(input int rel, output int bcast_out);
    load_prog_stale_a();
    t36_reset();
    dmem_mem[32'h104 >> 2] = 32'hA5A5_0104;   // survivor load data
    dmem_mem[32'h100 >> 2] = 32'hFEED_FACE;   // wrong-path load data
    iter_reset();
    load_release_cyc = rel;
    wait_commits(10, 600);
    repeat (6) @(posedge clk);
    mon_on = 1'b0;
    bcast_out = bcast_cyc_seen;

    check_bit("A: exactly one recovery broadcast", (n_bcast == 1), 1'b1);
    check_bit("A: stale beat never accepted into the refilled row", fresh_done_early_err, 1'b0);
    check_arch("A: survivor load data committed", 2, 32'hA5A5_0104);
    check_arch("A: refilled instruction's value (not stale load data)",
               11, 32'h0000_0077);
    check_arch("A: wrong-path load rd never written", 9, 32'd0);
    check_arch("A: wrong-path addi rd never written", 10, 32'd0);
    check_dmem("A: refill store", 32'h10C, 32'h0000_0077);
    check_freelist_conservation("A");
    if (held_seen) begin
      check_bit("A: held mailbox packet granted later", held_grant_seen, 1'b1);
      if (held_grant_seen) begin
        check_bit("A: held packet bit-identical at its grant",
                  held_at_grant_pkt === held_at_bcast_pkt, 1'b1);
      end
    end
  endtask

  int b0, t, it_bcast;
  int tot_holder_at_bcast   = 0;
  int tot_surv_on_bcast     = 0;
  int tot_seq_reject        = 0;
  int tot_any_reject        = 0;
  int tot_three_holders     = 0;

  initial begin
    $display("[%s] starting", `T36_TB_NAME);
    mon_on = 1'b0;

    // Iteration 0: immediate responses measure the broadcast cycle B0.
    run_a(0, b0);
    check_bit("A: baseline broadcast cycle measured", (b0 > 0), 1'b1);
    $display("[%s] part A baseline B0=%0d", `T36_TB_NAME, b0);

    // Sweep the survivor-response release across the decision/broadcast
    // window. The broadcast cycle itself SHIFTS with the release value (the
    // load traffic competes with the mul's beat for CDB lanes and delays the
    // wake), so the sweep is wide and the monitors self-identify which
    // iterations entered each targeted window; every iteration must keep the
    // architectural contract regardless.
    for (t = b0 - 6; t <= b0 + 16; t++) begin
      run_a(t, it_bcast);
      $display("[%s] A rel=%0d bcast=%0d held=%0d survAcc=%0d seqRej=%0d anyRej=%0d",
               `T36_TB_NAME, t, it_bcast, ev_holder_at_bcast,
               ev_surv_accept_on_bcast, ev_stale_seq_reject, ev_stale_any_reject);
      tot_holder_at_bcast += ev_holder_at_bcast;
      tot_surv_on_bcast   += ev_surv_accept_on_bcast;
      tot_seq_reject      += ev_stale_seq_reject;
      tot_any_reject      += ev_stale_any_reject;
      tot_three_holders   += ev_three_holders;
    end

    check_bit("A: wrong-path packet held in mailbox ON a broadcast (entered)",
              (tot_holder_at_bcast > 0), 1'b1);
    check_bit("A: survivor beat accepted ON a broadcast (entered)",
              (tot_surv_on_bcast > 0), 1'b1);
    check_bit("A: stale beat rejected against the LIVE refilled row (seq arm)",
              (tot_seq_reject > 0), 1'b1);
    check_bit("A: stale-beat rejection observed",
              (tot_any_reject > 0), 1'b1);
    check_bit("A: three simultaneous completion holders (entered)",
              (tot_three_holders > 0), 1'b1);
    $display("[%s] A sweep: held=%0d survAcc=%0d seqRej=%0d anyRej=%0d 3hold=%0d",
             `T36_TB_NAME, tot_holder_at_bcast, tot_surv_on_bcast,
             tot_seq_reject, tot_any_reject, tot_three_holders);

    // ---------------- Part B: spared S_DONE div across the broadcast -------
    load_prog_stale_b();
    t36_reset();
    iter_reset();
    wait_commits(8, 600);
    repeat (6) @(posedge clk);
    mon_on = 1'b0;

    check_bit("B: wrong-path S_DONE spared on the broadcast (entered)",
              (ev_sdone_spared > 0), 1'b1);
    check_bit("B: spared div's beat rejected", (ev_md_reject > 0), 1'b1);
    check_arch("B: refill value at the reused idx", 5, 32'd88);
    check_arch("B: wrong-path div rd never written", 9, 32'd0);
    check_arch("B: wrong-path addi rd never written", 10, 32'd0);
    check_dmem("B: correct-path store", 32'h110, 32'd88);
    check_freelist_conservation("B");
    check_bit("B: muldiv unit back to idle sanity", `CORE.muldiv_busy, 1'b0);

    t36_finish();
  end

endmodule
