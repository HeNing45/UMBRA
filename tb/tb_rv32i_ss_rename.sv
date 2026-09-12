`timescale 1ns/1ps

// Rename bundle battery with rename and free list instantiated together.
// Covers same-bundle RAW bypass, WAW/stale-register ownership, cross-module
// register conservation, and checkpoint creation and recovery in either slot.
// A slot-0 branch checkpoints the pre-bundle map and reclaims a younger
// slot-1 allocation. A slot-1 branch checkpoints the map after slot 0 and
// preserves that older allocation on recovery.
//
// Recovery also exercises the isolated recovery-export block. A combinational
// feedback regression there can hang at zero time rather than fail a check.

module tb_rv32i_ss_rename;

  import rv32i_ss_pkg::*;

  localparam int CLK_PERIOD = 10;

  logic clk, rst_n;

  logic            decoded_valid;
  logic      [1:0] decoded_slot_valid;
  logic            decoded_ready;
  arch_reg_t [1:0] decoded_rs1, decoded_rs2, decoded_rd;
  logic      [1:0] decoded_rd_we;
  logic      [1:0] decoded_needs_checkpoint;

  logic [1:0]      preg_slot_need;
  logic            preg_avail;
  phys_reg_t [1:0] preg_alloc_reg;

  logic            rename_ready;
  phys_reg_t [1:0] rename_prs1, rename_prs2, rename_pdst, rename_stale_pdst;
  arch_reg_t [1:0] rename_rd;
  logic      [1:0] rename_rd_we;

  // per-position commit drive. This TB plays the core's sole-producer
  // role so every retire-effect orientation is directly reachable.
  logic [1:0]       commit_fire, commit_rd_we;
  arch_reg_t [1:0]  commit_rd;
  phys_reg_t [1:0]  commit_pdst, commit_stale_pdst;

  logic                     trap_flush;
  logic                     branch_recover_req;
  ckpt_idx_t                branch_recover_id;
  logic [OOO_PHYS_REGS-1:0] rename_branch_recover_alloc_list;
  branch_mask_t             rename_branch_mask;
  logic                     rename_checkpoint_valid;
  ckpt_idx_t                rename_checkpoint_id;
  branch_mask_t             checkpoint_release_mask;

  // the bundle's transitional fire (the core-seam shape): rename handshake
  logic tb_bundle_fire;
  assign tb_bundle_fire = decoded_valid && decoded_ready;

  integer checks = 0;
  integer errors = 0;

  rv32i_ss_rename u_rn (
    .clk(clk), .rst_n(rst_n),
    .decoded_valid(decoded_valid), .decoded_slot_valid(decoded_slot_valid),
    .decoded_ready(decoded_ready),
    .decoded_rs1(decoded_rs1), .decoded_rs2(decoded_rs2),
    .decoded_rd(decoded_rd), .decoded_rd_we(decoded_rd_we),
    .decoded_needs_checkpoint(decoded_needs_checkpoint),
    .preg_slot_need(preg_slot_need), .preg_avail(preg_avail),
    .preg_alloc_reg(preg_alloc_reg),
    .rename_ready(rename_ready), .bundle_fire(tb_bundle_fire),
    .rename_prs1(rename_prs1), .rename_prs2(rename_prs2),
    .rename_pdst(rename_pdst), .rename_stale_pdst(rename_stale_pdst),
    .rename_rd(rename_rd), .rename_rd_we(rename_rd_we),
    .commit_fire(commit_fire), .commit_rd_we(commit_rd_we),
    .commit_rd(commit_rd), .commit_pdst(commit_pdst),
    .trap_flush(trap_flush),
    .branch_recover_req(branch_recover_req),
    .branch_recover_id(branch_recover_id),
    .rename_branch_recover_alloc_list(rename_branch_recover_alloc_list),
    .rename_branch_mask(rename_branch_mask),
    .rename_checkpoint_valid(rename_checkpoint_valid),
    .rename_checkpoint_id(rename_checkpoint_id),
    .checkpoint_release_mask(checkpoint_release_mask)
  );

  rv32i_ss_free_list u_fl (
    .clk(clk), .rst_n(rst_n),
    .preg_slot_need(preg_slot_need),
    .preg_avail(preg_avail),
    .preg_alloc_reg(preg_alloc_reg),
    .bundle_fire(tb_bundle_fire),
    .commit_fire(commit_fire), .commit_rd_we(commit_rd_we),
    .commit_pdst(commit_pdst), .commit_stale_pdst(commit_stale_pdst),
    .branch_recover_req(branch_recover_req),
    .rename_branch_recover_alloc_list(rename_branch_recover_alloc_list),
    .trap_flush(trap_flush)
  );

  initial clk = 1'b0;
  always #(CLK_PERIOD / 2) clk = ~clk;

  initial begin
    #(CLK_PERIOD * 4000);
    $fatal(1, "[tb_rv32i_ss_rename] WATCHDOG timeout");
  end

  task automatic check(input string name, input logic cond);
    checks++;
    if (!cond) begin
      errors++;
      $display("  FAIL [%0d] %s", checks, name);
    end
  endtask

  function automatic integer free_count_now();
    integer i, c;
    begin
      c = 0;
      for (i = 0; i < OOO_PHYS_REGS; i = i + 1)
        if (u_fl.free_bits_q[i]) c = c + 1;
      free_count_now = c;
    end
  endfunction

  // present one bundle for one fire; sample the rename outputs pre-edge
  task automatic bundle(
      input arch_reg_t rs1_0, input arch_reg_t rs2_0,
      input arch_reg_t rd_0,  input logic we_0,
      input arch_reg_t rs1_1, input arch_reg_t rs2_1,
      input arch_reg_t rd_1,  input logic we_1,
      output phys_reg_t [1:0] o_prs1, output phys_reg_t [1:0] o_prs2,
      output phys_reg_t [1:0] o_pdst, output phys_reg_t [1:0] o_stale);
    @(negedge clk);
    decoded_valid = 1'b1; decoded_slot_valid = 2'b11;
    decoded_rs1 = {rs1_1, rs1_0}; decoded_rs2 = {rs2_1, rs2_0};
    decoded_rd  = {rd_1, rd_0};   decoded_rd_we = {we_1, we_0};
    #1;
    o_prs1[0] = rename_prs1[0]; o_prs1[1] = rename_prs1[1];
    o_prs2[0] = rename_prs2[0]; o_prs2[1] = rename_prs2[1];
    o_pdst[0] = rename_pdst[0]; o_pdst[1] = rename_pdst[1];
    o_stale[0] = rename_stale_pdst[0]; o_stale[1] = rename_stale_pdst[1];
    @(posedge clk); @(negedge clk);
    decoded_valid = 1'b0; decoded_slot_valid = 2'b00;
  endtask

  // one-instruction bundle: slot 0 real, slot 1 payload = GARBAGE bytes
  // that the shape must neutralize (the qualification semantics)
  task automatic bundle_one(
      input arch_reg_t rs1_0, input arch_reg_t rs2_0,
      input arch_reg_t rd_0,  input logic we_0,
      input arch_reg_t junk_rd_1, input logic junk_we_1,
      output phys_reg_t [1:0] o_pdst, output logic [1:0] o_need);
    @(negedge clk);
    decoded_valid = 1'b1; decoded_slot_valid = 2'b01;
    decoded_rs1 = {5'd31, rs1_0}; decoded_rs2 = {5'd31, rs2_0};
    decoded_rd  = {junk_rd_1, rd_0}; decoded_rd_we = {junk_we_1, we_0};
    #1;
    o_pdst[0] = rename_pdst[0]; o_pdst[1] = rename_pdst[1];
    o_need = preg_slot_need;
    @(posedge clk); @(negedge clk);
    decoded_valid = 1'b0; decoded_slot_valid = 2'b00;
  endtask

  // one commit beat through BOTH modules (rename arch map + free list)
  task automatic commit_one(input arch_reg_t rd, input phys_reg_t pdst,
                            input phys_reg_t stale);
    @(negedge clk);
    commit_fire = 2'b01; commit_rd_we = 2'b01;
    commit_rd = {5'd0, rd}; commit_pdst = {6'd0, pdst};
    commit_stale_pdst = {6'd0, stale};
    @(posedge clk); @(negedge clk);
    commit_fire = 2'b00; commit_rd_we = 2'b00;
  endtask

  // an ORDERED-PREFIX pair retiring on one edge. Position 0 is the
  // older instruction; the module seam isolates the map sink for direct proof.
  task automatic commit_pair(input arch_reg_t rd0, input phys_reg_t pdst0,
                             input phys_reg_t stale0,
                             input arch_reg_t rd1, input phys_reg_t pdst1,
                             input phys_reg_t stale1);
    @(negedge clk);
    commit_fire = 2'b11; commit_rd_we = 2'b11;
    commit_rd         = {rd1, rd0};
    commit_pdst       = {pdst1, pdst0};
    commit_stale_pdst = {stale1, stale0};
    @(posedge clk); @(negedge clk);
    commit_fire = 2'b00; commit_rd_we = 2'b00;
  endtask

  // bundle carrying a checkpoint request; samples the grant pre-edge
  task automatic bundle_ck(
      input arch_reg_t rs1_0, input arch_reg_t rs2_0,
      input arch_reg_t rd_0,  input logic we_0,
      input arch_reg_t rs1_1, input arch_reg_t rs2_1,
      input arch_reg_t rd_1,  input logic we_1,
      input logic [1:0] ck,
      output phys_reg_t [1:0] o_pdst,
      output logic o_ck_v, output ckpt_idx_t o_ck_id);
    @(negedge clk);
    decoded_valid = 1'b1; decoded_slot_valid = 2'b11;
    decoded_rs1 = {rs1_1, rs1_0}; decoded_rs2 = {rs2_1, rs2_0};
    decoded_rd  = {rd_1, rd_0};   decoded_rd_we = {we_1, we_0};
    decoded_needs_checkpoint = ck;
    #1;
    o_pdst[0] = rename_pdst[0]; o_pdst[1] = rename_pdst[1];
    o_ck_v = rename_checkpoint_valid; o_ck_id = rename_checkpoint_id;
    @(posedge clk); @(negedge clk);
    decoded_valid = 1'b0; decoded_slot_valid = 2'b00;
    decoded_needs_checkpoint = '0;
  endtask

  task automatic recover_beat(input ckpt_idx_t id);
    @(negedge clk);
    branch_recover_req = 1'b1; branch_recover_id = id;
    @(posedge clk); @(negedge clk);
    branch_recover_req = 1'b0;
  endtask

  task automatic release_beat(input ckpt_idx_t id);
    @(negedge clk);
    checkpoint_release_mask = branch_mask_t'(1'b1) << id;
    @(posedge clk); @(negedge clk);
    checkpoint_release_mask = '0;
  endtask

  // checkpoints are directed to land at id 0 (asserted at each grant),
  // so the peeks below use constant indices (Icarus-safe hierarchy).
  function automatic integer ck0_list_count();
    integer i, c;
    begin
      c = 0;
      for (i = 0; i < OOO_PHYS_REGS; i = i + 1)
        if (u_rn.checkpoint_alloc_list_q[0][i]) c = c + 1;
      ck0_list_count = c;
    end
  endfunction

  phys_reg_t [1:0] prs1, prs2, pdst, stale;
  phys_reg_t p_old5, save_pdst0, save_pdst1;
  integer    n0;
  // section state
  phys_reg_t p_old23, p_old24, p_old25, p_old26;
  phys_reg_t pd1_a41, py0, py1, pd0_a43, py2;
  logic      ck_v;
  ckpt_idx_t ck_id;
  integer    n_pre, n_pre2;

  logic [1:0] need_s;

  initial begin
    decoded_valid = 1'b0; decoded_slot_valid = 2'b00;
    decoded_rs1 = '0; decoded_rs2 = '0;
    decoded_rd = '0; decoded_rd_we = '0; decoded_needs_checkpoint = '0;
    rename_ready = 1'b1;
    commit_fire = 2'b00; commit_rd_we = 2'b00; commit_rd = '0;
    commit_pdst = '0; commit_stale_pdst = '0;
    trap_flush = 1'b0; branch_recover_req = 1'b0; branch_recover_id = '0;
    checkpoint_release_mask = '0;
    rst_n = 1'b0;
    repeat (4) @(posedge clk);
    @(negedge clk); rst_n = 1'b1; @(negedge clk);

    // ============ dual independent renames ============
    n0 = free_count_now();
    bundle(5'd1, 5'd2, 5'd5, 1'b1,  5'd7, 5'd8, 5'd6, 1'b1,
           prs1, prs2, pdst, stale);
    check("A2.1: distinct pdsts granted",
          (pdst[0] != '0) && (pdst[1] != '0) && (pdst[0] != pdst[1]));
    check("A2.1: both map entries updated",
          (u_rn.spec_map_q[5] == pdst[0]) && (u_rn.spec_map_q[6] == pdst[1]));
    check("A2.1: two pregs consumed", free_count_now() == n0 - 2);
    save_pdst0 = pdst[0];

    // ============ RAW bypass hits (rs1 and rs2 forms) ============
    bundle(5'd3, 5'd4, 5'd10, 1'b1,  5'd10, 5'd10, 5'd11, 1'b1,
           prs1, prs2, pdst, stale);
    check("A2.2: rs1 bypass forwards slot-0 grant", prs1[1] == pdst[0]);
    check("A2.2: rs2 bypass forwards slot-0 grant", prs2[1] == pdst[0]);

    // ============ RAW miss reads SLOT 1's own source
    // rs1[0]=x5 and rs1[1]=x6 map differently; a fallback indexed [0]
    // would return map[x5] for slot 1.
    bundle(5'd5, 5'd2, 5'd12, 1'b1,  5'd6, 5'd2, 5'd13, 1'b1,
           prs1, prs2, pdst, stale);
    check("A2.3: slot-1 fallback reads its own rs (not slot 0's)",
          (prs1[1] == u_rn.spec_map_q[6]) && (prs1[1] != u_rn.spec_map_q[5]));

    // ============ no bypass without the write qualifier (bug-2
    // regression): I0 does not write, register numbers collide anyway
    bundle(5'd1, 5'd2, 5'd14, 1'b0,  5'd14, 5'd2, 5'd15, 1'b1,
           prs1, prs2, pdst, stale);
    check("A2.4: rd_we=0 slot 0 never forwards",
          prs1[1] == u_rn.spec_map_q[14]);
    // x0 pin: rd0=x0 "written", rs1[1]=x0 must still read p0
    bundle(5'd1, 5'd2, 5'd0, 1'b1,  5'd0, 5'd2, 5'd16, 1'b1,
           prs1, prs2, pdst, stale);
    check("A2.4: x0 collision still reads p0", prs1[1] == '0);
    check("A2.4: x0 slot allocated nothing", pdst[0] == '0);

    // ============ WAW same-rd — the fate table live ============
    p_old5 = u_rn.spec_map_q[5];
    n0 = free_count_now();
    bundle(5'd1, 5'd2, 5'd5, 1'b1,  5'd3, 5'd4, 5'd5, 1'b1,
           prs1, prs2, pdst, stale);
    check("A3.1: final map owned by slot 1", u_rn.spec_map_q[5] == pdst[1]);
    check("A3.1: slot-0 stale = old mapping", stale[0] == p_old5);
    check("A3.1: slot-1 stale = slot-0 pdst (bypassed)", stale[1] == pdst[0]);
    check("A3.1: stale chain has no duplicate", stale[1] != stale[0]);
    check("A3.1: two pregs consumed", free_count_now() == n0 - 2);
    save_pdst0 = pdst[0]; save_pdst1 = pdst[1];

    // =========== conservation over the WAW commit chain =========
    // I0 commits: frees old_p. I1 commits: frees pdst0. Count returns to
    // n0-1 (pdst1 stays live as the architectural mapping).
    commit_one(5'd5, save_pdst0, p_old5);
    check("BUNDLE.1: old_p freed exactly once", u_fl.free_bits_q[p_old5] == 1'b1);
    commit_one(5'd5, save_pdst1, save_pdst0);
    check("BUNDLE.1: pdst0 freed by slot-1's commit",
          u_fl.free_bits_q[save_pdst0] == 1'b1);
    check("BUNDLE.1: pdst1 still live (the mapping)",
          u_fl.free_bits_q[save_pdst1] == 1'b0);
    check("BUNDLE.1: count conserved (pdst1 -1, old_p +1 => net n0)",
          free_count_now() == n0);
    check("BUNDLE.1: committed map points at pdst1",
          u_rn.committed_map_q[5] == save_pdst1);

    // ====== the SAME WAW chain retiring as ONE pair ======
    // The preceding case retires the chain in two beats;
    // here both positions retire on a single edge. The aliasing is the point:
    // position 1's stale IS
    // slot 0's pdst, so program order decides whether that preg ends free
    // (correct -- slot 1 killed it) or leaks forever (slot 1 applied first).
    p_old5 = u_rn.spec_map_q[5];
    n0     = free_count_now();
    bundle(5'd1, 5'd2, 5'd5, 1'b1,  5'd3, 5'd4, 5'd5, 1'b1,
           prs1, prs2, pdst, stale);
    save_pdst0 = pdst[0]; save_pdst1 = pdst[1];
    check("BUNDLE.1b: slot-1 stale aliases slot-0 pdst (the hazard)",
          stale[1] == save_pdst0);

    commit_pair(5'd5, save_pdst0, stale[0],
                5'd5, save_pdst1, stale[1]);

    check("BUNDLE.1b: committed map owned by slot 1 (younger wins)",
          u_rn.committed_map_q[5] == save_pdst1);
    check("BUNDLE.1b: slot-0 pdst freed by slot-1's stale return",
          u_fl.free_bits_q[save_pdst0] == 1'b1);
    check("BUNDLE.1b: slot-1 pdst live (it IS the mapping)",
          u_fl.free_bits_q[save_pdst1] == 1'b0);
    check("BUNDLE.1b: prior mapping freed by slot-0's stale return",
          u_fl.free_bits_q[p_old5] == 1'b1);
    check("BUNDLE.1b: pool conserved over the pair (2 out, 2 back)",
          free_count_now() == n0);

    // Distinct destinations in one pair: both map writes must land, and
    // neither stale return may be lost to the other slot's write.
    n0 = free_count_now();
    bundle(5'd1, 5'd2, 5'd8, 1'b1,  5'd3, 5'd4, 5'd9, 1'b1,
           prs1, prs2, pdst, stale);
    commit_pair(5'd8, pdst[0], stale[0],  5'd9, pdst[1], stale[1]);
    check("BUNDLE.1b: distinct-rd pair, slot-0 map applied",
          u_rn.committed_map_q[8] == pdst[0]);
    check("BUNDLE.1b: distinct-rd pair, slot-1 map applied",
          u_rn.committed_map_q[9] == pdst[1]);
    check("BUNDLE.1b: distinct-rd pair, both stales returned",
          u_fl.free_bits_q[stale[0]] && u_fl.free_bits_q[stale[1]]);

    // =========== trap flush restores both modules ===========
    n0 = free_count_now();
    bundle(5'd1, 5'd2, 5'd20, 1'b1,  5'd3, 5'd4, 5'd21, 1'b1,
           prs1, prs2, pdst, stale);
    check("BUNDLE.2: speculative damage present", free_count_now() == n0 - 2);
    @(negedge clk); trap_flush = 1'b1;
    @(posedge clk); @(negedge clk); trap_flush = 1'b0;
    check("BUNDLE.2: free list restored to committed image",
          u_fl.free_bits_q == u_fl.committed_free_bits_q);
    check("BUNDLE.2: the flushed bundle's pregs returned to the pool",
          u_fl.free_bits_q[pdst[0]] && u_fl.free_bits_q[pdst[1]]);
    check("BUNDLE.2: spec map restored (x5 back to committed)",
          u_rn.spec_map_q[5] == u_rn.committed_map_q[5]);

    // ========== slot-1-only allocation through the bundle =========
    n0 = free_count_now();
    bundle(5'd1, 5'd2, 5'd0, 1'b0,  5'd3, 5'd4, 5'd22, 1'b1,
           prs1, prs2, pdst, stale);
    check("A3.3: slot-0 pdst empty, slot-1 granted",
          (pdst[0] == '0) && (pdst[1] != '0));
    check("A3.3: exactly one preg consumed", free_count_now() == n0 - 1);
    check("A3.3: map updated for slot 1 only",
          u_rn.spec_map_q[22] == pdst[1]);

    // ============ rd-less bundle fires, consumes nothing ============
    n0 = free_count_now();
    bundle(5'd1, 5'd2, 5'd0, 1'b0,  5'd3, 5'd4, 5'd0, 1'b0,
           prs1, prs2, pdst, stale);
    check("A3.4: nothing consumed", free_count_now() == n0);

    // ============ slot-0 branch — Mpre map + seed INCLUDES pdst1 ====
    // Branch in slot 0 (no rd), slot 1 allocates x23. the
    // checkpoint map must be the PRE-BUNDLE map (x23 -> old), and the
    // alloc list must be seeded with pdst1 (younger than the branch).
    check("A4.1: no checkpoint outstanding at entry",
          u_rn.checkpoint_valid_q[0] == 1'b0);
    p_old23 = u_rn.spec_map_q[23];
    p_old24 = u_rn.spec_map_q[24];
    p_old25 = u_rn.spec_map_q[25];
    n_pre = free_count_now();
    bundle_ck(5'd1, 5'd2, 5'd0, 1'b0,  5'd3, 5'd4, 5'd23, 1'b1,
              2'b01, pdst, ck_v, ck_id);
    pd1_a41 = pdst[1];
    check("A4.1: checkpoint granted at id 0", ck_v && (ck_id == '0));
    check("A4.1: checkpoint map is Mpre (x23 -> OLD mapping, not pdst1)",
          u_rn.checkpoint_map_q[0][23] == p_old23);
    check("A4.1: seed includes pdst1", u_rn.checkpoint_alloc_list_q[0][pd1_a41]);
    check("A4.1: seed contains ONLY pdst1", ck0_list_count() == 1);
    check("A4.1: live map advanced past the checkpoint",
          u_rn.spec_map_q[23] == pd1_a41);

    // ============ recovery through the slot-0-branch checkpoint =====
    // Younger wrong-path damage after the branch, then recover. The seed
    // is what returns pdst1 — without it pdst1 leaks forever.
    bundle(5'd1, 5'd2, 5'd24, 1'b1,  5'd3, 5'd4, 5'd25, 1'b1,
           prs1, prs2, pdst, stale);
    py0 = pdst[0]; py1 = pdst[1];
    check("A4.2: younger allocs marked into the open checkpoint",
          u_rn.checkpoint_alloc_list_q[0][py0] &&
          u_rn.checkpoint_alloc_list_q[0][py1] && (ck0_list_count() == 3));
    recover_beat('0);
    check("A4.2: map restored to Mpre (x23 back to old)",
          u_rn.spec_map_q[23] == p_old23);
    check("A4.2: younger map writes rolled back",
          (u_rn.spec_map_q[24] == p_old24) && (u_rn.spec_map_q[25] == p_old25));
    check("A4.2: pdst1 RETURNED by the seed (the leak check)",
          u_fl.free_bits_q[pd1_a41] == 1'b1);
    check("A4.2: younger allocs returned",
          u_fl.free_bits_q[py0] && u_fl.free_bits_q[py1]);
    check("A4.2: conservation — count back to the pre-branch sample",
          free_count_now() == n_pre);
    check("A4.2: checkpoint retired by recovery",
          u_rn.checkpoint_valid_q[0] == 1'b0);

    // ============ slot-1 branch — Mpost0 map + seed EXCLUDES pdst0 ==
    // Slot 0 allocates x26 (older than the branch), branch in slot 1.
    // checkpoint map = Mpost0 (INCLUDES slot 0's write); the
    // alloc list must NOT contain pdst0 (older -> double-free hazard).
    p_old26 = u_rn.spec_map_q[26];
    n_pre2 = free_count_now();
    bundle_ck(5'd1, 5'd2, 5'd26, 1'b1,  5'd3, 5'd4, 5'd0, 1'b0,
              2'b10, pdst, ck_v, ck_id);
    pd0_a43 = pdst[0];
    check("A4.3: checkpoint granted at id 0", ck_v && (ck_id == '0));
    check("A4.3: checkpoint map is Mpost0 (x26 -> pdst0 overlay)",
          u_rn.checkpoint_map_q[0][26] == pd0_a43);
    check("A4.3: seed excludes pdst0 (empty list)", ck0_list_count() == 0);

    // ============ recovery through the slot-1-branch checkpoint =====
    // pdst0 must SURVIVE recovery (its instruction is older than the
    // branch): map keeps x26 -> pdst0 and the free list must NOT reclaim it.
    bundle(5'd1, 5'd2, 5'd27, 1'b1,  5'd3, 5'd4, 5'd0, 1'b0,
           prs1, prs2, pdst, stale);
    py2 = pdst[0];
    recover_beat('0);
    check("A4.4: restored map PRESERVES the older write (x26 -> pdst0)",
          u_rn.spec_map_q[26] == pd0_a43);
    check("A4.4: pdst0 NOT reclaimed (the double-free guard)",
          u_fl.free_bits_q[pd0_a43] == 1'b0);
    check("A4.4: younger alloc returned", u_fl.free_bits_q[py2] == 1'b1);
    check("A4.4: conservation — exactly pdst0 still held out",
          free_count_now() == n_pre2 - 1);
    commit_one(5'd26, pd0_a43, p_old26);
    check("A4.4: surviving instr commits — old mapping freed, net zero",
          (free_count_now() == n_pre2) && u_fl.free_bits_q[p_old26]);
    check("A4.4: committed map owns pdst0", u_rn.committed_map_q[26] == pd0_a43);

    // ============ correct-path resolve frees the checkpoint only ====
    bundle_ck(5'd1, 5'd2, 5'd0, 1'b0,  5'd3, 5'd4, 5'd0, 1'b0,
              2'b01, pdst, ck_v, ck_id);
    check("A4.5: checkpoint granted at id 0", ck_v && (ck_id == '0));
    n_pre = free_count_now();
    release_beat('0);
    check("A4.5: resolve retires the checkpoint",
          u_rn.checkpoint_valid_q[0] == 1'b0);
    check("A4.5: resolve moves no pregs", free_count_now() == n_pre);
    check("A4.5: resolve leaves the live map alone",
          u_rn.spec_map_q[26] == pd0_a43);

    // ============ one-instruction bundle neutralizes slot-1 junk ===
    // Shape 2'b01 with GARBAGE slot-1 payload (we=1, rd=x9): the ratified
    // needs_alloc[i] qualification must make the junk invisible.
    p_old5 = u_rn.spec_map_q[9];
    n_pre = free_count_now();
    bundle_one(5'd1, 5'd2, 5'd28, 1'b1,  5'd9, 1'b1, pdst, need_s);
    check("A5b.1: slot 0 granted, junk slot 1 empty",
          (pdst[0] != '0) && (pdst[1] == '0));
    check("A5b.1: demand mirrors the bundle shape", need_s == 2'b01);
    check("A5b.1: junk slot-1 payload allocated nothing (map untouched)",
          u_rn.spec_map_q[9] == p_old5);
    check("A5b.1: exactly one preg consumed", free_count_now() == n_pre - 1);
    check("A5b.1: slot-0 map write landed", u_rn.spec_map_q[28] == pdst[0]);

    // ============ pure demand — visible while blocked, pops nothing
    // no ready/checkpoint/fire term in preg_slot_need. Block the
    // downstream (rename_ready=0), present a two-alloc bundle: demand must
    // SHOW while nothing pops; releasing ready fires the held bundle.
    n_pre = free_count_now();
    p_old23 = u_rn.spec_map_q[29];
    p_old24 = u_rn.spec_map_q[30];
    @(negedge clk);
    rename_ready = 1'b0;
    decoded_valid = 1'b1; decoded_slot_valid = 2'b11;
    decoded_rs1 = {5'd2, 5'd1}; decoded_rs2 = {5'd4, 5'd3};
    decoded_rd  = {5'd30, 5'd29}; decoded_rd_we = 2'b11;
    #1;
    check("A5b.2: demand visible while backend blocked",
          (preg_slot_need == 2'b11) && !decoded_ready);
    repeat (2) @(negedge clk);
    check("A5b.2: held bundle popped nothing", free_count_now() == n_pre);
    rename_ready = 1'b1;
    @(posedge clk); @(negedge clk);
    decoded_valid = 1'b0; decoded_slot_valid = 2'b00; decoded_rd_we = '0;
    check("A5b.2: release fires the held bundle (both allocs land)",
          (free_count_now() == n_pre - 2) &&
          (u_rn.spec_map_q[29] != p_old23) && (u_rn.spec_map_q[30] != p_old24) &&
          (u_fl.free_bits_q[u_rn.spec_map_q[29]] == 1'b0) &&
          (u_fl.free_bits_q[u_rn.spec_map_q[30]] == 1'b0));

    if (errors == 0) begin
      $display("[tb_rv32i_ss_rename] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_rename] FAIL errors=%0d checks=%0d",
             errors, checks);
    end
  end

endmodule
