// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// tb_rv32i_ss_iq_diff — constrained-random differential of the two-scan IQ
// selector against an independent behavioral oracle (60,000 cycles).
// Coverage floors require dual grants, solo skips, singleton-FU blocking,
// age inversions, recovery kills, trap flushes and ALU1 bindings.
//
// Oracle rules:
//   - Two sequential full scans select by ROB ring age and bind an exact FU
//     instance; physical IQ position does not define instruction age.
//   - Jump/CSR instructions issue alone on ALU0. A solo first winner prevents
//     a second grant, and the second scan skips solo candidates.
//   - Eligibility requires a valid entry, ready register operands and a free
//     instance of the required FU. Capacity is decremented after scan 0.
//   - Store rs2 is payload captured by the SQ, not an AGEN source; store
//     src2_sel is IMM.
//
// The oracle is derived from these behavioral rules, independently of the DUT
// implementation. The DUT tripwires remain enabled for the whole run.
// Stimulus fires a bundle only when alloc_ready covers its shape, suppresses
// issue on recovery/flush, keeps the head behind live entries, and never gives
// two live entries the same rob_idx.

module tb_rv32i_ss_iq_diff;
  import rv32i_ss_pkg::*;
  import rv32i_pipeline_pkg::csr_op_e;

  localparam time CLK_PERIOD = 10ns;

  logic clk;
  logic rst_n;
  logic trap_flush;
  logic branch_recover_req;
  rob_idx_t recover_rob_idx;

  logic [1:0]   alloc_slot_valid;
  logic         bundle_fire_drv;
  iq_entry_t [1:0] iq_alloc_entry;
  logic         alloc_ready;
  logic [OOO_PHYS_REGS-1:0] ready_vec;
  rob_idx_t     rob_head_idx;

  logic        [1:0] issue_valid;
  iq_entry_t   [1:0] issue_entry;
  issue_unit_e [1:0] issue_unit;
  logic              issue_accept;
  logic              alu0_fu_ready;
  logic              alu1_fu_ready;
  logic              muldiv_fu_ready;
  logic              lsu_fu_ready;

  logic [1:0] tb_size;
  assign tb_size = {1'b0, alloc_slot_valid[0]} + {1'b0, alloc_slot_valid[1]};

  int errors = 0;
  int checks = 0;

  rv32i_ss_iq dut (
    .clk               (clk),
    .rst_n             (rst_n),
    .trap_flush        (trap_flush),
    .branch_recover_req(branch_recover_req),
    .recover_rob_idx   (recover_rob_idx),
    .bundle_size       (tb_size),
    .iq_alloc_slot_valid (alloc_slot_valid),
    .bundle_fire       (bundle_fire_drv),
    .iq_alloc_entry    (iq_alloc_entry),
    .iq_alloc_ready    (alloc_ready),
    .ready_vec         (ready_vec),
    .rob_head_idx      (rob_head_idx),
    .issue_valid       (issue_valid),
    .issue_entry       (issue_entry),
    .issue_unit        (issue_unit),
    .issue_accept      (issue_accept),
    .alu0_fu_ready     (alu0_fu_ready),
    .alu1_fu_ready     (alu1_fu_ready),
    .muldiv_fu_ready   (muldiv_fu_ready),
    .lsu_fu_ready      (lsu_fu_ready)
  );

  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;

  initial begin
    repeat (200000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_iq_diff exceeded 200000 cycles");
  end

  // ---------------- model state (stimulus bookkeeping) ----------------
  localparam int MAXL = 24;
  int        m_n;                       // live entries
  rob_idx_t  m_idx   [MAXL];
  rob_seq_t  m_seq   [MAXL];
  ooo_op_class_e m_opc [MAXL];
  ooo_fu_class_e m_fuc [MAXL];
  csr_op_e   m_csr   [MAXL];
  ooo_src_sel_e m_s1 [MAXL];
  ooo_src_sel_e m_s2 [MAXL];
  phys_reg_t m_prs1  [MAXL];
  phys_reg_t m_prs2  [MAXL];

  int unsigned next_seq;
  int unsigned alloc_ctr;               // rob_idx allocator (mod 32, in-window)
  int seed;

  // ---------------- coverage ----------------
  int cyc_n, n_dual, n_solo_grant, n_scan1_skipsolo, n_singlefu_block;
  int n_inversion, n_recovery, n_flush, n_fullstall, n_alu1_bind, n_grant;

  function automatic int unsigned age_of(input rob_idx_t idx);
    age_of = (int'(idx) - int'(rob_head_idx)) & 31;
  endfunction

  function automatic bit m_solo(input int i);
    m_solo = (m_opc[i] == OOO_OP_JUMP) || (m_csr[i] != rv32i_pipeline_pkg::CSR_NONE);
  endfunction

  function automatic bit m_elig(input int i);
    bit s1r, s2r;
    s1r = (m_s1[i] != OOO_SRC_REG) || ready_vec[m_prs1[i]];
    s2r = (m_s2[i] != OOO_SRC_REG) || ready_vec[m_prs2[i]];
    m_elig = s1r && s2r;
  endfunction

  // contract binding law against a shadow-capacity vector.
  // Returns {bindable, unit[1:0]} packed (Icarus: no function output ports).
  function automatic logic [2:0] m_bind(input int i, input bit sh_a0, input bit sh_a1,
                                        input bit sh_md, input bit sh_ag);
    m_bind = {1'b0, 2'(ISSUE_UNIT_ALU0)};
    case (m_fuc[i])
      OOO_FU_ALU: begin
        if (sh_a0) m_bind = {1'b1, 2'(ISSUE_UNIT_ALU0)};
        else if (!m_solo(i) && sh_a1) m_bind = {1'b1, 2'(ISSUE_UNIT_ALU1)};
      end
      OOO_FU_MULDIV: if (sh_md) m_bind = {1'b1, 2'(ISSUE_UNIT_MULDIV)};
      OOO_FU_LSU:    if (sh_ag) m_bind = {1'b1, 2'(ISSUE_UNIT_AGEN)};
      default: ;
    endcase
  endfunction

  // full two-scan oracle (module-level outputs; Icarus: no unpacked array ports)
  logic [1:0]  o_hit;
  int          o_win0, o_win1;
  issue_unit_e o_un0, o_un1;
  task automatic oracle();
    bit sh_a0, sh_a1, sh_md, sh_ag;
    int unsigned best_age;
    logic [2:0] cand;
    bit solo0;
    o_hit = '0; o_win0 = -1; o_win1 = -1; o_un0 = ISSUE_UNIT_ALU0; o_un1 = ISSUE_UNIT_ALU0;
    sh_a0 = alu0_fu_ready; sh_a1 = alu1_fu_ready;
    sh_md = muldiv_fu_ready; sh_ag = lsu_fu_ready;
    best_age = 0; solo0 = 0;
    for (int i = 0; i < m_n; i++) begin
      cand = m_bind(i, sh_a0, sh_a1, sh_md, sh_ag);
      if (m_elig(i) && cand[2]) begin
        if ((o_win0 == -1) || (age_of(m_idx[i]) < best_age)) begin
          o_win0 = i; o_un0 = issue_unit_e'(cand[1:0]); best_age = age_of(m_idx[i]);
        end
      end
    end
    if (o_win0 != -1) begin
      o_hit[0] = 1'b1;
      solo0 = m_solo(o_win0);
      case (o_un0)
        ISSUE_UNIT_ALU0:   sh_a0 = 1'b0;
        ISSUE_UNIT_ALU1:   sh_a1 = 1'b0;
        ISSUE_UNIT_MULDIV: sh_md = 1'b0;
        ISSUE_UNIT_AGEN:   sh_ag = 1'b0;
        default: ;
      endcase
      if (!solo0) begin
        best_age = 0;
        for (int i = 0; i < m_n; i++) begin
          cand = m_bind(i, sh_a0, sh_a1, sh_md, sh_ag);
          if ((i != o_win0) && !m_solo(i) && m_elig(i) && cand[2]) begin
            if ((o_win1 == -1) || (age_of(m_idx[i]) < best_age)) begin
              o_win1 = i; o_un1 = issue_unit_e'(cand[1:0]); best_age = age_of(m_idx[i]);
            end
          end
        end
        if (o_win1 != -1) o_hit[1] = 1'b1;
      end
    end
  endtask

  function automatic int owin(input int p);
    owin = (p == 0) ? o_win0 : o_win1;
  endfunction
  function automatic issue_unit_e oun(input int p);
    oun = (p == 0) ? o_un0 : o_un1;
  endfunction

  task automatic fail(input string nm);
    errors++;
    $error("[%s] cyc=%0d", nm, cyc_n);
  endtask

  // is candidate rob_idx already live in the model?
  function automatic bit idx_live(input int unsigned ci);
    idx_live = 1'b0;
    for (int i = 0; i < m_n; i++)
      if (int'(m_idx[i]) == (ci & 31)) idx_live = 1'b1;
  endfunction

  task automatic do_cycle();
    bit [1:0] exp_hit;
    logic [2:0] cand3;
    int rw0, rw1;
    bit do_flush, do_rec;
    int allocs;
    iq_entry_t e;
    bit ev_skip_solo, ev_block, ev_inv;
    int minage, adv;
    begin
      // ---- random event selection ----
      do_flush = ($urandom(seed) % 400) == 0;
      do_rec   = !do_flush && (($urandom(seed) % 60) == 0) && (m_n > 0);

      trap_flush         = do_flush;
      branch_recover_req = do_rec;
      recover_rob_idx    = do_rec ? m_idx[$urandom(seed) % m_n] : '0;

      // FU shadows: bias ready, with structured low periods
      alu0_fu_ready   = ($urandom(seed) % 100) < 82;
      alu1_fu_ready   = ($urandom(seed) % 100) < 82;
      muldiv_fu_ready = ($urandom(seed) % 100) < 70;
      lsu_fu_ready    = ($urandom(seed) % 100) < 70;
      if (($urandom(seed) % 12) == 0) alu0_fu_ready = 1'b0; // ALU1-binding window

      // ready_vec: random walk, biased ready; bit0 always ready (x0)
      for (int b = 1; b < OOO_PHYS_REGS; b++) begin
        if (($urandom(seed) % 100) < 30) ready_vec[b] = ($urandom(seed) % 100) < 70;
      end
      ready_vec[0] = 1'b1;

      issue_accept = !(do_flush || do_rec) && (($urandom(seed) % 100) < 85);

      // ---- alloc candidate build ----
      alloc_slot_valid = 2'b00;
      iq_alloc_entry[0] = '0;
      iq_alloc_entry[1] = '0;
      allocs = 0;
      if (!(do_flush || do_rec) && (m_n < MAXL - 3)) begin
        allocs = (($urandom(seed) % 100) < 55) ? (1 + (($urandom(seed) % 100) < 45)) : 0;
        // in-window guard (new idxs near head side) + no duplicate live idx
        if (allocs > 0 && idx_live(alloc_ctr)) allocs = 0;
        if (allocs > 1 && idx_live(alloc_ctr + 1)) allocs = 1;
        for (int a = 0; a < allocs; a++) begin
          e = '0;
          e.rob_idx  = rob_idx_t'(alloc_ctr + a);
          e.rob_seq  = rob_seq_t'(next_seq + a);
          e.rd_wen   = 1'b1;
          // class mix: ~6% jump-solo, ~6% csr-solo, ~15% muldiv, ~12% lsu
          case ($urandom(seed) % 100)
            0,1,2,3,4,5: begin
              e.op_class = OOO_OP_JUMP;
              e.fu_class = OOO_FU_ALU;
            end
            6,7,8,9,10,11: begin
              e.op_class = OOO_OP_ALU;
              e.fu_class = OOO_FU_ALU;
              e.csr_op   = rv32i_pipeline_pkg::CSR_RW;
            end
            12,13,14,15,16,17,18,19,20,21,22,23,24,25,26: begin
              e.op_class = OOO_OP_ALU;
              e.fu_class = OOO_FU_MULDIV;
            end
            27,28,29,30,31,32,33,34,35,36,37,38: begin
              e.op_class = OOO_OP_ALU;
              e.fu_class = OOO_FU_LSU;
            end
            default: begin
              e.op_class = OOO_OP_ALU;
              e.fu_class = OOO_FU_ALU;
            end
          endcase
          e.is_store = (e.fu_class == OOO_FU_LSU) && (($urandom(seed) % 100) < 40);
          e.prs1 = phys_reg_t'(32 + ($urandom(seed) % 8));
          e.prs2 = phys_reg_t'(32 + ($urandom(seed) % 8));
          e.src1_sel = ooo_src_sel_e'((($urandom(seed) % 100) < 75) ? OOO_SRC_REG : OOO_SRC_IMM);
          e.src2_sel = ooo_src_sel_e'((($urandom(seed) % 100) < 75) ? OOO_SRC_REG : OOO_SRC_IMM);
          alloc_slot_valid[a] = 1'b1;
          iq_alloc_entry[a]   = e;
        end
      end

      #1;  // settle: alloc_ready now reflects the offered shape
      // honesty: degrade the offer to what alloc_ready covers
      if ((alloc_slot_valid == 2'b11) && !alloc_ready) alloc_slot_valid = 2'b01;
      else if ((alloc_slot_valid == 2'b01) && !alloc_ready) alloc_slot_valid = 2'b00;
      if ((allocs > 0) && (alloc_slot_valid == 2'b00)) n_fullstall++;
      #1;
      bundle_fire_drv = (|alloc_slot_valid) && alloc_ready;

      // ---- oracle + compare (select sees only pre-edge state) ----
      oracle();
      exp_hit = o_hit;

      checks++;
      if (issue_valid !== exp_hit) begin
        fail($sformatf("issue_valid got=%b exp=%b", issue_valid, exp_hit));
        $display("  a0r=%0b a1r=%0b mdr=%0b agr=%0b m_n=%0d", alu0_fu_ready,
                 alu1_fu_ready, muldiv_fu_ready, lsu_fu_ready, m_n);
      end
      for (int p = 0; p < 2; p++) begin
        if (exp_hit[p]) begin
          checks++;
          if ((issue_entry[p].rob_seq !== m_seq[owin(p)]) ||
              (issue_entry[p].rob_idx !== m_idx[owin(p)])) begin
            fail($sformatf("grant[%0d] identity got={idx=%0d seq=%0d} exp={idx=%0d seq=%0d}",
                 p, issue_entry[p].rob_idx, issue_entry[p].rob_seq,
                 m_idx[owin(p)], m_seq[owin(p)]));
          end
          checks++;
          if (issue_unit[p] !== oun(p))
            fail($sformatf("grant[%0d] unit got=%0d exp=%0d", p, issue_unit[p], oun(p)));
          checks++;
          if (!m_elig(owin(p))) fail($sformatf("grant[%0d] of an ineligible entry", p));
        end
      end
      if (exp_hit == 2'b11) begin
        checks++;
        if (!(age_of(issue_entry[0].rob_idx) < age_of(issue_entry[1].rob_idx)))
          fail("grant age order inverted");
        n_dual++;
      end
      checks++;
      if ($isunknown(issue_valid)) fail("issue_valid X");
      if (exp_hit[0] && m_solo(o_win0)) n_solo_grant++;

      // ---- coverage events (pre-edge model state) ----
      ev_skip_solo = 0; ev_block = 0; ev_inv = 0;
      if (exp_hit[0] && !m_solo(o_win0)) begin
        for (int i = 0; i < m_n; i++)
          if ((i != o_win0) && m_solo(i) && m_elig(i)) ev_skip_solo = 1;
      end
      if (exp_hit[0] && !exp_hit[1]) begin
        for (int i = 0; i < m_n; i++) begin
          if ((i != o_win0) && m_elig(i) && !m_solo(i)) begin
            if ((m_fuc[i] == OOO_FU_MULDIV) && (o_un0 == ISSUE_UNIT_MULDIV)) ev_block = 1;
            if ((m_fuc[i] == OOO_FU_LSU)    && (o_un0 == ISSUE_UNIT_AGEN))   ev_block = 1;
          end
        end
      end
      if (exp_hit[0]) begin
        for (int i = 0; i < m_n; i++) begin
          cand3 = m_bind(i, alu0_fu_ready, alu1_fu_ready, muldiv_fu_ready,
                         lsu_fu_ready);
          if (m_elig(i) && (age_of(m_idx[i]) < age_of(m_idx[o_win0])) &&
              !cand3[2]) ev_inv = 1;
        end
      end
      if (ev_skip_solo) n_scan1_skipsolo++;
      if (ev_block)     n_singlefu_block++;
      if (ev_inv)       n_inversion++;
      if (do_rec)       n_recovery++;
      if (do_flush)     n_flush++;
      if (exp_hit[0] && (o_un0 == ISSUE_UNIT_ALU1)) n_alu1_bind++;
      if (exp_hit[1] && (o_un1 == ISSUE_UNIT_ALU1)) n_alu1_bind++;
      if (|exp_hit) n_grant++;

      @(posedge clk);
      // ---- model state update at the edge ----
      cyc_n++;
      if (do_flush) begin
        m_n = 0;
      end else if (do_rec) begin
        begin : rec_compact
          int wi;
          wi = 0;
          for (int i = 0; i < m_n; i++) begin
            if (age_of(m_idx[i]) <= age_of(recover_rob_idx)) begin
              m_idx[wi] = m_idx[i]; m_seq[wi] = m_seq[i]; m_opc[wi] = m_opc[i];
              m_fuc[wi] = m_fuc[i]; m_csr[wi] = m_csr[i]; m_s1[wi] = m_s1[i];
              m_s2[wi] = m_s2[i]; m_prs1[wi] = m_prs1[i]; m_prs2[wi] = m_prs2[i];
              wi++;
            end
          end
          m_n = wi;
        end
      end else begin
        // removals: fired grants (fire = issue_accept && issue_valid);
        // unrolled with mutable local indices (completing the module-scope
        // rw1 compacts after rw0's removal).
        rw0 = o_win0; rw1 = o_win1;
        if (issue_accept && exp_hit[0] && (rw0 >= 0)) begin
          for (int i = rw0; i < m_n - 1; i++) begin
            m_idx[i] = m_idx[i+1]; m_seq[i] = m_seq[i+1]; m_opc[i] = m_opc[i+1];
            m_fuc[i] = m_fuc[i+1]; m_csr[i] = m_csr[i+1]; m_s1[i] = m_s1[i+1];
            m_s2[i] = m_s2[i+1]; m_prs1[i] = m_prs1[i+1]; m_prs2[i] = m_prs2[i+1];
          end
          m_n--;
          if (exp_hit[1] && (rw1 > rw0)) rw1--;
        end
        if (issue_accept && exp_hit[1] && (rw1 >= 0)) begin
          for (int i = rw1; i < m_n - 1; i++) begin
            m_idx[i] = m_idx[i+1]; m_seq[i] = m_seq[i+1]; m_opc[i] = m_opc[i+1];
            m_fuc[i] = m_fuc[i+1]; m_csr[i] = m_csr[i+1]; m_s1[i] = m_s1[i+1];
            m_s2[i] = m_s2[i+1]; m_prs1[i] = m_prs1[i+1]; m_prs2[i] = m_prs2[i+1];
          end
          m_n--;
        end
        // additions: accepted allocs
        if (bundle_fire_drv) begin
          for (int a = 0; a < 2; a++) begin
            if (alloc_slot_valid[a]) begin
              m_idx[m_n]   = iq_alloc_entry[a].rob_idx;
              m_seq[m_n]   = iq_alloc_entry[a].rob_seq;
              m_opc[m_n]   = iq_alloc_entry[a].op_class;
              m_fuc[m_n]   = iq_alloc_entry[a].fu_class;
              m_csr[m_n]   = iq_alloc_entry[a].csr_op;
              m_s1[m_n]    = iq_alloc_entry[a].src1_sel;
              m_s2[m_n]    = iq_alloc_entry[a].src2_sel;
              m_prs1[m_n]  = iq_alloc_entry[a].prs1;
              m_prs2[m_n]  = iq_alloc_entry[a].prs2;
              m_n++;
            end
          end
          alloc_ctr += ((alloc_slot_valid == 2'b11) ? 2 :
                        (alloc_slot_valid == 2'b01) ? 1 : 0);
          next_seq  += ((alloc_slot_valid == 2'b11) ? 2 :
                        (alloc_slot_valid == 2'b01) ? 1 : 0);
        end
      end
      // head advance: 0-2, never past the oldest live entry (the real ROB
      // cannot commit an unissued entry either)
      minage = 30;
      for (int i = 0; i < m_n; i++)
        if (int'(age_of(m_idx[i])) < minage) minage = int'(age_of(m_idx[i]));
      adv = 0;
      if (($urandom(seed) % 100) < 50) adv = 1 + ((($urandom(seed) % 100)) < 30);
      if (adv > minage) adv = minage;
      rob_head_idx = rob_idx_t'(int'(rob_head_idx) + adv);
      @(negedge clk);
    end
  endtask

  initial begin
    if (!$value$plusargs("seed=%d", seed)) seed = 1;
    seed = 32'h0136_0001 + seed;
    cyc_n = 0; n_dual = 0; n_solo_grant = 0; n_scan1_skipsolo = 0;
    n_singlefu_block = 0; n_inversion = 0; n_recovery = 0; n_flush = 0;
    n_fullstall = 0; n_alu1_bind = 0; n_grant = 0;
    m_n = 0; next_seq = 0; alloc_ctr = 0;
    rob_head_idx = '0;
    trap_flush = 1'b0; branch_recover_req = 1'b0; recover_rob_idx = '0;
    alloc_slot_valid = 2'b00; bundle_fire_drv = 1'b0;
    iq_alloc_entry[0] = '0; iq_alloc_entry[1] = '0;
    issue_accept = 1'b0;
    alu0_fu_ready = 1'b0; alu1_fu_ready = 1'b0;
    muldiv_fu_ready = 1'b0; lsu_fu_ready = 1'b0;
    for (int b = 0; b < OOO_PHYS_REGS; b++) ready_vec[b] = (b < OOO_ARCH_REGS);

    rst_n = 1'b0;
    repeat (4) @(posedge clk);
    @(negedge clk) rst_n = 1'b1;
    @(negedge clk);

    repeat (60000) do_cycle();

    $display("T36STATS iq_diff cycles=%0d grants=%0d dual=%0d solo_grant=%0d scan1_skipsolo=%0d singlefu_block=%0d inversion=%0d recovery=%0d flush=%0d fullstall=%0d alu1_bind=%0d",
             cyc_n, n_grant, n_dual, n_solo_grant, n_scan1_skipsolo,
             n_singlefu_block, n_inversion, n_recovery, n_flush, n_fullstall, n_alu1_bind);

    // provably-entered gates
    if (n_dual < 100)            $fatal(1, "iq_diff: dual-grant family under-entered (%0d)", n_dual);
    if (n_solo_grant < 50)       $fatal(1, "iq_diff: solo-grant family under-entered (%0d)", n_solo_grant);
    if (n_scan1_skipsolo < 20)   $fatal(1, "iq_diff: scan1-skip-solo under-entered (%0d)", n_scan1_skipsolo);
    if (n_singlefu_block < 50)   $fatal(1, "iq_diff: single-FU block under-entered (%0d)", n_singlefu_block);
    if (n_inversion < 20)        $fatal(1, "iq_diff: oldest-inversion under-entered (%0d)", n_inversion);
    if (n_recovery < 30)         $fatal(1, "iq_diff: recovery-kill under-entered (%0d)", n_recovery);
    if (n_flush < 5)             $fatal(1, "iq_diff: trap-flush under-entered (%0d)", n_flush);
    if (n_alu1_bind < 100)       $fatal(1, "iq_diff: ALU1 binding under-entered (%0d)", n_alu1_bind);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_iq_diff] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "tb_rv32i_ss_iq_diff FAILED errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
