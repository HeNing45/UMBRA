// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// Free-list unit battery: per-slot demand (preg_slot_need), need-aware
// preg_avail, slot-indexed preg_alloc_reg[1:0], and consumption strictly on
// bundle_fire. Covers empty/full boundaries, stale-register reclamation,
// ordered dual retirement, recovery rollback and committed-image restoration.

module tb_rv32i_ss_free_list;

  import rv32i_ss_pkg::*;

  localparam int CLK_PERIOD = 10;

  logic clk;
  logic rst_n;

  logic [1:0]      preg_slot_need;
  logic            preg_avail;
  phys_reg_t [1:0] preg_alloc_reg;
  logic            bundle_fire;

  // per-position commit drive -- this TB stands in for the core's
  // sole fire producer so every reclamation orientation is directly reachable.
  logic [1:0]       commit_fire;
  logic [1:0]       commit_rd_we;
  phys_reg_t [1:0]  commit_pdst;
  phys_reg_t [1:0]  commit_stale_pdst;

  logic                     branch_recover_req;
  logic [OOO_PHYS_REGS-1:0] rename_branch_recover_alloc_list;
  logic                     trap_flush;

  integer checks = 0;
  integer errors = 0;

  rv32i_ss_free_list dut (
    .clk                       (clk),
    .rst_n                     (rst_n),
    .preg_slot_need            (preg_slot_need),
    .preg_avail                (preg_avail),
    .preg_alloc_reg            (preg_alloc_reg),
    .bundle_fire               (bundle_fire),
    .commit_fire               (commit_fire),
    .commit_rd_we              (commit_rd_we),
    .commit_pdst               (commit_pdst),
    .commit_stale_pdst         (commit_stale_pdst),
    .branch_recover_req        (branch_recover_req),
    .rename_branch_recover_alloc_list (rename_branch_recover_alloc_list),
    .trap_flush                (trap_flush)
  );

  initial clk = 1'b0;
  always #(CLK_PERIOD / 2) clk = ~clk;

  initial begin
    #(CLK_PERIOD * 2000);
    $fatal(1, "[tb_rv32i_ss_free_list] WATCHDOG timeout");
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
        if (dut.free_bits_q[i]) c = c + 1;
      free_count_now = c;
    end
  endfunction

  // one fire beat with the given need; grants sampled just before the edge
  task automatic pop_fire(input logic [1:0] need,
                          output phys_reg_t r0, output phys_reg_t r1);
    @(negedge clk);
    preg_slot_need = need;
    bundle_fire    = 1'b1;
    #1;
    r0 = preg_alloc_reg[0];
    r1 = preg_alloc_reg[1];
    @(posedge clk); @(negedge clk);
    preg_slot_need = 2'b00;
    bundle_fire    = 1'b0;
  endtask

  phys_reg_t r0, r1, ra, rb;
  integer    n_before;

  initial begin
    preg_slot_need = 2'b00; bundle_fire = 1'b0;
    commit_fire = 2'b00; commit_rd_we = 2'b00;
    commit_pdst = '0; commit_stale_pdst = '0;
    branch_recover_req = 1'b0; rename_branch_recover_alloc_list = '0;
    trap_flush = 1'b0;
    rst_n = 1'b0;
    repeat (4) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);

    // ============ reset image + need-aware avail ============
    check("A1.1: reset frees PHYS-ARCH regs",
          free_count_now() == (OOO_PHYS_REGS - OOO_ARCH_REGS));
    check("A1.1: p0 not free", dut.free_bits_q[0] == 1'b0);
    preg_slot_need = 2'b00; #1;
    check("A1.1: need=0 -> avail", preg_avail == 1'b1);
    preg_slot_need = 2'b11; #1;
    check("A1.1: need=2 at reset -> avail", preg_avail == 1'b1);
    @(negedge clk); preg_slot_need = 2'b00;

    // ============ slot-indexed binding ============
    pop_fire(2'b01, r0, r1);
    check("A1.2: slot0-only pop grants slot 0",
          (r0 != '0) && (r1 == '0));
    check("A1.2: granted reg now non-free", dut.free_bits_q[r0] == 1'b0);
    pop_fire(2'b10, ra, rb);
    check("A1.2: slot1-only pop grants slot 1 the first free reg",
          (rb != '0) && (ra == '0));
    check("A1.2: slot1 grant non-free", dut.free_bits_q[rb] == 1'b0);

    // ============ dual pop, distinct regs ============
    n_before = free_count_now();
    pop_fire(2'b11, ra, rb);
    check("A1.3: dual pop grants two distinct regs",
          (ra != '0) && (rb != '0) && (ra != rb));
    check("A1.3: both marked non-free",
          (dut.free_bits_q[ra] == 1'b0) && (dut.free_bits_q[rb] == 1'b0));
    check("A1.3: count fell by exactly 2",
          free_count_now() == n_before - 2);

    // ============ a held bundle pops nothing  ============
    n_before = free_count_now();
    @(negedge clk);
    preg_slot_need = 2'b11;   // demand presented, NO fire
    repeat (3) @(negedge clk);
    check("A1.4: demand without fire consumed nothing",
          free_count_now() == n_before);
    check("A1.4: avail combinational under held demand", preg_avail == 1'b1);
    preg_slot_need = 2'b00;

    // ============ rd-less bundle fire pops nothing ============
    n_before = free_count_now();
    pop_fire(2'b00, r0, r1);
    check("A1.5: need=0 fire pops nothing",
          free_count_now() == n_before);

    // ============ almost-empty boundary  ============
    while (free_count_now() > 1) pop_fire(2'b01, r0, r1);
    check("A1.6: drained to exactly one free reg", free_count_now() == 1);
    @(negedge clk); preg_slot_need = 2'b11; #1;
    check("A1.6: need=2 with one free -> NOT avail", preg_avail == 1'b0);
    preg_slot_need = 2'b01; #1;
    check("A1.6: need=1 with one free -> avail (no 1-bundle wedge)",
          preg_avail == 1'b1);
    @(negedge clk); preg_slot_need = 2'b00;
    pop_fire(2'b01, r0, r1);
    check("A1.6: last reg allocatable", r0 != '0);
    @(negedge clk); preg_slot_need = 2'b01; #1;
    check("A1.6: empty -> not avail", preg_avail == 1'b0);
    @(negedge clk); preg_slot_need = 2'b00;

    // ============ commit frees the stale; it becomes allocatable ====
    @(negedge clk);
    commit_fire = 2'b01; commit_rd_we = 2'b01;
    commit_pdst = {6'd0, r0};         // the last allocated reg retires
    commit_stale_pdst = {6'd0, ra};   // frees an earlier alloc
    @(posedge clk); @(negedge clk);
    commit_fire = 2'b00; commit_rd_we = 2'b00;
    check("A1.7: freed stale is free again", dut.free_bits_q[ra] == 1'b1);
    check("A1.7: committed image mirrors the free",
          dut.committed_free_bits_q[ra] == 1'b1);
    pop_fire(2'b01, rb, r1);
    check("A1.7: freed reg is re-allocatable", rb == ra);

    // ============ recovery rollback returns wrong-path allocs =======
    n_before = free_count_now();
    rename_branch_recover_alloc_list = '0;
    rename_branch_recover_alloc_list[rb] = 1'b1;   // pretend rb was wrong-path
    @(negedge clk);
    branch_recover_req = 1'b1;
    @(posedge clk); @(negedge clk);
    branch_recover_req = 1'b0; rename_branch_recover_alloc_list = '0;
    check("A1.8: rollback returned the alloc",
          (dut.free_bits_q[rb] == 1'b1) && (free_count_now() == n_before + 1));

    // ============ trap flush restores the committed image ===========
    // (single pop: only one reg is free here, and a need=2 fire would be
    // an illegal bundle_fire without preg_avail — the tripwire proved it)
    pop_fire(2'b01, ra, rb);            // speculative damage
    @(negedge clk);
    trap_flush = 1'b1;
    @(posedge clk); @(negedge clk);
    trap_flush = 1'b0;
    check("A1.9: flush restores committed image",
          dut.free_bits_q == dut.committed_free_bits_q);

    // ====== - : DUAL commit, ordered-prefix retire ======
    // Direct module-level sink proof. Reset first so the pool is a known size;
    // the earlier cases have
    // deliberately exhausted and partially restored it.
    @(negedge clk); rst_n = 1'b0;
    repeat (2) @(posedge clk); @(negedge clk); rst_n = 1'b1; @(negedge clk);

    // ---- two INDEPENDENT retires on one edge ----
    pop_fire(2'b11, ra, rb);        // the two OLD mappings (returned as stales)
    pop_fire(2'b11, r0, r1);        // the two NEW mappings (claimed as pdsts)
    n_before = free_count_now();
    @(negedge clk);
    commit_fire = 2'b11; commit_rd_we = 2'b11;
    commit_pdst       = {r1, r0};
    commit_stale_pdst = {rb, ra};   // two DISTINCT stales return
    @(posedge clk); @(negedge clk);
    commit_fire = 2'b00; commit_rd_we = 2'b00;
    check("A1.10: slot-0 stale returned", dut.free_bits_q[ra] == 1'b1);
    check("A1.10: slot-1 stale returned", dut.free_bits_q[rb] == 1'b1);
    check("A1.10: both pdsts held live", (dut.free_bits_q[r0] == 1'b0) &&
                                         (dut.free_bits_q[r1] == 1'b0));
    check("A1.10: net +2 over the pair", free_count_now() == n_before + 2);
    check("A1.10: committed image mirrors both returns",
          dut.committed_free_bits_q[ra] && dut.committed_free_bits_q[rb]);

    // ---- the WAW ALIAS -- slot 1's stale IS slot 0's pdst ----
    // The ordering hazard in one test. Applied slot 0 then slot 1 the shared
    // preg ends FREE (slot 1 killed it); reversed, it leaks forever.
    pop_fire(2'b01, ra, rb);        // ra = the older mapping
    pop_fire(2'b11, r0, r1);        // r0 = slot-0 pdst, r1 = slot-1 pdst
    n_before = free_count_now();
    @(negedge clk);
    commit_fire = 2'b11; commit_rd_we = 2'b11;
    commit_pdst       = {r1, r0};
    commit_stale_pdst = {r0, ra};   // slot 1's stale == slot 0's pdst
    @(posedge clk); @(negedge clk);
    commit_fire = 2'b00; commit_rd_we = 2'b00;
    check("A1.11: aliased preg ends FREE (younger slot killed it)",
          dut.free_bits_q[r0] == 1'b1);
    check("A1.11: surviving mapping stays live", dut.free_bits_q[r1] == 1'b0);
    check("A1.11: older stale still returned", dut.free_bits_q[ra] == 1'b1);
    check("A1.11: pool conserved (2 out, 2 back)",
          free_count_now() == n_before + 2);
    check("A1.11: committed image agrees on the alias",
          dut.committed_free_bits_q[r0] == 1'b1);

    // ---- a length-1 prefix must not apply slot-1 payload ----
    pop_fire(2'b11, ra, rb);
    pop_fire(2'b11, r0, r1);
    n_before = free_count_now();
    @(negedge clk);
    commit_fire = 2'b01; commit_rd_we = 2'b01;   // prefix of length 1
    commit_pdst       = {r1, r0};                // slot-1 payload present...
    commit_stale_pdst = {rb, ra};                // ...but NOT fired
    @(posedge clk); @(negedge clk);
    commit_fire = 2'b00; commit_rd_we = 2'b00;
    check("A1.12: unfired slot-1 stale NOT returned",
          dut.free_bits_q[rb] == 1'b0);
    check("A1.12: fired slot-0 stale returned", dut.free_bits_q[ra] == 1'b1);
    check("A1.12: exactly one net return", free_count_now() == n_before + 1);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_free_list] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_free_list] FAIL errors=%0d checks=%0d",
             errors, checks);
    end
  end

endmodule
