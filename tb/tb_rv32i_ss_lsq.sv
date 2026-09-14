`timescale 1ns/1ps

// LSQ unit battery — coverage of allocation through completion: allocation lifecycle,
// kill/flush recovery, AGU deposits, launch selector + conservative
// ordering scan, snapshot/response channel, forwarding, and the
// armed contract-tripwire set (312 checks).
//
// Checks: program-order allocation with tail tickets,
// widened-count full detection (tail==head disambiguated by count),
// full-queue backpressure with NO entry overwrite, and
// LQ/SQ independence (one queue full never lowers the other's ready).
//
// Contract note: the core guarantees a dispatched op is load XOR store, so
// this TB never drives both allocs in the same cycle.

module tb_rv32i_ss_lsq;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::mem_size_e;
  import fyp_cpu_pkg::MEM_W;
  import fyp_cpu_pkg::MEM_H;
  import fyp_cpu_pkg::MEM_B;

  localparam time CLK_PERIOD = 10ns;

  logic       clk;
  logic       rst_n;

  logic       lq_alloc_valid;
  logic       lq_alloc_ready;
  lq_idx_t    lq_alloc_idx;
  rob_idx_t   lq_alloc_rob_idx;
  rob_seq_t   lq_alloc_rob_seq;
  phys_reg_t  lq_alloc_pdst;
  logic       lq_alloc_rd_wen;
  mem_size_e  lq_alloc_mem_size;
  logic       lq_alloc_mem_unsigned;

  logic       sq_alloc_valid;
  logic       sq_alloc_ready;
  sq_idx_t    sq_alloc_idx;
  rob_idx_t   sq_alloc_rob_idx;
  rob_seq_t   sq_alloc_rob_seq;
  phys_reg_t  sq_alloc_data_preg;

  logic       lq_deposit_fire;
  lq_idx_t    lq_deposit_idx;
  word_t      lq_deposit_addr;
  logic       lq_deposit_inert;

  logic       sq_deposit_fire;
  sq_idx_t    sq_deposit_idx;
  word_t      sq_deposit_addr;
  word_t      sq_deposit_data;
  logic       sq_deposit_data_valid;
  logic [3:0] sq_deposit_be;
  logic       sq_deposit_inert;
  logic       sq_deposit_deferred_pending;

  logic [1:0]      sq_data_wb_fire;
  phys_reg_t [1:0] sq_data_wb_pdst;
  word_t [1:0]     sq_data_wb_value;
  completion_packet_t sq_complete;
  logic               sq_complete_accept;

  // The LSQ derives commit slot p's ROB index as rob_head_idx + p,
  // so the testbench drives rob_head_idx with each commit group.
  logic [1:0] commit_fire;
  logic [1:0] commit_is_store;
  // identity tie: under this TB's always-ready environment the commit
  // WANT coincides with the commit fire for a store, which is exactly the
  // ready/valid contract. The held-request scenarios below override want
  // independently while ready is stalled.
  logic [1:0] store_commit_want;
  logic       m4_want_override = 1'b0;
  logic [1:0] m4_want          = 2'b00;
  assign store_commit_want[0] = m4_want_override ? m4_want[0]
                              : (commit_fire[0] && commit_is_store[0]);
  assign store_commit_want[1] = m4_want_override ? m4_want[1]
                              : (commit_fire[1] && commit_is_store[1]);
  logic sq_mem_accept_o, sq_accept_deferred_o;
  logic [1:0] commit_is_load;

  logic       branch_recover_req;
  rob_idx_t   recover_rob_idx;
  rob_idx_t   rob_head_idx;
  logic       trap_flush;

  logic       dmem_valid;
  logic       dmem_we;
  logic [3:0] dmem_be;
  word_t      dmem_addr;
  word_t      dmem_wdata;
  logic       dmem_ready;
  logic       dmem_rvalid;
  word_t      dmem_rdata;

  completion_packet_t lq_complete;
  logic               cdb_grant_lq;

  int errors = 0;
  int checks = 0;
  int i;
  bit count_recovery_reads = 1'b0;
  int recovery_reads = 0;
  int surviving_held_cases = 0;
  always @(posedge clk) begin
    if (rst_n && count_recovery_reads && dmem_valid && dmem_ready && !dmem_we)
      recovery_reads = recovery_reads + 1;
  end

  rv32i_ss_lsq dut (
    .clk                   (clk),
    .rst_n                 (rst_n),
    .lq_alloc_fire         (lq_alloc_valid & lq_alloc_ready),
    .lq_alloc_ready        (lq_alloc_ready),
    .lq_alloc_idx          (lq_alloc_idx),
    .lq_alloc_rob_idx      (lq_alloc_rob_idx),
    .lq_alloc_rob_seq      (lq_alloc_rob_seq),
    .lq_alloc_pdst         (lq_alloc_pdst),
    .lq_alloc_rd_wen       (lq_alloc_rd_wen),
    .lq_alloc_mem_size     (lq_alloc_mem_size),
    .lq_alloc_mem_unsigned (lq_alloc_mem_unsigned),
    .sq_alloc_fire         (sq_alloc_valid & sq_alloc_ready),
    .sq_alloc_ready        (sq_alloc_ready),
    .sq_alloc_idx          (sq_alloc_idx),
    .sq_alloc_rob_idx      (sq_alloc_rob_idx),
    .sq_alloc_rob_seq      (sq_alloc_rob_seq),
    .sq_alloc_data_preg    (sq_alloc_data_preg),
    .lq_deposit_fire       (lq_deposit_fire),
    .lq_deposit_idx        (lq_deposit_idx),
    .lq_deposit_addr       (lq_deposit_addr),
    .lq_deposit_inert      (lq_deposit_inert),
    .sq_deposit_fire       (sq_deposit_fire),
    .sq_deposit_idx        (sq_deposit_idx),
    .sq_deposit_addr       (sq_deposit_addr),
    .sq_deposit_data       (sq_deposit_data),
    .sq_deposit_data_valid (sq_deposit_data_valid),
    .sq_deposit_be         (sq_deposit_be),
    .sq_deposit_inert      (sq_deposit_inert),
    .sq_deposit_deferred_pending (sq_deposit_deferred_pending),
    .sq_data_wb_fire       (sq_data_wb_fire),
    .sq_data_wb_pdst       (sq_data_wb_pdst),
    .sq_data_wb_value      (sq_data_wb_value),
    .sq_complete           (sq_complete),
    .sq_complete_accept    (sq_complete_accept),
    .commit_fire           (commit_fire),
    .commit_is_store       (commit_is_store),
    .store_commit_want     (store_commit_want),
    .sq_mem_accept         (sq_mem_accept_o),
    .sq_accept_deferred    (sq_accept_deferred_o),
    .commit_is_load        (commit_is_load),
    .branch_recover_req    (branch_recover_req),
    .recover_rob_idx       (recover_rob_idx),
    .rob_head_idx          (rob_head_idx),
    .trap_flush            (trap_flush),
    .dmem_valid            (dmem_valid),
    .dmem_we               (dmem_we),
    .dmem_be               (dmem_be),
    .dmem_addr             (dmem_addr),
    .dmem_wdata            (dmem_wdata),
    .dmem_ready            (dmem_ready),
    .dmem_rvalid           (dmem_rvalid),
    .dmem_rdata            (dmem_rdata),
    .lq_complete           (lq_complete),
    .cdb_grant_lq          (cdb_grant_lq)
  );

  initial clk = 1'b0;
  always #(CLK_PERIOD / 2) clk = ~clk;

  // global watchdog
  initial begin
    #(CLK_PERIOD * 2000);
    $fatal(1, "[tb_rv32i_ss_lsq] WATCHDOG timeout");
  end

  task automatic check(input string name, input logic cond);
    checks++;
    if (!cond) begin
      errors++;
      $display("  FAIL [%0d] %s", checks, name);
    end
  endtask

  task automatic idle_inputs();
    lq_alloc_valid = 1'b0; sq_alloc_valid = 1'b0;
    lq_alloc_rob_idx = '0; lq_alloc_rob_seq = '0; lq_alloc_pdst = '0;
    lq_alloc_rd_wen = 1'b0; lq_alloc_mem_size = MEM_W;
    lq_alloc_mem_unsigned = 1'b0; sq_alloc_rob_idx = '0;
    sq_alloc_rob_seq = '0; sq_alloc_data_preg = '0;
    lq_deposit_fire = 1'b0; lq_deposit_idx = '0; lq_deposit_addr = '0;
    lq_deposit_inert = 1'b0;
    sq_deposit_fire = 1'b0; sq_deposit_idx = '0; sq_deposit_addr = '0;
    sq_deposit_data = '0; sq_deposit_data_valid = 1'b0;
    sq_deposit_be = '0; sq_deposit_inert = 1'b0;
    sq_deposit_deferred_pending = 1'b0;
    sq_data_wb_fire = '0; sq_data_wb_pdst = '0; sq_data_wb_value = '0;
    sq_complete_accept = 1'b0;
    commit_fire = 2'b00; commit_is_store = 2'b00; commit_is_load = 2'b00;
    branch_recover_req = 1'b0; recover_rob_idx = '0; rob_head_idx = '0;
    trap_flush = 1'b0; dmem_ready = 1'b1; dmem_rvalid = 1'b1; dmem_rdata = '0;
    cdb_grant_lq = 1'b0;
  endtask

  // A live read accepted during recovery must consume its issue opportunity
  // exactly once. Count external acceptances, not the DUT's executed flag.
  task automatic surviving_held_read(input rob_idx_t head,
                                     input bit immediate_response,
                                     input int recovery_cycles);
    lq_idx_t ticket;
    lq_entry_t row;
    flush_all();
    @(negedge clk);
    dmem_ready = 1'b0; dmem_rvalid = 1'b0;
    cdb_grant_lq = 1'b0; rob_head_idx = head;
    recovery_reads = 0; count_recovery_reads = 1'b1;
    lq_alloc_one(head, phys_reg_t'(51), MEM_W, ticket, rob_seq_t'(123));
    lq_dep(ticket, 32'h0000_0400, 1'b0);
    repeat (2) @(negedge clk);
    #1;
    check("survivor setup: stalled request is held",
          dmem_valid && !dmem_we && !dmem_ready && dut.dreq_held_q);
    @(negedge clk);
    branch_recover_req = 1'b1;
    recover_rob_idx = head + rob_idx_t'(2);
    dmem_ready = 1'b1;
    dmem_rvalid = immediate_response;
    dmem_rdata = 32'h7100_0123;
    #1;
    check("survivor setup: acceptance overlaps recovery",
          branch_recover_req && dut.lq_launch_held &&
          dmem_valid && dmem_ready && !dmem_we);
    @(posedge clk); @(negedge clk);
    dmem_rvalid = 1'b0;
    repeat (recovery_cycles - 1) @(negedge clk);
    branch_recover_req = 1'b0;
    repeat (3) @(negedge clk);
    if (recovery_reads != 1)
      $fatal(1, "surviving held read accepted %0d times, expected exactly one", recovery_reads);
    check("surviving held read accepted exactly once", recovery_reads == 1);
    row = dut.lq_entry_q[ticket];
    check("surviving accepted owner remains live and issued",
          row.valid && row.executed && row.rob_seq == rob_seq_t'(123));
    if (!immediate_response) begin
      dmem_rvalid = 1'b1;
      @(posedge clk); @(negedge clk);
      dmem_rvalid = 1'b0;
    end
    #1;
    check("survivor completion preserves accepted identity and value",
          lq_complete.valid && lq_complete.rob_idx == head &&
          lq_complete.rob_seq == rob_seq_t'(123) &&
          lq_complete.pdst == phys_reg_t'(51) &&
          lq_complete.result == 32'h7100_0123);
    cdb_grant_lq = 1'b1;
    @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0;
    commit_one(head, 1'b0);
    repeat (2) @(negedge clk);
    check("retired survivor cannot reissue", recovery_reads == 1 && !dmem_valid);
    count_recovery_reads = 1'b0;
    surviving_held_cases++;
  endtask

  // one-cycle LQ allocation; ticket sampled just before the edge
  task automatic lq_alloc_one(input rob_idx_t ridx, input phys_reg_t pd,
                              input mem_size_e sz, output lq_idx_t ticket,
                              input rob_seq_t seq = rob_seq_t'(1));
    @(negedge clk);
    lq_alloc_valid   = 1'b1;
    lq_alloc_rob_idx = ridx;
    lq_alloc_rob_seq = seq;
    lq_alloc_pdst    = pd;
    lq_alloc_rd_wen  = 1'b1;
    lq_alloc_mem_size = sz;
    check("lq ticket == tail pre-fire", lq_alloc_idx == dut.lq_tail_q);
    ticket = lq_alloc_idx;
    @(posedge clk);
    @(negedge clk);
    lq_alloc_valid = 1'b0;
  endtask

  task automatic sq_alloc_one(input rob_idx_t ridx, output sq_idx_t ticket);
    @(negedge clk);
    sq_alloc_valid   = 1'b1;
    sq_alloc_rob_idx = ridx;
    sq_alloc_rob_seq = rob_seq_t'(ridx) + rob_seq_t'(1);
    sq_alloc_data_preg = phys_reg_t'(40 + ridx);
    check("sq ticket == tail pre-fire", sq_alloc_idx == dut.sq_tail_q);
    ticket = sq_alloc_idx;
    @(posedge clk);
    @(negedge clk);
    sq_alloc_valid = 1'b0;
  endtask

  // one registered-recovery beat: branch at ridx recovers with the given
  // ROB head snapshot (kill = ring-age younger than the branch)
  task automatic recover_one(input rob_idx_t ridx, input rob_idx_t head);
    @(negedge clk);
    branch_recover_req = 1'b1;
    recover_rob_idx    = ridx;
    rob_head_idx       = head;
    @(posedge clk);
    @(negedge clk);
    branch_recover_req = 1'b0;
  endtask

  // One commit beat at SLOT 0. The pop/drain are now COMMANDS, so the class
  // must be stated explicitly: is_store drains the SQ head, otherwise this is
  // a load and pops the LQ head. The expected ROB index comes from
  // rob_head_idx, which the seam derives per slot.
  task automatic commit_one(input rob_idx_t ridx, input logic is_store);
    @(negedge clk);
    rob_head_idx    = ridx;
    commit_fire     = 2'b01;
    commit_is_store = {1'b0,  is_store};
    commit_is_load  = {1'b0, ~is_store};
    @(posedge clk);
    @(negedge clk);
    commit_fire     = 2'b00;
    commit_is_store = 2'b00;
    commit_is_load  = 2'b00;
  endtask

  // A committing NON-MEMORY op at slot 0: neither queue may move. Under the
  // command contract this is how "does not pop" is expressed -- a load
  // with no LQ entry is a contract violation, not a suppressed pop.
  task automatic commit_other(input rob_idx_t ridx);
    @(negedge clk);
    rob_head_idx    = ridx;
    commit_fire     = 2'b01;
    commit_is_store = 2'b00;
    commit_is_load  = 2'b00;
    @(posedge clk);
    @(negedge clk);
    commit_fire     = 2'b00;
  endtask

  // an ordered-prefix pair retiring on one edge. Position 0 is older.
  // The module seam keeps every legal memory orientation directly drivable,
  // independent of whole-core scheduling.
  task automatic commit_pair(input rob_idx_t head,
                             input logic is_store0, input logic is_load0,
                             input logic is_store1, input logic is_load1);
    @(negedge clk);
    rob_head_idx    = head;
    commit_fire     = 2'b11;
    commit_is_store = {is_store1, is_store0};
    commit_is_load  = {is_load1,  is_load0};
    @(posedge clk);
    @(negedge clk);
    commit_fire     = 2'b00;
    commit_is_store = 2'b00;
    commit_is_load  = 2'b00;
  endtask

  // one-cycle AGU deposit beats (fire-and-forget mailbox writes)
  task automatic lq_dep(input lq_idx_t idx, input word_t addr,
                        input logic inert);
    @(negedge clk);
    lq_deposit_fire = 1'b1; lq_deposit_idx = idx;
    lq_deposit_addr = addr; lq_deposit_inert = inert;
    @(posedge clk); @(negedge clk);
    lq_deposit_fire = 1'b0; lq_deposit_inert = 1'b0;
  endtask

  task automatic sq_dep(input sq_idx_t idx, input word_t addr,
                        input logic [3:0] be, input logic inert);
    @(negedge clk);
    sq_deposit_fire = 1'b1; sq_deposit_idx = idx;
    sq_deposit_addr = addr; sq_deposit_data = 32'hA5A5_A5A5;
    sq_deposit_data_valid = 1'b1;
    sq_deposit_be = be; sq_deposit_inert = inert;
    sq_deposit_deferred_pending = 1'b0;
    @(posedge clk); @(negedge clk);
    sq_deposit_fire = 1'b0; sq_deposit_data_valid = 1'b0;
    sq_deposit_inert = 1'b0; sq_deposit_deferred_pending = 1'b0;
  endtask

  // Deposit with caller-chosen store data (forwarding tests need
  // distinguishable words to prove WHICH store's data was forwarded).
  task automatic sq_dep_d(input sq_idx_t idx, input word_t addr,
                          input logic [3:0] be, input logic inert,
                          input word_t data);
    @(negedge clk);
    sq_deposit_fire = 1'b1; sq_deposit_idx = idx;
    sq_deposit_addr = addr; sq_deposit_data = data;
    sq_deposit_data_valid = 1'b1;
    sq_deposit_be = be; sq_deposit_inert = inert;
    sq_deposit_deferred_pending = 1'b0;
    @(posedge clk); @(negedge clk);
    sq_deposit_fire = 1'b0; sq_deposit_data_valid = 1'b0;
    sq_deposit_inert = 1'b0; sq_deposit_deferred_pending = 1'b0;
  endtask

  // Address-only store deposit: this is the address-valid/data-pending state supported
  // by store address/data decoupling.  Data and completion deliberately stay
  // absent until an accepted producer beat arrives through the CDB snoop.
  task automatic sq_dep_addr_only(input sq_idx_t idx, input word_t addr,
                                  input logic [3:0] be);
    @(negedge clk);
    sq_deposit_fire = 1'b1; sq_deposit_idx = idx;
    sq_deposit_addr = addr; sq_deposit_data = '0;
    sq_deposit_data_valid = 1'b0; sq_deposit_be = be;
    sq_deposit_inert = 1'b0; sq_deposit_deferred_pending = 1'b1;
    @(posedge clk); @(negedge clk);
    sq_deposit_fire = 1'b0;
  endtask

  // Clear speculative queue state and consume any held completion between
  // independent scenarios. Physical outstanding requests are deliberately
  // not cleared here; tests that create one must return its response first.
  task automatic flush_all();
    @(negedge clk);
    rob_head_idx = '0;
    trap_flush = 1'b1;
    cdb_grant_lq = 1'b1;
    @(posedge clk); @(negedge clk);
    trap_flush = 1'b0;
    cdb_grant_lq = 1'b0;
  endtask

  lq_idx_t   lq_t;
  lq_idx_t   lq_t2;
  sq_idx_t   m4_sq_ticket;
  lq_idx_t   m4_lq_ticket;
  integer    m4_count_before = 0;
  integer    m4_hold_cycles = 0;
  integer    m4_accept_in_recovery = 0;
  integer    m4_held_launches = 0;
  integer    m4_lock_cycles = 0;
  integer    m4_store_during_outstanding = 0;
  integer    m4_same_cycle_held = 0;
  integer    m4_pop_mark_collision = 0;
  integer    m5_two_outstanding = 0;
  integer    m5_same_edge = 0;
  integer    m5_kill_two = 0;
  integer    m5_compl_two = 0;
  integer    m5_compl_block = 0;
  integer    m4_scan_i;
  logic      m4_any_valid;
  lq_entry_t m4_scan_entry;
  sq_idx_t   sq_t;
  sq_idx_t   sq_t2;
  lq_entry_t le;
  sq_entry_t se;
  integer    fw_i;
  integer    dc_i;
  integer    f2_i;
  lq_entry_t f2_before, f2_after;

  initial begin
    idle_inputs();
    rst_n = 1'b0;
    repeat (4) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);

    // ==== tripwire RED modes: one illegal vector, expect DUT $fatal ===
    // House convention (scripts/run_ss_tbs.sh): the stimulus is illegal, the
    // DUT's own contract pin must trip. Reaching the TB_RED_FAIL line means
    // the pin is missing or too weak.
    if ($test$plusargs("red_lsq_pop_empty")) begin
      // Isolate the invalid-row pin from the independent occupancy pin.
      // Deliberately inconsistent count is fault injection, not a legal state.
      force dut.lq_count_q = 1;
      commit_one(rob_idx_t'(0), 1'b0);
      release dut.lq_count_q;
      $display("TB_RED_FAIL: LQ pop on an empty queue did not trip");
      $finish;
    end
    if ($test$plusargs("red_lsq_two_stores")) begin
      // Two stores in one commit group. The single dmem write port makes this
      // illegal. excludes it in the core; the LSQ independently pins the
      // consuming-boundary contract.
      // Deposit the SQ head first so the drain-honesty pins are satisfied and
      // the ONE-STORE pin is unambiguously the one under test.
      sq_alloc_one(rob_idx_t'(0), sq_t);
      sq_alloc_one(rob_idx_t'(1), sq_t);
      sq_dep(sq_idx_t'(0), 32'h0000_0040, 4'b1111, 1'b0);
      sq_dep(sq_idx_t'(1), 32'h0000_0044, 4'b1111, 1'b0);
      // Keep the accepting head's correspondence legal while injecting the
      // illegal two-store retirement group. Otherwise a second pin masks a
      // missing one-store assertion in the counterfactual run.
      m4_want_override = 1'b1;
      m4_want = 2'b01;
      commit_pair(rob_idx_t'(0), 1'b1, 1'b0, 1'b1, 1'b0);
      $display("TB_RED_FAIL: two stores in one commit group did not trip");
      $finish;
    end
    if ($test$plusargs("red_lsq_correspondence")) begin
      // A load whose LQ head does NOT correspond to rob_head_idx + slot.
      // An index mismatch must trigger the correspondence assertion.
      lq_alloc_one(rob_idx_t'(5), phys_reg_t'(10), MEM_W, lq_t);
      commit_one(rob_idx_t'(29), 1'b0);
      $display("TB_RED_FAIL: LQ pop correspondence mismatch did not trip");
      $finish;
    end

    // ========== POSITIVE dual-commit seam proof ====================
    // These are legal commands. They complement the RED modes above: the RED
    // legs prove bad commands die loudly; these cases prove every slot-1 data
    // movement works when the correspondence contract is satisfied.

    // DC1: two adjacent loads retire together and pop two distinct LQ rows.
    lq_alloc_one(rob_idx_t'(10), phys_reg_t'(20), MEM_W, lq_t);
    lq_alloc_one(rob_idx_t'(11), phys_reg_t'(21), MEM_W, lq_t2);
    commit_pair(rob_idx_t'(10), 1'b0, 1'b1, 1'b0, 1'b1);
    check("DC1: dual load pop drains both rows", dut.lq_count_q == '0);
    check("DC1: dual load pop advances head by two", dut.lq_head_q == lq_idx_t'(2));
    le = dut.lq_entry_q[lq_t];
    check("DC1: first popped LQ row cleared", le.valid == 1'b0);
    le = dut.lq_entry_q[lq_t2];
    check("DC1: second popped LQ row cleared", le.valid == 1'b0);

    // DC2: slot 0 is non-memory; a slot-1 load still pops the LQ HEAD while
    // its ROB correspondence is head+1.
    flush_all();
    lq_alloc_one(rob_idx_t'(21), phys_reg_t'(22), MEM_W, lq_t);
    commit_pair(rob_idx_t'(20), 1'b0, 1'b0, 1'b0, 1'b1);
    check("DC2: slot-1-only load pops one row", dut.lq_count_q == '0);
    check("DC2: slot-1-only load advances LQ head once", dut.lq_head_q == lq_idx_t'(1));

    // DC3: load at slot 0 plus store at slot 1 -- one LQ pop and the sole SQ
    // drain coexist. The store's ROB index is head+1 and its payload must own
    // the memory beat.
    flush_all();
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(23), MEM_W, lq_t);
    sq_alloc_one(rob_idx_t'(5), sq_t);
    sq_dep_d(sq_t, 32'h0000_0120, 4'b1111, 1'b0, 32'h1357_9BDF);
    @(negedge clk);
    rob_head_idx    = rob_idx_t'(4);
    commit_fire     = 2'b11;
    commit_is_load  = 2'b01;
    commit_is_store = 2'b10;
    #1;
    check("DC3: load+store issues one LQ pop and slot-1 SQ drain",
          (dut.lq_pop_fire == 2'b01) && (dut.sq_drain_req == 2'b10));
    check("DC3: slot-1 store owns the dmem beat",
          dmem_valid && dmem_we && (dmem_addr == 32'h0000_0120) &&
          (dmem_wdata == 32'h1357_9BDF));
    @(posedge clk); @(negedge clk);
    commit_fire = 2'b00; commit_is_load = 2'b00; commit_is_store = 2'b00;
    check("DC3: load+store drains both queues",
          (dut.lq_count_q == '0) && (dut.sq_count_q == '0));

    // DC4: reverse orientation -- store at slot 0 plus load at slot 1.
    flush_all();
    sq_alloc_one(rob_idx_t'(8), sq_t);
    sq_dep_d(sq_t, 32'h0000_0124, 4'b1111, 1'b0, 32'h2468_ACE0);
    lq_alloc_one(rob_idx_t'(9), phys_reg_t'(24), MEM_W, lq_t);
    @(negedge clk);
    rob_head_idx    = rob_idx_t'(8);
    commit_fire     = 2'b11;
    commit_is_store = 2'b01;
    commit_is_load  = 2'b10;
    #1;
    check("DC4: store+load issues slot-0 SQ drain and one LQ pop",
          (dut.sq_drain_req == 2'b01) && (dut.lq_pop_fire == 2'b10));
    check("DC4: slot-0 store owns the dmem beat",
          dmem_valid && dmem_we && (dmem_addr == 32'h0000_0124) &&
          (dmem_wdata == 32'h2468_ACE0));
    @(posedge clk); @(negedge clk);
    commit_fire = 2'b00; commit_is_load = 2'b00; commit_is_store = 2'b00;
    check("DC4: store+load drains both queues",
          (dut.lq_count_q == '0) && (dut.sq_count_q == '0));

    // DC5: depth-minus-one allows an allocation beside TWO pops. The tail
    // row is disjoint from both head rows, count changes 7+1-2=6, and wrap is
    // exercised when tail 7 advances to 0.
    flush_all();
    for (dc_i = 0; dc_i < 7; dc_i = dc_i + 1)
      lq_alloc_one(rob_idx_t'(10 + dc_i), phys_reg_t'(25 + dc_i), MEM_W, lq_t);
    @(negedge clk);
    lq_alloc_valid    = 1'b1;
    lq_alloc_rob_idx  = rob_idx_t'(17);
    lq_alloc_rob_seq  = rob_seq_t'(17);
    lq_alloc_pdst     = phys_reg_t'(40);
    lq_alloc_rd_wen   = 1'b1;
    lq_alloc_mem_size = MEM_W;
    rob_head_idx      = rob_idx_t'(10);
    commit_fire       = 2'b11;
    commit_is_load    = 2'b11;
    #1;
    check("DC5: depth-1 allocation fires beside dual pop",
          (lq_alloc_valid && lq_alloc_ready) && (dut.lq_pop_count == 2));
    check("DC5: allocation index disjoint from both pop indices",
          (lq_alloc_idx != dut.lq_pop_idx[0]) &&
          (lq_alloc_idx != dut.lq_pop_idx[1]));
    @(posedge clk); @(negedge clk);
    lq_alloc_valid = 1'b0; commit_fire = 2'b00; commit_is_load = 2'b00;
    check("DC5: count is 7+1-2", dut.lq_count_q == 6);
    check("DC5: head advanced two and tail wrapped",
          (dut.lq_head_q == lq_idx_t'(2)) && (dut.lq_tail_q == lq_idx_t'(0)));
    le = dut.lq_entry_q[7];
    check("DC5: concurrent allocation landed", le.valid && (le.rob_idx == rob_idx_t'(17)));

    // DC6: at FULL, registered availability refuses allocation on the dual-
    // pop edge. The held offer fires only on the following edge, after the two
    // committed loads have made space.
    flush_all();
    for (dc_i = 0; dc_i < 8; dc_i = dc_i + 1)
      lq_alloc_one(rob_idx_t'(dc_i), phys_reg_t'(32 + dc_i), MEM_W, lq_t);
    @(negedge clk);
    lq_alloc_valid    = 1'b1;
    lq_alloc_rob_idx  = rob_idx_t'(8);
    lq_alloc_rob_seq  = rob_seq_t'(18);
    lq_alloc_pdst     = phys_reg_t'(41);
    lq_alloc_rd_wen   = 1'b1;
    lq_alloc_mem_size = MEM_W;
    rob_head_idx      = rob_idx_t'(0);
    commit_fire       = 2'b11;
    commit_is_load    = 2'b11;
    #1;
    check("DC6: full queue refuses same-edge allocation",
          !(lq_alloc_valid && lq_alloc_ready));
    @(posedge clk); @(negedge clk);
    commit_fire = 2'b00; commit_is_load = 2'b00;
    #1;
    check("DC6: held allocation becomes fireable next edge",
          lq_alloc_valid && lq_alloc_ready);
    @(posedge clk); @(negedge clk);
    lq_alloc_valid = 1'b0;
    check("DC6: full-pop-pop-then-alloc leaves seven rows",
          dut.lq_count_q == 7);
    le = dut.lq_entry_q[0];
    check("DC6: held allocation reused cleared wrapped row",
          le.valid && (le.rob_idx == rob_idx_t'(8)));

    // DC7: the SQ has the same registered boundary. At full, a held
    // allocation waits while a SLOT-1 store drains, then refills the cleared
    // head on the following edge. This is the positive slot-1-store proof.
    flush_all();
    for (dc_i = 0; dc_i < 8; dc_i = dc_i + 1)
      sq_alloc_one(rob_idx_t'(10 + dc_i), sq_t);
    sq_dep_d(sq_idx_t'(0), 32'h0000_0130, 4'b1111, 1'b0, 32'h55AA_33CC);
    @(negedge clk);
    sq_alloc_valid   = 1'b1;
    sq_alloc_rob_idx = rob_idx_t'(18);
    rob_head_idx     = rob_idx_t'(9);
    commit_fire      = 2'b11;
    commit_is_store  = 2'b10;
    #1;
    check("DC7: full SQ refuses allocation on slot-1 drain edge",
          !(sq_alloc_valid && sq_alloc_ready));
    check("DC7: slot-1 store drains the full SQ head",
          dmem_valid && dmem_we && (dmem_addr == 32'h0000_0130));
    @(posedge clk); @(negedge clk);
    commit_fire = 2'b00; commit_is_store = 2'b00;
    #1;
    check("DC7: held SQ allocation becomes fireable next edge",
          sq_alloc_valid && sq_alloc_ready);
    @(posedge clk); @(negedge clk);
    sq_alloc_valid = 1'b0;
    check("DC7: drain then held allocation restores full occupancy",
          dut.sq_count_q == 8);
    se = dut.sq_entry_q[0];
    check("DC7: held SQ allocation reused the cleared wrapped row",
          se.valid && (se.rob_idx == rob_idx_t'(18)));

    flush_all();

    // ---- reset state ----
    check("reset: lq ready", lq_alloc_ready === 1'b1);
    check("reset: sq ready", sq_alloc_ready === 1'b1);
    check("reset: lq count 0", dut.lq_count_q == '0);
    check("reset: sq count 0", dut.sq_count_q == '0);

    // ---- program-order allocation + entry integrity ----
    lq_alloc_one(rob_idx_t'(5), phys_reg_t'(10), MEM_W, lq_t);
    check("lq[0] ticket 0", lq_t == 0);
    lq_alloc_one(rob_idx_t'(6), phys_reg_t'(11), MEM_H, lq_t);
    lq_alloc_one(rob_idx_t'(7), phys_reg_t'(12), MEM_B, lq_t);
    check("lq count 3", dut.lq_count_q == 3);
    le = dut.lq_entry_q[1];
    check("lq[1] rob_idx", le.rob_idx == rob_idx_t'(6));
    check("lq[1] pdst",    le.pdst == phys_reg_t'(11));
    check("lq[1] size",    le.mem_size == MEM_H);
    check("lq[1] addr not yet valid", le.addr_valid == 1'b0);

    // interleave a store: queues advance independently
    sq_alloc_one(rob_idx_t'(8), sq_t);
    check("sq[0] ticket 0", sq_t == 0);
    se = dut.sq_entry_q[0];
    check("sq[0] rob_idx", se.rob_idx == rob_idx_t'(8));
    check("sq[0] addr/data not yet valid",
          (se.addr_valid == 1'b0) && (se.data_valid == 1'b0));
    check("lq count unchanged by sq alloc", dut.lq_count_q == 3);

    // ---- fill LQ to 8 ----
    for (i = 3; i < 8; i++) begin
      lq_alloc_one(rob_idx_t'(9 + i), phys_reg_t'(13 + i), MEM_W, lq_t);
    end
    check("lq full: count 8", dut.lq_count_q == 8);
    check("lq full: ready LOW", lq_alloc_ready === 1'b0);
    check("lq full: tail wrapped to head (count disambiguates)",
          (dut.lq_tail_q == dut.lq_head_q) && (dut.lq_count_q == 8));
    check("independence: sq ready still HIGH", sq_alloc_ready === 1'b1);

    // ---- full-queue backpressure: held valid must not overwrite ----
    @(negedge clk);
    lq_alloc_valid   = 1'b1;
    lq_alloc_rob_idx = rob_idx_t'(31);
    lq_alloc_pdst    = phys_reg_t'(63);
    repeat (3) @(posedge clk);
    @(negedge clk);
    lq_alloc_valid = 1'b0;
    check("held-full: count still 8", dut.lq_count_q == 8);
    le = dut.lq_entry_q[0];
    check("held-full: entry[0] not overwritten", le.rob_idx == rob_idx_t'(5));
    le = dut.lq_entry_q[7];
    check("held-full: entry[7] intact", le.rob_idx == rob_idx_t'(16));

    // ---- fill SQ to 8; same properties ----
    for (i = 1; i < 8; i++) begin
      sq_alloc_one(rob_idx_t'(17 + i), sq_t);
    end
    check("sq full: count 8", dut.sq_count_q == 8);
    check("sq full: ready LOW", sq_alloc_ready === 1'b0);
    check("sq full: tail==head + count",
          (dut.sq_tail_q == dut.sq_head_q) && (dut.sq_count_q == 8));
    check("independence: lq still full/LOW", lq_alloc_ready === 1'b0);

    // ================= lifecycle: pops at commit =================
    // LQ head is rob 5. A committing NON-LOAD must not disturb the LQ.
    // A load commit commands a pop and requires a corresponding entry;
    // a non-memory commit leaves the LQ untouched.
    commit_other(rob_idx_t'(29));
    check("pop guard: non-load commit pops nothing", dut.lq_count_q == 8);

    // matching commit pops the head, clears the entry, un-wedges ready
    commit_one(rob_idx_t'(5), 1'b0);
    check("lq pop: count 7", dut.lq_count_q == 7);
    check("lq pop: ready HIGH again (un-wedge)", lq_alloc_ready === 1'b1);
    check("lq pop: head advanced", dut.lq_head_q == 1);
    le = dut.lq_entry_q[0];
    check("lq pop: popped entry cleared", le.valid == 1'b0);

    // SQ pop path keys on commit_is_store (SQ head is rob 8). The machine
    // contract says a committing store HAS deposited (done implies
    // deposited) — the tripwire enforces it, so deposit first.
    @(negedge clk);
    sq_deposit_fire = 1'b1; sq_deposit_idx = sq_idx_t'(0);
    sq_deposit_addr = 32'h0000_0040; sq_deposit_data = 32'h0000_0011;
    sq_deposit_data_valid = 1'b1; sq_deposit_deferred_pending = 1'b0;
    sq_deposit_be = 4'b1111; sq_deposit_inert = 1'b0;
    @(posedge clk); @(negedge clk);
    sq_deposit_fire = 1'b0; sq_deposit_data_valid = 1'b0;
    sq_deposit_deferred_pending = 1'b0;
    commit_one(rob_idx_t'(8), 1'b1);
    check("sq pop: count 7", dut.sq_count_q == 7);
    check("sq pop: ready HIGH again", sq_alloc_ready === 1'b1);
    check("sq pop: head advanced", dut.sq_head_q == 1);

    // ---- simultaneous alloc + pop: count holds, both pointers move ----
    // LQ head is now rob 6 at entry 1; tail is at entry 0 (wrapped).
    @(negedge clk);
    lq_alloc_valid   = 1'b1;
    lq_alloc_rob_idx = rob_idx_t'(40);
    lq_alloc_pdst    = phys_reg_t'(41);
    lq_alloc_rd_wen  = 1'b1;
    lq_alloc_mem_size = MEM_W;
    rob_head_idx     = rob_idx_t'(6);
    commit_fire      = 2'b01;
    commit_is_store  = 2'b00;
    commit_is_load   = 2'b01;
    @(posedge clk);
    @(negedge clk);
    lq_alloc_valid = 1'b0;
    commit_fire    = 2'b00;
    commit_is_load = 2'b00;
    check("simul: count unchanged", dut.lq_count_q == 7);
    check("simul: head advanced", dut.lq_head_q == 2);
    check("simul: tail advanced", dut.lq_tail_q == 1);
    le = dut.lq_entry_q[0];
    check("simul: alloc landed at wrapped tail", le.rob_idx == rob_idx_t'(40)
          && le.valid == 1'b1);
    le = dut.lq_entry_q[1];
    check("simul: popped entry cleared", le.valid == 1'b0);

    // ---- drain LQ to empty; pop-when-empty must be inert ----
    commit_one(rob_idx_t'(7), 1'b0);
    for (i = 0; i < 5; i++) begin
      commit_one(rob_idx_t'(12 + i), 1'b0);   // rob 12..16
    end
    commit_one(rob_idx_t'(40), 1'b0);
    check("drain: count 0", dut.lq_count_q == 0);
    check("drain: head==tail at empty", dut.lq_head_q == dut.lq_tail_q);
    commit_other(rob_idx_t'(40));             // non-load commit at empty
    check("pop-empty: inert", dut.lq_count_q == 0);
    check("pop-empty: head unchanged", dut.lq_head_q == dut.lq_tail_q);

    // ---- allocate again after wrap: full lifecycle reuse ----
    lq_alloc_one(rob_idx_t'(21), phys_reg_t'(22), MEM_H, lq_t);
    check("reuse: count 1", dut.lq_count_q == 1);
    le = dut.lq_entry_q[dut.lq_head_q];
    check("reuse: entry integrity at wrapped index",
          le.rob_idx == rob_idx_t'(21) && le.mem_size == MEM_H);

    // ================= recovery: trap clear + branch kill =================
    // trap flush clears ALL state in both queues (LQ holds 1, SQ holds 7)
    @(negedge clk); trap_flush = 1'b1; @(posedge clk); @(negedge clk);
    trap_flush = 1'b0;
    check("trap: lq count 0", dut.lq_count_q == 0);
    check("trap: sq count 0", dut.sq_count_q == 0);
    check("trap: lq head==tail==0",
          (dut.lq_head_q == 0) && (dut.lq_tail_q == 0));
    check("trap: sq head==tail==0",
          (dut.sq_head_q == 0) && (dut.sq_tail_q == 0));
    check("trap: both readies HIGH",
          (lq_alloc_ready === 1'b1) && (sq_alloc_ready === 1'b1));
    le = dut.lq_entry_q[0]; se = dut.sq_entry_q[1];
    check("trap: entries invalidated", (le.valid == 1'b0) && (se.valid == 1'b0));

    // mid-kill — entries rob 10,12,14,16,18 (head_idx 10);
    // branch rob 15 -> survivors 10,12,14; killed 16,18
    lq_alloc_one(rob_idx_t'(10), phys_reg_t'(30), MEM_W, lq_t);
    lq_alloc_one(rob_idx_t'(12), phys_reg_t'(31), MEM_W, lq_t);
    lq_alloc_one(rob_idx_t'(14), phys_reg_t'(32), MEM_W, lq_t);
    lq_alloc_one(rob_idx_t'(16), phys_reg_t'(33), MEM_W, lq_t);
    lq_alloc_one(rob_idx_t'(18), phys_reg_t'(34), MEM_W, lq_t);
    recover_one(rob_idx_t'(15), rob_idx_t'(10));
    check("R1: survivor count 3", dut.lq_count_q == 3);
    check("R1: rebuilt tail = head+3", dut.lq_tail_q == 3);
    check("R1: head unchanged", dut.lq_head_q == 0);
    le = dut.lq_entry_q[0];
    check("R1: oldest survivor intact",
          le.valid && le.rob_idx == rob_idx_t'(10) && le.pdst == phys_reg_t'(30));
    le = dut.lq_entry_q[2];
    check("R1: youngest survivor intact",
          le.valid && le.rob_idx == rob_idx_t'(14));
    le = dut.lq_entry_q[3];
    check("R1: killed entry 3 cleared", le.valid == 1'b0);
    le = dut.lq_entry_q[4];
    check("R1: killed entry 4 cleared", le.valid == 1'b0);

    // zero survivors — branch rob 9 older than every entry (head_idx 8)
    recover_one(rob_idx_t'(9), rob_idx_t'(8));
    check("R2: count 0", dut.lq_count_q == 0);
    check("R2: tail==head", dut.lq_tail_q == dut.lq_head_q);
    check("R2: ready HIGH", lq_alloc_ready === 1'b1);
    le = dut.lq_entry_q[0];
    check("R2: all cleared", le.valid == 1'b0);

    // full-eight survivors — fill 8 (rob 10..17), branch rob 18 younger
    // than all: nothing dies, count-8 rebuild + tail cast exercised
    for (i = 0; i < 8; i++) begin
      lq_alloc_one(rob_idx_t'(10 + i), phys_reg_t'(30 + i), MEM_W, lq_t);
    end
    recover_one(rob_idx_t'(18), rob_idx_t'(10));
    check("R3: count still 8", dut.lq_count_q == 8);
    check("R3: ready still LOW", lq_alloc_ready === 1'b0);
    check("R3: tail==head at full", dut.lq_tail_q == dut.lq_head_q);
    le = dut.lq_entry_q[7];
    check("R3: youngest survivor intact", le.valid && le.rob_idx == rob_idx_t'(17));

    // wrapped recovered tail — pop 5 (head=5), alloc rob 19,21 (wraps),
    // branch rob 20 (head_idx 15): survivors 15,16,17,19; killed 21
    for (i = 0; i < 5; i++) commit_one(rob_idx_t'(10 + i), 1'b0);
    lq_alloc_one(rob_idx_t'(19), phys_reg_t'(40), MEM_W, lq_t);
    lq_alloc_one(rob_idx_t'(21), phys_reg_t'(41), MEM_W, lq_t);
    check("R4 setup: head 5 tail 2 count 5",
          (dut.lq_head_q == 5) && (dut.lq_tail_q == 2) && (dut.lq_count_q == 5));
    recover_one(rob_idx_t'(20), rob_idx_t'(15));
    check("R4: survivor count 4", dut.lq_count_q == 4);
    check("R4: WRAPPED rebuilt tail = (5+4)%8 = 1", dut.lq_tail_q == 1);
    le = dut.lq_entry_q[0];
    check("R4: wrapped survivor rob 19 intact",
          le.valid && le.rob_idx == rob_idx_t'(19));
    le = dut.lq_entry_q[1];
    check("R4: killed wrapped entry cleared", le.valid == 1'b0);
    le = dut.lq_entry_q[5];
    check("R4: oldest survivor intact", le.valid && le.rob_idx == rob_idx_t'(15));

    // ROB-index wrap in the AGE math — entries rob 30,31,1 (head_idx 30),
    // branch rob 0: ages 0,1,3 vs recover_age 2 -> survivors 30,31; kill 1
    @(negedge clk); trap_flush = 1'b1; @(posedge clk); @(negedge clk);
    trap_flush = 1'b0;
    lq_alloc_one(rob_idx_t'(30), phys_reg_t'(50), MEM_W, lq_t);
    lq_alloc_one(rob_idx_t'(31), phys_reg_t'(51), MEM_W, lq_t);
    lq_alloc_one(rob_idx_t'(1),  phys_reg_t'(52), MEM_W, lq_t);
    recover_one(rob_idx_t'(0), rob_idx_t'(30));
    check("R5: rob-wrapped ages — count 2", dut.lq_count_q == 2);
    le = dut.lq_entry_q[1];
    check("R5: survivor rob 31 intact", le.valid && le.rob_idx == rob_idx_t'(31));
    le = dut.lq_entry_q[2];
    check("R5: killed rob 1 cleared", le.valid == 1'b0);

    // SQ kill mirror — entries rob 8,10,12 (head_idx 8), branch rob 11:
    // survivors 8,10; killed 12
    sq_alloc_one(rob_idx_t'(8), sq_t);
    sq_alloc_one(rob_idx_t'(10), sq_t);
    sq_alloc_one(rob_idx_t'(12), sq_t);
    recover_one(rob_idx_t'(11), rob_idx_t'(8));
    check("R6: sq survivor count 2", dut.sq_count_q == 2);
    check("R6: sq rebuilt tail", dut.sq_tail_q == sq_idx_t'(dut.sq_head_q + 2));
    se = dut.sq_entry_q[1];
    check("R6: sq survivor intact", se.valid && se.rob_idx == rob_idx_t'(10));
    se = dut.sq_entry_q[2];
    check("R6: sq killed cleared", se.valid == 1'b0);
    // recovery is MACHINE-WIDE: the same event evaluates the LQ too — its
    // leftover entries (rob 30,31 vs head 8) are ring-younger than branch 11
    // and must die with the SQ's rob 12 (cross-queue simultaneity)
    check("R6: same recovery kills ring-younger LQ entries too",
          dut.lq_count_q == 0);

    // ============ deposits + drain payload + dmem ownership ============
    // State here: SQ holds rob 8 (entry 0, head) and rob 10 (entry 1), no deposits.

    // deposit into rob 8's reserved entry — RMW preserves valid/rob_idx.
    @(negedge clk);
    sq_deposit_fire = 1'b1; sq_deposit_idx = sq_idx_t'(0);
    sq_deposit_addr = 32'h0000_0100; sq_deposit_data = 32'hDEAD_BEEF;
    sq_deposit_data_valid = 1'b1; sq_deposit_deferred_pending = 1'b0;
    sq_deposit_be = 4'b1111; sq_deposit_inert = 1'b0;
    @(posedge clk); @(negedge clk);
    sq_deposit_fire = 1'b0; sq_deposit_data_valid = 1'b0;
    sq_deposit_deferred_pending = 1'b0;
    se = dut.sq_entry_q[0];
    check("D1: addr/data valid set", se.addr_valid && se.data_valid);
    check("D1: payload exact", se.addr == 32'h0000_0100 &&
          se.data == 32'hDEAD_BEEF && se.be == 4'b1111);
    check("D1: valid/rob_idx preserved", se.valid && se.rob_idx == rob_idx_t'(8));
    check("D1: not inert", se.inert == 1'b0);

    // inert deposit (misaligned store) into rob 10's entry.
    @(negedge clk);
    sq_deposit_fire = 1'b1; sq_deposit_idx = sq_idx_t'(1);
    sq_deposit_addr = 32'h0000_0102; sq_deposit_data = 32'h0000_0055;
    sq_deposit_data_valid = 1'b1; sq_deposit_deferred_pending = 1'b0;
    sq_deposit_be = 4'b0011; sq_deposit_inert = 1'b1;
    @(posedge clk); @(negedge clk);
    sq_deposit_fire = 1'b0; sq_deposit_data_valid = 1'b0;
    sq_deposit_inert = 1'b0; sq_deposit_deferred_pending = 1'b0;
    se = dut.sq_entry_q[1];
    check("D2: inert flagged", se.inert == 1'b1);
    check("D2: rob_idx preserved", se.rob_idx == rob_idx_t'(10));

    // idle port — no request without drain or load
    check("D3: dmem quiet when idle", dmem_valid === 1'b0 && dmem_we === 1'b0);

    // Drain presents the head payload, asserts the write strobe and pops
    // the entry. Later launch scenarios exercise load/store arbitration.
    @(negedge clk);
    rob_head_idx = rob_idx_t'(8); commit_fire = 2'b01; commit_is_store = 2'b01;
    #1;
    check("D5: drain drives write", dmem_valid === 1'b1 && dmem_we === 1'b1);
    check("D5: drain payload = head entry",
          dmem_addr == 32'h0000_0100 && dmem_wdata == 32'hDEAD_BEEF &&
          dmem_be == 4'b1111);
    @(posedge clk); @(negedge clk);
    commit_fire = 2'b00; commit_is_store = 2'b00;
    check("D5: drain popped (count 1, head 1)",
          dut.sq_count_q == 1 && dut.sq_head_q == 1);
    se = dut.sq_entry_q[0];
    check("D5: drained entry cleared", se.valid == 1'b0);

    // ============ LQ mailbox (load address deposits) ============
    // LQ is empty here; allocate two loads then deposit into their tickets
    lq_alloc_one(rob_idx_t'(12), phys_reg_t'(20), MEM_H, lq_t);
    lq_alloc_one(rob_idx_t'(14), phys_reg_t'(21), MEM_B, lq_t);

    @(negedge clk);
    lq_deposit_fire = 1'b1; lq_deposit_idx = lq_idx_t'(dut.lq_head_q);
    lq_deposit_addr = 32'h0000_0200; lq_deposit_inert = 1'b0;
    @(posedge clk); @(negedge clk);
    lq_deposit_fire = 1'b0;
    le = dut.lq_entry_q[dut.lq_head_q];
    check("B1: lq addr deposited + valid", le.addr_valid && le.addr == 32'h0000_0200);
    check("B1: dispatch metadata preserved",
          le.valid && le.rob_idx == rob_idx_t'(12) &&
          le.pdst == phys_reg_t'(20) && le.mem_size == MEM_H);
    check("B1: not inert", le.inert == 1'b0);

    // misaligned-load deposit — inert marked, second entry untouched until now.
    @(negedge clk);
    lq_deposit_fire = 1'b1;
    lq_deposit_idx  = lq_idx_t'(dut.lq_head_q + 1);
    lq_deposit_addr = 32'h0000_0201; lq_deposit_inert = 1'b1;
    @(posedge clk); @(negedge clk);
    lq_deposit_fire = 1'b0; lq_deposit_inert = 1'b0;
    le = dut.lq_entry_q[dut.lq_head_q + 1];
    check("B2: inert load flagged", le.inert && le.addr_valid);
    check("B2: metadata preserved", le.valid && le.rob_idx == rob_idx_t'(14) &&
          le.mem_size == MEM_B);
    le = dut.lq_entry_q[dut.lq_head_q];
    check("B2: neighbor entry untouched", le.addr == 32'h0000_0200 && !le.inert);
    check("B2: deposits change no counts",
          dut.lq_count_q == 2 && dut.sq_count_q == 1);

    // ============ /: selector + conservative ordering scan ========
    // All checks observe the combinational nomination and ordering scan
    // (lq_select_*, lq_mem_safe, load_be) hierarchically; each scenario
    // starts from a trap-flush-cleared state. rob_head_idx = 0 unless noted.

    // empty machine — no nomination, never safe
    flush_all();
    #1;
    check("T3.0: empty -> no select", dut.lq_select_valid == 1'b0);
    check("T3.0: empty -> not safe", dut.lq_mem_safe == 1'b0);

    // lone deposited load, no stores — selected, safe, word byte lanes.
    lq_alloc_one(rob_idx_t'(2), phys_reg_t'(5), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);
    #1;
    check("T3.1: selected", dut.lq_select_valid && (dut.lq_select_idx == lq_t));
    check("T3.1: no stores -> safe", dut.lq_mem_safe == 1'b1);
    check("T3.1: word byte lanes", dut.load_be == 4'b1111);

    // undeposited OLDER store blocks; disjoint deposit unblocks
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);              // older store, no addr yet
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(5), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);
    #1;
    check("T3.2: unknown older store blocks", dut.lq_mem_safe == 1'b0);
    check("T3.2: nomination unaffected by scan", dut.lq_select_valid == 1'b1);
    sq_dep(sq_t, 32'h0000_0200, 4'b1111, 1'b0);     // disjoint word
    #1;
    check("T3.2: disjoint deposit unblocks", dut.lq_mem_safe == 1'b1);

    // byte-granular precision — same word, disjoint byte lanes = safe;
    // intersecting byte lanes = blocked. Deposits are exactly-once (
    // tripwire), so each variant is a fresh scenario.
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);
    sq_dep(sq_t, 32'h0000_0100, 4'b0011, 1'b0);     // sh @ bytes 0-1
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(5), MEM_B, lq_t);
    lq_dep(lq_t, 32'h0000_0102, 1'b0);              // lb @ byte 2
    #1;
    check("T3.3: same word, disjoint bytes -> safe",
          (dut.load_be == 4'b0100) && (dut.lq_mem_safe == 1'b1));
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);
    sq_dep(sq_t, 32'h0000_0100, 4'b0011, 1'b0);
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(5), MEM_B, lq_t);
    lq_dep(lq_t, 32'h0000_0101, 1'b0);              // lb @ byte 1: intersects
    #1;
    check("T3.3: same word, hit byte -> blocked",
          (dut.load_be == 4'b0010) && (dut.lq_mem_safe == 1'b0));

    // halfword byte-lane derivation vs a word store (fresh per variant).
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);
    sq_dep(sq_t, 32'h0000_0100, 4'b1111, 1'b0);
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(5), MEM_H, lq_t);
    lq_dep(lq_t, 32'h0000_0102, 1'b0);              // upper half
    #1;
    check("T3.4: lh upper byte lanes", dut.load_be == 4'b1100);
    check("T3.4: word store hits -> blocked", dut.lq_mem_safe == 1'b0);
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);
    sq_dep(sq_t, 32'h0000_0100, 4'b1111, 1'b0);
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(5), MEM_H, lq_t);
    lq_dep(lq_t, 32'h0000_0200, 1'b0);              // other word, lower half
    #1;
    check("T3.4: lh lower byte lanes", dut.load_be == 4'b0011);
    check("T3.4: different word -> safe", dut.lq_mem_safe == 1'b1);

    // YOUNGER store never blocks, even fully overlapping. The store
    // is in place BEFORE the load's address arrives — otherwise the live
    // launcher fires the lone safe load before the store exists.
    flush_all();
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(5), MEM_W, lq_t);
    sq_alloc_one(rob_idx_t'(6), sq_t);              // younger than the load
    sq_dep(sq_t, 32'h0000_0100, 4'b1111, 1'b0);     // full overlap
    lq_dep(lq_t, 32'h0000_0100, 1'b0);              // load addr arrives last
    #1;
    check("T3.5: younger store invisible", dut.lq_mem_safe == 1'b1);

    // inert (misaligned) older store invisible to the CAM
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);
    sq_dep(sq_t, 32'h0000_0101, 4'b1111, 1'b1);     // inert deposit, overlaps
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(5), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);
    #1;
    check("T3.6: inert older store invisible", dut.lq_mem_safe == 1'b1);

    // A full-overlap older store FORWARDS — no dmem
    // read, winner data + load identity in the mailbox, executed one-shot,
    // and the store keeps its normal commit-time drain.
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);
    sq_dep(sq_t, 32'h0000_0100, 4'b1111, 1'b0);     // data = A5A5_A5A5
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(5), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);
    #1;
    check("T3.7: full overlap forwards instead of waiting",
          (dut.sq_forward_valid == 1'b1) && (dut.lq_forward_fire === 1'b1) &&
          (dut.lq_mem_req_fire === 1'b0));
    check("T3.7: forward drives no dmem request", dmem_valid === 1'b0);
    @(posedge clk); @(negedge clk);
    check("T3.7: completion = winner data + load identity",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(4)) &&
          (lq_complete.result == 32'hA5A5_A5A5));
    check("T3.7: forward leaves no outstanding request",
          dut.lq_out_count_q == 2'd0);
    le = dut.lq_entry_q[lq_t];
    check("T3.7: executed one-shot",
          le.executed && (dut.lq_forward_fire === 1'b0));
    check("T3.7: store awaits its normal drain", dut.sq_count_q == 1);
    commit_one(rob_idx_t'(2), 1'b1);
    check("T3.7: store drains at commit as usual", dut.sq_count_q == '0);

    // selector SKIPS an addressless older load; scan uses the
    // CANDIDATE's age (an in-between store still blocks the younger pick).
    // The unknown store is allocated BEFORE the younger load's deposit so
    // the live launcher cannot fire it in the gap.
    flush_all();
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(7), MEM_W, lq_t);  // older, no addr
    lq_alloc_one(rob_idx_t'(8), phys_reg_t'(9), MEM_W, lq_t);  // younger
    sq_alloc_one(rob_idx_t'(6), sq_t);              // between the two loads
    lq_dep(lq_t, 32'h0000_0300, 1'b0);              // younger addr arrives last
    #1;
    check("T3.8: skip to younger qualifying load",
          dut.lq_select_valid && (dut.lq_select_idx == lq_t));
    check("T3.8: candidate-age scan — unknown store rob6 blocks rob8 load",
          dut.lq_mem_safe == 1'b0);
    sq_dep(sq_t, 32'h0000_0400, 4'b1111, 1'b0);     // disjoint
    #1;
    check("T3.8: disjoint deposit frees the skipped pick",
          dut.lq_mem_safe == 1'b1);

    // wrapped ring ages (head = 30) — width discipline of the compare
    flush_all();
    @(negedge clk);
    rob_head_idx = rob_idx_t'(30);
    sq_alloc_one(rob_idx_t'(31), sq_t);             // age 1, undeposited
    lq_alloc_one(rob_idx_t'(1),  phys_reg_t'(5), MEM_W, lq_t); // age 3
    lq_dep(lq_t, 32'h0000_0100, 1'b0);
    #1;
    check("T3.9: wrapped unknown older blocks", dut.lq_mem_safe == 1'b0);
    sq_dep(sq_t, 32'h0000_0200, 4'b1111, 1'b0);
    #1;
    check("T3.9: wrapped disjoint safe", dut.lq_mem_safe == 1'b1);
    @(negedge clk);
    rob_head_idx = '0;

    // many older stores — ONE unknown among deposited still blocks
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);
    sq_dep(sq_t, 32'h0000_0200, 4'b1111, 1'b0);     // known, disjoint
    sq_alloc_one(rob_idx_t'(4), sq_t);              // unknown
    lq_alloc_one(rob_idx_t'(8), phys_reg_t'(5), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);
    #1;
    check("T3.10: one unknown among many blocks", dut.lq_mem_safe == 1'b0);
    sq_dep(sq_t, 32'h0000_0104, 4'b1111, 1'b0);     // known, adjacent word
    #1;
    check("T3.10: all known + disjoint -> safe", dut.lq_mem_safe == 1'b1);

    // ============ launch fire, port ownership, executed ============
    // The delayed-response case comes last: it leaves the outstanding FIFO
    // occupied for the following response-consumption checks.

    // lone safe load fires a read with its address; one-shot via
    // executed; tied-high rvalid means no outstanding
    flush_all();
    lq_alloc_one(rob_idx_t'(2), phys_reg_t'(5), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0140, 1'b0);
    #1;
    check("T4.1: req fires", dut.lq_mem_req_fire === 1'b1);
    check("T4.1: port = read of the load addr",
          (dmem_valid === 1'b1) && (dmem_we === 1'b0) &&
          (dmem_addr == 32'h0000_0140));
    @(posedge clk); @(negedge clk);
    le = dut.lq_entry_q[lq_t];
    check("T4.1: executed set at launch", le.executed == 1'b1);
    check("T4.1: fire is one-shot (executed excludes)",
          (dut.lq_select_valid == 1'b0) && (dut.lq_mem_req_fire === 1'b0));
    check("T4.1: same-cycle response -> no outstanding",
          dut.lq_out_count_q == 2'd0);
    // A same-cycle completion must carry the launched identity. No
    // outstanding record is retained, so check the completion mailbox.
    check("T4.1: same-cycle completion carries the launched identity",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(2)));

    // a launched load still pops at its own commit
    commit_one(rob_idx_t'(2), 1'b0);
    check("T4.2: launched load pops at commit", dut.lq_count_q == '0);

    // an occupied mailbox is the LEGAL launch hold (the always-ready
    // environment contract forbids ready-low stimulus, and its tripwire
    // enforces that on the TB too); drain wins the collision cycle; the
    // load takes the port the cycle after.
    flush_all();
    lq_alloc_one(rob_idx_t'(2), phys_reg_t'(5), MEM_W, lq_t);   // primer
    lq_dep(lq_t, 32'h0000_0300, 1'b0);            // completes -> mailbox held
    @(posedge clk); @(negedge clk);
    sq_alloc_one(rob_idx_t'(4), sq_t);
    sq_dep(sq_t, 32'h0000_0200, 4'b1111, 1'b0);
    lq_alloc_one(rob_idx_t'(6), phys_reg_t'(6), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);
    #1;
    check("T4.3: occupied mailbox holds the launch",
          lq_complete.valid && (dut.lq_mem_req_fire === 1'b0));
    // drain-or-empty: the grant releases the launch hold same-cycle,
    // so the store-drain collision is staged ON the grant cycle.
    @(negedge clk); cdb_grant_lq = 1'b1;          // release the mailbox
    rob_head_idx = rob_idx_t'(4); commit_fire = 2'b01; commit_is_store = 2'b01;
    #1;
    check("T4.3: drain wins the collision cycle",
          (dut.sq_drain_fire === 1'b1) && (dut.lq_mem_req_fire === 1'b0) &&
          (dmem_we === 1'b1) && (dmem_addr == 32'h0000_0200));
    @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0;
    commit_fire = 2'b00; commit_is_store = 2'b00;
    #1;
    check("T4.3: load takes the port after the drain",
          (dut.lq_mem_req_fire === 1'b1) && (dmem_we === 1'b0) &&
          (dmem_addr == 32'h0000_0100));

    // no launch on a recovery cycle (survivor fires after); no launch
    // on a trap-flush cycle. Same mailbox-hold, released into the event.
    flush_all();
    lq_alloc_one(rob_idx_t'(2), phys_reg_t'(5), MEM_W, lq_t);   // primer
    lq_dep(lq_t, 32'h0000_0300, 1'b0);
    @(posedge clk); @(negedge clk);
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(6), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);            // mailbox-held
    // drain-or-empty: grant and recovery on the SAME cycle -- the
    // released launch must still honor the recovery embargo.
    @(negedge clk); cdb_grant_lq = 1'b1;
    branch_recover_req = 1'b1; recover_rob_idx = rob_idx_t'(6);
    #1;
    check("T4.4: no launch on the recovery cycle",
          dut.lq_mem_req_fire === 1'b0);
    @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0;
    branch_recover_req = 1'b0;
    #1;
    check("T4.4: surviving load fires after recovery",
          (dut.lq_count_q == 2) && (dut.lq_mem_req_fire === 1'b1));
    flush_all();
    lq_alloc_one(rob_idx_t'(2), phys_reg_t'(5), MEM_W, lq_t);   // primer
    lq_dep(lq_t, 32'h0000_0300, 1'b0);
    @(posedge clk); @(negedge clk);
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(6), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);
    // Same restage for the flush embargo: grant and flush share the cycle.
    @(negedge clk); cdb_grant_lq = 1'b1; trap_flush = 1'b1;
    #1;
    check("T4.4: no launch on the flush cycle",
          dut.lq_mem_req_fire === 1'b0);
    @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0; trap_flush = 1'b0;
    #1;
    check("T4.4: flush cleared the queue (nothing to fire)",
          (dut.lq_count_q == '0) && (dut.lq_mem_req_fire === 1'b0));

    // A delayed response preserves its launch snapshot in the outstanding
    // FIFO. A second load can launch before either response returns.
    flush_all();
    @(negedge clk); dmem_rvalid = 1'b0;
    lq_alloc_one(rob_idx_t'(2), phys_reg_t'(5), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);
    @(posedge clk); @(negedge clk);
    check("T4.5: delayed response raises outstanding",
          dut.lq_out_count_q == 2'd1);
    le = dut.lq_out_head_entry;
    check("T4.5: snapshot = launched load", le.rob_idx == rob_idx_t'(2));
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(6), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0180, 1'b0);
    #1;
    // K=2 permits the second launch behind the first. A third request must
    // wait until outstanding capacity becomes available.
    check("T4.5: second launch pipelines behind the first (A1.3)",
          (dut.lq_mem_req_fire === 1'b1) && (dut.lq_out_count_q === 2'd1));
    repeat (3) @(negedge clk);
    check("T5.1: both responses pending, in order",
          dut.lq_out_count_q == 2'd2);

    dmem_rdata = 32'hA1B2_C3D4;
    dmem_rvalid = 1'b1;
    #1;
    check("T5.1: delayed response fires only with outstanding",
          dut.lq_mem_resp_fire === 1'b1);
    @(posedge clk); @(negedge clk);
    dmem_rvalid = 1'b0;
    check("T5.1: response drained the HEAD only",
          dut.lq_out_count_q == 2'd1);
    check("T5.1: delayed response fills mailbox",
          lq_complete.valid &&
          (lq_complete.rob_idx == rob_idx_t'(2)) &&
          (lq_complete.rob_seq == rob_seq_t'(1)) &&
          (lq_complete.pdst == phys_reg_t'(5)) &&
          lq_complete.rd_wen &&
          (lq_complete.result == 32'hA1B2_C3D4));
    check("T5.1: load completion has no trap/CSR side effect",
          !lq_complete.trap_valid && !lq_complete.csr_we);

    @(posedge clk); @(negedge clk);
    check("T5.1: mailbox holds without CDB grant",
          lq_complete.valid &&
          (lq_complete.result == 32'hA1B2_C3D4));

    // The second response arrives while the first completion is ungranted.
    // It buffers behind the stable presented head; grants drain both
    // completions in request-acceptance order.
    dmem_rdata = 32'h1122_3344;
    dmem_rvalid = 1'b1;
    #1;
    check("T5.2: second response accepted against the held mailbox",
          dut.lq_mem_resp_fire === 1'b1);
    @(posedge clk); @(negedge clk);
    dmem_rvalid = 1'b0;
    check("T5.2: presented head is STILL the first completion",
          lq_complete.valid &&
          (lq_complete.rob_idx == rob_idx_t'(2)) &&
          (lq_complete.result == 32'hA1B2_C3D4));
    check("T5.2: both completions buffered",
          dut.lq_compl_count_q === 2'd2);
    @(negedge clk); cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0; #1;
    check("T5.2: second drains in order with its own metadata",
          lq_complete.valid &&
          (lq_complete.rob_idx == rob_idx_t'(4)) &&
          (lq_complete.pdst == phys_reg_t'(6)) &&
          (lq_complete.result == 32'h1122_3344));
    check("T5.2: no outstanding request remains",
          dut.lq_out_count_q == 2'd0);
    // Restore tied-high rvalid for the following immediate-response cases.
    dmem_rvalid = 1'b1;

    // tied-high rvalid without a paired request is not a response.
    flush_all();
    @(posedge clk); @(negedge clk);
    check("T5.3: idle rvalid creates no phantom completion",
          !lq_complete.valid && !dut.lq_mem_resp_fire);

    // response-side byte-lane extraction uses the launch snapshot.
    // Signed byte from byte lane 2: 0x80 -> 0xffff_ff80.
    @(negedge clk);
    lq_alloc_mem_unsigned = 1'b0;
    dmem_rdata = 32'h0080_0000;
    lq_alloc_one(rob_idx_t'(6), phys_reg_t'(7), MEM_B, lq_t);
    lq_dep(lq_t, 32'h0000_0102, 1'b0);
    @(posedge clk); @(negedge clk);
    check("T5.4: signed byte-lane extract",
          lq_complete.valid &&
          (lq_complete.rob_idx == rob_idx_t'(6)) &&
          (lq_complete.result == 32'hFFFF_FF80));

    // Unsigned halfword from the upper byte lanes: 0xabcd -> 0x0000_abcd.
    flush_all();
    @(negedge clk);
    lq_alloc_mem_unsigned = 1'b1;
    dmem_rdata = 32'hABCD_1234;
    lq_alloc_one(rob_idx_t'(8), phys_reg_t'(9), MEM_H, lq_t);
    lq_dep(lq_t, 32'h0000_0102, 1'b0);
    @(posedge clk); @(negedge clk);
    check("T5.4: unsigned halfword byte-lane extract",
          lq_complete.valid &&
          (lq_complete.rob_idx == rob_idx_t'(8)) &&
          (lq_complete.result == 32'h0000_ABCD));

    // a trap flush kills the queue entry but not the physical channel;
    // a response landing on that cycle still completes from the snapshot.
    flush_all();
    @(negedge clk);
    lq_alloc_mem_unsigned = 1'b0;
    dmem_rvalid = 1'b0;
    lq_alloc_one(rob_idx_t'(10), phys_reg_t'(11), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0200, 1'b0);
    @(posedge clk); @(negedge clk);
    check("T5.5: request outstanding before flush",
          dut.lq_out_count_q == 2'd1);
    trap_flush = 1'b1;
    dmem_rdata = 32'hCAFE_BABE;
    dmem_rvalid = 1'b1;
    #1;
    check("T5.5: response accepted on flush cycle",
          dut.lq_mem_resp_fire === 1'b1);
    @(posedge clk); @(negedge clk);
    trap_flush = 1'b0;
    check("T5.5: flush clears queue but response drains channel",
          (dut.lq_count_q == '0) &&
          (dut.lq_out_count_q == 2'd0));
    check("T5.5: killed load response retains snapshot identity",
          lq_complete.valid &&
          (lq_complete.rob_idx == rob_idx_t'(10)) &&
          (lq_complete.rob_seq == rob_seq_t'(1)) &&
          (lq_complete.result == 32'hCAFE_BABE));

    // drain-during-wait — capture-at-request frees the port while a
    // load response is outstanding (SS memory-response contract):
    // a store drain proceeds mid-wait, and the late response still completes
    // from the snapshot.
    @(negedge clk); cdb_grant_lq = 1'b1;
    @(posedge clk); @(negedge clk); cdb_grant_lq = 1'b0;
    check("T5.6: grant drains the held mailbox", lq_complete.valid == 1'b0);
    flush_all();
    @(negedge clk); dmem_rvalid = 1'b0;
    sq_alloc_one(rob_idx_t'(2), sq_t);              // will commit mid-wait
    sq_dep(sq_t, 32'h0000_0300, 4'b1111, 1'b0);     // disjoint from the load
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(12), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);              // launches, delayed resp
    @(posedge clk); @(negedge clk);
    check("T5.6: load outstanding, port released",
          (dut.lq_out_count_q == 2'd1) && (dmem_valid === 1'b0));
    rob_head_idx = rob_idx_t'(2); commit_fire = 2'b01; commit_is_store = 2'b01;
    #1;
    check("T5.6: drain proceeds during the wait",
          (dut.sq_drain_fire === 1'b1) && (dmem_valid === 1'b1) &&
          (dmem_we === 1'b1) && (dmem_addr == 32'h0000_0300));
    @(posedge clk); @(negedge clk);
    commit_fire = 2'b00; commit_is_store = 2'b00;
    dmem_rdata = 32'h1234_5678; dmem_rvalid = 1'b1;
    #1;
    check("T5.6: late response pairs with the outstanding load",
          dut.lq_mem_resp_fire === 1'b1);
    @(posedge clk); @(negedge clk);
    check("T5.6: completion from snapshot after mid-wait drain",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(4)) &&
          (lq_complete.result == 32'h1234_5678));
    check("T5.6: outstanding consumed", dut.lq_out_count_q == 2'd0);

    // ============ store-to-load forwarding directed battery =======

    // unknown older store blocks BOTH paths even with a younger full match
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);              // unknown address
    sq_alloc_one(rob_idx_t'(4), sq_t);
    sq_dep_d(sq_t, 32'h0000_0100, 4'b1111, 1'b0, 32'h1111_2222); // younger full
    lq_alloc_one(rob_idx_t'(6), phys_reg_t'(5), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);
    #1;
    check("F1: unknown older blocks launch AND forward",
          (dut.sq_unknown_present == 1'b1) && (dut.lq_mem_safe == 1'b0) &&
          (dut.sq_forward_valid == 1'b0) &&
          (dut.lq_mem_req_fire === 1'b0) && (dut.lq_forward_fire === 1'b0));

    // older partial match shadowed by a younger full winner -> forward,
    // and specifically the WINNER's data
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);
    sq_dep_d(sq_t, 32'h0000_0100, 4'b0011, 1'b0, 32'h1111_2222); // older partial
    sq_alloc_one(rob_idx_t'(4), sq_t);
    sq_dep_d(sq_t, 32'h0000_0100, 4'b1111, 1'b0, 32'h3333_4444); // younger full
    lq_alloc_one(rob_idx_t'(6), phys_reg_t'(5), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);
    #1;
    check("F2: shadowed partial does not block the younger full winner",
          (dut.sq_forward_valid == 1'b1) && (dut.lq_forward_fire === 1'b1) &&
          (dut.lq_mem_req_fire === 1'b0) && (dmem_valid === 1'b0));
    @(posedge clk); @(negedge clk);
    check("F2: the winner's data is forwarded",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(6)) &&
          (lq_complete.result == 32'h3333_4444));

    // Older FULL match + younger PARTIAL winner -> WAIT. Choosing the
    // youngest full match would wrongly bypass the younger partial store.
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);
    sq_dep_d(sq_t, 32'h0000_0100, 4'b1111, 1'b0, 32'h5555_6666); // older FULL
    sq_alloc_one(rob_idx_t'(4), sq_t);
    sq_dep_d(sq_t, 32'h0000_0100, 4'b0011, 1'b0, 32'h7777_8888); // younger PARTIAL
    lq_alloc_one(rob_idx_t'(6), phys_reg_t'(5), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);
    #1;
    check("F3: younger partial winner forces WAIT — shadowed full must not forward",
          (dut.sq_overlap_winner_valid == 1'b1) && (dut.sq_forward_valid == 1'b0) &&
          (dut.lq_mem_safe == 1'b0) &&
          (dut.lq_forward_fire === 1'b0) && (dut.lq_mem_req_fire === 1'b0));

    // winner independent of PHYSICAL entry order — rotate the SQ ring so
    // the YOUNGER store sits at a LOWER physical entry than the older one.
    flush_all();
    for (fw_i = 0; fw_i < 7; fw_i = fw_i + 1) begin
      sq_alloc_one(rob_idx_t'(2), sq_t);
      sq_dep(sq_t, 32'h0000_0400 + word_t'(fw_i * 4), 4'b1111, 1'b0);
      commit_one(rob_idx_t'(2), 1'b1);
    end
    sq_alloc_one(rob_idx_t'(20), sq_t);             // physical entry 7, OLDER
    sq_dep_d(sq_t, 32'h0000_0100, 4'b1111, 1'b0, 32'hAAAA_0001);
    sq_alloc_one(rob_idx_t'(22), sq_t);             // wraps to entry 0, YOUNGER
    sq_dep_d(sq_t, 32'h0000_0100, 4'b1111, 1'b0, 32'hBBBB_0002);
    lq_alloc_one(rob_idx_t'(24), phys_reg_t'(5), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);
    #1;
    check("F4: youngest winner despite inverted entry order",
          (dut.sq_forward_valid == 1'b1) &&
          (dut.sq_forward_idx == sq_idx_t'(0)));
    @(posedge clk); @(negedge clk);
    check("F4: younger store's data wins across the ring wrap",
          lq_complete.valid && (lq_complete.result == 32'hBBBB_0002));

    // ROB-age wraparound — head=30, stores at rob 31 (age 1) and rob 1
    // (age 3), load rob 3 (age 5); the mod-32 youngest must win
    flush_all();
    @(negedge clk); rob_head_idx = rob_idx_t'(30);
    sq_alloc_one(rob_idx_t'(31), sq_t);
    sq_dep_d(sq_t, 32'h0000_0100, 4'b1111, 1'b0, 32'hCCCC_0003);
    sq_alloc_one(rob_idx_t'(1), sq_t);
    sq_dep_d(sq_t, 32'h0000_0100, 4'b1111, 1'b0, 32'hDDDD_0004);
    lq_alloc_one(rob_idx_t'(3), phys_reg_t'(5), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);
    #1;
    check("F5: wrapped-age winner selected", dut.sq_forward_valid == 1'b1);
    @(posedge clk); @(negedge clk);
    check("F5: wrapped-age winner's data forwarded",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(3)) &&
          (lq_complete.result == 32'hDDDD_0004));
    @(negedge clk); rob_head_idx = '0;

    // byte-lane extraction on the FORWARD path — signed/unsigned byte/half.
    // Store word 80FF_7F01: bytes {01,7F,FF,80}, halves {7F01, 80FF}.
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);
    sq_dep_d(sq_t, 32'h0000_0100, 4'b1111, 1'b0, 32'h80FF_7F01);
    lq_alloc_mem_unsigned = 1'b0;
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(5), MEM_B, lq_t);
    lq_dep(lq_t, 32'h0000_0102, 1'b0);              // byte FF, signed
    @(posedge clk); @(negedge clk);
    check("F6: signed byte forward sign-extends",
          lq_complete.valid && (lq_complete.result == 32'hFFFF_FFFF));
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);
    sq_dep_d(sq_t, 32'h0000_0100, 4'b1111, 1'b0, 32'h80FF_7F01);
    lq_alloc_mem_unsigned = 1'b1;
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(5), MEM_B, lq_t);
    lq_dep(lq_t, 32'h0000_0103, 1'b0);              // byte 80, unsigned
    @(posedge clk); @(negedge clk);
    check("F6: unsigned byte forward zero-extends",
          lq_complete.valid && (lq_complete.result == 32'h0000_0080));
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);
    sq_dep_d(sq_t, 32'h0000_0100, 4'b1111, 1'b0, 32'h80FF_7F01);
    lq_alloc_mem_unsigned = 1'b0;
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(5), MEM_H, lq_t);
    lq_dep(lq_t, 32'h0000_0102, 1'b0);              // half 80FF, signed
    @(posedge clk); @(negedge clk);
    check("F6: signed half forward sign-extends",
          lq_complete.valid && (lq_complete.result == 32'hFFFF_80FF));
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);
    sq_dep_d(sq_t, 32'h0000_0100, 4'b1111, 1'b0, 32'h80FF_7F01);
    lq_alloc_mem_unsigned = 1'b1;
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(5), MEM_H, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);              // half 7F01, unsigned
    @(posedge clk); @(negedge clk);
    check("F6: unsigned half forward zero-extends",
          lq_complete.valid && (lq_complete.result == 32'h0000_7F01));
    lq_alloc_mem_unsigned = 1'b0;

    // forward fires WHILE the drain owns the dmem port — the forward
    // does not need the port, so both proceed in one cycle
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);              // head store: drain victim
    sq_dep(sq_t, 32'h0000_0200, 4'b1111, 1'b0);
    sq_alloc_one(rob_idx_t'(4), sq_t);              // forward source
    sq_dep_d(sq_t, 32'h0000_0100, 4'b1111, 1'b0, 32'hEEEE_0005);
    lq_alloc_one(rob_idx_t'(6), phys_reg_t'(5), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);              // forward window opens
    rob_head_idx = rob_idx_t'(2); commit_fire = 2'b01; commit_is_store = 2'b01;
    #1;
    check("F7: drain and forward proceed in one cycle",
          (dut.sq_drain_fire === 1'b1) && (dut.lq_forward_fire === 1'b1) &&
          (dmem_we === 1'b1) && (dmem_addr == 32'h0000_0200));
    @(posedge clk); @(negedge clk);
    commit_fire = 2'b00; commit_is_store = 2'b00;
    check("F7: forward completed during the drain",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(6)) &&
          (lq_complete.result == 32'hEEEE_0005));
    check("F7: drain popped, source store remains", dut.sq_count_q == 1);

    // an outstanding response blocks a forward; then the occupied
    // mailbox blocks it; the CDB grant frees it
    flush_all();
    @(negedge clk); dmem_rvalid = 1'b0;
    sq_alloc_one(rob_idx_t'(2), sq_t);              // forward source for load B
    sq_dep_d(sq_t, 32'h0000_0100, 4'b1111, 1'b0, 32'hDDDD_0099);
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(5), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0300, 1'b0);              // load A: launches, waits
    @(posedge clk); @(negedge clk);
    check("F8: load A outstanding", dut.lq_out_count_q == 2'd1);
    lq_alloc_one(rob_idx_t'(6), phys_reg_t'(6), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);              // load B: forwardable
    #1;
    check("F8: outstanding response blocks the forward",
          (dut.sq_forward_valid == 1'b1) && (dut.lq_forward_fire === 1'b0));
    @(negedge clk); dmem_rdata = 32'h5A5A_0000; dmem_rvalid = 1'b1;
    @(posedge clk); @(negedge clk);
    check("F8: load A completed by its late response",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(4)) &&
          (lq_complete.result == 32'h5A5A_0000) &&
          (dut.lq_out_count_q == 2'd0));
    #1;
    check("F8: occupied mailbox blocks the forward",
          dut.lq_forward_fire === 1'b0);
    // drain-or-empty: the forward fires ON the grant cycle.
    @(negedge clk); cdb_grant_lq = 1'b1;
    #1;
    check("F8: grant frees the forward", dut.lq_forward_fire === 1'b1);
    @(posedge clk); @(negedge clk); cdb_grant_lq = 1'b0;
    check("F8: load B forwarded after the grant",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(6)) &&
          (lq_complete.result == 32'hDDDD_0099));

    // recovery/flush cycles suppress a new forward; a surviving
    // candidate forwards the cycle after recovery
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);
    sq_dep_d(sq_t, 32'h0000_0100, 4'b1111, 1'b0, 32'hCAFE_0001);
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(5), MEM_W, lq_t);
    @(negedge clk);
    lq_deposit_fire = 1'b1; lq_deposit_idx = lq_t;
    lq_deposit_addr = 32'h0000_0100; lq_deposit_inert = 1'b0;
    @(posedge clk); @(negedge clk);
    lq_deposit_fire = 1'b0;
    branch_recover_req = 1'b1; recover_rob_idx = rob_idx_t'(8);
    #1;
    check("F9: recovery cycle suppresses the forward",
          (dut.sq_forward_valid == 1'b1) && (dut.lq_forward_fire === 1'b0));
    @(posedge clk); @(negedge clk);
    branch_recover_req = 1'b0;
    #1;
    check("F9: surviving candidate forwards after recovery",
          dut.lq_forward_fire === 1'b1);
    @(posedge clk); @(negedge clk);
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);
    sq_dep(sq_t, 32'h0000_0100, 4'b1111, 1'b0);
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(5), MEM_W, lq_t);
    @(negedge clk);
    lq_deposit_fire = 1'b1; lq_deposit_idx = lq_t;
    lq_deposit_addr = 32'h0000_0100; lq_deposit_inert = 1'b0;
    @(posedge clk); @(negedge clk);
    lq_deposit_fire = 1'b0;
    trap_flush = 1'b1;
    #1;
    check("F9: flush cycle suppresses the forward",
          dut.lq_forward_fire === 1'b0);
    @(posedge clk); @(negedge clk);
    trap_flush = 1'b0;
    check("F9: flush killed the candidate — nothing completed",
          (dut.lq_count_q == '0) && (lq_complete.valid == 1'b0));

    // SD1: the reachability delta.  A store with a ready base and late data
    // deposits its address, while a younger same-address load sees a
    // full-cover winner whose payload is not yet valid.  Neither memory nor
    // forwarding may progress until the accepted producer beat arrives.
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);              // data_preg = p42
    sq_dep_addr_only(sq_t, 32'h0000_0100, 4'b1111);
    se = dut.sq_entry_q[sq_t];
    check("SD1: address-only store state entered",
          se.valid && se.addr_valid && !se.data_valid &&
          se.deferred_pending && (se.data_preg == phys_reg_t'(42)));

    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(5), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);
    #1;
    check("SD1: pending-data store is the full-cover winner",
          dut.sq_overlap_winner_valid &&
          !dut.sq_overlap_winner_data_valid &&
          ((dut.sq_overlap_winner_be & dut.load_be) == dut.load_be));
    check("SD1: pending full-cover winner blocks both load paths",
          !dut.lq_mem_safe && !dut.sq_forward_valid &&
          !dut.lq_mem_req_fire && !dut.lq_forward_fire);

    // The producer result is raw architectural store data.  The SQ owns the
    // byte-lane alignment on capture; this word case also makes any accidental
    // early zero-forward architecturally visible.
    sq_data_wb_fire[0] = 1'b1;
    sq_data_wb_pdst[0] = phys_reg_t'(42);
    sq_data_wb_value[0] = 32'hDEAD_BEEF;
    #1;
    check("SD1: accepted CDB beat names the pending store row",
          dut.sq_data_wb_match[0][sq_t]);
    @(posedge clk); @(negedge clk);
    sq_data_wb_fire[0] = 1'b0;
    se = dut.sq_entry_q[sq_t];
    check("SD1: late data captured without completing early",
          se.data_valid && se.deferred_pending &&
          (se.data == 32'hDEAD_BEEF));
    check("SD1: load forwards only after data capture",
          dut.sq_forward_valid && dut.lq_forward_fire &&
          !dut.lq_mem_req_fire && !dmem_valid);

    // One edge consumes the load and loads the registered deferred-store
    // completion holder.  Hold it for an extra edge to exercise stability.
    @(posedge clk); @(negedge clk);
    check("SD1: delayed load receives the real store value",
          lq_complete.valid &&
          (lq_complete.rob_idx == rob_idx_t'(4)) &&
          (lq_complete.result == 32'hDEAD_BEEF));
    check("SD1: deferred store completion carries exact identity",
          sq_complete.valid &&
          (sq_complete.rob_idx == rob_idx_t'(2)) &&
          (sq_complete.rob_seq == rob_seq_t'(3)) &&
          !sq_complete.rd_wen && !sq_complete.trap_valid &&
          (sq_complete.result == 32'h0000_0100));
    @(posedge clk); @(negedge clk);
    check("SD1: unaccepted deferred completion remains stable",
          sq_complete.valid &&
          (sq_complete.rob_idx == rob_idx_t'(2)) &&
          (sq_complete.result == 32'h0000_0100));

    sq_complete_accept = 1'b1;
    @(posedge clk); @(negedge clk);
    sq_complete_accept = 1'b0;
    se = dut.sq_entry_q[sq_t];
    check("SD1: CDB acceptance marks exactly one store complete",
          !sq_complete.valid && !se.deferred_pending && se.data_valid);

    // Once the ROB can retire it, the complete store drains the captured
    // value.  This is the end-to-end consequence of the new split lifetime.
    rob_head_idx = rob_idx_t'(2);
    commit_fire = 2'b01; commit_is_store = 2'b01;
    #1;
    check("SD1: completed store drains the captured payload",
          dmem_valid && dmem_we &&
          (dmem_addr == 32'h0000_0100) &&
          (dmem_wdata == 32'hDEAD_BEEF));
    @(posedge clk); @(negedge clk);
    commit_fire = 2'b00; commit_is_store = 2'b00;
    check("SD1: completed store leaves the SQ", dut.sq_count_q == '0);

    // SD2: recovery composes an accepted late-data beat into an older
    // surviving row while dropping the younger suffix.  Without this case a
    // fire-and-forget CDB value can be lost exactly on the recovery edge.
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);              // survivor, p42
    sq_dep_addr_only(sq_t, 32'h0000_0200, 4'b1111);
    sq_alloc_one(rob_idx_t'(8), sq_t2);             // killed by branch 6
    sq_dep_addr_only(sq_t2, 32'h0000_0204, 4'b1111);
    @(negedge clk);
    rob_head_idx = rob_idx_t'(0);
    recover_rob_idx = rob_idx_t'(6);
    branch_recover_req = 1'b1;
    sq_data_wb_fire[0] = 1'b1;
    sq_data_wb_pdst[0] = phys_reg_t'(42);
    sq_data_wb_value[0] = 32'h1234_5678;
    #1;
    check("SD2: survivor late-data beat is live on recovery edge",
          dut.sq_data_wb_match[0][sq_t]);
    @(posedge clk); @(negedge clk);
    branch_recover_req = 1'b0;
    sq_data_wb_fire[0] = 1'b0;
    se = dut.sq_entry_q[sq_t];
    check("SD2: recovery retained accepted data in survivor",
          (dut.sq_count_q == 1) && se.valid && se.addr_valid &&
          se.data_valid && se.deferred_pending &&
          (se.data == 32'h1234_5678));
    se = dut.sq_entry_q[sq_t2];
    check("SD2: recovery killed the younger address-only row", !se.valid);
    @(posedge clk); @(negedge clk);
    check("SD2: recovered survivor reaches deferred completion",
          sq_complete.valid &&
          (sq_complete.rob_idx == rob_idx_t'(2)) &&
          (sq_complete.result == 32'h0000_0200));

    // SD3: lane 1 of the widened accepted-CDB seam, plus sub-word placement.
    // The producer carries raw 0xAB; an SB at +1 must store 0000_AB00 and an
    // unsigned byte load must extract 0000_00AB after the pending-data wait.
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);              // data_preg = p42
    sq_dep_addr_only(sq_t, 32'h0000_0101, 4'b0010);
    lq_alloc_mem_unsigned = 1'b1;
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(5), MEM_B, lq_t);
    lq_dep(lq_t, 32'h0000_0101, 1'b0);
    #1;
    check("SD3: byte load waits for lane-1 late store data",
          dut.sq_overlap_winner_valid &&
          !dut.sq_overlap_winner_data_valid &&
          !dut.lq_mem_req_fire && !dut.lq_forward_fire);
    sq_data_wb_fire[1] = 1'b1;
    sq_data_wb_pdst[1] = phys_reg_t'(42);
    sq_data_wb_value[1] = 32'h0000_00AB;
    @(posedge clk); @(negedge clk);
    sq_data_wb_fire[1] = 1'b0;
    se = dut.sq_entry_q[sq_t];
    check("SD3: lane-1 capture applies byte placement",
          se.data_valid && (se.data == 32'h0000_AB00));
    check("SD3: byte load forwards after lane-1 capture",
          dut.sq_forward_valid && dut.lq_forward_fire);
    @(posedge clk); @(negedge clk);
    check("SD3: forwarded unsigned byte is exact",
          lq_complete.valid && (lq_complete.result == 32'h0000_00AB));
    lq_alloc_mem_unsigned = 1'b0;

    // SD4: the one-shot producer beat may coincide with address deposit. The
    // LSQ must compose both inputs into the new row; waiting for a later snoop
    // would lose the value permanently because the accepted CDB beat does not
    // replay. Lane placement remains owned by the SQ capture path.
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);              // data_preg = p42
    @(negedge clk);
    sq_deposit_fire = 1'b1;
    sq_deposit_idx = sq_t;
    sq_deposit_addr = 32'h0000_0102;
    sq_deposit_data = '0;
    sq_deposit_data_valid = 1'b0;
    sq_deposit_be = 4'b0100;
    sq_deposit_inert = 1'b0;
    sq_deposit_deferred_pending = 1'b1;
    sq_data_wb_fire[0] = 1'b1;
    sq_data_wb_pdst[0] = phys_reg_t'(42);
    sq_data_wb_value[0] = 32'h0000_00CD;
    #1;
    check("SD4: simultaneous address/CDB match is recognized",
          dut.sq_deposit_wb_match[0]);
    @(posedge clk); @(negedge clk);
    sq_deposit_fire = 1'b0;
    sq_data_wb_fire[0] = 1'b0;
    se = dut.sq_entry_q[sq_t];
    check("SD4: simultaneous address/CDB inputs compose into one row",
          se.addr_valid && se.data_valid && se.deferred_pending &&
          (se.addr == 32'h0000_0102) &&
          (se.data == 32'h00CD_0000));
    @(posedge clk); @(negedge clk);
    check("SD4: composed row reaches deferred completion",
          sq_complete.valid &&
          (sq_complete.rob_idx == rob_idx_t'(2)) &&
          (sq_complete.result == 32'h0000_0102));

    // an INERT full-overlap store never forwards and never blocks —
    // the load launches to memory (sound: the inert store's trap flushes it)
    flush_all();
    sq_alloc_one(rob_idx_t'(2), sq_t);
    sq_dep(sq_t, 32'h0000_0101, 4'b1111, 1'b1);     // inert, overlapping
    lq_alloc_one(rob_idx_t'(4), phys_reg_t'(5), MEM_W, lq_t);
    lq_dep(lq_t, 32'h0000_0100, 1'b0);
    #1;
    check("F10: inert store invisible to forwarding — load launches",
          (dut.lq_mem_safe == 1'b1) && (dut.sq_forward_valid == 1'b0) &&
          (dut.lq_mem_req_fire === 1'b1));

    // ================= directed section =======
    // The D-side under withheld ready, driven at the LSQ boundary where the
    // recovery window is a controllable input — which is where 3.3a's
    // condition 3 (a MULTI-CYCLE recovery window across the acceptance
    // edge) can be entered deterministically.

    // Clean slate: flush both queues and drain any ungranted completion
    // left by the earlier sections (a held completion blocks the load
    // intent through lq_complete_ready, which stores never consult).
    @(negedge clk); trap_flush = 1'b1; @(posedge clk); @(negedge clk);
    trap_flush = 1'b0; idle_inputs();
    @(negedge clk); cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0;
    repeat (2) @(negedge clk);

    // ---- store presented through a stall; pop at acceptance; the
    // deferred record on the plain path stays clear.
    sq_alloc_one(rob_idx_t'(8), m4_sq_ticket);
    sq_dep(m4_sq_ticket, 32'h0000_0140, 4'hF, 1'b0);
    @(negedge clk);
    dmem_ready = 1'b0;
    rob_head_idx = rob_idx_t'(8);
    m4_want = 2'b01; m4_want_override = 1'b1;
    m4_count_before = dut.sq_count_q;
    repeat (3) begin
      @(negedge clk); #1;
      check("M4.1 store presented through the stall",
            dmem_valid && dmem_we && (dmem_addr == 32'h0000_0140) &&
            (dmem_wdata == 32'hA5A5_A5A5));
      check("M4.1 no pop while unaccepted", dut.sq_count_q == m4_count_before);
      m4_hold_cycles = m4_hold_cycles + 1;
    end
    dmem_ready = 1'b1;
    @(posedge clk);                              // acceptance edge
    @(negedge clk); #1;
    check("M4.1 pop exactly at acceptance",
          dut.sq_count_q == (m4_count_before - 1));
    check("M4.1 acceptance without retire raises the deferred record",
          sq_accept_deferred_o === 1'b1);
    check("M4.1 no re-present inside the deferred window",
          (dmem_valid === 1'b0) && (dmem_we === 1'b0));
    commit_one(rob_idx_t'(8), 1'b1);             // deferred retirement
    m4_want = 2'b00; m4_want_override = 1'b0;
    #1;
    check("M4.1 retirement clears the deferred record",
          sq_accept_deferred_o === 1'b0);

    // ---- acceptance ON a multi-cycle recovery window.
    sq_alloc_one(rob_idx_t'(9), m4_sq_ticket);
    sq_dep(m4_sq_ticket, 32'h0000_0148, 4'hF, 1'b0);
    @(negedge clk);
    dmem_ready = 1'b0;
    rob_head_idx = rob_idx_t'(9);
    m4_want = 2'b01; m4_want_override = 1'b1;
    repeat (2) @(negedge clk);
    // Multi-cycle recovery window: a YOUNGER branch (ring-age 2) recovers
    // while the head store is presented-held. The store must survive, stay
    // presented, and its acceptance mid-window must pop and defer.
    branch_recover_req = 1'b1;
    recover_rob_idx    = rob_idx_t'(11);
    @(negedge clk); #1;
    check("M4.2 held store presented through the recovery window",
          dmem_valid && dmem_we && (dmem_wdata == 32'hA5A5_A5A5));
    m4_count_before = dut.sq_count_q;
    dmem_ready = 1'b1;
    @(posedge clk);                              // acceptance ON the window
    if (branch_recover_req) m4_accept_in_recovery = m4_accept_in_recovery + 1;
    @(negedge clk); #1;
    check("M4.2 pop on the recovery-edge acceptance",
          dut.sq_count_q == (m4_count_before - 1));
    check("M4.2 deferred record set by the recovery-edge acceptance",
          sq_accept_deferred_o === 1'b1);
    @(negedge clk);
    branch_recover_req = 1'b0;                   // window closes
    @(negedge clk);
    commit_one(rob_idx_t'(9), 1'b1);             // deferred retirement
    m4_want = 2'b00; m4_want_override = 1'b0;
    #1;
    check("M4.2 deferred retirement lands after the window",
          sq_accept_deferred_o === 1'b0);

    // ---- load held through a stall AND a recovery that kills it;
    // the launch runs from the SNAPSHOT and completes with the snapshot
    // identity (the semantics, unchanged) for downstream rejection.
    @(negedge clk);
    dmem_ready = 1'b0; dmem_rvalid = 1'b0;   // stall FIRST, or the load
    rob_head_idx = rob_idx_t'(12);           // launches the moment it lands
    lq_alloc_one(rob_idx_t'(12), phys_reg_t'(21), MEM_W, m4_lq_ticket);
    lq_dep(m4_lq_ticket, 32'h0000_0150, 1'b0);
    repeat (2) begin
      @(negedge clk); #1;
      check("M4.3 load presented through the stall",
            dmem_valid && !dmem_we && (dmem_addr == 32'h0000_0150));
    end
    // Multi-cycle recovery killing the held load (ring-age 0 >= age 0
    // requires an OLDER branch: recover at the load's own head).
    branch_recover_req = 1'b1;
    recover_rob_idx    = rob_idx_t'(12);
    rob_head_idx       = rob_idx_t'(12);
    repeat (2) begin
      @(negedge clk); #1;
      check("M4.3 killed held load stays presented (no withdrawal)",
            dmem_valid === 1'b1);
    end
    // Acceptance while recovery is still asserted must not revive the row.
    dmem_ready = 1'b1;
    @(posedge clk);
    m4_held_launches = m4_held_launches + 1;
    @(negedge clk);
    le = dut.lq_entry_q[m4_lq_ticket];
    check("killed held acceptance during recovery cannot revive its row",
          !le.valid && !le.executed);
    branch_recover_req = 1'b0;
    dmem_rvalid = 1'b1; dmem_rdata = 32'h5151_5151;
    @(posedge clk); @(negedge clk); #1;
    dmem_rvalid = 1'b0;
    check("M4.3 completion carries the SNAPSHOT identity",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(12)) &&
          (lq_complete.result == 32'h5151_5151));
    @(negedge clk); cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0;
    check("M4.3 no re-request after the killed entry vanished",
          dmem_valid === 1'b0);

    // ---- arbitration lock — a store want arriving mid-hold must not
    // steal the port from the held load.
    @(negedge clk);
    dmem_ready = 1'b0; dmem_rvalid = 1'b0;   // stall before the load lands
    rob_head_idx = rob_idx_t'(16);
    lq_alloc_one(rob_idx_t'(16), phys_reg_t'(22), MEM_W, m4_lq_ticket);
    lq_dep(m4_lq_ticket, 32'h0000_0158, 1'b0);
    sq_alloc_one(rob_idx_t'(17), m4_sq_ticket);
    sq_dep(m4_sq_ticket, 32'h0000_015C, 4'hF, 1'b0);
    @(negedge clk); #1;
    check("M4.4 load owns the port", dmem_valid && !dmem_we);
    m4_want = 2'b01; m4_want_override = 1'b1;    // store want arrives mid-hold
    rob_head_idx = rob_idx_t'(17);
    repeat (2) begin
      @(negedge clk); #1;
      check("M4.4 lock: the held load keeps the port against the store want",
            dmem_valid && !dmem_we && (dmem_addr == 32'h0000_0158));
      m4_lock_cycles = m4_lock_cycles + 1;
    end
    dmem_ready = 1'b1;
    @(posedge clk); @(negedge clk);              // load accepted
    dmem_rvalid = 1'b1; dmem_rdata = 32'h6262_6262;
    @(posedge clk); @(negedge clk); #1;
    dmem_rvalid = 1'b0;
    check("M4.4 held load completes after the lock",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(16)));
    @(negedge clk); cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0; #1;
    // With ready high again, the store fired the moment the lock released —
    // during the load's response window — and
    // its acceptance without a same-cycle retire raised the deferred record.
    check("M4.4 store accepted the moment the lock released",
          sq_accept_deferred_o === 1'b1);
    commit_one(rob_idx_t'(17), 1'b1);
    m4_want = 2'b00; m4_want_override = 1'b0;
    #1;
    check("M4.4 deferred store retired", sq_accept_deferred_o === 1'b0);
    commit_one(rob_idx_t'(16), 1'b0);            // pop the completed load

    // ---- independence — a store accepted while a
    // load response is pending.
    lq_alloc_one(rob_idx_t'(20), phys_reg_t'(23), MEM_W, m4_lq_ticket);
    lq_dep(m4_lq_ticket, 32'h0000_0160, 1'b0);
    sq_alloc_one(rob_idx_t'(21), m4_sq_ticket);
    sq_dep(m4_sq_ticket, 32'h0000_0164, 4'hF, 1'b0);
    @(negedge clk);
    dmem_ready = 1'b1; dmem_rvalid = 1'b0;       // load launches, resp pends
    rob_head_idx = rob_idx_t'(20);
    @(posedge clk); @(negedge clk); #1;
    check("M4.5 load response outstanding", dut.lq_out_count_q === 2'd1);
    m4_want = 2'b01; m4_want_override = 1'b1;    // store wants mid-window
    rob_head_idx = rob_idx_t'(21);
    @(negedge clk); #1;
    // With ready high the acceptance beats a presentation check: the store
    // fired on the intervening edge WHILE the load response was pending —
    // 5.2 independence, evidenced by the deferred record coexisting with
    // the outstanding window.
    check("M4.5 store accepted while the load response is pending",
          (sq_accept_deferred_o === 1'b1) &&
          (dut.lq_out_count_q === 2'd1));
    if ((dut.lq_out_count_q != 2'd0) && (sq_accept_deferred_o === 1'b1))
      m4_store_during_outstanding = m4_store_during_outstanding + 1;
    commit_one(rob_idx_t'(21), 1'b1);
    m4_want = 2'b00; m4_want_override = 1'b0;
    dmem_rvalid = 1'b1; dmem_rdata = 32'h7373_7373;
    @(posedge clk); @(negedge clk); #1;
    dmem_rvalid = 1'b0;
    check("M4.5 pending load completes after the store's acceptance",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(20)) &&
          (lq_complete.result == 32'h7373_7373));
    @(negedge clk); cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0;
    commit_one(rob_idx_t'(20), 1'b0);            // pop the completed load

    // ---- a held launch whose response is SAME-CYCLE (the RESP_LATENCY=0
    // environment shape). Every held launch above delivers its response a cycle
    // late, so all of them route through the outstanding register, which
    // carries the launch snapshot. A same-cycle response never raises that
    // register, so the completion metadata must come from the HELD snapshot
    // directly — and the entry that happens to be selected at the launch cycle
    // is a different, LIVE load, so a wrong source lands real data under a real
    // load's identity rather than a harmless zero.
    @(negedge clk);
    flush_all();
    dmem_ready = 1'b0; dmem_rvalid = 1'b0;   // stall first: hold before launch
    rob_head_idx = rob_idx_t'(30);
    lq_alloc_one(rob_idx_t'(30), phys_reg_t'(30), MEM_W, m4_lq_ticket);
    lq_dep(m4_lq_ticket, 32'h0000_0180, 1'b0);
    @(negedge clk); #1;
    check("M4.6 load presented and held under the stall",
          dmem_valid && !dmem_we && (dmem_addr == 32'h0000_0180));
    // Kill the held load, so the launch-cycle select is NOT the held entry.
    branch_recover_req = 1'b1;
    recover_rob_idx    = rob_idx_t'(30);
    rob_head_idx       = rob_idx_t'(30);
    @(negedge clk); #1;
    branch_recover_req = 1'b0;
    // A live younger load takes over the select while the kill victim stays
    // presented (producer ownership).
    // A reused slot needs a fresh generation, just as real ROB allocation
    // supplies. Reusing the helper's default seq=1 would impersonate the
    // old held owner and cannot test the identity guard.
    lq_alloc_one(rob_idx_t'(31), phys_reg_t'(31), MEM_W, m4_lq_ticket, rob_seq_t'(2));
    lq_dep(m4_lq_ticket, 32'h0000_0184, 1'b0);
    @(negedge clk); #1;
    check("M4.6 a different live load is selected at the launch cycle",
          dut.lq_select_valid &&
          (dut.lq_select_entry.rob_idx == rob_idx_t'(31)));
    check("M4.6 killed held load still presented (no withdrawal)",
          dmem_valid && (dmem_addr == 32'h0000_0180));
    // A second recovery preserves the new owner across ROB wraparound.
    // The old held request still cannot mark that reused row as executed.
    branch_recover_req = 1'b1;
    recover_rob_idx = rob_idx_t'(0);
    rob_head_idx = rob_idx_t'(31);
    // Acceptance and response on the SAME recovery edge.
    dmem_ready = 1'b1; dmem_rvalid = 1'b1; dmem_rdata = 32'h9A9A_9A9A;
    #1;
    check("M4.6 held launch presented with a same-cycle response",
          (dut.lq_launch_held === 1'b1) && (dmem_rvalid === 1'b1));
    if (dut.lq_launch_held === 1'b1) m4_same_cycle_held = m4_same_cycle_held + 1;
    @(posedge clk);
    @(negedge clk); #1;
    dmem_ready = 1'b0; dmem_rvalid = 1'b0;
    branch_recover_req = 1'b0;
    check("M4.6 same-cycle response never raised the outstanding register",
          dut.lq_out_count_q === 2'd0);
    check("M4.6 completion carries the HELD SNAPSHOT identity, not the select",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(30)) &&
          (lq_complete.pdst == phys_reg_t'(30)) &&
          (lq_complete.result == 32'h9A9A_9A9A));
    check("M4.6 the live selected load was NOT completed in its place",
          lq_complete.rob_idx != rob_idx_t'(31));
    le = dut.lq_entry_q[m4_lq_ticket];
    check("F2 reused slot identity differs from held snapshot",
          le.valid && le.rob_seq != dut.dreq_load_entry_q.rob_seq &&
          m4_lq_ticket == dut.dreq_load_idx_q);
    check("F2 stale held acceptance did not mark the new owner", !le.executed);
    @(negedge clk); cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0;
    // Consequence: the new owner must still request, return its OWN value,
    // and retire. Merely checking the stale completion above cannot prove
    // that an erroneous executed mark did not silently suppress this load.
    dmem_ready = 1'b1; dmem_rvalid = 1'b1; dmem_rdata = 32'hB4B4_B4B4;
    #1;
    check("F2 reused load still requests its own address",
          dmem_valid && !dmem_we && dmem_addr == 32'h0000_0184);
    @(posedge clk); @(negedge clk); #1;
    dmem_ready = 1'b0; dmem_rvalid = 1'b0;
    le = dut.lq_entry_q[m4_lq_ticket];
    check("F2 new owner's actual launch marks executed", le.executed);
    check("F2 reused load completes its own identity and value",
          lq_complete.valid && lq_complete.rob_idx == rob_idx_t'(31) &&
          lq_complete.pdst == phys_reg_t'(31) &&
          lq_complete.result == 32'hB4B4_B4B4);
    @(negedge clk); cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0;
    commit_one(rob_idx_t'(31), 1'b0);
    le = dut.lq_entry_q[m4_lq_ticket];
    check("F2 reused load retires and clears executed", !le.valid && !le.executed);
    flush_all();

    // ---- the held slot CHANGES HANDS on the acceptance cycle. The
    // executed mark is written later in the lifecycle block than the pop and
    // the alloc, so if it does not stand down it wins over them: a retired
    // entry revived by the mark stays valid at an index the head has already
    // moved past, and the queue drifts silently until the head wraps onto it.
    @(negedge clk);
    flush_all();
    dmem_ready = 1'b0; dmem_rvalid = 1'b0;
    rob_head_idx = rob_idx_t'(34);
    lq_alloc_one(rob_idx_t'(34), phys_reg_t'(34), MEM_W, m4_lq_ticket);
    lq_dep(m4_lq_ticket, 32'h0000_0190, 1'b0);
    @(negedge clk); #1;
    check("M4.7 load presented and held", dmem_valid && !dmem_we);

    // Retire the load and accept its held request on the SAME edge.
    rob_head_idx    = rob_idx_t'(34);
    commit_fire     = 2'b01;
    commit_is_store = 2'b00;
    commit_is_load  = 2'b01;
    dmem_ready = 1'b1; dmem_rvalid = 1'b1; dmem_rdata = 32'hC7C7_C7C7;
    #1;
    check("M4.7 pop and held launch collide on one slot",
          (dut.lq_launch_held === 1'b1) && (dut.lq_pop_fire[0] === 1'b1) &&
          (dut.lq_pop_idx[0] === dut.dreq_load_idx_q));
    if (dut.lq_launch_held && dut.lq_pop_fire[0] &&
        (dut.lq_pop_idx[0] === dut.dreq_load_idx_q))
      m4_pop_mark_collision = m4_pop_mark_collision + 1;
    @(posedge clk);
    @(negedge clk); #1;
    commit_fire = 2'b00; commit_is_load = 2'b00;
    dmem_ready = 1'b0; dmem_rvalid = 1'b0;
    // Icarus-safe: whole-entry reads through an unrolled constant index,
    // never a dynamic member select. "No entry survives" is also the
    // stronger statement — a revived entry is valid at an index the head
    // has already moved past, so scanning all of them is the real check.
    m4_any_valid = 1'b0;
    for (m4_scan_i = 0; m4_scan_i < SS_LQ_DEPTH; m4_scan_i = m4_scan_i + 1) begin
      m4_scan_entry = dut.lq_entry_q[m4_scan_i];
      if (m4_scan_entry.valid) m4_any_valid = 1'b1;
    end
    check("M4.7 the retired entry is NOT revived by the executed mark",
          m4_any_valid === 1'b0);
    check("M4.7 the queue did not drift", dut.lq_count_q === '0);
    le = dut.lq_entry_q[m4_lq_ticket];
    check("F2 same-edge held pop also clears executed", !le.executed);
    @(negedge clk); cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0;
    flush_all();

    // ==== Outstanding FIFO directed battery (K=2) =====================
    // Entry counters require concurrent requests, same-edge handover,
    // recovery, completion buffering and backpressure to be exercised.

    // ---- two reads genuinely outstanding; in-order completion;
    // third read refused at K=2.
    @(negedge clk);
    flush_all();
    dmem_ready = 1'b1; dmem_rvalid = 1'b0;
    rob_head_idx = rob_idx_t'(8);
    lq_alloc_one(rob_idx_t'(8), phys_reg_t'(40), MEM_W, m4_lq_ticket);
    lq_dep(m4_lq_ticket, 32'h0000_0200, 1'b0);
    @(negedge clk); #1;
    check("M5.1 A launched, response pending (count 1)",
          dut.lq_out_count_q === 2'd1);
    lq_alloc_one(rob_idx_t'(9), phys_reg_t'(41), MEM_W, m4_lq_ticket);
    lq_dep(m4_lq_ticket, 32'h0000_0204, 1'b0);
    @(negedge clk); #1;
    check("M5.1 B accepted behind A: two reads outstanding",
          dut.lq_out_count_q === 2'd2);
    if (dut.lq_out_count_q === 2'd2)
      m5_two_outstanding = m5_two_outstanding + 1;
    lq_alloc_one(rob_idx_t'(10), phys_reg_t'(42), MEM_W, m4_lq_ticket);
    lq_dep(m4_lq_ticket, 32'h0000_0208, 1'b0);
    @(negedge clk); #1;
    check("M5.1 third read refused at K=2 (no presentation)",
          (dut.lq_out_count_q === 2'd2) && (dmem_valid === 1'b0));
    dmem_rdata = 32'hAAAA_0001; dmem_rvalid = 1'b1;
    @(negedge clk); #1;
    dmem_rvalid = 1'b0;
    check("M5.1 first response completed the HEAD (A), in order",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(8)) &&
          (lq_complete.result == 32'hAAAA_0001));
    check("M5.1 count fell behind the response", dut.lq_out_count_q === 2'd1);
    @(negedge clk); cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0;
    dmem_rdata = 32'hBBBB_0002; dmem_rvalid = 1'b1;
    @(negedge clk); #1;
    dmem_rvalid = 1'b0;
    check("M5.1 second response completed B, in order",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(9)) &&
          (lq_complete.result == 32'hBBBB_0002));
    @(negedge clk); cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0;
    dmem_rdata = 32'hCCCC_0003; dmem_rvalid = 1'b1;
    @(negedge clk); #1;
    dmem_rvalid = 1'b0;
    check("M5.1 C launched after the window drained and completed",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(10)) &&
          (lq_complete.result == 32'hCCCC_0003));
    @(negedge clk); cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0;
    commit_one(rob_idx_t'(8), 1'b0);
    commit_one(rob_idx_t'(9), 1'b0);
    commit_one(rob_idx_t'(10), 1'b0);

    // ---- a read accepted on the SAME edge the head's response
    // arrives (the latency-1 steady state). The completion must carry
    // the outstanding HEAD's identity, not the newly launching read's.
    @(negedge clk);
    flush_all();
    dmem_ready = 1'b1; dmem_rvalid = 1'b0;
    rob_head_idx = rob_idx_t'(12);
    lq_alloc_one(rob_idx_t'(12), phys_reg_t'(43), MEM_W, m4_lq_ticket);
    lq_dep(m4_lq_ticket, 32'h0000_0210, 1'b0);
    @(negedge clk); #1;
    check("M5.2 A outstanding", dut.lq_out_count_q === 2'd1);
    lq_alloc_one(rob_idx_t'(13), phys_reg_t'(44), MEM_W, m4_lq_ticket);
    lq_dep(m4_lq_ticket, 32'h0000_0214, 1'b0);
    // B's fire cycle: deliver A's response on the same edge.
    dmem_rdata = 32'h5A5A_0005; dmem_rvalid = 1'b1;
    #1;
    check("M5.2 acceptance and head response share the edge",
          (dut.lq_mem_req_fire === 1'b1) && (dut.lq_out_count_q === 2'd1));
    if (dut.lq_mem_req_fire && (dut.lq_out_count_q === 2'd1))
      m5_same_edge = m5_same_edge + 1;
    @(posedge clk); @(negedge clk); #1;
    dmem_rvalid = 1'b0;
    check("M5.2 same-edge pop+push kept the count at one",
          dut.lq_out_count_q === 2'd1);
    check("M5.2 completion carries the HEAD (A), not the launcher (B)",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(12)) &&
          (lq_complete.result == 32'h5A5A_0005));
    @(negedge clk); cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0;
    dmem_rdata = 32'h6B6B_0006; dmem_rvalid = 1'b1;
    @(negedge clk); #1;
    dmem_rvalid = 1'b0;
    check("M5.2 B then completed with its own identity",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(13)) &&
          (lq_complete.result == 32'h6B6B_0006));
    @(negedge clk); cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0;
    commit_one(rob_idx_t'(12), 1'b0);
    commit_one(rob_idx_t'(13), 1'b0);

    // ---- recovery kills the YOUNGER of two outstanding reads.
    // The FIFO is untouched by the kill; both responses complete with their
    // launch snapshots. The stale completion is rejected downstream.
    @(negedge clk);
    flush_all();
    dmem_ready = 1'b1; dmem_rvalid = 1'b0;
    rob_head_idx = rob_idx_t'(16);
    lq_alloc_one(rob_idx_t'(16), phys_reg_t'(45), MEM_W, m4_lq_ticket);
    lq_dep(m4_lq_ticket, 32'h0000_0218, 1'b0);
    @(negedge clk); #1;
    lq_alloc_one(rob_idx_t'(17), phys_reg_t'(46), MEM_W, m4_lq_ticket);
    lq_dep(m4_lq_ticket, 32'h0000_021C, 1'b0);
    @(negedge clk); #1;
    check("M5.3 two outstanding before the kill",
          dut.lq_out_count_q === 2'd2);
    recover_one(rob_idx_t'(17), rob_idx_t'(16));
    check("M5.3 kill left the outstanding FIFO intact",
          dut.lq_out_count_q === 2'd2);
    if (dut.lq_out_count_q === 2'd2) m5_kill_two = m5_kill_two + 1;
    dmem_rdata = 32'h1111_0007; dmem_rvalid = 1'b1;
    @(negedge clk); #1;
    dmem_rvalid = 1'b0;
    check("M5.3 surviving head completed live",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(16)) &&
          (lq_complete.result == 32'h1111_0007));
    @(negedge clk); cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0;
    dmem_rdata = 32'h2222_0008; dmem_rvalid = 1'b1;
    @(negedge clk); #1;
    dmem_rvalid = 1'b0;
    check("M5.3 killed read completed with its SNAPSHOT identity",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(17)) &&
          (lq_complete.result == 32'h2222_0008));
    @(negedge clk); cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0;
    check("M5.3 window drained clean", dut.lq_out_count_q === 2'd0);
    commit_one(rob_idx_t'(16), 1'b0);

    // ---- two completions buffered under a withheld CDB grant.
    // The presented head is stable until its own grant; the second drains
    // in order behind it.
    @(negedge clk);
    flush_all();
    dmem_ready = 1'b1; dmem_rvalid = 1'b0;
    rob_head_idx = rob_idx_t'(20);
    lq_alloc_one(rob_idx_t'(20), phys_reg_t'(47), MEM_W, m4_lq_ticket);
    lq_dep(m4_lq_ticket, 32'h0000_0220, 1'b0);
    @(negedge clk); #1;
    lq_alloc_one(rob_idx_t'(21), phys_reg_t'(48), MEM_W, m4_lq_ticket);
    lq_dep(m4_lq_ticket, 32'h0000_0224, 1'b0);
    @(negedge clk); #1;
    check("M5.4 two outstanding, grant withheld",
          dut.lq_out_count_q === 2'd2);
    dmem_rdata = 32'h3333_0009; dmem_rvalid = 1'b1;
    @(negedge clk);
    dmem_rdata = 32'h4444_000A;
    @(negedge clk); #1;
    dmem_rvalid = 1'b0;
    check("M5.4 both completions buffered",
          dut.lq_compl_count_q === 2'd2);
    if (dut.lq_compl_count_q === 2'd2) m5_compl_two = m5_compl_two + 1;
    check("M5.4 presented head is A, stable while ungranted",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(20)) &&
          (lq_complete.result == 32'h3333_0009));
    @(negedge clk); cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0; #1;
    check("M5.4 after the grant the second drains in order",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(21)) &&
          (lq_complete.result == 32'h4444_000A));
    @(negedge clk); cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0; #1;
    check("M5.4 buffer drained", dut.lq_compl_count_q === 2'd0);
    commit_one(rob_idx_t'(20), 1'b0);
    commit_one(rob_idx_t'(21), 1'b0);

    // ---- A launch is REFUSED while a completion waits ungranted.
    // The immediate-response environment exercises this admission guard.
    @(negedge clk);
    flush_all();
    dmem_ready = 1'b1; dmem_rvalid = 1'b1;   // identity shape: same-cycle resp
    rob_head_idx = rob_idx_t'(24);
    dmem_rdata = 32'h5555_000B;
    lq_alloc_one(rob_idx_t'(24), phys_reg_t'(49), MEM_W, m4_lq_ticket);
    lq_dep(m4_lq_ticket, 32'h0000_0228, 1'b0);
    @(negedge clk); #1;
    dmem_rvalid = 1'b0;
    check("M5.5 C completed same-cycle into the buffer, ungranted",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(24)));
    lq_alloc_one(rob_idx_t'(25), phys_reg_t'(50), MEM_W, m4_lq_ticket);
    lq_dep(m4_lq_ticket, 32'h0000_022C, 1'b0);
    repeat (2) begin
      @(negedge clk); #1;
      check("M5.5 launch refused while the completion waits",
            dmem_valid === 1'b0);
      m5_compl_block = m5_compl_block + 1;
    end
    @(negedge clk); cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0;
    dmem_rdata = 32'h6666_000C; dmem_rvalid = 1'b1;
    @(negedge clk); #1;
    dmem_rvalid = 1'b0;
    check("M5.5 presentation resumed after the grant and completed",
          lq_complete.valid && (lq_complete.rob_idx == rob_idx_t'(25)) &&
          (lq_complete.result == 32'h6666_000C));
    @(negedge clk); cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
    cdb_grant_lq = 1'b0;
    commit_one(rob_idx_t'(24), 1'b0);
    commit_one(rob_idx_t'(25), 1'b0);

    // drive every physical LQ slot through real launch, completion and
    // retirement, then wrap and reuse. Metadata is bit-identical across a
    // mark; missing clears/sets must affect the next request or completion.
    flush_all();
    dmem_ready = 1'b1; dmem_rvalid = 1'b1;
    for (f2_i = 0; f2_i < SS_LQ_DEPTH + 2; f2_i++) begin
      rob_head_idx = rob_idx_t'(26 + f2_i);
      dmem_rdata = 32'hF200_0000 + word_t'(f2_i);
      lq_alloc_one(rob_idx_t'(26 + f2_i), phys_reg_t'(40 + f2_i), MEM_W, lq_t);
      le = dut.lq_entry_q[lq_t];
      check("F2 allocation clears executed across ring wrap",
            !le.executed && lq_t == lq_idx_t'(f2_i));
      lq_dep(lq_t, 32'h0000_0300 + word_t'(4 * f2_i), 1'b0);
      f2_before = dut.lq_entry_q[lq_t];
      #1;
      check("F2 unexecuted owner launches", dut.lq_mem_req_fire);
      @(posedge clk); @(negedge clk); #1;
      f2_after = dut.lq_entry_q[lq_t];
      check("F2 actual launch sets executed", f2_after.executed);
      f2_after.executed = f2_before.executed;
      check("F2 launch leaves all metadata unchanged", f2_after === f2_before);
      check("F2 exactly-once completion after wrap",
            lq_complete.valid && lq_complete.rob_idx == rob_idx_t'(26 + f2_i) &&
            lq_complete.pdst == phys_reg_t'(40 + f2_i) &&
            lq_complete.result == (32'hF200_0000 + word_t'(f2_i)) &&
            !dut.lq_mem_req_fire);
      cdb_grant_lq = 1'b1; @(posedge clk); @(negedge clk);
      cdb_grant_lq = 1'b0;
      commit_one(rob_idx_t'(26 + f2_i), 1'b0);
      le = dut.lq_entry_q[lq_t];
      check("F2 committed pop clears the whole logical entry", le === '0);
    end

    for (int wrap_case = 0; wrap_case < 2; wrap_case++) begin
      for (int response_case = 0; response_case < 2; response_case++) begin
        surviving_held_read(rob_idx_t'(wrap_case ? 30 : 10),
                            response_case != 0, 1);
        surviving_held_read(rob_idx_t'(wrap_case ? 30 : 10),
                            response_case != 0, 3);
      end
    end
    check("all held-survivor timing and wrap cases entered", surviving_held_cases == 8);

    // ---entry proofs ------------------------------------------------
    check("M5 entered: two reads outstanding", m5_two_outstanding >= 1);
    check("M5 entered: same-edge pop+push", m5_same_edge >= 1);
    check("M5 entered: kill inside a two-outstanding window", m5_kill_two >= 1);
    check("M5 entered: two completions buffered", m5_compl_two >= 1);
    check("M5 entered: launch refused by completion occupancy",
          m5_compl_block >= 2);

    // ---entry proofs ------------------------------------------------
    check("M4 entered: stall-hold cycles", m4_hold_cycles >= 3);
    check("M4 entered: acceptance inside a recovery window",
          m4_accept_in_recovery >= 1);
    check("M4 entered: held-load launches", m4_held_launches >= 1);
    check("M4 entered: lock-hold cycles", m4_lock_cycles >= 2);
    check("M4 entered: store accepted during an outstanding response",
          m4_store_during_outstanding >= 1);
    check("M4 entered: held launch with a same-cycle response",
          m4_same_cycle_held >= 1);
    check("M4 entered: pop and held mark colliding on one slot",
          m4_pop_mark_collision >= 1);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_lsq] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_lsq] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
