`timescale 1ns/1ps

module tb_rv32i_ss_rob;
  import rv32i_ss_pkg::*;

  localparam time CLK_PERIOD = 10ns;

  logic      clk;
  logic      rst_n;

  logic      rob_alloc_valid;
  logic [1:0] tb_slot_valid;      // bundle shape (01 for single allocation)
  logic [1:0] tb_bundle_size;
  logic      rob_alloc_ready;
  word_t     [1:0] rob_alloc_pc;
  word_t     [1:0] rob_alloc_instr;
  logic      [1:0] rob_alloc_rd_we;
  arch_reg_t [1:0] rob_alloc_rd;
  phys_reg_t [1:0] rob_alloc_pdst;
  phys_reg_t [1:0] rob_alloc_stale_pdst;
  logic      rob_alloc_trap_valid;
  word_t     rob_alloc_trap_cause;
  word_t     rob_alloc_trap_tval;
  logic      rob_alloc_is_mret;
  rob_idx_t  [1:0] rob_alloc_idx;
  rob_seq_t  [1:0] rob_alloc_seq;

  assign tb_bundle_size = {1'b0, tb_slot_valid[0]} + {1'b0, tb_slot_valid[1]};

  logic     wb_valid;
  rob_idx_t wb_rob_idx;
  rob_seq_t wb_rob_seq;
  word_t    wb_result;
  logic     wb_csr_we;
  word_t    wb_csr_wdata;
  logic     wb_trap_valid;   // execute-detected trap rides the writeback
  word_t    wb_trap_cause;
  word_t    wb_trap_tval;

  logic [1:0]      commit_valid;
  logic [1:0]      commit_fire;
  logic            commit_enable;
  word_t [1:0]     commit_pc;
  word_t [1:0]     commit_instr;
  logic [1:0]      commit_rd_we;
  arch_reg_t [1:0] commit_rd;
  phys_reg_t [1:0] commit_pdst;
  phys_reg_t [1:0] commit_stale_pdst;
  word_t [1:0]     commit_result;
  rob_idx_t  rob_head_idx;
  rob_seq_t  rob_head_seq;
  logic      rob_head_valid;
  logic      rob_head_done;
  commit_order_t commit_order;
  logic [1:0]      commit_trap_valid;
  word_t [1:0]     commit_trap_cause;
  word_t [1:0]     commit_trap_tval;
  logic [1:0]      commit_is_mret;

  logic trap_flush;

  logic      branch_recover_req;
  rob_idx_t  recover_rob_idx;

  int errors = 0;
  int checks = 0;
  commit_order_t expected_commit_order;
  rob_seq_t expected_rob_alloc_seq;
  rob_seq_t last_rob_alloc_seq;
  rob_seq_t seq_by_idx [OOO_ROB_DEPTH];

  // This standalone TB produces ordered-prefix commit events: slot 1 may
  // fire only with slot 0. commit_enable controls hold/fire, while
  // dual_enable selects single or dual retirement at the ROB boundary.
  logic commit_dual_enable;
  // scratch for the dual-retire expectations
  rob_idx_t      exp_head;
  commit_order_t exp_order;
  logic [5:0]    exp_count;
  assign commit_fire[0] = commit_valid[0] && commit_enable;
  assign commit_fire[1] = commit_fire[0] && commit_valid[1] && commit_dual_enable;

  rv32i_ss_rob dut (
    .clk                 (clk),
    .rst_n               (rst_n),
    // the TB stands in for the core's single fire producer
    // (fire only when ready — the fire-honesty tripwire enforces it)
    .bundle_fire          (rob_alloc_valid & rob_alloc_ready),
    .rob_alloc_slot_valid (tb_slot_valid),
    .bundle_size          (tb_bundle_size),
    .rob_alloc_ready      (rob_alloc_ready),
    .rob_alloc_pc         (rob_alloc_pc),
    .rob_alloc_instr      (rob_alloc_instr),
    .rob_alloc_rd_we      (rob_alloc_rd_we),
    .rob_alloc_rd         (rob_alloc_rd),
      .rob_alloc_pdst       (rob_alloc_pdst),
      .rob_alloc_stale_pdst (rob_alloc_stale_pdst),
      .rob_alloc_trap_valid (rob_alloc_trap_valid),
      .rob_alloc_trap_cause (rob_alloc_trap_cause),
      .rob_alloc_trap_tval  (rob_alloc_trap_tval),
      .rob_alloc_is_mret    (rob_alloc_is_mret),
      .rob_alloc_is_csr     (1'b0),    // inert here; CSR-commit test drives these later
      .rob_alloc_csr_addr   ('0),
      .rob_alloc_idx    (rob_alloc_idx),
      .rob_alloc_seq    (rob_alloc_seq),
      .wb_valid            (wb_valid),
	      .wb_rob_idx          (wb_rob_idx),
	      .wb_rob_seq          (wb_rob_seq),
	      .wb_result           (wb_result),
	      .wb_csr_we           (wb_csr_we),
	      .wb_csr_wdata        (wb_csr_wdata),
      .wb_trap_valid       (wb_trap_valid),
      .wb_trap_cause       (wb_trap_cause),
      .wb_trap_tval        (wb_trap_tval),
    .commit_valid        (commit_valid),
    .commit_fire         (commit_fire),
    .commit_pc           (commit_pc),
    .commit_instr        (commit_instr),
    .commit_rd_we        (commit_rd_we),
    .commit_rd           (commit_rd),
    .commit_pdst         (commit_pdst),
    .commit_stale_pdst   (commit_stale_pdst),
    .commit_result       (commit_result),
	      .commit_is_csr        (),
	      .commit_csr_we        (),
	      .commit_csr_addr      (),
	      .commit_csr_wdata     (),
	      .trap_flush               (trap_flush),
      .branch_recover_req     (branch_recover_req),
      .recover_rob_idx (recover_rob_idx),
      .rob_head_idx        (rob_head_idx),
      .rob_head_seq        (rob_head_seq),
      .rob_head_valid      (rob_head_valid),
    .rob_head_done       (rob_head_done),
    .commit_order        (commit_order),
    .commit_trap_valid   (commit_trap_valid),
    .commit_trap_cause   (commit_trap_cause),
    .commit_trap_tval    (commit_trap_tval),
    .commit_is_mret      (commit_is_mret)
  );

  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;

  initial begin
    repeat (2000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_rob exceeded 2000 cycles");
  end

  function automatic rob_idx_t wrap_idx(input int value);
    wrap_idx = value % OOO_ROB_DEPTH;
  endfunction

  task automatic clear_inputs();
    rob_alloc_valid      = 1'b0;
    tb_slot_valid        = 2'b00;
    rob_alloc_pc         = '0;
    rob_alloc_instr      = '0;
    rob_alloc_rd_we      = 1'b0;
    rob_alloc_rd         = '0;
    rob_alloc_pdst       = '0;
    rob_alloc_stale_pdst = '0;
    rob_alloc_trap_valid = 1'b0;
    rob_alloc_trap_cause = '0;
    rob_alloc_trap_tval  = '0;
    rob_alloc_is_mret    = 1'b0;
      wb_valid            = 1'b0;
	      wb_rob_idx          = '0;
	      wb_rob_seq          = '0;
	      wb_result           = '0;
	      wb_csr_we           = 1'b0;
	      wb_csr_wdata        = '0;
    wb_trap_valid       = 1'b0;
    wb_trap_cause       = '0;
    wb_trap_tval        = '0;
    commit_enable       = 1'b0;
    commit_dual_enable  = 1'b0;
    trap_flush               = 1'b0;
    branch_recover_req     = 1'b0;
    recover_rob_idx = '0;
  endtask

  task automatic check_bit(input string name, input logic got, input logic exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got %0b expected %0b", name, got, exp);
      errors++;
    end
  endtask

  task automatic check_word(input string name, input word_t got, input word_t exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got %08h expected %08h", name, got, exp);
      errors++;
    end
  endtask

  task automatic check_arch(input string name, input arch_reg_t got, input arch_reg_t exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got x%0d expected x%0d", name, got, exp);
      errors++;
    end
  endtask

  task automatic check_phys(input string name, input phys_reg_t got, input phys_reg_t exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got p%0d expected p%0d", name, got, exp);
      errors++;
    end
  endtask

    task automatic check_idx(input string name, input rob_idx_t got, input rob_idx_t exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got ROB%0d expected ROB%0d", name, got, exp);
      errors++;
    end
    endtask

    task automatic check_seq(input string name, input rob_seq_t got, input rob_seq_t exp);
      checks++;
      if (got !== exp) begin
        $error("[%s] got seq %0d expected seq %0d", name, got, exp);
        errors++;
      end
    endtask

  task automatic check_order(input string name, input commit_order_t got, input commit_order_t exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got %0d expected %0d", name, got, exp);
      errors++;
    end
  endtask

    task automatic check_head(
      input string    name,
      input rob_idx_t exp_idx,
      input rob_seq_t exp_seq,
      input logic     exp_valid,
      input logic     exp_done
    );
    #1;
    check_idx({name, " rob_head_idx"}, rob_head_idx, exp_idx);
    if (exp_valid) begin
      check_seq({name, " rob_head_seq"}, rob_head_seq, exp_seq);
    end
    check_bit({name, " rob_head_valid"}, rob_head_valid, exp_valid);
    check_bit({name, " rob_head_done"}, rob_head_done, exp_done);
  endtask

  task automatic reset_dut();
      clear_inputs();
      expected_commit_order = '0;
      expected_rob_alloc_seq = '0;
      last_rob_alloc_seq     = '0;
      rst_n = 1'b0;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    @(negedge clk);
    #1;
    check_bit("reset rob_alloc_ready", rob_alloc_ready, 1'b1);
      check_bit("reset commit_valid[0]", commit_valid[0], 1'b0);
      check_bit("reset commit_valid[1]", commit_valid[1], 1'b0);
      check_idx("reset rob_alloc_idx", rob_alloc_idx, '0);
      check_seq("reset rob_alloc_seq", rob_alloc_seq, '0);
      check_head("reset", '0, '0, 1'b0, 1'b0);
      check_order("reset commit_order", commit_order, '0);
    endtask


  // ---- dual-allocation helpers ----
  task automatic do_rob_alloc2(
    input  string     name,
    input  word_t     pc0, input word_t pc1,
    input  phys_reg_t pd0, input phys_reg_t pd1,
    output rob_idx_t  i0,  output rob_idx_t i1,
    output rob_seq_t  s0,  output rob_seq_t s1);
    @(negedge clk);
    rob_alloc_valid = 1'b1;
    tb_slot_valid   = 2'b11;
    rob_alloc_pc    = {pc1, pc0};
    rob_alloc_instr = {32'h2222_2222, 32'h1111_1111};
    rob_alloc_rd_we = 2'b11;
    rob_alloc_rd    = {5'd7, 5'd6};
    rob_alloc_pdst  = {pd1, pd0};
    rob_alloc_stale_pdst = '0;
    #1;
    check_bit({name, " dual ready"}, rob_alloc_ready, 1'b1);
    i0 = rob_alloc_idx[0]; i1 = rob_alloc_idx[1];
    s0 = rob_alloc_seq[0]; s1 = rob_alloc_seq[1];
    seq_by_idx[i0] = s0; seq_by_idx[i1] = s1;
    @(posedge clk); @(negedge clk);
    expected_rob_alloc_seq = expected_rob_alloc_seq + 64'd2;
    rob_alloc_valid = 1'b0;
    tb_slot_valid   = 2'b00;
    rob_alloc_pc = '0; rob_alloc_instr = '0; rob_alloc_rd_we = '0;
    rob_alloc_rd = '0; rob_alloc_pdst = '0;
  endtask

  // one wb + commit beat for the current head
  task automatic wb_commit_head(input string name);
    do_wb_with_seq(name, rob_head_idx, seq_by_idx[rob_head_idx], 32'hc0de_0000);
    @(negedge clk); commit_enable = 1'b1;
    @(posedge clk); @(negedge clk); commit_enable = 1'b0;
  endtask

  task automatic do_rob_alloc(
    input  string     name,
    input  word_t     pc,
    input  word_t     instr,
    input  logic      rd_we,
    input  arch_reg_t rd,
    input  phys_reg_t pdst,
    input  phys_reg_t stale_pdst,
    input  rob_idx_t  expected_idx,
    output rob_idx_t  actual_idx
  );
    @(negedge clk);
    rob_alloc_valid      = 1'b1;
    tb_slot_valid        = 2'b01;
    rob_alloc_pc         = pc;
    rob_alloc_instr      = instr;
    rob_alloc_rd_we      = rd_we;
    rob_alloc_rd         = rd;
    rob_alloc_pdst       = pdst;
    rob_alloc_stale_pdst = stale_pdst;
    #1;
      check_bit({name, " rob_alloc_ready"}, rob_alloc_ready, 1'b1);
      actual_idx = rob_alloc_idx;
      check_idx({name, " rob_alloc_idx"}, actual_idx, expected_idx);
      check_seq({name, " rob_alloc_seq"}, rob_alloc_seq, expected_rob_alloc_seq);
      seq_by_idx[actual_idx] = rob_alloc_seq;
      last_rob_alloc_seq = rob_alloc_seq;
      @(posedge clk);
      expected_rob_alloc_seq++;
      @(negedge clk);
    rob_alloc_valid      = 1'b0;
    tb_slot_valid        = 2'b00;
    rob_alloc_pc         = '0;
    rob_alloc_instr      = '0;
    rob_alloc_rd_we      = 1'b0;
    rob_alloc_rd         = '0;
    rob_alloc_pdst       = '0;
    rob_alloc_stale_pdst = '0;
  endtask

    task automatic do_wb_with_seq(
      input string    name,
      input rob_idx_t idx,
      input rob_seq_t seq,
      input word_t    result
    );
      @(negedge clk);
      wb_valid   = 1'b1;
      wb_rob_idx = idx;
      wb_rob_seq = seq;
      wb_result  = result;
      @(posedge clk);
      @(negedge clk);
      wb_valid   = 1'b0;
      wb_rob_idx = '0;
      wb_rob_seq = '0;
      wb_result  = '0;
      #1;
      $display("[%s] writeback ROB%0d seq=%0d result=%08h", name, idx, seq, result);
    endtask

    task automatic do_wb(input string name, input rob_idx_t idx, input word_t result);
      do_wb_with_seq(name, idx, seq_by_idx[idx], result);
    endtask

  task automatic expect_commit(
    input string     name,
    input word_t     pc,
    input word_t     instr,
    input logic      rd_we,
    input arch_reg_t rd,
    input phys_reg_t pdst,
    input phys_reg_t stale_pdst,
    input word_t     result
  );
    #1;
    check_bit({name, " commit_valid"}, commit_valid[0], 1'b1);
    check_word({name, " pc"}, commit_pc[0], pc);
    check_word({name, " instr"}, commit_instr[0], instr);
    check_bit({name, " rd_we"}, commit_rd_we[0], rd_we);
    check_arch({name, " rd"}, commit_rd[0], rd);
    check_phys({name, " pdst"}, commit_pdst[0], pdst);
    check_phys({name, " stale_pdst"}, commit_stale_pdst[0], stale_pdst);
    check_word({name, " result"}, commit_result[0], result);
    check_bit({name, " trap_valid"}, commit_trap_valid[0], 1'b0);
    check_word({name, " trap_cause"}, commit_trap_cause[0], '0);
    check_word({name, " trap_tval"}, commit_trap_tval[0], '0);
    check_bit({name, " is_mret"}, commit_is_mret[0], 1'b0);
  endtask

  task automatic commit_one(
    input string     name,
    input word_t     pc,
    input word_t     instr,
    input logic      rd_we,
    input arch_reg_t rd,
    input phys_reg_t pdst,
    input phys_reg_t stale_pdst,
    input word_t     result
  );
    @(negedge clk);
    commit_enable = 1'b1;
    expect_commit(name, pc, instr, rd_we, rd, pdst, stale_pdst, result);
    check_order({name, " commit_order"}, commit_order, expected_commit_order);
    @(posedge clk);
    @(negedge clk);
    commit_enable = 1'b0;
    expected_commit_order++;
  endtask

  task automatic rob_alloc_and_commit(
    input  string     name,
    input  word_t     commit_pc_exp,
    input  word_t     commit_instr_exp,
    input  logic      commit_rd_we_exp,
    input  arch_reg_t commit_rd_exp,
    input  phys_reg_t commit_pdst_exp,
    input  phys_reg_t commit_stale_exp,
    input  word_t     commit_result_exp,
    input  word_t     rob_alloc_pc_new,
    input  word_t     rob_alloc_instr_new,
    input  arch_reg_t rob_alloc_rd_new,
    input  phys_reg_t rob_alloc_pdst_new,
    input  phys_reg_t rob_alloc_stale_new,
    input  rob_idx_t  expected_rob_alloc_idx,
    output rob_idx_t  actual_rob_alloc_idx
  );
    @(negedge clk);
    commit_enable       = 1'b1;
    rob_alloc_valid      = 1'b1;
    tb_slot_valid        = 2'b01;
    rob_alloc_pc         = rob_alloc_pc_new;
    rob_alloc_instr      = rob_alloc_instr_new;
    rob_alloc_rd_we      = 1'b1;
    rob_alloc_rd         = rob_alloc_rd_new;
    rob_alloc_pdst       = rob_alloc_pdst_new;
    rob_alloc_stale_pdst = rob_alloc_stale_new;
    #1;
    expect_commit(name, commit_pc_exp, commit_instr_exp, commit_rd_we_exp,
                  commit_rd_exp, commit_pdst_exp, commit_stale_exp,
                  commit_result_exp);
    check_order({name, " commit_order"}, commit_order, expected_commit_order);
      check_bit({name, " rob_alloc_ready"}, rob_alloc_ready, 1'b1);
      actual_rob_alloc_idx = rob_alloc_idx;
      check_idx({name, " rob_alloc_idx"}, actual_rob_alloc_idx, expected_rob_alloc_idx);
      check_seq({name, " rob_alloc_seq"}, rob_alloc_seq, expected_rob_alloc_seq);
      seq_by_idx[actual_rob_alloc_idx] = rob_alloc_seq;
      last_rob_alloc_seq = rob_alloc_seq;
      @(posedge clk);
      @(negedge clk);
      expected_commit_order++;
      expected_rob_alloc_seq++;
    commit_enable       = 1'b0;
    rob_alloc_valid      = 1'b0;
    tb_slot_valid        = 2'b00;
    rob_alloc_pc         = '0;
    rob_alloc_instr      = '0;
    rob_alloc_rd_we      = 1'b0;
    rob_alloc_rd         = '0;
    rob_alloc_pdst       = '0;
    rob_alloc_stale_pdst = '0;
  endtask

  task automatic pulse_flush();
    @(negedge clk);
    trap_flush = 1'b1;
    @(posedge clk);
    @(negedge clk);
    trap_flush = 1'b0;
    #1;
      check_bit("trap_flush rob_alloc_ready", rob_alloc_ready, 1'b1);
      check_bit("trap_flush commit_valid", commit_valid[0], 1'b0);
      check_idx("trap_flush rob_alloc_idx", rob_alloc_idx, '0);
      check_seq("trap_flush rob_alloc_seq holds", rob_alloc_seq, expected_rob_alloc_seq);
      check_head("trap_flush", '0, '0, 1'b0, 1'b0);
      check_order("trap_flush commit_order holds", commit_order, expected_commit_order);
    endtask

  // positively prove decode-detected trap carry + commit-boundary gating.
  // (Existing commits check trap_* stay 0; this allocates real trap entries and
  // checks they become visible at the head -- and ONLY once the head is done.)
  task automatic test_trap_carry();
    rob_idx_t t_idx;
    rob_seq_t t_seq;
    reset_dut();

    // ---- ecall-class entry: trap_valid, cause 11, tval 0x42, no rd ----
    @(negedge clk);
    rob_alloc_valid      = 1'b1;
    tb_slot_valid        = 2'b01;
    rob_alloc_pc         = 32'h0000_0100;
    rob_alloc_instr      = 32'h0000_0073;
    rob_alloc_rd_we      = 1'b0;
    rob_alloc_trap_valid = 1'b1;
    rob_alloc_trap_cause = 32'd11;
    rob_alloc_trap_tval  = 32'h0000_0042;
    #1;
    t_idx = rob_alloc_idx;
    t_seq = rob_alloc_seq;
    @(posedge clk);
    @(negedge clk);
    rob_alloc_valid      = 1'b0;
    tb_slot_valid        = 2'b00;
    rob_alloc_trap_valid = 1'b0;
    rob_alloc_trap_cause = '0;
    rob_alloc_trap_tval  = '0;
    #1;
    // head is valid but NOT done -> commit_valid=0 -> trap exposure gated low
    check_bit("trap carry: gated low before done", commit_trap_valid[0], 1'b0);
    do_wb_with_seq("trap carry", t_idx, t_seq, 32'd0);   // mark done
    #1;
    check_bit ("trap carry: commit_trap_valid", commit_trap_valid[0], 1'b1);
    check_word("trap carry: commit_trap_cause", commit_trap_cause[0], 32'd11);
    check_word("trap carry: commit_trap_tval",  commit_trap_tval[0],  32'h0000_0042);
    check_bit ("trap carry: commit_is_mret",    commit_is_mret[0],    1'b0);
    @(negedge clk); commit_enable = 1'b1; @(posedge clk); @(negedge clk); commit_enable = 1'b0;

    // ---- mret-class entry: is_mret only (trap_valid stays 0) ----
    @(negedge clk);
    rob_alloc_valid   = 1'b1;
    tb_slot_valid = 2'b01;
    rob_alloc_pc      = 32'h0000_0200;
    rob_alloc_instr   = 32'h3020_0073;
    rob_alloc_rd_we   = 1'b0;
    rob_alloc_is_mret = 1'b1;
    #1;
    t_idx = rob_alloc_idx;
    t_seq = rob_alloc_seq;
    @(posedge clk);
    @(negedge clk);
    rob_alloc_valid   = 1'b0;
    tb_slot_valid = 2'b00;
    rob_alloc_is_mret = 1'b0;
    do_wb_with_seq("mret carry", t_idx, t_seq, 32'd0);
    #1;
    check_bit("mret carry: commit_is_mret",     commit_is_mret[0],    1'b1);
    check_bit("mret carry: trap_valid stays 0", commit_trap_valid[0], 1'b0);
    @(negedge clk); commit_enable = 1'b1; @(posedge clk); @(negedge clk); commit_enable = 1'b0;
  endtask

  // Recover-count wraparound at a FULL ROB.
  // When head=0 and the recovering branch is the YOUNGEST live entry (idx 31,
  // age 31), a narrow tail-based count (recover_tail - head) would
  // wrap to 0 while 32 entries remain valid. The
  // kill loop is correct (nothing is younger than idx 31, so it clears nothing),
  // so the bug is purely the count: a false-empty ROB would let dispatch
  // overwrite live entries. The age+1 formula gives 32. Age 31 is the UNIQUE
  // trigger: for age<31, (age+1) mod 32 == age+1 already, so only the full-ROB
  // youngest-branch case exposes it.
  task automatic test_recover_count_wrap();
    rob_idx_t fill_idx_rcw;
    int nvalid;
    reset_dut();

    // Fill all 32 entries: head stays 0, tail wraps 0..31 -> 0, count -> 32.
    for (int i = 0; i < OOO_ROB_DEPTH; i++) begin
      do_rob_alloc("rcw fill", 32'h0000_4000 + (i * 4), 32'h0000_0013,
                  1'b1, arch_reg_t'((i % 31) + 1),
                  phys_reg_t'(6'd32 + (i % 32)),
                  phys_reg_t'((i % 31) + 1),
                  wrap_idx(i), fill_idx_rcw);
    end
    #1;
    tb_slot_valid = 2'b01;  #1;  // present a one-slot offer (need-aware ready)
    check_bit ("rcw: ROB full (alloc_ready==0)",      rob_alloc_ready, 1'b0);
    tb_slot_valid = 2'b00;
    check_word("rcw: count==32 before recover",       word_t'(dut.count_q), 32'd32);
    check_idx ("rcw: head==0",                        dut.head_q, wrap_idx(0));

    // Recover the YOUNGEST entry: branch at idx 31, age 31. Nothing is younger,
    // so all 32 must survive and the count must stay 32 (not false-empty).
    @(negedge clk);
    branch_recover_req     = 1'b1;
    recover_rob_idx = wrap_idx(31);
    @(posedge clk);
    @(negedge clk);
    branch_recover_req     = 1'b0;
    #1;
    check_word("rcw: count stays 32 after age-31 recover", word_t'(dut.count_q), 32'd32);
    check_idx ("rcw: tail wraps to head (still full)",     dut.tail_q, dut.head_q);
    tb_slot_valid = 2'b01;  #1;  // present a one-slot offer (need-aware ready)
    check_bit ("rcw: alloc_ready stays 0 (not false-empty)", rob_alloc_ready, 1'b0);
    tb_slot_valid = 2'b00;
    nvalid = 0;
    for (int i = 0; i < OOO_ROB_DEPTH; i++) if (dut.valid_q[i]) nvalid++;
    check_word("rcw: all 32 entries survive", word_t'(nvalid), 32'd32);
  endtask

  // Writeback coincident with recovery for a SURVIVING older entry.
  // Recovery must also apply that writeback. Making recovery and writeback
  // mutually exclusive would drop the fire-and-forget CDB beat, leaving the
  // survivor permanently not-done and preventing forward commit progress.
  task automatic test_wb_during_recover();
    rob_idx_t m_idx, b_idx;
    rob_seq_t m_seq;
    reset_dut();
    // M at idx0 (older -> survives); B (branch) at idx1 (younger -> recoverer).
    do_rob_alloc("wbr M", 32'h0000_5000, 32'h0000_0013, 1'b1, 5'd5, 6'd40, 6'd5,
                wrap_idx(0), m_idx);
    m_seq = last_rob_alloc_seq;
    do_rob_alloc("wbr B", 32'h0000_5004, 32'h0000_0063, 1'b0, '0, '0, '0,
                wrap_idx(1), b_idx);

    // Drive M's writeback in the SAME cycle as B's recovery.
    @(negedge clk);
    wb_valid   = 1'b1;
    wb_rob_idx = m_idx;
    wb_rob_seq = m_seq;
    wb_result  = 32'hA000_0001;
    branch_recover_req     = 1'b1;
    recover_rob_idx = b_idx;   // recover idx1; M (idx0) is older -> survives
    @(posedge clk);
    @(negedge clk);
    wb_valid           = 1'b0;
    branch_recover_req = 1'b0;
    #1;
    check_bit("wbr: M survives recovery (valid)", dut.valid_q[0], 1'b1);
    // The bug: this coincident writeback is dropped, so M never becomes done.
    check_bit("wbr: M's coincident writeback accepted (done)", dut.done_q[0], 1'b1);
  endtask

  // An execute-detected misalignment trap completing on the SAME cycle as a younger
  // branch's recovery must still deposit its trap fields — the recovery-branch
  // writeback site must mirror the normal site. If only the normal site writes
  // trap fields, this survivor completes done-but-trapless and COMMITS AS A
  // NORMAL OP: silent architectural corruption, worse than the stall.
  task automatic test_wb_trap_during_recover();
    rob_idx_t m_idx, b_idx;
    rob_seq_t m_seq;
    reset_dut();
    // M = misaligned store at idx0 (older -> survives); B = branch at idx1.
    do_rob_alloc("wtr M", 32'h0000_6000, 32'h0000_0023, 1'b0, '0, '0, '0,
                wrap_idx(0), m_idx);
    m_seq = last_rob_alloc_seq;
    do_rob_alloc("wtr B", 32'h0000_6004, 32'h0000_0063, 1'b0, '0, '0, '0,
                wrap_idx(1), b_idx);

    // M's trap-carrying writeback coincident with B's recovery.
    @(negedge clk);
    wb_valid      = 1'b1;
    wb_rob_idx    = m_idx;
    wb_rob_seq    = m_seq;
    wb_result     = 32'h0;
    wb_trap_valid = 1'b1;
    wb_trap_cause = 32'd6;          // store-address-misaligned
    wb_trap_tval  = 32'h0000_0203;  // the faulting address
    branch_recover_req     = 1'b1;
    recover_rob_idx = b_idx;
    @(posedge clk);
    @(negedge clk);
    wb_valid           = 1'b0;
    wb_trap_valid      = 1'b0;
    branch_recover_req = 1'b0;
    #1;
    check_bit ("wtr: M survives recovery (valid)", dut.valid_q[0], 1'b1);
    check_bit ("wtr: M done via coincident wb",    dut.done_q[0], 1'b1);
    // M is the head: its trap must be visible at the commit boundary.
    check_bit ("wtr: trap fields survived the recovery-site wb", commit_trap_valid[0], 1'b1);
    check_word("wtr: trap cause 6",  commit_trap_cause[0], 32'd6);
    check_word("wtr: trap tval = faulting addr", commit_trap_tval[0], 32'h0000_0203);
  endtask

  initial begin : test_sequence
    rob_idx_t idx_a;
    rob_idx_t idx_b;
    rob_idx_t idx_c;
      rob_idx_t idx_d;
      rob_idx_t idx_e;
      rob_idx_t idx_old;
      rob_idx_t idx_new;
      rob_idx_t fill_idx;
      rob_seq_t old_seq;

    $display("[tb_rv32i_ss_rob] starting");

    reset_dut();

    do_rob_alloc("A", 32'h0000_0100, 32'h0010_82b3, 1'b1, 5'd5, 6'd32, 6'd5,
                wrap_idx(0), idx_a);
      check_head("A dispatched at head", idx_a, seq_by_idx[idx_a], 1'b1, 1'b0);
    do_rob_alloc("B", 32'h0000_0104, 32'h0021_0333, 1'b1, 5'd6, 6'd33, 6'd6,
                wrap_idx(1), idx_b);
      check_head("B behind A", idx_a, seq_by_idx[idx_a], 1'b1, 1'b0);

    do_wb("B before A", idx_b, 32'hbbbb_0002);
    #1;
    check_bit("younger done cannot commit before head", commit_valid[0], 1'b0);
      check_head("B done but A head not done", idx_a, seq_by_idx[idx_a], 1'b1, 1'b0);

    do_wb("A", idx_a, 32'haaaa_0001);
      check_head("A done at head", idx_a, seq_by_idx[idx_a], 1'b1, 1'b1);

    @(negedge clk);
    commit_enable = 1'b0;
    expect_commit("A held by commit_enable=0", 32'h0000_0100, 32'h0010_82b3,
                  1'b1, 5'd5, 6'd32, 6'd5, 32'haaaa_0001);
    check_order("A held order first cycle", commit_order, expected_commit_order);
    @(posedge clk);
    @(negedge clk);
    expect_commit("A still held", 32'h0000_0100, 32'h0010_82b3,
                  1'b1, 5'd5, 6'd32, 6'd5, 32'haaaa_0001);
    check_order("A held order second cycle", commit_order, expected_commit_order);
    commit_enable = 1'b0;

    commit_one("A commit", 32'h0000_0100, 32'h0010_82b3,
               1'b1, 5'd5, 6'd32, 6'd5, 32'haaaa_0001);
      check_head("B becomes head after A", idx_b, seq_by_idx[idx_b], 1'b1, 1'b1);
    commit_one("B commit", 32'h0000_0104, 32'h0021_0333,
               1'b1, 5'd6, 6'd33, 6'd6, 32'hbbbb_0002);
    #1;
    check_bit("empty after A/B", commit_valid[0], 1'b0);
      check_head("empty after A/B head", wrap_idx(2), seq_by_idx[wrap_idx(2)], 1'b0, 1'b0);

    do_rob_alloc("C", 32'h0000_0200, 32'h0032_83b3, 1'b1, 5'd7, 6'd34, 6'd7,
                wrap_idx(2), idx_c);
    do_wb("C", idx_c, 32'hcccc_0003);
    rob_alloc_and_commit("C commit while D dispatches",
                        32'h0000_0200, 32'h0032_83b3, 1'b1, 5'd7, 6'd34, 6'd7,
                        32'hcccc_0003,
                        32'h0000_0204, 32'h0043_0433, 5'd8, 6'd35, 6'd8,
                        wrap_idx(3), idx_d);
    #1;
    check_bit("D not done after same-cycle dispatch", commit_valid[0], 1'b0);

    do_wb("D", idx_d, 32'hdddd_0004);
    commit_one("D commit", 32'h0000_0204, 32'h0043_0433,
               1'b1, 5'd8, 6'd35, 6'd8, 32'hdddd_0004);
    #1;
    check_bit("empty after D", commit_valid[0], 1'b0);

    for (int i = 0; i < OOO_ROB_DEPTH; i++) begin
      do_rob_alloc("fill", 32'h0000_1000 + (i * 4), 32'h0000_0013,
                  1'b1, arch_reg_t'((i % 31) + 1),
                  phys_reg_t'(6'd32 + (i % 32)),
                  phys_reg_t'((i % 31) + 1),
                  wrap_idx(4 + i), fill_idx);
    end
    #1;
    tb_slot_valid = 2'b01;  #1;  // present a one-slot offer (need-aware ready)
    check_bit("full rob_alloc_ready", rob_alloc_ready, 1'b0);
    tb_slot_valid = 2'b00;

      pulse_flush();

      do_rob_alloc("old flushed occupant", 32'h0000_3000, 32'h0052_82b3,
                  1'b1, 5'd9, 6'd36, 6'd9, wrap_idx(0), idx_old);
      old_seq = last_rob_alloc_seq;
      check_head("old occupant before trap_flush", idx_old, old_seq, 1'b1, 1'b0);
      pulse_flush();

      do_rob_alloc("new reused occupant", 32'h0000_3004, 32'h0063_0333,
                  1'b1, 5'd10, 6'd37, 6'd10, wrap_idx(0), idx_new);
      check_head("new reused occupant before stale wb", idx_new, seq_by_idx[idx_new],
                 1'b1, 1'b0);
      do_wb_with_seq("stale old writeback rejected", idx_new, old_seq, 32'hbad0_0001);
      #1;
      check_bit("stale writeback must not make reused entry commit", commit_valid[0], 1'b0);
      check_head("new reused occupant after stale wb", idx_new, seq_by_idx[idx_new],
                 1'b1, 1'b0);

      do_wb("new reused occupant", idx_new, 32'h3737_0001);
      commit_one("new reused occupant commit", 32'h0000_3004, 32'h0063_0333,
                 1'b1, 5'd10, 6'd37, 6'd10, 32'h3737_0001);

      do_rob_alloc("E no rd", 32'h0000_0300, 32'h0000_2023, 1'b0, '0, '0, '0,
                  wrap_idx(1), idx_e);
      check_head("E no rd at head", idx_e, seq_by_idx[idx_e], 1'b1, 1'b0);
    do_wb("E no rd", idx_e, 32'heeee_0005);
    commit_one("E no rd commit", 32'h0000_0300, 32'h0000_2023,
               1'b0, '0, '0, '0, 32'heeee_0005);

    test_trap_carry();

    test_recover_count_wrap();

    test_wb_during_recover();
    test_wb_trap_during_recover();


    // ================= dual allocation (locked-plan cases) =============
    begin
      rob_idx_t bi0, bi1, junk_idx;
      rob_seq_t bs0, bs1;
      int k;

      // ---- basic dual — distinct idx/seq, per-slot payload, count +2
      reset_dut();
      rst_n = 1'b0; repeat (2) @(posedge clk); @(negedge clk); rst_n = 1'b1;
      @(negedge clk);
      expected_rob_alloc_seq = '0;
      do_rob_alloc2("B2b.1", 32'h0000_a000, 32'h0000_a004, 6'd40, 6'd41,
                    bi0, bi1, bs0, bs1);
      check_idx ("B2b.1 dual-allocation idx distinct (+1)", bi1, rob_idx_t'(bi0 + 5'd1));
      check_seq ("B2b.1 dual-allocation seq distinct (+1)", bs1, bs0 + 64'd1);
      check_bit ("B2b.1 slot-0 entry valid", dut.valid_q[bi0], 1'b1);
      check_bit ("B2b.1 slot-1 entry valid", dut.valid_q[bi1], 1'b1);
      check_word("B2b.1 slot-0 pc", dut.pc_q[bi0], 32'h0000_a000);
      check_word("B2b.1 slot-1 pc", dut.pc_q[bi1], 32'h0000_a004);
      check_phys("B2b.1 slot-1 pdst", dut.pdst_q[bi1], 6'd41);
      check_word("B2b.1 commit slot-1 raw pc export", commit_pc[1], 32'h0000_a004);
      check_phys("B2b.1 commit slot-1 raw pdst export", commit_pdst[1], 6'd41);
      check_bit("B2b.1 commit slot-1 initially not done", commit_valid[1], 1'b0);
      check_bit ("B2b.1 slot-1 trap cleared (solo pin)", dut.trap_valid_q[bi1], 1'b0);
      check_bit ("B2b.1 slot-1 is_csr cleared (solo pin)", dut.is_csr_q[bi1], 1'b0);
      check_word("B2b.1 count +2", word_t'({26'b0, dut.count_q}), 32'd2);

      // Complete only head+1. Its exported fact becomes visible, but this
      // TB's dual_enable=0 role must keep the position-1 event dormant.
      do_wb_with_seq("B2b.1 slot-1 done", bi1, bs1, 32'hc0de_0001);
      check_bit("B2b.1 head remains not done", commit_valid[0], 1'b0);
      check_bit("B2b.1 commit slot-1 valid export", commit_valid[1], 1'b1);
      check_bit("B2b.1 commit slot-1 fire clamped", commit_fire[1], 1'b0);

      // ---- ORDERED-PREFIX DUAL RETIRE ----
      // head+1 is already done from above. Raising dual_enable with the head
      // still incomplete must NOT fire slot 1: the prefix rule is fire[1] ->
      // fire[0], not "any completed row may retire".
      commit_dual_enable = 1'b1;
      #1;
      check_bit("B2b.1d slot-1 cannot fire under an incomplete head",
                commit_fire[1], 1'b0);

      // Complete the head. Both rows are now done, so one edge must retire
      // BOTH: head advances by two, order advances by two, count drops by
      // two, and both rows clear. also reaches this state through the
      // core; here it is isolated at the ROB seam.
      do_wb_with_seq("B2b.1d head done", bi0, bs0, 32'hc0de_0000);
      #1;
      check_bit("B2b.1d both slots valid", commit_valid[0] & commit_valid[1], 1'b1);
      exp_head  = rob_idx_t'(dut.head_q + 5'd2);
      exp_order = dut.commit_order_q + 64'd2;
      exp_count = dut.count_q - 3'd2;
      @(negedge clk); commit_enable = 1'b1;
      #1;
      check_bit("B2b.1d ordered prefix fires slot 0", commit_fire[0], 1'b1);
      check_bit("B2b.1d ordered prefix fires slot 1", commit_fire[1], 1'b1);
      @(posedge clk); @(negedge clk);
      commit_enable = 1'b0; commit_dual_enable = 1'b0;

      check_idx ("B2b.1d head advanced by TWO", dut.head_q, exp_head);
      check_seq ("B2b.1d commit_order advanced by TWO",
                 dut.commit_order_q, exp_order);
      check_word("B2b.1d count dropped by TWO",
                 word_t'({26'b0, dut.count_q}), word_t'({26'b0, exp_count}));
      check_bit ("B2b.1d slot-0 row retired", dut.valid_q[bi0], 1'b0);
      check_bit ("B2b.1d slot-1 row retired", dut.valid_q[bi1], 1'b0);
      check_bit ("B2b.1d slot-0 done cleared", dut.done_q[bi0], 1'b0);
      check_bit ("B2b.1d slot-1 done cleared", dut.done_q[bi1], 1'b0);

      // ---- tail wrap (mandatory case 2): slot0=DEPTH-1, slot1=0 ----
      for (k = 2; k < 31; k++) begin
        do_rob_alloc($sformatf("B2b.2 fill%0d", k), word_t'(32'h0000_b000 + k*4),
                     32'h0000_0013, 1'b0, '0, '0, '0, wrap_idx(k), junk_idx);
      end
      // count=31, tail=31; free room for the dual: commit two heads
      wb_commit_head("B2b.2 c0");
      wb_commit_head("B2b.2 c1");
      do_rob_alloc2("B2b.2", 32'h0000_c000, 32'h0000_c004, 6'd42, 6'd43,
                    bi0, bi1, bs0, bs1);
      check_idx ("B2b.2 slot 0 gets last index", bi0, rob_idx_t'(31));
      check_idx ("B2b.2 slot 1 wraps to zero",  bi1, rob_idx_t'(0));
      check_bit ("B2b.2 wrapped entries valid",
                 dut.valid_q[31] & dut.valid_q[0], 1'b1);
      check_idx ("B2b.2 tail advanced past wrap", dut.tail_q, rob_idx_t'(1));
      check_seq ("B2b.2 seq distinct across wrap", bs1, bs0 + 64'd1);

      // ---- dual alloc + same-cycle commit (mandatory case 4) ----
      wb_commit_head("B2b.3 room0");
      wb_commit_head("B2b.3 room1");
      wb_commit_head("B2b.3 room2");
      // head is now valid+not-done; make it committable, then coincide
      do_wb_with_seq("B2b.3 head-done", rob_head_idx, seq_by_idx[rob_head_idx],
                     32'hc0de_1111);
      @(negedge clk);
      rob_alloc_valid = 1'b1;
      tb_slot_valid   = 2'b11;
      rob_alloc_pc    = {32'h0000_d004, 32'h0000_d000};
      rob_alloc_rd_we = 2'b00;
      commit_enable   = 1'b1;
      #1;
      check_bit("B2b.3 dual ready with room", rob_alloc_ready, 1'b1);
      k = int'(dut.count_q);
      @(posedge clk); @(negedge clk);
      rob_alloc_valid = 1'b0; tb_slot_valid = 2'b00; commit_enable = 1'b0;
      expected_rob_alloc_seq = expected_rob_alloc_seq + 64'd2;  // manual dual
      check_word("B2b.3 count = old + 2 - 1 (coincident commit)",
                 word_t'({26'b0, dut.count_q}), word_t'(k + 1));

      // ---- whole-bundle ready honesty at free==1 ----
      while (int'(dut.count_q) < 31) begin
        do_rob_alloc("B2b.4 fill", 32'h0000_e000, 32'h0000_0013,
                     1'b0, '0, '0, '0, dut.tail_q, junk_idx);
      end
      @(negedge clk);
      tb_slot_valid = 2'b11; #1;   // probe only: no valid, no fire
      check_bit("B2b.4 free==1: dual offer NOT ready", rob_alloc_ready, 1'b0);
      tb_slot_valid = 2'b01; #1;
      check_bit("B2b.4 free==1: single offer ready", rob_alloc_ready, 1'b1);
      tb_slot_valid = 2'b00;

      // ---- Dual retirement ACROSS the index wrap ----
      // proved wrap ALLOCATION of {31,0}; this retires a wrapped pair in
      // ONE edge: head 31 -> 1, order +2, both rows cleared. Directed, so the
      // wrap-retire composition is provably entered rather than implied by
      // typed ring arithmetic and long-program crossings.
      //
      // Wipe to a known-empty ROB via the trap arm (head/tail/count -> 0;
      // next_seq_q deliberately keeps counting), then walk the head to 31.
      @(negedge clk); trap_flush = 1'b1;
      @(posedge clk); @(negedge clk); trap_flush = 1'b0;
      expected_rob_alloc_seq = dut.next_seq_q;
      #1;
      check_word("B2b.5 flush emptied", word_t'({26'b0, dut.count_q}), 32'd0);
      while (dut.head_q != rob_idx_t'(31)) begin
        do_rob_alloc("B2b.5 spin", 32'h0000_d000, 32'h0000_0013,
                     1'b0, '0, '0, '0, dut.tail_q, junk_idx);
        wb_commit_head("B2b.5 spin-c");
      end
      do_rob_alloc2("B2b.5", 32'h0000_f000, 32'h0000_f004, 6'd44, 6'd45,
                    bi0, bi1, bs0, bs1);
      check_idx ("B2b.5 slot 0 at last index", bi0, rob_idx_t'(31));
      check_idx ("B2b.5 slot 1 wrapped to zero", bi1, rob_idx_t'(0));
      do_wb_with_seq("B2b.5 head done", bi0, bs0, 32'hc0de_0031);
      do_wb_with_seq("B2b.5 wrap done", bi1, bs1, 32'hc0de_0100);
      commit_dual_enable = 1'b1;
      #1;
      check_bit("B2b.5 both slots valid at the wrap seam",
                commit_valid[0] & commit_valid[1], 1'b1);
      exp_order = dut.commit_order_q + 64'd2;
      @(negedge clk); commit_enable = 1'b1;
      #1;
      check_bit("B2b.5 wrap dual fires slot 0 (head=31)", commit_fire[0], 1'b1);
      check_bit("B2b.5 wrap dual fires slot 1 (head+1=0)", commit_fire[1], 1'b1);
      @(posedge clk); @(negedge clk);
      commit_enable = 1'b0; commit_dual_enable = 1'b0;
      check_idx ("B2b.5 head wrapped 31 -> 1", dut.head_q, rob_idx_t'(1));
      check_seq ("B2b.5 commit_order advanced by TWO across the wrap",
                 dut.commit_order_q, exp_order);
      check_word("B2b.5 count drained to zero",
                 word_t'({26'b0, dut.count_q}), 32'd0);
      check_bit ("B2b.5 row 31 retired", dut.valid_q[31], 1'b0);
      check_bit ("B2b.5 row 0 retired",  dut.valid_q[0], 1'b0);
    end

    if (errors == 0) begin
      $display("[tb_rv32i_ss_rob] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_rob] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
