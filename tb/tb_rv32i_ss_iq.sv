`timescale 1ns/1ps

// =============================================================================
// tb_rv32i_ss_iq.sv  --  unit TB / executable spec for the dual-grant IQ.
// =============================================================================
// Coverage:
//   1. reset             -- empty: no issue, alloc_ready high
//   2. single ready uop  -- issues, carries the right payload
//   3. wakeup            -- a not-ready source blocks; ready_vec set -> issues
//   4. age ordering      -- two ready uops issue together in strict age order
//   5. oldest-READY      -- younger-ready issues before older-not-ready, then
//                           the older wins once it wakes (the core of OoO select)
//   6. FU binding        -- an unavailable singleton FU does not block another
//   7. age WRAP          -- ring-distance, not numeric compare, picks the oldest
//   8. iq_full           -- 16 entries fill, alloc_ready drops, issue frees one
//   9. trap_flush        -- clears every entry
//  10. dual allocation   -- distinct holes and need-aware atomic acceptance
// =============================================================================

module tb_rv32i_ss_iq;
  import rv32i_ss_pkg::*;

  localparam time CLK_PERIOD = 10ns;

  logic clk;
  logic rst_n;
  logic trap_flush;

  logic      alloc_valid;
  iq_entry_t [1:0] iq_alloc_entry;
  logic      alloc_ready;
  logic [1:0] tb_shape;                 // 01 for a single allocation; 11 for dual allocation
  logic [1:0] tb_size;
  assign tb_size = {1'b0, tb_shape[0]} + {1'b0, tb_shape[1]};

  logic [OOO_PHYS_REGS-1:0] ready_vec;
  rob_idx_t                 rob_head_idx;

  logic        [1:0] issue_valid;
  iq_entry_t   [1:0] issue_entry;
  issue_unit_e [1:0] issue_unit;
  logic              issue_accept;
  logic              alu0_fu_ready;
  logic              alu1_fu_ready;
  logic              muldiv_fu_ready;
  logic              lsu_fu_ready;

  int errors = 0;
  int checks = 0;

  rv32i_ss_iq dut (
    .clk          (clk),
    .rst_n        (rst_n),
    .trap_flush        (trap_flush),
    .bundle_fire  (alloc_valid & alloc_ready),
    .bundle_size  (tb_size),
    .iq_alloc_slot_valid (tb_shape),
    .iq_alloc_entry (iq_alloc_entry),
    .iq_alloc_ready (alloc_ready),
    .ready_vec    (ready_vec),
    .rob_head_idx (rob_head_idx),
    .issue_valid  (issue_valid),
    .issue_entry  (issue_entry),
    .issue_unit   (issue_unit),
    .issue_accept (issue_accept),
    .alu0_fu_ready   (alu0_fu_ready),
    .alu1_fu_ready   (alu1_fu_ready),
    .muldiv_fu_ready (muldiv_fu_ready),
    .lsu_fu_ready    (lsu_fu_ready)
  );

  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;

  initial begin
    repeat (3000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_iq exceeded 3000 cycles");
  end

  // ----- check helpers -----
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
      $error("[%s] got=%0d (%08h) exp=%0d (%08h)", name, got, got, exp, exp);
      errors++;
    end
  endtask

  // ----- ready_vec helpers (TB models the PRF's centralized ready table) -----
  task automatic set_ready(input int p); ready_vec[p] = 1'b1; endtask
  task automatic clr_ready(input int p); ready_vec[p] = 1'b0; endtask

  task automatic ready_reset();
    // Mirror the PRF reset: arch physregs p0..p31 ready, free pool p32..p63 not.
    int b;
    for (b = 0; b < OOO_PHYS_REGS; b++) ready_vec[b] = (b < OOO_ARCH_REGS);
  endtask

  // ----- build a minimal iq_entry_t (only the fields the scheduler reads) -----
  function automatic iq_entry_t mk(input int ridx, input int p1, input int p2, input int pd);
    iq_entry_t e;
    e          = '0;
    e.rob_idx  = rob_idx_t'(ridx);
    e.prs1     = phys_reg_t'(p1);
    e.prs2     = phys_reg_t'(p2);
    e.pdst     = phys_reg_t'(pd);
    e.rd_wen   = 1'b1;
    e.op_class = OOO_OP_ALU;
    e.src1_sel = OOO_SRC_REG;
    e.src2_sel = OOO_SRC_REG;
    mk = e;
  endfunction

  // ----- stimulus tasks -----
  task automatic clear_inputs();
    trap_flush       = 1'b0;
    alloc_valid = 1'b0;
    tb_shape    = 2'b01;
    iq_alloc_entry = '0;
    issue_accept = 1'b0;
    alu0_fu_ready = 1'b1;
    alu1_fu_ready = 1'b1;
    muldiv_fu_ready = 1'b1;
    lsu_fu_ready = 1'b1;
  endtask

  task automatic reset_dut();
    clear_inputs();
    ready_reset();
    rob_head_idx = '0;
    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    @(negedge clk);
    #1;
  endtask

  // Allocate one uop into a free IQ entry (assumes alloc_ready).
  task automatic do_alloc(input iq_entry_t e);
    @(negedge clk);
    alloc_valid = 1'b1;
    iq_alloc_entry = e;
    @(posedge clk);
    @(negedge clk);
    alloc_valid = 1'b0;
    iq_alloc_entry = '0;
    #1;
  endtask

  // expect a uop is selected now, with the given rob_idx, then drain it
  task automatic issue_expect(input string name, input int exp_ridx);
    #1;
    check_bit ({name, " issue_valid[0]"},  issue_valid[0], 1'b1);
    check_word({name, " issued rob_idx"}, word_t'(issue_entry[0].rob_idx), word_t'(exp_ridx));
    @(negedge clk);
    issue_accept = 1'b1;
    @(posedge clk);     // selected entry frees here
    @(negedge clk);
    issue_accept = 1'b0;
    #1;
  endtask

  // Two ready same-class entries leave together in strict position age order.
  task automatic issue_expect_pair(input string name,
                                   input int exp0, input int exp1);
    #1;
    check_bit ({name, " issue_valid==11"}, &issue_valid, 1'b1);
    check_word({name, " position-0 rob_idx"},
               word_t'(issue_entry[0].rob_idx), word_t'(exp0));
    check_word({name, " position-1 rob_idx"},
               word_t'(issue_entry[1].rob_idx), word_t'(exp1));
    @(negedge clk);
    issue_accept = 1'b1;
    @(posedge clk);     // BOTH selected entries free here
    @(negedge clk);
    issue_accept = 1'b0;
    #1;
  endtask

  iq_entry_t e;
  int n;

  initial begin
    $display("[tb_rv32i_ss_iq] starting");

    // ---- 1. reset: empty ----
    reset_dut();
    check_bit("reset: no issue",     issue_valid[0], 1'b0);
    check_bit("reset: alloc_ready",  alloc_ready, 1'b1);

    // ---- 2. single fully-ready uop issues with the right payload ----
    do_alloc(mk(.ridx(0), .p1(1), .p2(2), .pd(32)));
    #1;
    check_bit ("single: issue_valid[0]",      issue_valid[0], 1'b1);
    check_word("single: rob_idx",          word_t'(issue_entry[0].rob_idx), 32'd0);
    check_word("single: pdst",             word_t'(issue_entry[0].pdst),    32'd32);
    issue_expect("single drain", 0);
    check_bit ("single: empty after issue", issue_valid[0], 1'b0);

    // ---- 3. wakeup: not-ready source blocks, then ready_vec set -> issues ----
    do_alloc(mk(.ridx(1), .p1(40), .p2(3), .pd(33)));  // p40 not ready
    #1;
    check_bit("wakeup: blocked while p40 busy", issue_valid[0], 1'b0);
    set_ready(40);
    #1;
    check_bit ("wakeup: issues once p40 ready", issue_valid[0], 1'b1);
    check_word("wakeup: rob_idx",               word_t'(issue_entry[0].rob_idx), 32'd1);
    issue_expect("wakeup drain", 1);
    clr_ready(40);

    // ---- 3b. non-register operands must NOT be awaited (src_sel gating) ----
    // The frontend hands the IQ a STALE prs for immediate/pc/zero operands
    // (decode extracts rs1/rs2 unconditionally; rename maps them anyway). An entry
    // must key readiness off src*_sel, not blindly off ready_vec[prs]. p50/p51/
    // p52 are in the free pool -> not ready by default; a naive readiness test would
    // falsely wait on them.
    // addi-style: src2_sel = IMM, prs2 = stale p50 -> operand 2 is the immediate.
    e = mk(.ridx(4), .p1(1), .p2(50), .pd(40));
    e.src2_sel = OOO_SRC_IMM;
    do_alloc(e);
    #1;
    check_bit ("imm-src: issues despite stale prs2", issue_valid[0], 1'b1);
    check_word("imm-src: rob_idx", word_t'(issue_entry[0].rob_idx), 32'd4);
    issue_expect("imm-src drain", 4);

    // lui-style: src1_sel = ZERO, src2_sel = IMM, both prs stale.
    e = mk(.ridx(6), .p1(51), .p2(52), .pd(41));
    e.src1_sel = OOO_SRC_ZERO;
    e.src2_sel = OOO_SRC_IMM;
    do_alloc(e);
    #1;
    check_bit ("non-reg srcs: issues despite stale prs1/prs2", issue_valid[0], 1'b1);
    check_word("non-reg srcs: rob_idx", word_t'(issue_entry[0].rob_idx), 32'd6);
    issue_expect("non-reg drain", 6);

    // Store address uses rs1+imm. Store data comes from prs2, but the SQ now
    // captures it independently: a ready base must let AGEN issue even while
    // the store-data preg is busy.
    e = mk(.ridx(7), .p1(1), .p2(53), .pd(0));
    e.src2_sel = OOO_SRC_IMM;
    e.rd_wen   = 1'b0;
    e.is_store = 1'b1;
    do_alloc(e);
    #1;
    check_bit ("store-data: late prs2 does not block AGEN", issue_valid[0], 1'b1);
    check_word("store-data: rob_idx", word_t'(issue_entry[0].rob_idx), 32'd7);
    issue_expect("store-data-independent AGEN drain", 7);

    // split transaction: a load may issue to the AGU before becoming the
    // ROB head. Its later dmem launch is guarded independently by the LQ.
    rob_head_idx = '0;
    e = mk(.ridx(7), .p1(1), .p2(50), .pd(44));
    e.fu_class = OOO_FU_LSU;
    e.src2_sel = OOO_SRC_IMM;
    e.is_load  = 1'b1;
    do_alloc(e);
    #1;
    check_bit("load-AGU: non-head ready load may issue", issue_valid[0], 1'b1);
    check_word("load-AGU: selected non-head rob_idx",
               word_t'(issue_entry[0].rob_idx), 32'd7);
    issue_expect("load-AGU: drain non-head load", 7);

    // ---- 4. age ordering: smaller ROB ring-distance issues first ----
    rob_head_idx = '0;
    do_alloc(mk(.ridx(5), .p1(1), .p2(2), .pd(34)));   // age 5
    do_alloc(mk(.ridx(2), .p1(1), .p2(2), .pd(35)));   // age 2 (older)
    issue_expect_pair("age: dual grant in age order", 2, 5);
    #1;
    check_bit("age: empty after both", issue_valid[0], 1'b0);

    // ---- 5. oldest-READY: younger-ready beats older-not-ready, then older wins ----
    rob_head_idx = '0;
    do_alloc(mk(.ridx(3), .p1(50), .p2(2), .pd(36)));  // OLDER (age3), p50 not ready
    do_alloc(mk(.ridx(8), .p1(1),  .p2(2), .pd(37)));  // YOUNGER (age8), ready
    #1;
    check_bit ("oldest-ready: younger issues while older blocked", issue_valid[0], 1'b1);
    check_word("oldest-ready: younger rob_idx", word_t'(issue_entry[0].rob_idx), 32'd8);
    set_ready(50);                                     // older wakes
    #1;
    check_word("oldest-ready: older now wins", word_t'(issue_entry[0].rob_idx), 32'd3);
    issue_expect_pair("oldest-ready: dual drain {3,8}", 3, 8);
    clr_ready(50);

    // ---- 5b. FU availability: skip older ready uop if its FU is busy ----
    rob_head_idx = '0;
    e = mk(.ridx(3), .p1(1), .p2(2), .pd(42));
    e.fu_class = OOO_FU_MULDIV;
    do_alloc(e);                                            // OLDER, but MULDIV unavailable
    e = mk(.ridx(8), .p1(1), .p2(2), .pd(43));
    e.fu_class = OOO_FU_ALU;
    do_alloc(e);                                            // YOUNGER ALU, available
    muldiv_fu_ready = 1'b0;
    #1;
    check_bit ("fu-ready: younger ALU issues while older MULDIV blocked",
               issue_valid[0], 1'b1);
    check_word("fu-ready: younger ALU rob_idx", word_t'(issue_entry[0].rob_idx), 32'd8);
    issue_expect("fu-ready: drain younger ALU", 8);
    #1;
    check_bit("fu-ready: older MULDIV remains blocked", issue_valid[0], 1'b0);
    muldiv_fu_ready = 1'b1;
    issue_expect("fu-ready: older MULDIV issues once FU ready", 3);

    // ---5c. solo guard: a solo op is ALU0-only ----
    // Directed leg for the grant-0 solo guard (the solo_blocks_alu1 argument
    // of iq_bind_unit): with ALU0 busy and ALU1 free, an operand-READY solo
    // is UNBINDABLE -- it must not issue on any unit and must not block a
    // younger bindable ALU candidate from the age competition. Provably
    // entered: the ready-solo + alu0-busy + alu1-free state holds across
    // posedges with the DUT pin net armed, and the checks below prove the
    // grant went AROUND the older solo. Dropping the guard (mutation-RED
    // ) binds the solo to ALU1: these checks fail and the DUT's own
    // "solo op bound to ALU1" pin fatals at the next posedge.
    rob_head_idx = '0;
    e = mk(.ridx(2), .p1(1), .p2(2), .pd(44));
    e.op_class = OOO_OP_JUMP;                          // issue-solo class
    do_alloc(e);                                       // OLDER solo (age 2)
    do_alloc(mk(.ridx(9), .p1(1), .p2(2), .pd(45)));   // YOUNGER plain ALU
    alu0_fu_ready = 1'b0;                              // ALU0 busy, ALU1 free
    #1;
    check_bit ("solo-guard: a grant still lands", issue_valid[0], 1'b1);
    check_word("solo-guard: younger ALU wins grant 0, not the older solo",
               word_t'(issue_entry[0].rob_idx), 32'd9);
    check_word("solo-guard: younger bound to ALU1",
               word_t'(issue_unit[0]), word_t'(ISSUE_UNIT_ALU1));
    check_bit ("solo-guard: no second grant (no ALU capacity left)",
               issue_valid[1], 1'b0);
    issue_expect("solo-guard: drain younger via ALU1", 9);
    #1;
    check_bit ("solo-guard: solo waits while ALU0 busy (never takes ALU1)",
               issue_valid[0], 1'b0);
    alu0_fu_ready = 1'b1;
    #1;
    check_bit ("solo-guard: solo issues once ALU0 frees", issue_valid[0], 1'b1);
    check_word("solo-guard: solo rob_idx", word_t'(issue_entry[0].rob_idx), 32'd2);
    check_word("solo-guard: solo bound to ALU0",
               word_t'(issue_unit[0]), word_t'(ISSUE_UNIT_ALU0));
    issue_expect("solo-guard: drain solo on ALU0", 2);

    // ---- 6. age WRAP: ring-distance, not numeric compare ----
    rob_head_idx = rob_idx_t'(30);
    do_alloc(mk(.ridx(0),  .p1(1), .p2(2), .pd(38)));  // age (0-30)&31  = 2
    do_alloc(mk(.ridx(31), .p1(1), .p2(2), .pd(39)));  // age (31-30)&31 = 1 (older)
    #1;
    check_bit ("wrap: someone ready", issue_valid[0], 1'b1);
    check_word("wrap: idx31 is oldest (age1), not numeric-min idx0",
               word_t'(issue_entry[0].rob_idx), 32'd31);
    issue_expect_pair("wrap: dual drain ring order {31,0}", 31, 0);
    rob_head_idx = '0;

    // ---- 7. iq_full: fill all OOO_IQ_DEPTH entries, alloc_ready drops ----
    issue_accept = 1'b0;
    for (n = 0; n < OOO_IQ_DEPTH; n++) begin
      do_alloc(mk(.ridx(n), .p1(1), .p2(2), .pd(32 + n)));
    end
    #1;
    check_bit("full: alloc_ready low when full", alloc_ready, 1'b0);
    // an issue frees exactly one entry -> alloc_ready high next cycle
    @(negedge clk);
    issue_accept = 1'b1;
    @(posedge clk);
    @(negedge clk);
    issue_accept = 1'b0;
    #1;
    check_bit("full: alloc_ready high after one issue", alloc_ready, 1'b1);

    // ---- 8. trap_flush: clears every entry ----
    @(negedge clk);
    trap_flush = 1'b1;
    @(posedge clk);
    @(negedge clk);
    trap_flush = 1'b0;
    #1;
    check_bit("trap_flush: no issue after trap_flush",   issue_valid[0], 1'b0);
    check_bit("trap_flush: alloc_ready after trap_flush", alloc_ready, 1'b1);


    // ============ dual enqueue — distinct holes + need-aware ready ====
    begin
      iq_entry_t e0, e1;
      int n_before, hole0, hole1;
      // drain-free baseline count
      n_before = 0;
      for (int i = 0; i < OOO_IQ_DEPTH; i++) if (!dut.valid_q[i]) n_before++;
      if (n_before < 2) $fatal(1, "B2c needs >=2 free IQ entries at entry");
      e0 = '0; e0.pc = 32'hb2c0_0000; e0.rob_idx = 5'd10; e0.rob_seq = 64'd100;
      e1 = '0; e1.pc = 32'hb2c0_0004; e1.rob_idx = 5'd11; e1.rob_seq = 64'd101;
      @(negedge clk);
      tb_shape = 2'b11;
      iq_alloc_entry = {e1, e0};
      alloc_valid = 1'b1;
      #1;
      checks++;
      if (alloc_ready !== 1'b1) begin errors++; $error("[B2c dual ready] low"); end
      hole0 = int'(dut.free_idx[0]); hole1 = int'(dut.free_idx[1]);
      checks++;
      if (hole0 == hole1) begin errors++; $error("[B2c holes DISTINCT] both=%0d", hole0); end
      @(posedge clk); @(negedge clk);
      alloc_valid = 1'b0; tb_shape = 2'b01; iq_alloc_entry = '0;
      checks++;
      if (!(dut.valid_q[hole0] && dut.valid_q[hole1])) begin
        errors++; $error("[B2c both holes filled]"); end
      begin
        iq_entry_t chk0, chk1;   // whole-struct copies (Icarus lesson)
        chk0 = dut.entry_q[hole0]; chk1 = dut.entry_q[hole1];
        checks++;
        if (!(chk0.pc == 32'hb2c0_0000 && chk1.pc == 32'hb2c0_0004)) begin
          errors++; $error("[B2c per-slot payload routing]"); end
      end
      // need-aware refusal: fill until ONE hole remains, offer a dual
      begin
        int free_n;
        iq_entry_t ef;
        forever begin
          free_n = 0;
          for (int i = 0; i < OOO_IQ_DEPTH; i++) if (!dut.valid_q[i]) free_n++;
          if (free_n <= 1) break;
          ef = '0; ef.pc = 32'hb2cf_0000; ef.rob_idx = 5'd20; ef.rob_seq = 64'd200;
          @(negedge clk); iq_alloc_entry = {ef, ef}; tb_shape = 2'b01; alloc_valid = 1'b1;
          @(posedge clk); @(negedge clk); alloc_valid = 1'b0;
        end
        @(negedge clk); tb_shape = 2'b11; #1;
        checks++;
        if (alloc_ready !== 1'b0) begin errors++; $error("[B2c one hole: dual REFUSED] ready=1"); end
        tb_shape = 2'b01; #1;
        checks++;
        if (alloc_ready !== 1'b1) begin errors++; $error("[B2c one hole: single accepted] ready=0"); end
        tb_shape = 2'b01;
      end
    end

    if (errors == 0) begin
      $display("[tb_rv32i_ss_iq] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_iq] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
