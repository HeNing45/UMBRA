// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// =============================================================================
// tb_rv32i_ss_core_recover.sv -- directed mispredict-recovery proof.
// =============================================================================
// Exercises younger-uop recovery: holds the branch not-ready while younger uops
// dispatch past it, then releases the branch so it issues TAKEN. With registered
// recovery, the branch decides in
// cycle N and the recovery fans out in N+1 (recover_q), so branch_recover_req is
// itself the registered pulse -- the TB keys on it, then samples one posedge
// later when the recovery state has applied.
//
// Scenario A proves the full recovery datapath:
//   * rename spec-map restored to the checkpoint
//   * free-list reclaims the wrong-path pdsts
//   * ROB tail rolled back, younger entries invalid, next_seq_q NOT rolled back
//   * IQ younger entries killed
//   * a younger muldiv is killed by ROB ring age
// Scenario B proves an OLDER muldiv survives the same age-based kill decision.
// =============================================================================

module tb_rv32i_ss_core_recover;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;

  localparam time CLK_PERIOD = 10ns;

  logic clk, rst_n;
  logic           decoded_valid, decoded_ready;
  word_t [1:0] decoded_pc, decoded_instr;
  arch_reg_t [1:0] decoded_rs1, decoded_rs2, decoded_rd;
  logic [1:0] decoded_rd_we, decoded_needs_checkpoint;
  ooo_op_class_e [1:0]  decoded_op_class;
  alu_op_e [1:0]        decoded_alu_op;
  br_type_e [1:0]       decoded_branch_op;
  ooo_src_sel_e [1:0] decoded_src1_sel, decoded_src2_sel;
  word_t [1:0]          decoded_imm;
  ooo_fu_class_e [1:0]  decoded_fu_class;
  muldiv_op_e [1:0]     decoded_muldiv_op;
  decoded_trap_t  decoded_trap;
  logic [1:0]           decoded_is_load;
  logic [1:0]           decoded_is_store;
  mem_size_e [1:0]      decoded_mem_size;
  logic [1:0]           decoded_mem_unsigned;
  logic           redirect_valid;
  word_t          redirect_target;
  logic [1:0]           commit_fire;
  commit_order_t  commit_order;
  word_t [1:0]          commit_pc, commit_inst;
  arch_reg_t [1:0]      commit_rd;
  logic [1:0]           commit_rd_wen;
  word_t [1:0]          commit_wdata;

  int errors = 0;
  int checks = 0;

  rv32i_ss_core dut (
    .clk(clk), .rst_n(rst_n),
    .decoded_valid(decoded_valid), .decoded_slot_valid({1'b0, decoded_valid}), .decoded_ready(decoded_ready),
    .decoded_pc(decoded_pc), .decoded_instr(decoded_instr),
    .decoded_rs1(decoded_rs1), .decoded_rs2(decoded_rs2), .decoded_rd(decoded_rd),
    .decoded_rd_we(decoded_rd_we), .decoded_needs_checkpoint(decoded_needs_checkpoint),
    .decoded_op_class(decoded_op_class), .decoded_alu_op(decoded_alu_op),
    .decoded_branch_op(decoded_branch_op),
    .decoded_src1_sel(decoded_src1_sel), .decoded_src2_sel(decoded_src2_sel),
    .decoded_imm(decoded_imm),
    .decoded_fu_class(decoded_fu_class), .decoded_muldiv_op(decoded_muldiv_op),
    .decoded_trap(decoded_trap),
    .decoded_is_load(decoded_is_load),
    .decoded_is_store(decoded_is_store),
    .decoded_mem_size(decoded_mem_size),
    .decoded_mem_unsigned(decoded_mem_unsigned),
    .decoded_pred_taken('0),  // tie-off: static not-taken prediction
    .decoded_pred_target('0),
    .decoded_csr_op           ('0),
    .decoded_csr_addr         ('0),
    .decoded_csr_zimm         ('0),
    .redirect_valid(redirect_valid), .redirect_target(redirect_target),
    .commit_fire(commit_fire), .commit_order(commit_order),
    .commit_pc(commit_pc), .commit_inst(commit_inst), .commit_rd(commit_rd),
    .commit_rd_wen(commit_rd_wen), .commit_wdata(commit_wdata)
  );

  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;
  initial begin
    repeat (2000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_recover exceeded 2000 cycles");
  end

  `define ROB dut.u_rob
  `define RN  dut.u_rename
  `define PRF dut.u_prf
  `define IQ  dut.u_iq
  `define FL  dut.u_free_list

  function automatic arch_reg_t areg(input int v); areg = arch_reg_t'(v); endfunction

  function automatic int iq_count();
    int n; n = 0;
    for (int i = 0; i < OOO_IQ_DEPTH; i++) if (`IQ.valid_q[i]) n++;
    iq_count = n;
  endfunction

  task automatic check_bit(input string name, input logic got, input logic exp);
    checks++;
    if (got !== exp) begin $error("[%s] got=%0b exp=%0b", name, got, exp); errors++; end
  endtask
  task automatic check_word(input string name, input word_t got, input word_t exp);
    checks++;
    if (got !== exp) begin $error("[%s] got=%0d (%08h) exp=%0d (%08h)", name, got, got, exp, exp); errors++; end
  endtask

  task automatic clear_inputs();
    decoded_valid = 1'b0; decoded_pc = '0; decoded_instr = '0;
    decoded_rs1 = '0; decoded_rs2 = '0; decoded_rd = '0;
    decoded_rd_we = 1'b0; decoded_needs_checkpoint = 1'b0;
    decoded_op_class = OOO_OP_ALU; decoded_alu_op = fyp_cpu_pkg::ALU_ADD;
    decoded_branch_op = rv32i_pipeline_pkg::BR_NONE;
    decoded_src1_sel = OOO_SRC_REG; decoded_src2_sel = OOO_SRC_REG; decoded_imm = '0;
    decoded_fu_class = OOO_FU_ALU; decoded_muldiv_op = rv32i_pipeline_pkg::MD_MUL;
    decoded_trap = '0;
    decoded_is_load = 1'b0;
    decoded_is_store = 1'b0;
    decoded_mem_size = fyp_cpu_pkg::MEM_W;
    decoded_mem_unsigned = 1'b0;
  endtask

  task automatic reset_dut();
    clear_inputs(); rst_n = 1'b0;
    repeat (3) @(posedge clk); rst_n = 1'b1; @(negedge clk);
  endtask

  task automatic disp(
    input word_t pc, input arch_reg_t rs1, input arch_reg_t rs2, input arch_reg_t rd,
    input logic rd_we, input ooo_op_class_e opc, input ooo_fu_class_e fu,
    input br_type_e bop, input muldiv_op_e mop,
    input ooo_src_sel_e s1, input ooo_src_sel_e s2, input word_t imm);
    @(negedge clk);
    decoded_valid = 1'b1; decoded_pc = pc;
    decoded_rs1 = rs1; decoded_rs2 = rs2; decoded_rd = rd; decoded_rd_we = rd_we;
    decoded_needs_checkpoint = (opc == OOO_OP_BRANCH);
    decoded_op_class = opc; decoded_fu_class = fu;
    decoded_branch_op = bop; decoded_muldiv_op = mop;
    decoded_src1_sel = s1; decoded_src2_sel = s2; decoded_imm = imm;
    decoded_alu_op = fyp_cpu_pkg::ALU_ADD;
    #1;
    while (decoded_ready !== 1'b1) @(negedge clk);
    @(posedge clk); @(negedge clk);
    decoded_valid = 1'b0;
  endtask

  // wait for the registered recovery pulse, then sample one posedge later (state applied)
  task automatic wait_recover_applied();
    int g; g = 0;
    while (dut.branch_recover_req !== 1'b1) begin
      @(negedge clk); g++;
      if (g > 100) $fatal(1, "recover never fired");
    end
    @(posedge clk); #1;
  endtask

  rob_idx_t  b_idx, y1_idx, y2_idx, m_idx, b2_idx, bc_idx, yc_idx;
  rob_seq_t  next_seq_before;
  int drain;

  // Scenario C arming: latch any recovery/redirect that fires during the
  // correctly-predicted not-taken window -- both must stay 0.
  bit arm_c = 1'b0;
  bit recover_seen_c = 1'b0;
  bit redirect_seen_c = 1'b0;
  always @(posedge clk) begin
    if (arm_c) begin
      if (dut.branch_recover_req) recover_seen_c  <= 1'b1;
      if (redirect_valid)         redirect_seen_c <= 1'b1;
    end
  end

  initial begin
    $display("[tb_rv32i_ss_core_recover] starting");

    // ======================================================================
    // Scenario A -- younger uops behind a taken branch are precisely recovered.
    // ======================================================================
    reset_dut();
    force `PRF.ready_q[5] = 1'b0;   // hold B's source x5 not-ready
    force `PRF.ready_q[6] = 1'b0;   // hold Y1's source x6 not-ready (sits in IQ)
    `PRF.regs_q[7]  = 32'd7;        // nonzero divisor for Y2's REM. Plain
                                    // deposit (Icarus cannot force an unpacked
                                    // array word); it persists because no FF
                                    // write ever targets p7 (alloc starts at p32)
                                    // and reset (which zeroes regs_q) is done.

    // B at ROB0: BEQ x5, x5 (taken when ready: x5==x5), creates checkpoint c.
    disp(32'h0000_1000, areg(5), areg(5), areg(0), 1'b0,
         OOO_OP_BRANCH, OOO_FU_ALU, rv32i_pipeline_pkg::BR_BEQ,
         rv32i_pipeline_pkg::MD_MUL, OOO_SRC_REG, OOO_SRC_REG, 32'd16);
    b_idx = rob_idx_t'(`ROB.tail_q - 1'b1);

    // branches no longer serialize -- younger uops dispatch behind the
    // unresolved branch B on their own. This is the natural speculative flow
    // The unresolved branch permits younger dispatch without a test override.

    // Y1 at ROB1: addi x10, x6, 7 -> allocates p32, src x6 not-ready -> sits in IQ.
    disp(32'h0000_1004, areg(6), areg(0), areg(10), 1'b1,
         OOO_OP_ALU, OOO_FU_ALU, rv32i_pipeline_pkg::BR_NONE,
         rv32i_pipeline_pkg::MD_MUL, OOO_SRC_REG, OOO_SRC_IMM, 32'd7);
    y1_idx = rob_idx_t'(`ROB.tail_q - 1'b1);

    // Y2 at ROB2: rem x11, x0, x7 -> issues to the muldiv FU, captures mask[c].
    // REM (0 rem 7, 32-cycle restoring divide), not MUL: the pipelined
    // multiplier is too fast to still be busy at the recovery, and a zero
    // divisor would take the div-by-zero fast path -- hence the forced x7.
    disp(32'h0000_1008, areg(0), areg(7), areg(11), 1'b1,
         OOO_OP_ALU, OOO_FU_MULDIV, rv32i_pipeline_pkg::BR_NONE,
         rv32i_pipeline_pkg::MD_REM, OOO_SRC_REG, OOO_SRC_REG, 32'd0);
    y2_idx = rob_idx_t'(`ROB.tail_q - 1'b1);
    clear_inputs();

    repeat (3) @(posedge clk); #1;   // let Y2 reach the muldiv FU

    // Pre-recovery snapshot: the wrong-path state is live.
    next_seq_before = `ROB.next_seq_q;
    check_word("pre: x10 -> p32",            word_t'(`RN.spec_map_q[10]), 32'd32);
    check_word("pre: x11 -> p33",            word_t'(`RN.spec_map_q[11]), 32'd33);
    check_bit ("pre: p32 allocated",         `FL.free_bits_q[32], 1'b0);
    check_bit ("pre: p33 allocated",         `FL.free_bits_q[33], 1'b0);
    check_bit ("pre: younger muldiv busy",   dut.muldiv_busy, 1'b1);

    // Release B -> issues taken in N, registered recovery fans out in N+1.
    force `PRF.ready_q[5] = 1'b1;
    wait_recover_applied();

    // ---- recovery invariants ----
    check_word("rename restored x10 -> p10", word_t'(`RN.spec_map_q[10]), 32'd10);
    check_word("rename restored x11 -> p11", word_t'(`RN.spec_map_q[11]), 32'd11);
    check_bit ("free-list reclaimed p32",    `FL.free_bits_q[32], 1'b1);
    check_bit ("free-list reclaimed p33",    `FL.free_bits_q[33], 1'b1);
    check_word("ROB tail rolled back to B+1",word_t'(`ROB.tail_q), word_t'(b_idx + 1'b1));
    check_bit ("ROB Y1 entry invalidated",   `ROB.valid_q[y1_idx], 1'b0);
    check_bit ("ROB Y2 entry invalidated",   `ROB.valid_q[y2_idx], 1'b0);
    check_word("next_seq did NOT roll back", word_t'(`ROB.next_seq_q), word_t'(next_seq_before));
    check_word("IQ drained of younger work", word_t'(iq_count()), 32'd0);
    check_bit ("younger muldiv was killed",  dut.muldiv_busy, 1'b0);

    // Correct path resumes: B alone commits in order.
    force `PRF.ready_q[6] = 1'b1;
    release `PRF.ready_q[5]; release `PRF.ready_q[6];
    drain = 0;
    while (`ROB.count_q !== '0) begin
      @(posedge clk); drain++;
      if (drain > 200) $fatal(1, "phaseA drain stuck: ROB.count=%0d", `ROB.count_q);
    end
    repeat (2) @(posedge clk);
    check_word("phaseA: only B committed (order==1)", word_t'(commit_order), 32'd1);

    // ======================================================================
    // Scenario B -- an OLDER muldiv without the branch's mask bit is NOT killed.
    // ======================================================================
    reset_dut();
    `PRF.regs_q[7] = 32'd7;         // nonzero divisor again, deposited after
                                    // the reset that zeroes regs_q (Scenario A note)

    // M at ROB0: rem x20, x0, x7 -> muldiv FU, branch_mask == 0 (no branch yet).
    // REM keeps the 32-cycle occupancy the older-survives proof needs.
    disp(32'h0000_2000, areg(0), areg(7), areg(20), 1'b1,
         OOO_OP_ALU, OOO_FU_MULDIV, rv32i_pipeline_pkg::BR_NONE,
         rv32i_pipeline_pkg::MD_REM, OOO_SRC_REG, OOO_SRC_REG, 32'd0);
    m_idx = rob_idx_t'(`ROB.tail_q - 1'b1);

    // at ROB1: taken branch, held not-ready briefly so M is firmly in the FU.
    force `PRF.ready_q[5] = 1'b0;
    disp(32'h0000_2004, areg(5), areg(5), areg(0), 1'b0,
         OOO_OP_BRANCH, OOO_FU_ALU, rv32i_pipeline_pkg::BR_BEQ,
         rv32i_pipeline_pkg::MD_MUL, OOO_SRC_REG, OOO_SRC_REG, 32'd16);
    b2_idx = rob_idx_t'(`ROB.tail_q - 1'b1);
    clear_inputs();
    repeat (2) @(posedge clk); #1;
    check_bit("phaseB pre: older muldiv busy", dut.muldiv_busy, 1'b1);

    force `PRF.ready_q[5] = 1'b1;
    wait_recover_applied();

    check_bit("phaseB: older muldiv NOT killed (mask mismatch)", dut.muldiv_busy, 1'b1);
    check_bit("phaseB: older muldiv entry preserved", `ROB.valid_q[m_idx], 1'b1);
    release `PRF.ready_q[5];

    drain = 0;
    while (`ROB.count_q !== '0) begin
      @(posedge clk); drain++;
      if (drain > 200) $fatal(1, "phaseB drain stuck: ROB.count=%0d", `ROB.count_q);
    end
    repeat (2) @(posedge clk);
    check_word("phaseB: M and B2 committed (order==2)", word_t'(commit_order), 32'd2);
    check_word("phaseB: M result x20 == 0", `PRF.regs_q[`RN.committed_map_q[20]], 32'd0);

    // ======================================================================
    // Scenario C -- a correctly-predicted NOT-taken branch flushes NOTHING.
    // invariant: mispredict-only redirect must not fire on a correct
    // prediction, else the already-fetched younger path is wrongly flushed
    // (or double-executed). Bc resolves not-taken while a younger uop sits
    // behind it; assert no recover, no redirect, and both retire in order.
    // ======================================================================
    reset_dut();
    recover_seen_c  = 1'b0;
    redirect_seen_c = 1'b0;
    arm_c           = 1'b1;

    // Bc at ROB0: BNE x5, x5 -> x5==x5 -> NOT taken (correct prediction).
    // Hold x5 not-ready so Yc dispatches behind the unresolved branch.
    force `PRF.ready_q[5] = 1'b0;
    disp(32'h0000_4000, areg(5), areg(5), areg(0), 1'b0,
         OOO_OP_BRANCH, OOO_FU_ALU, rv32i_pipeline_pkg::BR_BNE,
         rv32i_pipeline_pkg::MD_MUL, OOO_SRC_REG, OOO_SRC_REG, 32'd16);
    bc_idx = rob_idx_t'(`ROB.tail_q - 1'b1);

    // Yc at ROB1: addi x12, x0, 42 -> independent + ready, flows behind Bc.
    disp(32'h0000_4004, areg(0), areg(0), areg(12), 1'b1,
         OOO_OP_ALU, OOO_FU_ALU, rv32i_pipeline_pkg::BR_NONE,
         rv32i_pipeline_pkg::MD_MUL, OOO_SRC_REG, OOO_SRC_IMM, 32'd42);
    yc_idx = rob_idx_t'(`ROB.tail_q - 1'b1);
    clear_inputs();

    repeat (3) @(posedge clk); #1;   // Yc issues OoO; Bc waits on x5

    force `PRF.ready_q[5] = 1'b1;     // release Bc -> resolves NOT taken
    repeat (4) @(posedge clk); #1;
    release `PRF.ready_q[5];

    drain = 0;
    while (`ROB.count_q !== '0) begin
      @(posedge clk); drain++;
      if (drain > 200) $fatal(1, "phaseC drain stuck: ROB.count=%0d", `ROB.count_q);
    end
    repeat (2) @(posedge clk);
    arm_c = 1'b0;

    check_bit ("phaseC: not-taken fired NO recovery",        recover_seen_c, 1'b0);
    check_bit ("phaseC: not-taken fired NO redirect",        redirect_seen_c, 1'b0);
    check_word("phaseC: Bc and Yc both committed (order==2)", word_t'(commit_order), 32'd2);
    check_word("phaseC: younger Yc result x12 == 42",
               `PRF.regs_q[`RN.committed_map_q[12]], 32'd42);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_core_recover] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_core_recover] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
