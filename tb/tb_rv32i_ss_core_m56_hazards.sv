`timescale 1ns/1ps

// tb_rv32i_ss_core_m56_hazards.sv — recovery issue gating and checkpoint reuse.
//
// Recovery-cycle issue gating: branch A issues alone and requests recovery.
// An older independent ALU op C is woken during the registered broadcast,
// making it valid, operand-ready and FU-bindable on that exact cycle. The
// test requires issue_valid without issue_fire, then observes C issuing on
// the next cycle and retiring. A younger candidate alone would not challenge
// this gate because recovery would kill it.
//
// Checkpoint reuse: branch A resolves correctly and frees checkpoint 0 while
// an older surviving instruction Y remains in the IQ with branch_mask[0].
// Branch B reuses checkpoint 0 and recovers. The stale mask bit must not kill
// Y; recovery uses ROB ring age and Y must eventually commit.

module tb_rv32i_ss_core_m56_hazards;
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
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_m56_hazards exceeded 2000 cycles");
  end

  `define ROB dut.u_rob
  `define RN  dut.u_rename
  `define PRF dut.u_prf
  `define IQ  dut.u_iq

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
    if (got !== exp) begin $error("[%s] got=%0d exp=%0d", name, got, exp); errors++; end
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
    input br_type_e bop, input ooo_src_sel_e s1, input ooo_src_sel_e s2, input word_t imm);
    @(negedge clk);
    decoded_valid = 1'b1; decoded_pc = pc;
    decoded_rs1 = rs1; decoded_rs2 = rs2; decoded_rd = rd; decoded_rd_we = rd_we;
    decoded_needs_checkpoint = (opc == OOO_OP_BRANCH);
    decoded_op_class = opc; decoded_fu_class = fu;
    // MD_REM, not MD_MUL: only this test's OOO_FU_MULDIV packet consumes this, and
    // it needs the DIV family's 32-cycle occupancy now that MUL is 2-stage.
    decoded_branch_op = bop; decoded_muldiv_op = rv32i_pipeline_pkg::MD_REM;
    decoded_src1_sel = s1; decoded_src2_sel = s2; decoded_imm = imm;
    decoded_alu_op = fyp_cpu_pkg::ALU_ADD;
    #1;
    while (decoded_ready !== 1'b1) @(negedge clk);
    @(posedge clk); @(negedge clk);
    decoded_valid = 1'b0;
  endtask

  // ---- B invariant + recovery monitors -------------------------------
  // issue_during_recover: exec must never fire during a recovery cycle (the
  // issue gate enforces this). recover_pulses: distinct recovery events,
  //   so a phase can prove its own recovery actually fired (non-vacuous).
  bit issue_during_recover = 1'b0;
  bit recover_fired = 1'b0;
  bit brr_d = 1'b0;
  int recover_pulses = 0;
  always @(posedge clk) begin
    if (rst_n && dut.branch_recover_req && (|dut.issue_fire)) issue_during_recover <= 1'b1;
    if (rst_n && dut.branch_recover_req) recover_fired <= 1'b1;
    if (rst_n && dut.branch_recover_req && !brr_d) recover_pulses <= recover_pulses + 1;
    brr_d <= rst_n && dut.branch_recover_req;
  end

  int drain;
  int p3_rec0;
  int p1_wait;

  initial begin
    $display("[tb_rv32i_ss_core_m56_hazards] starting");

    // ======================================================================
    // Scenario 1 -- Blocker B: wrong-path branch issues during the recovery cycle.
    // Hold BOTH branches not-ready so they sit in the IQ together, then release
    // together: A (older) issues, B issues the NEXT cycle = A's recovery cycle.
    // ======================================================================
    reset_dut();
    force `PRF.ready_q[7] = 1'b0;   // hold C (older ALU op, reads x7)
    force `PRF.ready_q[5] = 1'b0;   // hold A (BEQ x5,x5)
    // C FIRST (older): it survives A's recovery, so the broadcast gate is the
    // ONLY thing that can withhold its grant on the broadcast cycle.
    disp(32'h0000_0FFC, areg(7), areg(0), areg(9), 1'b1,
         OOO_OP_ALU, OOO_FU_ALU, rv32i_pipeline_pkg::BR_NONE,
         OOO_SRC_REG, OOO_SRC_IMM, 32'd1);
    disp(32'h0000_1000, areg(5), areg(5), areg(0), 1'b0,
         OOO_OP_BRANCH, OOO_FU_ALU, rv32i_pipeline_pkg::BR_BEQ,
         OOO_SRC_REG, OOO_SRC_REG, 32'd16);
    clear_inputs();
    repeat (2) @(posedge clk);
    check_word("P1 setup: C+A both in IQ", word_t'(iq_count()), 32'd2);
    @(negedge clk);                // release A ALONE: it issues and recovers
    force `PRF.ready_q[5] = 1'b1;
    p1_wait = 0;
    while (dut.branch_recover_req !== 1'b1) begin
      @(posedge clk); #1;
      p1_wait++;
      if (p1_wait > 10) $fatal(1, "P1: A's recovery broadcast never fired");
    end
    // Inside A's broadcast cycle: wake C at the mid-cycle negedge. ready_vec
    // is combinational, so C becomes a valid+ready+bindable candidate THIS
    // cycle, and only the broadcast gate can be withholding the grant.
    @(negedge clk);
    force `PRF.ready_q[7] = 1'b1;
    #1;
    check_bit("P1: gate CHALLENGED (candidate ready, no fire, broadcast live)",
              dut.branch_recover_req && dut.iq_select_valid[0] &&
              !dut.iq_select_accept && !(|dut.issue_fire),
              1'b1);
    @(posedge clk); #1;            // first post-broadcast cycle
    check_bit("P1: survivor selected immediately after the broadcast",
              dut.iq_select_valid[0] && dut.iq_select_accept, 1'b1);
    @(posedge clk); #1;
    check_bit("P1: survivor transfers after the select register",
              dut.issue_fire[0], 1'b1);
    repeat (6) @(posedge clk);
    check_bit("P1: A actually recovered (non-vacuous)", recover_fired, 1'b1);
    check_bit("P1: no issue_fire during branch_recover_req", issue_during_recover, 1'b0);
    release `PRF.ready_q[5]; release `PRF.ready_q[7];
    drain = 0;
    while (`ROB.count_q !== '0 && drain < 200) begin @(posedge clk); drain++; end

    // ======================================================================
    // Scenario 2 -- Blocker A: stale checkpoint-mask kills an older survivor.
    // Hold A so Y dispatches UNDER it (branch_mask[0]=1); hold Y so it lingers.
    // ======================================================================
    reset_dut();
    force `PRF.ready_q[5] = 1'b0;   // hold A so Y dispatches under it
    force `PRF.ready_q[7] = 1'b0;   // hold Y -> lingers in the IQ

    // A at ROB0: BNE x5,x5 -> NOT taken. Correct resolve frees checkpoint 0.
    disp(32'h0000_2000, areg(5), areg(5), areg(0), 1'b0,
         OOO_OP_BRANCH, OOO_FU_ALU, rv32i_pipeline_pkg::BR_BNE,
         OOO_SRC_REG, OOO_SRC_REG, 32'd16);
    // Y at ROB1 (under A, branch_mask[0]=1): addi x10, x7, 1 -> stuck (x7 held).
    disp(32'h0000_2004, areg(7), areg(0), areg(10), 1'b1,
         OOO_OP_ALU, OOO_FU_ALU, rv32i_pipeline_pkg::BR_NONE,
         OOO_SRC_REG, OOO_SRC_IMM, 32'd1);
    clear_inputs();
    repeat (2) @(posedge clk);
    @(negedge clk);
    force `PRF.ready_q[5] = 1'b1;   // release A -> resolves not-taken, frees ckpt 0
    repeat (4) @(posedge clk);
    check_word("P2 pre: Y lingering in IQ", word_t'(iq_count()), 32'd1);

    // B at ROB2: BEQ x8,x8 -> taken; needs_checkpoint -> REUSES freed checkpoint 0.
    disp(32'h0000_2008, areg(8), areg(8), areg(0), 1'b0,
         OOO_OP_BRANCH, OOO_FU_ALU, rv32i_pipeline_pkg::BR_BEQ,
         OOO_SRC_REG, OOO_SRC_REG, 32'd16);
    clear_inputs();
    repeat (5) @(posedge clk);   // includes full-picker select_q stage

    // Y is OLDER than B -> must survive B's recovery. Stale mask[0] must NOT kill it.
    check_word("P2: older survivor Y NOT killed (iq_count==1)", word_t'(iq_count()), 32'd1);

    // release Y's source; Y must issue, complete, and commit.
    force `PRF.ready_q[7] = 1'b1;
    release `PRF.ready_q[5]; release `PRF.ready_q[7];
    drain = 0;
    while (`ROB.count_q !== '0 && drain < 300) begin @(posedge clk); drain++; end
    if (`ROB.count_q !== '0) begin
      $error("P2 RED: Y never retired (ROB head stall), count=%0d", `ROB.count_q);
      errors++;
    end else begin
      repeat (2) @(posedge clk);
      check_word("P2: Y committed result x10==1", `PRF.regs_q[`RN.committed_map_q[10]], 32'd1);
    end

    // ======================================================================
    // Scenario 3 -- Blocker A (muldiv holder): stale checkpoint-mask kills an
    // older IN-FLIGHT muldiv op. A long-latency REM dispatches UNDER A
    // (branch_mask[0]=1) and starts running in the muldiv unit. A resolves
    // not-taken, freeing checkpoint 0. Branch B reuses checkpoint 0 and
    // recovers; the stale bit aliases and the mask-kill falsely resets the
    // OLDER op. Ring-distance spares it (it is older than B by rob_idx).
    // Assert it survives recovery and retires. A mask-based kill would
    // leave its ROB entry incomplete and stall the head. REM provides
    // the long occupancy needed to enter this overlap.
    // ======================================================================
    reset_dut();
    force `PRF.ready_q[5] = 1'b0;   // hold A so the REM dispatches under it
    `PRF.regs_q[7]  = 32'd7;        // nonzero divisor, plain deposit (Icarus
                                    // cannot force an unpacked-array word; p7 is
                                    // never FF-written, so the value persists);
                                    // rs2==x0 would hit the div-by-zero fast path

    // A at ROB0: BNE x5,x5 -> NOT taken. Correct resolve frees checkpoint 0.
    disp(32'h0000_3000, areg(5), areg(5), areg(0), 1'b0,
         OOO_OP_BRANCH, OOO_FU_ALU, rv32i_pipeline_pkg::BR_BNE,
         OOO_SRC_REG, OOO_SRC_REG, 32'd16);
    // REM at ROB1 (under A, branch_mask[0]=1): rem x10,x0,x7 -> long latency.
    // Both sources are ready (x0 and the forced x7), so it issues + starts
    // immediately.
    disp(32'h0000_3004, areg(0), areg(7), areg(10), 1'b1,
         OOO_OP_ALU, OOO_FU_MULDIV, rv32i_pipeline_pkg::BR_NONE,
         OOO_SRC_REG, OOO_SRC_REG, 32'd0);
    clear_inputs();
    repeat (6) @(posedge clk);
    check_bit("P3 pre: muldiv op running under A", dut.muldiv_busy, 1'b1);

    @(negedge clk);
    force `PRF.ready_q[5] = 1'b1;   // release A -> resolves not-taken, frees ckpt 0
    repeat (3) @(posedge clk);

    p3_rec0 = recover_pulses;
    // B at ROB2: BEQ x8,x8 -> taken; needs_checkpoint -> REUSES freed checkpoint 0.
    disp(32'h0000_3008, areg(8), areg(8), areg(0), 1'b0,
         OOO_OP_BRANCH, OOO_FU_ALU, rv32i_pipeline_pkg::BR_BEQ,
         OOO_SRC_REG, OOO_SRC_REG, 32'd16);
    clear_inputs();
    repeat (5) @(posedge clk);   // includes full-picker select_q stage

    check_bit("P3: B actually recovered (non-vacuous)", (recover_pulses > p3_rec0), 1'b1);
    // The REM is OLDER than B -> must survive. Stale mask[0] must NOT reset it.
    check_bit("P3: older in-flight muldiv survives B's recovery", dut.muldiv_busy, 1'b1);

    release `PRF.ready_q[5];
    drain = 0;
    while (`ROB.count_q !== '0 && drain < 300) begin @(posedge clk); drain++; end
    if (`ROB.count_q !== '0) begin
      $error("P3 RED: muldiv op never retired (ROB head stall), count=%0d", `ROB.count_q);
      errors++;
    end

    if (errors == 0) begin
      $display("[tb_rv32i_ss_core_m56_hazards] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_core_m56_hazards] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
