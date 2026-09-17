// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// true IQ issue TB for rv32i_ss_core.
//
// Proves issue is no longer pinned to the ROB head. The older head uop is held
// not-ready by forcing its source ready bit low, while younger independent uops
// are ready. Younger uops may issue/write back first, but commit_order must stay
// blocked until the older head later completes.

module tb_rv32i_ss_core_ooo_issue;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;

  localparam time CLK_PERIOD = 10ns;

  logic clk;
  logic rst_n;

  logic           decoded_valid;
  logic           decoded_ready;
  word_t [1:0]          decoded_pc;
  word_t [1:0]          decoded_instr;
  arch_reg_t [1:0]      decoded_rs1;
  arch_reg_t [1:0]      decoded_rs2;
  arch_reg_t [1:0]      decoded_rd;
  logic [1:0]           decoded_rd_we;
  logic [1:0]           decoded_needs_checkpoint;
  ooo_op_class_e [1:0]  decoded_op_class;
  alu_op_e [1:0]        decoded_alu_op;
  br_type_e [1:0]       decoded_branch_op;
  ooo_src_sel_e [1:0]   decoded_src1_sel;
  ooo_src_sel_e [1:0]   decoded_src2_sel;
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
  word_t [1:0]          commit_pc;
  word_t [1:0]          commit_inst;
  arch_reg_t [1:0]      commit_rd;
  logic [1:0]           commit_rd_wen;
  word_t [1:0]          commit_wdata;

  int errors = 0;
  int checks = 0;

  rv32i_ss_core dut (
    .clk                      (clk),
    .rst_n                    (rst_n),
    .decoded_valid            (decoded_valid),
    .decoded_slot_valid       ({1'b0, decoded_valid}),
    .decoded_ready            (decoded_ready),
    .decoded_pc               (decoded_pc),
    .decoded_instr            (decoded_instr),
    .decoded_rs1              (decoded_rs1),
    .decoded_rs2              (decoded_rs2),
    .decoded_rd               (decoded_rd),
    .decoded_rd_we            (decoded_rd_we),
    .decoded_needs_checkpoint (decoded_needs_checkpoint),
    .decoded_op_class         (decoded_op_class),
    .decoded_alu_op           (decoded_alu_op),
    .decoded_branch_op        (decoded_branch_op),
    .decoded_src1_sel         (decoded_src1_sel),
    .decoded_src2_sel         (decoded_src2_sel),
    .decoded_imm              (decoded_imm),
    .decoded_fu_class         (decoded_fu_class),
    .decoded_muldiv_op        (decoded_muldiv_op),
    .decoded_trap             (decoded_trap),
    .decoded_is_load          (decoded_is_load),
    .decoded_is_store         (decoded_is_store),
    .decoded_mem_size         (decoded_mem_size),
    .decoded_mem_unsigned     (decoded_mem_unsigned),
    .decoded_pred_taken('0),  // tie-off: static not-taken prediction
    .decoded_pred_target('0),
    .decoded_csr_op           ('0),
    .decoded_csr_addr         ('0),
    .decoded_csr_zimm         ('0),
    .redirect_valid           (redirect_valid),
    .redirect_target          (redirect_target),
    .commit_fire              (commit_fire),
    .commit_order             (commit_order),
    .commit_pc                (commit_pc),
    .commit_inst              (commit_inst),
    .commit_rd                (commit_rd),
    .commit_rd_wen            (commit_rd_wen),
    .commit_wdata             (commit_wdata)
  );

  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;

  initial begin
    repeat (1000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_ooo_issue exceeded 1000 cycles");
  end

  `define ROB dut.u_rob
  `define RN  dut.u_rename
  `define PRF dut.u_prf

  function automatic arch_reg_t areg(input int v);
    areg = arch_reg_t'(v);
  endfunction

  task automatic clear_inputs();
    decoded_valid            = 1'b0;
    decoded_pc               = '0;
    decoded_instr            = '0;
    decoded_rs1              = '0;
    decoded_rs2              = '0;
    decoded_rd               = '0;
    decoded_rd_we            = 1'b0;
    decoded_needs_checkpoint = 1'b0;
    decoded_op_class         = OOO_OP_ALU;
    decoded_alu_op           = fyp_cpu_pkg::ALU_ADD;
    decoded_branch_op        = rv32i_pipeline_pkg::BR_NONE;
    decoded_src1_sel         = OOO_SRC_REG;
    decoded_src2_sel         = OOO_SRC_REG;
    decoded_imm              = '0;
    decoded_fu_class         = OOO_FU_ALU;
    decoded_muldiv_op        = rv32i_pipeline_pkg::MD_MUL;
    decoded_trap             = '0;
    decoded_is_load          = 1'b0;
    decoded_is_store         = 1'b0;
    decoded_mem_size         = fyp_cpu_pkg::MEM_W;
    decoded_mem_unsigned     = 1'b0;
  endtask

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

  task automatic reset_dut();
    clear_inputs();
    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    @(negedge clk);
  endtask

  task automatic disp_alu(
    input word_t         pc,
    input arch_reg_t     rs1,
    input arch_reg_t     rs2,
    input arch_reg_t     rd,
    input ooo_src_sel_e  src1_sel,
    input ooo_src_sel_e  src2_sel,
    input word_t         imm
  );
    @(negedge clk);
    decoded_valid    = 1'b1;
    decoded_pc       = pc;
    decoded_rs1      = rs1;
    decoded_rs2      = rs2;
    decoded_rd       = rd;
    decoded_rd_we    = 1'b1;
    decoded_op_class = OOO_OP_ALU;
    decoded_alu_op   = fyp_cpu_pkg::ALU_ADD;
    decoded_src1_sel = src1_sel;
    decoded_src2_sel = src2_sel;
    decoded_imm      = imm;

    #1;
    while (decoded_ready !== 1'b1) @(negedge clk);
    @(posedge clk);
    @(negedge clk);
    decoded_valid = 1'b0;
  endtask

  task automatic disp_pkt(
    input word_t         pc,
    input arch_reg_t     rs1,
    input arch_reg_t     rs2,
    input arch_reg_t     rd,
    input logic          rd_we,
    input ooo_op_class_e op_class,
    input alu_op_e       alu_op,
    input br_type_e      branch_op,
    input ooo_src_sel_e  src1_sel,
    input ooo_src_sel_e  src2_sel,
    input word_t         imm
  );
    @(negedge clk);
    decoded_valid    = 1'b1;
    decoded_pc       = pc;
    decoded_rs1      = rs1;
    decoded_rs2      = rs2;
    decoded_rd       = rd;
    decoded_rd_we    = rd_we;
    decoded_needs_checkpoint = (op_class == OOO_OP_BRANCH);
    decoded_op_class = op_class;
    decoded_alu_op   = alu_op;
    decoded_branch_op = branch_op;
    decoded_src1_sel = src1_sel;
    decoded_src2_sel = src2_sel;
    decoded_imm      = imm;

    #1;
    while (decoded_ready !== 1'b1) @(negedge clk);
    @(posedge clk);
    @(negedge clk);
    decoded_valid = 1'b0;
  endtask

  task automatic wait_for_cdb();
    int wait_count;
    wait_count = 0;
    // Poll mid-cycle (negedge): with back-to-back completions the
    // CDB payload re-latches every posedge, so a posedge-exit waiter can
    // detect one event's valid and then read the NEXT event's payload.
    while (dut.cdb_q[0].valid !== 1'b1) begin
      @(negedge clk);
      wait_count++;
      if (wait_count > 100) $fatal(1, "CDB wait timed out");
    end
    #1;
  endtask

  task automatic wait_for_dual_cdb();
    int wait_count;
    wait_count = 0;
    while (!(dut.cdb_q[0].valid === 1'b1 &&
             dut.cdb_q[1].valid === 1'b1)) begin
      @(negedge clk);
      wait_count++;
      if (wait_count > 100) $fatal(1, "dual CDB wait timed out");
    end
    #1;
  endtask

  task automatic wait_for_redirect();
    int wait_count;
    wait_count = 0;
    #1;
    while (redirect_valid !== 1'b1) begin
      @(negedge clk);
      #1;
      wait_count++;
      if (wait_count > 100) $fatal(1, "redirect wait timed out");
    end
  endtask

  rob_idx_t  older_idx;
  rob_idx_t  younger_idx;
  rob_idx_t  younger2_idx;
  phys_reg_t x2_p;
  phys_reg_t x3_p;
  phys_reg_t x4_p;
  int drain;

  initial begin
    $display("[tb_rv32i_ss_core_ooo_issue] starting");

    reset_dut();

    // Older uop at ROB0: add x2, x1, x0. Its src p1 is forced not-ready so
    // it remains the unissued head.
    disp_alu(32'h0000_1000, areg(1), areg(0), areg(2),
             OOO_SRC_REG, OOO_SRC_REG, 32'd0);
    older_idx = rob_idx_t'(`ROB.tail_q - 1'b1);
    force `PRF.ready_q[1] = 1'b0;

    // Younger uop at ROB1: addi x3, x0, 9. Its rs2 bits are not a real source
    // because src2_sel=IMM, so it is ready even if that stale prs is not ready.
    disp_alu(32'h0000_1004, areg(0), areg(1), areg(3),
             OOO_SRC_REG, OOO_SRC_IMM, 32'd9);
    younger_idx = rob_idx_t'(`ROB.tail_q - 1'b1);
    clear_inputs();

    wait_for_cdb();
    check_word("first CDB is younger ROB idx", word_t'(dut.cdb_q[0].rob_idx),
               word_t'(younger_idx));
    check_bit("younger issued before older", dut.cdb_q[0].rob_idx != older_idx, 1'b1);
    check_word("younger result is 9", dut.cdb_q[0].result, 32'd9);
    check_word("commit blocked by older head", word_t'(commit_order), 32'd0);
    check_bit("older head still not done", `ROB.done_q[older_idx], 1'b0);

    @(posedge clk);
    #1;
    check_bit("younger accepted as done", `ROB.done_q[younger_idx], 1'b1);
    check_bit("older still not done after younger wb", `ROB.done_q[older_idx], 1'b0);
    check_word("commit still waits for older", word_t'(commit_order), 32'd0);

    // The force masked the PRF flop driver; explicitly force the source ready
    // high for the second phase so the older head can now issue.
    force `PRF.ready_q[1] = 1'b1;

    drain = 0;
    while (`ROB.count_q !== '0) begin
      @(posedge clk);
      drain++;
      if (drain > 100) $fatal(1, "drain stuck: ROB.count=%0d", `ROB.count_q);
    end
    repeat (2) @(posedge clk);

    x2_p = `RN.committed_map_q[2];
    x3_p = `RN.committed_map_q[3];

    check_word("commit_order==2", word_t'(commit_order), 32'd2);
    check_word("older x2 result is 0", `PRF.regs_q[x2_p], 32'd0);
    check_word("younger x3 result is 9", `PRF.regs_q[x3_p], 32'd9);
    check_bit("x2/x3 physregs differ", x2_p != x3_p, 1'b1);

    release `PRF.ready_q[1];

    // ---- Hardening 1: among multiple ready younger entries, IQ picks oldest
    // ready by ROB ring age (ROB1 before ROB2), while commit stays blocked.
    reset_dut();

    disp_alu(32'h0000_2000, areg(1), areg(0), areg(2),
             OOO_SRC_REG, OOO_SRC_REG, 32'd0);
    older_idx = rob_idx_t'(`ROB.tail_q - 1'b1);
    force `PRF.ready_q[1] = 1'b0;
    force `PRF.ready_q[5] = 1'b0;

    disp_alu(32'h0000_2004, areg(5), areg(1), areg(3),
             OOO_SRC_REG, OOO_SRC_IMM, 32'd9);
    younger_idx = rob_idx_t'(`ROB.tail_q - 1'b1);

    disp_alu(32'h0000_2008, areg(5), areg(1), areg(4),
             OOO_SRC_REG, OOO_SRC_IMM, 32'd11);
    younger2_idx = rob_idx_t'(`ROB.tail_q - 1'b1);
    clear_inputs();
    force `PRF.ready_q[5] = 1'b1;

    wait_for_dual_cdb();
    check_word("multi first CDB is ROB1", word_t'(dut.cdb_q[0].rob_idx),
               word_t'(younger_idx));
    check_word("multi first result is 9", dut.cdb_q[0].result, 32'd9);
    check_word("multi second CDB is ROB2", word_t'(dut.cdb_q[1].rob_idx),
               word_t'(younger2_idx));
    check_word("multi second result is 11", dut.cdb_q[1].result, 32'd11);
    check_word("multi commit still blocked by head", word_t'(commit_order), 32'd0);
    @(posedge clk);
    #1;

    force `PRF.ready_q[1] = 1'b1;

    drain = 0;
    while (`ROB.count_q !== '0) begin
      @(posedge clk);
      drain++;
      if (drain > 100) $fatal(1, "multi drain stuck: ROB.count=%0d", `ROB.count_q);
    end
    repeat (2) @(posedge clk);

    x3_p = `RN.committed_map_q[3];
    x4_p = `RN.committed_map_q[4];

    check_word("multi commit_order==3", word_t'(commit_order), 32'd3);
    check_word("multi younger x3 result", `PRF.regs_q[x3_p], 32'd9);
    check_word("multi younger x4 result", `PRF.regs_q[x4_p], 32'd11);

    release `PRF.ready_q[1];
    release `PRF.ready_q[5];

    // ---- Hardening 2: a ready branch younger than a blocked head resolves
    // through the IQ issue path (OoO issue) and redirects, yet cannot COMMIT
    // until the older blocked head retires. Under the taken branch is a
    // mispredict -> registered recover -> redirect at N+1. No younger work
    // exists here only because the test dispatches none (branches no longer
    // serialize dispatch).
    reset_dut();

    disp_alu(32'h0000_3000, areg(1), areg(0), areg(2),
             OOO_SRC_REG, OOO_SRC_REG, 32'd0);
    older_idx = rob_idx_t'(`ROB.tail_q - 1'b1);
    force `PRF.ready_q[1] = 1'b0;

    disp_pkt(32'h0000_3004, areg(0), areg(0), areg(0), 1'b0,
             OOO_OP_BRANCH, fyp_cpu_pkg::ALU_ADD, rv32i_pipeline_pkg::BR_BEQ,
             OOO_SRC_REG, OOO_SRC_REG, 32'd8);
    clear_inputs();

    wait_for_redirect();
    check_bit("branch redirect fires before older done", redirect_valid, 1'b1);
    check_word("branch redirect target", redirect_target, 32'h0000_300c);
    check_word("branch commit blocked by older head", word_t'(commit_order), 32'd0);
    check_bit("branch older head still not done", `ROB.done_q[older_idx], 1'b0);

    @(posedge clk);
    #1;
    // the registered recovery pulse is one cycle wide -- it must self-clear
    // so dispatch (gated by ~branch_recover_req) reopens. Branches no longer use a
    // serialization gate; only jumps do (jump_inflight_q).
    check_bit("branch recover pulse self-clears", dut.branch_recover_req, 1'b0);

    force `PRF.ready_q[1] = 1'b1;

    drain = 0;
    while (`ROB.count_q !== '0) begin
      @(posedge clk);
      drain++;
      if (drain > 100) $fatal(1, "branch drain stuck: ROB.count=%0d", `ROB.count_q);
    end
    repeat (2) @(posedge clk);

    check_word("branch commit_order==2", word_t'(commit_order), 32'd2);

    release `PRF.ready_q[1];

    if (errors == 0) begin
      $display("[tb_rv32i_ss_core_ooo_issue] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_core_ooo_issue] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
