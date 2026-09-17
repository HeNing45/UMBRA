// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// natural-OoO proof for rv32i_ss_core.
//
// No forced ready bits: an older muldiv occupies the muldiv FU naturally, then
// a younger independent ALU issues and writes back before that older muldiv
// completes. The ROB must still commit in program order.

module tb_rv32i_ss_core_m45_natural;
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
  ooo_fu_class_e [1:0]  decoded_fu_class;
  alu_op_e [1:0]        decoded_alu_op;
  muldiv_op_e [1:0]     decoded_muldiv_op;
  br_type_e [1:0]       decoded_branch_op;
  ooo_src_sel_e [1:0]   decoded_src1_sel;
  ooo_src_sel_e [1:0]   decoded_src2_sel;
  word_t [1:0]          decoded_imm;
  decoded_trap_t  decoded_trap;
  logic [1:0]           decoded_is_load;
  logic [1:0]           decoded_is_store;
  mem_size_e [1:0]      decoded_mem_size;
  logic [1:0]           decoded_mem_unsigned;

  logic          redirect_valid;
  word_t         redirect_target;
  logic [1:0]          commit_fire;
  commit_order_t commit_order;
  word_t [1:0]         commit_pc;
  word_t [1:0]         commit_inst;
  arch_reg_t [1:0]     commit_rd;
  logic [1:0]          commit_rd_wen;
  word_t [1:0]         commit_wdata;

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
    .decoded_fu_class         (decoded_fu_class),
    .decoded_alu_op           (decoded_alu_op),
    .decoded_muldiv_op        (decoded_muldiv_op),
    .decoded_branch_op        (decoded_branch_op),
    .decoded_src1_sel         (decoded_src1_sel),
    .decoded_src2_sel         (decoded_src2_sel),
    .decoded_imm              (decoded_imm),
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
    repeat (5000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_m45_natural exceeded 5000 cycles");
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
    decoded_fu_class         = OOO_FU_ALU;
    decoded_alu_op           = fyp_cpu_pkg::ALU_ADD;
    decoded_muldiv_op        = rv32i_pipeline_pkg::MD_NONE;
    decoded_branch_op        = rv32i_pipeline_pkg::BR_NONE;
    decoded_src1_sel         = OOO_SRC_REG;
    decoded_src2_sel         = OOO_SRC_REG;
    decoded_imm              = '0;
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
    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    @(negedge clk);
  endtask

  task automatic disp(
    input word_t         pc,
    input arch_reg_t     rs1,
    input arch_reg_t     rs2,
    input arch_reg_t     rd,
    input ooo_fu_class_e fu_class,
    input alu_op_e       alu_op,
    input muldiv_op_e    md_op,
    input ooo_src_sel_e  src1_sel,
    input ooo_src_sel_e  src2_sel,
    input word_t         imm
  );
    int wait_cycles;
    @(negedge clk);
    decoded_valid     = 1'b1;
    decoded_pc        = pc;
    decoded_instr     = pc;
    decoded_rs1       = rs1;
    decoded_rs2       = rs2;
    decoded_rd        = rd;
    decoded_rd_we     = 1'b1;
    decoded_op_class  = OOO_OP_ALU;
    decoded_fu_class  = fu_class;
    decoded_alu_op    = alu_op;
    decoded_muldiv_op = md_op;
    decoded_src1_sel  = src1_sel;
    decoded_src2_sel  = src2_sel;
    decoded_imm       = imm;

    #1;
    wait_cycles = 0;
    while (decoded_ready !== 1'b1) begin
      @(negedge clk);
      wait_cycles++;
      if (wait_cycles > 200) begin
        $fatal(1, "dispatch stuck pc=%08h ROB.count=%0d iq_alloc_ready=%0b",
               pc, `ROB.count_q, dut.iq_alloc_ready);
      end
    end
    @(posedge clk);
    @(negedge clk);
    decoded_valid = 1'b0;
  endtask

  task automatic drain_rob(input int max_cycles);
    int drain;
    drain = 0;
    while (`ROB.count_q !== '0) begin
      @(posedge clk);
      drain++;
      if (drain > max_cycles) begin
        $fatal(1, "drain stuck: ROB.count=%0d", `ROB.count_q);
      end
    end
    repeat (2) @(posedge clk);
  endtask

  task automatic wait_for_cdb(input int max_cycles);
    int wait_count;
    wait_count = 0;
    while (dut.cdb_q[0].valid !== 1'b1) begin
      @(posedge clk);
      wait_count++;
      if (wait_count > max_cycles) begin
        $fatal(1, "CDB wait timed out");
      end
    end
    #1;
  endtask

  rob_idx_t  mul_idx;
  rob_idx_t  alu_idx;
  phys_reg_t x3_p;
  phys_reg_t x4_p;

  initial begin
    $display("[tb_rv32i_ss_core_m45_natural] starting");

    reset_dut();

    // Establish x1=6, x2=7, then drain so the real proof starts clean.
    disp(32'h0000_5000, areg(0), areg(0), areg(1), OOO_FU_ALU,
         fyp_cpu_pkg::ALU_ADD, rv32i_pipeline_pkg::MD_NONE,
         OOO_SRC_REG, OOO_SRC_IMM, 32'd6);
    disp(32'h0000_5004, areg(0), areg(0), areg(2), OOO_FU_ALU,
         fyp_cpu_pkg::ALU_ADD, rv32i_pipeline_pkg::MD_NONE,
         OOO_SRC_REG, OOO_SRC_IMM, 32'd7);
    clear_inputs();
    drain_rob(100);
    check_word("setup commit_order==2", word_t'(commit_order), 32'd2);

    // Older long-latency op at lower ROB age. REM, not MUL: the DIV family
    // keeps the 32-cycle occupancy this natural-order proof needs now that
    // the multiplier is 2-stage pipelined.
    disp(32'h0000_5008, areg(1), areg(2), areg(3), OOO_FU_MULDIV,
         fyp_cpu_pkg::ALU_ADD, rv32i_pipeline_pkg::MD_REM,
         OOO_SRC_REG, OOO_SRC_REG, '0);
    mul_idx = rob_idx_t'(`ROB.tail_q - 1'b1);

    // Younger independent ALU. No force: all operands are naturally ready.
    disp(32'h0000_500c, areg(0), areg(0), areg(4), OOO_FU_ALU,
         fyp_cpu_pkg::ALU_ADD, rv32i_pipeline_pkg::MD_NONE,
         OOO_SRC_REG, OOO_SRC_IMM, 32'd11);
    alu_idx = rob_idx_t'(`ROB.tail_q - 1'b1);
    clear_inputs();

    check_bit("muldiv is older than ALU", mul_idx != alu_idx, 1'b1);

    wait_for_cdb(100);
    check_word("first post-mul CDB is younger ALU", word_t'(dut.cdb_q[0].rob_idx),
               word_t'(alu_idx));
    check_word("younger ALU result", dut.cdb_q[0].result, 32'd11);
    check_bit("older muldiv still busy when younger writes back", dut.u_muldiv.busy, 1'b1);
    check_bit("older muldiv not done yet", `ROB.done_q[mul_idx], 1'b0);
    check_word("commit still blocked behind older muldiv", word_t'(commit_order), 32'd2);

    @(posedge clk);
    #1;
    check_bit("younger ALU accepted before older", `ROB.done_q[alu_idx], 1'b1);
    check_bit("older still not done after younger accept", `ROB.done_q[mul_idx], 1'b0);
    check_word("commit still waits after younger accept", word_t'(commit_order), 32'd2);

    drain_rob(300);

    x3_p = `RN.committed_map_q[3];
    x4_p = `RN.committed_map_q[4];
    check_word("final commit_order==4", word_t'(commit_order), 32'd4);
    check_word("older muldiv result", `PRF.regs_q[x3_p], 32'd6);  // 6 rem 7
    check_word("younger ALU result retained", `PRF.regs_q[x4_p], 32'd11);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_core_m45_natural] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_core_m45_natural] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
