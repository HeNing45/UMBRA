// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// Occupancy-only FU admission (diagnostic, not a production schedule).
//
// Reachability delta:
//   Newly impossible: holder.valid && cdb_grant && issue into that same
//     holder in one cycle.
//   Newly required: IQ sees fu_ready=0 while that holder is still valid,
//     including the drain cycle. The waiting uop may issue on the following
//     cycle after valid clears.
//
// Same-cycle consumers at this seam: issue_fire / per-FU fires,
// early_set, issue-qualified branch resolve, AGEN/dmem commands, CDB accept
// (unchanged). Recovery is already registered.
//
// Consequential coverage: park four independent ADDIs, wake them together so both ALU
// holders fill, then observe the drain cycle. A full holder is not free
// capacity, so the two remaining ready ALU uops must not issue while the
// holders are still valid — even if CDB grants this cycle. Restoring grant
// in fu_ready makes those waiting uops issue on the drain cycle (wrong
// extra issue). Committed x5..x8 are the architectural record of the four
// results.

module tb_rv32i_ss_occupancy_cut;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;

  localparam time CLK_PERIOD = 10ns;

  logic clk;
  logic rst_n;

  logic           decoded_valid;
  logic [1:0]     decoded_slot_valid;
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
    .decoded_slot_valid       (decoded_slot_valid),
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
    .decoded_pred_taken('0),
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
    repeat (2000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_occupancy_cut exceeded 2000 cycles");
  end

  `define ROB dut.u_rob
  `define RN  dut.u_rename
  `define PRF dut.u_prf
  `define IQ  dut.u_iq

  function automatic arch_reg_t areg(input int v);
    areg = arch_reg_t'(v);
  endfunction

  function automatic int iq_valid_count();
    int n;
    int i;
    n = 0;
    for (i = 0; i < OOO_IQ_DEPTH; i++) begin
      if (`IQ.valid_q[i]) n++;
    end
    iq_valid_count = n;
  endfunction

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

  task automatic clear_inputs();
    decoded_valid            = 1'b0;
    decoded_slot_valid       = 2'b00;
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
    decoded_muldiv_op        = rv32i_pipeline_pkg::MD_NONE;
    decoded_trap             = '0;
    decoded_is_load          = 1'b0;
    decoded_is_store         = 1'b0;
    decoded_mem_size         = fyp_cpu_pkg::MEM_W;
    decoded_mem_unsigned     = 1'b0;
  endtask

  task automatic reset_dut();
    clear_inputs();
    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);
  endtask

  task automatic disp(input word_t pc, input arch_reg_t rd, input word_t imm);
    @(negedge clk);
    decoded_valid               = 1'b1;
    decoded_slot_valid          = 2'b01;
    decoded_pc[0]               = pc;
    decoded_instr[0]            = pc;
    decoded_rs1[0]              = areg(15);
    decoded_rs2[0]              = areg(0);
    decoded_rd[0]               = rd;
    decoded_rd_we[0]            = 1'b1;
    decoded_needs_checkpoint[0] = 1'b0;
    decoded_op_class[0]         = OOO_OP_ALU;
    decoded_alu_op[0]           = fyp_cpu_pkg::ALU_ADD;
    decoded_branch_op[0]        = rv32i_pipeline_pkg::BR_NONE;
    decoded_src1_sel[0]         = OOO_SRC_REG;
    decoded_src2_sel[0]         = OOO_SRC_IMM;
    decoded_imm[0]              = imm;
    decoded_fu_class[0]         = OOO_FU_ALU;
    #1;
    while (decoded_ready !== 1'b1) @(negedge clk);
    @(posedge clk);
    @(negedge clk);
    decoded_valid      = 1'b0;
    decoded_slot_valid = 2'b00;
  endtask

  bit saw_dual_fill;
  bit saw_alu0_drain_idle;
  bit saw_alu1_drain_idle;
  bit saw_waiter_on_drain;
  bit refill_on_grant;
  bit ready_while_valid;
  int wait_i;
  phys_reg_t x5_p, x6_p, x7_p, x8_p;

  // Sample after the edge so holder flops and combinational grant/issue
  // have settled. A full holder is not free capacity.
  always @(posedge clk) begin
    if (rst_n) begin
      #1;
      if (dut.alu0_complete.valid && dut.alu1_complete.valid) begin
        saw_dual_fill = 1'b1;
      end
      if (dut.alu0_fu_ready !== !(dut.alu0_in_q[0].valid && dut.alu0_in_q[1].valid))
        ready_while_valid = 1'b1;
      if (dut.alu1_fu_ready !== !(dut.alu1_in_q[0].valid && dut.alu1_in_q[1].valid))
        ready_while_valid = 1'b1;
      if (dut.muldiv_fu_ready !== !dut.md_in_q.valid)
        ready_while_valid = 1'b1;
      if (dut.lsu_fu_ready !== !(dut.agen_in_q[0].valid && dut.agen_in_q[1].valid))
        ready_while_valid = 1'b1;
      if (dut.cdb_grant_alu0 && dut.alu0_issue_fire &&
          dut.alu0_in_q[0].valid && dut.alu0_in_q[1].valid)
        refill_on_grant = 1'b1;
      if ((dut.alu0_in_q[0].valid || dut.alu0_in_q[1].valid) &&
          !dut.alu0_exec_fire)
        saw_alu0_drain_idle = 1'b1;
      if ((dut.alu1_in_q[0].valid || dut.alu1_in_q[1].valid) &&
          !dut.alu1_exec_fire)
        saw_alu1_drain_idle = 1'b1;
      if ((iq_valid_count() >= 1) &&
          (dut.alu0_in_q[0].valid || dut.alu1_in_q[0].valid))
        saw_waiter_on_drain = 1'b1;
    end
  end

  initial begin
    $display("[tb_rv32i_ss_occupancy_cut] starting");

    reset_dut();
    saw_dual_fill = 1'b0;
    saw_alu0_drain_idle = 1'b0;
    saw_alu1_drain_idle = 1'b0;
    saw_waiter_on_drain = 1'b0;
    refill_on_grant = 1'b0;
    ready_while_valid = 1'b0;

    force `PRF.ready_q[15] = 1'b0;
    disp(32'h0000_5000, areg(5), 32'd10);
    disp(32'h0000_5004, areg(6), 32'd20);
    disp(32'h0000_5008, areg(7), 32'd30);
    disp(32'h0000_500c, areg(8), 32'd40);
    disp(32'h0000_5010, areg(9), 32'd50);
    disp(32'h0000_5014, areg(10), 32'd60);
    check_word("parked six ALU uops in IQ", word_t'(iq_valid_count()), 32'd6);

    @(negedge clk);
    release `PRF.ready_q[15];
    force `PRF.ready_q[15] = 1'b1;
    for (wait_i = 0; wait_i < 100 && `ROB.count_q != 0; wait_i++) @(posedge clk);
    repeat (2) @(posedge clk);
    release `PRF.ready_q[15];

    check_bit("both ALU holders filled together", saw_dual_fill, 1'b1);
    check_bit("grant did not admit into a full input", refill_on_grant, 1'b0);
    check_bit("fu_ready is input occupancy, not CDB grant", ready_while_valid, 1'b0);

    check_word("commit_order==6", word_t'(commit_order), 32'd6);
    x5_p = `RN.committed_map_q[5];
    x6_p = `RN.committed_map_q[6];
    x7_p = `RN.committed_map_q[7];
    x8_p = `RN.committed_map_q[8];
    check_word("x5==10", `PRF.regs_q[x5_p], 32'd10);
    check_word("x6==20", `PRF.regs_q[x6_p], 32'd20);
    check_word("x7==30", `PRF.regs_q[x7_p], 32'd30);
    check_word("x8==40", `PRF.regs_q[x8_p], 32'd40);
    check_word("x9==50", `PRF.regs_q[`RN.committed_map_q[9]], 32'd50);
    check_word("x10==60", `PRF.regs_q[`RN.committed_map_q[10]], 32'd60);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_occupancy_cut] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_occupancy_cut] FAIL errors=%0d checks=%0d",
             errors, checks);
    end
  end

endmodule
