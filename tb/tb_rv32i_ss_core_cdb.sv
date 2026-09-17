// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// Registered CDB/writeback TB for rv32i_ss_core.
//
// Proves a stale CDB beat with the right ROB index but wrong rob_seq is not
// allowed to complete the ROB entry or write the PRF. The per-FU issue path may
// still accept a real IQ entry while that stale beat is rejected; the writeback
// identity check is the invariant under test.

module tb_rv32i_ss_core_cdb;
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
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_cdb exceeded 1000 cycles");
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

  task automatic disp_addi_x1();
    @(negedge clk);
    decoded_valid    = 1'b1;
    decoded_pc       = 32'h0000_1000;
    decoded_instr    = 32'h0050_0093; // addi x1, x0, 5
    decoded_rs1      = areg(0);
    decoded_rs2      = areg(0);
    decoded_rd       = areg(1);
    decoded_rd_we    = 1'b1;
    decoded_op_class = OOO_OP_ALU;
    decoded_alu_op   = fyp_cpu_pkg::ALU_ADD;
    decoded_src1_sel = OOO_SRC_REG;
    decoded_src2_sel = OOO_SRC_IMM;
    decoded_imm      = 32'd5;

    #1;
    while (decoded_ready !== 1'b1) @(negedge clk);
    @(posedge clk);
    @(negedge clk);
    decoded_valid = 1'b0;
  endtask

  task automatic disp_add_x7_from_x5();
    @(negedge clk);
    decoded_valid    = 1'b1;
    decoded_pc       = 32'h0000_2000;
    decoded_instr    = 32'h0002_83b3; // add x7, x5, x0
    decoded_rs1      = areg(5);
    decoded_rs2      = areg(0);
    decoded_rd       = areg(7);
    decoded_rd_we    = 1'b1;
    decoded_op_class = OOO_OP_ALU;
    decoded_alu_op   = fyp_cpu_pkg::ALU_ADD;
    decoded_src1_sel = OOO_SRC_REG;
    decoded_src2_sel = OOO_SRC_REG;
    decoded_imm      = '0;

    #1;
    while (decoded_ready !== 1'b1) @(negedge clk);
    @(posedge clk);
    @(negedge clk);
    decoded_valid = 1'b0;
  endtask

  rob_seq_t bad_seq;
  rob_idx_t old_idx;
  rob_seq_t old_seq;
  rob_idx_t new_idx;
  phys_reg_t new_pdst;
  word_t new_pdst_before_stale;
  phys_reg_t x1_p;
  int drain;
  int wrap_i;

  // Module-scope staging for whole-struct force of cdb_q[0]. Icarus silently
  // ignores force on members of packed struct-array elements; do not force
  // individual fields of dut.cdb_q[0], and do not put automatic-task args on
  // the RHS of force.
  completion_packet_t f_cdb0;

  initial begin
    $display("[tb_rv32i_ss_core_cdb] starting");

    reset_dut();
    disp_addi_x1();

    x1_p = `RN.spec_map_q[1];
    check_word("x1 renamed to p32", word_t'(x1_p), 32'd32);
    check_bit("CDB idle before stale injection", dut.cdb_q[0].valid, 1'b0);
    check_bit("CDB lane1 idle with one completion", dut.cdb_q[1].valid, 1'b0);
    check_bit("ALU completion idle before stale injection", dut.alu0_complete.valid, 1'b0);

    bad_seq = `ROB.rob_head_seq + 64'd1;
    f_cdb0 = '0;
    f_cdb0.valid   = 1'b1;
    f_cdb0.rob_idx = `ROB.rob_head_idx;
    f_cdb0.rob_seq = bad_seq;
    f_cdb0.pdst    = x1_p;
    f_cdb0.rd_wen  = 1'b1;
    f_cdb0.result  = 32'hdead_beef;
    force dut.cdb_q[0] = f_cdb0;
    @(posedge clk);
    #1;
    release dut.cdb_q[0];

    check_bit("stale CDB rejected: head not done", `ROB.rob_head_done, 1'b0);
    check_bit("stale CDB not accepted by ROB", dut.rob_wb_accept[0], 1'b0);
    check_word("stale CDB did not write PRF", `PRF.regs_q[x1_p], 32'd0);
    check_word("no commit after stale CDB", word_t'(commit_order), 32'd0);

    drain = 0;
    while (`ROB.count_q !== '0) begin
      @(posedge clk);
      drain++;
      if (drain > 100) $fatal(1, "drain stuck: ROB.count=%0d", `ROB.count_q);
    end
    repeat (2) @(posedge clk);

    x1_p = `RN.committed_map_q[1];
    check_word("commit_order==1", word_t'(commit_order), 32'd1);
    check_word("committed x1 physreg p32", word_t'(x1_p), 32'd32);
    check_word("correct CDB eventually writes PRF", `PRF.regs_q[x1_p], 32'd5);

    // ---- Actual reuse hazard: old ROB0 completes and retires, the tail wraps
    // around, ROB0 is reused by a new not-ready uop, and a late stale CDB beat
    // from the old ROB0 occupant must not complete or write the new occupant.
    reset_dut();
    disp_addi_x1();
    old_idx = `ROB.rob_head_idx;
    old_seq = `ROB.rob_head_seq;

    drain = 0;
    while (`ROB.count_q !== '0) begin
      @(posedge clk);
      drain++;
      if (drain > 100) $fatal(1, "old ROB0 drain stuck: ROB.count=%0d", `ROB.count_q);
    end

    for (wrap_i = 0; wrap_i < OOO_ROB_DEPTH - 1; wrap_i++) begin
      disp_addi_x1();
    end

    drain = 0;
    while (`ROB.count_q !== '0) begin
      @(posedge clk);
      drain++;
      if (drain > 500) $fatal(1, "wrap drain stuck: ROB.count=%0d", `ROB.count_q);
    end

    force `PRF.ready_q[5] = 1'b0;
    disp_add_x7_from_x5();
    new_idx  = rob_idx_t'(`ROB.tail_q - 1'b1);
    new_pdst = `RN.spec_map_q[7];
    new_pdst_before_stale = `PRF.regs_q[new_pdst];

    check_word("ROB entry reused at index 0", word_t'(new_idx), word_t'(old_idx));
    check_bit("new reused entry starts not done", `ROB.done_q[new_idx], 1'b0);

    f_cdb0 = '0;
    f_cdb0.valid   = 1'b1;
    f_cdb0.rob_idx = old_idx;
    f_cdb0.rob_seq = old_seq;
    f_cdb0.pdst    = new_pdst;
    f_cdb0.rd_wen  = 1'b1;
    f_cdb0.result  = 32'hcafe_f00d;
    force dut.cdb_q[0] = f_cdb0;
    @(posedge clk);
    #1;
    release dut.cdb_q[0];

    check_bit("reused entry rejects old seq", `ROB.done_q[new_idx], 1'b0);
    check_word("reused entry stale beat did not write PRF",
               `PRF.regs_q[new_pdst], new_pdst_before_stale);

    force `PRF.ready_q[5] = 1'b1;

    drain = 0;
    while (`ROB.count_q !== '0) begin
      @(posedge clk);
      drain++;
      if (drain > 100) $fatal(1, "reused entry drain stuck: ROB.count=%0d", `ROB.count_q);
    end
    repeat (2) @(posedge clk);

    check_word("reused entry eventually commits", word_t'(commit_order), 32'd33);
    check_word("reused entry correct PRF result", `PRF.regs_q[new_pdst], 32'd0);
    release `PRF.ready_q[5];

    if (errors == 0) begin
      $display("[tb_rv32i_ss_core_cdb] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_core_cdb] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
