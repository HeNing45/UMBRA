// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// directed checkpoint lifecycle test.
//
// Drives the core decoded-packet interface directly so the test can prove the
// checkpoint create/free path independently of the predictor:
//   branch dispatch -> rename creates checkpoint id 0
//   branch issue/resolve -> rename frees checkpoint id 0
//   second branch reuses checkpoint id 0

module tb_rv32i_ss_core_checkpoint;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;

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
  always #5 clk = ~clk;

  initial begin
    repeat (1000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_checkpoint exceeded 1000 cycles");
  end

  task automatic clear_inputs();
    decoded_valid            = 1'b0;
    decoded_pc               = '0;
    decoded_instr            = 32'h00000063;  // beq x0,x0,0 encoding shape
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
    decoded_src1_sel         = OOO_SRC_ZERO;
    decoded_src2_sel         = OOO_SRC_ZERO;
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

  task automatic check_ckpt(input string name, input ckpt_idx_t got, input ckpt_idx_t exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got=%0d exp=%0d", name, got, exp);
      errors++;
    end
  endtask

  task automatic check_mask(input string name, input branch_mask_t got, input branch_mask_t exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got=%b exp=%b", name, got, exp);
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

  task automatic drive_checkpointed_branch(input string name, input word_t pc);
    @(negedge clk);
    decoded_valid            = 1'b1;
    decoded_pc               = pc;
    decoded_instr            = 32'h00000063;
    decoded_needs_checkpoint = 1'b1;
    decoded_op_class         = OOO_OP_BRANCH;
    decoded_fu_class         = OOO_FU_ALU;
    decoded_alu_op           = fyp_cpu_pkg::ALU_ADD;
    decoded_branch_op        = rv32i_pipeline_pkg::BR_BNE;
    decoded_src1_sel         = OOO_SRC_ZERO;
    decoded_src2_sel         = OOO_SRC_ZERO;
    decoded_imm              = 32'd4;
    #1;

    check_bit({name, " decoded_ready"}, decoded_ready, 1'b1);
    check_bit({name, " checkpoint_valid comb"}, dut.rename_checkpoint_valid, 1'b1);
    check_ckpt({name, " checkpoint_id comb"}, dut.rename_checkpoint_id, ckpt_idx_t'(0));
    check_mask({name, " branch mask before create"}, dut.rename_branch_mask, '0);

    @(posedge clk);
    @(negedge clk);
    decoded_valid = 1'b0;
    #1;

    check_bit({name, " checkpoint valid after dispatch"}, dut.u_rename.checkpoint_valid_q[0], 1'b1);
    check_bit({name, " IQ has issue candidate"}, dut.iq_select_valid[0], 1'b1);
    check_ckpt({name, " packet checkpoint id"}, dut.issue_entry[0].checkpoint_id, ckpt_idx_t'(0));
    check_mask({name, " packet branch mask"}, dut.issue_entry[0].branch_mask, '0);
  endtask

  task automatic wait_for_release_and_free(input string name);
    bit seen_release;
    seen_release = 1'b0;
    #1;
    if (|dut.checkpoint_release_mask) begin
      seen_release = 1'b1;
      check_mask({name, " release mask"}, dut.checkpoint_release_mask,
                 branch_mask_t'(1'b1));
      @(posedge clk);
      #1;
      check_bit({name, " checkpoint freed"}, dut.u_rename.checkpoint_valid_q[0], 1'b0);
      return;
    end
    for (int cyc = 0; cyc < 30; cyc++) begin
      @(negedge clk);
      if (|dut.checkpoint_release_mask) begin
        seen_release = 1'b1;
        check_mask({name, " release mask"}, dut.checkpoint_release_mask,
                   branch_mask_t'(1'b1));
        @(posedge clk);
        #1;
        check_bit({name, " checkpoint freed"}, dut.u_rename.checkpoint_valid_q[0], 1'b0);
        return;
      end
    end
    checks++;
    $error("[%s] checkpoint_release_mask was not observed", name);
    errors++;
  endtask

  initial begin
    $display("[tb_rv32i_ss_core_checkpoint] starting");
    reset_dut();

    check_bit("reset checkpoint 0 invalid", dut.u_rename.checkpoint_valid_q[0], 1'b0);

    drive_checkpointed_branch("branch A", 32'h0000_0100);
    wait_for_release_and_free("branch A");

    drive_checkpointed_branch("branch B reuses checkpoint row", 32'h0000_0200);
    wait_for_release_and_free("branch B reuses checkpoint row");

    if (errors == 0) begin
      $display("[tb_rv32i_ss_core_checkpoint] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_core_checkpoint] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
