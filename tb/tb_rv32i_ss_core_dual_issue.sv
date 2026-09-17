// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// tb_rv32i_ss_core_dual_issue -- dual-issue live-direction battery.
//
// Dual-issue behaviors, each observed end-to-end at FLOP level
// (cross-module COMB sampling is unreliable under Icarus coarse
// element-select sensitivity; the RTL pins carry the comb-level
// assertions, these observers read registered state only):
// two independent ALU ops DUAL-ISSUE in one cycle — proven by the
//       two ALU completion holders being valid SIMULTANEOUSLY — and both
//       drain through the dual-lane CDB and commit in order.
// the solo rule live — with a jump in play, the two ALU holders
//       never coexist (the IQ solo pins are the cycle-exact checkers).
// TWO CORRECT branches (different bundles) resolve in the SAME
//       cycle -> two-bit release mask -> rename frees BOTH checkpoint
//       rows on one edge through the multi-bit release mask.
// an ALU1 taken-MISALIGNED branch TRAPS and never recovers (the
// consequential direction: cross-unit misalign wiring would
//       reclassify the IADDR trap as a branch recovery).

module tb_rv32i_ss_core_dual_issue;
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
    repeat (2000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_dual_issue exceeded 2000 cycles");
  end

  `define ROB dut.u_rob
  `define RN  dut.u_rename
  `define PRF dut.u_prf

  function automatic arch_reg_t areg(input int v);
    areg = arch_reg_t'(v);
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
      $error("[%s] got=%h exp=%h", name, got, exp);
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
    decoded_muldiv_op        = rv32i_pipeline_pkg::MD_MUL;
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

  // Slot-0 dispatch beat; ops read x15 (forced not-ready) to park together.
  task automatic disp(input word_t pc, input arch_reg_t rs1,
                      input arch_reg_t rd, input logic rd_we,
                      input ooo_op_class_e opc, input br_type_e brop,
                      input word_t imm, input logic ckpt);
    @(negedge clk);
    decoded_valid               = 1'b1;
    decoded_slot_valid          = 2'b01;
    decoded_pc[0]               = pc;
    decoded_rs1[0]              = rs1;
    decoded_rs2[0]              = areg(0);
    decoded_rd[0]               = rd;
    decoded_rd_we[0]            = rd_we;
    decoded_needs_checkpoint[0] = ckpt;
    decoded_op_class[0]         = opc;
    decoded_branch_op[0]        = brop;
    decoded_src1_sel[0]         = OOO_SRC_REG;
    decoded_src2_sel[0]         = (opc == OOO_OP_BRANCH) ? OOO_SRC_REG : OOO_SRC_IMM;
    decoded_imm[0]              = imm;
    decoded_fu_class[0]         = OOO_FU_ALU;
    #1;
    while (decoded_ready !== 1'b1) @(negedge clk);
    @(posedge clk);
    @(negedge clk);
    decoded_valid      = 1'b0;
    decoded_slot_valid = 2'b00;
    decoded_needs_checkpoint[0] = 1'b0;
  endtask

  bit dual_holders_seen;   // ALU0+ALU1 holders valid simultaneously
  bit dual_release_seen;   // both checkpoint rows drop on one edge
  bit trap_seen;           // trap_q_valid rose (flop)
  bit recover_seen;        // branch_recover_req rose (flop)
  logic ck0_prev, ck1_prev;
  int wait_i;

  always @(posedge clk) begin
    if (rst_n) begin
      if (dut.alu0_complete.valid && dut.alu1_complete.valid) begin
        dual_holders_seen <= 1'b1;
      end
      if (dut.trap_q_valid) trap_seen <= 1'b1;
      if (dut.branch_recover_req) recover_seen <= 1'b1;
      ck0_prev <= `RN.checkpoint_valid_q[0];
      ck1_prev <= `RN.checkpoint_valid_q[1];
      if (ck0_prev && ck1_prev &&
          !`RN.checkpoint_valid_q[0] && !`RN.checkpoint_valid_q[1]) begin
        dual_release_seen <= 1'b1;
      end
    end
  end

  initial begin
    $display("[tb_rv32i_ss_core_dual_issue] starting");

    // ============== two independent ALU uops dual-issue ===============
    reset_dut();
    dual_holders_seen = 1'b0;
    force `PRF.ready_q[15] = 1'b0;
    disp(32'h1000, areg(15), areg(5), 1'b1, OOO_OP_ALU,
         rv32i_pipeline_pkg::BR_NONE, 32'd5, 1'b0);
    disp(32'h1004, areg(15), areg(6), 1'b1, OOO_OP_ALU,
         rv32i_pipeline_pkg::BR_NONE, 32'd6, 1'b0);
    @(negedge clk);
    release `PRF.ready_q[15];
    force `PRF.ready_q[15] = 1'b1;
    for (wait_i = 0; wait_i < 100 && `ROB.count_q != 0; wait_i++) @(posedge clk);
    check_bit("D1 dual issue observed (both ALU holders valid)",
              dual_holders_seen, 1'b1);
    check_word("D1 both committed", word_t'(`ROB.commit_order_q), 32'd2);
    release `PRF.ready_q[15];

    // ================= solo rule, live direction =================
    reset_dut();
    dual_holders_seen = 1'b0;
    force `PRF.ready_q[15] = 1'b0;
    // ALU first, then the jump (a parked jump's jump_inflight would block
    // any later dispatch); both park on x15 and wake together. The live IQ
    // solo pins fatal on any grant-level violation; the flop-level
    // consequence checked here is that the two ALU holders never coexist.
    disp(32'h2000, areg(15), areg(7), 1'b1, OOO_OP_ALU,
         rv32i_pipeline_pkg::BR_NONE, 32'd7, 1'b0);
    disp(32'h2004, areg(15), areg(1), 1'b1, OOO_OP_JUMP,
         rv32i_pipeline_pkg::BR_NONE, 32'h100, 1'b0);
    @(negedge clk);
    release `PRF.ready_q[15];
    force `PRF.ready_q[15] = 1'b1;
    for (wait_i = 0; wait_i < 150 && `ROB.count_q != 0; wait_i++) @(posedge clk);
    check_bit("D2 no dual occupancy with a solo in play",
              dual_holders_seen, 1'b0);
    check_word("D2 both committed", word_t'(`ROB.commit_order_q), 32'd2);
    release `PRF.ready_q[15];

    // ============ dual correct branches, two-bit release ============
    reset_dut();
    dual_release_seen = 1'b0;
    ck0_prev = 1'b0; ck1_prev = 1'b0;
    force `PRF.ready_q[15] = 1'b0;
    // Two BNE x15,x0 (x15=0 -> not taken -> CORRECT), different bundles,
    // each with its own checkpoint; they park and wake together.
    disp(32'h3000, areg(15), areg(0), 1'b0, OOO_OP_BRANCH,
         rv32i_pipeline_pkg::BR_BNE, 32'h40, 1'b1);
    disp(32'h3004, areg(15), areg(0), 1'b0, OOO_OP_BRANCH,
         rv32i_pipeline_pkg::BR_BNE, 32'h40, 1'b1);
    #1;
    check_bit("D3 two checkpoints live", `RN.checkpoint_valid_q[0]
              && `RN.checkpoint_valid_q[1], 1'b1);
    @(negedge clk);
    release `PRF.ready_q[15];
    force `PRF.ready_q[15] = 1'b1;
    for (wait_i = 0; wait_i < 100 && `ROB.count_q != 0; wait_i++) @(posedge clk);
    check_bit("D3 dual-resolve two-bit release observed",
              dual_release_seen, 1'b1);
    check_bit("D3 both checkpoint rows freed",
              !`RN.checkpoint_valid_q[0] && !`RN.checkpoint_valid_q[1], 1'b1);
    check_word("D3 both committed", word_t'(`ROB.commit_order_q), 32'd2);
    release `PRF.ready_q[15];

    // ========= ALU1 taken-MISALIGNED branch -> trap, never recovery ====
    reset_dut();
    trap_seen = 1'b0; recover_seen = 1'b0;
    force `PRF.ready_q[15] = 1'b0;
    // Older: BNE x15,x0 -> not taken -> CORRECT (releases).
    disp(32'h4000, areg(15), areg(0), 1'b0, OOO_OP_BRANCH,
         rv32i_pipeline_pkg::BR_BNE, 32'h40, 1'b1);
    // Younger: BEQ x15,x0 -> 0==0 TAKEN, imm 0x42 -> misaligned target ->
    // IADDR trap at commit, NOT a recovery.
    disp(32'h4004, areg(15), areg(0), 1'b0, OOO_OP_BRANCH,
         rv32i_pipeline_pkg::BR_BEQ, 32'h42, 1'b1);
    @(negedge clk);
    release `PRF.ready_q[15];
    force `PRF.ready_q[15] = 1'b1;
    for (wait_i = 0; wait_i < 150 && !trap_seen; wait_i++) @(posedge clk);
    repeat (4) @(posedge clk);
    check_bit("D4 misaligned-taken branch TRAPPED", trap_seen, 1'b1);
    check_bit("D4 no branch recovery ever fired", recover_seen, 1'b0);
    check_word("D4 only the older correct branch committed",
               word_t'(`ROB.commit_order_q), 32'd1);
    check_bit("D4 all checkpoint rows clear after the flush",
              !`RN.checkpoint_valid_q[0] && !`RN.checkpoint_valid_q[1], 1'b1);
    release `PRF.ready_q[15];

    if (errors == 0)
      $display("[tb_rv32i_ss_core_dual_issue] PASS checks=%0d", checks);
    else
      $display("[tb_rv32i_ss_core_dual_issue] FAIL checks=%0d errors=%0d",
               checks, errors);
    $finish;
  end

endmodule
