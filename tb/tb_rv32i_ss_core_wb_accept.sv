// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// tb_rv32i_ss_core_wb_accept -- acceptance-seam battery.
//
// The flow-and-reject contract: ROB done/result, the PRF data write, and
// the ready-table set are all gated by ONE broadcast-aware acceptance
// verdict (wb_accept = match && !trap && survivor-age). This TB forces
// CDB beats coincident with recovery/trap and observes EVERY side effect:
// recovery-coincident VICTIM beat -> rejected everywhere;
//       then the same packet presented late (stale {idx,seq}) -> rejected.
// recovery-coincident SURVIVOR beat -> accepted everywhere.
// trap-coincident beat -> rejected everywhere.
// Staging is force-based (house idiom from the arbiter TB): entries are
// parked un-issued by holding their source not-ready, so every beat and
// broadcast is TB-controlled and cycle-exact.

module tb_rv32i_ss_core_wb_accept;
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
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_wb_accept exceeded 1000 cycles");
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
    // Park every dispatched op un-issued: x1 never becomes ready.
    force `PRF.ready_q[1] = 1'b0;
  endtask

  // Dispatch one slot-0 uop that reads x1 (never ready -> never issues).
  task automatic disp_parked(input arch_reg_t rd, input logic rd_we,
                             input logic is_branch);
    @(negedge clk);
    decoded_valid            = 1'b1;
    decoded_pc[0]            = 32'h0000_1000;
    decoded_rs1[0]           = areg(1);
    decoded_rs2[0]           = areg(0);
    decoded_rd[0]            = rd;
    decoded_rd_we[0]         = rd_we;
    decoded_needs_checkpoint[0] = is_branch;
    decoded_op_class[0]      = is_branch ? OOO_OP_BRANCH : OOO_OP_ALU;
    decoded_branch_op[0]     = is_branch ? rv32i_pipeline_pkg::BR_BEQ
                                         : rv32i_pipeline_pkg::BR_NONE;
    decoded_src1_sel[0]      = OOO_SRC_REG;
    decoded_src2_sel[0]      = OOO_SRC_REG;
    decoded_fu_class[0]      = OOO_FU_ALU;
    #1;
    while (decoded_ready !== 1'b1) @(negedge clk);
    @(posedge clk);
    @(negedge clk);
    decoded_valid = 1'b0;
  endtask

  // Module-scope staging for whole-struct force of cdb_q[0]. Icarus silently
  // ignores force on members of packed struct-array elements; do not force
  // individual fields of dut.cdb_q[0], and do not put automatic-task args on
  // the RHS of force.
  completion_packet_t f_cdb0;

  // Force one CDB beat for a parked ROB entry (held exactly one cycle by
  // the caller's force/release bracket). Static task: Icarus rejects
  // force statements that reference automatically allocated variables.
  // Copy args into module-scope f_cdb0, then force the whole cdb_q[0] packet.
  task force_beat(input rob_idx_t idx, input word_t result);
    f_cdb0 = '0;
    f_cdb0.valid   = 1'b1;
    f_cdb0.rob_idx = idx;
    f_cdb0.rob_seq = `ROB.seq_q[idx];
    f_cdb0.pdst    = `ROB.pdst_q[idx];
    f_cdb0.rd_wen  = 1'b1;
    f_cdb0.result  = result;
    force dut.cdb_q[0] = f_cdb0;
  endtask

  task automatic release_beat();
    release dut.cdb_q[0];
  endtask

  phys_reg_t pd_victim, pd_surv, pd_trap;
  word_t     old_data;
  rob_seq_t  stale_seq;
  phys_reg_t stale_pd;
  commit_order_t co_before;

  initial begin
    $display("[tb_rv32i_ss_core_wb_accept] starting");

    // ================= recovery-coincident VICTIM =================
    reset_dut();
    disp_parked(areg(0), 1'b0, 1'b1);   // entry 0: branch, checkpoint 0
    disp_parked(areg(5), 1'b1, 1'b0);   // entry 1: ALU rd=x5 (the victim)
    #1;
    check_bit("P1 checkpoint 0 live", `RN.checkpoint_valid_q[0], 1'b1);
    pd_victim = `ROB.pdst_q[1];
    old_data  = `PRF.regs_q[pd_victim];
    stale_seq = `ROB.seq_q[1];
    stale_pd  = pd_victim;

    @(negedge clk);
    // Recovery broadcast targeting the branch at entry 0; every younger
    // entry (the victim at 1) dies this cycle. The victim's beat arrives
    // on the same broadcast cycle.
    force dut.recover_q_valid   = 1'b1;
    force dut.recover_q_rob_idx = rob_idx_t'(0);
    force dut.recover_q_ckpt_id = ckpt_idx_t'(0);
    force dut.recover_q_target  = 32'h0000_2000;
    force_beat(rob_idx_t'(1), 32'hDEAD_BEEF);
    #1;
    check_bit("P1 victim beat rejected (wb_accept)", dut.rob_wb_accept[0], 1'b0);
    @(posedge clk);
    @(negedge clk);
    // Retention hazard: a released force on a flop keeps the forced value
    // until the flop's next assignment edge. Drive the valid to 0 for one
    // edge so rename never sees a second recovery over the cleared row.
    force dut.recover_q_valid = 1'b0;
    release dut.recover_q_rob_idx;
    release dut.recover_q_ckpt_id;
    release dut.recover_q_target;
    release_beat();
    @(posedge clk);
    @(negedge clk);
    release dut.recover_q_valid;
    #1;
    check_bit("P1 victim not done",   `ROB.done_q[1], 1'b0);
    check_bit("P1 victim entry dead", `ROB.valid_q[1], 1'b0);
    check_word("P1 PRF data unchanged", `PRF.regs_q[pd_victim], old_data);
    check_bit("P1 ready bit not set", `PRF.ready_q[pd_victim], 1'b0);

    // ---- the same packet presented LATE (stale {idx,seq}) ----
    @(negedge clk);
    f_cdb0 = '0;
    f_cdb0.valid   = 1'b1;
    f_cdb0.rob_idx = rob_idx_t'(1);
    f_cdb0.rob_seq = stale_seq;
    f_cdb0.pdst    = stale_pd;
    f_cdb0.rd_wen  = 1'b1;
    f_cdb0.result  = 32'hDEAD_BEEF;
    force dut.cdb_q[0] = f_cdb0;
    #1;
    check_bit("P1b stale late beat rejected", dut.rob_wb_accept[0], 1'b0);
    @(posedge clk);
    @(negedge clk);
    release_beat();
    #1;
    check_word("P1b PRF still unchanged", `PRF.regs_q[stale_pd], old_data);
    check_bit("P1b ready still clear", `PRF.ready_q[stale_pd], 1'b0);

    // ================= recovery-coincident SURVIVOR =================
    reset_dut();
    disp_parked(areg(6), 1'b1, 1'b0);   // entry 0: ALU rd=x6 (the survivor)
    disp_parked(areg(0), 1'b0, 1'b1);   // entry 1: branch, checkpoint 0
    #1;
    check_bit("P2 checkpoint 0 live", `RN.checkpoint_valid_q[0], 1'b1);
    pd_surv   = `ROB.pdst_q[0];
    co_before = `ROB.commit_order_q;

    @(negedge clk);
    force dut.recover_q_valid   = 1'b1;
    force dut.recover_q_rob_idx = rob_idx_t'(1);
    force dut.recover_q_ckpt_id = ckpt_idx_t'(0);
    force dut.recover_q_target  = 32'h0000_3000;
    force_beat(rob_idx_t'(0), 32'hCAFE_0001);
    #1;
    check_bit("P2 survivor beat accepted (wb_accept)", dut.rob_wb_accept[0], 1'b1);
    @(posedge clk);
    @(negedge clk);
    force dut.recover_q_valid = 1'b0;
    release dut.recover_q_rob_idx;
    release dut.recover_q_ckpt_id;
    release dut.recover_q_target;
    release_beat();
    @(posedge clk);
    @(negedge clk);
    release dut.recover_q_valid;
    #1;
    // The accepted survivor becomes done at the broadcast edge and, as the
    // ROB head, RETIRES on the following cycle — assert the retirement.
    check_bit("P2 survivor retired (entry clear)", `ROB.valid_q[0], 1'b0);
    check_bit("P2 commit advanced",
              `ROB.commit_order_q == (co_before + 64'd1), 1'b1);
    check_word("P2 PRF data written", `PRF.regs_q[pd_surv], 32'hCAFE_0001);
    check_bit("P2 ready bit set",     `PRF.ready_q[pd_surv], 1'b1);

    // ================= trap-coincident beat =================
    reset_dut();
    disp_parked(areg(7), 1'b1, 1'b0);   // entry 0: ALU rd=x7
    #1;
    pd_trap  = `ROB.pdst_q[0];
    old_data = `PRF.regs_q[pd_trap];

    @(negedge clk);
    force dut.trap_q_valid = 1'b1;
    force_beat(rob_idx_t'(0), 32'hBAD0_0003);
    #1;
    check_bit("P3 trap-coincident beat rejected", dut.rob_wb_accept[0], 1'b0);
    @(posedge clk);
    @(negedge clk);
    force dut.trap_q_valid = 1'b0;
    release_beat();
    @(posedge clk);
    @(negedge clk);
    release dut.trap_q_valid;
    #1;
    check_bit("P3 entry flushed",      `ROB.valid_q[0], 1'b0);
    check_bit("P3 not done",           `ROB.done_q[0], 1'b0);
    check_word("P3 PRF data unchanged", `PRF.regs_q[pd_trap], old_data);
    check_bit("P3 ready bit not set",  `PRF.ready_q[pd_trap], 1'b0);

    if (errors == 0)
      $display("[tb_rv32i_ss_core_wb_accept] PASS checks=%0d", checks);
    else
      $display("[tb_rv32i_ss_core_wb_accept] FAIL checks=%0d errors=%0d",
               checks, errors);
    $finish;
  end

endmodule
