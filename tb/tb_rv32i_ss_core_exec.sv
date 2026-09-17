// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// RAW-chain execute+commit TB for rv32i_ss_core.
//
// Dispatches a RAW chain, lets the IQ issue each uop as its operands become
// ready, and checks architectural state after the ROB drains:
//
//   I0: addi x1, x0, 5     -> x1 = 5
//   I1: addi x2, x1, 3     -> x2 = 8   (reads x1 from PRF -> RAW through PRF)
//   I2: add  x3, x1, x2    -> x3 = 13  (reads x1 and x2)
//
// Proves: ready-table wakeup drives dependent issue, CDB writeback updates the
// PRF and ROB, in-order commit advances the head, committed_map updates, stale
// physregs are freed, and RAW values flow producer->PRF->consumer.

module tb_rv32i_ss_core_exec;
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
    .redirect_target          (redirect_target)
  );

  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;

  initial begin
    repeat (2000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_exec exceeded 2000 cycles");
  end

  `define ROB dut.u_rob
  `define RN  dut.u_rename
  `define FL  dut.u_free_list
  `define PRF dut.u_prf

  function automatic arch_reg_t areg(input int v); areg = arch_reg_t'(v); endfunction
  function automatic phys_reg_t preg(input int v); preg = phys_reg_t'(v); endfunction

  // commit monitor (debug visibility)
  always @(posedge clk) begin
    if (rst_n && `ROB.commit_valid[0]) begin
      $display("[commit %0d] pc=%08h rd=x%0d pdst=p%0d result=%0d",
               `ROB.commit_order_q, `ROB.commit_pc[0], `ROB.commit_rd[0],
               `ROB.commit_pdst[0], `ROB.commit_result[0]);
    end
  end

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

  task automatic check_word(input string name, input word_t got, input word_t exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got=%0d (%08h) exp=%0d (%08h)", name, got, got, exp, exp);
      errors++;
    end
  endtask

  task automatic check_bit(input string name, input logic got, input logic exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got=%0b exp=%0b", name, got, exp);
      errors++;
    end
  endtask

  // Drive one decoded packet; wait until it is accepted (decoded_ready) once.
  task automatic disp(
    input word_t         pc,
    input arch_reg_t     rs1,
    input arch_reg_t     rs2,
    input arch_reg_t     rd,
    input logic          rd_we,
    input alu_op_e       alu_op,
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
    decoded_op_class = OOO_OP_ALU;
    decoded_alu_op   = alu_op;
    decoded_src1_sel = src1_sel;
    decoded_src2_sel = src2_sel;
    decoded_imm      = imm;
    // wait for an accepted cycle
    #1;
    while (decoded_ready !== 1'b1) @(negedge clk);
    @(posedge clk);
    @(negedge clk);
    decoded_valid = 1'b0;
  endtask

  task automatic reset_dut();
    clear_inputs();
    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    @(negedge clk);
  endtask

  int drain;
  phys_reg_t x1_p;
  phys_reg_t x2_p;
  phys_reg_t x3_p;

  initial begin
    $display("[tb_rv32i_ss_core_exec] starting");

    reset_dut();

    // I0: addi x1, x0, 5
    disp(32'h0000_1000, areg(0), areg(0), areg(1), 1'b1,
         fyp_cpu_pkg::ALU_ADD, OOO_SRC_REG, OOO_SRC_IMM, 32'd5);
    // I1: addi x2, x1, 3
    disp(32'h0000_1004, areg(1), areg(0), areg(2), 1'b1,
         fyp_cpu_pkg::ALU_ADD, OOO_SRC_REG, OOO_SRC_IMM, 32'd3);
    // I2: add x3, x1, x2
    disp(32'h0000_1008, areg(1), areg(2), areg(3), 1'b1,
         fyp_cpu_pkg::ALU_ADD, OOO_SRC_REG, OOO_SRC_REG, 32'd0);

    clear_inputs();

    // drain: wait for the ROB to empty
    drain = 0;
    while (`ROB.count_q !== '0) begin
      @(posedge clk);
      drain++;
      if (drain > 200) $fatal(1, "drain stuck: ROB.count=%0d", `ROB.count_q);
    end
    repeat (2) @(posedge clk);

    // ---- architectural-state checks ----
    check_word("commit_order==3", word_t'(`ROB.commit_order_q), 32'd3);
    check_word("rob.count==0",    word_t'({27'b0, `ROB.count_q}), 32'd0);

    x1_p = `RN.committed_map_q[1];
    x2_p = `RN.committed_map_q[2];
    x3_p = `RN.committed_map_q[3];

    // PRF holds the renamed results. The exact x3 physreg may be a reused
    // stale preg because commits while the front end is still dispatching.
    check_word("PRF[cmap[x1]]=5",  `PRF.regs_q[x1_p], 32'd5);
    check_word("PRF[cmap[x2]]=8",  `PRF.regs_q[x2_p], 32'd8);
    check_word("PRF[cmap[x3]]=13", `PRF.regs_q[x3_p], 32'd13);

    // committed map points arch -> live physregs.
    check_word("cmap[x1]=p32", word_t'(x1_p), 32'd32);
    check_word("cmap[x2]=p33", word_t'(x2_p), 32'd33);
    check_bit("x3 live physreg differs from p0", x3_p != '0, 1'b1);
    check_bit("x1/x2 physregs differ", x1_p != x2_p, 1'b1);
    check_bit("x1/x3 physregs differ", x1_p != x3_p, 1'b1);
    check_bit("x2/x3 physregs differ", x2_p != x3_p, 1'b1);

    // free-list: committed live physregs are not free; stale p2/p3 are free.
    check_bit("x1 physreg in use", `FL.free_bits_q[x1_p], 1'b0);
    check_bit("x2 physreg in use", `FL.free_bits_q[x2_p], 1'b0);
    check_bit("x3 physreg in use", `FL.free_bits_q[x3_p], 1'b0);
    check_bit("free p2 (stale)",   `FL.free_bits_q[2], 1'b1);
    check_bit("free p3 (stale)",   `FL.free_bits_q[3], 1'b1);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_core_exec] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_core_exec] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
