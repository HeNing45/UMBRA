// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// Standalone unit TB for rv32i_ss_decode.
// Sweeps a representative instruction set and checks the OoO-facing decode
// fields. Combinational DUT, so each case just drives instr and samples.

module tb_rv32i_ss_decode;
  import fyp_cpu_pkg::*;
  import rv32i_pipeline_pkg::*;
  import rv32i_ss_pkg::arch_reg_t;
  import rv32i_ss_pkg::ooo_op_class_e;
  import rv32i_ss_pkg::ooo_fu_class_e;
  import rv32i_ss_pkg::ooo_src_sel_e;
  import rv32i_ss_pkg::OOO_OP_ALU;
  import rv32i_ss_pkg::OOO_OP_BRANCH;
  import rv32i_ss_pkg::OOO_OP_JUMP;
  import rv32i_ss_pkg::OOO_FU_ALU;
  import rv32i_ss_pkg::OOO_FU_MULDIV;
  import rv32i_ss_pkg::OOO_SRC_REG;
  import rv32i_ss_pkg::OOO_SRC_IMM;
  import rv32i_ss_pkg::OOO_SRC_PC;
  import rv32i_ss_pkg::OOO_SRC_ZERO;
  import rv32i_ss_pkg::csr_addr_t;
  import rv32i_ss_pkg::csr_zimm_t;

  logic [31:0]   instr;
  arch_reg_t     rs1, rs2, rd;
  logic          rd_we, illegal;
  ooo_op_class_e op_class;
  ooo_fu_class_e fu_class;
  alu_op_e       alu_op;
  muldiv_op_e    muldiv_op;
  br_type_e      branch_op;
  ooo_src_sel_e  src1_sel, src2_sel;
  imm_sel_e      imm_sel;
  logic          is_csr;
  csr_op_e       csr_op;
  csr_addr_t     csr_addr;
  csr_zimm_t     csr_zimm;
  trap_op_e      trap_op;
  logic          is_mret;
  logic          is_mem;
  logic          is_load;
  logic          is_store;
  mem_size_e     mem_size;
  logic          mem_unsigned;

  rv32i_ss_decode dut (.*);

  int errors = 0;
  int checks = 0;
  task automatic chk(input string name, input int got, input int exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got=%0d exp=%0d", name, got, exp);
      errors++;
    end
  endtask

  initial begin
    // ---- add x3,x1,x2 ----
    instr = 32'h002081b3; #1;
    chk("add op_class", int'(op_class), int'(OOO_OP_ALU));
    chk("add fu",       int'(fu_class), int'(OOO_FU_ALU));
    chk("add alu_op",   int'(alu_op),   int'(ALU_ADD));
    chk("add s1",       int'(src1_sel), int'(OOO_SRC_REG));
    chk("add s2",       int'(src2_sel), int'(OOO_SRC_REG));
    chk("add rd_we",    rd_we, 1);  chk("add illegal", illegal, 0);
    chk("add is_mem",   is_mem, 0);
    chk("add trap",     int'(trap_op), int'(TRAP_NONE));
    chk("add rs1", int'(rs1), 1); chk("add rs2", int'(rs2), 2); chk("add rd", int'(rd), 3);

    // ---- addi x1,x0,5 ----
    instr = 32'h00500093; #1;
    chk("addi op_class", int'(op_class), int'(OOO_OP_ALU));
    chk("addi alu_op",   int'(alu_op),   int'(ALU_ADD));
    chk("addi s2",       int'(src2_sel), int'(OOO_SRC_IMM));
    chk("addi imm_sel",  int'(imm_sel),  int'(IMM_I));
    chk("addi rd_we",    rd_we, 1);

    // ---- lw x5,0(x1) ----
    instr = 32'h0000a283; #1;
    chk("lw op_class", int'(op_class), int'(OOO_OP_ALU));
    chk("lw s1",       int'(src1_sel), int'(OOO_SRC_REG));
    chk("lw s2",       int'(src2_sel), int'(OOO_SRC_IMM));
    chk("lw imm_sel",  int'(imm_sel),  int'(IMM_I));
    chk("lw rd_we",    rd_we, 1);  chk("lw illegal", illegal, 0);
    chk("lw is_mem",   is_mem, 1);
    chk("lw is_load",  is_load, 1);
    chk("lw is_store", is_store, 0);
    chk("lw mem_size", int'(mem_size), int'(MEM_W));
    chk("lw unsigned", mem_unsigned, 0);

    // ---- sw x2,0(x1) ----
    instr = 32'h0020a023; #1;
    chk("sw op_class", int'(op_class), int'(OOO_OP_ALU));
    chk("sw s2",       int'(src2_sel), int'(OOO_SRC_IMM));
    chk("sw imm_sel",  int'(imm_sel),  int'(IMM_S));
    chk("sw rd_we",    rd_we, 0);
    chk("sw is_mem",   is_mem, 1);
    chk("sw is_load",  is_load, 0);
    chk("sw is_store", is_store, 1);
    chk("sw mem_size", int'(mem_size), int'(MEM_W));
    chk("sw unsigned", mem_unsigned, 0);

    // ---- beq x1,x2,8 ----
    instr = 32'h00208463; #1;
    chk("beq op_class", int'(op_class),  int'(OOO_OP_BRANCH));
    chk("beq branch",   int'(branch_op), int'(BR_BEQ));
    chk("beq s1",       int'(src1_sel),  int'(OOO_SRC_REG));
    chk("beq s2",       int'(src2_sel),  int'(OOO_SRC_REG));
    chk("beq rd_we",    rd_we, 0);  chk("beq illegal", illegal, 0);

    // ---- jal x1,0 ----
    instr = 32'h000000ef; #1;
    chk("jal op_class", int'(op_class), int'(OOO_OP_JUMP));
    chk("jal s1",       int'(src1_sel), int'(OOO_SRC_PC));
    chk("jal s2",       int'(src2_sel), int'(OOO_SRC_IMM));
    chk("jal imm_sel",  int'(imm_sel),  int'(IMM_J));
    chk("jal rd_we",    rd_we, 1);

    // ---- jalr x1,0(x2) ----
    instr = 32'h000100e7; #1;
    chk("jalr op_class", int'(op_class), int'(OOO_OP_JUMP));
    chk("jalr s1",       int'(src1_sel), int'(OOO_SRC_REG));
    chk("jalr s2",       int'(src2_sel), int'(OOO_SRC_IMM));

    // ---- lui x1,0x12345 ----
    instr = 32'h123450b7; #1;
    chk("lui op_class", int'(op_class), int'(OOO_OP_ALU));
    chk("lui alu_op",   int'(alu_op),   int'(ALU_COPY_B));
    chk("lui s1",       int'(src1_sel), int'(OOO_SRC_ZERO));
    chk("lui imm_sel",  int'(imm_sel),  int'(IMM_U));

    // ---- auipc x1,1 ----
    instr = 32'h00001097; #1;
    chk("auipc op_class", int'(op_class), int'(OOO_OP_ALU));
    chk("auipc s1",       int'(src1_sel), int'(OOO_SRC_PC));
    chk("auipc imm_sel",  int'(imm_sel),  int'(IMM_U));

    // ---- mul x3,x1,x2 ----
    instr = 32'h022081b3; #1;
    chk("mul op_class", int'(op_class),  int'(OOO_OP_ALU));
    chk("mul fu",       int'(fu_class),  int'(OOO_FU_MULDIV));
    chk("mul md_op",    int'(muldiv_op), int'(MD_MUL));
    chk("mul rd_we",    rd_we, 1);  chk("mul illegal", illegal, 0);

    // ---- ecall ----
    instr = 32'h00000073; #1;
    chk("ecall trap",    int'(trap_op), int'(TRAP_ECALL));
    chk("ecall illegal", illegal, 0);  chk("ecall rd_we", rd_we, 0);
    chk("ecall is_mret", is_mret, 0);  chk("ecall is_csr", is_csr, 0);

    // ---- ebreak ----
    instr = 32'h00100073; #1;
    chk("ebreak trap", int'(trap_op), int'(TRAP_EBREAK));
    chk("ebreak illegal", illegal, 0);

    // ---- mret ----
    instr = 32'h30200073; #1;
    chk("mret trap",    int'(trap_op), int'(TRAP_MRET));
    chk("mret is_mret", is_mret, 1);
    chk("mret illegal", illegal, 0);

    // ---- csrrw x1,mstatus,x2 ----
    instr = 32'h30011073; #1;
    chk("csrrw is_csr",  is_csr, 1);
    chk("csrrw csr_op",  int'(csr_op), int'(CSR_RW));
    chk("csrrw rd_we",   rd_we, 1);  chk("csrrw illegal", illegal, 0);

    // ---- illegal (all-zero) ----
    instr = 32'h00000000; #1;
    chk("zero illegal", illegal, 1);
    chk("zero trap",    int'(trap_op), int'(TRAP_ILLEGAL));
    chk("zero rd_we",   rd_we, 0);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_decode] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_decode] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
