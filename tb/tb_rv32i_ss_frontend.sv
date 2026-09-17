// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// Formation unit battery: the
// classifier truth table, entry beats, PC stepping, hold, redirect, and
// refetch-no-skip/no-double — driven directly at the frontend boundary.
module tb_rv32i_ss_frontend;
  import rv32i_ss_pkg::*;

  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  word_t imem_addr;
  word_t [1:0] imem_rdata;
  logic imem_req_valid, imem_req_ready;
  word_t imem_req_addr;
  logic imem_resp_valid, imem_resp_ready;
  word_t [1:0] imem_resp_data;
  logic redirect_valid = 0;
  word_t redirect_target = '0;
  logic decoded_valid, decoded_ready;
  logic [1:0] decoded_slot_valid;
  word_t [1:0] decoded_pc, decoded_instr, decoded_imm;
  logic [1:0] decoded_pred_taken;
  word_t decoded_pred_target;
  arch_reg_t [1:0] d_rs1, d_rs2, d_rd;
  logic [1:0] d_we, d_ck, d_ld, d_st, d_mu;
  ooo_op_class_e [1:0] d_op; ooo_fu_class_e [1:0] d_fu;
  fyp_cpu_pkg::alu_op_e [1:0] d_alu;
  rv32i_pipeline_pkg::muldiv_op_e [1:0] d_md;
  rv32i_pipeline_pkg::br_type_e [1:0] d_br;
  ooo_src_sel_e [1:0] d_s1, d_s2;
  fyp_cpu_pkg::mem_size_e [1:0] d_ms;
  decoded_trap_t d_trap;
  rv32i_pipeline_pkg::csr_op_e d_csrop;
  csr_addr_t d_csra; csr_zimm_t d_csrz;

  rv32i_ss_frontend #(.RESET_PC(32'h0)) dut (
    .clk(clk), .rst_n(rst_n),
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr),
    .imem_resp_valid(imem_resp_valid), .imem_resp_ready(imem_resp_ready),
    .imem_resp_data(imem_resp_data),
    .bp_update_valid(1'b0),  // tie-off: predictor never trains (inert)
    .bp_update_pc('0), .bp_update_taken(1'b0), .bp_update_target('0),
    .bp_return_update_valid(1'b0), .bp_return_update_pc('0),
    .ras_fetch_valid(1'b0), .ras_fetch_target('0),
    .redirect_valid(redirect_valid), .redirect_target(redirect_target),
    .decoded_valid(decoded_valid), .decoded_slot_valid(decoded_slot_valid),
    .decoded_ready(decoded_ready),
    .decoded_pc(decoded_pc), .decoded_instr(decoded_instr),
    .decoded_rs1(d_rs1), .decoded_rs2(d_rs2), .decoded_rd(d_rd),
    .decoded_rd_we(d_we), .decoded_needs_checkpoint(d_ck),
    .decoded_op_class(d_op), .decoded_fu_class(d_fu),
    .decoded_muldiv_op(d_md), .decoded_alu_op(d_alu),
    .decoded_branch_op(d_br), .decoded_src1_sel(d_s1), .decoded_src2_sel(d_s2),
    .decoded_imm(decoded_imm), .decoded_trap(d_trap),
    .decoded_csr_op(d_csrop), .decoded_csr_addr(d_csra), .decoded_csr_zimm(d_csrz),
    .decoded_is_load(d_ld), .decoded_is_store(d_st),
    .decoded_mem_size(d_ms), .decoded_mem_unsigned(d_mu),
    .decoded_pred_taken(decoded_pred_taken),
    .decoded_pred_target(decoded_pred_target)
  );

  // Program memory: aligned 64-bit line read.
  word_t mem [0:63];
  assign imem_rdata = {mem[{imem_addr[7:3], 1'b1}], mem[{imem_addr[7:3], 1'b0}]};

  rv32i_ss_imem_zero_latency_adapter u_imem_adapter (
    .imem_req_valid  (imem_req_valid),
    .imem_req_ready  (imem_req_ready),
    .imem_req_addr   (imem_req_addr),
    .imem_resp_valid (imem_resp_valid),
    .imem_resp_ready (imem_resp_ready),
    .imem_resp_data  (imem_resp_data),
    .line_addr       (imem_addr),
    .line_data       (imem_rdata)
  );

  localparam word_t ADDI1 = 32'h00100093;  // addi x1,x0,1
  localparam word_t ADDI2 = 32'h00200113;  // addi x2,x0,2
  localparam word_t LW    = 32'h00002103;  // lw x2,0(x0)
  localparam word_t SW    = 32'h00202023;  // sw x2,0(x0)
  localparam word_t BEQ   = 32'h00000463;  // beq x0,x0,+8
  localparam word_t CSRRW = 32'h30001073;  // csrrw x0,mstatus,x0
  localparam word_t JAL   = 32'h0080006f;  // jal x0,+8
  localparam word_t MRET  = 32'h30200073;
  localparam word_t ECALL = 32'h00000073;

  int checks = 0, errors = 0;
  task automatic chk(input string name, input logic cond);
    checks++;
    if (!cond) begin errors++; $display("  FAIL [%0d] %s", checks, name); end
  endtask

  initial begin
    repeat (1500) @(posedge clk);
    $fatal(1, "WATCHDOG tb_rv32i_ss_frontend");
  end

  // one accepted step: verify the offer then fire it
  task automatic step(input string name, input word_t exp_pc,
                      input logic [1:0] exp_shape, input logic [1:0] exp_ck);
    while (!decoded_valid) @(negedge clk);
    @(negedge clk); #1;
    chk({name, ": pc"},    decoded_pc[0] == exp_pc);
    chk({name, ": shape"}, decoded_slot_valid == exp_shape);
    chk({name, ": ck"},    d_ck == exp_ck);
    decoded_ready = 1'b1;
    @(posedge clk); @(negedge clk);
    decoded_ready = 1'b0;
  endtask

  initial begin
    decoded_ready = 1'b0;
    for (int i = 0; i < 64; i++) mem[i] = 32'h0000_0013;
    mem[0]  = ADDI1; mem[1]  = ADDI2;   // 0x00: 11
    mem[2]  = LW;    mem[3]  = ADDI1;   // 0x08: 11 (one mem)
    mem[4]  = LW;    mem[5]  = SW;      // 0x10: 01 (two mem) -> beat
    mem[6]  = BEQ;   mem[7]  = ADDI1;   // 0x18: 11, ck=01
    mem[8]  = ADDI1; mem[9]  = BEQ;     // 0x20: 11, ck=10
    mem[10] = BEQ;   mem[11] = BEQ;     // 0x28: 01 (two br) -> beat
    mem[12] = CSRRW; mem[13] = ADDI1;   // 0x30: 01 (solo0) -> beat
    mem[14] = ADDI1; mem[15] = JAL;     // 0x38: 01 (solo1 jump) -> beat
    mem[16] = ADDI1; mem[17] = MRET;    // 0x40: 01 (solo1 MRET) -> beat
    mem[18] = ECALL; mem[19] = ADDI1;   // 0x48: 01 (solo0 trap) -> beat
    mem[20] = ADDI1; mem[21] = ADDI2;   // 0x50
    rst_n = 0; repeat (3) @(posedge clk); @(negedge clk); rst_n = 1;

    step("T1 dual ALU",        32'h00, 2'b11, 2'b00);
    step("T2 ALU+LW bundle",   32'h08, 2'b11, 2'b00);
    step("T3 two-mem split",   32'h10, 2'b01, 2'b00);
    step("T3b Q1 SW+branch",    32'h14, 2'b11, 2'b10);
    step("T4 stalled upper 01",32'h1C, 2'b01, 2'b00);
    step("T5 br slot1 bundle", 32'h20, 2'b11, 2'b10);
    step("T6 two-br split",    32'h28, 2'b01, 2'b01);
    step("T6b beat",           32'h2C, 2'b01, 2'b01);
    step("T7 csr solo0",       32'h30, 2'b01, 2'b00);
    step("T7b Q1 ALU pair",    32'h34, 2'b11, 2'b00);
    step("T8 JAL solo0",       32'h3C, 2'b01, 2'b00);
    step("T8b MRET slot1 split",32'h40, 2'b01, 2'b00);
    step("T9 MRET solo0",      32'h44, 2'b01, 2'b00);
    step("T9b ECALL solo0",    32'h48, 2'b01, 2'b00);
    step("T10 Q1 ALU pair",    32'h4C, 2'b11, 2'b00);
    step("T10b stalled upper", 32'h54, 2'b01, 2'b00);
    // refetch-no-skip/no-double: the walk above visited every address
    // through 0x54 exactly once, in order — pc now stands at 0x58.
    @(negedge clk); #1;
    chk("T11 refetch walk complete (pc=0x58)", decoded_pc[0] == 32'h58);

    // hold: ready low -> same offer, PC frozen
    repeat (2) @(negedge clk); #1;
    chk("T12 hold keeps pc",    decoded_pc[0] == 32'h58);
    chk("T12 hold keeps shape", decoded_slot_valid == 2'b11);

    // Redirect into an odd word: entry beat, then re-aligned formation.
    @(negedge clk); redirect_valid = 1'b1; redirect_target = 32'h04;
    @(posedge clk); @(negedge clk); redirect_valid = 1'b0; #1;
    while (!decoded_valid) @(negedge clk);
    chk("T13 redirect lands",     decoded_pc[0] == 32'h04);
    chk("T13 entry beat is 01",   decoded_slot_valid == 2'b01);
    step("T13 fire the beat",     32'h04, 2'b01, 2'b00);
    @(negedge clk); #1;
    chk("T14 re-aligned bundle",  decoded_pc[0] == 32'h08 && decoded_slot_valid == 2'b11);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_frontend] PASS checks=%0d", checks);
      $finish;
    end else
      $fatal(1, "[tb_rv32i_ss_frontend] FAIL errors=%0d checks=%0d", errors, checks);
  end
endmodule
