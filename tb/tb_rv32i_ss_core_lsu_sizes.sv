// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// tb_rv32i_ss_core_lsu_sizes — load-datapath battery (frontend + core +
// dmem model, program-driven through the REAL decoder).
//
// Every legal aligned size x offset combination, with CONSEQUENTIAL data:
// the word 0xF7B38291 has every byte lane distinct AND bit-7-set, so a byte-lane
// mix-up or a sign/zero-extension bug must flip an observable committed value
// (with benign data, LB and LBU are indistinguishable).
//
//   bytes:  [0]=0x91 [1]=0x82 [2]=0xB3 [3]=0xF7   halves: [0]=0x8291 [2]=0xF7B3
//
// SCOPE BOUNDARY: this battery covers only aligned-for-size extraction — LB at
// any offset, LH at {0,2}, and LW at 0. Precise misalignment traps (cause 4/6,
// mtval = address) are covered end-to-end by tb_rv32i_ss_core_misalign.
//
// Program (RESET_PC = 0), all loads from base x1 = 0x100:
//   0x00 addi x1, x0, 0x100
//   0x04 lb  x5,0(x1)  0x08 lbu x6,0(x1)   -> FFFFFF91 / 00000091
//   0x0c lb  x7,1(x1)  0x10 lbu x8,1(x1)   -> FFFFFF82 / 00000082
//   0x14 lb  x9,2(x1)  0x18 lbu x10,2(x1)  -> FFFFFFB3 / 000000B3
//   0x1c lb x11,3(x1)  0x20 lbu x12,3(x1)  -> FFFFFFF7 / 000000F7
//   0x24 lh x13,0(x1)  0x28 lhu x14,0(x1)  -> FFFF8291 / 00008291
//   0x2c lh x15,2(x1)  0x30 lhu x16,2(x1)  -> FFFFF7B3 / 0000F7B3
//   0x34 lw x17,0(x1)                      -> F7B38291
//   0x38 addi x18, x0, 0x55                (tail marker)
//   0x3c jal  x0, 0                        (spin)

module tb_rv32i_ss_core_lsu_sizes;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;

  logic clk;
  logic rst_n;

  logic          dec_valid, dec_ready;
  logic [1:0] dec_slot_valid;
  word_t [1:0] dec_pc, dec_instr;
  arch_reg_t [1:0] dec_rs1, dec_rs2, dec_rd;
  logic [1:0] dec_rd_we, dec_needs_checkpoint;
  ooo_op_class_e [1:0] dec_op_class;
  ooo_fu_class_e [1:0] dec_fu_class;
  muldiv_op_e [1:0]    dec_muldiv_op;
  alu_op_e [1:0]       dec_alu_op;
  br_type_e [1:0]      dec_branch_op;
  ooo_src_sel_e [1:0] dec_src1_sel, dec_src2_sel;
  word_t [1:0]         dec_imm;
  decoded_trap_t dec_trap;
  csr_op_e       dec_csr_op;
  csr_addr_t     dec_csr_addr;
  csr_zimm_t     dec_csr_zimm;
  logic [1:0] dec_is_load, dec_is_store;
  mem_size_e [1:0]     dec_mem_size;
  logic [1:0]          dec_mem_unsigned;

  logic [1:0]          commit_fire;
  commit_order_t commit_order;
  word_t [1:0]         commit_pc, commit_inst, commit_wdata;
  arch_reg_t [1:0]     commit_rd;
  logic [1:0]          commit_rd_wen;

  logic          redirect_valid;
  word_t         redirect_target;

  logic        dmem_valid, dmem_we;
  logic [3:0]  dmem_be;
  word_t       dmem_addr, dmem_wdata, dmem_rdata;

  word_t       imem_addr;
  word_t [1:0]       imem_rdata;
  logic imem_req_valid, imem_req_ready;
  word_t imem_req_addr;
  logic imem_resp_valid, imem_resp_ready;
  word_t [1:0] imem_resp_data;
  logic [31:0] imem [256];
  assign imem_rdata = !$isunknown(imem_addr[9:3])
      ? {imem[{imem_addr[9:3], 1'b1}], imem[{imem_addr[9:3], 1'b0}]}
      : {32'h0000_0013, 32'h0000_0013};

  rv32i_ss_imem_zero_latency_adapter u_imem_adapter (
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr), .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready), .imem_resp_data(imem_resp_data),
    .line_addr(imem_addr), .line_data(imem_rdata)
  );

  rv32i_ss_frontend #(.RESET_PC(32'h0)) u_fe (
    .clk(clk), .rst_n(rst_n),
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr), .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready), .imem_resp_data(imem_resp_data),
    .bp_update_valid(1'b0),  // tie-off: predictor never trains (inert)
    .bp_update_pc('0), .bp_update_taken(1'b0), .bp_update_target('0),
    .redirect_valid(redirect_valid), .redirect_target(redirect_target),
    .decoded_valid(dec_valid), .decoded_slot_valid(dec_slot_valid), .decoded_ready(dec_ready),
    .decoded_pc(dec_pc), .decoded_instr(dec_instr),
    .decoded_rs1(dec_rs1), .decoded_rs2(dec_rs2), .decoded_rd(dec_rd),
    .decoded_rd_we(dec_rd_we), .decoded_needs_checkpoint(dec_needs_checkpoint),
    .decoded_op_class(dec_op_class), .decoded_fu_class(dec_fu_class),
    .decoded_muldiv_op(dec_muldiv_op), .decoded_alu_op(dec_alu_op),
    .decoded_branch_op(dec_branch_op),
    .decoded_src1_sel(dec_src1_sel), .decoded_src2_sel(dec_src2_sel),
    .decoded_imm(dec_imm),
    .decoded_trap(dec_trap),
    .decoded_csr_op(dec_csr_op),
    .decoded_csr_addr(dec_csr_addr),
    .decoded_csr_zimm(dec_csr_zimm),
    .decoded_is_load(dec_is_load),
    .decoded_is_store(dec_is_store),
    .decoded_mem_size(dec_mem_size),
    .decoded_mem_unsigned(dec_mem_unsigned)
  );

  rv32i_ss_core u_core (
    .clk(clk), .rst_n(rst_n),
    .decoded_valid(dec_valid), .decoded_slot_valid(dec_slot_valid), .decoded_ready(dec_ready),
    .decoded_pc(dec_pc), .decoded_instr(dec_instr),
    .decoded_rs1(dec_rs1), .decoded_rs2(dec_rs2), .decoded_rd(dec_rd),
    .decoded_rd_we(dec_rd_we), .decoded_needs_checkpoint(dec_needs_checkpoint),
    .decoded_op_class(dec_op_class), .decoded_fu_class(dec_fu_class),
    .decoded_muldiv_op(dec_muldiv_op), .decoded_alu_op(dec_alu_op),
    .decoded_branch_op(dec_branch_op),
    .decoded_src1_sel(dec_src1_sel), .decoded_src2_sel(dec_src2_sel),
    .decoded_imm(dec_imm),
    .decoded_trap(dec_trap),
    .decoded_csr_op(dec_csr_op),
    .decoded_csr_addr(dec_csr_addr),
    .decoded_csr_zimm(dec_csr_zimm),
    .decoded_is_load(dec_is_load),
    .decoded_is_store(dec_is_store),
    .decoded_mem_size(dec_mem_size),
    .decoded_mem_unsigned(dec_mem_unsigned),
    .decoded_pred_taken('0),  // tie-off: static not-taken prediction
    .decoded_pred_target('0),
    .redirect_valid(redirect_valid), .redirect_target(redirect_target),
    .commit_fire(commit_fire), .commit_order(commit_order),
    .commit_pc(commit_pc), .commit_inst(commit_inst),
    .commit_rd(commit_rd), .commit_rd_wen(commit_rd_wen),
    .commit_wdata(commit_wdata),
    .dmem_valid(dmem_valid), .dmem_we(dmem_we), .dmem_be(dmem_be),
    .dmem_addr(dmem_addr), .dmem_wdata(dmem_wdata),
    .dmem_ready(1'b1), .dmem_rvalid(1'b1),
    .dmem_rdata(dmem_rdata)
  );

  ooo_dmem_model #(.MEM_WORDS(256), .MEM_MSB(9)) u_dmem (
    .clk(clk), .rst_n(rst_n),
    .addr(dmem_addr), .rdata(dmem_rdata),
    .we(dmem_we), .be(dmem_be), .wdata(dmem_wdata),
    .tohost_addr(32'hFFFF_FFFC), .tohost_full_addr(32'hFFFF_FFFC),
    .tohost_we(), .tohost_val()
  );

  initial clk = 1'b0;
  always #5 clk = ~clk;

  int checks, fails;
  task automatic chk(input logic cond, input string name,
                     input logic [31:0] got, input logic [31:0] exp);
    checks++;
    if (!cond) begin
      fails++;
      $display("  FAIL [%0d] %s got=%08h exp=%08h", checks, name, got, exp);
    end
  endtask

  // Capture committed wdata by program PC (32 entries cover PCs 0x00-0x7c).
  word_t got [32];
  logic  seen_tail;
  logic  x0_load_rd_wen;   // must stay 0: lw x0 reads dmem but writes nothing
  int    x0_load_reads;    // dmem_valid pulses observed for the x0 load
  // per-slot capture scratch (Icarus rejects a part-select chained
  // onto a variable array-element select, so copy the word first)
  integer cap_slot;
  word_t  cap_pc;
  word_t  cap_wdata;
  always @(posedge clk) begin
    if (rst_n) begin
      // capture every fired commit position in program order.
      for (cap_slot = 0; cap_slot < 2; cap_slot++) begin
        if (commit_fire[cap_slot]) begin
          cap_pc    = commit_pc[cap_slot];
          cap_wdata = commit_wdata[cap_slot];
          if (cap_pc[31:7] == '0) got[cap_pc[6:2]] <= cap_wdata;
          if (cap_pc == 32'h38)   x0_load_rd_wen <= commit_rd_wen[cap_slot];
          if (cap_pc == 32'h3c)   seen_tail <= 1'b1;
        end
      end
    end
  end
  always @(posedge clk)
    if (rst_n && dmem_valid) x0_load_reads <= x0_load_reads + 1;

  task automatic reset_dut();
    rst_n = 1'b0;
    repeat (4) @(posedge clk);
    rst_n = 1'b1;
  endtask

  integer i;
  initial begin
    checks = 0; fails = 0; seen_tail = 1'b0;
    x0_load_rd_wen = 1'b1; x0_load_reads = 0;
    for (i = 0; i < 32; i++) got[i] = '0;

    for (i = 0; i < 256; i++) imem[i] = 32'h00000013;  // NOP fill
    imem[ 0] = 32'h10000093;  // addi x1, x0, 0x100
    imem[ 1] = 32'h00008283;  // lb   x5,  0(x1)
    imem[ 2] = 32'h0000C303;  // lbu  x6,  0(x1)
    imem[ 3] = 32'h00108383;  // lb   x7,  1(x1)
    imem[ 4] = 32'h0010C403;  // lbu  x8,  1(x1)
    imem[ 5] = 32'h00208483;  // lb   x9,  2(x1)
    imem[ 6] = 32'h0020C503;  // lbu  x10, 2(x1)
    imem[ 7] = 32'h00308583;  // lb   x11, 3(x1)
    imem[ 8] = 32'h0030C603;  // lbu  x12, 3(x1)
    imem[ 9] = 32'h00009683;  // lh   x13, 0(x1)
    imem[10] = 32'h0000D703;  // lhu  x14, 0(x1)
    imem[11] = 32'h00209783;  // lh   x15, 2(x1)
    imem[12] = 32'h0020D803;  // lhu  x16, 2(x1)
    imem[13] = 32'h0000A883;  // lw   x17, 0(x1)
    imem[14] = 32'h0000A003;  // lw   x0,  0(x1)  (reads dmem, discards result)
    imem[15] = 32'h05500913;  // addi x18, x0, 0x55
    imem[16] = 32'h0000006F;  // jal  x0, 0 (spin)

    #1;  // let the dmem model's zero-fill initial run first
    u_dmem.mem[32'h100 >> 2] = 32'hF7B38291;

    reset_dut();
    for (i = 0; i < 4000 && !seen_tail; i++) @(posedge clk);
    repeat (4) @(posedge clk);

    chk(seen_tail,                    "liveness: tail committed", {31'b0, seen_tail}, 32'h1);
    chk(got[ 1] === 32'hFFFF_FF91,    "lb  off0 sign",  got[ 1], 32'hFFFF_FF91);
    chk(got[ 2] === 32'h0000_0091,    "lbu off0 zero",  got[ 2], 32'h0000_0091);
    chk(got[ 3] === 32'hFFFF_FF82,    "lb  off1 sign",  got[ 3], 32'hFFFF_FF82);
    chk(got[ 4] === 32'h0000_0082,    "lbu off1 zero",  got[ 4], 32'h0000_0082);
    chk(got[ 5] === 32'hFFFF_FFB3,    "lb  off2 sign",  got[ 5], 32'hFFFF_FFB3);
    chk(got[ 6] === 32'h0000_00B3,    "lbu off2 zero",  got[ 6], 32'h0000_00B3);
    chk(got[ 7] === 32'hFFFF_FFF7,    "lb  off3 sign",  got[ 7], 32'hFFFF_FFF7);
    chk(got[ 8] === 32'h0000_00F7,    "lbu off3 zero",  got[ 8], 32'h0000_00F7);
    chk(got[ 9] === 32'hFFFF_8291,    "lh  off0 sign",  got[ 9], 32'hFFFF_8291);
    chk(got[10] === 32'h0000_8291,    "lhu off0 zero",  got[10], 32'h0000_8291);
    chk(got[11] === 32'hFFFF_F7B3,    "lh  off2 sign",  got[11], 32'hFFFF_F7B3);
    chk(got[12] === 32'h0000_F7B3,    "lhu off2 zero",  got[12], 32'h0000_F7B3);
    chk(got[13] === 32'hF7B3_8291,    "lw  full word",  got[13], 32'hF7B3_8291);
    chk(x0_load_rd_wen === 1'b0,      "x0 load: rd write suppressed", {31'b0, x0_load_rd_wen}, 32'h0);
    chk(x0_load_reads == 14,          "x0 load still read dmem (14 loads total)", x0_load_reads[31:0], 32'd14);
    chk(got[15] === 32'h0000_0055,    "tail value",     got[15], 32'h0000_0055);

    if (fails == 0)
      $display("tb_rv32i_ss_core_lsu_sizes PASS checks=%0d", checks);
    else
      $display("tb_rv32i_ss_core_lsu_sizes FAIL fails=%0d checks=%0d", fails, checks);
    $finish;
  end

  initial begin
    #400000;
    $display("tb_rv32i_ss_core_lsu_sizes FAIL watchdog");
    $finish;
  end

endmodule
