// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// program-level TB: umbra_ss_cpu_top (frontend + core) + instruction
// memory.
//
// Loads a straight-line RV32I program into imem, lets the core fetch/decode/
// execute/commit it, then checks the architectural registers (read through the
// committed map into the PRF) against hand-computed expected values.
//
// Program (base PC = 0):
//   0x00: addi x1, x0, 5      x1 = 5
//   0x04: addi x2, x0, 7      x2 = 7
//   0x08: add  x3, x1, x2     x3 = 12
//   0x0c: sub  x4, x2, x1     x4 = 2
//   0x10: and  x5, x3, x4     x5 = 0
//   0x14: addi x6, x5, -1     x6 = 0xffffffff
//   0x18: auipc x7, 0x1       x7 = 0x18 + 0x1000 = 0x1018
//   0x1c+: NOP padding

module tb_rv32i_ss_core_prog;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;

  localparam time CLK_PERIOD = 10ns;

  logic clk;
  logic rst_n;

  // instruction memory
  word_t imem_addr;
  word_t [1:0] imem_rdata;
  logic imem_req_valid, imem_req_ready;
  word_t imem_req_addr;
  logic imem_resp_valid, imem_resp_ready;
  word_t [1:0] imem_resp_data;
  logic [31:0] imem [0:255];
  assign imem_rdata = {imem[{imem_addr[9:3], 1'b1}], imem[{imem_addr[9:3], 1'b0}]};

  rv32i_ss_imem_zero_latency_adapter u_imem_adapter (
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr), .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready), .imem_resp_data(imem_resp_data),
    .line_addr(imem_addr), .line_data(imem_rdata)
  );

  int errors = 0;
  int checks = 0;

  // DUT: wiring-only CPU top (frontend + core). Commit-trace and dmem ports
// stay unconnected;
  // this program is pure ALU, so dmem never handshakes.
  umbra_ss_cpu_top u_cpu (
    .clk        (clk),
    .rst_n      (rst_n),
    .imem_req_valid  (imem_req_valid),
    .imem_req_ready  (imem_req_ready),
    .imem_req_addr   (imem_req_addr),
    .imem_resp_valid (imem_resp_valid),
    .imem_resp_ready (imem_resp_ready),
    .imem_resp_data  (imem_resp_data)
  );

  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;

  initial begin
    repeat (3000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_prog exceeded 3000 cycles");
  end

  `define ROB u_cpu.u_core.u_rob
  `define RN  u_cpu.u_core.u_rename
  `define PRF u_cpu.u_core.u_prf

  // commit monitor (debug visibility)
  always @(posedge clk) begin
    if (rst_n && `ROB.commit_valid[0]) begin
      $display("[commit %0d] pc=%08h rd=x%0d result=%08h",
               `ROB.commit_order_q, `ROB.commit_pc[0], `ROB.commit_rd[0],
               `ROB.commit_result[0]);
    end
  end

  task automatic check_arch(input string name, input int r, input word_t exp);
    automatic phys_reg_t p = `RN.committed_map_q[r];
    checks++;
    if (`PRF.regs_q[p] !== exp) begin
      $error("[%s] x%0d=%08h (via p%0d) expected %08h",
             name, r, `PRF.regs_q[p], p, exp);
      errors++;
    end
  endtask

  integer i;

  initial begin
    $display("[tb_rv32i_ss_core_prog] starting");

    // default-fill imem with NOP (addi x0,x0,0)
    for (i = 0; i < 256; i = i + 1) imem[i] = 32'h0000_0013;

    // program
    imem[0] = 32'h0050_0093;  // addi x1, x0, 5
    imem[1] = 32'h0070_0113;  // addi x2, x0, 7
    imem[2] = 32'h0020_81b3;  // add  x3, x1, x2
    imem[3] = 32'h4011_0233;  // sub  x4, x2, x1
    imem[4] = 32'h0041_f2b3;  // and  x5, x3, x4
    imem[5] = 32'hfff2_8313;  // addi x6, x5, -1
    imem[6] = 32'h0000_1397;  // auipc x7, 0x1

    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;

    // run until the 7 real instructions have committed
    i = 0;
    while (`ROB.commit_order_q < 64'd7) begin
      @(posedge clk);
      i = i + 1;
      if (i > 800) $fatal(1, "stuck: commit_order=%0d", `ROB.commit_order_q);
    end
    repeat (2) @(posedge clk);

    check_arch("x1", 1, 32'h0000_0005);
    check_arch("x2", 2, 32'h0000_0007);
    check_arch("x3", 3, 32'h0000_000c);
    check_arch("x4", 4, 32'h0000_0002);
    check_arch("x5", 5, 32'h0000_0000);
    check_arch("x6", 6, 32'hffff_ffff);
    check_arch("x7", 7, 32'h0000_1018);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_core_prog] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_core_prog] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
