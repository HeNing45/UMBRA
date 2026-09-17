// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// tb_rv32i_ss_ras_bench — alternating-call-site return microbenchmark.
// The program calls one function from two alternating sites in a 32-iteration
// loop (64 calls and 64 returns). A last-target predictor would miss nearly
// every return; the RAS is primed by each call and can predict all 64.
// The test asserts architectural results and prints performance counters so
// serialized-return and RAS implementations can be compared.
//
// Output: [tb_rv32i_ss_ras_bench] MEASURE cycles=<C> jump_allocs=<J>
//   cycles: reset release to end-marker commit.
//   jump_allocs: jump_alloc_fire count; each serialized jump blocks frontend
//               dispatch until it resolves at issue.

module tb_rv32i_ss_ras_bench;
  import rv32i_ss_pkg::*;

  localparam time CLK_PERIOD = 10ns;

  logic clk;
  logic rst_n;

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
    repeat (8000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_ras_bench exceeded 8000 cycles");
  end

  `define CORE u_cpu.u_core
  `define RN   u_cpu.u_core.u_rename
  `define PRF  u_cpu.u_core.u_prf

  task automatic check_arch(input string name, input int r, input word_t exp);
    automatic phys_reg_t p = `RN.committed_map_q[r];
    checks++;
    if (`PRF.regs_q[p] !== exp) begin
      $error("[%s] x%0d=%08h (via p%0d) expected %08h",
             name, r, `PRF.regs_q[p], p, exp);
      errors++;
    end
  endtask

  // ---------------- measurement monitors ----------------
  int cycles;
  int n_jump_allocs;
  int n_ras_predictions;
  int n_decode_ras_redirects;
  int n_fetch_ras_dispatches;
  bit marker_seen;

  always @(posedge clk) begin
    if (rst_n === 1'b1) begin
      if (!marker_seen) cycles++;
      if (`CORE.jump_alloc_fire) n_jump_allocs++;
      if (`CORE.ras_pred_fire) n_ras_predictions++;
      if (`CORE.ras_decode_redirect_fire) n_decode_ras_redirects++;
      if (`CORE.ras_pred_fire && `CORE.ras_fetch_pred_want)
        n_fetch_ras_dispatches++;
      if ((`CORE.commit_fire[0] && `CORE.commit_rd_wen[0] &&
           (`CORE.commit_rd[0] == 14) && (`CORE.commit_wdata[0] == 32'd14)) ||
          (`CORE.commit_fire[1] && `CORE.commit_rd_wen[1] &&
           (`CORE.commit_rd[1] == 14) && (`CORE.commit_wdata[1] == 32'd14)))
        marker_seen = 1'b1;
    end
  end

  integer i;
  int wait_i;

  initial begin
    $display("[tb_rv32i_ss_ras_bench] starting");
    cycles = 0;
    n_jump_allocs = 0;
    n_ras_predictions = 0;
    n_decode_ras_redirects = 0;
    n_fetch_ras_dispatches = 0;
    marker_seen = 1'b0;

    for (i = 0; i < 256; i = i + 1) imem[i] = 32'h0000_0013;

    imem[0]  = 32'h0000_0393;  // 0x00 addi x7,x0,0    (accumulator)
    imem[1]  = 32'h0200_0313;  // 0x04 addi x6,x0,32   (loop count)
    // loop:
    imem[2]  = 32'h0380_00EF;  // 0x08 jal  x1,+0x38 -> F (call site A)
    imem[3]  = 32'h0013_8393;  // 0x0c addi x7,x7,1
    imem[4]  = 32'h0300_00EF;  // 0x10 jal  x1,+0x30 -> F (call site B)
    imem[5]  = 32'h0033_8393;  // 0x14 addi x7,x7,3
    imem[6]  = 32'hFFF3_0313;  // 0x18 addi x6,x6,-1
    imem[7]  = 32'hFE03_16E3;  // 0x1c bne  x6,x0,-20 -> 0x08
    imem[8]  = 32'h00E0_0713;  // 0x20 addi x14,x0,14  (end marker)
    imem[9]  = 32'h0000_006F;  // 0x24 jal  x0,0       (self-loop)
    // F:
    imem[16] = 32'h0053_8393;  // 0x40 addi x7,x7,5
    imem[17] = 32'h0000_8067;  // 0x44 jalr x0,0(x1)   (ret)

    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;

    wait_i = 0;
    while (!marker_seen) begin
      @(posedge clk);
      wait_i++;
      if (wait_i > 6000) $fatal(1, "stuck: end marker never committed");
    end
    repeat (4) @(posedge clk);

    // Architectural consequence: 32 iterations x (5+1+5+3) = 448.
    check_arch("x7 accumulator (64 calls landed)", 7,  32'd448);
    check_arch("x6 counter drained",               6,  32'd0);
    check_arch("x14 end marker",                   14, 32'd14);

    checks++;
    if (cycles != 374) begin
      $error("[D-015 no duplicate return refill] cycles=%0d exp=374", cycles);
      errors++;
    end

    checks++;
    if (n_ras_predictions != 65) begin
      $error("[all 64 returns predicted] got=%0d exp=65", n_ras_predictions);
      errors++;
    end
    checks++;
    if (n_decode_ras_redirects != 1) begin
      $error("[only cold return used dispatch redirect] got=%0d exp=1",
             n_decode_ras_redirects);
      errors++;
    end
    checks++;
    if (n_fetch_ras_dispatches != 64) begin
      $error("[learned returns used fetch prediction] got=%0d exp=64",
             n_fetch_ras_dispatches);
      errors++;
    end
    checks++;
    if (u_cpu.u_fe.d015_req_return_upper_count < n_fetch_ras_dispatches) begin
      $error("[request-time return predictions cover dispatched fetch predictions] req=%0d dispatch=%0d",
             u_cpu.u_fe.d015_req_return_upper_count,
             n_fetch_ras_dispatches);
      errors++;
    end

    $display("[tb_rv32i_ss_ras_bench] MEASURE cycles=%0d jump_allocs=%0d fetch_ras=%0d decode_ras=%0d",
             cycles, n_jump_allocs,
             n_fetch_ras_dispatches,
             n_decode_ras_redirects);
    if (errors == 0) begin
      $display("[tb_rv32i_ss_ras_bench] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_ras_bench] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
