// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

module tb_rv32i_ss_crossline_undispatched_repair;
  import rv32i_ss_pkg::*;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  logic memory_enable = 1'b0;
  logic imem_req_valid, adapter_req_ready, imem_req_ready;
  word_t imem_req_addr;
  logic adapter_resp_valid, imem_resp_valid, imem_resp_ready;
  word_t [1:0] imem_resp_data;
  word_t imem_addr;
  word_t [1:0] imem_rdata;
  word_t imem [0:255];

  logic [1:0] commit_fire;
  commit_order_t commit_order;
  word_t [1:0] commit_pc, commit_inst, commit_wdata;
  arch_reg_t [1:0] commit_rd;
  logic [1:0] commit_rd_wen;
  logic dmem_valid, dmem_we;
  logic [3:0] dmem_be;
  word_t dmem_addr, dmem_wdata;

  assign imem_rdata = {imem[{imem_addr[9:3], 1'b1}],
                       imem[{imem_addr[9:3], 1'b0}]};
  assign imem_req_ready = memory_enable && adapter_req_ready;
  assign imem_resp_valid = memory_enable && adapter_resp_valid;

  rv32i_ss_imem_zero_latency_adapter u_imem_adapter (
    .imem_req_valid (imem_req_valid),
    .imem_req_ready (adapter_req_ready),
    .imem_req_addr  (imem_req_addr),
    .imem_resp_valid(adapter_resp_valid),
    .imem_resp_ready(imem_resp_ready),
    .imem_resp_data (imem_resp_data),
    .line_addr      (imem_addr),
    .line_data      (imem_rdata)
  );

  umbra_ss_cpu_top u_cpu (
    .clk(clk), .rst_n(rst_n),
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr), .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready), .imem_resp_data(imem_resp_data),
    .commit_fire(commit_fire), .commit_order(commit_order),
    .commit_pc(commit_pc), .commit_inst(commit_inst),
    .commit_rd(commit_rd), .commit_rd_wen(commit_rd_wen),
    .commit_wdata(commit_wdata),
    .dmem_valid(dmem_valid), .dmem_we(dmem_we), .dmem_be(dmem_be),
    .dmem_addr(dmem_addr), .dmem_wdata(dmem_wdata),
    .dmem_ready(1'b1), .dmem_rvalid(1'b1), .dmem_rdata('0)
  );

  `define FE  u_cpu.u_fe
  `define RN  u_cpu.u_core.u_rename
  `define PRF u_cpu.u_core.u_prf

  int checks = 0;
  int errors = 0;
  bit pc08_retired = 1'b0;
  bit x20_retired = 1'b0;
  bit marker_seen = 1'b0;
  bit acc04_seen = 1'b0;
  bit acc08_seen = 1'b0;
  logic [1:0] acc04_slot_valid;
  logic acc04_pred0, acc04_pred1;
  logic acc04_mm0, acc04_mm1, acc04_mm;
  logic acc04_repair_fire;
  word_t acc04_pred_target;
  logic [1:0] acc08_slot_valid;
  logic acc08_mm, acc08_repair_fire;
  word_t acc08_repair_target;

  task automatic check(input string name, input logic condition);
    checks++;
    if (!condition) begin
      errors++;
      $error("[%s]", name);
    end
  endtask

  task automatic check_arch(input string name, input int r, input word_t exp);
    automatic phys_reg_t p = `RN.committed_map_q[r];
    checks++;
    if (`PRF.regs_q[p] !== exp) begin
      errors++;
      $error("[%s] x%0d=%08h via p%0d, expected %08h",
             name, r, `PRF.regs_q[p], p, exp);
    end
  endtask

  always @(posedge clk) begin
    if (rst_n === 1'b1) begin
      if (!acc04_seen && `FE.bundle_fire &&
          (`FE.decoded_pc[0] == 32'h04)) begin
        acc04_seen        <= 1'b1;
        acc04_slot_valid  <= `FE.decoded_slot_valid;
        acc04_pred0       <= `FE.head_pred_at_slot0;
        acc04_pred1       <= `FE.head_pred_at_slot1;
        acc04_mm0         <= `FE.head_pred_target_mismatch0;
        acc04_mm1         <= `FE.head_pred_target_mismatch1;
        acc04_mm          <= `FE.head_pred_target_mismatch;
        acc04_pred_target <= `FE.decoded_pred_target;
        acc04_repair_fire <= `FE.steer_repair_fire;
      end
      if (!acc08_seen && `FE.bundle_fire &&
          (`FE.decoded_pc[0] == 32'h08)) begin
        acc08_seen          <= 1'b1;
        acc08_slot_valid    <= `FE.decoded_slot_valid;
        acc08_mm            <= `FE.head_pred_target_mismatch;
        acc08_repair_fire   <= `FE.steer_repair_fire;
        acc08_repair_target <= `FE.steer_repair_target;
      end
      for (int lane = 0; lane < 2; lane++) begin
        if (commit_fire[lane]) begin
          if (commit_pc[lane] == 32'h08)
            pc08_retired = 1'b1;
          if (commit_rd_wen[lane] && (commit_rd[lane] == 5'd20) &&
              (commit_wdata[lane] == 32'd99))
            x20_retired = 1'b1;
          if (commit_rd_wen[lane] && (commit_rd[lane] == 5'd21) &&
              (commit_wdata[lane] == 32'd21))
            marker_seen = 1'b1;
        end
      end
    end
  end

  initial begin
    repeat (3000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_crossline_undispatched_repair exceeded 3000 cycles");
  end

  initial begin
    for (int i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
    imem[0] = 32'h3000_1073;  // 0x00 csrrw x0,mstatus,x0
    imem[1] = 32'h0000_0263;  // 0x04 beq   x0,x0,+4 -> 0x08
    imem[2] = 32'h0000_1c63;  // 0x08 bne   x0,x0,+24 -> 0x20, actually not taken
    imem[3] = 32'h0630_0a13;  // 0x0c addi  x20,x0,99
    imem[4] = 32'h0100_006f;  // 0x10 jal   x0,+16 -> 0x20
    imem[8] = 32'h0150_0a93;  // 0x20 addi  x21,x0,21
    imem[9] = 32'h0000_006f;  // 0x24 jal   x0,0

    force u_cpu.decoded_ready = 1'b0;
    repeat (3) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;

    force u_cpu.bp_update_valid  = 1'b1;
    force u_cpu.bp_update_pc     = 32'h0000_0004;
    force u_cpu.bp_update_taken  = 1'b1;
    force u_cpu.bp_update_target = 32'h0000_0008;
    @(posedge clk);
    @(negedge clk);
    release u_cpu.bp_update_valid;
    release u_cpu.bp_update_pc;
    release u_cpu.bp_update_taken;
    release u_cpu.bp_update_target;

    force u_cpu.bp_update_valid  = 1'b1;
    force u_cpu.bp_update_pc     = 32'h0001_0008;
    force u_cpu.bp_update_taken  = 1'b1;
    force u_cpu.bp_update_target = 32'h0001_0040;
    @(posedge clk);
    @(negedge clk);
    release u_cpu.bp_update_valid;
    release u_cpu.bp_update_pc;
    release u_cpu.bp_update_taken;
    release u_cpu.bp_update_target;
    memory_enable = 1'b1;

    for (int wait_i = 0; `FE.fq_count_q != 2; wait_i++) begin
      @(negedge clk);
      if (wait_i > 100)
        $fatal(1, "queue never filled before undispatched follower pair");
    end
    release u_cpu.decoded_ready;

    for (int wait_i = 0; !marker_seen; wait_i++) begin
      @(posedge clk);
      if (wait_i > 1000)
        $fatal(1, "marker never committed");
    end
    repeat (3) @(posedge clk);

    check("PC 0x04 bundle accepted", acc04_seen);
    check("PC 0x04 accepted as slot-0-only shape", acc04_slot_valid == 2'b01);
    check("PC 0x04 carries its slot-0 prediction", acc04_pred0);
    check("undispatched follower prediction visible at slot 1", acc04_pred1);
    check("PC 0x04 slot-0 target matches", !acc04_mm0);
    check("follower slot-1 target mismatches", acc04_mm1);
    check("PC 0x04 packet keeps trained target", acc04_pred_target == 32'h08);
    check("undispatched follower mismatch is not consumed", !acc04_mm);
    check("no spurious repair on PC 0x04", !acc04_repair_fire);
    check("PC 0x08 bundle accepted", acc08_seen);
    check("PC 0x08 accepted as slot-0-only shape", acc08_slot_valid == 2'b01);
    check("consumed stale prediction mismatches", acc08_mm);
    check("legitimate repair fires on PC 0x08", acc08_repair_fire);
    check("repair target is the exact branch target", acc08_repair_target == 32'h20);
    check("PC 0x08 retires", pc08_retired);
    check("x20=99 write retires", x20_retired);
    check("x21=21 marker retires", marker_seen);
    check_arch("architectural x20", 20, 32'd99);
    check_arch("architectural x21", 21, 32'd21);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_crossline_undispatched_repair] PASS checks=%0d", checks);
      $finish;
    end
    $fatal(1, "[tb_rv32i_ss_crossline_undispatched_repair] FAIL errors=%0d checks=%0d",
           errors, checks);
  end
endmodule
