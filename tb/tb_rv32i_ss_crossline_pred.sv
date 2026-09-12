`timescale 1ns/1ps

// cross-line slot-1 prediction consequence.
//
// The predictor is trained through the frontend's real update port while the
// memory request is withheld. The queue is then filled before dispatch is
// released, forcing {0x04 ALU, 0x08 predicted-taken branch}. Correct
// consumption removes both physical lines, so poison at follower.upper 0x0c
// can never commit; the already-steered target at 0x10 must commit instead.
module tb_rv32i_ss_crossline_pred;
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
  bit pred_pair_seen = 1'b0;
  bit drop_both_seen = 1'b0;
  bit poison_committed = 1'b0;
  bit marker_seen = 1'b0;

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
      if (`FE.bundle_fire && `FE.consume_cross_line &&
          (`FE.decoded_pc[0] == 32'h4) &&
          (`FE.decoded_pc[1] == 32'h8) &&
          (`FE.decoded_pred_taken == 2'b10) &&
          (`FE.decoded_pred_target == 32'h10))
        pred_pair_seen = 1'b1;
      if (`FE.consume_cross_line_drop_follower)
        drop_both_seen = 1'b1;

      for (int lane = 0; lane < 2; lane++) begin
        if (commit_fire[lane]) begin
          if (commit_pc[lane] == 32'h0c)
            poison_committed = 1'b1;
          if (commit_rd_wen[lane] && (commit_rd[lane] == 5'd21) &&
              (commit_wdata[lane] == 32'd21))
            marker_seen = 1'b1;
        end
      end
    end
  end

  initial begin
    repeat (3000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_crossline_pred exceeded 3000 cycles");
  end

  initial begin
    for (int i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
    imem[0] = 32'h3000_1073;  // 0x00 csrrw x0,mstatus,x0 -- solo
    imem[1] = 32'h0010_0093;  // 0x04 addi  x1,x0,1
    imem[2] = 32'h0000_0463;  // 0x08 beq   x0,x0,+8 -> 0x10
    imem[3] = 32'h0630_0a13;  // 0x0c POISON addi x20,x0,99
    imem[4] = 32'h0150_0a93;  // 0x10 addi  x21,x0,21 marker
    imem[5] = 32'h0000_006f;  // 0x14 jal   x0,0

    force u_cpu.decoded_ready = 1'b0;
    repeat (3) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;

    // Train the conditional entry before its request identity is accepted.
    force u_cpu.bp_update_valid  = 1'b1;
    force u_cpu.bp_update_pc     = 32'h0000_0008;
    force u_cpu.bp_update_taken  = 1'b1;
    force u_cpu.bp_update_target = 32'h0000_0010;
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
        $fatal(1, "queue never filled before Q1 predicted pair");
    end
    check("both registered lines present before dispatch", `FE.fq_count_q == 2);
    release u_cpu.decoded_ready;

    for (int wait_i = 0; !marker_seen; wait_i++) begin
      @(posedge clk);
      if (wait_i > 1000)
        $fatal(1, "predicted Q1 target marker never committed");
    end
    repeat (3) @(posedge clk);

    check("cross-line slot-1 prediction entered", pred_pair_seen);
    check("predicted slot 1 retired both queue lines", drop_both_seen);
    check("fall-through poison never committed", !poison_committed);
    check_arch("poison destination remains architectural zero", 20, 32'd0);
    check_arch("predicted target committed", 21, 32'd21);
    check("drop-follower counter records the event",
          `FE.q1_cross_line_drop_follower_count == 1);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_crossline_pred] PASS checks=%0d", checks);
      $finish;
    end
    $fatal(1, "[tb_rv32i_ss_crossline_pred] FAIL errors=%0d checks=%0d",
           errors, checks);
  end
endmodule
