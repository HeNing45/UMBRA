// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// Predictor-kind and dual-update contract.
//
// Return-site learning updates metadata and must never displace a conditional
// execute update on a shared row. Different rows may update together.
module tb_rv32i_ss_bp_kind;
  import fyp_cpu_pkg::*;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  word_t pc_f0, pc_f1;
  logic predict_taken_f0, predict_taken_f1;
  logic predict_return_f0, predict_return_f1;
  word_t predict_target_f0, predict_target_f1;

  logic update_valid_e;
  word_t update_pc_e;
  logic update_taken_e;
  word_t update_target_e;
  logic return_update_valid;
  word_t return_update_pc;

  int checks = 0;
  int errors = 0;

  rv32i_ss_bp dut (
    .clk(clk),
    .rst_n(rst_n),
    .pc_f0(pc_f0),
    .predict_taken_f0(predict_taken_f0),
    .predict_return_f0(predict_return_f0),
    .predict_target_f0(predict_target_f0),
    .pc_f1(pc_f1),
    .predict_taken_f1(predict_taken_f1),
    .predict_return_f1(predict_return_f1),
    .predict_target_f1(predict_target_f1),
    .update_valid_e(update_valid_e),
    .update_pc_e(update_pc_e),
    .update_taken_e(update_taken_e),
    .update_target_e(update_target_e),
    .return_update_valid(return_update_valid),
    .return_update_pc(return_update_pc)
  );

  task automatic check(input string name, input logic condition);
    checks++;
    if (!condition) begin
      $error("[%s]", name);
      errors++;
    end
  endtask

  task automatic update_edge(
    input logic branch_valid,
    input word_t branch_pc,
    input word_t branch_target,
    input logic ret_valid,
    input word_t ret_pc
  );
    @(negedge clk);
    update_valid_e = branch_valid;
    update_pc_e = branch_pc;
    update_taken_e = 1'b1;
    update_target_e = branch_target;
    return_update_valid = ret_valid;
    return_update_pc = ret_pc;
    @(posedge clk);
    @(negedge clk);
    update_valid_e = 1'b0;
    return_update_valid = 1'b0;
  endtask

  initial begin
    repeat (100) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_bp_kind exceeded 100 cycles");
  end

  initial begin
    $display("[tb_rv32i_ss_bp_kind] starting");
    pc_f0 = '0;
    pc_f1 = '0;
    update_valid_e = 1'b0;
    update_pc_e = '0;
    update_taken_e = 1'b0;
    update_target_e = '0;
    return_update_valid = 1'b0;
    return_update_pc = '0;

    repeat (3) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;

    pc_f0 = 32'h0000_0040;
    #1;
    check("reset leaves lookup cold", !predict_taken_f0 && !predict_return_f0);

    // A metadata-only return update creates a return-kind row. Its stale
    // target payload is intentionally irrelevant to a return prediction.
    update_edge(1'b0, '0, '0, 1'b1, 32'h0000_0040);
    pc_f0 = 32'h0000_0040;
    #1;
    check("return-site update creates return kind",
          predict_return_f0 && !predict_taken_f0);

    // Same row: conditional training wins and converts the row back to a
    // branch. The conditional update wins this collision.
    update_edge(1'b1, 32'h0000_0040, 32'h0000_0100,
                1'b1, 32'h0000_0040);
    pc_f0 = 32'h0000_0040;
    #1;
    check("same-row conditional update wins kind",
          predict_taken_f0 && !predict_return_f0);
    check("same-row conditional update wins target",
          predict_target_f0 == 32'h0000_0100);

    // Different rows: both writes survive the same edge and both lookup
    // ports expose their independently typed entries.
    update_edge(1'b1, 32'h0000_0080, 32'h0000_0180,
                1'b1, 32'h0000_0084);
    pc_f0 = 32'h0000_0080;
    pc_f1 = 32'h0000_0084;
    #1;
    check("different-row branch update survives",
          predict_taken_f0 && !predict_return_f0 &&
          (predict_target_f0 == 32'h0000_0180));
    check("different-row return update survives",
          predict_return_f1 && !predict_taken_f1);

    // Same physical row but different tag: row ownership, not a matching
    // tag, defines the collision. The conditional update must still win.
    update_edge(1'b1, 32'h0000_0140, 32'h0000_0240,
                1'b1, 32'h0000_0040);
    pc_f0 = 32'h0000_0140;
    pc_f1 = 32'h0000_0040;
    #1;
    check("same-index different-tag branch owns row",
          predict_taken_f0 && !predict_return_f0 &&
          (predict_target_f0 == 32'h0000_0240));
    check("same-index losing return tag is absent",
          !predict_taken_f1 && !predict_return_f1);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_bp_kind] PASS checks=%0d", checks);
      $finish;
    end
    $fatal(1, "[tb_rv32i_ss_bp_kind] FAIL errors=%0d checks=%0d",
           errors, checks);
  end
endmodule
