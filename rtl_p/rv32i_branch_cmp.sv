// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// Conditional-branch comparator. Lives in the E stage and feeds the PC mux
// alongside the unconditional-jump path. Pure combinational.
module rv32i_branch_cmp
  import fyp_cpu_pkg::*;
  import rv32i_pipeline_pkg::*;
(
  input  br_type_e branch_op,
  input  word_t    a,
  input  word_t    b,
  output logic     taken
);
  logic eq;
  logic lt_s;
  logic lt_u;
  logic taken_next;

  assign eq   = (a == b);
  assign lt_s = ($signed(a) < $signed(b));
  assign lt_u = (a < b);

  always_comb begin
    taken_next = 1'b0;

    unique case (branch_op)
      BR_NONE: taken_next = 1'b0;
      BR_BEQ:  taken_next = eq;
      BR_BNE:  taken_next = !eq;
      BR_BLT:  taken_next = lt_s;
      BR_BGE:  taken_next = !lt_s;
      BR_BLTU: taken_next = lt_u;
      BR_BGEU: taken_next = !lt_u;
      default: taken_next = 1'b0;
    endcase
  end

  assign taken = taken_next;
endmodule
