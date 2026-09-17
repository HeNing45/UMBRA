// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

module rv32i_alu
  import fyp_cpu_pkg::*;
(
  input  word_t   operand_a,
  input  word_t   operand_b,
  input  alu_op_e alu_op,
  output word_t   result,
  output logic    zero
);
  word_t result_next;

  always_comb begin
    result_next = '0;

    unique case (alu_op)
      ALU_ADD: begin
        // Word-width addition; carry out is discarded.
        result_next = operand_a + operand_b;
      end

      ALU_SUB: begin
        // Word-width subtraction; borrow out is discarded.
        result_next = operand_a - operand_b;
      end

      ALU_AND: begin
        result_next = operand_a & operand_b;
      end

      ALU_OR: begin
        result_next = operand_a | operand_b;
      end

      ALU_XOR: begin
        result_next = operand_a ^ operand_b;
      end

      ALU_SLT: begin
        result_next = ($signed(operand_a) < $signed(operand_b)) ? 32'd1 : 32'd0;
      end

      ALU_SLTU: begin
        result_next = operand_a < operand_b ? 32'd1 : 32'd0;
      end

      ALU_SLL: begin
        result_next = operand_a << operand_b[4:0];
      end

      ALU_SRL: begin
        result_next = operand_a >> operand_b[4:0];
      end

      ALU_SRA: begin
        result_next = $signed(operand_a) >>> operand_b[4:0];
      end

      ALU_COPY_B: begin
        // Pass the immediate operand through for upper-immediate writes.
        result_next = operand_b;
      end

      default: begin
        result_next = '0;
      end
    endcase
  end

  assign result = result_next;
  assign zero = (result_next == '0);
endmodule
