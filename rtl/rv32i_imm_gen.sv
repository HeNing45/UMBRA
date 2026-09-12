`timescale 1ns/1ps

module rv32i_imm_gen
  import fyp_cpu_pkg::*;
(
  input  logic [31:0] instr,
  input  imm_sel_e    imm_sel,
  output word_t       imm
);
  word_t imm_next;

  always_comb begin
    imm_next = '0;

    unique case (imm_sel)
      IMM_I: begin
        // Sign-extend the contiguous I-type immediate.
        imm_next = {{20{instr[31]}}, {instr[31:20]}};
      end

      IMM_S: begin
        // Assemble and sign-extend the split S-type immediate.
        imm_next = {{20{instr[31]}}, instr[31:25], instr[11:7]};
      end

      IMM_B: begin
        // Assemble the signed branch displacement with its implicit zero bit.
        imm_next = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
      end

      IMM_U: begin
        // Place the upper immediate above twelve zero bits.
        imm_next = {instr[31:12], 12'b0};
      end

      IMM_J: begin
        // Assemble the signed jump displacement with its implicit zero bit.
        imm_next = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
      end

      default: begin
        imm_next = '0;
      end
    endcase
  end

  assign imm = imm_next;
endmodule
