`timescale 1ns/1ps

module rv32i_pc_logic
  import fyp_cpu_pkg::*;
(
  input  word_t   pc_current,
  input  word_t   imm,
  input  word_t   rs1_data,
  input  logic    branch_taken,
  input  pc_sel_e pc_sel,
  output word_t   pc_next
);
  word_t pc_next_comb;

  always_comb begin
    pc_next_comb = pc_current;

    unique case (pc_sel)
      PC_PLUS4: begin
        // Instructions occupy four bytes.
        pc_next_comb = pc_current + 32'd4;
      end

      PC_BRANCH: begin
        // A taken branch selects its PC-relative target.
        if (branch_taken) begin
          pc_next_comb = pc_current + imm;
        end
        else begin
          pc_next_comb = pc_current + 32'd4;
        end
      end

      PC_JAL: begin
        pc_next_comb = pc_current + imm;
      end

      PC_JALR: begin
        // JALR adds the immediate to rs1 and clears target bit zero.
        pc_next_comb = rs1_data + imm;
        pc_next_comb[0] = 1'b0;
      end

      PC_HOLD: begin
        pc_next_comb = pc_current;
      end

      default: begin
        pc_next_comb = pc_current;
      end
    endcase
  end

  assign pc_next = pc_next_comb;
endmodule
