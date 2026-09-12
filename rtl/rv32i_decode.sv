`timescale 1ns/1ps

module rv32i_decode
  import fyp_cpu_pkg::*;
(
  input  logic [31:0] instr,
  output reg_addr_t   rs1_addr,
  output reg_addr_t   rs2_addr,
  output reg_addr_t   rd_addr,
  output decode_ctrl_t ctrl
);
  logic [6:0] opcode;
  logic [2:0] funct3;
  logic [6:0] funct7;
  decode_ctrl_t ctrl_next;

  assign opcode   = instr[6:0];
  assign rd_addr  = instr[11:7];
  assign funct3   = instr[14:12];
  assign rs1_addr = instr[19:15];
  assign rs2_addr = instr[24:20];
  assign funct7   = instr[31:25];

  always_comb begin
    ctrl_next = '0;
    ctrl_next.wb_sel = WB_NONE;
    ctrl_next.pc_sel = PC_PLUS4;
    ctrl_next.branch_op = SC_BR_NONE;
    ctrl_next.alu_op = ALU_INVALID;
    ctrl_next.imm_sel = IMM_NONE;
    ctrl_next.illegal = 1'b1;
    ctrl_next.mem_size = MEM_W;
    ctrl_next.mem_unsigned = 1'b0;

    unique case (opcode)
      OPCODE_OP: begin
        // Select register-register operations using funct3 and funct7.
        ctrl_next.reg_write = 1'b1;
        ctrl_next.mem_write = 1'b0;
        ctrl_next.imm_sel = IMM_NONE;
        ctrl_next.illegal = 1'b0;
        ctrl_next.alu_a_sel = ALU_A_RS1;
        ctrl_next.alu_b_sel = ALU_B_RS2;
        ctrl_next.wb_sel = WB_ALU;
        unique case ({funct7, funct3})
          {7'b0000000, 3'b000} : ctrl_next.alu_op = ALU_ADD;
          {7'b0100000, 3'b000} : ctrl_next.alu_op = ALU_SUB;
          {7'b0000000, 3'b001} : ctrl_next.alu_op = ALU_SLL;
          {7'b0000000, 3'b010} : ctrl_next.alu_op = ALU_SLT;
          {7'b0000000, 3'b011} : ctrl_next.alu_op = ALU_SLTU;
          {7'b0000000, 3'b100} : ctrl_next.alu_op = ALU_XOR;
          {7'b0000000, 3'b101} : ctrl_next.alu_op = ALU_SRL;
          {7'b0100000, 3'b101} : ctrl_next.alu_op = ALU_SRA;
          {7'b0000000, 3'b110} : ctrl_next.alu_op = ALU_OR;
          {7'b0000000, 3'b111} : ctrl_next.alu_op = ALU_AND;
          default: begin
            ctrl_next.illegal = 1'b1;
            ctrl_next.reg_write = 1'b0;
            ctrl_next.alu_op = ALU_INVALID;
            ctrl_next.wb_sel = WB_NONE;
          end
        endcase
      end

      OPCODE_OP_IMM: begin
        // Select ALU-immediate operations; shifts also validate funct7.
        ctrl_next.reg_write = 1'b1;
        ctrl_next.mem_write = 1'b0;
        ctrl_next.imm_sel = IMM_I;
        ctrl_next.illegal = 1'b0;
        ctrl_next.alu_a_sel = ALU_A_RS1;
        ctrl_next.alu_b_sel = ALU_B_IMM;
        ctrl_next.wb_sel = WB_ALU;
        unique case (funct3)
          3'b000 : ctrl_next.alu_op = ALU_ADD;
          3'b010 : ctrl_next.alu_op = ALU_SLT;
          3'b011 : ctrl_next.alu_op = ALU_SLTU;
          3'b100 : ctrl_next.alu_op = ALU_XOR;
          3'b110 : ctrl_next.alu_op = ALU_OR;
          3'b111 : ctrl_next.alu_op = ALU_AND;
          3'b001 : unique case (funct7)
            7'b0000000 : ctrl_next.alu_op = ALU_SLL;
            default: begin
            ctrl_next.illegal = 1'b1;
            ctrl_next.reg_write = 1'b0;
            ctrl_next.alu_op = ALU_INVALID;
            ctrl_next.wb_sel = WB_NONE;
            end
          endcase
          3'b101 : unique case (funct7)
            7'b0000000 : ctrl_next.alu_op = ALU_SRL;
            7'b0100000 : ctrl_next.alu_op = ALU_SRA;
            default: begin
              ctrl_next.illegal = 1'b1;
              ctrl_next.reg_write = 1'b0;
              ctrl_next.alu_op = ALU_INVALID;
              ctrl_next.wb_sel = WB_NONE;
            end
          endcase

          default: begin
            ctrl_next.illegal = 1'b1;
            ctrl_next.reg_write = 1'b0;
            ctrl_next.alu_op = ALU_INVALID;
            ctrl_next.wb_sel = WB_NONE;
          end
        endcase
      end

      OPCODE_LOAD: begin
        // Decode byte, halfword and word loads, including unsigned variants.
        ctrl_next.reg_write = 1'b1;
        ctrl_next.mem_write = 1'b0;
        ctrl_next.imm_sel   = IMM_I;
        ctrl_next.alu_a_sel = ALU_A_RS1;
        ctrl_next.alu_b_sel = ALU_B_IMM;
        ctrl_next.alu_op    = ALU_ADD;
        ctrl_next.wb_sel    = WB_MEM;
        ctrl_next.illegal   = 1'b0;
        unique case(funct3)
          3'b000 : begin ctrl_next.mem_size = MEM_B;
                        ctrl_next.mem_unsigned = 1'b0;
          end
          3'b001 : begin ctrl_next.mem_size = MEM_H;
                        ctrl_next.mem_unsigned = 1'b0;
          end
          3'b010 : begin ctrl_next.mem_size = MEM_W;
                        ctrl_next.mem_unsigned = 1'b0;
          end

          3'b100 : begin ctrl_next.mem_size = MEM_B;
                        ctrl_next.mem_unsigned = 1'b1;
          end

          3'b101 : begin ctrl_next.mem_size = MEM_H;
                        ctrl_next.mem_unsigned = 1'b1;
          end

          default: begin
            ctrl_next.illegal = 1'b1;
            ctrl_next.reg_write = 1'b0;
            ctrl_next.alu_op = ALU_INVALID;
            ctrl_next.wb_sel = WB_NONE;
          end
        endcase
      end

      OPCODE_STORE: begin
        // Decode byte, halfword and word stores.
        ctrl_next.reg_write = 1'b0;
        ctrl_next.mem_write = 1'b1;
        ctrl_next.imm_sel = IMM_S;
        ctrl_next.alu_a_sel = ALU_A_RS1;
        ctrl_next.alu_b_sel = ALU_B_IMM;
        ctrl_next.alu_op = ALU_ADD;
        ctrl_next.wb_sel = WB_NONE;
        ctrl_next.illegal = 1'b0;
        unique case (funct3)
          3'b000: begin
            ctrl_next.mem_size = MEM_B;
          end
          3'b001: begin
            ctrl_next.mem_size = MEM_H;
          end
          3'b010: begin
            ctrl_next.mem_size = MEM_W;
          end

          default: begin
            ctrl_next.illegal = 1'b1;
            ctrl_next.mem_write = 1'b0;
            ctrl_next.alu_op = ALU_INVALID;
          end
        endcase

      end

      OPCODE_BRANCH: begin
        ctrl_next.reg_write = 1'b0;
        ctrl_next.mem_write = 1'b0;
        ctrl_next.imm_sel = IMM_B;
        ctrl_next.alu_a_sel = ALU_A_RS1;
        ctrl_next.alu_b_sel = ALU_B_RS2;
        ctrl_next.alu_op = ALU_SUB;
        ctrl_next.wb_sel = WB_NONE;
        ctrl_next.pc_sel = PC_BRANCH;
        // Select equality or signed/unsigned ordering for branch resolution.
        unique case (funct3)
          3'b000 : begin
            ctrl_next.branch_op = SC_BR_EQ;
            ctrl_next.illegal = 1'b0;
          end

          3'b001 : begin
            ctrl_next.branch_op = SC_BR_NE;
            ctrl_next.illegal = 1'b0;
          end

          3'b100 : begin
            ctrl_next.branch_op = SC_BR_LT;
            ctrl_next.alu_op = ALU_SLT;   // reuse ALU signed less-than
            ctrl_next.illegal = 1'b0;
          end

          3'b101 : begin
            ctrl_next.branch_op = SC_BR_GE;
            ctrl_next.alu_op = ALU_SLT;   // bge = !(rs1 <s rs2)
            ctrl_next.illegal = 1'b0;
          end

          3'b110 : begin
            ctrl_next.branch_op = SC_BR_LTU;
            ctrl_next.alu_op = ALU_SLTU;  // reuse ALU unsigned less-than
            ctrl_next.illegal = 1'b0;
          end

          3'b111 : begin
            ctrl_next.branch_op = SC_BR_GEU;
            ctrl_next.alu_op = ALU_SLTU;  // bgeu = !(rs1 <u rs2)
            ctrl_next.illegal = 1'b0;
          end

          default: begin
            ctrl_next.illegal = 1'b1;
            ctrl_next.pc_sel = PC_PLUS4;
            ctrl_next.alu_op = ALU_INVALID;
          end
        endcase
      end

      OPCODE_JAL: begin
        ctrl_next.reg_write = 1'b1;
        ctrl_next.mem_write = 1'b0;
        ctrl_next.imm_sel = IMM_J;
        ctrl_next.alu_a_sel = ALU_A_PC;
        ctrl_next.alu_b_sel = ALU_B_IMM;
        ctrl_next.alu_op = ALU_ADD;
        ctrl_next.wb_sel = WB_PC4;
        ctrl_next.pc_sel = PC_JAL;
        ctrl_next.illegal = 1'b0;
      end

      OPCODE_JALR: begin
        unique case (funct3)
          3'b000 : begin
            ctrl_next.reg_write = 1'b1;
            ctrl_next.mem_write = 1'b0;
            ctrl_next.imm_sel = IMM_I;
            ctrl_next.alu_a_sel = ALU_A_RS1;
            ctrl_next.alu_b_sel = ALU_B_IMM;
            ctrl_next.alu_op = ALU_ADD;
            ctrl_next.wb_sel = WB_PC4;
            ctrl_next.pc_sel = PC_JALR;
            ctrl_next.illegal = 1'b0;
          end

          default: begin
            ctrl_next.illegal = 1'b1;
            ctrl_next.pc_sel = PC_PLUS4;
            ctrl_next.alu_op = ALU_INVALID;
          end
        endcase
      end

      OPCODE_LUI: begin
        ctrl_next.reg_write = 1'b1;
        ctrl_next.mem_write = 1'b0;
        ctrl_next.imm_sel = IMM_U;
        ctrl_next.alu_a_sel = ALU_A_ZERO;
        ctrl_next.alu_b_sel = ALU_B_IMM;
        ctrl_next.alu_op = ALU_COPY_B;
        ctrl_next.wb_sel = WB_ALU;
        ctrl_next.illegal = 1'b0;
      end

      OPCODE_AUIPC: begin
        // AUIPC adds the upper immediate to the instruction PC.
        ctrl_next.reg_write = 1'b1;
        ctrl_next.mem_write = 1'b0;
        ctrl_next.imm_sel = IMM_U;
        ctrl_next.alu_a_sel = ALU_A_PC;
        ctrl_next.alu_b_sel = ALU_B_IMM;
        ctrl_next.alu_op = ALU_ADD;
        ctrl_next.wb_sel = WB_ALU;
        ctrl_next.illegal = 1'b0;
      end

      OPCODE_MISC_MEM: begin
        // FENCE/FENCE.I are architectural NOPs for this in-order,
        // single-hart, no-cache teaching core.
        unique case (funct3)
          3'b000,
          3'b001: begin
            ctrl_next.illegal = 1'b0;
          end

          default: begin
            ctrl_next.illegal = 1'b1;
          end
        endcase
      end

      default: begin
        // Keep safe illegal defaults.
        ctrl_next.illegal = 1'b1;
        ctrl_next.pc_sel = PC_PLUS4;
        ctrl_next.alu_op = ALU_INVALID;
      end
    endcase
  end

  assign ctrl = ctrl_next;
endmodule
