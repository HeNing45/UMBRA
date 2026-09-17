// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// Decoder for the 5-stage pipeline. Same structure as the single-cycle
// rv32i_decode but produces the richer pipe_ctrl_t (adds mem_read, mem_size,
// mem_unsigned, branch_op, is_jump, is_jalr, is_muldiv, muldiv_op, result_src).
//
// Default at the top of always_comb is the safe bubble: reg_write=0,
// mem_write=0, alu_op=ALU_INVALID, illegal=1.
module rv32i_pipe_decode
  import fyp_cpu_pkg::*;
  import rv32i_pipeline_pkg::*;
(
  input  logic [31:0] instr,
  output pipe_ctrl_t  ctrl
);
  logic [6:0]  opcode;
  logic [2:0]  funct3;
  logic [6:0]  funct7;
  pipe_ctrl_t  ctrl_next;

  assign opcode = instr[6:0];
  assign funct3 = instr[14:12];
  assign funct7 = instr[31:25];

  always_comb begin
    // -------- safe default = illegal bubble --------
    ctrl_next             = '0;
    ctrl_next.valid       = 1'b1;
    ctrl_next.sim_halt    = (instr == 32'h00000073) || (instr == 32'h00100073);
    ctrl_next.alu_op      = ALU_INVALID;
    ctrl_next.imm_sel     = IMM_NONE;
    ctrl_next.result_src  = RES_ALU;
    ctrl_next.branch_op   = rv32i_pipeline_pkg::BR_NONE;
    ctrl_next.muldiv_op   = MD_NONE;
    ctrl_next.csr         = CSR_NONE;
    ctrl_next.trap_op    = TRAP_NONE;
    ctrl_next.mem_size    = MEM_W;
    ctrl_next.illegal     = 1'b1;

    unique case (opcode)
      // ===========================================================
      // OP : R-type integer  +  M-extension (funct7 == 7'b0000001)
      // ===========================================================
      OPCODE_OP: begin
        ctrl_next.reg_write  = 1'b1;
        ctrl_next.alu_a_sel  = ALU_A_RS1;
        ctrl_next.alu_b_sel  = ALU_B_RS2;
        ctrl_next.imm_sel    = IMM_NONE;
        ctrl_next.illegal    = 1'b0;

        if (funct7 == 7'b0000001) begin
          // ---- M-extension routing ----
          ctrl_next.is_muldiv  = 1'b1;
          ctrl_next.result_src = RES_MULDIV;
          unique case (funct3)
            3'b000: ctrl_next.muldiv_op = MD_MUL;
            3'b001: ctrl_next.muldiv_op = MD_MULH;
            3'b010: ctrl_next.muldiv_op = MD_MULHSU;
            3'b011: ctrl_next.muldiv_op = MD_MULHU;
            3'b100: ctrl_next.muldiv_op = MD_DIV;
            3'b101: ctrl_next.muldiv_op = MD_DIVU;
            3'b110: ctrl_next.muldiv_op = MD_REM;
            3'b111: ctrl_next.muldiv_op = MD_REMU;
            default: begin
              ctrl_next.muldiv_op = MD_NONE;
              ctrl_next.illegal   = 1'b1;
              ctrl_next.reg_write = 1'b0;
            end
          endcase
          // alu_op is unused for muldiv but set to ADD so the ALU lint is happy.
          ctrl_next.alu_op = ALU_ADD;
        end else begin
          // ---- Plain R-type ----
          ctrl_next.result_src = RES_ALU;
          unique case ({funct7, funct3})
            {7'b0000000, 3'b000}: ctrl_next.alu_op = ALU_ADD;
            {7'b0100000, 3'b000}: ctrl_next.alu_op = ALU_SUB;
            {7'b0000000, 3'b001}: ctrl_next.alu_op = ALU_SLL;
            {7'b0000000, 3'b010}: ctrl_next.alu_op = ALU_SLT;
            {7'b0000000, 3'b011}: ctrl_next.alu_op = ALU_SLTU;
            {7'b0000000, 3'b100}: ctrl_next.alu_op = ALU_XOR;
            {7'b0000000, 3'b101}: ctrl_next.alu_op = ALU_SRL;
            {7'b0100000, 3'b101}: ctrl_next.alu_op = ALU_SRA;
            {7'b0000000, 3'b110}: ctrl_next.alu_op = ALU_OR;
            {7'b0000000, 3'b111}: ctrl_next.alu_op = ALU_AND;
            default: begin
              ctrl_next.alu_op    = ALU_INVALID;
              ctrl_next.illegal   = 1'b1;
              ctrl_next.reg_write = 1'b0;
            end
          endcase
        end
      end

      // ===========================================================
      // OP-IMM : I-type ALU immediates
      // ===========================================================
      OPCODE_OP_IMM: begin
        ctrl_next.reg_write  = 1'b1;
        ctrl_next.alu_a_sel  = ALU_A_RS1;
        ctrl_next.alu_b_sel  = ALU_B_IMM;
        ctrl_next.imm_sel    = IMM_I;
        ctrl_next.result_src = RES_ALU;
        ctrl_next.illegal    = 1'b0;
        unique case (funct3)
          3'b000: ctrl_next.alu_op = ALU_ADD;     // addi
          3'b010: ctrl_next.alu_op = ALU_SLT;     // slti
          3'b011: ctrl_next.alu_op = ALU_SLTU;    // sltiu
          3'b100: ctrl_next.alu_op = ALU_XOR;     // xori
          3'b110: ctrl_next.alu_op = ALU_OR;      // ori
          3'b111: ctrl_next.alu_op = ALU_AND;     // andi
          3'b001: begin                            // slli (funct7 must be 0)
            if (funct7 == 7'b0000000) begin
              ctrl_next.alu_op = ALU_SLL;
            end else begin
              ctrl_next.alu_op    = ALU_INVALID;
              ctrl_next.illegal   = 1'b1;
              ctrl_next.reg_write = 1'b0;
            end
          end
          3'b101: begin                            // srli / srai
            unique case (funct7)
              7'b0000000: ctrl_next.alu_op = ALU_SRL;
              7'b0100000: ctrl_next.alu_op = ALU_SRA;
              default: begin
                ctrl_next.alu_op    = ALU_INVALID;
                ctrl_next.illegal   = 1'b1;
                ctrl_next.reg_write = 1'b0;
              end
            endcase
          end
          default: begin
            ctrl_next.alu_op    = ALU_INVALID;
            ctrl_next.illegal   = 1'b1;
            ctrl_next.reg_write = 1'b0;
          end
        endcase
      end

      // ===========================================================
      // LOAD : lb/lh/lw/lbu/lhu
      // ===========================================================
      OPCODE_LOAD: begin
        ctrl_next.reg_write  = 1'b1;
        ctrl_next.mem_read   = 1'b1;
        ctrl_next.alu_a_sel  = ALU_A_RS1;
        ctrl_next.alu_b_sel  = ALU_B_IMM;
        ctrl_next.imm_sel    = IMM_I;
        ctrl_next.alu_op     = ALU_ADD;     // address calc
        ctrl_next.result_src = RES_MEM;
        ctrl_next.illegal    = 1'b0;
        unique case (funct3)
          3'b000: begin ctrl_next.mem_size = MEM_B; ctrl_next.mem_unsigned = 1'b0; end // lb
          3'b001: begin ctrl_next.mem_size = MEM_H; ctrl_next.mem_unsigned = 1'b0; end // lh
          3'b010: begin ctrl_next.mem_size = MEM_W; ctrl_next.mem_unsigned = 1'b0; end // lw
          3'b100: begin ctrl_next.mem_size = MEM_B; ctrl_next.mem_unsigned = 1'b1; end // lbu
          3'b101: begin ctrl_next.mem_size = MEM_H; ctrl_next.mem_unsigned = 1'b1; end // lhu
          default: begin
            ctrl_next.illegal   = 1'b1;
            ctrl_next.reg_write = 1'b0;
            ctrl_next.mem_read  = 1'b0;
            ctrl_next.mem_size  = MEM_W;
          end
        endcase
      end

      // ===========================================================
      // STORE : sb / sh / sw
      // ===========================================================
      OPCODE_STORE: begin
        ctrl_next.mem_write  = 1'b1;
        ctrl_next.alu_a_sel  = ALU_A_RS1;
        ctrl_next.alu_b_sel  = ALU_B_IMM;
        ctrl_next.imm_sel    = IMM_S;
        ctrl_next.alu_op     = ALU_ADD;     // address calc
        ctrl_next.result_src = RES_ALU;
        ctrl_next.illegal    = 1'b0;
        unique case (funct3)
          3'b000: ctrl_next.mem_size = MEM_B;  // sb
          3'b001: ctrl_next.mem_size = MEM_H;  // sh
          3'b010: ctrl_next.mem_size = MEM_W;  // sw
          default: begin
            ctrl_next.illegal   = 1'b1;
            ctrl_next.mem_write = 1'b0;
            ctrl_next.mem_size  = MEM_W;
          end
        endcase
      end

      // ===========================================================
      // BRANCH : beq / bne / blt / bge / bltu / bgeu
      // ===========================================================
      OPCODE_BRANCH: begin
        ctrl_next.alu_a_sel = ALU_A_RS1;
        ctrl_next.alu_b_sel = ALU_B_RS2;
        ctrl_next.imm_sel   = IMM_B;
        ctrl_next.alu_op    = ALU_ADD;      // unused; pc_target uses pc+imm
        ctrl_next.illegal   = 1'b0;
        unique case (funct3)
          3'b000: ctrl_next.branch_op = BR_BEQ;
          3'b001: ctrl_next.branch_op = BR_BNE;
          3'b100: ctrl_next.branch_op = BR_BLT;
          3'b101: ctrl_next.branch_op = BR_BGE;
          3'b110: ctrl_next.branch_op = BR_BLTU;
          3'b111: ctrl_next.branch_op = BR_BGEU;
          default: begin
            ctrl_next.branch_op = rv32i_pipeline_pkg::BR_NONE;
            ctrl_next.illegal   = 1'b1;
          end
        endcase
      end

      // ===========================================================
      // JAL
      // ===========================================================
      OPCODE_JAL: begin
        ctrl_next.reg_write  = 1'b1;
        ctrl_next.alu_a_sel  = ALU_A_PC;
        ctrl_next.alu_b_sel  = ALU_B_IMM;
        ctrl_next.imm_sel    = IMM_J;
        ctrl_next.alu_op     = ALU_ADD;
        ctrl_next.result_src = RES_PC4;
        ctrl_next.is_jump    = 1'b1;
        ctrl_next.is_jalr    = 1'b0;
        ctrl_next.illegal    = 1'b0;
      end

      // ===========================================================
      // JALR (funct3 must be 3'b000)
      // ===========================================================
      OPCODE_JALR: begin
        if (funct3 == 3'b000) begin
          ctrl_next.reg_write  = 1'b1;
          ctrl_next.alu_a_sel  = ALU_A_RS1;
          ctrl_next.alu_b_sel  = ALU_B_IMM;
          ctrl_next.imm_sel    = IMM_I;
          ctrl_next.alu_op     = ALU_ADD;
          ctrl_next.result_src = RES_PC4;
          ctrl_next.is_jump    = 1'b1;
          ctrl_next.is_jalr    = 1'b1;
          ctrl_next.illegal    = 1'b0;
        end else begin
          ctrl_next.illegal    = 1'b1;
          ctrl_next.reg_write  = 1'b0;
        end
      end

      // ===========================================================
      // LUI
      // ===========================================================
      OPCODE_LUI: begin
        ctrl_next.reg_write  = 1'b1;
        ctrl_next.alu_a_sel  = ALU_A_ZERO;
        ctrl_next.alu_b_sel  = ALU_B_IMM;
        ctrl_next.imm_sel    = IMM_U;
        ctrl_next.alu_op     = ALU_COPY_B;
        ctrl_next.result_src = RES_ALU;
        ctrl_next.illegal    = 1'b0;
      end

      // ===========================================================
      // AUIPC
      // ===========================================================
      OPCODE_AUIPC: begin
        ctrl_next.reg_write  = 1'b1;
        ctrl_next.alu_a_sel  = ALU_A_PC;
        ctrl_next.alu_b_sel  = ALU_B_IMM;
        ctrl_next.imm_sel    = IMM_U;
        ctrl_next.alu_op     = ALU_ADD;
        ctrl_next.result_src = RES_ALU;
        ctrl_next.illegal    = 1'b0;
      end

      // ===========================================================
      // MISC-MEM : fence / fence.i
      // ===========================================================
      OPCODE_MISC_MEM: begin
        // No caches, no prefetch buffer, single hart: FENCE/FENCE.I are legal
        // architectural no-ops in this teaching core. Future I-cache/frontend
        // work can hang an explicit flush request off this decode point.
        ctrl_next.alu_op     = ALU_ADD;
        ctrl_next.result_src = RES_ALU;
        unique case (funct3)
          3'b000,  // fence
          3'b001: begin // fence.i
            ctrl_next.illegal = 1'b0;
          end
          default: begin
            ctrl_next.illegal = 1'b1;
          end
        endcase
      end
      OPCODE_SYSTEM: begin
        ctrl_next.alu_op  = ALU_ADD;
        ctrl_next.imm_sel = IMM_NONE;
        unique case (funct3)
          3'b001: begin
            ctrl_next.reg_write  = 1'b1;
            ctrl_next.result_src = RES_CSR;
            ctrl_next.illegal    = 1'b0;
            ctrl_next.csr        = CSR_RW; // HAND-CODE: csrrw -> CSR_RW
          end
          3'b010: begin
            ctrl_next.reg_write  = 1'b1;
            ctrl_next.result_src = RES_CSR;
            ctrl_next.illegal    = 1'b0;
            ctrl_next.csr        = CSR_RS; // HAND-CODE: csrrs -> CSR_RS
          end
          3'b011: begin
            ctrl_next.reg_write  = 1'b1;
            ctrl_next.result_src = RES_CSR;
            ctrl_next.illegal    = 1'b0;
            ctrl_next.csr        = CSR_RC; // HAND-CODE: csrrc -> CSR_RC
          end
          3'b101: begin
            ctrl_next.reg_write  = 1'b1;
            ctrl_next.result_src = RES_CSR;
            ctrl_next.illegal    = 1'b0;
            ctrl_next.csr        = CSR_RWI; // HAND-CODE: csrrwi -> CSR_RWI
          end
          3'b110: begin
            ctrl_next.reg_write  = 1'b1;
            ctrl_next.result_src = RES_CSR;
            ctrl_next.illegal    = 1'b0;
            ctrl_next.csr        = CSR_RSI; // HAND-CODE: csrrsi -> CSR_RSI
          end
          3'b111: begin
            ctrl_next.reg_write  = 1'b1;
            ctrl_next.result_src = RES_CSR;
            ctrl_next.illegal    = 1'b0;
            ctrl_next.csr        = CSR_RCI; // HAND-CODE: csrrci -> CSR_RCI
          end
          3'b000: begin
            unique case (instr)
              32'h0000_0073: begin // ecall
                ctrl_next.illegal = 1'b0;
                ctrl_next.trap_op = TRAP_ECALL;
              end
              32'h0010_0073: begin // ebreak
                ctrl_next.illegal = 1'b0;
                ctrl_next.trap_op = TRAP_EBREAK;
              end
              32'h3020_0073: begin // mret
                ctrl_next.illegal = 1'b0;
                ctrl_next.trap_op = TRAP_MRET;
              end
              default: begin
                ctrl_next.illegal = 1'b1;
                ctrl_next.trap_op = TRAP_ILLEGAL;
              end
            endcase
          end
          default: begin
            ctrl_next.illegal = 1'b1;
          end
        endcase
      end
      default: begin
        // illegal bubble (defaults set above)
      end
    endcase

    // Any real decoded instruction that remains illegal must retire as an
    // illegal-instruction trap. Legal instructions clear illegal above.
    if (ctrl_next.illegal) begin
      ctrl_next.trap_op = TRAP_ILLEGAL;
    end
  end

  assign ctrl = ctrl_next;
endmodule
