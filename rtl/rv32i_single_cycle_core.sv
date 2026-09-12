`timescale 1ns/1ps

module rv32i_single_cycle_core
  import fyp_cpu_pkg::*;
(
  input  logic clk,
  input  logic rst_n,

  output word_t imem_addr,
  input  logic [31:0] imem_rdata,

  output logic dmem_we,
  output logic [3:0] dmem_be,
  output word_t dmem_addr,
  output word_t dmem_wdata,
  input  word_t dmem_rdata,

  // Verification observability: asserts when the current instruction is
  // ECALL or EBREAK. In this single-cycle core, fetch and retire occur in
  // the same cycle, so detecting from imem_rdata IS retire-based detection.
  // This core exposes a simulation halt request rather than a CSR trap vector.
  output logic sim_halt
);
  word_t pc_q;
  word_t pc_next;
  word_t imm;
  word_t rs1_data;
  word_t rs2_data;
  word_t alu_operand_a;
  word_t alu_operand_b;
  word_t alu_result;
  word_t wb_data;
  logic  alu_zero;
  logic  branch_taken;
  word_t load_data;
  logic [1:0] byte_off;
  logic [7:0] byte_raw;
  logic [15:0] half_raw;
  reg_addr_t rs1_addr;
  reg_addr_t rs2_addr;
  reg_addr_t rd_addr;
  decode_ctrl_t ctrl;

  assign imem_addr = pc_q;

  // ECALL = 32'h00000073 (opcode=SYSTEM, funct3=0, imm=0)
  // EBREAK = 32'h00100073 (opcode=SYSTEM, funct3=0, imm=1)
  assign sim_halt = (imem_rdata == 32'h00000073) ||
                    (imem_rdata == 32'h00100073);

  rv32i_decode u_decode (
    .instr    (imem_rdata),
    .rs1_addr (rs1_addr),
    .rs2_addr (rs2_addr),
    .rd_addr  (rd_addr),
    .ctrl     (ctrl)
  );

  rv32i_imm_gen u_imm_gen (
    .instr   (imem_rdata),
    .imm_sel (ctrl.imm_sel),
    .imm     (imm)
  );

  rv32i_regfile u_regfile (
    .clk      (clk),
    .rst_n    (rst_n),
    .write_en (ctrl.reg_write),
    .rs1_addr (rs1_addr),
    .rs2_addr (rs2_addr),
    .rd_addr  (rd_addr),
    .rd_data  (wb_data),
    .rs1_data (rs1_data),
    .rs2_data (rs2_data)
  );

  // ---------------------------------------------------------------------------
  // ALU INPUT side: operand muxes + store data/enable.
  //
  // This block deliberately does NOT read alu_result / alu_zero / dmem_rdata.
  // Keeping the ALU *consumers* out of the block that drives the ALU *inputs*
  // breaks a false combinational event cycle
  //   core.always_comb -> alu_operand_* -> u_alu -> alu_result/alu_zero
  //     -> core.always_comb (re-trigger) -> alu_operand_* -> ...
  // The data dependency is acyclic, so Verilator levelizes and settles it, but
  // Icarus/vvp is event-driven and never converges this cross-module loop: it
  // re-fires both blocks forever at t=0 (stable values), so time never advances
  // and the simulation hangs. Splitting input/output sides removes alu_result
  // and alu_zero from this block's sensitivity and breaks the cycle.
  // ---------------------------------------------------------------------------
  always_comb begin
    alu_operand_a = '0;
    alu_operand_b = '0;

    unique case (ctrl.alu_a_sel)
      ALU_A_RS1:  alu_operand_a = rs1_data;
      ALU_A_PC:   alu_operand_a = pc_q;
      ALU_A_ZERO: alu_operand_a = '0;
      default:    alu_operand_a = '0;
    endcase

    unique case (ctrl.alu_b_sel)
      ALU_B_RS2: alu_operand_b = rs2_data;
      ALU_B_IMM: alu_operand_b = imm;
      default:   alu_operand_b = '0;
    endcase
  end

  // ---------------------------------------------------------------------------
  // ALU OUTPUT side: consumes alu_result / alu_zero / dmem_rdata.
  // Separated from the operand muxes above so that block is not sensitive to
  // the ALU outputs (see note above). Drives a disjoint set of signals.
  // ---------------------------------------------------------------------------
  always_comb begin
    branch_taken = 1'b0;
    dmem_addr    = alu_result;
    wb_data      = '0;
    dmem_wdata = '0;
    dmem_be = 4'b0000;
    load_data = '0;
    byte_raw = 8'h00;
    byte_off = alu_result[1:0];
    half_raw = 16'h0000;
    half_raw = byte_off[1] ? dmem_rdata[31:16] : dmem_rdata[15:0];
    dmem_we    = ctrl.mem_write;

    // Branch comparison reuses the ALU result instead of standalone comparators:
    //   beq/bne   -> alu_op=SUB  -> alu_zero
    //   blt/bge   -> alu_op=SLT  -> alu_result[0] (1 = rs1 <s rs2)
    //   bltu/bgeu -> alu_op=SLTU -> alu_result[0] (1 = rs1 <u rs2)
    unique case (ctrl.branch_op)
      SC_BR_EQ:  branch_taken = (ctrl.pc_sel == PC_BRANCH) &&  alu_zero;
      SC_BR_NE:  branch_taken = (ctrl.pc_sel == PC_BRANCH) && !alu_zero;
      SC_BR_LT:  branch_taken = (ctrl.pc_sel == PC_BRANCH) &&  alu_result[0];
      SC_BR_GE:  branch_taken = (ctrl.pc_sel == PC_BRANCH) && !alu_result[0];
      SC_BR_LTU: branch_taken = (ctrl.pc_sel == PC_BRANCH) &&  alu_result[0];
      SC_BR_GEU: branch_taken = (ctrl.pc_sel == PC_BRANCH) && !alu_result[0];
      default:   branch_taken = 1'b0;
    endcase

    unique case (byte_off)
      2'b00: byte_raw = dmem_rdata[7:0];
      2'b01: byte_raw = dmem_rdata[15:8];
      2'b10: byte_raw = dmem_rdata[23:16];
      2'b11: byte_raw = dmem_rdata[31:24];
      default: byte_raw = 8'h00;
    endcase

    unique case (ctrl.mem_size)
      MEM_W: begin
        load_data = dmem_rdata;
      end
      MEM_H: begin
        load_data = ctrl.mem_unsigned ? {16'h0000, half_raw}
                      : {{16{half_raw[15]}}, half_raw};
      end
      MEM_B: begin
        load_data = ctrl.mem_unsigned ? {24'h0000, byte_raw}
                      : {{24{byte_raw[7]}}, byte_raw};
      end
      default: begin
        load_data = '0;
      end

    endcase

    if (ctrl.mem_write) begin
      unique case (ctrl.mem_size)
        MEM_W: begin
          dmem_wdata = rs2_data;
          dmem_be = 4'b1111;
        end
        MEM_H: begin
          if (byte_off[1]) begin
            dmem_wdata = {rs2_data[15:0], 16'h0000};
            dmem_be = 4'b1100;
          end else begin
            dmem_wdata = {16'h0000, rs2_data[15:0]};
            dmem_be = 4'b0011;
          end
        end

        MEM_B: begin
          unique case (byte_off)
            2'b00: begin
              dmem_wdata = {24'h0, rs2_data[7:0]};
              dmem_be = 4'b0001;
            end
            2'b01: begin
              dmem_wdata = {16'h0, rs2_data[7:0], 8'h0};
              dmem_be = 4'b0010;
            end
            2'b10: begin
              dmem_wdata = {8'h0, rs2_data[7:0], 16'h0};
              dmem_be = 4'b0100;
            end
            2'b11: begin
              dmem_wdata = {rs2_data[7:0], 24'h0};
              dmem_be = 4'b1000;
            end
          endcase
        end
        default: begin
          dmem_wdata = '0;
        end

      endcase
    end

    unique case (ctrl.wb_sel)
      WB_ALU :  wb_data = alu_result;
      WB_MEM :  wb_data = load_data;
      WB_PC4 :  wb_data = pc_q + 32'd4;
      WB_NONE : wb_data = '0;
      default : wb_data = '0;
    endcase
  end

  rv32i_alu u_alu (
    .operand_a (alu_operand_a),
    .operand_b (alu_operand_b),
    .alu_op    (ctrl.alu_op),
    .result    (alu_result),
    .zero      (alu_zero)
  );

  rv32i_pc_logic u_pc_logic (
    .pc_current   (pc_q),
    .imm          (imm),
    .rs1_data     (rs1_data),
    .branch_taken (branch_taken),
    .pc_sel       (ctrl.pc_sel),
    .pc_next      (pc_next)
  );

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pc_q <= '0;
    end else begin
      pc_q <= pc_next;
    end
  end
endmodule
