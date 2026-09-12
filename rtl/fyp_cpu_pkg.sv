`timescale 1ns/1ps

package fyp_cpu_pkg;
  parameter int XLEN = 32;
  parameter int REG_ADDR_W = 5;

  typedef logic [XLEN-1:0] word_t;
  typedef logic [REG_ADDR_W-1:0] reg_addr_t;

  localparam logic [6:0] OPCODE_LUI    = 7'b0110111;
  localparam logic [6:0] OPCODE_AUIPC  = 7'b0010111;
  localparam logic [6:0] OPCODE_JAL    = 7'b1101111;
  localparam logic [6:0] OPCODE_JALR   = 7'b1100111;
  localparam logic [6:0] OPCODE_BRANCH = 7'b1100011;
  localparam logic [6:0] OPCODE_LOAD   = 7'b0000011;
  localparam logic [6:0] OPCODE_STORE  = 7'b0100011;
  localparam logic [6:0] OPCODE_MISC_MEM = 7'b0001111;
  localparam logic [6:0] OPCODE_OP_IMM = 7'b0010011;
  localparam logic [6:0] OPCODE_OP     = 7'b0110011;
  localparam logic [6:0] OPCODE_SYSTEM = 7'b1110011;

  typedef enum logic [3:0] {
    ALU_ADD,
    ALU_SUB,
    ALU_AND,
    ALU_OR,
    ALU_XOR,
    ALU_SLT,
    ALU_SLTU,
    ALU_SLL,
    ALU_SRL,
    ALU_SRA,
    ALU_COPY_B,
    ALU_INVALID
  } alu_op_e;

  typedef enum logic [2:0] {
    IMM_NONE,
    IMM_I,
    IMM_S,
    IMM_B,
    IMM_U,
    IMM_J
  } imm_sel_e;

  typedef enum logic [1:0] {
    ALU_A_RS1,
    ALU_A_PC,
    ALU_A_ZERO
  } alu_a_sel_e;

  typedef enum logic [1:0] {
    ALU_B_RS2,
    ALU_B_IMM
  } alu_b_sel_e;

  typedef enum logic [1:0] {
    WB_ALU,
    WB_MEM,
    WB_PC4,
    WB_NONE
  } wb_sel_e;

  typedef enum logic [2:0] {
    PC_PLUS4,
    PC_BRANCH,
    PC_JAL,
    PC_JALR,
    PC_HOLD
  } pc_sel_e;

  typedef enum logic [2:0] {
    SC_BR_NONE,
    SC_BR_EQ,
    SC_BR_NE,
    SC_BR_LT,
    SC_BR_GE,
    SC_BR_LTU,
    SC_BR_GEU
  } branch_op_e;


  // Shared memory-access size for the single-cycle and pipelined cores.
  // MEM_W has value zero for reset/bubble initialization; memory side effects
  // are separately qualified by the control packet's enables.
  typedef enum logic [1:0] {
    MEM_W,    // 0 - word  (default; reset-to-bubble safe)
    MEM_H,    // 1 - half
    MEM_B,    // 2 - byte
    MEM_NONE  // 3 - no memory op (pipeline uses for non-load/store ctrl)
  } mem_size_e;

  typedef struct packed {
    logic       reg_write;
    logic       mem_write;
    alu_op_e    alu_op;
    imm_sel_e    imm_sel;
    alu_a_sel_e  alu_a_sel;
    alu_b_sel_e  alu_b_sel;
    wb_sel_e     wb_sel;
    pc_sel_e     pc_sel;
    branch_op_e  branch_op;
    logic       illegal;
    mem_size_e   mem_size;
    logic       mem_unsigned;
  } decode_ctrl_t;
endpackage
