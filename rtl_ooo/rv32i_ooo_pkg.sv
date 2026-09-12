`timescale 1ns/1ps
package rv32i_ooo_pkg;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;

  parameter int OOO_XLEN         = 32;
  parameter int OOO_ARCH_REGS    = 32;
  parameter int OOO_PHYS_REGS    = 64;
  parameter int OOO_ROB_DEPTH    = 32;
  parameter int OOO_IQ_DEPTH     = 16;
  parameter int OOO_BRANCH_CKPTS = 4;

  localparam int OOO_ARCH_BITS = 5;
  localparam int OOO_PHYS_BITS = 6;
  localparam int OOO_ROB_BITS  = 5;
  localparam int OOO_IQ_BITS   = 4;
  localparam int OOO_CKPT_BITS = 2;

  typedef logic [OOO_XLEN-1:0]         word_t;
  typedef logic [OOO_ARCH_BITS-1:0]    arch_reg_t;
  typedef logic [OOO_PHYS_BITS-1:0]    phys_reg_t;
  typedef logic [OOO_ROB_BITS-1:0]     rob_idx_t;
  typedef logic [OOO_CKPT_BITS-1:0]    ckpt_idx_t;
  typedef logic [OOO_BRANCH_CKPTS-1:0] branch_mask_t;
  typedef logic [63:0]                 commit_order_t;
  typedef logic [63:0]                 rob_seq_t;
  typedef logic [11:0]                 csr_addr_t;
  typedef logic [4:0]                  csr_zimm_t;

  // M-mode synchronous trap cause codes used by this core (mcause values).
  parameter word_t OOO_CAUSE_IADDR_MISALIGN = 32'd0;
  parameter word_t OOO_CAUSE_ILLEGAL        = 32'd2;
  parameter word_t OOO_CAUSE_EBREAK         = 32'd3;
  parameter word_t OOO_CAUSE_LOAD_MISALIGN  = 32'd4;
  parameter word_t OOO_CAUSE_STORE_MISALIGN = 32'd6;
  parameter word_t OOO_CAUSE_ECALL_M        = 32'd11;

  typedef enum logic [1:0] {
    OOO_OP_ALU,
    OOO_OP_BRANCH,
    OOO_OP_JUMP
  } ooo_op_class_e;

  typedef enum logic [1:0] {
    OOO_FU_ALU,
    OOO_FU_MULDIV,
    OOO_FU_LSU
  } ooo_fu_class_e;

  typedef enum logic [1:0] {
    OOO_SRC_REG,
    OOO_SRC_IMM,
    OOO_SRC_PC,
    OOO_SRC_ZERO
  } ooo_src_sel_e;

  typedef struct packed {
    word_t         pc;
    rob_idx_t      rob_idx;
    rob_seq_t      rob_seq;
    ooo_op_class_e op_class;
    ooo_fu_class_e fu_class;
    alu_op_e       alu_op;
    br_type_e      branch_op;
    muldiv_op_e    muldiv_op;
    phys_reg_t     prs1;
    phys_reg_t     prs2;
    phys_reg_t     pdst;
    ooo_src_sel_e  src1_sel;
    ooo_src_sel_e  src2_sel;
    word_t         imm;
    logic          rd_wen;
    branch_mask_t  branch_mask;
    ckpt_idx_t     checkpoint_id;
    csr_op_e       csr_op;
    csr_addr_t     csr_addr;
    csr_zimm_t     csr_zimm;
    logic          is_load;
    logic          is_store;
    mem_size_e     mem_size;
    logic          mem_unsigned;
  } iq_entry_t;

  typedef struct packed {
    logic      valid;     // a completion is offered this cycle (= FU done)
    rob_idx_t  rob_idx;
    rob_seq_t  rob_seq;
    phys_reg_t pdst;
    logic      rd_wen;
    word_t     result;
    logic      trap_valid;
    word_t     trap_cause;
    word_t     trap_tval;
    logic      csr_we;
    word_t     csr_wdata;
  } completion_packet_t;

  // Decode-detected trap, carried frontend -> core -> ROB.
  typedef struct packed {
    logic  valid;     // a trap was decoded (ecall / ebreak / illegal)
    word_t cause;     // mcause code (2 illegal / 3 ebreak / 11 ecall)
    word_t tval;      // mtval
    logic  is_mret;   // mret = trap-return (NOT a trap; valid stays 0)
  } decoded_trap_t;

endpackage
