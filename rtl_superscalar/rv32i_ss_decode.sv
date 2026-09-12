`timescale 1ns/1ps

// rv32i_ss_decode -- the OoO front-end decoder.
//
// Wrapper over rtl_p/rv32i_pipe_decode.sv. It translates decoded controls
// into op_class, fu_class, source selectors and trap/CSR metadata. Rename
// and execution handle physical dependencies outside this decoder.

module rv32i_ss_decode
  import fyp_cpu_pkg::*;
  import rv32i_pipeline_pkg::*;
  import rv32i_ss_pkg::arch_reg_t;
  import rv32i_ss_pkg::ooo_op_class_e;
  import rv32i_ss_pkg::ooo_fu_class_e;
  import rv32i_ss_pkg::ooo_src_sel_e;
  import rv32i_ss_pkg::OOO_OP_ALU;
  import rv32i_ss_pkg::OOO_OP_BRANCH;
  import rv32i_ss_pkg::OOO_OP_JUMP;
  import rv32i_ss_pkg::OOO_FU_ALU;
  import rv32i_ss_pkg::OOO_FU_MULDIV;
  import rv32i_ss_pkg::OOO_FU_LSU;
  import rv32i_ss_pkg::OOO_SRC_REG;
  import rv32i_ss_pkg::OOO_SRC_IMM;
  import rv32i_ss_pkg::OOO_SRC_PC;
  import rv32i_ss_pkg::OOO_SRC_ZERO;
  import rv32i_ss_pkg::csr_addr_t;
  import rv32i_ss_pkg::csr_zimm_t;
(
  input  logic [31:0]   instr,

  output arch_reg_t      rs1,
  output arch_reg_t      rs2,
  output arch_reg_t      rd,
  output logic           rd_we,
  output logic           illegal,

  output ooo_op_class_e  op_class,
  output ooo_fu_class_e  fu_class,
  output alu_op_e        alu_op,
  output muldiv_op_e     muldiv_op,
  output br_type_e       branch_op,
  output ooo_src_sel_e   src1_sel,
  output ooo_src_sel_e   src2_sel,
  output imm_sel_e       imm_sel,

  output logic           is_csr,
  output csr_op_e        csr_op,        // used for CSR serialization
  output csr_addr_t      csr_addr,
  output csr_zimm_t      csr_zimm,
  output logic           is_mem,        // load/store: routed through the LSQ path
  output trap_op_e       trap_op,
  output logic           is_mret,

  output logic           is_load,
  output logic           is_store,
  output mem_size_e      mem_size,
  output logic           mem_unsigned
);

  // Register operands come straight from the instruction (unconditional, as in
  // the pipeline decoder). Real dependencies are gated downstream by src*_sel.
  assign rs1 = arch_reg_t'(instr[19:15]);
  assign rs2 = arch_reg_t'(instr[24:20]);
  assign rd  = arch_reg_t'(instr[11:7]);

  // Pipeline decode (rtl_p module).
  pipe_ctrl_t ctrl;
  logic       csr_imm_form;

  rv32i_pipe_decode u_pipe_decode (
    .instr (instr),
    .ctrl  (ctrl)
  );

  // ---- straight-through fields ----
  assign rd_we     = ctrl.reg_write;
  assign illegal   = ctrl.illegal;
  assign alu_op    = ctrl.alu_op;
  assign muldiv_op = ctrl.muldiv_op;
  assign branch_op = ctrl.branch_op;
  assign imm_sel   = ctrl.imm_sel;
  assign csr_op    = ctrl.csr;
  assign csr_addr  = csr_addr_t'(instr[31:20]);
  assign csr_zimm  = csr_zimm_t'(instr[19:15]);
  assign trap_op   = ctrl.trap_op;
  assign is_csr    = (ctrl.csr != rv32i_pipeline_pkg::CSR_NONE);
  assign is_mret   = (ctrl.trap_op == rv32i_pipeline_pkg::TRAP_MRET);
  assign is_mem    = ctrl.mem_read | ctrl.mem_write;
  assign is_load   = ctrl.mem_read;
  assign is_store  = ctrl.mem_write;
  assign mem_size  = ctrl.mem_size;
  assign mem_unsigned = ctrl.mem_unsigned;
  // Continuous assigns (not always_*) keep Icarus from going whole-struct
  // sensitive on ctrl; enum ternaries need an explicit cast for Icarus.
  assign fu_class  = ooo_fu_class_e'(is_mem ? OOO_FU_LSU:
                              ctrl.is_muldiv ? OOO_FU_MULDIV : OOO_FU_ALU);

  assign csr_imm_form =
      (ctrl.csr == CSR_RWI) ||
      (ctrl.csr == CSR_RSI) ||
      (ctrl.csr == CSR_RCI);

  // op_class: branch / jump / alu (from the pipeline's branch_op + is_jump)
  assign op_class  = ooo_op_class_e'(
      (ctrl.branch_op != rv32i_pipeline_pkg::BR_NONE) ? OOO_OP_BRANCH :
      ctrl.is_jump                                    ? OOO_OP_JUMP   :
                                                        OOO_OP_ALU);

  // Operand source selects. CSR register forms consume rs1; immediate forms
  // consume the separately carried zimm. The encoded rs2 field is the CSR
  // address for every CSR instruction, so it must never create a dependency.
  assign src1_sel = ooo_src_sel_e'(
      is_csr                          ? (csr_imm_form ? OOO_SRC_ZERO : OOO_SRC_REG) :
      (ctrl.alu_a_sel == ALU_A_PC)   ? OOO_SRC_PC   :
      (ctrl.alu_a_sel == ALU_A_ZERO) ? OOO_SRC_ZERO :
                                       OOO_SRC_REG);
  assign src2_sel = ooo_src_sel_e'(
      is_csr                         ? OOO_SRC_ZERO :
      (ctrl.alu_b_sel == ALU_B_IMM) ? OOO_SRC_IMM  :
                                      OOO_SRC_REG);

endmodule
