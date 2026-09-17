// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// rv32i_ooo_core - scalar out-of-order RV32IM core.
//
// rename + free-list + ROB + IQ + PRF + per-FU execute (ALU + multi-cycle
// muldiv + head-only LSU) + a single registered CDB/writeback lane. Branches
// speculate with checkpoint recovery; jumps serialize. Precise exceptions
// include decode traps (ecall/ebreak/illegal), mret, and execute-detected
// misalignment (causes 0/4/6, mtval = faulting address/target). These events are
// carried to the ROB head and TAKEN via the registered trap_q full flush; CSRs
// serialize with read-modify-write at commit. Memory uses head-gated loads
// and commit-time store writes over request/response dmem ports; this core
// has no load/store queue.

module rv32i_ooo_core
  import rv32i_ooo_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;
(
    input  logic          clk,
    input  logic          rst_n,

    input  logic          decoded_valid,
    output logic          decoded_ready,          // backend can accept the decoded packet
    input  word_t         decoded_pc,
    input  word_t         decoded_instr,
    input  arch_reg_t     decoded_rs1,
    input  arch_reg_t     decoded_rs2,
    input  arch_reg_t     decoded_rd,
    input  logic          decoded_rd_we,
    input  logic          decoded_needs_checkpoint,
    input  ooo_op_class_e decoded_op_class,
    input  ooo_fu_class_e decoded_fu_class,
    input  alu_op_e       decoded_alu_op,
    input  muldiv_op_e    decoded_muldiv_op,
    input  br_type_e      decoded_branch_op,
    input  ooo_src_sel_e  decoded_src1_sel,
    input  ooo_src_sel_e  decoded_src2_sel,
    input  word_t         decoded_imm,
    input  decoded_trap_t decoded_trap,   // decode-detected trap from the frontend
    input  csr_op_e       decoded_csr_op,
    input  csr_addr_t     decoded_csr_addr,
    input  csr_zimm_t     decoded_csr_zimm,

    // ---- branch/jump redirect back to the frontend ----
    output logic          redirect_valid,
    output word_t         redirect_target,

    // ---- architectural commit-trace channel ----
    output logic          commit_fire,
    output commit_order_t commit_order,
    output word_t         commit_pc,
    output word_t         commit_inst,
    output arch_reg_t     commit_rd,
    output logic          commit_rd_wen,
    output word_t         commit_wdata,

    // Load and store
    input logic           decoded_is_load,
    input logic           decoded_is_store,
    input mem_size_e      decoded_mem_size,
    input logic           decoded_mem_unsigned,

    //To memory
    output logic  dmem_valid,
    output logic  dmem_we,
    output logic  [3:0] dmem_be,
    output word_t dmem_addr,
    output word_t dmem_wdata,
    input  logic  dmem_ready,
    input  logic  dmem_rvalid,
    input  word_t dmem_rdata
);

  // ============================= internal nets =============================

  // ---- dispatch / rename / free-list ----
  logic       preg_alloc_req;
  logic       preg_avail;
  phys_reg_t  preg_alloc_reg;
  logic       rename_decoded_ready;
  logic       rename_valid;
  phys_reg_t  rename_prs1, rename_prs2, rename_pdst, rename_stale_pdst;
  arch_reg_t  rename_rd;
  logic       rename_rd_we;
  branch_mask_t rename_branch_mask;
  logic       rename_checkpoint_valid;
  ckpt_idx_t  rename_checkpoint_id;
  logic [OOO_PHYS_REGS-1:0] rename_branch_recover_alloc_list;
  logic       backend_decoded_valid;
  logic       rob_alloc_fire;
  iq_entry_t  dispatch_entry;        // shared dispatch entry: ROB + IQ

  // ---- ROB / commit ----
  logic       rob_alloc_ready;
  rob_idx_t   rob_alloc_idx;
  rob_seq_t   rob_alloc_seq;
  rob_idx_t   rob_head_idx;           // oldest in-flight; CDB + IQ age reference
  logic       rob_head_valid;
  logic       rob_commit_valid;
  logic       rob_commit_rd_we;
  arch_reg_t  rob_commit_rd;
  phys_reg_t  rob_commit_pdst;
  phys_reg_t  rob_commit_stale_pdst;
  logic          commit_ready;
  word_t         rob_commit_pc;
  word_t         rob_commit_instr;
  word_t         rob_commit_result;
  commit_order_t rob_commit_order;
  logic          rob_commit_is_csr;
  logic          rob_commit_is_store;
  logic          rob_commit_csr_we;
  csr_addr_t     rob_commit_csr_addr;
  word_t         rob_commit_csr_wdata;


  // ---- decode-detected trap (carried to ROB, taken when oldest and complete) ----
  logic       rob_alloc_trap_valid;
  word_t      rob_alloc_trap_cause;
  word_t      rob_alloc_trap_tval;
  logic       rob_alloc_is_mret;
  logic       rob_alloc_is_csr;
  logic       rob_alloc_is_store;

  // ---- IQ issue ----
  logic                     issue_valid;
  iq_entry_t                issue_entry;
  logic                     iq_alloc_ready;
  logic                     issue_accept;
  logic [OOO_PHYS_REGS-1:0] ready_vec;
  logic                     alu_fu_ready;
  logic                     muldiv_fu_ready;
  logic                     lsu_fu_ready;

  // ---- issued-uop unpack + classification ----
  logic       issue_fire;
  word_t      issue_pc;
  word_t      issue_imm;
  phys_reg_t  issue_prs1;
  phys_reg_t  issue_prs2;
  phys_reg_t  issue_pdst;
  ooo_src_sel_e issue_src1_sel;
  ooo_src_sel_e issue_src2_sel;
  ooo_op_class_e issue_op_class;
  ooo_fu_class_e issue_fu_class;
  br_type_e   issue_branch_op;
  alu_op_e    issue_alu_op;
  muldiv_op_e issue_muldiv_op;
  logic       issue_rd_wen;
  logic       issue_is_branch;
  logic       issue_is_jump;
  logic       issue_is_control;
  logic       issue_is_alu;
  logic       issue_is_muldiv;
  logic       issue_is_lsu;
  logic       issue_is_csr;
  csr_addr_t  issue_csr_addr;
  csr_zimm_t  issue_csr_zimm;
  csr_op_e    issue_csr_op;

  // ---- ALU / muldiv functional units ----
  word_t      prf_rdata1, prf_rdata2;
  word_t      operand_a, operand_b;
  word_t      alu_result;
  word_t      exec_result;
  logic       alu_issue_fire;
  logic       lsu_issue_fire;
  logic       store_commit_fire;
  logic       muldiv_start;
  logic       muldiv_busy;
  completion_packet_t alu_complete;
  completion_packet_t muldiv_complete;
  completion_packet_t lsu_complete;
  rob_idx_t muldiv_rob_idx_q;   // in-flight muldiv identity for ring-distance recovery

  // ---- store payload side array (written at LSU execute, consumed at commit) ----
  word_t      store_addr_q  [OOO_ROB_DEPTH];
  word_t      store_wdata_q [OOO_ROB_DEPTH];
  logic [3:0] store_be_q    [OOO_ROB_DEPTH];
  word_t      store_wdata_next;
  logic [3:0] store_be_next;
  integer     store_i;

  // ---- CDB / writeback (single registered lane) ----
  logic      cdb_fire;
  logic      rob_wb_accept;
  logic      cdb_grant_alu;
  logic      cdb_grant_muldiv;
  logic      cdb_grant_lsu;
  logic      cdb_valid_q;
  rob_idx_t  cdb_rob_idx_q;
  rob_seq_t  cdb_rob_seq_q;
  phys_reg_t cdb_pdst_q;
  logic      cdb_rd_wen_q;
  word_t     cdb_result_q;
  logic      cdb_csr_we_q;
  word_t     cdb_csr_wdata_q;
  logic      cdb_trap_valid_q;
  word_t     cdb_trap_cause_q;
  word_t     cdb_trap_tval_q;

  // ---- jump serialization (branches speculate) ----
  logic       jump_alloc_fire;
  logic       jump_inflight_q;

  // csr serialization
  logic       csr_alloc_fire;
  logic       csr_inflight_q;
  word_t      csr_rdata;
  word_t      csr_src;
  word_t      csr_wdata_exec;
  logic       csr_we_exec;
  word_t      csr_mtvec;
  word_t      csr_mepc;
  // ---- branch resolve / recovery / redirect ----
  logic       branch_taken;
  logic       branch_resolve_valid;
  ckpt_idx_t  branch_resolve_id;
  logic       branch_decide_recover;
  logic       branch_recover_req;
  ckpt_idx_t  branch_recover_id;
  logic       muldiv_kill;
  logic       control_resolve_fire;
  logic       jump_resolve_fire;
  logic       csr_resolve_fire;
  word_t      fallthrough_pc;
  // registered recovery: decide at cycle N, drive all consumers from the flop at N+1
  logic         recover_q_valid;
  ckpt_idx_t    recover_q_ckpt_id;
  rob_idx_t     recover_q_rob_idx;
  word_t        recover_q_target;     // mispredict redirect target, latched at N

  // registered trap taken -- the trap-side mirror of branch_decide_recover ->
  // recover_q_valid: a combinational decision at the commit boundary latches one
  // registered event (trap_q_valid) that next cycle drives the full flush +
  // CSR write + mtvec/mepc redirect. trap_latch_fire needs the !trap_q_valid
  // guard because the ROB head still presents the same trap for the cycle while
  // the registered flush takes effect (else mstatus would be pushed twice).
  logic         trap_commit_event;
  logic         trap_latch_fire;
  logic         trap_q_valid;
  logic         trap_q_is_mret;
  word_t        trap_q_pc;       // mepc for trap (the faulting/return PC)
  word_t        trap_q_cause;
  word_t        trap_q_tval;
  word_t        trap_q_target;   // mtvec for trap, mepc for mret

  logic         commit_trap_valid;
  word_t        commit_trap_cause;
  word_t        commit_trap_tval;
  logic         commit_is_mret;

  word_t        agen_addr;   // AGU output for the issuing mem op (rs1+imm via the shared ALU adder)
  word_t        control_target;
  logic         control_target_misalign;
  logic         load_misalign;
  logic         store_misalign;
  logic         lsu_misalign;
  word_t        exec_trap_cause;
  word_t        exec_trap_tval;
  logic         exec_trap_valid;

  // ---------------- dispatch / jump-serialization gate ----------------
  // Branches speculate freely; jumps serialize through jump_inflight_q.
  // jump_alloc_fire is driven once, in the resolve/control-signal block below.
  assign backend_decoded_valid = decoded_valid & ~jump_inflight_q & ~branch_recover_req
                                  & ~csr_inflight_q & ~trap_q_valid;
  assign decoded_ready         = rename_decoded_ready & ~jump_inflight_q
                                  & ~branch_recover_req & ~csr_inflight_q & ~trap_q_valid;
  assign rob_alloc_fire        = rename_valid & rob_alloc_ready & iq_alloc_ready &
                                 ~jump_inflight_q & ~branch_recover_req & ~csr_inflight_q
                                 & ~trap_q_valid;

  //temporal mem handshake
  assign agen_addr = exec_result;
  assign store_commit_fire = commit_fire && rob_commit_is_store;
  assign dmem_valid = (lsu_issue_fire && issue_entry.is_load && !load_misalign) || store_commit_fire;
  assign dmem_we    = store_commit_fire;
  assign dmem_be    = store_commit_fire ? store_be_q[rob_head_idx] : 4'b0000;
  assign dmem_addr  = store_commit_fire ? store_addr_q[rob_head_idx] : agen_addr;
  assign dmem_wdata = store_commit_fire ? store_wdata_q[rob_head_idx] : '0;

  // ---------------- dispatch entry assembly (drives ROB + IQ) ----------------
  always_comb begin
    dispatch_entry           = '0;
    dispatch_entry.pc        = decoded_pc;
    dispatch_entry.rob_idx   = rob_alloc_idx;
    dispatch_entry.rob_seq   = rob_alloc_seq;
    dispatch_entry.op_class  = decoded_op_class;
    dispatch_entry.fu_class  = decoded_fu_class;
    dispatch_entry.alu_op    = decoded_alu_op;
    dispatch_entry.branch_op = decoded_branch_op;
    dispatch_entry.branch_mask = rename_branch_mask;
    dispatch_entry.checkpoint_id = rename_checkpoint_valid ? rename_checkpoint_id : '0;
    dispatch_entry.muldiv_op = decoded_muldiv_op;
    dispatch_entry.prs1      = rename_prs1;        // names come from rename
    dispatch_entry.prs2      = rename_prs2;
    dispatch_entry.pdst      = rename_pdst;
    dispatch_entry.src1_sel  = decoded_src1_sel;
    dispatch_entry.src2_sel  = decoded_src2_sel;
    dispatch_entry.imm       = decoded_imm;
    dispatch_entry.rd_wen    = rename_rd_we;
    dispatch_entry.csr_op    = decoded_csr_op;
    dispatch_entry.csr_addr  = decoded_csr_addr;
    dispatch_entry.csr_zimm  = decoded_csr_zimm;
    dispatch_entry.is_load   = decoded_is_load;
    dispatch_entry.is_store  = decoded_is_store;
    dispatch_entry.mem_size  = decoded_mem_size;
    dispatch_entry.mem_unsigned  = decoded_mem_unsigned;
  end

  // ---------------- decode-detected trap (from rv32i_ooo_decode via frontend) ----------------
  // The decoder tags ecall/ebreak/illegal/mret and the frontend maps them to the
  // direct-cause form; unpack here into the ROB allocation trap ports.
  // Misalignment instead arrives with an execute completion. All traps wait
  // for the oldest-complete ROB boundary before the core takes them.
  assign rob_alloc_trap_valid = decoded_trap.valid;
  assign rob_alloc_trap_cause = decoded_trap.cause;
  assign rob_alloc_trap_tval  = decoded_trap.tval;
  assign rob_alloc_is_mret    = decoded_trap.is_mret;
  assign rob_alloc_is_csr     = (decoded_csr_op != rv32i_pipeline_pkg::CSR_NONE);
  assign rob_alloc_is_store   = decoded_is_store;

  // ================= commit-fire and trace channel ========================
  // Retirement side effects (committed map, stale free, commit_order) must be
  // gated by commit_fire, not raw commit_valid. A held head or recovery cycle
  // must not retire architectural state without the ROB actually advancing.
  // Traps and mret suppress ordinary commit and use the registered flush path.
  assign commit_ready = ~(commit_trap_valid || commit_is_mret);

  // Branch recovery has priority inside the ROB and prevents the head from
  // advancing that cycle. Keep all retirement side effects aligned with the
  // actual ROB advance.
  assign commit_fire  = rob_commit_valid & commit_ready & ~branch_recover_req;

  // Commit-trace output taps expose the retiring ROB entry.
  assign commit_pc     = rob_commit_pc;
  assign commit_inst   = rob_commit_instr;
  assign commit_rd     = rob_commit_rd;
  assign commit_rd_wen = rob_commit_rd_we;
  assign commit_wdata  = rob_commit_result;
  assign commit_order  = rob_commit_order;

  // ================= RENAME =================
  rv32i_ooo_rename u_rename (
    .clk                       (clk),
    .rst_n                     (rst_n),

    .decoded_valid             (backend_decoded_valid),
    .decoded_ready             (rename_decoded_ready),
    .decoded_rs1               (decoded_rs1),
    .decoded_rs2               (decoded_rs2),
    .decoded_rd                (decoded_rd),
    .decoded_rd_we             (decoded_rd_we),
    .decoded_needs_checkpoint  (decoded_needs_checkpoint),

    .preg_alloc_req                 (preg_alloc_req),
    .preg_avail               (preg_avail),
    .preg_alloc_reg             (preg_alloc_reg),

    .rename_valid              (rename_valid),
    .rename_ready              (rob_alloc_ready & iq_alloc_ready),
    .rename_prs1               (rename_prs1),
    .rename_prs2               (rename_prs2),
    .rename_pdst               (rename_pdst),
    .rename_stale_pdst         (rename_stale_pdst),
    .rename_rd                 (rename_rd),
    .rename_rd_we              (rename_rd_we),

    // ---- retirement gated by commit_fire (not raw commit_valid) ----
    .commit_fire              (commit_fire),
    .commit_rd_we              (rob_commit_rd_we),
    .commit_rd                 (rob_commit_rd),
    .commit_pdst               (rob_commit_pdst),
    .trap_flush         (trap_q_valid),
    .branch_recover_req        (branch_recover_req),
    .branch_recover_id         (branch_recover_id),
    .branch_resolve_valid      (branch_resolve_valid),
    .branch_resolve_id         (branch_resolve_id),

    // ---- recovery and checkpoint outputs ----
    .branch_recover_alloc_list (rename_branch_recover_alloc_list),
    .rename_branch_mask        (rename_branch_mask),
    .rename_checkpoint_valid   (rename_checkpoint_valid),
    .rename_checkpoint_id      (rename_checkpoint_id)
  );

  // ================= FREE LIST =================
  rv32i_ooo_free_list u_free_list (
    .clk                       (clk),
    .rst_n                     (rst_n),

    .preg_alloc_req                 (preg_alloc_req),
    .preg_avail               (preg_avail),
    .preg_alloc_reg             (preg_alloc_reg),

    // ---- retirement gated by commit_fire ----
    .commit_fire              (commit_fire),
    .commit_rd_we              (rob_commit_rd_we),
    .commit_pdst               (rob_commit_pdst),
    .commit_stale_pdst         (rob_commit_stale_pdst),
    .branch_recover_req        (branch_recover_req),
    .branch_recover_alloc_list (rename_branch_recover_alloc_list),
    .trap_flush         (trap_q_valid)
  );

  // ================= ROB =================
  rv32i_ooo_rob u_rob (
    .clk                       (clk),
    .rst_n                     (rst_n),

    .rob_alloc_valid            (rename_valid & iq_alloc_ready),
    .rob_alloc_ready            (rob_alloc_ready),
    .rob_alloc_pc               (decoded_pc),
    .rob_alloc_instr            (decoded_instr),
    .rob_alloc_rd_we            (rename_rd_we),
    .rob_alloc_rd               (rename_rd),
    .rob_alloc_pdst             (rename_pdst),
    .rob_alloc_stale_pdst       (rename_stale_pdst),
    .rob_alloc_trap_valid       (rob_alloc_trap_valid),
    .rob_alloc_trap_cause       (rob_alloc_trap_cause),
    .rob_alloc_trap_tval        (rob_alloc_trap_tval),
    .rob_alloc_is_mret          (rob_alloc_is_mret),
    .rob_alloc_is_store         (rob_alloc_is_store),
    .rob_alloc_idx          (rob_alloc_idx),
    .rob_alloc_seq          (rob_alloc_seq),

    .rob_head_idx              (rob_head_idx),
    .rob_head_valid            (rob_head_valid),
    .rob_head_seq              (),   // issue identity is carried by the selected IQ entry
    .rob_head_done             (),
    .rob_alloc_is_csr         (rob_alloc_is_csr),
    .rob_alloc_csr_addr       (decoded_csr_addr),
    // ---- writeback from the registered CDB beat ----
      .wb_valid                  (cdb_fire),
      .wb_rob_idx                (cdb_rob_idx_q),
      .wb_rob_seq                (cdb_rob_seq_q),
      .wb_result                 (cdb_result_q),
      .wb_csr_we                 (cdb_csr_we_q),
      .wb_csr_wdata              (cdb_csr_wdata_q),
      .wb_accept                 (rob_wb_accept),
      .wb_trap_valid             (cdb_trap_valid_q),
      .wb_trap_cause             (cdb_trap_cause_q),
      .wb_trap_tval              (cdb_trap_tval_q),

    // ---- commit acceptance and retirement taps ----
    .commit_ready              (commit_ready),
    .commit_valid              (rob_commit_valid),       // ROB "head valid&done"; feeds commit_fire
    .commit_rd_we              (rob_commit_rd_we),
    .commit_rd                 (rob_commit_rd),
    .commit_pdst               (rob_commit_pdst),
    .commit_stale_pdst         (rob_commit_stale_pdst),
    .commit_is_csr             (rob_commit_is_csr),
    .commit_is_store           (rob_commit_is_store),
    .commit_csr_we             (rob_commit_csr_we),
    .commit_csr_addr           (rob_commit_csr_addr),
    .commit_csr_wdata          (rob_commit_csr_wdata),

    .trap_flush                     (trap_q_valid),
    .branch_recover_req        (branch_recover_req),
    .recover_rob_idx    (recover_q_rob_idx),

    // ---- commit-trace taps ----
    .commit_pc                 (rob_commit_pc),
    .commit_instr              (rob_commit_instr),
    .commit_result             (rob_commit_result),
    .commit_order              (rob_commit_order),

    // ---- completed-head trap metadata consumed by the precise-trap logic ----
    .commit_trap_valid         (commit_trap_valid),
    .commit_trap_cause         (commit_trap_cause),
    .commit_trap_tval          (commit_trap_tval),
    .commit_is_mret            (commit_is_mret)
  );

  // ================= ISSUE QUEUE =================
  rv32i_ooo_iq u_iq (
    .clk          (clk),
    .rst_n        (rst_n),
    .trap_flush        (trap_q_valid),     // full IQ flush on a taken trap/mret
    .branch_recover_req(branch_recover_req),
    .recover_rob_idx (recover_q_rob_idx),

    .alloc_valid  (rename_valid & rob_alloc_ready),
    .alloc_entry  (dispatch_entry),
    .alloc_ready  (iq_alloc_ready),
    .ready_vec    (ready_vec),
    .rob_head_idx (rob_head_idx),
    .issue_valid  (issue_valid),
    .issue_entry  (issue_entry),
    .issue_ready  (issue_accept),
    .alu_fu_ready(alu_fu_ready),
    .muldiv_fu_ready(muldiv_fu_ready),
    .lsu_fu_ready  (lsu_fu_ready)
  );

  // ================= PER-FU ISSUE + COMPLETION ARBITER ======================
  // The IQ selects the oldest ready uop whose FU can accept it; commit still
  // retires in ROB-head order. FU completions are held until the single CDB
  // writeback lane grants one packet, then the ROB validates {rob_idx, rob_seq}.

  // Never issue or execute during a recovery cycle.
  // Dispatch is already gated by ~branch_recover_req; gating issue the same way
  // keeps a wrong-path uop from executing in the registered recovery bubble.
  // A trap full-flush also blocks issue from the IQ being cleared.
  assign issue_accept = ~(branch_recover_req | trap_q_valid);
  assign alu_fu_ready = !alu_complete.valid;
  assign muldiv_fu_ready = !muldiv_busy && !muldiv_complete.valid;
  assign lsu_fu_ready  = !lsu_complete.valid;
  assign issue_fire = issue_valid & issue_accept;
  assign cdb_fire  = cdb_valid_q;

  // ---- unpack the issued uop ----
  assign issue_pc       = issue_entry.pc;
  assign issue_imm      = issue_entry.imm;
  assign issue_prs1     = issue_entry.prs1;
  assign issue_prs2     = issue_entry.prs2;
  assign issue_pdst     = issue_entry.pdst;
  assign issue_src1_sel = issue_entry.src1_sel;
  assign issue_src2_sel = issue_entry.src2_sel;
  assign issue_op_class = issue_entry.op_class;
  assign issue_fu_class = issue_entry.fu_class;
  assign issue_branch_op = issue_entry.branch_op;
  assign issue_alu_op   = issue_entry.alu_op;
  assign issue_muldiv_op  = issue_entry.muldiv_op;
  assign issue_rd_wen   = issue_entry.rd_wen;
  assign issue_csr_addr = issue_entry.csr_addr;
  assign issue_csr_zimm = issue_entry.csr_zimm;
  assign issue_csr_op   = issue_entry.csr_op;

  // ---- classify + per-FU fire / control resolve / branch-recovery decision ----
  assign issue_is_branch      = (issue_op_class == OOO_OP_BRANCH);
  assign issue_is_jump        = (issue_op_class == OOO_OP_JUMP);
  assign issue_is_control     = issue_is_branch | issue_is_jump;
  assign issue_is_alu         = (issue_fu_class == OOO_FU_ALU);
  assign issue_is_muldiv      = (issue_fu_class == OOO_FU_MULDIV);
  assign issue_is_lsu         = (issue_fu_class == OOO_FU_LSU);
  assign issue_is_csr         = (issue_entry.csr_op != rv32i_pipeline_pkg::CSR_NONE);
  assign alu_issue_fire        = issue_fire & issue_is_alu;
  assign muldiv_start         = issue_fire & issue_is_muldiv;
  assign lsu_issue_fire        = issue_fire & issue_is_lsu;
  assign control_resolve_fire = issue_fire & issue_is_control;
  assign jump_resolve_fire    = issue_fire & issue_is_jump;
  assign jump_alloc_fire      = rob_alloc_fire &&
                                (decoded_op_class == OOO_OP_JUMP);
  assign csr_resolve_fire     = issue_fire & issue_is_csr;
  assign csr_alloc_fire       = rob_alloc_fire && rob_alloc_is_csr;
  assign branch_resolve_valid = issue_fire & issue_is_branch & ~branch_taken;
  assign branch_decide_recover = issue_fire & issue_is_branch
                                & branch_taken & !control_target_misalign;
  assign branch_recover_req   = recover_q_valid;
  assign branch_resolve_id    = issue_entry.checkpoint_id;
  assign branch_recover_id    = recover_q_ckpt_id;
  // Kill the in-flight muldiv iff it is younger than the recovering branch
  // by ROB ring distance (same predicate as the IQ + ROB rollback). muldiv_busy
  // gates out the idle case; an older muldiv (age <= branch age) is spared.
  // A taken trap squashes everything younger than the head, so every
  // in-flight muldiv (necessarily younger than the trapping head) is killed too,
  // not just younger-than-a-recovering-branch.
  assign muldiv_kill                 = (branch_recover_req && muldiv_busy &&
                                ((muldiv_rob_idx_q - rob_head_idx) >
                                 (recover_q_rob_idx - rob_head_idx)))
                                || (trap_q_valid && muldiv_busy);
  assign fallthrough_pc      = issue_pc + 32'd4;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      recover_q_valid    <= 1'b0;
      recover_q_ckpt_id  <= '0;
      recover_q_rob_idx  <= '0;
      recover_q_target   <= '0;
    end else begin
      recover_q_valid <= branch_decide_recover;
      if (branch_decide_recover) begin
        recover_q_ckpt_id  <= issue_entry.checkpoint_id;
        recover_q_rob_idx  <= issue_entry.rob_idx;
        recover_q_target   <= issue_pc + issue_imm;
      end
    end
  end

  // Combinational decision at the commit boundary; edge-detected so it latches
  // exactly one registered event even though the head holds the trap one more
  // cycle while the flush is in flight.
  assign trap_commit_event = commit_trap_valid || commit_is_mret;
  assign trap_latch_fire   = trap_commit_event && !trap_q_valid;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      trap_q_valid   <= 1'b0;
      trap_q_is_mret <= 1'b0;
      trap_q_pc      <= '0;
      trap_q_cause   <= '0;
      trap_q_tval    <= '0;
      trap_q_target  <= '0;
    end else begin
      trap_q_valid <= trap_latch_fire;

      if (trap_latch_fire) begin
        trap_q_is_mret <= commit_is_mret;
        trap_q_pc      <= rob_commit_pc;
        trap_q_cause   <= commit_trap_cause;
        trap_q_tval    <= commit_trap_tval;

        if (commit_is_mret) begin
          trap_q_target <= csr_mepc;
        end else begin
          trap_q_target <= {csr_mtvec[31:2], 2'b00};
        end
      end
    end
  end

  // Track the in-flight muldiv's rob_idx so the muldiv_kill predicate
  // above can compare ages. Captured at start, cleared on muldiv_kill or CDB grant.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      muldiv_rob_idx_q <= '0;
    end else if (muldiv_kill || cdb_grant_muldiv) begin
      muldiv_rob_idx_q <= '0;
    end else if (muldiv_start) begin
      muldiv_rob_idx_q <= issue_entry.rob_idx;
    end
  end


  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      jump_inflight_q <= 1'b0;
    end else if (branch_recover_req || trap_q_valid) begin
      // A mispredict recovery OR a taken-trap full flush wipes everything younger
      // than the branch/trap, which can include a speculatively-dispatched jump
      // that set jump_inflight_q. That jump never executes, so it would never
      // clear the gate -- recovery/trap must, else dispatch deadlocks (an
      // in-flight jump is always younger than the trapping head).
      jump_inflight_q <= 1'b0;
    end else if (issue_fire && issue_is_jump) begin
      jump_inflight_q <= 1'b0;
    end else if (jump_alloc_fire) begin
      jump_inflight_q <= 1'b1;
    end
  end


  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      csr_inflight_q <= 1'b0;
    end else if (branch_recover_req || trap_q_valid) begin
      csr_inflight_q <= 1'b0;   // a taken trap flushes a younger in-flight CSR
    end else if (commit_fire && rob_commit_is_csr) begin
      csr_inflight_q <= 1'b0;
    end else if (csr_alloc_fire) begin
      csr_inflight_q <= 1'b1;
    end
  end

  //helper function for age comparison
  function automatic logic cdb_older_or_same(
    input rob_idx_t a,
    input rob_idx_t b,
    input rob_idx_t head
  );
    cdb_older_or_same = ((a - head) <= (b - head));
  endfunction

  // Choose one pending FU completion for the single CDB/writeback lane.
  always_comb begin
    cdb_grant_alu    = 1'b0;
    cdb_grant_muldiv = 1'b0;
    cdb_grant_lsu    = 1'b0;

    if (alu_complete.valid &&
        (!muldiv_complete.valid ||
        cdb_older_or_same(alu_complete.rob_idx, muldiv_complete.rob_idx, rob_head_idx)) &&
        (!lsu_complete.valid ||
        cdb_older_or_same(alu_complete.rob_idx, lsu_complete.rob_idx, rob_head_idx))) begin
      cdb_grant_alu = 1'b1;

    end else if (muldiv_complete.valid &&
        (!lsu_complete.valid ||
        cdb_older_or_same(muldiv_complete.rob_idx, lsu_complete.rob_idx, rob_head_idx))) begin
      cdb_grant_muldiv = 1'b1;

    end else if (lsu_complete.valid) begin
      cdb_grant_lsu = 1'b1;
    end
  end

  always_comb begin
    csr_src        = operand_a;
    csr_wdata_exec = csr_rdata;
    csr_we_exec    = 1'b0;

    unique case (issue_csr_op)
      rv32i_pipeline_pkg::CSR_RWI, rv32i_pipeline_pkg::CSR_RSI, rv32i_pipeline_pkg::CSR_RCI: begin
        csr_src = {27'b0, issue_csr_zimm};
      end

      default: begin
        csr_src = operand_a;
      end
    endcase

    unique case (issue_csr_op)
      rv32i_pipeline_pkg::CSR_RW: begin
        csr_we_exec    = 1'b1;
        csr_wdata_exec = csr_src;
      end

      rv32i_pipeline_pkg::CSR_RS: begin
        csr_we_exec    = (issue_csr_zimm != 5'd0);
        csr_wdata_exec = csr_rdata | csr_src;
      end

      rv32i_pipeline_pkg::CSR_RC: begin
        csr_we_exec    = (issue_csr_zimm != 5'd0);
        csr_wdata_exec = csr_rdata & ~csr_src;
      end

      rv32i_pipeline_pkg::CSR_RWI: begin
        csr_we_exec    = 1'b1;
        csr_wdata_exec = csr_src;
      end

      rv32i_pipeline_pkg::CSR_RSI: begin
        csr_we_exec    = (issue_csr_zimm != 5'd0);
        csr_wdata_exec = csr_rdata | csr_src;
      end

      rv32i_pipeline_pkg::CSR_RCI: begin
        csr_we_exec    = (issue_csr_zimm != 5'd0);
        csr_wdata_exec = csr_rdata & ~csr_src;
      end

      default: begin
        csr_we_exec    = 1'b0;
        csr_wdata_exec = csr_rdata;
      end
    endcase
  end

  // Hold an ALU completion until the CDB arbiter accepts it. This makes the
  // single-cycle ALU look like the multi-cycle FU completion interface.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      alu_complete <= '0;
    end else begin
      if (cdb_grant_alu) begin
        alu_complete.valid <= 1'b0;
      end
      if (alu_issue_fire) begin
        alu_complete.valid   <= 1'b1;
        alu_complete.rob_idx <= issue_entry.rob_idx;
        alu_complete.rob_seq <= issue_entry.rob_seq;
        alu_complete.pdst    <= issue_pdst;
        alu_complete.rd_wen  <= issue_rd_wen;
        alu_complete.result  <= exec_result;
        alu_complete.trap_valid <= 1'b0;
        alu_complete.trap_cause <= '0;
        alu_complete.trap_tval  <= '0;
        alu_complete.csr_we  <= csr_we_exec;
        alu_complete.csr_wdata <= csr_wdata_exec;
        alu_complete.trap_valid <= exec_trap_valid;
        alu_complete.trap_cause <= exec_trap_cause;
        alu_complete.trap_tval  <= exec_trap_tval;
        alu_complete.rd_wen     <= exec_trap_valid ? 1'b0 : issue_rd_wen;
      end
    end
  end

  // combinational select lane
  word_t load_result;
  logic [15:0] load_half_raw;
  logic [7:0]  load_byte_raw;

  always_comb begin
    load_half_raw = dmem_addr[1] ? dmem_rdata[31:16] : dmem_rdata[15:0];

    unique case (dmem_addr[1:0])
      2'b00: load_byte_raw = dmem_rdata[7:0];
      2'b01: load_byte_raw = dmem_rdata[15:8];
      2'b10: load_byte_raw = dmem_rdata[23:16];
      default: load_byte_raw = dmem_rdata[31:24];
    endcase

    unique case (issue_entry.mem_size)
      fyp_cpu_pkg::MEM_W: load_result = dmem_rdata;
      fyp_cpu_pkg::MEM_H: load_result = issue_entry.mem_unsigned
                                      ? {16'h0000, load_half_raw}
                                      : {{16{load_half_raw[15]}}, load_half_raw};
      fyp_cpu_pkg::MEM_B: load_result = issue_entry.mem_unsigned
                                      ? {24'h000000, load_byte_raw}
                                      : {{24{load_byte_raw[7]}}, load_byte_raw};
      default: load_result = dmem_rdata;
    endcase
  end

  //mirroring alu hold, result for load is read mem data
  // for store, result is the store address using alu add
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      lsu_complete <= '0;
    end else begin
      if (cdb_grant_lsu) begin
        lsu_complete.valid <= 1'b0;
      end

      if (lsu_issue_fire) begin
        lsu_complete.valid   <= 1'b1;
        lsu_complete.rob_idx <= issue_entry.rob_idx;
        lsu_complete.rob_seq <= issue_entry.rob_seq;
        lsu_complete.pdst    <= issue_pdst;
        lsu_complete.rd_wen  <= issue_rd_wen;
        lsu_complete.result  <= issue_entry.is_load ? load_result : agen_addr;
        lsu_complete.trap_valid <= 1'b0;
        lsu_complete.trap_cause <= '0;
        lsu_complete.trap_tval  <= '0;
        lsu_complete.csr_we  <= 1'b0;
        lsu_complete.csr_wdata <= '0;
        lsu_complete.trap_valid <= exec_trap_valid;
        lsu_complete.trap_cause <= exec_trap_cause;
        lsu_complete.trap_tval  <= exec_trap_tval;
        lsu_complete.rd_wen     <= exec_trap_valid ? 1'b0 : issue_rd_wen;
      end
    end
  end

  // Registered CDB beat consumed by ROB/PRF on the following cycle.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cdb_valid_q   <= 1'b0;
      cdb_rob_idx_q <= '0;
      cdb_rob_seq_q <= '0;
      cdb_pdst_q    <= '0;
      cdb_rd_wen_q  <= 1'b0;
      cdb_result_q  <= '0;
      cdb_csr_we_q  <= 1'b0;
      cdb_csr_wdata_q <= '0;
      cdb_trap_valid_q <= 1'b0;
      cdb_trap_cause_q <= '0;
      cdb_trap_tval_q  <= '0;
    end else begin
      cdb_valid_q <= 1'b0;
      cdb_csr_we_q <= 1'b0;
      cdb_csr_wdata_q <= '0;

      if (cdb_grant_alu) begin
        cdb_valid_q   <= 1'b1;
        cdb_rob_idx_q <= alu_complete.rob_idx;
        cdb_rob_seq_q <= alu_complete.rob_seq;
        cdb_pdst_q    <= alu_complete.pdst;
        cdb_rd_wen_q  <= alu_complete.rd_wen;
        cdb_result_q  <= alu_complete.result;
        cdb_csr_we_q  <= alu_complete.csr_we;
        cdb_csr_wdata_q <= alu_complete.csr_wdata;
        cdb_trap_valid_q <= alu_complete.trap_valid;
        cdb_trap_cause_q <= alu_complete.trap_cause;
        cdb_trap_tval_q  <= alu_complete.trap_tval;
      end else if (cdb_grant_muldiv) begin
        cdb_valid_q   <= 1'b1;
        cdb_rob_idx_q <= muldiv_complete.rob_idx;
        cdb_rob_seq_q <= muldiv_complete.rob_seq;
        cdb_pdst_q    <= muldiv_complete.pdst;
        cdb_rd_wen_q  <= muldiv_complete.rd_wen;
        cdb_result_q  <= muldiv_complete.result;
        cdb_csr_we_q  <= muldiv_complete.csr_we;
        cdb_csr_wdata_q <= muldiv_complete.csr_wdata;
        cdb_trap_valid_q <= muldiv_complete.trap_valid;
        cdb_trap_cause_q <= muldiv_complete.trap_cause;
        cdb_trap_tval_q  <= muldiv_complete.trap_tval;
      end else if (cdb_grant_lsu) begin
        cdb_valid_q   <= 1'b1;
        cdb_rob_idx_q <= lsu_complete.rob_idx;
        cdb_rob_seq_q <= lsu_complete.rob_seq;
        cdb_pdst_q    <= lsu_complete.pdst;
        cdb_rd_wen_q  <= lsu_complete.rd_wen;
        cdb_result_q  <= lsu_complete.result;
        cdb_csr_we_q  <= lsu_complete.csr_we;
        cdb_csr_wdata_q <= lsu_complete.csr_wdata;
        cdb_trap_valid_q <= lsu_complete.trap_valid;
        cdb_trap_cause_q <= lsu_complete.trap_cause;
        cdb_trap_tval_q  <= lsu_complete.trap_tval;
      end
    end
  end




  // Operand selection from src1_sel / src2_sel.
  always_comb begin
    case (issue_src1_sel)
      OOO_SRC_PC:   operand_a = issue_pc;
      OOO_SRC_ZERO: operand_a = '0;
      default:      operand_a = prf_rdata1;   // OOO_SRC_REG
    endcase
    case (issue_src2_sel)
      OOO_SRC_IMM:  operand_b = issue_imm;
      OOO_SRC_PC:   operand_b = issue_pc;
      OOO_SRC_ZERO: operand_b = '0;
      default:      operand_b = prf_rdata2;   // OOO_SRC_REG
    endcase
  end

  always_comb begin
    control_target = '0;
    control_target_misalign = '0;
    if (jump_resolve_fire) begin
      control_target = (operand_a + issue_imm) & 32'hffff_fffe;
      control_target_misalign = (control_target[1:0] != 2'b00);
    end else if (issue_fire && issue_is_branch && branch_taken) begin
      control_target = issue_pc + issue_imm;
      control_target_misalign = (control_target[1:0] != 2'b00);
    end

    load_misalign = lsu_issue_fire && issue_entry.is_load &&
                    (((issue_entry.mem_size == fyp_cpu_pkg::MEM_W)
                     && (agen_addr[1:0] != 2'b00)) ||
                    ((issue_entry.mem_size == fyp_cpu_pkg::MEM_H)
                    && (agen_addr[0]   != 1'b0)));
    store_misalign = lsu_issue_fire && issue_entry.is_store &&
                    (((issue_entry.mem_size == fyp_cpu_pkg::MEM_W)
                    && (agen_addr[1:0] != 2'b00)) ||
                    ((issue_entry.mem_size == fyp_cpu_pkg::MEM_H)
                    && (agen_addr[0]   != 1'b0)));

    lsu_misalign = load_misalign || store_misalign;
    exec_trap_cause = '0;
    exec_trap_valid = '0;
    exec_trap_tval  = '0;
    exec_trap_valid = lsu_misalign || control_target_misalign;

    if (control_target_misalign) begin
      exec_trap_cause = OOO_CAUSE_IADDR_MISALIGN;
      exec_trap_tval  = control_target;
    end else if (load_misalign) begin
      exec_trap_cause = OOO_CAUSE_LOAD_MISALIGN;
      exec_trap_tval  = agen_addr;
    end else if (store_misalign) begin
      exec_trap_cause = OOO_CAUSE_STORE_MISALIGN;
      exec_trap_tval  = agen_addr;
    end
  end


  // Store-side lane placement is computed at LSU execute. The commit-time
  // memory write will later consume these already-final bus values by rob_idx.
  always_comb begin
    store_wdata_next = '0;
    store_be_next    = 4'b0000;

    unique case (issue_entry.mem_size)
      fyp_cpu_pkg::MEM_W: begin
        store_wdata_next = prf_rdata2;
        store_be_next    = 4'b1111;
      end

      fyp_cpu_pkg::MEM_H: begin
        if (agen_addr[1]) begin
          store_wdata_next = {prf_rdata2[15:0], 16'h0000};
          store_be_next    = 4'b1100;
        end else begin
          store_wdata_next = {16'h0000, prf_rdata2[15:0]};
          store_be_next    = 4'b0011;
        end
      end

      fyp_cpu_pkg::MEM_B: begin
        unique case (agen_addr[1:0])
          2'b00: begin
            store_wdata_next = {24'h000000, prf_rdata2[7:0]};
            store_be_next    = 4'b0001;
          end
          2'b01: begin
            store_wdata_next = {16'h0000, prf_rdata2[7:0], 8'h00};
            store_be_next    = 4'b0010;
          end
          2'b10: begin
            store_wdata_next = {8'h00, prf_rdata2[7:0], 16'h0000};
            store_be_next    = 4'b0100;
          end
          default: begin
            store_wdata_next = {prf_rdata2[7:0], 24'h000000};
            store_be_next    = 4'b1000;
          end
        endcase
      end

      default: begin
        store_wdata_next = prf_rdata2;
        store_be_next    = 4'b1111;
      end
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (store_i = 0; store_i < OOO_ROB_DEPTH; store_i = store_i + 1) begin
        store_addr_q[store_i]  <= '0;
        store_wdata_q[store_i] <= '0;
        store_be_q[store_i]    <= 4'b0000;
      end
    end else begin
      if (trap_q_valid) begin
        for (store_i = 0; store_i < OOO_ROB_DEPTH; store_i = store_i + 1) begin
          store_addr_q[store_i]  <= '0;
          store_wdata_q[store_i] <= '0;
          store_be_q[store_i]    <= 4'b0000;
        end
      end else if (branch_recover_req) begin
        for (store_i = 0; store_i < OOO_ROB_DEPTH; store_i = store_i + 1) begin
          if ((rob_idx_t'(store_i) - rob_head_idx) > (recover_q_rob_idx - rob_head_idx)) begin
            store_addr_q[store_i]  <= '0;
            store_wdata_q[store_i] <= '0;
            store_be_q[store_i]    <= 4'b0000;
          end
        end
      end

      if (commit_fire && rob_commit_is_store) begin
        store_addr_q[rob_head_idx]  <= '0;
        store_wdata_q[rob_head_idx] <= '0;
        store_be_q[rob_head_idx]    <= 4'b0000;
      end

      if (lsu_issue_fire && issue_entry.is_store && !store_misalign) begin
        store_addr_q[issue_entry.rob_idx]  <= agen_addr;
        store_wdata_q[issue_entry.rob_idx] <= store_wdata_next;
        store_be_q[issue_entry.rob_idx]    <= store_be_next;
      end
    end
  end

  rv32i_ooo_prf u_prf (
    .clk      (clk),
    .rst_n    (rst_n),
    .raddr1   (issue_prs1),
    .rdata1   (prf_rdata1),
    .raddr2   (issue_prs2),
    .rdata2   (prf_rdata2),
    .write_en (rob_wb_accept & cdb_rd_wen_q),
    .waddr    (cdb_pdst_q),
    .wdata    (cdb_result_q),

    // ---- ready/busy table ----
    // A real-dest uop allocates its pdst at dispatch -> mark it busy.
    .alloc_en   (rob_alloc_fire & rename_rd_we),
    .alloc_phys (rename_pdst),
    .ready_vec  (ready_vec)                       // registered source readiness for the IQ
  );

  rv32i_alu u_alu (
    .operand_a (operand_a),
    .operand_b (operand_b),
    .alu_op    (issue_alu_op),
    .result    (alu_result),
    .zero      ()
  );

  rv32i_ooo_muldiv u_muldiv (
    .clk            (clk),
    .rst_n          (rst_n),
    .start          (muldiv_start),
    .op             (issue_muldiv_op),
    .a              (operand_a),
    .b              (operand_b),
    .kill           (muldiv_kill),          // driven by branch recovery or precise trap flush
    .rob_idx        (issue_entry.rob_idx),
    .rob_seq        (issue_entry.rob_seq),
    .branch_mask    (issue_entry.branch_mask),
    .pdst           (issue_pdst),
    .rd_wen         (issue_rd_wen),
    .complete_ready (cdb_grant_muldiv),
    .busy           (muldiv_busy),
    .complete       (muldiv_complete)
  );

  // ---- machine-mode CSR file (Zicsr). The CSR execute
  //      path reads the old CSR value for rd, while the architectural CSR write
  //      is replayed from the committing ROB head. Trap/mret have dedicated ports. ----
  rv32i_ooo_csr_file u_csr_file (
    .clk        (clk),
    .rst_n      (rst_n),
    .we         (commit_fire && rob_commit_is_csr && rob_commit_csr_we),
    .waddr      (rob_commit_csr_addr),
    .wdata      (rob_commit_csr_wdata),
    .rdata      (csr_rdata),
    .raddr      (issue_csr_addr),
    .trap_we    (trap_q_valid && !trap_q_is_mret),
    .trap_pc    (trap_q_pc),
    .trap_cause (trap_q_cause),
    .trap_tval  (trap_q_tval),
    .mret_we    (trap_q_valid && trap_q_is_mret),
    .mtvec      (csr_mtvec),
    .mepc       (csr_mepc)
  );

  rv32i_branch_cmp u_branch_cmp (
    .branch_op (issue_branch_op),
    .a         (operand_a),
    .b         (operand_b),
    .taken     (branch_taken)
  );
  // Redirect to the frontend. Branch redirect is MISPREDICT-ONLY and rides the
  // REGISTERED recovery (recover_q_target, valid at N+1) -- a correctly-predicted
  // not-taken branch never redirects. A serialized jump redirects on its own
  // resolve (the issue port still holds its target that cycle).
  always_comb begin
    exec_result     = alu_result;
    redirect_valid  = 1'b0;
    redirect_target = fallthrough_pc;
    if (issue_is_csr) begin
      exec_result = csr_rdata;
    end
    if (trap_q_valid) begin
      redirect_valid = 1'b1;
      redirect_target = trap_q_target;
    end else if (branch_recover_req) begin
      redirect_valid = 1'b1;
      redirect_target = recover_q_target;
    end else if (jump_resolve_fire && !control_target_misalign) begin
      redirect_valid = 1'b1;
      redirect_target = (operand_a + issue_imm) & 32'hffff_fffe;
      exec_result = fallthrough_pc;
    end
  end

`ifndef SYNTHESIS
  always @(posedge clk) begin
    if (rst_n === 1'b1) begin
      if (issue_fire && !issue_valid) begin
        $fatal(1, "rv32i_ooo_core: issue_fire without IQ issue_valid");
      end

      if (issue_fire && !rob_head_valid) begin
        $fatal(1, "rv32i_ooo_core: issue_fire while ROB is empty");
      end

      if (rob_wb_accept && !cdb_fire) begin
        $fatal(1, "rv32i_ooo_core: ROB accepted writeback without CDB fire");
      end

      if ((cdb_grant_alu && cdb_grant_muldiv) ||
          (cdb_grant_alu && cdb_grant_lsu) ||
          (cdb_grant_muldiv && cdb_grant_lsu)) begin
        $fatal(1, "rv32i_ooo_core: CDB granted two completions in one cycle");
      end

      if (cdb_grant_alu && !alu_complete.valid) begin
        $fatal(1, "rv32i_ooo_core: CDB granted invalid ALU completion");
      end

      if (cdb_grant_muldiv && !muldiv_complete.valid) begin
        $fatal(1, "rv32i_ooo_core: CDB granted invalid muldiv completion");
      end

      if (cdb_grant_lsu && !lsu_complete.valid) begin
        $fatal(1, "rv32i_ooo_core: CDB granted invalid LSU completion");
      end

      if (alu_issue_fire && alu_complete.valid) begin
        $fatal(1, "rv32i_ooo_core: ALU issue would overwrite a held completion");
      end

      if (lsu_issue_fire && lsu_complete.valid) begin
        $fatal(1, "rv32i_ooo_core: LSU issue would overwrite a held completion");
      end

      if ((lsu_issue_fire && issue_entry.is_load && !load_misalign) && !(dmem_ready && dmem_rvalid)) begin
        $fatal(1, "rv32i_ooo_core: load dmem access without ready/rvalid");
      end

      if ((lsu_issue_fire && issue_entry.is_load && !load_misalign) && store_commit_fire) begin
        $fatal(1, "rv32i_ooo_core: load dmem access collides with store commit");
      end

      if (store_commit_fire && !dmem_ready) begin
        $fatal(1, "rv32i_ooo_core: store commit without dmem_ready");
      end

      if (dmem_we && !store_commit_fire) begin
        $fatal(1, "rv32i_ooo_core: dmem_we asserted outside store commit");
      end

      if (store_commit_fire && !dmem_valid) begin
        $fatal(1, "rv32i_ooo_core: store commit without dmem_valid");
      end

      if (cdb_fire && cdb_rd_wen_q && (cdb_pdst_q == '0)) begin
        $fatal(1, "rv32i_ooo_core: CDB writes p0 for a real destination");
      end

      if (jump_inflight_q && jump_alloc_fire) begin
        $fatal(1, "rv32i_ooo_core: allocated a new jump while a jump is in flight");
      end

      if (redirect_valid && !(branch_recover_req | jump_resolve_fire | trap_q_valid)) begin
        $fatal(1, "rv32i_ooo_core: redirect asserted without branch recover, jump resolve, or trap");
      end
    end
  end
`endif

endmodule
