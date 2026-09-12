`timescale 1ns/1ps

// rv32i_ss_core - two-wide front/dispatch and dual-execute RV32IM core.
//
// The core contains rename, a free list, ROB, IQ and PRF; exact-unit issue
// binding for ALU0/ALU1/muldiv/AGEN; completion holders; and two registered
// CDB/writeback lanes. Branches speculate with checkpoint recovery. An
// aligned-target JAL redirects at dispatch and executes as a link-producing
// operation. Unpredicted JALR and misaligned-target JAL serialize; predicted
// returns use checkpoint recovery to validate their targets at execute.
// Decode traps, mret and execute-detected misalignment reach the ROB head
// before the registered trap flush applies their architectural effects.
// CSRs serialize and perform read-modify-write at commit. Loads deposit
// addresses at AGU execution and launch from the ordering-checked LQ;
// stores drain from the SQ through the data-memory handshake.

module rv32i_ss_core
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;
(
    input  logic          clk,
    input  logic          rst_n,

    input  logic          decoded_valid,
    input  logic [1:0]    decoded_slot_valid,
    output logic          decoded_ready,
    input  word_t         [1:0] decoded_pc,
    input  word_t         [1:0] decoded_instr,
    input  arch_reg_t     [1:0] decoded_rs1,
    input  arch_reg_t     [1:0] decoded_rs2,
    input  arch_reg_t     [1:0] decoded_rd,
    input  logic          [1:0] decoded_rd_we,
    input  logic          [1:0] decoded_needs_checkpoint,
    input  ooo_op_class_e [1:0] decoded_op_class,
    input  ooo_fu_class_e [1:0] decoded_fu_class,
    input  alu_op_e       [1:0] decoded_alu_op,
    input  muldiv_op_e    [1:0] decoded_muldiv_op,
    input  br_type_e      [1:0] decoded_branch_op,
    input  ooo_src_sel_e  [1:0] decoded_src1_sel,
    input  ooo_src_sel_e  [1:0] decoded_src2_sel,
    input  word_t         [1:0] decoded_imm,
    input  decoded_trap_t decoded_trap,   // decode-detected trap (from frontend)
    input  csr_op_e       decoded_csr_op,
    input  csr_addr_t     decoded_csr_addr,
    input  csr_zimm_t     decoded_csr_zimm,

    // Fetch-time prediction carried by the decoded bundle. Zero prediction
    // bits select the not-taken path.
    input  logic  [1:0]   decoded_pred_taken,
    input  word_t         decoded_pred_target,

    // ---- branch/jump redirect back to the frontend ----
    output logic          redirect_valid,
    output word_t         redirect_target,

    // ---- predictor training back to the frontend (execute-time,
    // alu0-priority when both ALUs resolve in one cycle) ----
    output logic          bp_update_valid,
    output word_t         bp_update_pc,
    output logic          bp_update_taken,
    output word_t         bp_update_target,

    // ---- learned return-site update + read-only RAS lookup ----
    output logic          bp_return_update_valid,
    output word_t         bp_return_update_pc,
    output logic          ras_fetch_valid,
    output word_t         ras_fetch_target,

    // Per-slot commit trace. Payload ports are observation only. commit_fire
    // also drives architectural retirement in rename, the free list, the ROB,
    // the LSQ and the CSR path. Slot 1 obeys ordered-prefix, serialization and
    // one-store-per-group rules.
    //
    // commit_order is one counter: record p has order commit_order + p.
    output logic [1:0]      commit_fire,
    output commit_order_t   commit_order,
    output word_t [1:0]     commit_pc,
    output word_t [1:0]     commit_inst,
    output arch_reg_t [1:0] commit_rd,
    output logic [1:0]      commit_rd_wen,
    output word_t [1:0]     commit_wdata,

    // Load and store
    input logic      [1:0] decoded_is_load,
    input logic      [1:0] decoded_is_store,
    input mem_size_e [1:0] decoded_mem_size,
    input logic      [1:0] decoded_mem_unsigned,

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

  // ---- dispatch control ----
  logic      bundle_fire;
  logic [1:0] bundle_size;
  logic      dispatch_mem_ready;
  iq_entry_t [1:0] iq_alloc_entry;
  iq_entry_t iq_entry_next;
  logic      dispatch_accept;
  integer    iq_build_i;

  // ---- rename / free-list ----
  logic [1:0] preg_slot_need;
  logic       preg_avail;
  phys_reg_t [1:0] preg_alloc_reg;
  phys_reg_t [1:0] rename_prs1, rename_prs2, rename_pdst, rename_stale_pdst;
  arch_reg_t [1:0] rename_rd;
  logic      [1:0] rename_rd_we;
  branch_mask_t rename_branch_mask;
  logic       rename_checkpoint_valid;
  ckpt_idx_t  rename_checkpoint_id;
  logic [OOO_PHYS_REGS-1:0] rename_branch_recover_alloc_list;

  // ---- ROB / commit ----
  logic       rob_alloc_ready;
  rob_idx_t [1:0] rob_alloc_idx;
  rob_seq_t [1:0] rob_alloc_seq;
  rob_idx_t   rob_head_idx;           // oldest in-flight; CDB + IQ age reference
  logic       rob_head_valid;
  // ROB commit facts are per position: slot 0 is the head and slot 1 is
  // head+1. Each architectural sink consumes the same commit event vector.
  logic [1:0]      rob_commit_valid;
  logic [1:0]      rob_commit_rd_we;
  arch_reg_t [1:0] rob_commit_rd;
  phys_reg_t [1:0] rob_commit_pdst;
  phys_reg_t [1:0] rob_commit_stale_pdst;
  word_t [1:0]     rob_commit_pc;
  word_t [1:0]     rob_commit_instr;
  word_t [1:0]     rob_commit_result;
  commit_order_t   rob_commit_order;
  logic [1:0]      rob_commit_is_csr;
  logic [1:0]      rob_commit_is_store;
  logic [1:0]      rob_commit_is_load;
  logic [1:0]      rob_commit_csr_we;
  csr_addr_t [1:0] rob_commit_csr_addr;
  word_t [1:0]     rob_commit_csr_wdata;


  // ---- decode-detected trap carried to the ROB head ----
  logic       rob_alloc_trap_valid;
  word_t      rob_alloc_trap_cause;
  word_t      rob_alloc_trap_tval;
  logic       rob_alloc_is_mret;
  logic       rob_alloc_is_csr;
  logic [1:0] rob_alloc_is_store;
  logic [1:0] rob_alloc_is_load;

  // ---- IQ issue ----
  logic              [1:0] iq_select_valid;
  iq_entry_t          [1:0] iq_select_entry;
  issue_unit_e        [1:0] iq_select_unit;
  logic                     iq_select_accept;
  logic              [1:0] issue_valid;
  iq_entry_t          [1:0] issue_entry;
  issue_unit_e        [1:0] issue_unit;
  logic                     iq_alloc_ready;
  logic                     issue_accept;
  logic [OOO_PHYS_REGS-1:0] ready_vec;
  logic                     alu0_fu_ready;
  logic                     alu1_fu_ready;
  logic                     muldiv_fu_ready;
  logic                     lsu_fu_ready;

  // Registered input/result banks. issue_fire enqueues;
  // exec_fire consumes a previously registered occupant exactly once.
  exec_input_slot_t  [1:0] alu0_in_q;
  exec_input_slot_t  [1:0] alu1_in_q;
  exec_input_slot_t  [1:0] agen_in_q;
  exec_input_slot_t        md_in_q;
  exec_result_slot_t [1:0] alu0_res_q;
  exec_result_slot_t [1:0] alu1_res_q;
  logic                    alu0_exec_fire;
  logic                    alu1_exec_fire;
  logic                    agen_exec_fire;
  logic                    md_exec_fire;
  logic                    alu0_exec_idx;
  logic                    alu1_exec_idx;
  logic                    agen_exec_idx;
  logic                    alu0_res_offer_valid;
  logic                    alu1_res_offer_valid;
  logic                    alu0_res_offer_idx;
  logic                    alu1_res_offer_idx;
  logic                    alu0_res_free;
  logic                    alu1_res_free;
  logic                    alu0_res_push_idx;
  logic                    alu1_res_push_idx;
  logic                    alu0_in_push_idx;
  logic                    alu1_in_push_idx;
  logic                    agen_in_push_idx;
  logic                    solo_alu0_exec;
  logic                    exec_ok;
  logic                    in_slot_younger [6];
  integer                  in_kill_i;
  iq_entry_t               alu0_exec_entry;
  iq_entry_t               alu1_exec_entry;
  iq_entry_t               agen_exec_entry;
  iq_entry_t               md_exec_entry;
  logic                    muldiv_issue_fire;
  word_t                   alu0_exec_a, alu0_exec_b;
  word_t                   alu1_exec_a, alu1_exec_b;
  word_t                   agen_exec_a, agen_exec_b;
  word_t                   md_exec_a, md_exec_b;
  word_t                   agen_exec_store_data;
  logic                    agen_exec_store_data_valid;
  logic                    alu0_exec_is_jal_deser;
  logic                    alu0_exec_is_jump_pred;
  logic                    alu0_exec_is_jump_ser;
  logic                    store_data_ready_at_exec;
  iq_entry_t               agen_enq_entry;
  word_t                   agen_enq_a, agen_enq_b, agen_enq_store_data;

  // ---- LSQ allocation ----
  logic    lq_alloc_fire;
  logic    lq_alloc_ready;
  lq_idx_t lq_alloc_idx;
  logic    sq_alloc_fire;
  logic    sq_alloc_ready;
  sq_idx_t sq_alloc_idx;
  logic bundle_has_load;
  logic bundle_has_store;
  logic mem_slot_idx;

  // ---- issued-uop unpack + classification (per grant position;
  // indexed arrays, position 0 = older; consumed through the
  // position-to-unit router).
  logic [1:0] issue_fire;
  word_t         [1:0] issue_pc;
  word_t         [1:0] issue_imm;
  phys_reg_t     [1:0] issue_prs1;
  phys_reg_t     [1:0] issue_prs2;
  phys_reg_t     [1:0] issue_pdst;
  ooo_src_sel_e  [1:0] issue_src1_sel;
  ooo_src_sel_e  [1:0] issue_src2_sel;
  ooo_op_class_e [1:0] issue_op_class;
  ooo_fu_class_e [1:0] issue_fu_class;
  br_type_e      [1:0] issue_branch_op;
  logic          [1:0] issue_rd_wen;
  logic          [1:0] issue_is_branch;
  logic          [1:0] issue_is_jump;
  logic          [1:0] issue_is_control;
  logic          [1:0] issue_is_lsu;
  logic          [1:0] issue_is_csr;
  csr_addr_t     [1:0] issue_csr_addr;
  csr_zimm_t     [1:0] issue_csr_zimm;
  csr_op_e       [1:0] issue_csr_op;

  // ---- ALU / muldiv functional units ----
  word_t      prf_rdata1, prf_rdata2;
  // Read ports 3/4 serve grant position 1.
  word_t      prf_rdata3, prf_rdata4;
  // Bypassed operands may come from live ALU result holders or accepted
  // CDB transit beats. Non-ALU readiness waits for registered writeback.
  // ALU, branch, muldiv, AGU, store-data and CSR consumers all use these
  // values; identity-qualified acceptance gates transit bypass.
  word_t      byp_rdata1, byp_rdata2, byp_rdata3, byp_rdata4;
  word_t[1:0] operand_a, operand_b;
  word_t[1:0]      exec_result;
  logic       alu0_issue_fire, alu1_issue_fire;
  logic       agen_issue_fire;
  logic       load_agen_fire;
  logic       store_agen_fire;
  logic       store_commit_fire;
  logic [1:0] commit_want;
  logic [1:0] store_commit_want;
  logic       store_ready_ok;
  logic       sq_mem_accept;
  logic       sq_accept_deferred;
  logic       muldiv_start;
  logic       muldiv_busy;
  word_t      alu0_result;
  word_t      alu1_result;
  iq_entry_t  alu0_issue_entry, alu1_issue_entry;
  iq_entry_t muldiv_issue_entry;
  iq_entry_t agen_issue_entry;
  word_t     muldiv_operand_a, muldiv_operand_b;
  word_t     alu0_operand_a, alu0_operand_b;
  word_t     alu1_operand_a, alu1_operand_b;
  word_t     agen_operand_a, agen_operand_b;
  word_t     agen_store_data;
  completion_packet_t alu0_complete;
  completion_packet_t alu1_complete;
  // Execution-driven ALU wakeup: these bits track whether each registered holder
  // still names a live ROB generation. Flow-and-reject intentionally leaves a
  // killed packet physically resident until CDB drain; tag-only holder bypass
  // would therefore be unsafe after recovery-tail reuse.
  logic               alu0_holder_live_q;
  logic               alu1_holder_live_q;
  completion_packet_t muldiv_complete;
  completion_packet_t agen_complete;
  completion_packet_t agen_cdb_complete;
  completion_packet_t lq_complete;
  completion_packet_t sq_complete;
  completion_packet_t [1:0] cdb_q;
  // Grant selection ends at cdb_q. No combinational grant-to-ready path:
  // ordinary readiness is set with the ROB-accepted registered writeback.
  completion_packet_t [1:0] cdb_next;

  // Store-data decoupling: accepted CDB writes feed late SQ payload capture.
  // The deferred SQ completion shares the existing AGU CDB client.
  logic [1:0]      sq_data_wb_fire;
  phys_reg_t [1:0] sq_data_wb_pdst;
  word_t [1:0]     sq_data_wb_value;
  logic            sq_complete_accept;
  logic            agen_complete_select;
  logic            agen_complete_accept;
  logic            store_data_ready_at_issue;
  logic            sq_deposit_data_valid;
  logic            sq_deposit_deferred_pending;
  logic            agen_direct_complete_fire;

  rob_idx_t muldiv_rob_idx_q;   // rob_idx of the in-flight muldiv (ring-distance muldiv_kill)

  // Store-byte formatting is deposited into the SQ at AGU execution.
  word_t      store_wdata_next;
  logic [3:0] store_be_next;

  // ---- dual-lane CDB / writeback ----
  logic [1:0] rob_wb_accept;
  logic      cdb_grant_alu0;
  logic      cdb_grant_alu1;
  logic      cdb_grant_muldiv;
  logic      cdb_grant_agen;
  logic      cdb_grant_lq;

  // Jump serialization. Aligned JAL targets are exact at dispatch, so they
  // redirect below trap, recovery and serialized-jump resolution in priority.
  // They do not set jump_inflight_q and execute on ALU0 as rd <- pc+4.
  // Unpredicted JALR waits for rs1. Misaligned-target JAL uses execute-side
  // trap detection. Dispatch and execute classification must agree on JAL
  // source selection and the alignment of the packet's pc + imm.
  logic       jump_alloc_fire;
  logic       jump_inflight_q;
  logic       decoded_slot0_is_jal;
  logic       decoded_jal_deser_ok;
  word_t      jal_deser_target;
  logic       jal_deser_fire;
  logic       issue_is_jal_deser;
  logic       issue_is_jump_serialized;

  // Return-address stack and speculative return handling.
  // The depth-8 stack stores pc+4, wraps on overwrite, and updates at dispatch
  // using the x1/x5 link-register hints. Jumps occupy slot 0 alone. Prediction
  // is consumed only for pop forms that allocate no destination, because a
  // checkpointing slot cannot allocate a destination register. Other pop forms
  // serialize while still updating the stack.
  //
  // A predicted return requests a rename checkpoint and validates
  // (rs1 + imm) & ~1 at execute. A mismatch recovers; a match releases the
  // checkpoint. A misaligned actual target follows the precise trap path.
  //
  // RAS contents are hints, so incorrect contents cause ordinary misprediction.
  // Each checkpoint saves the post-dispatch {tos, count} pointers for recovery.
  // A predicted return retains its own pop when it recovers. Wrong-path pushes
  // may overwrite contents; a full trap flush clears the stack.
  localparam int RAS_DEPTH = 8;
  logic                        decoded_slot0_is_jalr;
  logic                        decoded_link_rd0;
  logic                        decoded_link_rs10;
  logic                        decoded_ras_push;
  logic                        decoded_ras_pop;
  logic                        ras_pred_want;
  logic                        ras_pred_fire;
  logic                        ras_fetch_pred_want;
  logic                        ras_decode_pred_want;
  logic                        ras_decode_redirect_fire;
  word_t                       ras_pred_target;
  logic [1:0]                  dispatch_needs_checkpoint;
  word_t                       ras_q [RAS_DEPTH];
  logic [$clog2(RAS_DEPTH)-1:0] ras_tos_q;
  logic [$clog2(RAS_DEPTH):0]   ras_count_q;
  logic [$clog2(RAS_DEPTH)-1:0] ras_tos_d;
  logic [$clog2(RAS_DEPTH):0]   ras_count_d;
  logic                        ras_wr_en;
  logic [$clog2(RAS_DEPTH)-1:0] ras_wr_idx;
  logic                        issue_is_jump_predicted;
  logic                        jump_pred_resolve_fire;
  logic [$clog2(RAS_DEPTH)-1:0] ras_ckpt_tos_q   [OOO_BRANCH_CKPTS];
  logic [$clog2(RAS_DEPTH):0]   ras_ckpt_count_q [OOO_BRANCH_CKPTS];
  integer ras_rst_i;

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
  logic [1:0] branch_taken;  // indexed by physical ALU unit
  branch_candidate_t [1:0] branch_candidate;
  branch_candidate_t selected_recovery;
  branch_mask_t checkpoint_release_mask;
  logic       branch_recover_req;
  ckpt_idx_t  branch_recover_id;
  logic       muldiv_kill;
  logic       jump_resolve_fire;
  logic       csr_issue_fire;
  word_t      fallthrough_pc;
  // registered recovery: decide at cycle N, drive all consumers from the flop at N+1
  logic         recover_q_valid;
  ckpt_idx_t    recover_q_ckpt_id;
  rob_idx_t     recover_q_rob_idx;
  word_t        recover_q_target;     // mispredict redirect target, latched at N

  // registered trap taken -- the trap-side mirror of the branch pipeline
  // (selected_recovery.recover_valid -> recover_q_valid). Here, the same shape:
  // a combinational decision at the commit boundary latches one registered
  // event (trap_q_valid) that next cycle drives the full flush + CSR write +
  // mtvec/mepc redirect. trap_latch_fire needs the !trap_q_valid guard because
  // the ROB head still presents the same trap for the cycle while the
  // registered flush takes effect (else mstatus would be pushed twice).
  logic         trap_commit_event;
  logic         trap_latch_fire;
  logic         trap_q_valid;
  logic         trap_q_is_mret;
  word_t        trap_q_pc;       // mepc for trap (the faulting/return PC)
  word_t        trap_q_cause;
  word_t        trap_q_tval;
  word_t        trap_q_target;   // mtvec for trap, mepc for mret

  logic [1:0]   commit_trap_valid;
  word_t [1:0]  commit_trap_cause;
  word_t [1:0]  commit_trap_tval;
  logic [1:0]   commit_is_mret;

  word_t        agen_addr;   // AGU output for the executing memory operation (rs1+imm via the shared ALU adder)
  word_t[1:0]        control_target;
  logic [1:0]        control_target_misalign;
  logic         load_misalign;
  logic         store_misalign;
  logic         lsu_misalign;
  word_t [1:0]  exec_trap_cause;
  word_t [1:0]  exec_trap_tval;
  logic [1:0]   exec_trap_valid;
  logic         agen_trap_valid;
  word_t        agen_trap_cause;
  word_t        agen_trap_tval;


  // ---------------- dispatch / jump-serialization gate ----------------
  // Branches and predicted returns use checkpoints. Unpredicted JALR and
  // misaligned-target JAL serialize through jump_inflight_q. The control
  // block below drives jump_alloc_fire once.
  assign bundle_has_load =  |(decoded_slot_valid & decoded_is_load);
  assign bundle_has_store = |(decoded_slot_valid & decoded_is_store);
  assign mem_slot_idx = decoded_slot_valid[1] && (decoded_is_load[1] || decoded_is_store[1]);
  assign dispatch_mem_ready = bundle_has_load  ? lq_alloc_ready :
                              bundle_has_store ? sq_alloc_ready :
                              1'b1;
  assign lq_alloc_fire = bundle_fire && bundle_has_load;
  assign sq_alloc_fire = bundle_fire && bundle_has_store;
  assign bundle_fire = decoded_valid && decoded_ready;
  assign bundle_size = {1'b0, decoded_slot_valid[0]}
                       + {1'b0, decoded_slot_valid[1]};
  assign dispatch_accept  = ~jump_inflight_q & ~branch_recover_req
                            & ~csr_inflight_q & ~trap_q_valid;

  // AGU address (rs1 + imm through the shared adder) and the store-commit event.
  assign agen_addr = agen_exec_a + agen_exec_b;
  // a store may commit in EITHER slot, so the dmem-port assertions below
  // must see the whole group. At most one store retires per group (single
  // write port), so this stays a scalar event.
  assign store_commit_fire = (commit_fire[0] && rob_commit_is_store[0])
                           || (commit_fire[1] && rob_commit_is_store[1]);

  // ---------------- dispatch entry assembly (drives ROB + IQ) ----------------
  always_comb begin
    iq_alloc_entry = '0;
    iq_entry_next  = '0;

    for (iq_build_i = 0; iq_build_i < 2; iq_build_i++) begin
      iq_entry_next = '0;

      if (decoded_slot_valid[iq_build_i]) begin
        iq_entry_next.pc      = decoded_pc[iq_build_i];
        iq_entry_next.rob_idx = rob_alloc_idx[iq_build_i];
        iq_entry_next.rob_seq = rob_alloc_seq[iq_build_i];

        // Enum-array element selects lose the enum type -> explicit casts.
        iq_entry_next.op_class  = ooo_op_class_e'(decoded_op_class[iq_build_i]);
        iq_entry_next.fu_class  = ooo_fu_class_e'(decoded_fu_class[iq_build_i]);
        iq_entry_next.alu_op    = alu_op_e'(decoded_alu_op[iq_build_i]);
        iq_entry_next.branch_op = br_type_e'(decoded_branch_op[iq_build_i]);
        iq_entry_next.muldiv_op = muldiv_op_e'(decoded_muldiv_op[iq_build_i]);
        iq_entry_next.src1_sel  = ooo_src_sel_e'(decoded_src1_sel[iq_build_i]);
        iq_entry_next.src2_sel  = ooo_src_sel_e'(decoded_src2_sel[iq_build_i]);

        iq_entry_next.prs1   = rename_prs1[iq_build_i];
        iq_entry_next.prs2   = rename_prs2[iq_build_i];
        iq_entry_next.pdst   = rename_pdst[iq_build_i];
        iq_entry_next.imm    = decoded_imm[iq_build_i];
        iq_entry_next.rd_wen = rename_rd_we[iq_build_i];

        // the prediction rides only the slot it was made for; the
        // shared target word is captured only where pred_taken is set.
        // a RAS-predicted return (slot 0, jump-solo) carries its
        // prediction the same way. A learned site already arrives in the
        // decoded payload; a cold site uses the dispatch-time RAS fallback.
        iq_entry_next.pred_taken  = decoded_pred_taken[iq_build_i]
                                    || ((iq_build_i == 0) && ras_pred_want);
        iq_entry_next.pred_target = decoded_pred_taken[iq_build_i]
                                    ? decoded_pred_target
                                    : (((iq_build_i == 0) && ras_pred_want)
                                       ? ras_pred_target : '0);

        // checkpoint stamping reads the EFFECTIVE request vector so a
        // predicted return owns the checkpoint rename allocated for it.
        iq_entry_next.branch_mask =
            rename_branch_mask |
            (((iq_build_i == 1) && rename_checkpoint_valid &&
              dispatch_needs_checkpoint[0])
             ? (branch_mask_t'(1) << rename_checkpoint_id) : '0);
        iq_entry_next.checkpoint_id =
            (dispatch_needs_checkpoint[iq_build_i] && rename_checkpoint_valid)
            ? rename_checkpoint_id : '0;

        // Trap/CSR bundles are SOLO, so their scalar metadata belongs to slot 0.
        if (iq_build_i == 0) begin
          iq_entry_next.csr_op   = decoded_csr_op;
          iq_entry_next.csr_addr = decoded_csr_addr;
          iq_entry_next.csr_zimm = decoded_csr_zimm;
        end

        iq_entry_next.is_load      = decoded_is_load[iq_build_i];
        iq_entry_next.is_store     = decoded_is_store[iq_build_i];
        iq_entry_next.mem_size     = mem_size_e'(decoded_mem_size[iq_build_i]);
        iq_entry_next.mem_unsigned = decoded_mem_unsigned[iq_build_i];
        iq_entry_next.lq_idx =
            decoded_is_load[iq_build_i] ? lq_alloc_idx : '0;
        iq_entry_next.sq_idx =
            decoded_is_store[iq_build_i] ? sq_alloc_idx : '0;
      end

      // Whole-entry copy avoids Icarus field-selects on struct arrays.
      iq_alloc_entry[iq_build_i] = iq_entry_next;
    end
  end

  // Decode-detected trap packet from the frontend. The decoder tags
  // ecall/ebreak/illegal/mret; the frontend supplies cause and trap value.
  // Misalignment instead enters through an execute completion. Both paths
  // take effect only when the trapping instruction reaches the ROB head.
  assign rob_alloc_trap_valid = decoded_trap.valid;
  assign rob_alloc_trap_cause = decoded_trap.cause;
  assign rob_alloc_trap_tval  = decoded_trap.tval;
  assign rob_alloc_is_mret    = decoded_trap.is_mret;
  assign rob_alloc_is_csr     = (decoded_csr_op != rv32i_pipeline_pkg::CSR_NONE);
  assign rob_alloc_is_store   = decoded_is_store;
  assign rob_alloc_is_load    = decoded_is_load;

  // Architectural commit event and trace.
  // Committed-map updates, stale-register returns and commit_order advance
  // only with commit_fire. Trap and mret heads block their own ordinary
  // retirement while redirect effects execute. The core owns this event;
  // all retirement consumers must agree with the ROB's actual advance.
  //
  // During trap_q_valid the head has not advanced, so its trap/mret facts
  // continue to block commit. Branch recovery separately blocks retirement.
  // commit_want omits recovery and store acceptance gating so a presented
  // store can remain stable through stalls and recovery. Store retirement
  // requires acceptance now or a deferred record of acceptance on a recovery
  // edge; the latter retires on the next permitted commit cycle.
  assign commit_want[0] = rob_commit_valid[0]
                        & ~(commit_trap_valid[0] | commit_is_mret[0]);
  assign commit_fire[0] = commit_want[0]
                        & ~branch_recover_req
                        & (~rob_commit_is_store[0] | store_ready_ok);

  // Dual commit uses an ordered prefix: slot 1 can retire only with slot 0.
  // Recovery exclusion follows through commit_fire[0]. A trap or mret in slot 1
  // waits to become the head; a CSR waits for the scalar CSR side-effect path;
  // two stores cannot retire together because there is one memory write port.
  // Load retirement is bookkeeping, so two loads or a load/store pair may
  // retire together.
  assign commit_want[1] = commit_want[0]
                        & rob_commit_valid[1]
                        & ~(commit_trap_valid[1] | commit_is_mret[1])
                        & ~rob_commit_is_csr[1]
                        & ~(rob_commit_is_store[0] & rob_commit_is_store[1]);
  assign commit_fire[1] = commit_fire[0]
                        & commit_want[1]
                        & (~rob_commit_is_store[1] | store_ready_ok);

  // The LSQ presents stores from per-slot commit intent. A store becomes
  // retirable after current acceptance or a recorded deferred acceptance.
  assign store_commit_want[0] = commit_want[0] & rob_commit_is_store[0];
  assign store_commit_want[1] = commit_want[1] & rob_commit_is_store[1];
  assign store_ready_ok       = sq_mem_accept | sq_accept_deferred;

  // Per-slot commit trace taps; the trace consumer selects each retiring slot.
  assign commit_pc     = rob_commit_pc;
  assign commit_inst   = rob_commit_instr;
  assign commit_rd     = rob_commit_rd;
  assign commit_rd_wen = rob_commit_rd_we;
  assign commit_wdata  = rob_commit_result;
  assign commit_order  = rob_commit_order;

  // ================= RENAME =================
  rv32i_ss_rename u_rename (
    .clk                       (clk),
    .rst_n                     (rst_n),

    .decoded_valid             (decoded_valid),
    .decoded_slot_valid        (decoded_slot_valid),
    .decoded_ready             (decoded_ready),
    .decoded_rs1               (decoded_rs1),
    .decoded_rs2               (decoded_rs2),
    .decoded_rd                (decoded_rd),
    .decoded_rd_we             (decoded_rd_we),
    // the effective vector -- frontend branches plus the core-injected
    // predicted-return request (slot 0, solo, allocates no dest, so every
    // rename checkpoint arm and pin sees a legal requester).
    .decoded_needs_checkpoint  (dispatch_needs_checkpoint),

    .preg_slot_need           (preg_slot_need),
    .preg_avail               (preg_avail),
    .preg_alloc_reg            (preg_alloc_reg),

    .rename_ready              (rob_alloc_ready & iq_alloc_ready & dispatch_mem_ready
                                & dispatch_accept),
    .bundle_fire               (bundle_fire),
    .rename_prs1               (rename_prs1),
    .rename_prs2               (rename_prs2),
    .rename_pdst               (rename_pdst),
    .rename_stale_pdst         (rename_stale_pdst),
    .rename_rd                 (rename_rd),
    .rename_rd_we              (rename_rd_we),

    // ---- retirement gated by commit_fire (not raw commit_valid) ----
    // per-slot. The committed map applies slot 0 then slot 1, so a
    // same-rd pair leaves slot 1 owning the mapping (younger wins).
    .commit_fire              (commit_fire),
    .commit_rd_we              (rob_commit_rd_we),
    .commit_rd                 (rob_commit_rd),
    .commit_pdst               (rob_commit_pdst),
    .trap_flush         (trap_q_valid),
    .branch_recover_req        (branch_recover_req),
    .branch_recover_id         (branch_recover_id),
    .checkpoint_release_mask   (checkpoint_release_mask),

    // ---- recovery / checkpoint outputs ----
    .rename_branch_recover_alloc_list (rename_branch_recover_alloc_list),
    .rename_branch_mask        (rename_branch_mask),
    .rename_checkpoint_valid   (rename_checkpoint_valid),
    .rename_checkpoint_id      (rename_checkpoint_id)
  );

  // ================= FREE LIST =================
  rv32i_ss_free_list u_free_list (
    .clk                       (clk),
    .rst_n                     (rst_n),

    .preg_slot_need           (preg_slot_need),
    .preg_avail               (preg_avail),
    .preg_alloc_reg            (preg_alloc_reg),
    .bundle_fire               (bundle_fire),

    // ---- retirement gated by commit_fire ----
    // per-slot, applied in program order -- slot 1's stale return may
    // alias slot 0's pdst, and the ordering is what frees it rather than
    // leaking it.
    .commit_fire              (commit_fire),
    .commit_rd_we              (rob_commit_rd_we),
    .commit_pdst               (rob_commit_pdst),
    .commit_stale_pdst         (rob_commit_stale_pdst),
    .branch_recover_req        (branch_recover_req),
    .rename_branch_recover_alloc_list (rename_branch_recover_alloc_list),
    .trap_flush         (trap_q_valid)
  );

  // ================= ROB =================
  rv32i_ss_rob u_rob (
    .clk                       (clk),
    .rst_n                     (rst_n),

    .bundle_fire                (bundle_fire),
    .rob_alloc_slot_valid       (decoded_slot_valid),
    .bundle_size                (bundle_size),
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
    .rob_alloc_is_load          (rob_alloc_is_load),
    .rob_alloc_idx          (rob_alloc_idx),
    .rob_alloc_seq          (rob_alloc_seq),

    .rob_head_idx              (rob_head_idx),
    .rob_head_valid            (rob_head_valid),
    .rob_head_seq              (),   // unused by the current core
    .rob_head_done             (),
    .rob_alloc_is_csr         (rob_alloc_is_csr),
    .rob_alloc_csr_addr       (decoded_csr_addr),
    // ---- writeback from the two registered CDB beats ----
      .wb_valid                  ({cdb_q[1].valid, cdb_q[0].valid}),
      .wb_rob_idx                ({cdb_q[1].rob_idx, cdb_q[0].rob_idx}),
      .wb_rob_seq                ({cdb_q[1].rob_seq, cdb_q[0].rob_seq}),
      .wb_result                 ({cdb_q[1].result, cdb_q[0].result}),
      .wb_csr_we                 ({cdb_q[1].csr_we, cdb_q[0].csr_we}),
      .wb_csr_wdata              ({cdb_q[1].csr_wdata, cdb_q[0].csr_wdata}),
      .wb_accept                 (rob_wb_accept),
      .wb_trap_valid             ({cdb_q[1].trap_valid, cdb_q[0].trap_valid}),
      .wb_trap_cause             ({cdb_q[1].trap_cause, cdb_q[0].trap_cause}),
      .wb_trap_tval              ({cdb_q[1].trap_tval, cdb_q[0].trap_tval}),

    // Commit event and retirement taps share the core-owned ordered-prefix vector.
    .commit_fire               (commit_fire),
    .commit_valid              (rob_commit_valid),       // per-slot valid&&done; feeds commit_fire
    .commit_rd_we              (rob_commit_rd_we),
    .commit_rd                 (rob_commit_rd),
    .commit_pdst               (rob_commit_pdst),
    .commit_stale_pdst         (rob_commit_stale_pdst),
    .commit_is_csr             (rob_commit_is_csr),
    .commit_is_store           (rob_commit_is_store),
    .commit_is_load            (rob_commit_is_load),
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

    // The ROB exposes precise trap/mret metadata from the committing head;
    // the registered trap path below consumes it.
    .commit_trap_valid         (commit_trap_valid),
    .commit_trap_cause         (commit_trap_cause),
    .commit_trap_tval          (commit_trap_tval),
    .commit_is_mret            (commit_is_mret)

  );

  // ================= ISSUE QUEUE =================
  rv32i_ss_iq u_iq (
    .clk          (clk),
    .rst_n        (rst_n),
    .trap_flush        (trap_q_valid),     // full IQ flush on a taken trap/mret
    .branch_recover_req(branch_recover_req),
    .recover_rob_idx (recover_q_rob_idx),

    .bundle_fire  (bundle_fire),
    .bundle_size  (bundle_size),
    .iq_alloc_slot_valid (decoded_slot_valid),
    .iq_alloc_entry (iq_alloc_entry),
    .iq_alloc_ready (iq_alloc_ready),
    .ready_vec    (ready_vec),
    .rob_head_idx (rob_head_idx),
    .issue_valid  (iq_select_valid),
    .issue_entry  (iq_select_entry),
    .issue_unit   (iq_select_unit),
    .issue_accept (iq_select_accept),
    .alu0_fu_ready(alu0_fu_ready),
    .alu1_fu_ready(alu1_fu_ready),
    .muldiv_fu_ready(muldiv_fu_ready),
    .lsu_fu_ready  (lsu_fu_ready)
  );

  // Complete picker output is registered BEFORE all PRF addresses/bypass.
  // Only this old registered bundle can enqueue execute inputs this cycle.
  rv32i_ss_select_pipe u_select_pipe (
    .clk(clk), .rst_n(rst_n),
    .trap_flush(trap_q_valid),
    .branch_recover_req(branch_recover_req),
    .recover_rob_idx(recover_q_rob_idx), .rob_head_idx(rob_head_idx),
    .iq_select_valid(iq_select_valid), .iq_select_entry(iq_select_entry),
    .iq_select_unit(iq_select_unit), .iq_select_accept(iq_select_accept),
    .alu0_fu_ready(alu0_fu_ready), .alu1_fu_ready(alu1_fu_ready),
    .muldiv_fu_ready(muldiv_fu_ready), .lsu_fu_ready(lsu_fu_ready),
    .issue_valid(issue_valid), .issue_entry(issue_entry),
    .issue_unit(issue_unit), .issue_accept(issue_accept)
  );

  // ================= LOAD/STORE QUEUE =================
  rv32i_ss_lsq u_lsq (
    .clk                   (clk),
    .rst_n                 (rst_n),

    .lq_alloc_fire         (lq_alloc_fire),
    .lq_alloc_ready        (lq_alloc_ready),
    .lq_alloc_idx          (lq_alloc_idx),
    .lq_alloc_rob_idx      (rob_alloc_idx[mem_slot_idx]),
    .lq_alloc_rob_seq      (rob_alloc_seq[mem_slot_idx]),
    .lq_alloc_pdst         (rename_pdst[mem_slot_idx]),
    .lq_alloc_rd_wen       (rename_rd_we[mem_slot_idx]),
    .lq_alloc_mem_size     (decoded_mem_size[mem_slot_idx]),
    .lq_alloc_mem_unsigned (decoded_mem_unsigned[mem_slot_idx]),

    .sq_alloc_fire         (sq_alloc_fire),
    .sq_alloc_ready        (sq_alloc_ready),
    .sq_alloc_idx          (sq_alloc_idx),
    .sq_alloc_rob_idx      (rob_alloc_idx[mem_slot_idx]),
    .sq_alloc_rob_seq      (rob_alloc_seq[mem_slot_idx]),
    .sq_alloc_data_preg    (rename_prs2[mem_slot_idx]),

    .lq_deposit_fire      (load_agen_fire),
    .lq_deposit_idx        (agen_exec_entry.lq_idx),
    .lq_deposit_addr       (agen_addr),
    .lq_deposit_inert      (load_misalign),

    .sq_deposit_fire      (store_agen_fire),
    .sq_deposit_idx        (agen_exec_entry.sq_idx),
    .sq_deposit_addr       (agen_addr),
    .sq_deposit_data       (store_wdata_next),
    .sq_deposit_data_valid (sq_deposit_data_valid),
    .sq_deposit_be         (store_be_next),
    .sq_deposit_inert      (store_misalign),
    .sq_deposit_deferred_pending (sq_deposit_deferred_pending),

    .sq_data_wb_fire       (sq_data_wb_fire),
    .sq_data_wb_pdst       (sq_data_wb_pdst),
    .sq_data_wb_value      (sq_data_wb_value),

    .sq_complete           (sq_complete),
    .sq_complete_accept    (sq_complete_accept),

    // the LSQ commit seam is per-slot. It derives slot p's expected ROB
    // index as rob_head_idx + p, so there is no separate commit-index port.
    .commit_fire           (commit_fire),
    .commit_is_store       (rob_commit_is_store),
    .commit_is_load        (rob_commit_is_load),

    .store_commit_want     (store_commit_want),
    .sq_mem_accept         (sq_mem_accept),
    .sq_accept_deferred    (sq_accept_deferred),

    .branch_recover_req    (branch_recover_req),
    .recover_rob_idx       (recover_q_rob_idx),
    .rob_head_idx          (rob_head_idx),
    .trap_flush            (trap_q_valid),

    .dmem_valid            (dmem_valid),
    .dmem_we               (dmem_we),
    .dmem_be               (dmem_be),
    .dmem_addr             (dmem_addr),
    .dmem_wdata            (dmem_wdata),
    .dmem_ready            (dmem_ready),
    .dmem_rvalid           (dmem_rvalid),
    .dmem_rdata            (dmem_rdata),

    .lq_complete           (lq_complete),
    .cdb_grant_lq          (cdb_grant_lq)
  );

  // Accepted CDB writeback is the only late-data capture authority. A stale
  // packet can share a reallocated pdst, so raw cdb_q.valid is insufficient;
  // the ROB's {rob_idx,rob_seq} acceptance verdict is load-bearing here.
  assign sq_data_wb_fire[0] = rob_wb_accept[0] && cdb_q[0].rd_wen;
  assign sq_data_wb_fire[1] = rob_wb_accept[1] && cdb_q[1].rd_wen;
  assign sq_data_wb_pdst[0] = cdb_q[0].pdst;
  assign sq_data_wb_pdst[1] = cdb_q[1].pdst;
  assign sq_data_wb_value[0] = cdb_q[0].result;
  assign sq_data_wb_value[1] = cdb_q[1].result;

  // Store address generation may run before data is ready. Ready data
  // deposits directly and completes through AGU; otherwise the SQ captures
  // the later accepted producer value and emits one deferred completion.
  assign store_data_ready_at_exec = agen_exec_store_data_valid;
  assign store_data_ready_at_issue = store_data_ready_at_exec;
  assign agen_issue_entry = agen_exec_entry;
  assign sq_deposit_data_valid = store_data_ready_at_exec;
  assign sq_deposit_deferred_pending =
      !store_misalign && !store_data_ready_at_exec;
  assign agen_direct_complete_fire = load_misalign ||
      (store_agen_fire && (store_misalign || store_data_ready_at_exec));

  // ================= PER-FU ISSUE + DUAL-CDB ARBITRATION =================
  // The IQ selects the oldest ready uop whose FU can accept it; commit still
  // retires in ROB-head order. FU completions are held until a CDB transport
  // lane grants each packet, then the ROB validates
  // {rob_idx, rob_seq} per lane.

  // never issue/execute during a recovery cycle.
  // Dispatch is already gated by ~branch_recover_req; gating issue the same way
  // keeps a wrong-path uop from executing in the registered recovery bubble.
  // same for the trap full-flush cycle (don't issue from an IQ being flushed).
  // u_select_pipe adds whole-bundle downstream capacity to the recovery gate.
  // iq_select_accept removes IQ entries into select_q; issue_accept transfers
  // that registered bundle through PRF/bypass into the execute-input banks.
  // Occupancy-only IQ admission: a full registered input is not capacity,
  // including the cycle an occupant executes. Do not fold CDB grant, AGEN
  // accept, exec pop, or next-cycle free-space into fu_ready.
  assign alu0_fu_ready = !(alu0_in_q[0].valid && alu0_in_q[1].valid);
  assign alu1_fu_ready = !(alu1_in_q[0].valid && alu1_in_q[1].valid);
  assign muldiv_fu_ready = !md_in_q.valid;
  assign lsu_fu_ready  = !(agen_in_q[0].valid && agen_in_q[1].valid);
  assign issue_fire = issue_accept ? issue_valid : 2'b00;

  // ---- unpack the issued uop ----
  assign issue_pc[0]       = issue_entry[0].pc;
  assign issue_imm[0]      = issue_entry[0].imm;
  assign issue_prs1[0]     = issue_entry[0].prs1;
  assign issue_prs2[0]     = issue_entry[0].prs2;
  assign issue_pdst[0]     = issue_entry[0].pdst;
  assign issue_src1_sel[0] = issue_entry[0].src1_sel;
  assign issue_src2_sel[0] = issue_entry[0].src2_sel;
  assign issue_op_class[0] = issue_entry[0].op_class;
  assign issue_fu_class[0] = issue_entry[0].fu_class;
  assign issue_branch_op[0] = issue_entry[0].branch_op;
  assign issue_rd_wen[0]   = issue_entry[0].rd_wen;
  assign issue_csr_addr[0] = issue_entry[0].csr_addr;
  assign issue_csr_zimm[0] = issue_entry[0].csr_zimm;
  assign issue_csr_op[0]   = issue_entry[0].csr_op;

  // Position-1 unpack.
  assign issue_pc[1]       = issue_entry[1].pc;
  assign issue_imm[1]      = issue_entry[1].imm;
  assign issue_prs1[1]     = issue_entry[1].prs1;
  assign issue_prs2[1]     = issue_entry[1].prs2;
  assign issue_pdst[1]     = issue_entry[1].pdst;
  assign issue_src1_sel[1] = issue_entry[1].src1_sel;
  assign issue_src2_sel[1] = issue_entry[1].src2_sel;
  assign issue_op_class[1] = issue_entry[1].op_class;
  assign issue_fu_class[1] = issue_entry[1].fu_class;
  assign issue_branch_op[1] = issue_entry[1].branch_op;
  assign issue_rd_wen[1]   = issue_entry[1].rd_wen;
  assign issue_csr_addr[1] = issue_entry[1].csr_addr;
  assign issue_csr_zimm[1] = issue_entry[1].csr_zimm;
  assign issue_csr_op[1]   = issue_entry[1].csr_op;

  // ---- classify + per-FU fire / control resolve / branch-recovery decision ----
  assign issue_is_branch[0]      = (issue_op_class[0] == OOO_OP_BRANCH);
  assign issue_is_jump[0]        = (issue_op_class[0] == OOO_OP_JUMP);
  assign issue_is_control[0]     = issue_is_branch[0] | issue_is_jump[0];
  assign issue_is_lsu[0]         = (issue_fu_class[0] == OOO_FU_LSU);
  assign issue_is_csr[0]         = (issue_entry[0].csr_op != rv32i_pipeline_pkg::CSR_NONE);
  // Position-1 classify.
  assign issue_is_branch[1]      = (issue_op_class[1] == OOO_OP_BRANCH);
  assign issue_is_jump[1]        = (issue_op_class[1] == OOO_OP_JUMP);
  assign issue_is_control[1]     = issue_is_branch[1] | issue_is_jump[1];
  assign issue_is_lsu[1]         = (issue_fu_class[1] == OOO_FU_LSU);
  assign issue_is_csr[1]         = (issue_entry[1].csr_op != rv32i_pipeline_pkg::CSR_NONE);
  assign load_agen_fire        = agen_exec_fire & agen_exec_entry.is_load;
  assign store_agen_fire       = agen_exec_fire & agen_exec_entry.is_store;
  // dispatch-side JAL classification (see the declaration block note).
  assign decoded_slot0_is_jal = (decoded_op_class[0] == OOO_OP_JUMP) &&
                                (decoded_src1_sel[0] == OOO_SRC_PC);
  assign jal_deser_target     = decoded_pc[0] + decoded_imm[0];
  assign decoded_jal_deser_ok = decoded_slot0_is_jal &&
                                (jal_deser_target[1:0] == 2'b00);
  assign jal_deser_fire       = bundle_fire && decoded_slot_valid[0] &&
                                decoded_jal_deser_ok;

  // issue-side split. A de-serialized JAL's issue must be INVISIBLE to
  // the serialization machinery: it must neither clear jump_inflight_q (an
  // older still-queued JAL issuing under a younger in-flight JALR would
  // unfreeze dispatch early) nor redirect (its redirect already fired at
  // dispatch; a second one would re-dispatch the target path).
  assign issue_is_jal_deser = issue_is_jump[0] &&
                              (issue_src1_sel[0] == OOO_SRC_PC) &&
                              (((issue_pc[0] + issue_imm[0]) & 32'h0000_0003)
                               == 32'h0);
  // A jump carrying pred_taken is a predicted return. It validates its
  // actual target through branch recovery at execute, without a second
  // redirect on a match or any update to jump_inflight_q.
  assign issue_is_jump_predicted  = issue_is_jump[0] && issue_entry[0].pred_taken;
  assign issue_is_jump_serialized = issue_is_jump[0] && !issue_is_jal_deser
                                    && !issue_is_jump_predicted;

  assign jump_resolve_fire    = alu0_exec_fire & alu0_exec_is_jump_ser;
  assign jump_pred_resolve_fire = alu0_exec_fire & alu0_exec_is_jump_pred;
  assign jump_alloc_fire      = bundle_fire &&
                                (decoded_op_class[0] == OOO_OP_JUMP) &&
                                !decoded_jal_deser_ok &&
                                !ras_pred_want;

  // decode-side RAS classification (slot 0; jumps are formation-solo).
  assign decoded_slot0_is_jalr = (decoded_op_class[0] == OOO_OP_JUMP) &&
                                 (decoded_src1_sel[0] == OOO_SRC_REG);
  assign decoded_link_rd0  = (decoded_rd[0]  == arch_reg_t'(1)) ||
                             (decoded_rd[0]  == arch_reg_t'(5));
  assign decoded_link_rs10 = (decoded_rs1[0] == arch_reg_t'(1)) ||
                             (decoded_rs1[0] == arch_reg_t'(5));
  // ISA link-register hint table: JAL pushes iff rd is a link reg;
  // JALR pushes iff rd is a link reg (all three rd-link rows push), and
  // pops iff rs1 is a link reg UNLESS rd == rs1 (the push-only row). The
  // pop-then-push row does both in one instruction: the update logic below
  // chains them (net tos unchanged, top overwritten with pc+4).
  assign decoded_ras_push = decoded_slot_valid[0] &&
                            (decoded_slot0_is_jal || decoded_slot0_is_jalr) &&
                            decoded_link_rd0;
  assign decoded_ras_pop  = decoded_slot_valid[0] && decoded_slot0_is_jalr &&
                            decoded_link_rs10 &&
                            (!decoded_link_rd0 ||
                             (decoded_rd[0] != decoded_rs1[0]));
  // Return prediction has two timing cases. A learned site carries its
  // request-time RAS target in decoded_pred_target; a cold site samples the
  // core-owned RAS at dispatch and redirects then. Both allocate one
  // checkpoint and pop the stack once at dispatch.
  assign ras_fetch_pred_want = decoded_pred_taken[0] && decoded_ras_pop &&
                               !(decoded_rd_we[0] && (decoded_rd[0] != '0));
  assign ras_decode_pred_want = decoded_ras_pop && (ras_count_q != '0) &&
                                !(decoded_rd_we[0] && (decoded_rd[0] != '0)) &&
                                !ras_fetch_pred_want;
  assign ras_pred_want   = ras_fetch_pred_want || ras_decode_pred_want;
  assign ras_pred_fire   = bundle_fire && ras_pred_want;
  assign ras_decode_redirect_fire = bundle_fire && ras_decode_pred_want;
  assign ras_pred_target = ras_q[ras_tos_q];
  assign ras_fetch_valid = (ras_count_q != '0);
  assign ras_fetch_target = ras_q[ras_tos_q];
  assign bp_return_update_valid = bundle_fire && decoded_ras_pop &&
                                  !(decoded_rd_we[0] &&
                                    (decoded_rd[0] != '0));
  assign bp_return_update_pc = decoded_pc[0];
  // The injected checkpoint request: rename and the IQ-entry build consume
  // this EFFECTIVE vector; the frontend payload and every offer-side
  // formation pin keep reading the raw port.
  assign dispatch_needs_checkpoint = decoded_needs_checkpoint
                                     | {1'b0, ras_pred_want};
  assign csr_issue_fire       = alu0_exec_fire
                                & (alu0_exec_entry.csr_op != rv32i_pipeline_pkg::CSR_NONE);
  assign csr_alloc_fire       = bundle_fire && rob_alloc_is_csr;
  // Expose registered recovery state to the recovery consumers.
  assign branch_recover_req   = recover_q_valid;
  assign branch_recover_id    = recover_q_ckpt_id;
  // muldiv_kill the in-flight muldiv iff it is YOUNGER than the recovering branch
  // by ROB ring distance (same predicate as the IQ + ROB rollback). muldiv_busy
  // gates out the idle case; an older muldiv (age <= branch age) is spared.
  // a taken trap squashes EVERYTHING younger than the head, so every
  // in-flight muldiv (necessarily younger than the trapping head) is killed too,
  // not just younger-than-a-recovering-branch.
  assign muldiv_kill           = (branch_recover_req && muldiv_busy &&
                                ((muldiv_rob_idx_q - rob_head_idx) >
                                 (recover_q_rob_idx - rob_head_idx)))
                                || (trap_q_valid && muldiv_busy);
  assign fallthrough_pc      = alu0_exec_entry.pc + 32'd4;

  // Per-POSITION operand selection (src-sel muxes; position 0 reads PRF
  // ports 1/2, position 1 reads ports 3/4). Placed ABOVE the router so the
  // dataflow reads forward: unpack -> operands -> position->unit router.
  always_comb begin
    case (issue_src1_sel[0])
      OOO_SRC_PC:   operand_a[0] = issue_pc[0];
      OOO_SRC_ZERO: operand_a[0] = '0;
      default:      operand_a[0] = byp_rdata1;   // OOO_SRC_REG (bypassed)
    endcase
    case (issue_src2_sel[0])
      OOO_SRC_IMM:  operand_b[0] = issue_imm[0];
      OOO_SRC_PC:   operand_b[0] = issue_pc[0];
      OOO_SRC_ZERO: operand_b[0] = '0;
      default:      operand_b[0] = byp_rdata2;   // OOO_SRC_REG (bypassed)
    endcase

    case (issue_src1_sel[1])
      OOO_SRC_PC:   operand_a[1] = issue_pc[1];
      OOO_SRC_ZERO: operand_a[1] = '0;
      default:      operand_a[1] = byp_rdata3;   // OOO_SRC_REG (bypassed)
    endcase
    case (issue_src2_sel[1])
      OOO_SRC_IMM:  operand_b[1] = issue_imm[1];
      OOO_SRC_PC:   operand_b[1] = issue_pc[1];
      OOO_SRC_ZERO: operand_b[1] = '0;
      default:      operand_b[1] = byp_rdata4;   // OOO_SRC_REG (bypassed)
    endcase
  end

  // Translate age-ordered grant positions into FU *enqueue* strobes.
  // Execution is a later, distinct event (exec_fire) from a registered slot.
  always_comb begin
    alu0_issue_fire  = 1'b0;
    alu1_issue_fire  = 1'b0;
    alu0_issue_entry = '0;
    alu1_issue_entry = '0;
    alu0_operand_a   = '0;
    alu0_operand_b   = '0;
    alu1_operand_a   = '0;
    alu1_operand_b   = '0;
    muldiv_issue_fire  = 1'b0;
    muldiv_issue_entry = '0;
    muldiv_operand_a   = '0;
    muldiv_operand_b   = '0;
    agen_issue_fire    = 1'b0;
    agen_enq_entry     = '0;
    agen_enq_a         = '0;
    agen_enq_b         = '0;
    agen_enq_store_data = '0;
    agen_operand_a     = '0;
    agen_operand_b     = '0;
    agen_store_data    = '0;

    if (issue_fire[0]) begin
      case (issue_unit[0])
        ISSUE_UNIT_ALU0: begin
          alu0_issue_fire  = 1'b1;
          alu0_issue_entry = issue_entry[0];
          alu0_operand_a   = operand_a[0];
          alu0_operand_b   = operand_b[0];
        end
        ISSUE_UNIT_ALU1: begin
          alu1_issue_fire  = 1'b1;
          alu1_issue_entry = issue_entry[0];
          alu1_operand_a   = operand_a[0];
          alu1_operand_b   = operand_b[0];
        end
        ISSUE_UNIT_MULDIV: begin
          muldiv_issue_fire  = 1'b1;
          muldiv_issue_entry = issue_entry[0];
          muldiv_operand_a   = operand_a[0];
          muldiv_operand_b   = operand_b[0];
        end
        ISSUE_UNIT_AGEN: begin
          agen_issue_fire  = 1'b1;
          agen_enq_entry   = issue_entry[0];
          agen_enq_a       = operand_a[0];
          agen_enq_b       = operand_b[0];
          agen_enq_store_data = byp_rdata2;
          agen_operand_a   = operand_a[0];
          agen_operand_b   = operand_b[0];
          agen_store_data  = byp_rdata2;
        end
        default: begin end
      endcase
    end
    if (issue_fire[1]) begin
      case (issue_unit[1])
        ISSUE_UNIT_ALU0: begin
          alu0_issue_fire  = 1'b1;
          alu0_issue_entry = issue_entry[1];
          alu0_operand_a   = operand_a[1];
          alu0_operand_b   = operand_b[1];
        end
        ISSUE_UNIT_ALU1: begin
          alu1_issue_fire  = 1'b1;
          alu1_issue_entry = issue_entry[1];
          alu1_operand_a   = operand_a[1];
          alu1_operand_b   = operand_b[1];
        end
        ISSUE_UNIT_MULDIV: begin
          muldiv_issue_fire  = 1'b1;
          muldiv_issue_entry = issue_entry[1];
          muldiv_operand_a   = operand_a[1];
          muldiv_operand_b   = operand_b[1];
        end
        ISSUE_UNIT_AGEN: begin
          agen_issue_fire  = 1'b1;
          agen_enq_entry   = issue_entry[1];
          agen_enq_a       = operand_a[1];
          agen_enq_b       = operand_b[1];
          agen_enq_store_data = byp_rdata4;
          agen_operand_a   = operand_a[1];
          agen_operand_b   = operand_b[1];
          agen_store_data  = byp_rdata4;
        end
        default: begin end
      endcase
    end
  end

  // ---- exec pick: oldest surviving registered occupant, no fall-through ----
  // Pure helpers: pass all sampled state explicitly. Non-local reads inside
  // functions block Formality reference elaboration (FMR_VLOG-091/FM-089).
  function automatic logic slot_is_younger(
      input rob_idx_t idx, input rob_idx_t head_idx, input rob_idx_t recover_idx);
    slot_is_younger = ((idx - head_idx) > (recover_idx - head_idx));
  endfunction

  function automatic logic src_value_present(
      input phys_reg_t prs, input logic [OOO_PHYS_REGS-1:0] ready_state,
      input logic [1:0] wb_accept, input completion_packet_t [1:0] transit,
      input exec_result_slot_t [1:0] alu0_results,
      input exec_result_slot_t [1:0] alu1_results);
    src_value_present = (prs == '0) || ready_state[prs];
    for (int bi = 0; bi < 2; bi++) begin
      if (wb_accept[bi] && transit[bi].rd_wen && (transit[bi].pdst == prs) && (prs != '0))
        src_value_present = 1'b1;
      if (alu0_results[bi].live && alu0_results[bi].valid && alu0_results[bi].pkt.rd_wen &&
          (alu0_results[bi].pkt.pdst == prs) && (prs != '0))
        src_value_present = 1'b1;
      if (alu1_results[bi].live && alu1_results[bi].valid && alu1_results[bi].pkt.rd_wen &&
          (alu1_results[bi].pkt.pdst == prs) && (prs != '0))
        src_value_present = 1'b1;
    end
  endfunction

  function automatic logic oldest_of_two(
      input logic v0, input logic v1,
      input rob_idx_t i0, input rob_idx_t i1, input rob_idx_t head_idx);
    if (v0 && v1)
      oldest_of_two = ((i0 - head_idx) <= (i1 - head_idx)) ? 1'b0 : 1'b1;
    else
      oldest_of_two = v0 ? 1'b0 : 1'b1;
  endfunction

  always_comb begin
    alu0_in_push_idx = alu0_in_q[0].valid ? 1'b1 : 1'b0;
    alu1_in_push_idx = alu1_in_q[0].valid ? 1'b1 : 1'b0;
    agen_in_push_idx = agen_in_q[0].valid ? 1'b1 : 1'b0;
    alu0_res_free = !(alu0_res_q[0].valid && alu0_res_q[1].valid);
    alu1_res_free = !(alu1_res_q[0].valid && alu1_res_q[1].valid);
    alu0_res_push_idx = alu0_res_q[0].valid ? 1'b1 : 1'b0;
    alu1_res_push_idx = alu1_res_q[0].valid ? 1'b1 : 1'b0;

    alu0_res_offer_valid = alu0_res_q[0].valid || alu0_res_q[1].valid;
    alu1_res_offer_valid = alu1_res_q[0].valid || alu1_res_q[1].valid;
    if (alu0_res_q[0].valid && alu0_res_q[1].valid)
      alu0_res_offer_idx = alu0_res_q[0].older ? 1'b0 : 1'b1;
    else
      alu0_res_offer_idx = alu0_res_q[0].valid ? 1'b0 : 1'b1;
    if (alu1_res_q[0].valid && alu1_res_q[1].valid)
      alu1_res_offer_idx = alu1_res_q[0].older ? 1'b0 : 1'b1;
    else
      alu1_res_offer_idx = alu1_res_q[0].valid ? 1'b0 : 1'b1;

    exec_ok = ~(branch_recover_req | trap_q_valid);

    alu0_exec_idx = oldest_of_two(alu0_in_q[0].valid, alu0_in_q[1].valid,
                                  alu0_in_q[0].uop.rob_idx, alu0_in_q[1].uop.rob_idx, rob_head_idx);
    alu1_exec_idx = oldest_of_two(alu1_in_q[0].valid, alu1_in_q[1].valid,
                                  alu1_in_q[0].uop.rob_idx, alu1_in_q[1].uop.rob_idx, rob_head_idx);
    agen_exec_idx = oldest_of_two(agen_in_q[0].valid, agen_in_q[1].valid,
                                  agen_in_q[0].uop.rob_idx, agen_in_q[1].uop.rob_idx, rob_head_idx);

    alu0_exec_entry = alu0_in_q[alu0_exec_idx].uop;
    alu1_exec_entry = alu1_in_q[alu1_exec_idx].uop;
    agen_exec_entry = agen_in_q[agen_exec_idx].uop;
    md_exec_entry   = md_in_q.uop;
    alu0_exec_a = alu0_in_q[alu0_exec_idx].op_a;
    alu0_exec_b = alu0_in_q[alu0_exec_idx].op_b;
    alu1_exec_a = alu1_in_q[alu1_exec_idx].op_a;
    alu1_exec_b = alu1_in_q[alu1_exec_idx].op_b;
    agen_exec_a = agen_in_q[agen_exec_idx].op_a;
    agen_exec_b = agen_in_q[agen_exec_idx].op_b;
    md_exec_a   = md_in_q.op_a;
    md_exec_b   = md_in_q.op_b;

    alu0_exec_is_jal_deser = (alu0_exec_entry.op_class == OOO_OP_JUMP) &&
                             (alu0_exec_entry.src1_sel == OOO_SRC_PC) &&
                             (((alu0_exec_entry.pc + alu0_exec_entry.imm) & 32'h3) == 32'h0);
    alu0_exec_is_jump_pred = (alu0_exec_entry.op_class == OOO_OP_JUMP) &&
                             alu0_exec_entry.pred_taken;
    alu0_exec_is_jump_ser  = (alu0_exec_entry.op_class == OOO_OP_JUMP) &&
                             !alu0_exec_is_jal_deser && !alu0_exec_is_jump_pred;

    alu0_exec_fire = exec_ok && (alu0_in_q[0].valid || alu0_in_q[1].valid) &&
                     alu0_res_free;
    solo_alu0_exec = alu0_exec_fire &&
                     ((alu0_exec_entry.op_class == OOO_OP_JUMP) ||
                      (alu0_exec_entry.csr_op != rv32i_pipeline_pkg::CSR_NONE));
    alu1_exec_fire = exec_ok && !solo_alu0_exec &&
                     (alu1_in_q[0].valid || alu1_in_q[1].valid) &&
                     alu1_res_free;
    md_exec_fire = exec_ok && !solo_alu0_exec && md_in_q.valid &&
                   !muldiv_busy && !muldiv_complete.valid;
    agen_exec_fire = exec_ok && !solo_alu0_exec &&
                     (agen_in_q[0].valid || agen_in_q[1].valid) &&
                     !agen_complete.valid;

    muldiv_start = md_exec_fire;

    agen_exec_store_data_valid = agen_in_q[agen_exec_idx].store_data_valid;
    agen_exec_store_data = agen_in_q[agen_exec_idx].store_data;
    if (agen_exec_fire && agen_exec_entry.is_store &&
        agen_in_q[agen_exec_idx].store_data_pending) begin
      for (int wb_i = 0; wb_i < 2; wb_i++) begin
        if (sq_data_wb_fire[wb_i] &&
            (sq_data_wb_pdst[wb_i] == agen_in_q[agen_exec_idx].store_prs2) &&
            (sq_data_wb_pdst[wb_i] != '0)) begin
          agen_exec_store_data_valid = 1'b1;
          agen_exec_store_data = sq_data_wb_value[wb_i];
        end
      end
    end
  end

  always_comb begin
    alu0_complete = '0;
    alu1_complete = '0;
    alu0_holder_live_q = 1'b0;
    alu1_holder_live_q = 1'b0;
    if (alu0_res_offer_valid) begin
      alu0_complete = alu0_res_q[alu0_res_offer_idx].pkt;
      alu0_holder_live_q = alu0_res_q[alu0_res_offer_idx].live;
    end
    if (alu1_res_offer_valid) begin
      alu1_complete = alu1_res_q[alu1_res_offer_idx].pkt;
      alu1_holder_live_q = alu1_res_q[alu1_res_offer_idx].live;
    end
  end

  // Build per-ALU candidates for recovery and checkpoint release.
  // A branch mispredicts when direction differs, or when a taken prediction
  // has the wrong exact pc + imm target. Recovery selects the actual target
  // when taken and pc + 4 otherwise. Misaligned taken targets follow the trap
  // path; trap flush reclaims their checkpoints.
  //
  // Candidate 0 also resolves predicted returns, which execute on ALU0 alone.
  // Their actual target is (rs1 + imm) & ~1. An exact match releases the
  // checkpoint without redirect; a mismatch recovers to the actual target.
  // A misaligned actual return target follows the precise trap path.
  logic  [1:0] branch_mispredict;
  logic        jump_pred_resolve_alu0;
  word_t       jump_actual_target;
  always_comb begin
    branch_mispredict[0] = (branch_taken[0] != alu0_exec_entry.pred_taken) ||
                           (branch_taken[0] &&
                            (alu0_exec_entry.pred_target !=
                             (alu0_exec_entry.pc + alu0_exec_entry.imm)));
    branch_mispredict[1] = (branch_taken[1] != alu1_exec_entry.pred_taken) ||
                           (branch_taken[1] &&
                            (alu1_exec_entry.pred_target !=
                             (alu1_exec_entry.pc + alu1_exec_entry.imm)));

    jump_pred_resolve_alu0 = alu0_exec_fire && alu0_exec_is_jump_pred;
    jump_actual_target = (alu0_exec_a + alu0_exec_entry.imm)
                         & 32'hffff_fffe;

    branch_candidate = '0;
    branch_candidate[0].resolve_valid =
        (alu0_exec_fire && (alu0_exec_entry.op_class == OOO_OP_BRANCH))
        || jump_pred_resolve_alu0;
    branch_candidate[0].recover_valid = branch_candidate[0].resolve_valid &&
                                        (jump_pred_resolve_alu0
                                         ? (alu0_exec_entry.pred_target != jump_actual_target)
                                         : branch_mispredict[0]) &&
                                        !(jump_pred_resolve_alu0
                                          ? control_target_misalign[0]
                                          : (branch_taken[0] && control_target_misalign[0]));
    branch_candidate[0].correct_valid = branch_candidate[0].resolve_valid &&
                                        !(jump_pred_resolve_alu0
                                          ? (alu0_exec_entry.pred_target != jump_actual_target)
                                          : branch_mispredict[0]) &&
                                        !(jump_pred_resolve_alu0
                                          ? control_target_misalign[0]
                                          : (branch_taken[0] && control_target_misalign[0]));
    branch_candidate[0].checkpoint_id = alu0_exec_entry.checkpoint_id;
    branch_candidate[0].rob_idx       = alu0_exec_entry.rob_idx;
    branch_candidate[0].target        = jump_pred_resolve_alu0
                                        ? jump_actual_target
                                        : (branch_taken[0]
                                           ? (alu0_exec_entry.pc + alu0_exec_entry.imm)
                                           : (alu0_exec_entry.pc + 32'd4));
    branch_candidate[1].resolve_valid = alu1_exec_fire && (alu1_exec_entry.op_class == OOO_OP_BRANCH);
    branch_candidate[1].recover_valid = branch_candidate[1].resolve_valid &&
                                        branch_mispredict[1] &&
                                        !(branch_taken[1] && control_target_misalign[1]);
    branch_candidate[1].correct_valid = branch_candidate[1].resolve_valid &&
                                        !branch_mispredict[1] &&
                                        !(branch_taken[1] && control_target_misalign[1]);
    branch_candidate[1].checkpoint_id = alu1_exec_entry.checkpoint_id;
    branch_candidate[1].rob_idx       = alu1_exec_entry.rob_idx;
    branch_candidate[1].target        = branch_taken[1]
                                        ? (alu1_exec_entry.pc + alu1_exec_entry.imm)
                                        : (alu1_exec_entry.pc + 32'd4);
  end

  // Branch predictor training occurs at execute. ALU0 has priority when
  // both ALUs resolve a branch in the same cycle. Dropping the other update
  // or training from a wrong-path branch affects accuracy only.
  // Candidate 0 must be classified as a branch before it trains: predicted
  // returns share its resolution channel but must not write branch targets.
  // An ALU1 branch may train while ALU0 resolves a predicted return.
  always_comb begin
    bp_update_valid  = 1'b0;
    bp_update_pc     = '0;
    bp_update_taken  = 1'b0;
    bp_update_target = '0;
    if (branch_candidate[0].resolve_valid &&
        (alu0_exec_entry.op_class == OOO_OP_BRANCH)) begin
      bp_update_valid  = 1'b1;
      bp_update_pc     = alu0_exec_entry.pc;
      bp_update_taken  = branch_taken[0];
      bp_update_target = alu0_exec_entry.pc + alu0_exec_entry.imm;
    end else if (branch_candidate[1].resolve_valid) begin
      bp_update_valid  = 1'b1;
      bp_update_pc     = alu1_exec_entry.pc;
      bp_update_taken  = branch_taken[1];
      bp_update_target = alu1_exec_entry.pc + alu1_exec_entry.imm;
    end
  end

  rv32i_ss_branch_combiner u_branch_combiner (
    .clk                     (clk),
    .branch_candidate        (branch_candidate),
    .rob_head_idx            (rob_head_idx),
    .selected_recovery       (selected_recovery),
    .checkpoint_release_mask (checkpoint_release_mask)
  );


  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      recover_q_valid    <= 1'b0;
      recover_q_ckpt_id  <= '0;
      recover_q_rob_idx  <= '0;
      recover_q_target   <= '0;
    end else begin
      recover_q_valid <= selected_recovery.recover_valid;
      if (selected_recovery.recover_valid) begin
        recover_q_ckpt_id  <= selected_recovery.checkpoint_id;
        recover_q_rob_idx  <= selected_recovery.rob_idx;
        recover_q_target   <= selected_recovery.target;
      end
    end
  end

  // Combinational decision at the commit boundary; edge-detected so it latches
  // exactly one registered event even though the head holds the trap one more
  // cycle while the flush is in flight.
  assign trap_commit_event = commit_trap_valid[0] || commit_is_mret[0];
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
        trap_q_is_mret <= commit_is_mret[0];
        trap_q_pc      <= rob_commit_pc[0];
        trap_q_cause   <= commit_trap_cause[0];
        trap_q_tval    <= commit_trap_tval[0];

        if (commit_is_mret[0]) begin
          trap_q_target <= csr_mepc;
        end else begin
          trap_q_target <= {csr_mtvec[31:2], 2'b00};
        end
      end
    end
  end

  // track the executing muldiv's rob_idx so muldiv_kill can compare
  // ages. A drain-and-refill captures the replacement after the old holder
  // is granted, so a new start takes priority over grant-only clearing.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      muldiv_rob_idx_q <= '0;
    end else if (muldiv_kill) begin
      muldiv_rob_idx_q <= '0;
    end else if (muldiv_start) begin
      muldiv_rob_idx_q <= md_exec_entry.rob_idx;
    end else if (cdb_grant_muldiv) begin
      muldiv_rob_idx_q <= '0;
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
    end else if (jump_resolve_fire) begin
      // serialized jumps only -- a de-serialized JAL's issue must not
      // clear the gate an in-flight JALR is holding (see the split note).
      jump_inflight_q <= 1'b0;
    end else if (jump_alloc_fire) begin
      jump_inflight_q <= 1'b1;
    end
  end

  // ---------------- RAS update / restore ----------------
  // Next-state for the pointer pair, chained pop-then-push so the both-link
  // rd != rs1 JALR row nets to a top overwrite. Updates fire at DISPATCH
  // only (a held bundle updates nothing, matching rename's pure-demand
  // rule). Pop on an empty stack is a no-op (the hint has nothing to say);
  // push at saturation overwrites the oldest entry.
  always_comb begin
    ras_tos_d   = ras_tos_q;
    ras_count_d = ras_count_q;
    ras_wr_en   = 1'b0;
    ras_wr_idx  = '0;
    if (bundle_fire && decoded_ras_pop && (ras_count_q != '0)) begin
      ras_tos_d   = ras_tos_q - 1'b1;
      ras_count_d = ras_count_q - 1'b1;
    end
    if (bundle_fire && decoded_ras_push) begin
      ras_tos_d  = ras_tos_d + 1'b1;
      ras_wr_en  = 1'b1;
      ras_wr_idx = ras_tos_d;
      if (ras_count_d != 4'(RAS_DEPTH)) begin
        ras_count_d = ras_count_d + 1'b1;
      end
    end
  end

  // Trap flush clears the stack; branch recovery restores checkpointed
  // pointers; otherwise dispatch updates the pointers. Flush and recovery
  // block dispatch, preventing competing updates. Contents reset for
  // simulation determinism; count prevents reads of unwritten slots.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ras_tos_q   <= '0;
      ras_count_q <= '0;
      for (ras_rst_i = 0; ras_rst_i < RAS_DEPTH; ras_rst_i++) begin
        ras_q[ras_rst_i] <= '0;
      end
    end else if (trap_q_valid) begin
      ras_tos_q   <= '0;
      ras_count_q <= '0;
    end else if (branch_recover_req) begin
      ras_tos_q   <= ras_ckpt_tos_q[recover_q_ckpt_id];
      ras_count_q <= ras_ckpt_count_q[recover_q_ckpt_id];
    end else begin
      ras_tos_q   <= ras_tos_d;
      ras_count_q <= ras_count_d;
      if (ras_wr_en) begin
        ras_q[ras_wr_idx] <= decoded_pc[0] + 32'd4;
      end
    end
  end

  // Per-checkpoint pointer snapshots: POST-dispatch values (see the
  // declaration note for why post, not pre). Written for EVERY checkpoint
  // rename allocates -- for a branch the pair is unchanged this cycle, for
  // a predicted return it carries the pop. Rows are read only while their
  // checkpoint is live, so they need no reset and go stale harmlessly on
  // release / trap reclaim.
  always_ff @(posedge clk) begin
    if (bundle_fire && rename_checkpoint_valid) begin
      ras_ckpt_tos_q[rename_checkpoint_id]   <= ras_tos_d;
      ras_ckpt_count_q[rename_checkpoint_id] <= ras_count_d;
    end
  end


  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      csr_inflight_q <= 1'b0;
    end else if (branch_recover_req || trap_q_valid) begin
      csr_inflight_q <= 1'b0;   // a taken trap flushes a younger in-flight CSR
    end else if (commit_fire[0] && rob_commit_is_csr[0]) begin
      csr_inflight_q <= 1'b0;
    end else if (csr_alloc_fire) begin
      csr_inflight_q <= 1'b1;
    end
  end


  logic [4:0] holder_valid;
  rob_idx_t [4:0] holder_age;
  logic [2:0] holder_rank [4:0];
  logic [4:0] cdb_lane_select [1:0];

  // One physical CDB client, two stable sources. Preserve the arbiter's age
  // discipline inside the merged client as well: fixed direct-AGU priority
  // could indefinitely hide an older deferred store behind a stream of
  // younger direct completions. Equal ring ages can occur after recovery-tail
  // reuse leaves a stale direct packet beside a fresh deferred packet; the
  // strict comparison gives the deferred source the deterministic tie-break,
  // while ROB sequence validation rejects the stale packet. Each source gets
  // its own acceptance pulse even though the outer arbiter sees one packet.
  assign agen_complete_select = agen_complete.valid &&
      (!sq_complete.valid ||
       ((agen_complete.rob_idx - rob_head_idx) <
        (sq_complete.rob_idx - rob_head_idx)));
  assign agen_cdb_complete = agen_complete_select
                           ? agen_complete : sq_complete;
  assign agen_complete_accept = cdb_grant_agen && agen_complete_select;
  assign sq_complete_accept = cdb_grant_agen && !agen_complete_select &&
                              sq_complete.valid;

  assign holder_valid = {
    lq_complete.valid,
    agen_cdb_complete.valid,
    muldiv_complete.valid,
    alu1_complete.valid,
    alu0_complete.valid
  };
  assign holder_age[0] = alu0_complete.rob_idx   - rob_head_idx;
  assign holder_age[1] = alu1_complete.rob_idx   - rob_head_idx;
  assign holder_age[2] = muldiv_complete.rob_idx - rob_head_idx;
  assign holder_age[3] = agen_cdb_complete.rob_idx - rob_head_idx;
  assign holder_age[4] = lq_complete.rob_idx     - rob_head_idx;
  // Select up to two CDB holders by ring age, then client position on an
  // age tie. Ties are reachable when a killed holder and a reused ROB slot
  // have the same index but different sequences. ROB acceptance validates
  // the sequence, so draining the stale packet cannot affect the live one.
  // Each lane captures its selected packet. A holder's grant is the OR of
  // its lane selections and denotes that holder's drain event.
  always_comb begin
    cdb_grant_alu0    = 1'b0;
    cdb_grant_alu1    = 1'b0;
    cdb_grant_muldiv = 1'b0;
    cdb_grant_agen   = 1'b0;
    cdb_grant_lq     = 1'b0;
    for (int i = 0; i < 5; i++) begin
      holder_rank[i] = 3'd0;

      for (int j = 0; j < 5; j++) begin
        if ((i != j) &&
            holder_valid[i] &&
            holder_valid[j] &&
            ((holder_age[j] < holder_age[i]) ||
             ((holder_age[j] == holder_age[i]) && (j < i)))) begin
          holder_rank[i] = holder_rank[i] + 3'd1;
        end
      end
    end
    for (int i = 0; i < 5; i++) begin
      cdb_lane_select[0][i] =
          holder_valid[i] && (holder_rank[i] == 3'd0);

      cdb_lane_select[1][i] =
          holder_valid[i] && (holder_rank[i] == 3'd1);
    end
    cdb_grant_alu0   = cdb_lane_select[0][0] || cdb_lane_select[1][0];
    cdb_grant_alu1   = cdb_lane_select[0][1] || cdb_lane_select[1][1];
    cdb_grant_muldiv = cdb_lane_select[0][2] || cdb_lane_select[1][2];
    cdb_grant_agen   = cdb_lane_select[0][3] || cdb_lane_select[1][3];
    cdb_grant_lq     = cdb_lane_select[0][4] || cdb_lane_select[1][4];
  end

  always_comb begin
    csr_src        = operand_a[0];
    csr_wdata_exec = csr_rdata;
    csr_we_exec    = 1'b0;

    unique case (alu0_exec_entry.csr_op)
      rv32i_pipeline_pkg::CSR_RWI, rv32i_pipeline_pkg::CSR_RSI, rv32i_pipeline_pkg::CSR_RCI: begin
        csr_src = {27'b0, alu0_exec_entry.csr_zimm};
      end

      default: begin
        csr_src = alu0_exec_a;
      end
    endcase

    unique case (alu0_exec_entry.csr_op)
      rv32i_pipeline_pkg::CSR_RW: begin
        csr_we_exec    = 1'b1;
        csr_wdata_exec = csr_src;
      end

      rv32i_pipeline_pkg::CSR_RS: begin
        csr_we_exec    = (alu0_exec_entry.csr_zimm != 5'd0);
        csr_wdata_exec = csr_rdata | csr_src;
      end

      rv32i_pipeline_pkg::CSR_RC: begin
        csr_we_exec    = (alu0_exec_entry.csr_zimm != 5'd0);
        csr_wdata_exec = csr_rdata & ~csr_src;
      end

      rv32i_pipeline_pkg::CSR_RWI: begin
        csr_we_exec    = 1'b1;
        csr_wdata_exec = csr_src;
      end

      rv32i_pipeline_pkg::CSR_RSI: begin
        csr_we_exec    = (alu0_exec_entry.csr_zimm != 5'd0);
        csr_wdata_exec = csr_rdata | csr_src;
      end

      rv32i_pipeline_pkg::CSR_RCI: begin
        csr_we_exec    = (alu0_exec_entry.csr_zimm != 5'd0);
        csr_wdata_exec = csr_rdata & ~csr_src;
      end

      default: begin
        csr_we_exec    = 1'b0;
        csr_wdata_exec = csr_rdata;
      end
    endcase
  end

  function automatic exec_input_slot_t make_in_slot(
      input iq_entry_t uop, input word_t opa, input word_t opb,
      input word_t st_data, input logic st_ready);
    exec_input_slot_t s;
    s = '0;
    s.valid = 1'b1;
    s.uop = uop;
    s.op_a = opa;
    s.op_b = opb;
    if (uop.is_store) begin
      if (st_ready || (uop.prs2 == '0)) begin
        s.store_data_valid = 1'b1;
        s.store_data = st_data;
      end else begin
        s.store_data_pending = 1'b1;
        s.store_prs2 = uop.prs2;
      end
    end
    make_in_slot = s;
  endfunction

  function automatic exec_input_slot_t next_in_slot(
      input exec_input_slot_t cur, input logic trap_v, input logic recover_v,
      input rob_idx_t head_idx, input rob_idx_t recover_idx,
      input logic [1:0] data_wb_fire, input phys_reg_t [1:0] data_wb_pdst,
      input word_t [1:0] data_wb_value);
    exec_input_slot_t s;
    s = cur;
    if (trap_v)
      s.valid = 1'b0;
    else if (recover_v && s.valid && slot_is_younger(s.uop.rob_idx, head_idx, recover_idx))
      s.valid = 1'b0;
    else if (s.valid && s.store_data_pending) begin
      if (data_wb_fire[0] && (data_wb_pdst[0] == s.store_prs2) &&
          (data_wb_pdst[0] != '0)) begin
        s.store_data_valid = 1'b1;
        s.store_data_pending = 1'b0;
        s.store_data = data_wb_value[0];
      end else if (data_wb_fire[1] && (data_wb_pdst[1] == s.store_prs2) &&
                   (data_wb_pdst[1] != '0)) begin
        s.store_data_valid = 1'b1;
        s.store_data_pending = 1'b0;
        s.store_data = data_wb_value[1];
      end
    end
    next_in_slot = s;
  endfunction

  function automatic exec_result_slot_t pack_alu_res(
      input iq_entry_t uop, input word_t result, input logic trap_v,
      input word_t trap_c, input word_t trap_t, input logic older,
      input logic csr_we, input word_t csr_wdata);
    exec_result_slot_t r;
    r = '0;
    r.valid = 1'b1;
    r.live = !trap_v;
    r.older = older;
    r.pkt.valid = 1'b1;
    r.pkt.rob_idx = uop.rob_idx;
    r.pkt.rob_seq = uop.rob_seq;
    r.pkt.pdst = uop.pdst;
    r.pkt.rd_wen = trap_v ? 1'b0 : uop.rd_wen;
    r.pkt.result = result;
    r.pkt.trap_valid = trap_v;
    r.pkt.trap_cause = trap_c;
    r.pkt.trap_tval = trap_t;
    r.pkt.csr_we = (uop.csr_op != rv32i_pipeline_pkg::CSR_NONE) && csr_we;
    r.pkt.csr_wdata = csr_wdata;
    pack_alu_res = r;
  endfunction

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      alu0_in_q <= '0;
      alu1_in_q <= '0;
      agen_in_q <= '0;
      md_in_q   <= '0;
      alu0_res_q <= '0;
      alu1_res_q <= '0;
    end else begin
      alu0_in_q[0] <= next_in_slot(alu0_in_q[0], trap_q_valid, branch_recover_req, rob_head_idx,
          recover_q_rob_idx, sq_data_wb_fire, sq_data_wb_pdst, sq_data_wb_value);
      alu0_in_q[1] <= next_in_slot(alu0_in_q[1], trap_q_valid, branch_recover_req, rob_head_idx,
          recover_q_rob_idx, sq_data_wb_fire, sq_data_wb_pdst, sq_data_wb_value);
      alu1_in_q[0] <= next_in_slot(alu1_in_q[0], trap_q_valid, branch_recover_req, rob_head_idx,
          recover_q_rob_idx, sq_data_wb_fire, sq_data_wb_pdst, sq_data_wb_value);
      alu1_in_q[1] <= next_in_slot(alu1_in_q[1], trap_q_valid, branch_recover_req, rob_head_idx,
          recover_q_rob_idx, sq_data_wb_fire, sq_data_wb_pdst, sq_data_wb_value);
      agen_in_q[0] <= next_in_slot(agen_in_q[0], trap_q_valid, branch_recover_req, rob_head_idx,
          recover_q_rob_idx, sq_data_wb_fire, sq_data_wb_pdst, sq_data_wb_value);
      agen_in_q[1] <= next_in_slot(agen_in_q[1], trap_q_valid, branch_recover_req, rob_head_idx,
          recover_q_rob_idx, sq_data_wb_fire, sq_data_wb_pdst, sq_data_wb_value);
      md_in_q      <= next_in_slot(md_in_q, trap_q_valid, branch_recover_req, rob_head_idx,
          recover_q_rob_idx, sq_data_wb_fire, sq_data_wb_pdst, sq_data_wb_value);

      if (alu0_exec_fire) begin
        if (alu0_exec_idx)
          alu0_in_q[1].valid <= 1'b0;
        else
          alu0_in_q[0].valid <= 1'b0;
      end
      if (alu1_exec_fire) begin
        if (alu1_exec_idx)
          alu1_in_q[1].valid <= 1'b0;
        else
          alu1_in_q[0].valid <= 1'b0;
      end
      if (agen_exec_fire) begin
        if (agen_exec_idx)
          agen_in_q[1].valid <= 1'b0;
        else
          agen_in_q[0].valid <= 1'b0;
      end
      if (md_exec_fire)
        md_in_q.valid <= 1'b0;

      if (alu0_issue_fire) begin
        if (alu0_in_push_idx)
          alu0_in_q[1] <= make_in_slot(alu0_issue_entry, alu0_operand_a, alu0_operand_b, '0, 1'b0);
        else
          alu0_in_q[0] <= make_in_slot(alu0_issue_entry, alu0_operand_a, alu0_operand_b, '0, 1'b0);
      end
      if (alu1_issue_fire) begin
        if (alu1_in_push_idx)
          alu1_in_q[1] <= make_in_slot(alu1_issue_entry, alu1_operand_a, alu1_operand_b, '0, 1'b0);
        else
          alu1_in_q[0] <= make_in_slot(alu1_issue_entry, alu1_operand_a, alu1_operand_b, '0, 1'b0);
      end
      if (muldiv_issue_fire)
        md_in_q <= make_in_slot(muldiv_issue_entry, muldiv_operand_a, muldiv_operand_b, '0, 1'b0);
      if (agen_issue_fire) begin
        if (agen_in_push_idx)
          agen_in_q[1] <= make_in_slot(agen_enq_entry, agen_enq_a, agen_enq_b,
              agen_enq_store_data, src_value_present(agen_enq_entry.prs2, ready_vec, rob_wb_accept, cdb_q, alu0_res_q, alu1_res_q));
        else
          agen_in_q[0] <= make_in_slot(agen_enq_entry, agen_enq_a, agen_enq_b,
              agen_enq_store_data, src_value_present(agen_enq_entry.prs2, ready_vec, rob_wb_accept, cdb_q, alu0_res_q, alu1_res_q));
      end

      if (cdb_grant_alu0) begin
        if (alu0_res_offer_idx) begin
          alu0_res_q[1].valid <= 1'b0;
          alu0_res_q[1].live  <= 1'b0;
          if (alu0_res_q[0].valid) alu0_res_q[0].older <= 1'b1;
        end else begin
          alu0_res_q[0].valid <= 1'b0;
          alu0_res_q[0].live  <= 1'b0;
          if (alu0_res_q[1].valid) alu0_res_q[1].older <= 1'b1;
        end
      end
      if (cdb_grant_alu1) begin
        if (alu1_res_offer_idx) begin
          alu1_res_q[1].valid <= 1'b0;
          alu1_res_q[1].live  <= 1'b0;
          if (alu1_res_q[0].valid) alu1_res_q[0].older <= 1'b1;
        end else begin
          alu1_res_q[0].valid <= 1'b0;
          alu1_res_q[0].live  <= 1'b0;
          if (alu1_res_q[1].valid) alu1_res_q[1].older <= 1'b1;
        end
      end
      if (trap_q_valid) begin
        alu0_res_q[0].live <= 1'b0;
        alu0_res_q[1].live <= 1'b0;
        alu1_res_q[0].live <= 1'b0;
        alu1_res_q[1].live <= 1'b0;
      end else if (branch_recover_req) begin
        if (alu0_res_q[0].valid && slot_is_younger(alu0_res_q[0].pkt.rob_idx, rob_head_idx, recover_q_rob_idx))
          alu0_res_q[0].live <= 1'b0;
        if (alu0_res_q[1].valid && slot_is_younger(alu0_res_q[1].pkt.rob_idx, rob_head_idx, recover_q_rob_idx))
          alu0_res_q[1].live <= 1'b0;
        if (alu1_res_q[0].valid && slot_is_younger(alu1_res_q[0].pkt.rob_idx, rob_head_idx, recover_q_rob_idx))
          alu1_res_q[0].live <= 1'b0;
        if (alu1_res_q[1].valid && slot_is_younger(alu1_res_q[1].pkt.rob_idx, rob_head_idx, recover_q_rob_idx))
          alu1_res_q[1].live <= 1'b0;
      end

      // Exec requires a pre-edge free result bank. If the other bank drains
      // simultaneously, this new result is the sole/oldest survivor. Using
      // only !other.valid leaves older=0 on a sole bank0 occupant; the next
      // ungranted fill of bank1 then overtakes its stable offer.
      // Grant affects registered FIFO order ONLY, never FU admission.
      if (alu0_exec_fire) begin
        if (alu0_res_push_idx)
          alu0_res_q[1] <= pack_alu_res(alu0_exec_entry, exec_result[0],
              exec_trap_valid[0], exec_trap_cause[0], exec_trap_tval[0],
              !alu0_res_q[0].valid || cdb_grant_alu0, csr_we_exec, csr_wdata_exec);
        else
          alu0_res_q[0] <= pack_alu_res(alu0_exec_entry, exec_result[0],
              exec_trap_valid[0], exec_trap_cause[0], exec_trap_tval[0],
              !alu0_res_q[1].valid || cdb_grant_alu0, csr_we_exec, csr_wdata_exec);
      end
      if (alu1_exec_fire) begin
        if (alu1_res_push_idx)
          alu1_res_q[1] <= pack_alu_res(alu1_exec_entry, exec_result[1],
              exec_trap_valid[1], exec_trap_cause[1], exec_trap_tval[1],
              !alu1_res_q[0].valid || cdb_grant_alu1, csr_we_exec, csr_wdata_exec);
        else
          alu1_res_q[0] <= pack_alu_res(alu1_exec_entry, exec_result[1],
              exec_trap_valid[1], exec_trap_cause[1], exec_trap_tval[1],
              !alu1_res_q[1].valid || cdb_grant_alu1, csr_we_exec, csr_wdata_exec);
      end
    end
  end

  // Hold direct store completions and execute-detected memory traps until the
  // CDB grants the AGU client. A data-late aligned store completes through the
  // SQ's stable deferred packet instead; aligned loads use lq_complete.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      agen_complete <= '0;
    end else begin
      if (agen_complete_accept) begin
        agen_complete.valid <= 1'b0;
      end

      if (agen_direct_complete_fire) begin
        agen_complete.valid       <= 1'b1;
        agen_complete.rob_idx     <= agen_exec_entry.rob_idx;
        agen_complete.rob_seq     <= agen_exec_entry.rob_seq;
        agen_complete.pdst        <= agen_exec_entry.pdst;
        agen_complete.rd_wen      <= agen_trap_valid ? 1'b0 : agen_exec_entry.rd_wen;
        agen_complete.result      <= agen_addr;
        agen_complete.trap_valid  <= agen_trap_valid;
        agen_complete.trap_cause  <= agen_trap_cause;
        agen_complete.trap_tval   <= agen_trap_tval;
        agen_complete.csr_we      <= 1'b0;
        agen_complete.csr_wdata   <= '0;
      end
    end
  end

  // Registered CDB beat consumed by ROB/PRF on the following cycle.
  // Selection drives only the registered transport, not PRF readiness.
  // Lane 0 keeps its stale payload with valid/csr_we cleared; lane 1 zeroes.
  always_comb begin
    cdb_next[0]           = cdb_q[0];
    cdb_next[0].valid     = 1'b0;
    cdb_next[0].csr_we    = 1'b0;
    cdb_next[0].csr_wdata = '0;
    cdb_next[1]           = '0;

    if (cdb_lane_select[0][0]) begin
      cdb_next[0] = alu0_complete;
    end else if (cdb_lane_select[0][1]) begin
      cdb_next[0] = alu1_complete;
    end else if (cdb_lane_select[0][2]) begin
      cdb_next[0] = muldiv_complete;
    end else if (cdb_lane_select[0][3]) begin
      cdb_next[0] = agen_cdb_complete;
    end else if (cdb_lane_select[0][4]) begin
      cdb_next[0] = lq_complete;
    end

    if (cdb_lane_select[1][0]) begin
      cdb_next[1] = alu0_complete;
    end else if (cdb_lane_select[1][1]) begin
      cdb_next[1] = alu1_complete;
    end else if (cdb_lane_select[1][2]) begin
      cdb_next[1] = muldiv_complete;
    end else if (cdb_lane_select[1][3]) begin
      cdb_next[1] = agen_cdb_complete;
    end else if (cdb_lane_select[1][4]) begin
      cdb_next[1] = lq_complete;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cdb_q <= '0;
    end else begin
      cdb_q <= cdb_next;
    end
  end



  // Control target and misalign
  always_comb begin
    control_target = '0;
    control_target_misalign = '0;
    // Predicted and serialized returns compute the same actual target.
    // Prediction does not bypass execute-side misalignment detection.
    if (jump_resolve_fire || jump_pred_resolve_fire) begin
      control_target[0] = (alu0_exec_a + alu0_exec_entry.imm) & 32'hffff_fffe;
      control_target_misalign[0] = (control_target[0][1:0] != 2'b00);
    end else if (alu0_exec_fire && (alu0_exec_entry.op_class == OOO_OP_BRANCH)
                && branch_taken[0]) begin
      control_target[0] = alu0_exec_entry.pc + alu0_exec_entry.imm;
      control_target_misalign[0] = (control_target[0][1:0] != 2'b00);
    end
    // ALU1 handles branch comparisons but never jump/CSR solo operations.
    if (alu1_exec_fire && (alu1_exec_entry.op_class == OOO_OP_BRANCH)
               && branch_taken[1]) begin
      control_target[1] = alu1_exec_entry.pc + alu1_exec_entry.imm;
      control_target_misalign[1] = (control_target[1][1:0] != 2'b00);
    end

    load_misalign = load_agen_fire &&
                    (((agen_exec_entry.mem_size == fyp_cpu_pkg::MEM_W)
                     && (agen_addr[1:0] != 2'b00)) ||
                    ((agen_exec_entry.mem_size == fyp_cpu_pkg::MEM_H)
                    && (agen_addr[0]   != 1'b0)));
    store_misalign = store_agen_fire &&
                    (((agen_exec_entry.mem_size == fyp_cpu_pkg::MEM_W)
                    && (agen_addr[1:0] != 2'b00)) ||
                    ((agen_exec_entry.mem_size == fyp_cpu_pkg::MEM_H)
                    && (agen_addr[0]   != 1'b0)));

    lsu_misalign = load_misalign || store_misalign;
    exec_trap_cause = '0;
    exec_trap_valid = '0;
    exec_trap_tval  = '0;
    exec_trap_valid = control_target_misalign;

    if (control_target_misalign[0]) begin
      exec_trap_cause[0] = OOO_CAUSE_IADDR_MISALIGN;
      exec_trap_tval[0]  = control_target[0];
    end
    if (control_target_misalign[1]) begin
      exec_trap_cause[1] = OOO_CAUSE_IADDR_MISALIGN;
      exec_trap_tval[1]  = control_target[1];
    end
    agen_trap_valid = 1'b0;
    agen_trap_cause = '0;
    agen_trap_tval  = '0;

    agen_trap_valid = lsu_misalign;
    if (load_misalign) begin
      agen_trap_cause = OOO_CAUSE_LOAD_MISALIGN;
      agen_trap_tval  = agen_addr;
    end else if (store_misalign) begin
      agen_trap_cause = OOO_CAUSE_STORE_MISALIGN;
      agen_trap_tval  = agen_addr;
    end
  end


  // Store-byte placement is computed at LSU execute. The SQ stores these
  // already-final bus values and later drains them to memory at commit.
  always_comb begin
    store_wdata_next = '0;
    store_be_next    = 4'b0000;

    unique case (agen_exec_entry.mem_size)
      fyp_cpu_pkg::MEM_W: begin
        store_wdata_next = agen_exec_store_data;
        store_be_next    = 4'b1111;
      end

      fyp_cpu_pkg::MEM_H: begin
        if (agen_addr[1]) begin
          store_wdata_next = {agen_exec_store_data[15:0], 16'h0000};
          store_be_next    = 4'b1100;
        end else begin
          store_wdata_next = {16'h0000, agen_exec_store_data[15:0]};
          store_be_next    = 4'b0011;
        end
      end

      fyp_cpu_pkg::MEM_B: begin
        unique case (agen_addr[1:0])
          2'b00: begin
            store_wdata_next = {24'h000000, agen_exec_store_data[7:0]};
            store_be_next    = 4'b0001;
          end
          2'b01: begin
            store_wdata_next = {16'h0000, agen_exec_store_data[7:0], 8'h00};
            store_be_next    = 4'b0010;
          end
          2'b10: begin
            store_wdata_next = {8'h00, agen_exec_store_data[7:0], 16'h0000};
            store_be_next    = 4'b0100;
          end
          default: begin
            store_wdata_next = {agen_exec_store_data[7:0], 24'h000000};
            store_be_next    = 4'b1000;
          end
        endcase
      end

      default: begin
        store_wdata_next = agen_exec_store_data;
        store_be_next    = 4'b1111;
      end
    endcase
  end

  // Bypass operands from accepted CDB transit and live ALU result holders.
  // The ALU execute edge sets early readiness and registers the final value;
  // the holder retains it while CDB arbitration delays transport. p0 never
  // bypasses. Live-holder qualification prevents killed packets from supplying
  // values after ROB or physical-register reuse.
  always_comb begin
    byp_rdata1 = prf_rdata1;
    byp_rdata2 = prf_rdata2;
    byp_rdata3 = prf_rdata3;
    byp_rdata4 = prf_rdata4;
    for (int l = 0; l < 2; l++) begin
      // Accept-gated: a stale beat sharing a live beat's pdst (killed op,
      // pdst re-allocated, both granted together) fails the seq check and
      // must not serve operands -- acceptance names the one true value.
      if (rob_wb_accept[l] && cdb_q[l].rd_wen && (cdb_q[l].pdst != '0)) begin
        if (cdb_q[l].pdst == issue_prs1[0]) byp_rdata1 = cdb_q[l].result;
        if (cdb_q[l].pdst == issue_prs2[0]) byp_rdata2 = cdb_q[l].result;
        if (cdb_q[l].pdst == issue_prs1[1]) byp_rdata3 = cdb_q[l].result;
        if (cdb_q[l].pdst == issue_prs2[1]) byp_rdata4 = cdb_q[l].result;
      end
    end
    for (int rb = 0; rb < 2; rb++) begin
      if (alu0_res_q[rb].live && alu0_res_q[rb].valid &&
          alu0_res_q[rb].pkt.rd_wen && (alu0_res_q[rb].pkt.pdst != '0)) begin
        if (alu0_res_q[rb].pkt.pdst == issue_prs1[0]) byp_rdata1 = alu0_res_q[rb].pkt.result;
        if (alu0_res_q[rb].pkt.pdst == issue_prs2[0]) byp_rdata2 = alu0_res_q[rb].pkt.result;
        if (alu0_res_q[rb].pkt.pdst == issue_prs1[1]) byp_rdata3 = alu0_res_q[rb].pkt.result;
        if (alu0_res_q[rb].pkt.pdst == issue_prs2[1]) byp_rdata4 = alu0_res_q[rb].pkt.result;
      end
      if (alu1_res_q[rb].live && alu1_res_q[rb].valid &&
          alu1_res_q[rb].pkt.rd_wen && (alu1_res_q[rb].pkt.pdst != '0)) begin
        if (alu1_res_q[rb].pkt.pdst == issue_prs1[0]) byp_rdata1 = alu1_res_q[rb].pkt.result;
        if (alu1_res_q[rb].pkt.pdst == issue_prs2[0]) byp_rdata2 = alu1_res_q[rb].pkt.result;
        if (alu1_res_q[rb].pkt.pdst == issue_prs1[1]) byp_rdata3 = alu1_res_q[rb].pkt.result;
        if (alu1_res_q[rb].pkt.pdst == issue_prs2[1]) byp_rdata4 = alu1_res_q[rb].pkt.result;
      end
    end
  end

  rv32i_ss_prf u_prf (
    .clk      (clk),
    .rst_n    (rst_n),
    .raddr1   (issue_prs1[0]),
    .rdata1   (prf_rdata1),
    .raddr2   (issue_prs2[0]),
    .rdata2   (prf_rdata2),
    .raddr3   (issue_prs1[1]),
    .rdata3   (prf_rdata3),
    .raddr4   (issue_prs2[1]),
    .rdata4   (prf_rdata4),
    .write_en ({
      rob_wb_accept[1] & cdb_q[1].rd_wen,
      rob_wb_accept[0] & cdb_q[0].rd_wen
    }),
    .waddr    ({cdb_q[1].pdst, cdb_q[0].pdst}),
    .wdata    ({cdb_q[1].result, cdb_q[0].result}),

    // ---- ready/busy table ----
    // A real-dest uop allocates its pdst at dispatch -> mark it busy.
    .alloc_fire ({
      bundle_fire && rename_rd_we[1],
      bundle_fire && rename_rd_we[0]
    }),
    .alloc_phys (rename_pdst),

    // Grant-time ready ports are inactive. Non-ALU readiness is set by the
    // registered writeback's identity-qualified acceptance, alongside its value.
    // Final non-trapping ALU execution supplies the early readiness event.
    .early_set  ({
      alu1_exec_fire & alu1_exec_entry.rd_wen & !exec_trap_valid[1],
      alu0_exec_fire & alu0_exec_entry.rd_wen & !exec_trap_valid[0],
      2'b00
    }),
    .early_pdst ({alu1_exec_entry.pdst, alu0_exec_entry.pdst,
                  phys_reg_t'('0), phys_reg_t'('0)}),

    .ready_vec  (ready_vec)                       // IQ consumes this
  );

  rv32i_alu u_alu0 (
    .operand_a (alu0_exec_a),
    .operand_b (alu0_exec_b),
    .alu_op    (alu0_exec_entry.alu_op),
    .result    (alu0_result),
    .zero      ()
  );

  // ALU1: the second physical ALU.
  rv32i_alu u_alu1 (
    .operand_a (alu1_exec_a),
    .operand_b (alu1_exec_b),
    .alu_op    (alu1_exec_entry.alu_op),
    .result    (alu1_result),
    .zero      ()
  );

  rv32i_ss_muldiv u_muldiv (
    .clk            (clk),
    .rst_n          (rst_n),
    .start          (muldiv_start),
    .op          (md_exec_entry.muldiv_op),
    .a           (md_exec_a),
    .b           (md_exec_b),
    .rob_idx     (md_exec_entry.rob_idx),
    .rob_seq     (md_exec_entry.rob_seq),
    .branch_mask (md_exec_entry.branch_mask),
    .pdst        (md_exec_entry.pdst),
    .rd_wen      (md_exec_entry.rd_wen),
    .complete_ready (cdb_grant_muldiv),
    .kill           (muldiv_kill),
    .busy           (muldiv_busy),
    .complete       (muldiv_complete)
  );

  // Machine-mode CSR file. Execute reads the previous CSR value for rd;
  // architectural writes occur at the committing ROB head. Trap and mret
  // use the registered precise-event path.
  rv32i_ss_csr_file u_csr_file (
    .clk        (clk),
    .rst_n      (rst_n),
    .we         (commit_fire[0] && rob_commit_is_csr[0] && rob_commit_csr_we[0]),
    .waddr      (rob_commit_csr_addr[0]),
    .wdata      (rob_commit_csr_wdata[0]),
    .rdata      (csr_rdata),
    .raddr      (alu0_exec_entry.csr_addr),
    .trap_we    (trap_q_valid && !trap_q_is_mret),
    .trap_pc    (trap_q_pc),
    .trap_cause (trap_q_cause),
    .trap_tval  (trap_q_tval),
    .mret_we    (trap_q_valid && trap_q_is_mret),
    .mtvec      (csr_mtvec),
    .mepc       (csr_mepc)
  );

  rv32i_branch_cmp u_branch_cmp0 (
    .branch_op (alu0_exec_entry.branch_op),
    .a         (alu0_exec_a),
    .b         (alu0_exec_b),
    .taken     (branch_taken[0])
  );

  rv32i_branch_cmp u_branch_cmp1 (
    .branch_op (alu1_exec_entry.branch_op),
    .a         (alu1_exec_a),
    .b         (alu1_exec_b),
    .taken     (branch_taken[1])
  );
  // Frontend redirect priority is trap, registered branch recovery,
  // serialized-jump resolution, aligned JAL dispatch, then return fallback
  // at dispatch. Correct predictions require no repair redirect. Dispatch
  // gating excludes its redirects during flush, recovery and serialized-jump
  // execution; the explicit priority also defines simultaneous-input behavior.
  always_comb begin
    exec_result     = '0;
    exec_result[0]     = alu0_result;
    exec_result[1]     = alu1_result;
    redirect_valid  = 1'b0;
    redirect_target = fallthrough_pc;
    if (alu0_exec_fire &&
        (alu0_exec_entry.csr_op != rv32i_pipeline_pkg::CSR_NONE)) begin
      exec_result[0] = csr_rdata;
    end
    // Link value for ANY executing jump (serialized or de-serialized): rd gets
    // pc+4. Dispatch-time JAL/RAS redirects do not repeat here.
    if (alu0_exec_fire && (alu0_exec_entry.op_class == OOO_OP_JUMP) &&
        !control_target_misalign[0]) begin
      exec_result[0] = fallthrough_pc;
    end
    if (trap_q_valid) begin
      redirect_valid = 1'b1;
      redirect_target = trap_q_target;
    end else if (branch_recover_req) begin
      redirect_valid = 1'b1;
      redirect_target = recover_q_target;
    end else if (jump_resolve_fire && !control_target_misalign[0]) begin
      redirect_valid = 1'b1;
      redirect_target = (alu0_exec_a + alu0_exec_entry.imm) & 32'hffff_fffe;
    end else if (jal_deser_fire) begin
      redirect_valid  = 1'b1;
      redirect_target = jal_deser_target;
    end else if (ras_decode_redirect_fire) begin
      // predicted-return dispatch steers fetch to the RAS target --
      // a dispatch-cycle redirect like jal_deser (the two are mutually
      // exclusive: one slot-0 instruction is a JAL or a JALR, never both).
      redirect_valid  = 1'b1;
      redirect_target = ras_pred_target;
    end
  end

`ifndef SYNTHESIS
  completion_packet_t tw_alu0_prev_q;
  logic               tw_alu0_held_prev_q = 1'b0;
  completion_packet_t tw_alu1_prev_q;
  logic               tw_alu1_held_prev_q = 1'b0;
  completion_packet_t tw_agen_prev_q;
  logic               tw_agen_held_prev_q = 1'b0;
  logic               tw_trap_latch_prev_q;

  // coherence-recompute helpers: candidate 0's class, verdict, kill and
  // target re-derived from ALU0's OWN facts, independently of the candidate
  // block's wiring.
  logic  tw_c0_is_pjump;
  word_t tw_c0_jump_actual;
  logic  tw_c0_mispredict;
  logic  tw_c0_misalign_kill;
  word_t tw_c0_target;
  assign tw_c0_is_pjump    = alu0_exec_fire && alu0_exec_is_jump_pred;
  assign tw_c0_jump_actual = (alu0_exec_a + alu0_exec_entry.imm)
                             & 32'hffff_fffe;
  assign tw_c0_mispredict  = tw_c0_is_pjump
                             ? (alu0_exec_entry.pred_target != tw_c0_jump_actual)
                             : ((branch_taken[0] != alu0_exec_entry.pred_taken) ||
                                (branch_taken[0] &&
                                 (alu0_exec_entry.pred_target !=
                                  (alu0_exec_entry.pc + alu0_exec_entry.imm))));
  assign tw_c0_misalign_kill = tw_c0_is_pjump
                               ? control_target_misalign[0]
                               : (branch_taken[0] && control_target_misalign[0]);
  assign tw_c0_target      = tw_c0_is_pjump
                             ? tw_c0_jump_actual
                             : (branch_taken[0]
                                ? (alu0_exec_entry.pc + alu0_exec_entry.imm)
                                : (alu0_exec_entry.pc + 32'd4));

  always @(posedge clk) begin
    if (rst_n === 1'b1) begin
      if (issue_fire[0] && !issue_valid[0]) begin
        $fatal(1, "rv32i_ss_core: issue_fire[0] without IQ issue_valid[0]");
      end

      // Flow-and-reject: ungranted held completions are never
      // scrubbed or mutated; they leave only through a CDB grant.
      if (tw_alu0_held_prev_q && (alu0_complete !== tw_alu0_prev_q)) begin
        $fatal(1, "rv32i_ss_core: held ALU0 completion scrubbed or mutated");
      end
      if (tw_alu1_held_prev_q && (alu1_complete !== tw_alu1_prev_q)) begin
        $fatal(1, "rv32i_ss_core: held ALU1 completion scrubbed or mutated");
      end
      if (tw_agen_held_prev_q && (agen_complete !== tw_agen_prev_q)) begin
        $fatal(1, "rv32i_ss_core: held AGU completion scrubbed or mutated");
      end

      // Execution-driven ALU wakeup couples a registered ready bit to a live
      // registered value holder. A live holder without valid/value state, or
      // two live value locations naming one preg, would let the tag wake a
      // consumer while bypass priority silently chooses the wrong value.
      if (alu0_holder_live_q && !alu0_complete.valid) begin
        $fatal(1, "rv32i_ss_core: live ALU0 holder has no valid packet");
      end
      if (alu1_holder_live_q && !alu1_complete.valid) begin
        $fatal(1, "rv32i_ss_core: live ALU1 holder has no valid packet");
      end
      if (alu0_holder_live_q && alu0_complete.valid && alu0_complete.rd_wen &&
          (alu0_complete.pdst != '0) && !ready_vec[alu0_complete.pdst]) begin
        $fatal(1, "rv32i_ss_core: live ALU0 holder destination is not ready");
      end
      if (alu1_holder_live_q && alu1_complete.valid && alu1_complete.rd_wen &&
          (alu1_complete.pdst != '0) && !ready_vec[alu1_complete.pdst]) begin
        $fatal(1, "rv32i_ss_core: live ALU1 holder destination is not ready");
      end
      if (alu0_holder_live_q && alu0_complete.valid && alu0_complete.rd_wen &&
          alu1_holder_live_q && alu1_complete.valid && alu1_complete.rd_wen &&
          (alu0_complete.pdst == alu1_complete.pdst) &&
          (alu0_complete.pdst != '0)) begin
        $fatal(1, "rv32i_ss_core: both live ALU holders name one pdst");
      end
      for (int pin_l = 0; pin_l < 2; pin_l++) begin
        if (rob_wb_accept[pin_l] && cdb_q[pin_l].rd_wen &&
            alu0_holder_live_q && alu0_complete.valid &&
            alu0_complete.rd_wen &&
            (cdb_q[pin_l].pdst == alu0_complete.pdst) &&
            (cdb_q[pin_l].pdst != '0)) begin
          $fatal(1, "rv32i_ss_core: CDB transit and live ALU0 holder name one pdst");
        end
        if (rob_wb_accept[pin_l] && cdb_q[pin_l].rd_wen &&
            alu1_holder_live_q && alu1_complete.valid &&
            alu1_complete.rd_wen &&
            (cdb_q[pin_l].pdst == alu1_complete.pdst) &&
            (cdb_q[pin_l].pdst != '0)) begin
          $fatal(1, "rv32i_ss_core: CDB transit and live ALU1 holder name one pdst");
        end
      end
      if (alu0_issue_fire && alu0_issue_entry.rd_wen &&
          (alu0_issue_entry.pdst != '0) &&
          ready_vec[alu0_issue_entry.pdst]) begin
        $fatal(1, "rv32i_ss_core: ALU0 issue names an already-ready destination");
      end
      if (alu1_issue_fire && alu1_issue_entry.rd_wen &&
          (alu1_issue_entry.pdst != '0) &&
          ready_vec[alu1_issue_entry.pdst]) begin
        $fatal(1, "rv32i_ss_core: ALU1 issue names an already-ready destination");
      end

      if (issue_fire[0] && !rob_head_valid) begin
        $fatal(1, "rv32i_ss_core: issue_fire[0] while ROB is empty");
      end

      // Combiner input contract at the machine seam: one ROB entry can never
      // resolve on two ALU units in one cycle — the equality the combiner's
      // strict age compares treat as unreachable.
      if (branch_candidate[0].resolve_valid && branch_candidate[1].resolve_valid &&
          (branch_candidate[0].rob_idx == branch_candidate[1].rob_idx)) begin
        $fatal(1, "rv32i_ss_core: two branch resolutions share one ROB entry");
      end

      // Dual-candidate integrity pins.
      if (branch_candidate[0].resolve_valid && branch_candidate[1].resolve_valid &&
          (branch_candidate[0].checkpoint_id == branch_candidate[1].checkpoint_id)) begin
        $fatal(1, "rv32i_ss_core: two branch resolutions share one checkpoint id");
      end
      if ((branch_candidate[0].correct_valid || branch_candidate[0].recover_valid)
          && !branch_candidate[0].resolve_valid) begin
        $fatal(1, "rv32i_ss_core: candidate 0 child valid without resolve");
      end
      if ((branch_candidate[1].correct_valid || branch_candidate[1].recover_valid)
          && !branch_candidate[1].resolve_valid) begin
        $fatal(1, "rv32i_ss_core: candidate 1 child valid without resolve");
      end

      // Independently rebuild each physical ALU's completion metadata and
      // control-flow outcome from that unit's execution facts. Branch recovery
      // uses actual pc+imm or pc+4 and compares direction and exact target.
      // ALU0 predicted returns compare (rs1+imm)&~1 against their prediction;
      // misaligned actual targets take the trap path. This catches cross-unit
      // wiring that could misclassify a trap as ordinary branch recovery.
      if (branch_candidate[0].resolve_valid &&
          ((branch_candidate[0].rob_idx != alu0_exec_entry.rob_idx) ||
           (branch_candidate[0].checkpoint_id != alu0_exec_entry.checkpoint_id) ||
           (branch_candidate[0].target != tw_c0_target))) begin
        $fatal(1, "rv32i_ss_core: candidate 0 incoherent with ALU0's entry");
      end
      if (branch_candidate[1].resolve_valid &&
          ((branch_candidate[1].rob_idx != alu1_exec_entry.rob_idx) ||
           (branch_candidate[1].checkpoint_id != alu1_exec_entry.checkpoint_id) ||
           (branch_candidate[1].target !=
            (branch_taken[1] ? (alu1_exec_entry.pc + alu1_exec_entry.imm)
                             : (alu1_exec_entry.pc + 32'd4))))) begin
        $fatal(1, "rv32i_ss_core: candidate 1 incoherent with ALU1's entry");
      end
      if (branch_candidate[0].resolve_valid !==
          ((alu0_exec_fire && (alu0_exec_entry.op_class == OOO_OP_BRANCH))
           || tw_c0_is_pjump)) begin
        $fatal(1, "rv32i_ss_core: candidate 0 resolve incoherent with ALU0's fire");
      end
      if (branch_candidate[0].recover_valid !==
          (branch_candidate[0].resolve_valid &&
           tw_c0_mispredict && !tw_c0_misalign_kill)) begin
        $fatal(1, "rv32i_ss_core: candidate 0 recover incoherent with ALU0's outcome");
      end
      if (branch_candidate[0].correct_valid !==
          (branch_candidate[0].resolve_valid &&
           !tw_c0_mispredict && !tw_c0_misalign_kill)) begin
        $fatal(1, "rv32i_ss_core: candidate 0 correct incoherent with ALU0's outcome");
      end

      // Return-resolution invariants.
      // Candidate 1 must never carry a jump: jumps are issue-solo and
      // ALU0-only (IQ pins), so a jump on ALU1 is a routing break.
      if (branch_candidate[1].resolve_valid &&
          (alu1_exec_entry.op_class == OOO_OP_JUMP)) begin
        $fatal(1, "rv32i_ss_core: jump resolution on the ALU1 candidate seam");
      end
      // A predicted jump must be a JALR (the RAS stamps only pop-JALRs;
      // a pred_taken JAL means the stamp leaked across classes).
      if (tw_c0_is_pjump && (alu0_exec_entry.src1_sel != OOO_SRC_REG)) begin
        $fatal(1, "rv32i_ss_core: predicted jump is not a JALR");
      end
      // premise at the execute end: a predicted return never
      // allocates a destination (ret-form narrowing).
      if (tw_c0_is_pjump && alu0_exec_entry.rd_wen) begin
        $fatal(1, "rv32i_ss_core: predicted return allocates a destination");
      end
      // The BTB is branch-only: a training beat must never carry the
      // resolving jump's pc (a same-cycle ALU1 branch legitimately trains,
      // and its pc can never equal the jump's -- one static instruction
      // has one class).
      if (bp_update_valid && tw_c0_is_pjump &&
          (bp_update_pc == alu0_exec_entry.pc)) begin
        $fatal(1, "rv32i_ss_core: BTB trained on a predicted return's pc");
      end
      if (branch_candidate[1].resolve_valid !==
          (alu1_exec_fire && (alu1_exec_entry.op_class == OOO_OP_BRANCH))) begin
        $fatal(1, "rv32i_ss_core: candidate 1 resolve incoherent with ALU1's fire");
      end
      if (branch_candidate[1].recover_valid !==
          (branch_candidate[1].resolve_valid &&
           ((branch_taken[1] != alu1_exec_entry.pred_taken) ||
            (branch_taken[1] && (alu1_exec_entry.pred_target !=
                                 (alu1_exec_entry.pc + alu1_exec_entry.imm)))) &&
           !(branch_taken[1] && control_target_misalign[1]))) begin
        $fatal(1, "rv32i_ss_core: candidate 1 recover incoherent with ALU1's outcome");
      end
      if (branch_candidate[1].correct_valid !==
          (branch_candidate[1].resolve_valid &&
           !((branch_taken[1] != alu1_exec_entry.pred_taken) ||
             (branch_taken[1] && (alu1_exec_entry.pred_target !=
                                  (alu1_exec_entry.pc + alu1_exec_entry.imm)))) &&
           !(branch_taken[1] && control_target_misalign[1]))) begin
        $fatal(1, "rv32i_ss_core: candidate 1 correct incoherent with ALU1's outcome");
      end

      // Two-grant routing integrity and zero payload for inactive grants.
      if (issue_fire[0] && issue_fire[1] &&
          (issue_unit[0] == issue_unit[1])) begin
        $fatal(1, "rv32i_ss_core: both grant positions bound one FU unit");
      end
      if ((|issue_fire) && (branch_recover_req || trap_q_valid)) begin
        $fatal(1, "rv32i_ss_core: issue fired on a broadcast cycle");
      end

      if (rob_wb_accept[0] && !cdb_q[0].valid) begin
        $fatal(1, "rv32i_ss_core: ROB accepted lane-0 writeback without CDB fire");
      end
      if (rob_wb_accept[1] && !cdb_q[1].valid) begin
        $fatal(1, "rv32i_ss_core: ROB accepted lane-1 writeback without CDB fire");
      end

      // CDB select tripwires: each transport lane is independently
      // one-hot, and no completion holder may drain through both lanes.
      if ((cdb_lane_select[0] & (cdb_lane_select[0] - 5'd1)) != 5'd0) begin
        $fatal(1, "rv32i_ss_core: CDB lane-0 select is not one-hot");
      end
      if ((cdb_lane_select[1] & (cdb_lane_select[1] - 5'd1)) != 5'd0) begin
        $fatal(1, "rv32i_ss_core: CDB lane-1 select is not one-hot");
      end
      if (|(cdb_lane_select[0] & cdb_lane_select[1])) begin
        $fatal(1, "rv32i_ss_core: CDB lanes selected the same client");
      end
      // Two accepted transit beats cannot carry the same real destination.
      // Raw valid beats can collide when a killed operation's pdst is reused;
      // the acceptance check identifies which generation may supply operands.
      if (rob_wb_accept[0] && cdb_q[0].rd_wen &&
          rob_wb_accept[1] && cdb_q[1].rd_wen &&
          (cdb_q[0].pdst == cdb_q[1].pdst) && (cdb_q[0].pdst != '0)) begin
        $fatal(1, "rv32i_ss_core: both CDB transit lanes carry one pdst");
      end
      if ((|cdb_lane_select[1]) && !(|cdb_lane_select[0])) begin
        $fatal(1, "rv32i_ss_core: CDB lane 1 selected without lane 0");
      end
      if (|(cdb_lane_select[0] & ~holder_valid) ||
          |(cdb_lane_select[1] & ~holder_valid)) begin
        $fatal(1, "rv32i_ss_core: CDB lane selected an invalid holder");
      end

      if (cdb_grant_alu0 && !alu0_complete.valid) begin
        $fatal(1, "rv32i_ss_core: CDB granted invalid ALU0 completion");
      end

      if (cdb_grant_alu1 && !alu1_complete.valid) begin
        $fatal(1, "rv32i_ss_core: CDB granted invalid ALU1 completion");
      end

      if (cdb_grant_muldiv && !muldiv_complete.valid) begin
        $fatal(1, "rv32i_ss_core: CDB granted invalid muldiv completion");
      end

      if (cdb_grant_agen && !agen_cdb_complete.valid) begin
        $fatal(1, "rv32i_ss_core: CDB granted invalid AGU completion");
      end

      if (cdb_grant_lq && !lq_complete.valid) begin
        $fatal(1, "rv32i_ss_core: CDB granted invalid LQ completion");
      end

      // Occupancy-only: a full input is not IQ capacity, including an exec
      // pop this cycle. CDB grant is not admission.
      if (alu0_issue_fire && alu0_in_q[0].valid && alu0_in_q[1].valid) begin
        $fatal(1, "rv32i_ss_core: ALU0 issue into a full input");
      end
      if (alu1_issue_fire && alu1_in_q[0].valid && alu1_in_q[1].valid) begin
        $fatal(1, "rv32i_ss_core: ALU1 issue into a full input");
      end
      if (agen_issue_fire && agen_in_q[0].valid && agen_in_q[1].valid) begin
        $fatal(1, "rv32i_ss_core: AGEN issue into a full input");
      end
      if (muldiv_issue_fire && md_in_q.valid) begin
        $fatal(1, "rv32i_ss_core: muldiv issue into a full input");
      end
      if ((alu0_in_q[0].valid && alu0_in_q[1].valid) && alu0_fu_ready) begin
        $fatal(1, "rv32i_ss_core: ALU0 fu_ready while both input slots occupied");
      end
      if (alu0_exec_fire && alu0_res_q[0].valid && alu0_res_q[1].valid) begin
        $fatal(1, "rv32i_ss_core: ALU0 exec without a free result slot");
      end
      if (alu1_exec_fire && alu1_res_q[0].valid && alu1_res_q[1].valid) begin
        $fatal(1, "rv32i_ss_core: ALU1 exec without a free result slot");
      end
      if (agen_exec_fire && agen_complete.valid) begin
        $fatal(1, "rv32i_ss_core: AGEN exec with a live direct holder");
      end
      if ((branch_recover_req || trap_q_valid) &&
          (alu0_exec_fire || alu1_exec_fire || agen_exec_fire || md_exec_fire)) begin
        $fatal(1, "rv32i_ss_core: exec_fire during recovery/flush broadcast");
      end

      if (agen_complete.valid && sq_complete.valid &&
          (agen_complete.rob_idx == sq_complete.rob_idx) &&
          (agen_complete.rob_seq == sq_complete.rob_seq)) begin
        $fatal(1, "rv32i_ss_core: one ROB identity has direct and deferred AGU completions");
      end

      // Store retirement requires a write accepted now or a recorded acceptance
      // from an earlier recovery edge.
      if (store_commit_fire && !(sq_mem_accept || sq_accept_deferred)) begin
        $fatal(1, "rv32i_ss_core: store retired without an accepted write");
      end

      // the write strobe may assert through a stall window before the
      // commit fires, but never outside a committing store's want.
      if (dmem_we && !(store_commit_want[0] | store_commit_want[1])) begin
        $fatal(1, "rv32i_ss_core: dmem_we asserted outside a committing store's window");
      end

      if (store_commit_fire && !(dmem_valid || sq_accept_deferred)) begin
        $fatal(1, "rv32i_ss_core: store commit without a presented or recorded request");
      end

      if (cdb_q[0].valid && cdb_q[0].rd_wen && (cdb_q[0].pdst == '0)) begin
        $fatal(1, "rv32i_ss_core: CDB lane 0 writes p0 for a real destination");
      end
      if (cdb_q[1].valid && cdb_q[1].rd_wen && (cdb_q[1].pdst == '0)) begin
        $fatal(1, "rv32i_ss_core: CDB lane 1 writes p0 for a real destination");
      end

      if (jump_inflight_q && jump_alloc_fire) begin
        $fatal(1, "rv32i_ss_core: allocated a new jump while a jump is in flight");
      end

      if (redirect_valid && !(branch_recover_req | jump_resolve_fire
                              | trap_q_valid | jal_deser_fire
                              | ras_decode_redirect_fire)) begin
        $fatal(1, "rv32i_ss_core: redirect asserted without branch recover, jump resolve, trap, JAL dispatch, or RAS-predicted return");
      end

      // a de-serialized JAL dispatch is mutually exclusive with every
      // higher-priority redirect cause -- dispatch_accept blocks bundle_fire
      // on recovery / trap / inflight-serialized-jump cycles. If this fires,
      // the dispatch gate has been weakened and two redirect sources are
      // presenting in one cycle.
      if (jal_deser_fire && (branch_recover_req || trap_q_valid
                             || jump_inflight_q)) begin
        $fatal(1, "rv32i_ss_core: JAL dispatch redirect coincides with recovery/trap/serialized-jump");
      end

      // mirror of the pin above for the predicted-return dispatch
      // redirect (same dispatch_accept exclusivity argument).
      if (ras_decode_redirect_fire && (branch_recover_req || trap_q_valid
                                       || jump_inflight_q)) begin
        $fatal(1, "rv32i_ss_core: RAS-predicted dispatch redirect coincides with recovery/trap/serialized-jump");
      end

      // a learned-site return already steered its request stream. It
      // still dispatches, pops, checkpoints, and verifies, but must not fire
      // a decode redirect a second time.
      if (ras_pred_fire && ras_fetch_pred_want && redirect_valid) begin
        $fatal(1, "rv32i_ss_core: fetch-predicted return redirected again at dispatch");
      end

      // injection seam: a predicted return fires only through rename's
      // checkpoint_ok gate, so its dispatch cycle MUST carry an allocation.
      if (ras_pred_fire && !rename_checkpoint_valid) begin
        $fatal(1, "rv32i_ss_core: predicted return dispatched without a checkpoint allocation");
      end

      // A naturally latched trap/mret retains its ROB-head facts throughout
      // the registered flush cycle. Qualify by the prior latch event so direct
      // testbench forcing of trap_q_valid does not invent an architectural origin.
      if (tw_trap_latch_prev_q && (!trap_q_valid || !trap_commit_event)) begin
        $fatal(1, "rv32i_ss_core: latched trap/mret lost its held commit event");
      end
    end

    // Sample unconditionally so a reset crossing cannot leave a stale origin
    // bit that falsely attributes a later forced seam value to the RTL latch.
    tw_trap_latch_prev_q <= (rst_n === 1'b1) && trap_latch_fire;

    // Sample holder stability through reset as well. Reset clears valid, so
    // the sampled held flags clear and cannot compare against a removed packet.
    tw_alu0_held_prev_q <= alu0_complete.valid && !cdb_grant_alu0;
    tw_alu0_prev_q      <= alu0_complete;
    tw_alu1_held_prev_q <= alu1_complete.valid && !cdb_grant_alu1;
    tw_alu1_prev_q      <= alu1_complete;
    tw_agen_held_prev_q <= agen_complete.valid && !agen_complete_accept;
    tw_agen_prev_q      <= agen_complete;
  end
`endif


`ifndef SYNTHESIS
  /* verilator lint_off SYNCASYNCNET */
  // FORMATION PIN (permanent): at most ONE memory op per valid
  // bundle — the LSQ is one-allocation-wide by contract and the
  // mem_slot_idx metadata mux assumes a unique mem slot.
  always @(posedge clk) begin
    if (rst_n === 1'b1) begin
      // Check every presented bundle, including one stalled behind a full
      // queue. Illegal memory-operation pairing must not wait for acceptance
      // to become visible.
      if (decoded_valid &&
          (({1'b0, (decoded_slot_valid[0] & (decoded_is_load[0] | decoded_is_store[0]))} +
            {1'b0, (decoded_slot_valid[1] & (decoded_is_load[1] | decoded_is_store[1]))}) > 2'd1)) begin
        $fatal(1, "rv32i_ss_core: more than one memory op in an OFFERED bundle (formation SOLO-mem guarantee)");
      end

      // Trap, CSR and jump instructions occupy slot 0 alone. Scalar trap/CSR
      // fields prevent independent reconstruction of slot-1 detail here, so the
      // frontend checks those classes. This boundary also checks slot-0 solo
      // instructions and either slot's jump classification.
      if (decoded_valid && (bundle_size == 2'd2) &&
          (decoded_trap.valid || decoded_trap.is_mret ||
           (decoded_csr_op != rv32i_pipeline_pkg::CSR_NONE) ||
           ((decoded_op_class[0] == OOO_OP_JUMP) && decoded_slot_valid[0]) ||
           ((decoded_op_class[1] == OOO_OP_JUMP) && decoded_slot_valid[1]))) begin
        $fatal(1, "rv32i_ss_core: trap/CSR/jump op in a size-2 bundle (formation SOLO guarantee)");
      end

      // At most one checkpoint request is allowed per presented bundle.
      // Otherwise two instructions would share the single allocated checkpoint ID.
      if (decoded_valid &&
          ((decoded_needs_checkpoint & decoded_slot_valid) == 2'b11)) begin
        $fatal(1, "rv32i_ss_core: two branches in one OFFERED bundle (formation <=1-branch guarantee)");
      end
    end
  end
  /* verilator lint_on SYNCASYNCNET */
`endif

endmodule
