// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// Five-stage pipelined RV32IM core with precise machine-mode traps.
//
// Stages: F (fetch) -> D (decode/regread) -> E (execute) -> M (memory) -> W (writeback)
// Per-stage signal suffix: _f / _d / _e / _m / _w
//
// Reuses the single-cycle leaf modules:
//   rv32i_alu, rv32i_imm_gen, rv32i_regfile
// Adds:
//   rv32i_pipe_decode  (richer pipe_ctrl_t)
//   rv32i_branch_cmp   (conditional-branch resolver in E)
//   rv32i_muldiv       (multi-cycle iterative mul/div for RV32M)
//   rv32i_hazard       (forwarding + load-use stall + branch flush)
//
// Current pipeline policy:
//   - Pipeline registers support hold, flush, and bubble injection.
//   - rv32i_hazard handles EX/MEM and MEM/WB forwarding, load-use stalls,
//     multi-cycle muldiv stalls, and taken-branch/jump flushes.
//   - Same-cycle W -> D register dependencies are handled with an explicit
//     decode-stage bypass mux around the reused rv32i_regfile.
//
// Pipeline payload (uop_t):
//   The decoded instruction travels D -> E -> M -> W as a single uop_t
//   packet (pc, instr, imm, reg data/addrs, BP/trap/CSR metadata,
//   pipe_ctrl_t).
//   Pipeline registers move the packet whole; stage logic reads
//   uop_x.field directly. The only aliases kept are the W-stage retire
//   names (pc_w/instr_w/ctrl_w/rd_addr_w/pc_plus4_w), which form the
//   de-facto retire interface that the trace TBs reference hierarchically.
//
// Retirement observability:
//   - pipe_ctrl_t.valid is set in decode and ANDed with fetch_valid_d_r so
//     bubbles, wrong-path flushes, and re-decoded reset NOPs all reach W
//     with valid=0. The W-stage trace gates on it.
//   - M-stage dmem_addr/wdata/be are snapshotted into MEM/WB so the
//     W-stage trace reports the values the RETIRING store actually drove,
//     not the next M-cycle's signals.
//   - sim_halt rides ctrl through the pipeline and exposes the raw
//     uop_w.ctrl.sim_halt bit at W, rather than detecting the fetch word.
module rv32i_pipeline_core
  import fyp_cpu_pkg::*;
  import rv32i_pipeline_pkg::*;
(
  input  logic        clk,
  input  logic        rst_n,

  output word_t       imem_addr,
  output logic        imem_req_valid,
  input  logic [31:0] imem_rdata,
  input  logic        imem_req_ready,
  input  logic        imem_resp_valid,

  output logic        dmem_we,
  output logic [3:0]  dmem_be,      // byte enables for sb/sh/sw
  output word_t       dmem_addr,
  output word_t       dmem_wdata,
  output logic        dmem_req_valid,
  output logic        dmem_req_write,
  input  logic        dmem_req_ready,
  input  logic        dmem_resp_valid,
  output logic        sim_halt,
  input  word_t       dmem_rdata
);
  // ============================================================
  // Hazard / forwarding signals (driven by rv32i_hazard below)
  // ============================================================
  fwd_sel_e forward_a_e;
  fwd_sel_e forward_b_e;
  logic     stall_f;
  logic     stall_d;
  logic     stall_e;
  logic     stall_m;
  logic     flush_d;
  logic     flush_e;
  logic     bubble_m;
  logic     bubble_w;
  logic     redirect_any;
  word_t     redirect_target;
  // ============================================================
  // F : Fetch
  // ============================================================
  word_t       pc_f;
  word_t       pc_plus4_f;
  word_t       pc_next_f;
  logic [31:0] instr_f;

  // ==================================================================
  // imem request/response handshake — naming mirrors the dmem M-stage
  // lifecycle (dmem_op_m / dmem_load_req_needed_m / _accepted_m /
  // _outstanding_q / dmem_load_resp_expected_m / dmem_done_m). Fetch is
  // always a read, so there is no write/op-class split.
  //
  //   imem_op_f             : F is issuing an instruction fetch this cycle
  //                           (analog of dmem_op_m). Fetch is unconditional, so
  //                           this is tied to 1'b1; request gating is done by
  //                           imem_req_needed_f. Kept as a named signal for
  //                           symmetry with the dmem lifecycle.
  //   imem_req_needed_f     : fetch request not yet accepted — still to issue
  //                           (analog of dmem_load_req_needed_m)
  //   imem_req_accepted_f   : fetch request handshake fires this cycle
  //                           (= imem_req_valid && imem_req_ready)
  //                           (analog of dmem_load_req_accepted_m)
  //   imem_req_outstanding_q: request accepted, waiting for imem_resp_valid
  //                           (analog of dmem_load_req_outstanding_q)
  //   imem_resp_expected_f  : a fetch response is due this cycle
  //                           (analog of dmem_load_resp_expected_m)
  //   imem_done_f           : instruction word is valid this cycle; F/D may
  //                           advance (analog of dmem_done_m)
  //
  // Implemented: one-shot request (imem_req_valid = imem_op_f &&
  // imem_req_needed_f). imem_req_outstanding_q tracks an accepted-but-unanswered
  // fetch and CANCELS on redirect_any, so a stale wrong-path response cannot be
  // mistaken for the redirected instruction. PC advance and the IF/ID latch gate
  // on imem_done_f (not bare imem_resp_valid). Serves both 0-latency (response
  // same cycle) and N-latency fetches.
  // ==================================================================
  logic imem_op_f;
  logic imem_req_needed_f;
  logic imem_req_accepted_f;
  logic imem_req_outstanding_q;
  logic imem_resp_expected_f;
  logic imem_done_f;
  logic imem_wait_f;

  // Branch predictor plumbing. The predictor makes a fetch-stage guess and
  // is trained from the resolved conditional branch in E. The prediction
  // metadata rides with the instruction so E can later detect mispredicts.
  logic        bp_pred_valid_f;
  logic        bp_pred_taken_f;
  word_t       bp_pred_target_f;
  word_t       bp_next_pc_f;
  logic        bp_pred_taken_d;
  word_t       bp_pred_target_d;
  logic        bp_update_valid_e;
  word_t       bp_update_pc_e;
  logic        bp_update_taken_e;
  word_t       bp_update_target_e;

  // PC redirect sources: E (taken branch / jump) and W (trap / mret).
  // Declared here (above their first use in F/E/M gating) and arbitrated into
  // redirect_any / redirect_target further below.
  logic        redirect_e;
  word_t       redirect_target_e;
  logic        redirect_w;
  word_t       redirect_target_w;

  assign pc_plus4_f   = pc_f + 32'd4;
  assign bp_next_pc_f = (bp_pred_valid_f && bp_pred_taken_f) ? bp_pred_target_f
                                                             : pc_plus4_f;
  assign pc_next_f    = bp_next_pc_f;
  assign imem_addr    = pc_f;
  assign instr_f      = imem_rdata;

  // imem handshake — one-shot request lifecycle (mirrors the dmem load
  // request). A held fetch is requested once, not re-issued every wait cycle;
  // imem_req_outstanding_q (below) holds the accepted state and cancels on
  // redirect_any.
  assign imem_op_f = 1'b1;
  assign imem_req_needed_f = !imem_req_outstanding_q;
  assign imem_req_accepted_f = imem_req_valid && imem_req_ready;
  assign imem_resp_expected_f = imem_req_outstanding_q || imem_req_accepted_f;
  assign imem_req_valid = imem_op_f && imem_req_needed_f;
  assign imem_done_f = imem_resp_expected_f && imem_resp_valid;
  assign imem_wait_f = !imem_done_f;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pc_f <= 32'd0;
    end else if (redirect_any) begin
      pc_f <= redirect_target;
    end else if (!stall_f && imem_done_f) begin
      pc_f <= pc_next_f;
    end
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      imem_req_outstanding_q <= '0;
    end else if(redirect_any) begin
      imem_req_outstanding_q <= '0;   // cancel in-flight fetch so a stale wrong-path response cannot latch
    end else if (imem_done_f) begin
      imem_req_outstanding_q <= '0;
    end else if (imem_req_accepted_f) begin
      imem_req_outstanding_q <= 1'b1;
    end
  end
  // ============================================================
  // IF/ID pipeline register
  // ============================================================
  word_t       pc_d;
  word_t       pc_plus4_d;
  logic [31:0] instr_d;
  // fetch_valid_d_r: 1 iff instr_d/pc_d came from a real fetch (not reset
  // or flush_d's NOP injection). Gates ctrl_d.valid before it enters the
  // ID/EX latch so re-decoded reset/flush NOPs don't propagate as committed
  // instructions at the W-stage trace.
  logic        fetch_valid_d_r;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pc_d             <= '0;
      pc_plus4_d       <= '0;
      instr_d          <= 32'h00000013;       // canonical NOP (addi x0,x0,0)
      bp_pred_taken_d  <= 1'b0;
      bp_pred_target_d <= '0;
      fetch_valid_d_r  <= 1'b0;
    end else if (flush_d) begin
      pc_d             <= '0;
      pc_plus4_d       <= '0;
      instr_d          <= 32'h00000013;
      bp_pred_taken_d  <= 1'b0;
      bp_pred_target_d <= '0;
      fetch_valid_d_r  <= 1'b0;                // flushed = not a real fetch
    end else if (!stall_d) begin
      if (imem_done_f) begin
        pc_d             <= pc_f;
        pc_plus4_d       <= pc_plus4_f;
        instr_d          <= instr_f;
        bp_pred_taken_d  <= bp_pred_taken_f;
        bp_pred_target_d <= bp_pred_target_f;
        fetch_valid_d_r  <= 1'b1;                // real fetch from F
      end else begin
        pc_d             <= '0;
        pc_plus4_d       <= '0;
        instr_d          <= 32'h00000013;       // canonical NOP (addi x0,x0,0)
        bp_pred_taken_d  <= 1'b0;
        bp_pred_target_d <= '0;
        fetch_valid_d_r  <= 1'b0;
      end
    end
    // else: stall — hold all fields including fetch_valid_d_r
  end

  rv32i_bp u_bp (
    .clk              (clk),
    .rst_n            (rst_n),
    .pc_f             (pc_f),
    .predict_taken_f  (bp_pred_taken_f),
    .predict_target_f (bp_pred_target_f),
    .predict_valid_f  (bp_pred_valid_f),
    .update_valid_e   (bp_update_valid_e),
    .update_pc_e      (bp_update_pc_e),
    .update_taken_e   (bp_update_taken_e),
    .update_target_e  (bp_update_target_e)
  );
  // ============================================================
  // D : Decode + register read + immediate gen
  // ============================================================
  reg_addr_t  rs1_addr_d;
  reg_addr_t  rs2_addr_d;
  reg_addr_t  rd_addr_d;
  pipe_ctrl_t ctrl_d;
  word_t      imm_d;
  word_t      rs1_data_d_raw;
  word_t      rs2_data_d_raw;
  word_t      rs1_data_d;
  word_t      rs2_data_d;
  uop_t       uop_d;

  assign rs1_addr_d = instr_d[19:15];
  assign rs2_addr_d = instr_d[24:20];
  assign rd_addr_d  = instr_d[11:7];

  rv32i_pipe_decode u_decode (
    .instr (instr_d),
    .ctrl  (ctrl_d)
  );

  // Gate ctrl_d.valid with fetch_valid_d_r: a NOP re-decoded from a flushed
  // IF/ID register must enter the pipeline as a true bubble, not as an
  // invalid NOP that still carries reg_write-to-x0 decode side effects.
  pipe_ctrl_t ctrl_d_gated;
  always_comb begin
    if (fetch_valid_d_r) begin
      ctrl_d_gated = ctrl_d;
    end else begin
      ctrl_d_gated = '0;
    end
  end

  rv32i_imm_gen u_imm_gen (
    .instr   (instr_d),
    .imm_sel (ctrl_d.imm_sel),
    .imm     (imm_d)
  );

  // Regfile write-back signals (defined in W stage below; declared early so
  // the regfile instantiation here can reference them).
  uop_t      uop_w;
  reg_addr_t rd_addr_w;
  word_t     result_w;

  rv32i_regfile u_regfile (
    .clk      (clk),
    .rst_n    (rst_n),
    .write_en (uop_w.ctrl.reg_write),
    .rs1_addr (rs1_addr_d),
    .rs2_addr (rs2_addr_d),
    .rd_addr  (uop_w.rd_addr),
    .rd_data  (result_w),
    .rs1_data (rs1_data_d_raw),
    .rs2_data (rs2_data_d_raw)
  );

  // Same-cycle W -> D bypass. The regfile writes on the rising edge, so a
  // value being written this cycle is not visible to the asynchronous read
  // until next cycle. Bypass it directly when rd_w matches rs?_d.
  always_comb begin
    if (uop_w.ctrl.reg_write && (uop_w.rd_addr != 5'd0) &&
        (uop_w.rd_addr == rs1_addr_d)) begin
      rs1_data_d = result_w;
    end else begin
      rs1_data_d = rs1_data_d_raw;
    end

    if (uop_w.ctrl.reg_write && (uop_w.rd_addr != 5'd0) &&
        (uop_w.rd_addr == rs2_addr_d)) begin
      rs2_data_d = result_w;
    end else begin
      rs2_data_d = rs2_data_d_raw;
    end
  end

  // Combinational D-stage payload. This is not a pipeline register; it is the
  // decoded packet captured by ID/EX when that stage advances.
  always_comb begin
    uop_d                 = uop_bubble();
    uop_d.pc              = pc_d;
    uop_d.pc_plus4        = pc_plus4_d;
    uop_d.instr           = instr_d;
    uop_d.imm             = imm_d;
    uop_d.rs1_data        = rs1_data_d;
    uop_d.rs2_data        = rs2_data_d;
    uop_d.rs1_addr        = rs1_addr_d;
    uop_d.rs2_addr        = rs2_addr_d;
    uop_d.rd_addr         = rd_addr_d;
    uop_d.bp_pred_taken   = bp_pred_taken_d;
    uop_d.bp_pred_target  = bp_pred_target_d;
    uop_d.ctrl            = ctrl_d_gated;
    uop_d.csr_addr        = instr_d[31:20];
    uop_d.csr_zimm        = instr_d[19:15];
  end

  // ============================================================
  // ID/EX pipeline register
  // ============================================================
  uop_t uop_e;
  word_t fwd_rs1_e;
  word_t fwd_rs2_e;
  logic  dmem_m_load_use_stall;
  logic  csr_m_use_stall;
  logic  is_csr_m;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      uop_e <= uop_bubble();
    end else if (stall_e) begin
      // Hold E while a multi-cycle op or M-stage memory wait is active.
      if (!dmem_m_load_use_stall) begin
        uop_e.rs1_data <= fwd_rs1_e;
        uop_e.rs2_data <= fwd_rs2_e;
      end
    end else if (flush_e) begin
      // Load-use bubble: only applies when E can advance into M.
      uop_e <= uop_bubble();
    end else begin
      uop_e <= uop_d;
    end
  end

  // ============================================================
  // E : Execute
  // ============================================================
  word_t alu_a_e;
  word_t alu_b_e;
  word_t alu_result_e;
  logic  alu_zero_e;
  word_t result_m;      // forwarding source from M stage (declared early)
  word_t muldiv_result_e;
  logic  muldiv_busy;
  logic  muldiv_done;
  logic  e_fire;
  logic  branch_taken_e;
  word_t result_e;
  logic  branch_is_e;
  word_t branch_target_e;
  logic  branch_mispredict_e;
  logic  iaddr_misalign_e;
  word_t iaddr_bad_target_e;
  logic  dmem_wait_m;
  logic  dmem_m_wait_use_hazard;
  logic  e_redirect_ready;

  assign e_fire = uop_e.ctrl.valid && !stall_e;
  assign e_redirect_ready = uop_e.ctrl.valid &&
                            !dmem_m_wait_use_hazard &&
                            !dmem_m_load_use_stall &&
                            !csr_m_use_stall;

  assign branch_is_e = uop_e.ctrl.valid && (uop_e.ctrl.branch_op != BR_NONE);
  assign branch_target_e = uop_e.pc + uop_e.imm;
  assign branch_mispredict_e = branch_is_e &&
                               ((uop_e.bp_pred_taken != branch_taken_e) ||
                                (branch_taken_e &&
                                 (uop_e.bp_pred_target != branch_target_e)));
  // Forwarding muxes. The hazard unit selects the newest available producer:
  // M-stage results take priority over W-stage results.
  always_comb begin
    fwd_rs1_e = uop_e.rs1_data;
    unique case (forward_a_e)
      FWD_NONE:   fwd_rs1_e = uop_e.rs1_data;
      FWD_FROM_M: fwd_rs1_e = result_m;
      FWD_FROM_W: fwd_rs1_e = result_w;
      default:    fwd_rs1_e = uop_e.rs1_data;
    endcase

    fwd_rs2_e = uop_e.rs2_data;
    unique case (forward_b_e)
      FWD_NONE:   fwd_rs2_e = uop_e.rs2_data;
      FWD_FROM_M: fwd_rs2_e = result_m;
      FWD_FROM_W: fwd_rs2_e = result_w;
      default:    fwd_rs2_e = uop_e.rs2_data;
    endcase
  end

  // ALU operand A / B muxes (same enum as single-cycle).
  always_comb begin
    alu_a_e = fwd_rs1_e;
    unique case (uop_e.ctrl.alu_a_sel)
      ALU_A_RS1:  alu_a_e = fwd_rs1_e;
      ALU_A_PC:   alu_a_e = uop_e.pc;
      ALU_A_ZERO: alu_a_e = '0;
      default:    alu_a_e = fwd_rs1_e;
    endcase

    alu_b_e = fwd_rs2_e;
    unique case (uop_e.ctrl.alu_b_sel)
      ALU_B_RS2: alu_b_e = fwd_rs2_e;
      ALU_B_IMM: alu_b_e = uop_e.imm;
      default:   alu_b_e = fwd_rs2_e;
    endcase
  end

  rv32i_alu u_alu (
    .operand_a (alu_a_e),
    .operand_b (alu_b_e),
    .alu_op    (uop_e.ctrl.alu_op),
    .result    (alu_result_e),
    .zero      (alu_zero_e)
  );

  rv32i_branch_cmp u_branch_cmp (
    .branch_op (uop_e.ctrl.branch_op),
    .a         (fwd_rs1_e),
    .b         (fwd_rs2_e),
    .taken     (branch_taken_e)
  );

  // Multi-cycle muldiv. start = level held while a muldiv op is in E and
  // the unit is IDLE; goes low automatically once the FSM enters RUN.
  logic muldiv_start_e;
  assign muldiv_start_e = uop_e.ctrl.valid && uop_e.ctrl.is_muldiv
                          && !dmem_wait_m && !dmem_m_load_use_stall &&
                          !csr_m_use_stall &&
                          !muldiv_busy && !redirect_w;

  rv32i_muldiv u_muldiv (
    .clk    (clk),
    .rst_n  (rst_n),
    .start  (muldiv_start_e),
    .op     (uop_e.ctrl.muldiv_op),
    .a      (fwd_rs1_e),
    .b      (fwd_rs2_e),
    .busy   (muldiv_busy),
    .done   (muldiv_done),
    .result (muldiv_result_e)
  );

  // E -> F redirect and branch-predictor training.
  //   jal/jalr     : always redirect; this predictor does not handle jumps.
  //   branch miss  : redirect to actual target if taken, else uop_e.pc_plus4.
  //   bogus hit    : a predicted-taken non-control instruction falls through.
  //   branch train : conditional branches only, gated by uop_e.ctrl.valid.
  always_comb begin
    redirect_e        = 1'b0;
    redirect_target_e = uop_e.pc_plus4;
    iaddr_misalign_e  = 1'b0;
    iaddr_bad_target_e = 32'h0;
    // First compute whether the resolved control-flow target is misaligned.
    if (e_redirect_ready && uop_e.ctrl.is_jump) begin
      if (uop_e.ctrl.is_jalr) begin
        iaddr_bad_target_e = (fwd_rs1_e + uop_e.imm) & ~32'h0000_0001;
      end else begin
        iaddr_bad_target_e = branch_target_e;
      end

      if (iaddr_bad_target_e[1:0] != 2'b00) begin
        iaddr_misalign_e = 1'b1;
      end
    end else if (e_redirect_ready && branch_is_e && branch_taken_e) begin
      iaddr_bad_target_e = branch_target_e;

      if (branch_target_e[1:0] != 2'b00) begin
        iaddr_misalign_e = 1'b1;
      end
    end

    // Normal redirects are suppressed when target misalignment retags the uop
    // as a precise instruction-address-misalignment trap.
    if (iaddr_misalign_e) begin
      redirect_e = 1'b0;
      redirect_target_e = uop_e.pc_plus4;
    end else if (e_redirect_ready && uop_e.ctrl.is_jump) begin
      redirect_e = 1'b1;
      redirect_target_e = iaddr_bad_target_e;
    end else if (e_redirect_ready && branch_mispredict_e) begin
      redirect_e = 1'b1;
      redirect_target_e = branch_taken_e ? branch_target_e : uop_e.pc_plus4;
    end else if (e_redirect_ready &&
                 !branch_is_e && !uop_e.ctrl.is_jump &&
                 uop_e.bp_pred_taken) begin
      redirect_e        = 1'b1;
      redirect_target_e = uop_e.pc_plus4;
    end

    bp_update_valid_e  = branch_is_e && e_fire && !redirect_w && !iaddr_misalign_e;
    bp_update_pc_e     = uop_e.pc;
    bp_update_taken_e  = branch_taken_e;
    bp_update_target_e = branch_target_e;
  end

  // E-stage result (only valid for ops that finish in E; loads finish in M).
  always_comb begin
    result_e = alu_result_e;
    unique case (uop_e.ctrl.result_src)
      RES_ALU:    result_e = alu_result_e;
      RES_PC4:    result_e = uop_e.pc_plus4;
      RES_MULDIV: result_e = muldiv_result_e;
      RES_MEM:    result_e = alu_result_e;   // address; data arrives in W
      default:    result_e = alu_result_e;
    endcase
  end

  // ============================================================
  // EX/MEM pipeline register
  // ============================================================
  uop_t  uop_m;
  uop_t  uop_e_forwarded;
  word_t write_data_m;            // store data forwarded from rs2

  always_comb begin
    uop_e_forwarded          = uop_e;
    uop_e_forwarded.rs1_data = fwd_rs1_e;
    uop_e_forwarded.rs2_data = fwd_rs2_e;

    if (iaddr_misalign_e) begin
      uop_e_forwarded.ctrl.trap_op = TRAP_IADDR_MISALIGN;
      uop_e_forwarded.trap_tval = iaddr_bad_target_e;
      uop_e_forwarded.ctrl.reg_write = 1'b0;
      uop_e_forwarded.ctrl.mem_write = 1'b0;
      uop_e_forwarded.ctrl.mem_read = 1'b0;
      uop_e_forwarded.ctrl.is_jump = 1'b0;
      uop_e_forwarded.ctrl.branch_op = BR_NONE;
    end
  end
  assign dmem_m_wait_use_hazard = dmem_wait_m &&
                                  uop_m.ctrl.mem_read &&
                                  (uop_m.rd_addr != 5'd0) &&
                                  uop_e.ctrl.valid &&
                                  ((uop_m.rd_addr == uop_e.rs1_addr) ||
                                   (uop_m.rd_addr == uop_e.rs2_addr));

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      uop_m        <= uop_bubble();
      result_m     <= '0;
      write_data_m <= '0;
    end else if (stall_m) begin
      // Hold M-stage memory op until request/response completes.
    end else if (bubble_m) begin
      // Inject bubble (used while muldiv is computing in E).
      uop_m        <= uop_bubble();
      result_m     <= '0;
      write_data_m <= '0;
    end else begin
      uop_m        <= uop_e_forwarded;
      result_m     <= result_e;
      write_data_m <= fwd_rs2_e;
    end
  end

  // ============================================================
  // M : Memory
  // ============================================================
  // Address LSBs select the lane within the 32-bit word.
  logic [1:0] byte_off_m;
  word_t      mem_data_m;
  logic [3:0] dmem_be_next;
  word_t      dmem_wdata_next;
  logic       dmem_op_m;
  logic       dmem_load_req_needed_m;
  logic       dmem_load_req_accepted_m;
  logic       dmem_load_req_outstanding_q;
  logic       dmem_load_resp_expected_m;
  logic       dmem_load_done_m;
  logic       dmem_store_done_m;
  logic       dmem_done_m;
  logic       load_misalign_m;
  logic       store_misalign_m;
  logic       data_misalign_m;

  assign load_misalign_m = uop_m.ctrl.valid && uop_m.ctrl.mem_read &&
                           ((uop_m.ctrl.mem_size == MEM_W && result_m[1:0] != 2'b00)||
                           (uop_m.ctrl.mem_size == MEM_H && result_m[0] != 1'b0));
  assign store_misalign_m = uop_m.ctrl.valid && uop_m.ctrl.mem_write &&
                           ((uop_m.ctrl.mem_size == MEM_W && result_m[1:0] != 2'b00)||
                           (uop_m.ctrl.mem_size == MEM_H && result_m[0] != 1'b0));
  assign data_misalign_m = load_misalign_m || store_misalign_m;
  assign dmem_op_m = uop_m.ctrl.valid &&
                         (uop_m.ctrl.mem_read || uop_m.ctrl.mem_write) && !data_misalign_m;
  // M-stage CSR op: its rd value is produced at W, so it must be excluded from
  // M-forwarding (named wire — iverilog mis-elaborates a compare expression
  // placed directly in the hazard port connection).
  assign is_csr_m = (uop_m.ctrl.csr != CSR_NONE);
  assign dmem_load_req_needed_m = uop_m.ctrl.mem_read &&
                                  !dmem_load_req_outstanding_q && !data_misalign_m;
  assign dmem_load_req_accepted_m = dmem_req_valid &&
                                  !dmem_req_write &&
                                  dmem_req_ready;
  assign dmem_load_resp_expected_m = uop_m.ctrl.mem_read &&
                                    (dmem_load_req_outstanding_q ||
                                     dmem_load_req_accepted_m);
  assign dmem_load_done_m = !uop_m.ctrl.mem_read || data_misalign_m ||
                            (dmem_load_resp_expected_m && dmem_resp_valid);
  assign dmem_store_done_m = !uop_m.ctrl.mem_write || dmem_req_ready || data_misalign_m;
  assign dmem_done_m       = dmem_load_done_m && dmem_store_done_m;

  assign dmem_req_valid = !redirect_w && dmem_op_m &&
                          (uop_m.ctrl.mem_write || dmem_load_req_needed_m) && !data_misalign_m;
  assign dmem_req_write = uop_m.ctrl.mem_write && !data_misalign_m;

  assign byte_off_m = result_m[1:0];
  assign dmem_addr  = result_m;
  assign dmem_we    = !redirect_w && uop_m.ctrl.valid &&
                      uop_m.ctrl.mem_write &&
                      dmem_req_ready && !data_misalign_m;

  // Remember that the current M-stage load request has been accepted. While
  // the load is held in M waiting for its response, do not issue it again.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dmem_load_req_outstanding_q <= 1'b0;
    end else if (uop_m.ctrl.valid && uop_m.ctrl.mem_read && stall_m) begin
      if (dmem_load_req_accepted_m) begin
        dmem_load_req_outstanding_q <= 1'b1;
      end
    end else begin
      dmem_load_req_outstanding_q <= 1'b0;
    end
  end
  // Store-side: shift the source bytes to the correct lane and form byte
  // enables. Halfword writes are 16-bit aligned; byte writes can land in any
  // of the four lanes. Word writes drop straight through.
  always_comb begin
    dmem_wdata_next = '0;
    dmem_be_next    = 4'b0000;

    unique case (uop_m.ctrl.mem_size)
      MEM_W: begin
        dmem_wdata_next = write_data_m;
        dmem_be_next    = 4'b1111;
      end
      MEM_H: begin
        // halfword: bit 1 of address picks low/high half
        if (byte_off_m[1]) begin
          dmem_wdata_next = {write_data_m[15:0], 16'h0};
          dmem_be_next    = 4'b1100;
        end else begin
          dmem_wdata_next = {16'h0, write_data_m[15:0]};
          dmem_be_next    = 4'b0011;
        end
      end
      MEM_B: begin
        unique case (byte_off_m)
          2'b00: begin
            dmem_wdata_next = {24'h0, write_data_m[7:0]};
            dmem_be_next    = 4'b0001;
          end
          2'b01: begin
            dmem_wdata_next = {16'h0, write_data_m[7:0], 8'h0};
            dmem_be_next    = 4'b0010;
          end
          2'b10: begin
            dmem_wdata_next = {8'h0, write_data_m[7:0], 16'h0};
            dmem_be_next    = 4'b0100;
          end
          2'b11: begin
            dmem_wdata_next = {write_data_m[7:0], 24'h0};
            dmem_be_next    = 4'b1000;
          end
          default: begin
            dmem_wdata_next = '0;
            dmem_be_next    = 4'b0000;
          end
        endcase
      end
      default: begin
        dmem_wdata_next = write_data_m;
        dmem_be_next    = 4'b1111;
      end
    endcase
  end

  assign dmem_wdata = dmem_wdata_next;
  assign dmem_be    = uop_m.ctrl.mem_write ? dmem_be_next : 4'b0000;

  // Load-side: extract the lane from dmem_rdata and sign/zero-extend.
  logic [15:0] half_raw;
  logic [7:0]  byte_raw;

  always_comb begin
    // halfword lane select (bit 1 of address)
    half_raw = byte_off_m[1] ? dmem_rdata[31:16] : dmem_rdata[15:0];

    // byte lane select
    unique case (byte_off_m)
      2'b00:  byte_raw = dmem_rdata[7:0];
      2'b01:  byte_raw = dmem_rdata[15:8];
      2'b10:  byte_raw = dmem_rdata[23:16];
      2'b11:  byte_raw = dmem_rdata[31:24];
      default: byte_raw = dmem_rdata[7:0];
    endcase

    unique case (uop_m.ctrl.mem_size)
      MEM_W: mem_data_m = dmem_rdata;
      MEM_H: mem_data_m = uop_m.ctrl.mem_unsigned
                            ? {16'h0, half_raw}
                            : {{16{half_raw[15]}}, half_raw};
      MEM_B: mem_data_m = uop_m.ctrl.mem_unsigned
                            ? {24'h0, byte_raw}
                            : {{24{byte_raw[7]}}, byte_raw};
      default: mem_data_m = dmem_rdata;
    endcase
  end

  // ============================================================
  // MEM/WB pipeline register
  // ============================================================
  word_t      result_pre_w;
  word_t      mem_data_w;
  word_t      pc_plus4_w;
  word_t      pc_w;                  // carried for trace (retiring instr PC)
  pipe_ctrl_t ctrl_w;
  logic [31:0] instr_w;              // carried for trace (retiring instr word)
  // Snapshot of M-stage memory-interface signals so the W-stage trace can
  // report the values that the *retiring* store actually drove to memory.
  // Without this, the trace would read dmem_addr/wdata/be from the *next*
  // M-stage cycle (because those signals are combinational from uop_m.ctrl).
  // These stay outside uop_t: they are trace observability, not payload.
  word_t       dmem_addr_w;
  word_t       dmem_wdata_w;
  logic [3:0]  dmem_be_w;
  word_t       csr_rdata_w;
  word_t       csr_wdata_w;
  word_t       csr_src_w;
  logic        csr_we_w;
  logic        trap_we_w;
  logic        mret_we_w;
  word_t       trap_pc_w;
  word_t       trap_cause_w;
  word_t       trap_mtval_w;
  word_t       mtvec_w;
  word_t       mepc_w;
  uop_t        uop_m_to_w;

  assign trap_pc_w = uop_w.pc;

  always_comb begin
    trap_we_w = 1'b0;
    trap_mtval_w = '0;
    mret_we_w = 1'b0;
    trap_cause_w = 32'd0;
    redirect_w = 1'b0;
    redirect_target_w = 32'd0;
    uop_m_to_w = uop_m;
    if (load_misalign_m || store_misalign_m) begin
      if (load_misalign_m) begin
        uop_m_to_w.ctrl.trap_op = TRAP_LOAD_MISALIGN;
      end else begin
        uop_m_to_w.ctrl.trap_op = TRAP_STORE_MISALIGN;
      end
      uop_m_to_w.trap_tval = result_m;
      uop_m_to_w.ctrl.reg_write = 1'b0;
      uop_m_to_w.ctrl.mem_read = 1'b0;
      uop_m_to_w.ctrl.mem_write = 1'b0;
    end
    if (uop_w.ctrl.valid) begin
      unique case (uop_w.ctrl.trap_op)
        TRAP_ILLEGAL: begin
          trap_we_w = 1'b1;
          trap_cause_w = 32'd2;
          trap_mtval_w = uop_w.instr;
          redirect_w = 1'b1;
          redirect_target_w = mtvec_w & ~32'h0000_0003;
        end
        TRAP_EBREAK: begin
          trap_we_w = 1'b1;
          trap_cause_w = 32'd3;
          trap_mtval_w = uop_w.pc;
          redirect_w = 1'b1;
          redirect_target_w = mtvec_w & ~32'h0000_0003;
        end
        TRAP_ECALL: begin
          trap_we_w = 1'b1;
          trap_cause_w = 32'd11;
          trap_mtval_w = '0;
          redirect_w = 1'b1;
          redirect_target_w = mtvec_w & ~32'h0000_0003;
        end
        TRAP_MRET: begin
          mret_we_w = 1'b1;
          redirect_w = 1'b1;
          redirect_target_w = mepc_w;
          trap_mtval_w = '0;
        end
        TRAP_IADDR_MISALIGN: begin
          trap_we_w = 1'b1;
          trap_mtval_w = uop_w.trap_tval;
          redirect_w = 1'b1;
          redirect_target_w = mtvec_w & ~32'h3;
          trap_cause_w = 32'd0;
        end
        TRAP_LOAD_MISALIGN: begin
          trap_we_w = 1'b1;
          trap_mtval_w = uop_w.trap_tval;
          redirect_w = 1'b1;
          redirect_target_w = mtvec_w & ~32'h3;
          trap_cause_w = 32'd4;
        end
        TRAP_STORE_MISALIGN: begin
          trap_we_w = 1'b1;
          trap_mtval_w = uop_w.trap_tval;
          redirect_w = 1'b1;
          redirect_target_w = mtvec_w & ~32'h3;
          trap_cause_w = 32'd6;
        end
        default: begin
        end
      endcase
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      uop_w        <= uop_bubble();
      result_pre_w <= '0;
      mem_data_w   <= '0;
      dmem_addr_w  <= '0;
      dmem_wdata_w <= '0;
      dmem_be_w    <= 4'b0000;
    end else if (bubble_w) begin
      uop_w        <= uop_bubble();
      result_pre_w <= '0;
      mem_data_w   <= '0;
      dmem_addr_w  <= '0;
      dmem_wdata_w <= '0;
      dmem_be_w    <= 4'b0000;
    end else begin
      uop_w        <= uop_m_to_w;
      result_pre_w <= result_m;
      mem_data_w   <= mem_data_m;
      dmem_addr_w  <= dmem_addr;
      dmem_wdata_w <= dmem_wdata;
      dmem_be_w    <= dmem_be;
    end
  end

  assign redirect_any = redirect_w || redirect_e;
  assign redirect_target = redirect_w ? redirect_target_w : redirect_target_e;

  assign pc_plus4_w = uop_w.pc_plus4;
  assign pc_w       = uop_w.pc;
  assign rd_addr_w  = uop_w.rd_addr;
  assign ctrl_w     = uop_w.ctrl;
  assign instr_w    = uop_w.instr;

  rv32i_csr_file u_csr_file (
    .clk   (clk),
    .rst_n (rst_n),

    .we    (csr_we_w),
    .addr  (uop_w.csr_addr),
    .wdata (csr_wdata_w),
    .rdata (csr_rdata_w),

    // ---- trap ports: precise trap/mret state updates at W ----
    .trap_we    (trap_we_w),
    .trap_pc    (trap_pc_w),
    .trap_cause (trap_cause_w),
    .trap_tval  (trap_mtval_w),
    .mret_we    (mret_we_w),
    .mtvec      (mtvec_w),
    .mepc       (mepc_w)
  );

  // ============================================================
  // W : Writeback
  // ============================================================
  // Drives:
  //   result_w  : final writeback value into the regfile (mux on result_src)
  //   sim_halt  : raw W-stage halt request carried via ctrl through D->E->M->W.
  //               This raw control bit is not qualified by the valid-retire
  //               gate used for register-file and CSR side effects.

  always_comb begin
    csr_we_w    = 1'b0;
    csr_src_w   = uop_w.rs1_data;
    csr_wdata_w = csr_rdata_w;

    unique case (uop_w.ctrl.csr)
      CSR_RWI, CSR_RSI, CSR_RCI: csr_src_w = {27'b0, uop_w.csr_zimm};
      default: csr_src_w = uop_w.rs1_data;
    endcase

    if (uop_w.ctrl.valid && !uop_w.ctrl.illegal) begin
      unique case (uop_w.ctrl.csr)
        CSR_RW: begin
          csr_we_w    = 1'b1;
          csr_wdata_w = csr_src_w;
        end
        CSR_RS: begin
          csr_we_w    = (uop_w.rs1_addr != 5'd0);
          csr_wdata_w = csr_rdata_w | csr_src_w;
        end
        CSR_RC: begin
          csr_we_w    = (uop_w.rs1_addr != 5'd0);
          csr_wdata_w = csr_rdata_w & ~csr_src_w;
        end
        CSR_RWI: begin
          csr_we_w    = 1'b1;
          csr_wdata_w = csr_src_w;
        end
        CSR_RSI: begin
          csr_we_w    = (uop_w.csr_zimm != 5'd0);
          csr_wdata_w = csr_rdata_w | csr_src_w;
        end
        CSR_RCI: begin
          csr_we_w    = (uop_w.csr_zimm != 5'd0);
          csr_wdata_w = csr_rdata_w & ~csr_src_w;
        end
        default: begin
          csr_we_w    = 1'b0;
          csr_wdata_w = csr_rdata_w;
        end
      endcase
    end
  end

  always_comb begin
    result_w = result_pre_w;
    sim_halt = uop_w.ctrl.sim_halt;
    unique case (uop_w.ctrl.result_src)
      RES_ALU:    result_w = result_pre_w;
      RES_MEM:    result_w = mem_data_w;
      RES_PC4:    result_w = uop_w.pc_plus4;
      RES_MULDIV: result_w = result_pre_w;
      RES_CSR:    result_w = csr_rdata_w;
      default:    result_w = result_pre_w;
    endcase
  end

  // ============================================================
  // Hazard / forwarding unit
  // ============================================================

  rv32i_hazard u_hazard (
    .rs1_d       (rs1_addr_d),
    .rs2_d       (rs2_addr_d),
    .valid_d     (ctrl_d_gated.valid),
    .rs1_e       (uop_e.rs1_addr),
    .rs2_e       (uop_e.rs2_addr),
    .rd_m        (uop_m.rd_addr),
    .reg_write_m (uop_m.ctrl.reg_write),
    .rd_w        (uop_w.rd_addr),
    .reg_write_w (uop_w.ctrl.reg_write),
    .rd_e        (uop_e.rd_addr),
    .mem_read_e  (uop_e.ctrl.mem_read),
    .is_csr_m    (is_csr_m),
    .redirect_w  (redirect_w),
    .redirect_any (redirect_any),
    .is_muldiv_e (uop_e.ctrl.is_muldiv),
    .muldiv_done (muldiv_done),
    .forward_a_e (forward_a_e),
    .forward_b_e (forward_b_e),
    .stall_f     (stall_f),
    .stall_d     (stall_d),
    .stall_m     (stall_m),
    .stall_e     (stall_e),
    .flush_d     (flush_d),
    .flush_e     (flush_e),
    .bubble_m    (bubble_m),
    .bubble_w    (bubble_w),
    .dmem_wait_m (dmem_wait_m),
    .dmem_m_load_use_stall (dmem_m_load_use_stall),
    .csr_m_use_stall (csr_m_use_stall),
    .valid_e     (uop_e.ctrl.valid),
    .valid_m     (uop_m.ctrl.valid),
    .mem_read_m  (uop_m.ctrl.mem_read),
    .mem_write_m (uop_m.ctrl.mem_write),
    .dmem_done_m (dmem_done_m),
    .imem_wait_f (imem_wait_f)
  );

  `include "rtl_p/rv32i_pipeline_assert.svh"
endmodule
