// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// rv32i_ss_frontend - in-order fetch -> OoO decoded packet.
//
// request/response -> two-line fetch queue -> decode/formation -> OoO packet
//
// Decode lives in rv32i_ss_decode, a wrapper over the pipeline decoder.
// The split request/response boundary feeds two registered aligned 64-bit
// lines. Formation may pair words within a line or across contiguous lines.
// Consumed words retire from the queue only when the decoded bundle fires.
// Decode traps carry cause and trap value; CSR fields occupy slot 0 alone.

module rv32i_ss_frontend
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import fyp_cpu_pkg::imm_sel_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;
  import rv32i_pipeline_pkg::trap_op_e;

#(
  parameter word_t RESET_PC = 32'h0000_0000
)(
  input  logic clk,
  input  logic rst_n,

  // Instruction request/response channel permits one accepted
  // request awaiting a response. A zero-latency adapter may return the line
  // on the request-fire cycle; a delayed response uses the same CPU port.
  output logic  imem_req_valid,
  input  logic  imem_req_ready,
  output word_t imem_req_addr,
  input  logic  imem_resp_valid,
  output logic  imem_resp_ready,
  input  word_t [1:0] imem_resp_data,

  // redirect (branch recovery / jump / trap)
  input  logic  redirect_valid,
  input  word_t redirect_target,

  // predictor training from the core's branch resolution (execute-time
  // policy; wrong-path resolutions may train — predictor state is
  // performance-only, never architectural).
  input  logic  bp_update_valid,
  input  word_t bp_update_pc,
  input  logic  bp_update_taken,
  input  word_t bp_update_target,

  // return-site metadata and read-only live RAS top. The core remains
  // the sole RAS state owner; the frontend only samples this pair when a
  // non-killed instruction request fires.
  input  logic  bp_return_update_valid,
  input  word_t bp_return_update_pc,
  input  logic  ras_fetch_valid,
  input  word_t ras_fetch_target,

  // decoded packet -> core
  // real 2-wide formation — shape/payload built by the
  // classifier below; invalid slots carry zero payload by contract.
  output logic          decoded_valid,
  output logic [1:0]    decoded_slot_valid,
  input  logic          decoded_ready,
  output word_t         [1:0] decoded_pc,
  output word_t         [1:0] decoded_instr,
  output arch_reg_t     [1:0] decoded_rs1,
  output arch_reg_t     [1:0] decoded_rs2,
  output arch_reg_t     [1:0] decoded_rd,
  output logic          [1:0] decoded_rd_we,
  output logic          [1:0] decoded_needs_checkpoint,
  output ooo_op_class_e [1:0] decoded_op_class,
  output ooo_fu_class_e [1:0] decoded_fu_class,
  output muldiv_op_e    [1:0] decoded_muldiv_op,
  output alu_op_e       [1:0] decoded_alu_op,
  output br_type_e      [1:0] decoded_branch_op,
  output ooo_src_sel_e  [1:0] decoded_src1_sel,
  output ooo_src_sel_e  [1:0] decoded_src2_sel,
  output word_t         [1:0] decoded_imm,
  output decoded_trap_t decoded_trap,     // decode-detected trap
  output csr_op_e       decoded_csr_op,
  output csr_addr_t     decoded_csr_addr,
  output csr_zimm_t     decoded_csr_zimm,

  output logic      [1:0] decoded_is_load,
  output logic      [1:0] decoded_is_store,
  output mem_size_e [1:0] decoded_mem_size,
  output logic      [1:0] decoded_mem_unsigned,

  // Fetch-time prediction follows its instruction into the bundle. At most
  // one control-flow prediction is carried: the target is one word and at most
  // one decoded_pred_taken bit is set. Zero bits mean no taken prediction.
  output logic  [1:0] decoded_pred_taken,
  output word_t       decoded_pred_target
);

  localparam int FQ_DEPTH = 2;
  localparam logic [1:0] FQ_DEPTH_COUNT = 2'd2;

  // Queue entries are aligned 64-bit lines. fq_upper_q marks an entry whose
  // lower word is already consumed, or whose redirect target started at the
  // upper word. Only the valid head may carry that state.
  logic [FQ_DEPTH-1:0] fq_valid_q, fq_valid_d;
  logic [FQ_DEPTH-1:0] fq_upper_q, fq_upper_d;
  logic [1:0]          fq_count_q, fq_count_d;
  logic                fq_head_q, fq_head_d;
  logic                fq_tail_q, fq_tail_d;
  logic                upper_solo_locked_q;
  word_t               fq_base_q [FQ_DEPTH-1:0];
  word_t               fq_base_d [FQ_DEPTH-1:0];
  word_t [1:0]         fq_data_q [FQ_DEPTH-1:0];
  word_t [1:0]         fq_data_d [FQ_DEPTH-1:0];
  logic [FQ_DEPTH-1:0] fq_pred_valid_q, fq_pred_valid_d;
  logic [FQ_DEPTH-1:0] fq_pred_half_q, fq_pred_half_d;
  logic [FQ_DEPTH-1:0] fq_pred_return_q, fq_pred_return_d;
  word_t               fq_pred_target_q [FQ_DEPTH-1:0];
  word_t               fq_pred_target_d [FQ_DEPTH-1:0];

  // A request offer owns a queue slot before it can fire. Once accepted, the
  // same identity moves to the one-entry in-flight record until response.
  logic  req_offer_valid_q, req_offer_valid_d;
  logic  req_offer_killed_q, req_offer_killed_d;
  logic  req_offer_slot_q, req_offer_slot_d;
  logic  req_offer_upper_q, req_offer_upper_d;
  word_t req_offer_addr_q, req_offer_addr_d;
  logic  req_inflight_q, req_inflight_d;
  logic  req_inflight_killed_q, req_inflight_killed_d;
  logic  req_inflight_slot_q, req_inflight_slot_d;
  logic  req_inflight_upper_q, req_inflight_upper_d;
  word_t req_inflight_addr_q, req_inflight_addr_d;
  logic  req_inflight_pred_valid_q, req_inflight_pred_valid_d;
  logic  req_inflight_pred_half_q, req_inflight_pred_half_d;
  logic  req_inflight_pred_return_q, req_inflight_pred_return_d;
  word_t req_inflight_pred_target_q, req_inflight_pred_target_d;
  word_t next_req_addr_q, next_req_addr_d;
  logic  next_req_upper_q, next_req_upper_d;

  word_t       head_line_base;
  word_t [1:0] head_line_data;
  word_t       head_pc;
  word_t       fetch_instr;
  logic        follower_slot;
  logic        cross_line_supply;
  word_t       slot1_pc;
  word_t       slot1_instr;
  logic        bundle_fire;
  logic        consume_cross_line;
  logic        consume_cross_line_drop_follower;
  logic        consume_head_line;
  logic        consume_lower_only;
  logic        steer_repair_fire;
  word_t       steer_repair_target;
  logic        fetch_flush;
  word_t       fetch_flush_target;
  logic        imem_req_fire;
  logic        imem_resp_fire;
  logic        response_from_inflight;
  logic        response_settles_offer;
  logic        response_slot;
  logic        response_upper;
  logic        response_killed;
  word_t       response_base;
  logic        response_pred_valid;
  logic        response_pred_half;
  logic        response_pred_return;
  word_t       response_pred_target;
  logic        req_pred_valid;
  logic        req_pred_half;
  logic        req_pred_return;
  word_t       req_pred_target;
  logic        head_pred_at_slot0;
  logic        head_pred_at_slot1;
  logic        head_pred_type_match0;
  logic        head_pred_type_match1;
  logic        head_pred_target_mismatch0;
  logic        head_pred_target_mismatch1;
  logic        head_pred_site_consumed;
  logic        head_pred_false_type;
  logic        head_pred_target_mismatch;
  logic        slot1_pred_valid;
  logic        slot1_pred_half;
  logic        slot1_pred_return;
  word_t       slot1_pred_target;
  logic        pred_taken0, pred_taken1;
  word_t       pred_target0, pred_target1;
  logic [31:0] dec_imm;

  // ---- decoder outputs ----
  arch_reg_t     dec_rs1, dec_rs2, dec_rd;
  logic          dec_rd_we;
  ooo_op_class_e dec_op_class;
  ooo_fu_class_e dec_fu_class;
  alu_op_e       dec_alu_op;
  muldiv_op_e    dec_muldiv_op;
  br_type_e      dec_branch_op;
  ooo_src_sel_e  dec_src1_sel, dec_src2_sel;
  imm_sel_e      dec_imm_sel;
  trap_op_e      dec_trap_op;
  logic          dec_is_mret;
  logic          dec_is_csr;     // used by the unsupported-class guard
  logic          dec_is_mem;     // used by the unsupported-class guard
  // illegal flows via trap_op = TRAP_ILLEGAL; csr_op carries CSR operations.
  /* verilator lint_off UNUSEDSIGNAL */
  logic    dec_illegal;
  csr_op_e dec_csr_op;
  csr_addr_t dec_csr_addr;
  csr_zimm_t dec_csr_zimm;
  /* verilator lint_on UNUSEDSIGNAL */

  logic           dec_is_load;
  logic           dec_is_store;
  mem_size_e      dec_mem_size;
  logic           dec_mem_unsigned;

  // ---- request, queue, and per-instruction consumption ----
  assign head_line_base = fq_base_q[fq_head_q];
  assign head_line_data = fq_data_q[fq_head_q];
  assign head_pc         = head_line_base
                         + (fq_upper_q[fq_head_q] ? 32'd4 : 32'd0);
  assign fetch_instr     = fq_upper_q[fq_head_q]
                         ? head_line_data[1] : head_line_data[0];

  // the second candidate may come from the following registered
  // queue entry, but only when the head is its upper word and the follower is
  // the exact contiguous lower-word line. A steered/noncontiguous entry is
  // never reinterpreted as the sequential successor.
  assign follower_slot = ~fq_head_q;
  assign cross_line_supply = (fq_count_q == FQ_DEPTH_COUNT) && head_pc[2]
                           && fq_valid_q[follower_slot]
                           && !fq_upper_q[follower_slot]
                           && (fq_base_q[follower_slot]
                               == (head_line_base + 32'd8));
  assign slot1_pc    = head_pc + 32'd4;
  assign slot1_instr = cross_line_supply
                     ? fq_data_q[follower_slot][0] : head_line_data[1];

  assign bundle_fire       = decoded_valid && decoded_ready;
  assign consume_cross_line = bundle_fire
                            && (decoded_slot_valid == 2'b11)
                            && head_pc[2];
  assign consume_cross_line_drop_follower = consume_cross_line
                                           && pred_taken1;
  assign consume_head_line = bundle_fire
                           && ((decoded_slot_valid == 2'b11)
                               || fq_upper_q[fq_head_q]
                               || (head_pred_at_slot0
                                   && head_pred_type_match0));
  assign consume_lower_only = bundle_fire
                            && (decoded_slot_valid == 2'b01)
                            && !fq_upper_q[fq_head_q];

  // a correct request-time prediction preserves the already queued
  // target stream. Only a core redirect or accepted-dequeue repair flushes.
  // Core redirect wins when an aliased prediction lands on a JAL/return that
  // also redirects from the core on this same dispatch edge.
  assign fetch_flush        = redirect_valid || steer_repair_fire;
  assign fetch_flush_target = redirect_valid ? redirect_target
                                              : steer_repair_target;

  // A valid packet is producer-owned while the core holds ready low. If an
  // upper-word 01 offer begins before its follower is registered, lock that
  // one-wide shape until acceptance; a later response may not expand the
  // stalled packet into a cross-line 11 offer.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      upper_solo_locked_q <= 1'b0;
    else if (fetch_flush || bundle_fire)
      upper_solo_locked_q <= 1'b0;
    else if (decoded_valid && !decoded_ready && head_pc[2] &&
             !cross_line_supply)
      upper_solo_locked_q <= 1'b1;
  end

  assign imem_req_valid = req_offer_valid_q;
  assign imem_req_addr  = req_offer_addr_q;
  assign imem_req_fire  = imem_req_valid && imem_req_ready;

  // A response belongs to the accepted in-flight request or to a request
  // accepted on this same edge. A reservation can overlap a still-resident
  // slot; a live response waits until that slot is empty. Killed responses
  // drain unconditionally because they cannot fill queue storage.
  //
  // Use imem_req_valid in the same-edge readiness term to avoid a
  // ready -> request-fire -> response-ready combinational cycle. A legal
  // same-edge response still requires request acceptance; the identity checker
  // below rejects any response with neither an in-flight identity nor a fire.
  assign imem_resp_ready       = (req_inflight_q || imem_req_valid) &&
                                 (response_killed ||
                                  !fq_valid_q[response_slot]);
  assign imem_resp_fire        = imem_resp_valid && imem_resp_ready;
  assign response_from_inflight = req_inflight_q;
  // A same-cycle response can only be settling the presented offer itself
  // (the zero-latency shape). A response for the IN-FLIGHT record leaves a
  // same-cycle fire as a DISTINCT transaction — the pipelined handover —
  // which must load the in-flight record the drain just cleared.
  assign response_settles_offer = imem_resp_fire && !response_from_inflight;
  assign response_slot = response_from_inflight ? req_inflight_slot_q
                                                 : req_offer_slot_q;
  assign response_upper = response_from_inflight ? req_inflight_upper_q
                                                  : req_offer_upper_q;
  assign response_base = response_from_inflight ? req_inflight_addr_q
                                                 : req_offer_addr_q;
  assign response_killed = (response_from_inflight
                            ? req_inflight_killed_q
                            : req_offer_killed_q) || fetch_flush;
  assign response_pred_valid = response_from_inflight
                             ? req_inflight_pred_valid_q : req_pred_valid;
  assign response_pred_half = response_from_inflight
                            ? req_inflight_pred_half_q : req_pred_half;
  assign response_pred_return = response_from_inflight
                              ? req_inflight_pred_return_q : req_pred_return;
  assign response_pred_target = response_from_inflight
                              ? req_inflight_pred_target_q : req_pred_target;

  always_comb begin : p_fetch_next
    integer i;

    fq_valid_d              = fq_valid_q;
    fq_upper_d              = fq_upper_q;
    fq_count_d              = fq_count_q;
    fq_head_d               = fq_head_q;
    fq_tail_d               = fq_tail_q;
    req_offer_valid_d       = req_offer_valid_q;
    req_offer_killed_d      = req_offer_killed_q;
    req_offer_slot_d        = req_offer_slot_q;
    req_offer_upper_d       = req_offer_upper_q;
    req_offer_addr_d        = req_offer_addr_q;
    req_inflight_d          = req_inflight_q;
    req_inflight_killed_d   = req_inflight_killed_q;
    req_inflight_slot_d     = req_inflight_slot_q;
    req_inflight_upper_d    = req_inflight_upper_q;
    req_inflight_addr_d     = req_inflight_addr_q;
    req_inflight_pred_valid_d = req_inflight_pred_valid_q;
    req_inflight_pred_half_d = req_inflight_pred_half_q;
    req_inflight_pred_return_d = req_inflight_pred_return_q;
    req_inflight_pred_target_d = req_inflight_pred_target_q;
    next_req_addr_d         = next_req_addr_q;
    next_req_upper_d        = next_req_upper_q;
    for (i = 0; i < FQ_DEPTH; i = i + 1) begin
      fq_base_d[i]        = fq_base_q[i];
      fq_data_d[i]        = fq_data_q[i];
      fq_pred_valid_d[i]  = fq_pred_valid_q[i];
      fq_pred_half_d[i]   = fq_pred_half_q[i];
      fq_pred_return_d[i] = fq_pred_return_q[i];
      fq_pred_target_d[i] = fq_pred_target_q[i];
    end

    if (fetch_flush) begin
      // Queued fall-through lines are younger than the redirecting event.
      // An unaccepted offer stays stable until it fires, but is marked killed;
      // an accepted request likewise drains and discards its response.
      fq_valid_d      = '0;
      fq_upper_d      = '0;
      fq_pred_valid_d = '0;
      fq_pred_half_d = '0;
      fq_pred_return_d = '0;
      fq_count_d      = '0;
      fq_head_d       = 1'b0;
      fq_tail_d       = 1'b0;
      next_req_addr_d = {fetch_flush_target[31:3], 3'b000};
      next_req_upper_d = fetch_flush_target[2];
      for (i = 0; i < FQ_DEPTH; i = i + 1) begin
        fq_pred_target_d[i] = '0;
      end

      // an offer and an accepted transaction may BOTH be live at a
      // flush, and each dies on its own record. A drained in-flight response
      // clears its record; a fire this cycle is a DISTINCT transaction
      // (unless the response settled the offer itself — the zero-latency
      // shape) and enters the in-flight record killed; and any record still
      // occupied after those events is marked killed in place. A presented
      // offer is producer-owned and cannot be withdrawn, so it drains
      // through the environment exactly like the accepted request it will
      // become.
      if (imem_resp_fire && response_from_inflight) begin
        req_inflight_d        = 1'b0;
        req_inflight_killed_d = 1'b0;
        req_inflight_pred_valid_d = 1'b0;
        req_inflight_pred_half_d = 1'b0;
        req_inflight_pred_return_d = 1'b0;
        req_inflight_pred_target_d = '0;
      end
      if (imem_req_fire) begin
        req_offer_valid_d  = 1'b0;
        req_offer_killed_d = 1'b0;
        if (!response_settles_offer) begin
          req_inflight_d          = 1'b1;
          req_inflight_killed_d   = 1'b1;
          req_inflight_slot_d     = req_offer_slot_q;
          req_inflight_upper_d    = req_offer_upper_q;
          req_inflight_addr_d     = req_offer_addr_q;
          req_inflight_pred_valid_d = 1'b0;
          req_inflight_pred_half_d = 1'b0;
          req_inflight_pred_return_d = 1'b0;
          req_inflight_pred_target_d = '0;
        end
      end
      if (req_inflight_d)    req_inflight_killed_d = 1'b1;
      if (req_offer_valid_d) req_offer_killed_d    = 1'b1;

      // Once the offer register is free, register the redirect target immediately.
      // The new offer may coexist with a killed accepted request while it drains;
      // the environment decides when the new offer can be accepted. Both requests
      // retain their identities, and killed responses are consumed and discarded.
      // The target response still enters registered queue storage.
      if (!req_offer_valid_d) begin
        req_offer_valid_d  = 1'b1;
        req_offer_killed_d = 1'b0;
        req_offer_slot_d   = 1'b0;
        req_offer_upper_d  = fetch_flush_target[2];
        req_offer_addr_d   = {fetch_flush_target[31:3], 3'b000};
        fq_tail_d          = 1'b1;
        next_req_addr_d    = {fetch_flush_target[31:3], 3'b000} + 32'd8;
        next_req_upper_d   = 1'b0;
      end
    end else begin
      if (consume_cross_line) begin
        // The old upper-word head is always retired. Ordinarily the follower
        // remains as a partial line at its upper word. If slot 1 is correctly
        // predicted taken, that upper word is fall-through behind the taken
        // branch and both registered lines must be retired.
        fq_valid_d[fq_head_q] = 1'b0;
        fq_upper_d[fq_head_q] = 1'b0;
        fq_pred_valid_d[fq_head_q] = 1'b0;
        fq_pred_half_d[fq_head_q] = 1'b0;
        fq_pred_return_d[fq_head_q] = 1'b0;
        fq_pred_target_d[fq_head_q] = '0;
        if (consume_cross_line_drop_follower) begin
          fq_valid_d[follower_slot] = 1'b0;
          fq_upper_d[follower_slot] = 1'b0;
          fq_pred_valid_d[follower_slot] = 1'b0;
          fq_pred_half_d[follower_slot] = 1'b0;
          fq_pred_return_d[follower_slot] = 1'b0;
          fq_pred_target_d[follower_slot] = '0;
          fq_count_d = fq_count_d - 2'd2;
          fq_head_d  = fq_head_q;
        end else begin
          fq_upper_d[follower_slot] = 1'b1;
          fq_count_d = fq_count_d - 2'd1;
          fq_head_d  = follower_slot;
        end
      end else if (consume_head_line) begin
        fq_valid_d[fq_head_q] = 1'b0;
        fq_upper_d[fq_head_q] = 1'b0;
        fq_pred_valid_d[fq_head_q] = 1'b0;
        fq_pred_half_d[fq_head_q] = 1'b0;
        fq_pred_return_d[fq_head_q] = 1'b0;
        fq_pred_target_d[fq_head_q] = '0;
        fq_count_d            = fq_count_d - 2'd1;
        fq_head_d             = ~fq_head_q;
      end else if (consume_lower_only) begin
        fq_upper_d[fq_head_q] = 1'b1;
      end

      if (imem_req_fire) begin
        req_offer_valid_d  = 1'b0;
        req_offer_killed_d = 1'b0;
        // Steering is a property of the accepted request identity. A killed
        // offer drains but cannot redirect the successor stream.
        if (!req_offer_killed_q && req_pred_valid) begin
          next_req_addr_d  = {req_pred_target[31:3], 3'b000};
          next_req_upper_d = req_pred_target[2];
        end
      end

      if (imem_resp_fire) begin
        if (response_from_inflight) begin
          req_inflight_d        = 1'b0;
          req_inflight_killed_d = 1'b0;
          req_inflight_pred_valid_d = 1'b0;
          req_inflight_pred_half_d = 1'b0;
          req_inflight_pred_return_d = 1'b0;
          req_inflight_pred_target_d = '0;
        end
        if (!response_killed) begin
          fq_valid_d[response_slot] = 1'b1;
          fq_base_d[response_slot]  = response_base;
          fq_data_d[response_slot]  = imem_resp_data;
          fq_upper_d[response_slot] = response_upper;
          fq_pred_valid_d[response_slot] = response_pred_valid;
          fq_pred_half_d[response_slot] = response_pred_half;
          fq_pred_return_d[response_slot] = response_pred_return;
          fq_pred_target_d[response_slot] = response_pred_target;
          fq_count_d                = fq_count_d + 2'd1;
        end
      end
      // A fire coinciding with an in-flight response belongs to a distinct
      // transaction. This assignment follows the clear so the new identity wins.
      // A request answered on its own acceptance edge needs no in-flight record.
      if (imem_req_fire && !response_settles_offer) begin
        req_inflight_d        = 1'b1;
        req_inflight_killed_d = req_offer_killed_q;
        req_inflight_slot_d   = req_offer_slot_q;
        req_inflight_upper_d  = req_offer_upper_q;
        req_inflight_addr_d   = req_offer_addr_q;
        req_inflight_pred_valid_d = req_pred_valid;
        req_inflight_pred_half_d = req_pred_half;
        req_inflight_pred_return_d = req_pred_return;
        req_inflight_pred_target_d = req_pred_target;
      end

      // Refill the offer after a request fire, response or queue pop. The offer
      // may coexist with an accepted transaction. Creation stops at two resident
      // lines; a reservation that still overlaps a resident slot waits for slot
      // emptiness at response fill.
      if (!req_offer_valid_d &&
          (fq_count_d < FQ_DEPTH_COUNT)) begin
        req_offer_valid_d  = 1'b1;
        req_offer_killed_d = 1'b0;
        req_offer_slot_d   = fq_tail_d;
        req_offer_upper_d  = next_req_upper_d;
        req_offer_addr_d   = next_req_addr_d;
        fq_tail_d          = ~fq_tail_d;
        next_req_addr_d    = next_req_addr_d + 32'd8;
        next_req_upper_d   = 1'b0;
      end
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_fetch_state
    integer i;
    if (!rst_n) begin
      fq_valid_q            <= '0;
      fq_upper_q            <= '0;
      fq_count_q            <= '0;
      fq_head_q             <= 1'b0;
      fq_tail_q             <= 1'b0;
      req_offer_valid_q     <= 1'b0;
      req_offer_killed_q    <= 1'b0;
      req_offer_slot_q      <= 1'b0;
      req_offer_upper_q     <= 1'b0;
      req_offer_addr_q      <= '0;
      req_inflight_q        <= 1'b0;
      req_inflight_killed_q <= 1'b0;
      req_inflight_slot_q   <= 1'b0;
      req_inflight_upper_q  <= 1'b0;
      req_inflight_addr_q   <= '0;
      req_inflight_pred_valid_q <= 1'b0;
      req_inflight_pred_half_q <= 1'b0;
      req_inflight_pred_return_q <= 1'b0;
      req_inflight_pred_target_q <= '0;
      next_req_addr_q       <= {RESET_PC[31:3], 3'b000};
      next_req_upper_q      <= RESET_PC[2];
      for (i = 0; i < FQ_DEPTH; i = i + 1) begin
        fq_base_q[i] <= '0;
        fq_data_q[i] <= '0;
        fq_pred_valid_q[i] <= 1'b0;
        fq_pred_half_q[i] <= 1'b0;
        fq_pred_return_q[i] <= 1'b0;
        fq_pred_target_q[i] <= '0;
      end
    end else begin
      fq_valid_q            <= fq_valid_d;
      fq_upper_q            <= fq_upper_d;
      fq_count_q            <= fq_count_d;
      fq_head_q             <= fq_head_d;
      fq_tail_q             <= fq_tail_d;
      req_offer_valid_q     <= req_offer_valid_d;
      req_offer_killed_q    <= req_offer_killed_d;
      req_offer_slot_q      <= req_offer_slot_d;
      req_offer_upper_q     <= req_offer_upper_d;
      req_offer_addr_q      <= req_offer_addr_d;
      req_inflight_q        <= req_inflight_d;
      req_inflight_killed_q <= req_inflight_killed_d;
      req_inflight_slot_q   <= req_inflight_slot_d;
      req_inflight_upper_q  <= req_inflight_upper_d;
      req_inflight_addr_q   <= req_inflight_addr_d;
      req_inflight_pred_valid_q <= req_inflight_pred_valid_d;
      req_inflight_pred_half_q <= req_inflight_pred_half_d;
      req_inflight_pred_return_q <= req_inflight_pred_return_d;
      req_inflight_pred_target_q <= req_inflight_pred_target_d;
      next_req_addr_q       <= next_req_addr_d;
      next_req_upper_q      <= next_req_upper_d;
      for (i = 0; i < FQ_DEPTH; i = i + 1) begin
        fq_base_q[i] <= fq_base_d[i];
        fq_data_q[i] <= fq_data_d[i];
        fq_pred_valid_q[i] <= fq_pred_valid_d[i];
        fq_pred_half_q[i] <= fq_pred_half_d[i];
        fq_pred_return_q[i] <= fq_pred_return_d[i];
        fq_pred_target_q[i] <= fq_pred_target_d[i];
      end
    end
  end

  // ---- decode (wrapper over the pipeline decoder) ----
  rv32i_ss_decode u_dec (
    .instr     (fetch_instr),
    .rs1       (dec_rs1),
    .rs2       (dec_rs2),
    .rd        (dec_rd),
    .rd_we     (dec_rd_we),
    .illegal   (dec_illegal),
    .op_class  (dec_op_class),
    .fu_class  (dec_fu_class),
    .alu_op    (dec_alu_op),
    .muldiv_op (dec_muldiv_op),
    .branch_op (dec_branch_op),
    .src1_sel  (dec_src1_sel),
    .src2_sel  (dec_src2_sel),
    .imm_sel   (dec_imm_sel),
    .is_csr    (dec_is_csr),
    .csr_op    (dec_csr_op),
    .is_mem    (dec_is_mem),
    .trap_op   (dec_trap_op),
    .is_mret   (dec_is_mret),
    .csr_addr  (dec_csr_addr),
    .csr_zimm  (dec_csr_zimm),
    .is_load    (dec_is_load),
    .is_store   (dec_is_store),
    .mem_size   (dec_mem_size),
    .mem_unsigned (dec_mem_unsigned)
  );

  // Slot-1 decoding supplies formation-relevant fields. Trap, CSR and mret
  // details are unused here because those instructions occupy slot 0 alone.
  // The lint scope covers these intentionally unconsumed detail fields.
  /* verilator lint_off UNUSEDSIGNAL */
  arch_reg_t     dec1_rs1, dec1_rs2, dec1_rd;
  logic          dec1_rd_we;
  ooo_op_class_e dec1_op_class;
  ooo_fu_class_e dec1_fu_class;
  alu_op_e       dec1_alu_op;
  muldiv_op_e    dec1_muldiv_op;
  br_type_e      dec1_branch_op;
  ooo_src_sel_e  dec1_src1_sel, dec1_src2_sel;
  imm_sel_e      dec1_imm_sel;
  trap_op_e      dec1_trap_op;
  logic          dec1_is_mret, dec1_is_csr, dec1_is_mem, dec1_illegal;
  csr_op_e       dec1_csr_op;
  csr_addr_t     dec1_csr_addr;
  csr_zimm_t     dec1_csr_zimm;
  logic          dec1_is_load, dec1_is_store, dec1_mem_unsigned;
  mem_size_e     dec1_mem_size;
  logic [31:0]   dec1_imm;
  /* verilator lint_on UNUSEDSIGNAL */

  //Classifiers
  logic slot0_is_branch;
  logic slot1_is_branch;
  logic slot0_is_ret;
  logic slot1_is_ret;
  logic slot0_is_solo;
  logic slot1_is_solo;
  logic formation_word_pair_available;
  logic formation_dual_legal;

rv32i_ss_decode u_dec1 (
    .instr     (slot1_instr),
    .rs1       (dec1_rs1),
    .rs2       (dec1_rs2),
    .rd        (dec1_rd),
    .rd_we     (dec1_rd_we),
    .illegal   (dec1_illegal),
    .op_class  (dec1_op_class),
    .fu_class  (dec1_fu_class),
    .alu_op    (dec1_alu_op),
    .muldiv_op (dec1_muldiv_op),
    .branch_op (dec1_branch_op),
    .src1_sel  (dec1_src1_sel),
    .src2_sel  (dec1_src2_sel),
    .imm_sel   (dec1_imm_sel),
    .is_csr    (dec1_is_csr),
    .csr_op    (dec1_csr_op),
    .is_mem    (dec1_is_mem),
    .trap_op   (dec1_trap_op),
    .is_mret   (dec1_is_mret),
    .csr_addr  (dec1_csr_addr),
    .csr_zimm  (dec1_csr_zimm),
    .is_load    (dec1_is_load),
    .is_store   (dec1_is_store),
    .mem_size   (dec1_mem_size),
    .mem_unsigned (dec1_mem_unsigned)
  );

  assign slot0_is_branch = (dec_op_class == OOO_OP_BRANCH);
  assign slot1_is_branch = (dec1_op_class == OOO_OP_BRANCH);
  // Ret-form matches the core's prediction-consumption narrowing: JALR,
  // rs1 is a link register, the hint table says pop, and no real destination
  // is allocated. These are decode facts only; RAS state remains in the core.
  assign slot0_is_ret = (dec_op_class == OOO_OP_JUMP) &&
                        (dec_src1_sel == OOO_SRC_REG) &&
                        ((dec_rs1 == arch_reg_t'(1)) ||
                         (dec_rs1 == arch_reg_t'(5))) &&
                        (!((dec_rd == arch_reg_t'(1)) ||
                           (dec_rd == arch_reg_t'(5))) ||
                         (dec_rd != dec_rs1)) &&
                        !(dec_rd_we && (dec_rd != '0));
  assign slot1_is_ret = (dec1_op_class == OOO_OP_JUMP) &&
                        (dec1_src1_sel == OOO_SRC_REG) &&
                        ((dec1_rs1 == arch_reg_t'(1)) ||
                         (dec1_rs1 == arch_reg_t'(5))) &&
                        (!((dec1_rd == arch_reg_t'(1)) ||
                           (dec1_rd == arch_reg_t'(5))) ||
                         (dec1_rd != dec1_rs1)) &&
                        !(dec1_rd_we && (dec1_rd != '0));
  assign slot0_is_solo = (dec_op_class == OOO_OP_JUMP) || dec_is_csr
    || (dec_trap_op != rv32i_pipeline_pkg::TRAP_NONE);
  assign slot1_is_solo = (dec1_op_class == OOO_OP_JUMP) || dec1_is_csr
    || (dec1_trap_op != rv32i_pipeline_pkg::TRAP_NONE);

  // ---- request-time branch / learned-return prediction ----
  // The lookup PCs are the registered request offer's two word addresses.
  // Prediction is captured only on request fire and then follows that request
  // identity into the queue. Decode validates the stored kind/target later;
  // it does not perform a new live lookup.
  logic  bp_predict_taken_f0, bp_predict_taken_f1;
  logic  bp_predict_return_f0, bp_predict_return_f1;
  word_t bp_predict_target_f0, bp_predict_target_f1;
  logic  req_pred_candidate0, req_pred_candidate1;
  word_t exact_branch_target0, exact_branch_target1;

  rv32i_ss_bp u_bp (
    .clk               (clk),
    .rst_n             (rst_n),
    .pc_f0             (req_offer_addr_q),
    .predict_taken_f0  (bp_predict_taken_f0),
    .predict_return_f0 (bp_predict_return_f0),
    .predict_target_f0 (bp_predict_target_f0),
    .pc_f1             (req_offer_addr_q + 32'd4),
    .predict_taken_f1  (bp_predict_taken_f1),
    .predict_return_f1 (bp_predict_return_f1),
    .predict_target_f1 (bp_predict_target_f1),
    .update_valid_e    (bp_update_valid),
    .update_pc_e       (bp_update_pc),
    .update_taken_e    (bp_update_taken),
    .update_target_e   (bp_update_target),
    .return_update_valid (bp_return_update_valid),
    .return_update_pc    (bp_return_update_pc)
  );

  assign req_pred_candidate0 = bp_predict_taken_f0 ||
                               (bp_predict_return_f0 && ras_fetch_valid);
  assign req_pred_candidate1 = bp_predict_taken_f1 ||
                               (bp_predict_return_f1 && ras_fetch_valid);

  // Oldest word wins if both words hit. A redirect-starting upper-half offer
  // cannot predict from the skipped lower word.
  always_comb begin
    req_pred_valid  = 1'b0;
    req_pred_half   = 1'b0;
    req_pred_return = 1'b0;
    req_pred_target = '0;
    if (req_offer_valid_q && !req_offer_killed_q) begin
      if (!req_offer_upper_q && req_pred_candidate0) begin
        req_pred_valid  = 1'b1;
        req_pred_half   = 1'b0;
        req_pred_return = bp_predict_return_f0;
        req_pred_target = bp_predict_return_f0
                        ? ras_fetch_target : bp_predict_target_f0;
      end else if (req_pred_candidate1) begin
        req_pred_valid  = 1'b1;
        req_pred_half   = 1'b1;
        req_pred_return = bp_predict_return_f1;
        req_pred_target = bp_predict_return_f1
                        ? ras_fetch_target : bp_predict_target_f1;
      end
    end
  end

  assign exact_branch_target0 = head_pc + word_t'(dec_imm);
  assign exact_branch_target1 = slot1_pc + word_t'(dec1_imm);

  // Slot 1 normally shares the head entry. In a cross-line pair, every
  // prediction fact must instead follow the registered follower identity.
  assign slot1_pred_valid = cross_line_supply
                          ? fq_pred_valid_q[follower_slot]
                          : fq_pred_valid_q[fq_head_q];
  assign slot1_pred_half = cross_line_supply
                         ? fq_pred_half_q[follower_slot]
                         : fq_pred_half_q[fq_head_q];
  assign slot1_pred_return = cross_line_supply
                           ? fq_pred_return_q[follower_slot]
                           : fq_pred_return_q[fq_head_q];
  assign slot1_pred_target = cross_line_supply
                           ? fq_pred_target_q[follower_slot]
                           : fq_pred_target_q[fq_head_q];

  assign head_pred_at_slot0 = (fq_count_q != 0) &&
                              fq_pred_valid_q[fq_head_q] &&
                              (fq_pred_half_q[fq_head_q] == head_pc[2]);
  assign head_pred_at_slot1 = (fq_count_q != 0) && slot1_pred_valid &&
      ((!head_pc[2] && slot1_pred_half) ||
       (cross_line_supply && !slot1_pred_half));
  assign head_pred_type_match0 = fq_pred_return_q[fq_head_q]
                               ? slot0_is_ret : slot0_is_branch;
  assign head_pred_type_match1 = slot1_pred_return
                               ? slot1_is_ret : slot1_is_branch;
  assign head_pred_target_mismatch0 = head_pred_at_slot0 &&
                                      !fq_pred_return_q[fq_head_q] &&
                                      slot0_is_branch &&
                                      (fq_pred_target_q[fq_head_q] !=
                                       exact_branch_target0);
  assign head_pred_target_mismatch1 = head_pred_at_slot1 &&
                                      !slot1_pred_return &&
                                      slot1_is_branch &&
                                      (slot1_pred_target !=
                                       exact_branch_target1);

  // A branch target is decode-exact by the time the bundle is formed; a RAS
  // target is not exact until execute and therefore keeps its captured value.
  assign pred_taken0 = head_pred_at_slot0 && head_pred_type_match0;
  assign pred_target0 = fq_pred_return_q[fq_head_q]
                      ? fq_pred_target_q[fq_head_q] : exact_branch_target0;
  assign pred_taken1 = head_pred_at_slot1 && head_pred_type_match1;
  assign pred_target1 = slot1_pred_return
                      ? slot1_pred_target : exact_branch_target1;

  assign formation_word_pair_available = !head_pc[2] ||
      (cross_line_supply && !upper_solo_locked_q);
  assign formation_dual_legal = (fq_count_q != 0)
    && formation_word_pair_available
    && !slot0_is_solo && !slot1_is_solo
    && !(slot0_is_branch && slot1_is_branch) && !(dec_is_mem && dec1_is_mem)
    && !pred_taken0;

  assign head_pred_site_consumed =
      (head_pred_at_slot0 && decoded_slot_valid[0]) ||
      (head_pred_at_slot1 && decoded_slot_valid[1]);
  assign head_pred_false_type = head_pred_site_consumed &&
      ((head_pred_at_slot0 && !head_pred_type_match0) ||
       (head_pred_at_slot1 && !head_pred_type_match1));
  assign head_pred_target_mismatch = head_pred_site_consumed &&
      (head_pred_target_mismatch0 || head_pred_target_mismatch1);
  assign steer_repair_fire = bundle_fire &&
      (head_pred_false_type || head_pred_target_mismatch);
  assign steer_repair_target = head_pred_target_mismatch
      ? (head_pred_target_mismatch0
         ? exact_branch_target0 : exact_branch_target1)
      : (head_pc + ((decoded_slot_valid == 2'b11) ? 32'd8 : 32'd4));


  rv32i_imm_gen u_imm (
    .instr   (fetch_instr),
    .imm_sel (dec_imm_sel),
    .imm     (dec_imm)
  );

  rv32i_imm_gen u_imm1 (
    .instr   (slot1_instr),
    .imm_sel (dec1_imm_sel),
    .imm     (dec1_imm)
  );

  // ---- formation + packet wiring ----
  // Start from an empty bundle, always present the current PC in slot 0, and
  // add the next word in slot 1 only when the bundle legality rules allow shape 2'b11.
  assign decoded_valid = |decoded_slot_valid;

  always_comb begin
    decoded_slot_valid       = '0;
    decoded_pc               = '0;
    decoded_instr            = '0;
    decoded_rs1              = '0;
    decoded_rs2              = '0;
    decoded_rd               = '0;
    decoded_rd_we            = '0;
    decoded_needs_checkpoint = '0;
    decoded_op_class         = '0;
    decoded_fu_class         = '0;
    decoded_muldiv_op        = '0;
    decoded_alu_op           = '0;
    decoded_branch_op        = '0;
    decoded_src1_sel         = '0;
    decoded_src2_sel         = '0;
    decoded_imm              = '0;
    decoded_is_load          = '0;
    decoded_is_store         = '0;
    decoded_mem_size         = '0;
    decoded_mem_unsigned     = '0;
    decoded_pred_taken       = '0;
    decoded_pred_target      = '0;

    if (fq_count_q != 0) begin
      decoded_slot_valid[0]       = 1'b1;
      decoded_pc[0]               = head_pc;
      decoded_instr[0]            = fetch_instr;
      decoded_rs1[0]              = dec_rs1;
      decoded_rs2[0]              = dec_rs2;
      decoded_rd[0]               = dec_rd;
      decoded_rd_we[0]            = dec_rd_we;
      decoded_needs_checkpoint[0] = slot0_is_branch;
      decoded_op_class[0]         = dec_op_class;
      decoded_fu_class[0]         = dec_fu_class;
      decoded_muldiv_op[0]        = dec_muldiv_op;
      decoded_alu_op[0]           = dec_alu_op;
      decoded_branch_op[0]        = dec_branch_op;
      decoded_src1_sel[0]         = dec_src1_sel;
      decoded_src2_sel[0]         = dec_src2_sel;
      decoded_imm[0]              = word_t'(dec_imm);
      decoded_is_load[0]          = dec_is_load;
      decoded_is_store[0]         = dec_is_store;
      decoded_mem_size[0]         = dec_mem_size;
      decoded_mem_unsigned[0]     = dec_mem_unsigned;
      decoded_pred_taken[0]       = pred_taken0;
      if (pred_taken0) begin
        decoded_pred_target = pred_target0;
      end

      if (formation_dual_legal) begin
        decoded_slot_valid[1]       = 1'b1;
        decoded_pc[1]               = slot1_pc;
        decoded_instr[1]            = slot1_instr;
        decoded_rs1[1]              = dec1_rs1;
        decoded_rs2[1]              = dec1_rs2;
        decoded_rd[1]               = dec1_rd;
        decoded_rd_we[1]            = dec1_rd_we;
        decoded_needs_checkpoint[1] = slot1_is_branch;
        decoded_op_class[1]         = dec1_op_class;
        decoded_fu_class[1]         = dec1_fu_class;
        decoded_muldiv_op[1]        = dec1_muldiv_op;
        decoded_alu_op[1]           = dec1_alu_op;
        decoded_branch_op[1]        = dec1_branch_op;
        decoded_src1_sel[1]         = dec1_src1_sel;
        decoded_src2_sel[1]         = dec1_src2_sel;
        decoded_imm[1]              = word_t'(dec1_imm);
        decoded_is_load[1]          = dec1_is_load;
        decoded_is_store[1]         = dec1_is_store;
        decoded_mem_size[1]         = dec1_mem_size;
        decoded_mem_unsigned[1]     = dec1_mem_unsigned;
        decoded_pred_taken[1]       = pred_taken1;
        if (pred_taken1) begin
          decoded_pred_target = pred_target1;
        end
      end
    end
  end

  assign decoded_csr_op           = dec_csr_op;
  assign decoded_csr_addr         = dec_csr_addr;
  assign decoded_csr_zimm         = dec_csr_zimm;

  // ---- decode-detected trap -> direct-cause form (architectural trap-value convention) ----
  always_comb begin
    decoded_trap         = '0;
    decoded_trap.is_mret = dec_is_mret;
    unique case (dec_trap_op)
      rv32i_pipeline_pkg::TRAP_ILLEGAL: begin
        decoded_trap.valid = 1'b1; decoded_trap.cause = OOO_CAUSE_ILLEGAL;  decoded_trap.tval = fetch_instr;
      end
      rv32i_pipeline_pkg::TRAP_ECALL: begin
        decoded_trap.valid = 1'b1; decoded_trap.cause = OOO_CAUSE_ECALL_M;
      end
      rv32i_pipeline_pkg::TRAP_EBREAK: begin
        decoded_trap.valid = 1'b1; decoded_trap.cause = OOO_CAUSE_EBREAK;  decoded_trap.tval = head_pc;
      end
      default: ;  // TRAP_NONE / TRAP_MRET (via is_mret) / misalign
    endcase
  end


`ifndef SYNTHESIS
  /* verilator lint_off SYNCASYNCNET */
  logic  req_stalled_q;
  word_t req_stalled_addr_q;
  logic        decoded_stalled_q;
  logic [1:0]  decoded_stalled_shape_q;
  word_t [1:0] decoded_stalled_pc_q;
  word_t [1:0] decoded_stalled_instr_q;
  logic [1:0]  decoded_stalled_pred_taken_q;
  word_t       decoded_stalled_pred_target_q;

  // event counters are observation only. They deliberately count
  // accepted identities/events rather than combinational offers so the
  // steering ledger reconciles against request and bundle fires.
  integer d015_req_branch_lower_count;
  integer d015_req_branch_upper_count;
  integer d015_req_return_lower_count;
  integer d015_req_return_upper_count;
  integer d015_false_type_count;
  integer d015_target_repair_count;
  integer d015_core_override_count;
  integer q1_cross_line_count;
  integer q1_cross_line_drop_follower_count;
  logic   q1_retain_check_q;
  word_t  q1_retain_base_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      d015_req_branch_lower_count <= 0;
      d015_req_branch_upper_count <= 0;
      d015_req_return_lower_count <= 0;
      d015_req_return_upper_count <= 0;
      d015_false_type_count <= 0;
      d015_target_repair_count <= 0;
      d015_core_override_count <= 0;
      q1_cross_line_count <= 0;
      q1_cross_line_drop_follower_count <= 0;
      q1_retain_check_q <= 1'b0;
      q1_retain_base_q <= '0;
    end else begin
      if (imem_req_fire && !req_offer_killed_q && req_pred_valid) begin
        if (req_pred_return && !req_pred_half)
          d015_req_return_lower_count <= d015_req_return_lower_count + 1;
        else if (req_pred_return && req_pred_half)
          d015_req_return_upper_count <= d015_req_return_upper_count + 1;
        else if (!req_pred_half)
          d015_req_branch_lower_count <= d015_req_branch_lower_count + 1;
        else
          d015_req_branch_upper_count <= d015_req_branch_upper_count + 1;
      end
      if (bundle_fire && head_pred_false_type)
        d015_false_type_count <= d015_false_type_count + 1;
      if (bundle_fire && head_pred_target_mismatch)
        d015_target_repair_count <= d015_target_repair_count + 1;
      if (bundle_fire && redirect_valid &&
          (head_pred_false_type || head_pred_target_mismatch))
        d015_core_override_count <= d015_core_override_count + 1;
      if (consume_cross_line)
        q1_cross_line_count <= q1_cross_line_count + 1;
      if (consume_cross_line_drop_follower)
        q1_cross_line_drop_follower_count <=
            q1_cross_line_drop_follower_count + 1;

      // Ordinary cross-line consumption makes the follower's upper word the
      // next registered head. Check that state transition independently of
      // the combinational instruction-selection mux.
      if (q1_retain_check_q &&
          ((fq_count_q == 0) || !fq_valid_q[fq_head_q] ||
           !fq_upper_q[fq_head_q] ||
           (fq_base_q[fq_head_q] != q1_retain_base_q))) begin
        $fatal(1, "rv32i_ss_frontend: cross-line follower upper word was not retained");
      end
      q1_retain_check_q <= consume_cross_line && !fetch_flush &&
                           !consume_cross_line_drop_follower;
      if (consume_cross_line)
        q1_retain_base_q <= fq_base_q[follower_slot];
    end
  end

  // The request producer owns valid/address until acceptance. This checker is
  // intentionally sampled from the public handshake, not from offer_d.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      req_stalled_q      <= 1'b0;
      req_stalled_addr_q <= '0;
    end else begin
      if (req_stalled_q &&
          (!imem_req_valid || (imem_req_addr != req_stalled_addr_q))) begin
        $fatal(1, "rv32i_ss_frontend: stalled imem request changed before acceptance");
      end
      req_stalled_q      <= imem_req_valid && !imem_req_ready;
      req_stalled_addr_q <= imem_req_addr;
    end
  end

  // The decoded valid/ready channel obeys the same producer-ownership rule as
  // the memory request channel. A stalled upper-word 01 offer must retain
  // its shape when the follower response arrives later.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      decoded_stalled_q <= 1'b0;
      decoded_stalled_shape_q <= '0;
      decoded_stalled_pc_q <= '0;
      decoded_stalled_instr_q <= '0;
      decoded_stalled_pred_taken_q <= '0;
      decoded_stalled_pred_target_q <= '0;
    end else begin
      if (decoded_stalled_q && !fetch_flush &&
          (!decoded_valid ||
           (decoded_slot_valid !== decoded_stalled_shape_q) ||
           (decoded_pc !== decoded_stalled_pc_q) ||
           (decoded_instr !== decoded_stalled_instr_q) ||
           (decoded_pred_taken !== decoded_stalled_pred_taken_q) ||
           (decoded_pred_target !== decoded_stalled_pred_target_q))) begin
        $fatal(1, "rv32i_ss_frontend: stalled decoded packet changed before acceptance");
      end
      decoded_stalled_q <= decoded_valid && !decoded_ready && !fetch_flush;
      decoded_stalled_shape_q <= decoded_slot_valid;
      decoded_stalled_pc_q <= decoded_pc;
      decoded_stalled_instr_q <= decoded_instr;
      decoded_stalled_pred_taken_q <= decoded_pred_taken;
      decoded_stalled_pred_target_q <= decoded_pred_target;
    end
  end

  // Formation and queue invariants. The core independently checks solo,
  // memory-operation and branch-count restrictions at the consuming boundary.
  always @(posedge clk) begin
    if (rst_n === 1'b1) begin
      if (fq_count_q > FQ_DEPTH_COUNT) begin
        $fatal(1, "rv32i_ss_frontend: fetch queue count exceeded depth");
      end
      if (fq_count_q != ({1'b0, fq_valid_q[0]} +
                         {1'b0, fq_valid_q[1]})) begin
        $fatal(1, "rv32i_ss_frontend: fetch queue count/valid mismatch");
      end
      if ((fq_count_q != 0) && !fq_valid_q[fq_head_q]) begin
        $fatal(1, "rv32i_ss_frontend: fetch queue head is not valid");
      end
      if ((fq_count_q == 0) && (|fq_upper_q)) begin
        $fatal(1, "rv32i_ss_frontend: empty fetch queue retained partial head state");
      end
      // At most one offered request and one accepted request coexist with
      // resident lines. Their total can reach three transiently: offer creation
      // stops at two residents, and filling retires the in-flight record.
      // Slot emptiness is required at fill, not at reservation.
      if (({1'b0, fq_count_q} + {2'b0, req_offer_valid_q} +
           {2'b0, req_inflight_q}) > 3'd3) begin
        $fatal(1, "rv32i_ss_frontend: fetch reservation window exceeded");
      end
      if (imem_req_valid && (imem_req_addr[2:0] != 3'b000)) begin
        $fatal(1, "rv32i_ss_frontend: imem request address is not line aligned");
      end
      if (imem_resp_valid && !req_inflight_q && !imem_req_fire) begin
        $fatal(1, "rv32i_ss_frontend: imem response has no accepted request identity");
      end
      // The environment may accept over an in-flight transaction only if its
      // response retires on this edge. Otherwise the single in-flight record
      // would be overwritten and the older request's identity lost.
      if (imem_req_fire && req_inflight_q && !imem_resp_fire) begin
        $fatal(1, "rv32i_ss_frontend: request accepted over a non-retiring in-flight transaction");
      end
      if (imem_resp_fire && !response_killed && fq_valid_q[response_slot]) begin
        $fatal(1, "rv32i_ss_frontend: live imem response overwrites a queued line");
      end
      if ((fq_count_q == 0) && decoded_valid) begin
        $fatal(1, "rv32i_ss_frontend: empty fetch queue offered a decoded packet");
      end
      if (|(fq_pred_valid_q & ~fq_valid_q)) begin
        $fatal(1, "rv32i_ss_frontend: prediction metadata survived without its queue line");
      end
      if (decoded_slot_valid[1] &&
          (slot0_is_solo || slot1_is_solo ||
           (slot0_is_branch && slot1_is_branch) ||
           (dec_is_mem && dec1_is_mem) ||
           !formation_word_pair_available ||
           pred_taken0)) begin
        $fatal(1, "rv32i_ss_frontend: formation offered an illegal bundle");
      end
      if (decoded_valid != (|decoded_slot_valid)) begin
        $fatal(1, "rv32i_ss_frontend: offer level disagrees with shape");
      end

      if (!decoded_slot_valid[1] &&
          ((|decoded_instr[1]) || (|decoded_rd_we[1]) ||
           (|decoded_needs_checkpoint[1]) || (|decoded_is_load[1]) ||
           (|decoded_is_store[1]) || (|decoded_pc[1]) ||
           decoded_pred_taken[1])) begin
        $fatal(1, "rv32i_ss_frontend: invalid slot carries nonzero payload");
      end

      // An aliased prediction kind is a repairable fetch-steering error. It
      // must be repaired before dispatch and cannot survive as packet prediction.
      if ((decoded_pred_taken[0] && !(slot0_is_branch || slot0_is_ret)) ||
          (decoded_pred_taken[1] && !(slot1_is_branch || slot1_is_ret))) begin
        $fatal(1, "rv32i_ss_frontend: prediction escaped type validation");
      end
      if (bundle_fire && head_pred_false_type && !steer_repair_fire) begin
        $fatal(1, "rv32i_ss_frontend: false-type steer dispatched without repair");
      end
      if (bundle_fire && head_pred_target_mismatch && !steer_repair_fire) begin
        $fatal(1, "rv32i_ss_frontend: stale-target steer dispatched without repair");
      end
      if (bundle_fire && head_pred_target_mismatch &&
          (decoded_pred_target != steer_repair_target)) begin
        $fatal(1, "rv32i_ss_frontend: stale BTB target was not decode-corrected");
      end
      if (redirect_valid && (!fetch_flush ||
          (fetch_flush_target != redirect_target))) begin
        $fatal(1, "rv32i_ss_frontend: core redirect lost fetch-flush priority");
      end
      if (decoded_pred_taken == 2'b11) begin
        $fatal(1, "rv32i_ss_frontend: both slots predicted taken");
      end
    end
  end
  /* verilator lint_on SYNCASYNCNET */
`endif

endmodule
