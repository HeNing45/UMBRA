// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// rv32i_ss_muldiv — multi-cycle multiplier/divider for RV32M.
//
// External contract:
// start : accepted in S_IDLE, or while S_DONE drains.
// busy : arithmetic engine occupied (S_MUL/S_DIV only).
// complete.valid : result held throughout S_DONE until complete_ready.
//
// FSM:
//
// S_IDLE ── start && op_is_mul ──▶ S_MUL ─┐
// S_IDLE ── start && !op_is_mul ──▶ S_DIV ─┴── cnt_q==0 ──▶ S_DONE ──▶ S_IDLE
//
// Datapaths:
// The multiplier registers two partial products, then combines them on the
// second arithmetic cycle. Start captures operands extended to 33 bits with
// per-operation signedness. Split B at bit 16: its low half is zero-extended,
// and its high half carries the sign. The 33x17 products combine as
// low + (high << 16) modulo 2**64; result selection implements the RV32M word.
// Splitting the arithmetic distributes work across the two registered cycles.
// The divider performs 32 restoring iterations on operand magnitudes, with
// divide-by-zero and signed-overflow fast paths at start.

module rv32i_ss_muldiv
  import fyp_cpu_pkg::*;
  import rv32i_pipeline_pkg::*;
  import rv32i_ss_pkg::rob_idx_t;
  import rv32i_ss_pkg::rob_seq_t;
  import rv32i_ss_pkg::branch_mask_t;
  import rv32i_ss_pkg::phys_reg_t;
  import rv32i_ss_pkg::completion_packet_t;
(
  input  logic       clk,
  input  logic       rst_n,
  input  logic       start,
  input  muldiv_op_e op,
  input  word_t      a,
  input  word_t      b,
  input  logic       kill,
  input rob_idx_t   rob_idx,
  input  rob_seq_t  rob_seq,
  input branch_mask_t branch_mask,
  input phys_reg_t   pdst,
  input logic        rd_wen,
  input logic        complete_ready,
  output logic       busy,
  output completion_packet_t  complete
);

  // ==================================================================
  // FSM state (shared)
  // ==================================================================
  typedef enum logic [1:0] {
    S_IDLE,
    S_MUL,
    S_DIV,
    S_DONE
  } state_e;

  state_e     state_q,  state_d;
  logic [5:0] cnt_q,    cnt_d;     // counts 32 → 0, one per iteration
  muldiv_op_e op_q;                 // latched at IDLE→active edge; used to slice
  word_t      result_q, result_d;
  rob_idx_t  rob_idx_q;
  rob_seq_t  rob_seq_q;
  // Captured metadata; kill is supplied by the core's ROB-age comparison,
  // so this block does not consume its saved checkpoint mask.
  branch_mask_t branch_mask_q;
  phys_reg_t pdst_q;
  logic      rd_wen_q;

  // ==================================================================
  // MUL datapath state
  // mult_a_q/mult_b_q : operands latched once at the accepted start,
  // sign-extended to 33 bits per the op's signedness
  // (MULHU: both unsigned; MULHSU: A signed only).
  // pp_lo_q/pp_hi_q : the two 33x17 partial products, registered on
  // the first S_MUL cycle. pp_lo = A * B[15:0]
  // (low half as a positive 17-bit value);
  // pp_hi = A * B[32:16] (signed -- carries B's
  // sign). |pp| < 2^49, so 50 bits hold both.
  // ==================================================================
  logic [32:0] mult_a_q, mult_b_q;
  logic [49:0] pp_lo_q, pp_lo_d;
  logic [49:0] pp_hi_q, pp_hi_d;

  // ==================================================================
  // DIV datapath state
  // remainder_q : 33 bits — extra MSB for trial-subtract borrow.
  // quotient_q : 32 bits, built bit-by-bit at the LSB.
  // divisor_q : |B|, latched once at IDLE→S_DIV.
  // neg_quot_q : neg_a XOR neg_b (post-correct quotient).
  // neg_rem_q : neg_a (post-correct remainder).
  // ==================================================================
  logic [32:0] remainder_q, remainder_d;
  logic [31:0] quotient_q,  quotient_d;
  logic [31:0] divisor_q;
  logic        neg_quot_q, neg_rem_q;

  // ==================================================================
  // Live combinational helpers — computed from inputs every cycle.
  //
  // op_is_mul : 1 if op ∈ {MUL, MULH, MULHSU, MULHU}.
  // mult_ext_* : the operands sign-extended to 33 bits per the MUL
  // family's signedness (the pipelined multiply carries
  // the signs; no magnitude conversion).
  // neg_a/_b : DIV family only — 1 if that operand is negated to
  // its magnitude for the restoring divider.
  // ua / ub : the magnitudes |A| and |B| (DIV family only).
  //
  // These are always live, regardless of state. The IDLE→active
  // transition just samples them on that cycle.
  //
  // Signedness decode:
  // MUL / MULH : both operands signed
  // MULHSU : only A signed (B is unsigned)
  // MULHU : both unsigned
  // DIV / REM : both operands signed (magnitudes + sign fixup)
  // DIVU / REMU : both unsigned → neg_a = neg_b = 0
  // ==================================================================
  logic  op_is_mul;
  logic  neg_a, neg_b;
  word_t ua, ub;
  logic [32:0] mult_ext_a, mult_ext_b;
  logic [32:0] rem_shift;
  logic [31:0] quot_shift;
  logic [32:0] rem_trial;
  logic [63:0] mult_sum;
  logic        start_accept;

  assign op_is_mul = (op == MD_MUL)    || (op == MD_MULH)
                  || (op == MD_MULHSU) || (op == MD_MULHU);

  assign mult_ext_a = {(op == MD_MULHU) ? 1'b0 : a[31], a};
  assign mult_ext_b = {((op == MD_MUL) || (op == MD_MULH)) ? b[31] : 1'b0, b};

  always_comb begin
    unique case (op)
      MD_DIV, MD_REM:  begin neg_a = a[31]; neg_b = b[31]; end
      default:          begin neg_a = 1'b0;  neg_b = 1'b0;  end
    endcase
  end

  assign ua = neg_a ? -a : a;
  assign ub = neg_b ? -b : b;
  assign start_accept = start &&
                        ((state_q == S_IDLE) ||
                         ((state_q == S_DONE) && complete_ready));

  // ==================================================================
  // Next-state / next-data
  // ==================================================================
  always_comb begin
    // hold defaults — every _d must have a defined assignment to avoid
    // inferred latches on the MUL/DIV datapath shadows.
    state_d     = state_q;
    cnt_d       = cnt_q;
    result_d    = result_q;
    pp_lo_d     = pp_lo_q;
    pp_hi_d     = pp_hi_q;
    remainder_d = remainder_q;
    quotient_d  = quotient_q;
    rem_shift   = '0;
    quot_shift  = '0;
    rem_trial   = '0;
    mult_sum    = '0;

    unique case (state_q)

      // ---------- S_IDLE: wait for an accepted start ---------------
      S_IDLE: begin
      end

      // ---------- S_MUL: 2-stage pipelined multiply ----------------
      // cnt==2 : the two 33x17 partial products register (each about
      // half the flat 33x33 array's depth)
      // cnt==1 : 64-bit recombination -- pp_lo + (pp_hi << 16), exact
      // mod 2^64, and the true product fits 64 bits for all
      // four MUL-family sign combinations -- then the
      // architectural word select -> S_DONE
      S_MUL: begin
        if (cnt_q == 6'd2) begin
          cnt_d   = 6'd1;
          pp_lo_d = 50'($signed(mult_a_q) * $signed({1'b0, mult_b_q[15:0]}));
          pp_hi_d = 50'($signed(mult_a_q) * $signed(mult_b_q[32:16]));
        end else begin
          state_d  = S_DONE;
          mult_sum = 64'($signed(pp_lo_q)) + (64'($signed(pp_hi_q)) << 16);
          unique case (op_q)
            MD_MUL : result_d = mult_sum[31:0];
            MD_MULH, MD_MULHSU, MD_MULHU : result_d = mult_sum[63:32];
            default: result_d = '0;
          endcase
        end
      end

      // ---------- S_DIV: 32-cycle restoring divide -----------------
      S_DIV: begin
        if (cnt_q == 6'd0) begin
          state_d = S_DONE;
        end else begin
          cnt_d = cnt_q - 6'd1;

          rem_shift = {remainder_q[31:0], quotient_q[31]};
          quot_shift = {quotient_q[30:0], 1'b0};
          rem_trial = rem_shift - {1'b0, divisor_q};

          if (rem_trial[32] == 1'b0) begin
            remainder_d = rem_trial;
            quotient_d = quot_shift | 32'h1;
          end else begin
            remainder_d = rem_shift;
            quotient_d = quot_shift;
          end

          if (cnt_q == 6'd1) begin
            unique case (op_q)
              MD_DIV : result_d = neg_quot_q ? -quotient_d : quotient_d;
              MD_DIVU: result_d = quotient_d;
              MD_REM : result_d = neg_rem_q ? -remainder_d[31:0]
                                            :  remainder_d[31:0];
              MD_REMU: result_d = remainder_d[31:0];
              default: result_d = '0;
            endcase
          end
        end
      end

      // ---------- S_DONE: hold, or drain and optionally refill ------
      S_DONE: begin
        if (complete_ready) begin
          state_d = S_IDLE;
        end
      end

      default: begin
        state_d = S_IDLE;
        cnt_d   = '0;
      end
    endcase

    // The same initialization serves a normal IDLE start and a replacement
    // accepted on the cycle S_DONE drains. This block intentionally follows
    // the state case so a refill overrides S_DONE's default return to IDLE.
    if (start_accept) begin
      cnt_d = 6'd32;

      if (op_is_mul) begin
        state_d = S_MUL;
        cnt_d   = 6'd2;
      end else if (b == 32'h0000_0000) begin
        state_d = S_DONE;
        cnt_d   = '0;

        unique case (op)
          MD_DIV, MD_DIVU : result_d = 32'hFFFF_FFFF;
          MD_REM, MD_REMU : result_d = a;
          default         : result_d = '0;
        endcase
      end else if (((op == MD_DIV) || (op == MD_REM)) &&
                   (a == 32'h8000_0000) &&
                   (b == 32'hFFFF_FFFF)) begin
        state_d = S_DONE;
        cnt_d   = '0;

        unique case (op)
          MD_DIV : result_d = 32'h8000_0000;
          MD_REM : result_d = 32'h0000_0000;
          default: result_d = '0;
        endcase
      end else begin
        state_d     = S_DIV;
        remainder_d = '0;
        quotient_d  = ua;
      end
    end
  end

  // ==================================================================
  // Sequential state
  // ==================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state_q        <= S_IDLE;
      cnt_q          <= '0;
      result_q       <= '0;
      op_q           <= MD_NONE;
      // MUL datapath
      rob_idx_q <= '0;
      rob_seq_q <= '0;
      branch_mask_q <= '0;
      pdst_q <= '0;
      rd_wen_q <= '0;
      pp_lo_q        <= '0;
      pp_hi_q        <= '0;
      mult_a_q       <= '0;
      mult_b_q       <= '0;
      // DIV datapath
      remainder_q    <= '0;
      quotient_q     <= '0;
      divisor_q      <= '0;
      neg_quot_q     <= 1'b0;
      neg_rem_q      <= 1'b0;
    end else if (kill && (state_q != S_DONE)) begin
      state_q        <= S_IDLE;
      cnt_q          <= '0;
      result_q       <= '0;
      op_q           <= MD_NONE;
      // MUL datapath
      rob_idx_q <= '0;
      rob_seq_q <= '0;
      branch_mask_q <= '0;
      pdst_q <= '0;
      rd_wen_q <= '0;
      pp_lo_q        <= '0;
      pp_hi_q        <= '0;
      mult_a_q       <= '0;
      mult_b_q       <= '0;
      // DIV datapath
      remainder_q    <= '0;
      quotient_q     <= '0;
      divisor_q      <= '0;
      neg_quot_q     <= 1'b0;
      neg_rem_q      <= 1'b0;
    end
    else begin
      // Per-cycle updates for regs that evolve over the iteration.
      state_q     <= state_d;
      cnt_q       <= cnt_d;
      result_q    <= result_d;
      pp_lo_q     <= pp_lo_d;
      pp_hi_q     <= pp_hi_d;
      remainder_q <= remainder_d;
      quotient_q  <= quotient_d;

      // Latch-once regs at each accepted-start edge. These don't need _d
      // shadows — they're written exactly once per op and held
      // constant through S_MUL / S_DIV.
      if (start_accept) begin
        op_q <= op;
        rob_idx_q <= rob_idx;
        rob_seq_q <= rob_seq;
        branch_mask_q <= branch_mask;
        pdst_q    <= pdst;
        rd_wen_q  <= rd_wen;
        if (op_is_mul) begin
          mult_a_q <= mult_ext_a;
          mult_b_q <= mult_ext_b;
        end else begin
          divisor_q  <= ub;
          neg_quot_q <= neg_a ^ neg_b;
          neg_rem_q  <= neg_a;
        end
      end
    end
  end

  // ==================================================================
  // Output packet — held throughout S_DONE until the CDB grants this client.
  // ==================================================================
  always_comb begin
    complete         = '0;
    complete.valid   = (state_q == S_DONE);
    complete.rob_idx = rob_idx_q;
    complete.rob_seq = rob_seq_q;
    complete.pdst    = pdst_q;
    complete.rd_wen  = rd_wen_q;
    complete.result  = result_q;
  end

  assign busy   = (state_q == S_MUL) || (state_q == S_DIV);

`ifndef SYNTHESIS
  // holder-protocol tripwires.
  logic               tw_held_prev_q = 1'b0;
  completion_packet_t tw_complete_prev_q;

  always @(posedge clk) begin
    if (rst_n === 1'b1) begin
      // A start the unit does not accept is a silently dropped op -- the
      // core's fu_ready gating and start_accept must agree cycle-exactly.
      if (start && !start_accept) begin
        $fatal(1, "rv32i_ss_muldiv: start while not acceptable (dropped op)");
      end

      // Flow-and-reject: an ungranted S_DONE result is never scrubbed or
      // mutated (kill spares S_DONE by design); it leaves only through
      // complete_ready. Whole-packet compare: seq/pdst/rd_wen included.
      if (tw_held_prev_q && (complete !== tw_complete_prev_q)) begin
        $fatal(1, "rv32i_ss_muldiv: held S_DONE completion scrubbed or mutated");
      end

    end
    // Sampled unconditionally: reset washes the held flag (complete.valid
    // is 0 outside S_DONE), so mid-test resets cannot strand it.
    tw_held_prev_q     <= complete.valid && !complete_ready;
    tw_complete_prev_q <= complete;
  end
`endif

endmodule
