// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// rv32i_muldiv — multi-cycle multiplier/divider for RV32M.
//
// External contract (must NOT change without updating pipeline_core.sv
// and rv32i_hazard.sv):
//   start  : level, held while the op is in E and the unit is IDLE.
//   busy   : asserted whenever state != IDLE; the pipeline stalls F/D/E.
//   done   : asserted only in S_DONE; the pipeline advances E that cycle.
//   result : valid throughout S_DONE.
//
// FSM:
//
//   S_IDLE ── start && op_is_mul  ──▶ S_MUL ─┐
//   S_IDLE ── start && !op_is_mul ──▶ S_DIV ─┴── cnt_q==0 ──▶ S_DONE ──▶ S_IDLE
//
// Datapaths:
//   S_MUL uses a 32-cycle shift-add multiplier over |A| and |B|, then applies
//   the saved result sign before selecting the low/high architectural word.
//   S_DIV uses a 32-cycle restoring divider over |A| and |B|, with RV32M
//   divide-by-zero and signed-overflow fast paths handled at dispatch.

module rv32i_muldiv
  import fyp_cpu_pkg::*;
  import rv32i_pipeline_pkg::*;
(
  input  logic       clk,
  input  logic       rst_n,
  input  logic       start,
  input  muldiv_op_e op,
  input  word_t      a,
  input  word_t      b,
  output logic       busy,
  output logic       done,
  output word_t      result
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

  // ==================================================================
  // Multiply datapath state
  //   product_q : 64-bit {accumulator, shrinking |B|}.
  //               init {32'h0, ub} at IDLE→S_MUL, shift-add per cycle.
  //   multiplicand_q : |A|, latched once at IDLE→S_MUL.
  //   neg_result_q   : neg_a XOR neg_b, drives the final 64-bit negate.
  // ==================================================================
  logic [63:0] product_q, product_d;
  word_t       multiplicand_q;
  logic        neg_result_q;

  // ==================================================================
  // Divide datapath state
  //   remainder_q : 33 bits — extra MSB for trial-subtract borrow.
  //   quotient_q  : 32 bits, built bit-by-bit at the LSB.
  //   divisor_q   : |B|, latched once at IDLE→S_DIV.
  //   neg_quot_q  : neg_a XOR neg_b (post-correct quotient).
  //   neg_rem_q   : neg_a            (post-correct remainder).
  // ==================================================================
  logic [32:0] remainder_q, remainder_d;
  logic [31:0] quotient_q,  quotient_d;
  logic [31:0] divisor_q;
  logic        neg_quot_q, neg_rem_q;

  // ==================================================================
  // Live combinational helpers — computed from inputs every cycle.
  //
  //   op_is_mul : 1 if op ∈ {MUL, MULH, MULHSU, MULHU}.
  //   neg_a/_b  : 1 if that operand should be negated to its magnitude.
  //   ua / ub   : the magnitudes |A| and |B|.
  //
  // These are always live, regardless of state. The IDLE→active
  // transition just samples them on that cycle.
  //
  // Decode covers both MUL and DIV families:
  //   MUL / MULH / DIV / REM : both operands signed
  //   MULHSU                  : only A signed (B is unsigned)
  //   MULHU / DIVU / REMU     : both unsigned → neg_a = neg_b = 0
  // ==================================================================
  logic  op_is_mul;
  logic  neg_a, neg_b;
  word_t ua, ub;
  logic [31:0] addend;
  logic [32:0] partial_sum;
  logic [63:0] final_64;
  logic [32:0] rem_shift;
  logic [31:0] quot_shift;
  logic [32:0] rem_trial;

  assign op_is_mul = (op == MD_MUL)    || (op == MD_MULH)
                  || (op == MD_MULHSU) || (op == MD_MULHU);

  always_comb begin
    unique case (op)
      MD_MUL, MD_MULH,
      MD_DIV, MD_REM:  begin neg_a = a[31]; neg_b = b[31]; end
      MD_MULHSU:        begin neg_a = a[31]; neg_b = 1'b0;  end
      default:          begin neg_a = 1'b0;  neg_b = 1'b0;  end
    endcase
  end

  assign ua = neg_a ? -a : a;
  assign ub = neg_b ? -b : b;

  // ==================================================================
  // Next-state / next-data
  // ==================================================================
  always_comb begin
    // hold defaults — every _d must have a defined assignment to avoid
    // inferred latches on the MUL/DIV datapath shadows.
    state_d     = state_q;
    cnt_d       = cnt_q;
    result_d    = result_q;
    product_d   = product_q;
    remainder_d = remainder_q;
    quotient_d  = quotient_q;
    addend      = '0;
    partial_sum = '0;
    final_64    = '0;
    rem_shift   = '0;
    quot_shift  = '0;
    rem_trial   = '0;

    unique case (state_q)

      // ---------- S_IDLE: dispatch on op class ---------------------
      S_IDLE: begin
        if (start) begin
          cnt_d = 6'd32;

          if (op_is_mul) begin
            state_d   = S_MUL;
            product_d = {32'h0, ub};
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

      // ---------- S_MUL: 32-cycle schoolbook iteration -------------
      S_MUL: begin
        if (cnt_q == 6'd0) begin
          state_d = S_DONE;
        end else begin
          cnt_d = cnt_q - 6'd1;
          addend = product_q[0] ? multiplicand_q : 32'h0;
          partial_sum = {1'b0, product_q[63:32]} + {1'b0, addend};
          product_d = {partial_sum, product_q[31:1]};
          if (cnt_q == 6'd1) begin
            final_64 = neg_result_q ? -product_d : product_d;
            unique case (op_q)
              MD_MUL : result_d = final_64[31:0];
              MD_MULH, MD_MULHSU, MD_MULHU : result_d = final_64[63:32];
              // Unreachable: S_MUL is entered only for the mul-family ops.
              // Use a deterministic fallback if the operation is invalid.
              default: result_d = '0;
            endcase
          end
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

      // ---------- S_DONE: hand off; one cycle and back to IDLE -----
      S_DONE: begin
        state_d = S_IDLE;
      end

      default: begin
        state_d = S_IDLE;
        cnt_d   = '0;
      end
    endcase
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
      product_q      <= '0;
      multiplicand_q <= '0;
      neg_result_q   <= 1'b0;
      // DIV datapath
      remainder_q    <= '0;
      quotient_q     <= '0;
      divisor_q      <= '0;
      neg_quot_q     <= 1'b0;
      neg_rem_q      <= 1'b0;
    end else begin
      // Per-cycle updates for regs that evolve over the iteration.
      state_q     <= state_d;
      cnt_q       <= cnt_d;
      result_q    <= result_d;
      product_q   <= product_d;
      remainder_q <= remainder_d;
      quotient_q  <= quotient_d;

      // Latch-once regs at the IDLE→active edge. These don't need _d
      // shadows — they're written exactly once per op and held
      // constant through S_MUL / S_DIV.
      if (state_q == S_IDLE && start) begin
        op_q <= op;
        if (op_is_mul) begin
          multiplicand_q <= ua;
          neg_result_q   <= neg_a ^ neg_b;
        end else begin
          divisor_q  <= ub;
          neg_quot_q <= neg_a ^ neg_b;
          neg_rem_q  <= neg_a;
        end
      end
    end
  end

  // ==================================================================
  // Output assigns — the external handshake.
  // DO NOT change without updating pipeline_core.sv and rv32i_hazard.sv.
  // ==================================================================
  assign busy   = (state_q != S_IDLE);
  assign done   = (state_q == S_DONE);
  assign result = result_q;

endmodule
