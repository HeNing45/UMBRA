`timescale 1ns/1ps

// rv32i_ss_bp — dual-lookup branch and return-site predictor.
// Each fetch word performs a combinational lookup. A row stores a partial PC
// tag, aligned branch target, two-bit direction counter and return-kind bit.
// Return-site learning updates metadata; the live RAS supplies return targets.
// Conditional training wins any same-row collision with return-site learning.
//
// Per lookup: index selects a row; valid plus matching tag gives a hit.
// A conditional row predicts taken when counter[1] is set. A return row
// identifies a learned return site. With default geometry, pc[31:16] is
// untagged, so instructions 64 KiB apart may alias. Decode validates kind
// and exact conditional targets; execute validates direction and return targets.
//
// Training a matching conditional row steps its saturating counter and
// refreshes the target when taken. A taken miss allocates weakly taken;
// a not-taken miss does not allocate. Counter states are 00 strongly
// not-taken, 01 weakly not-taken, 10 weakly taken and 11 strongly taken.

module rv32i_ss_bp
  import fyp_cpu_pkg::*;
#(
  // Number of direct-mapped predictor rows; must be a power of two.
  parameter int BTB_DEPTH = 64,
  // Derive row-index width so every allocated row is addressable.
  parameter int BTB_IDX_W = $clog2(BTB_DEPTH),
  // Partial tag width. At the default 64-row depth, eight tag bits distinguish
  // word addresses within 64 KiB. Aliases outside that span still require
  // kind validation and target repair.
  parameter int BTB_TAG_W = 8
) (
  input  logic   clk,
  input  logic   rst_n,

  // ---- dual F-stage predict ports (combinational) ----
  input  word_t  pc_f0,
  output logic   predict_taken_f0,
  output logic   predict_return_f0,
  output word_t  predict_target_f0,
  input  word_t  pc_f1,
  output logic   predict_taken_f1,
  output logic   predict_return_f1,
  output word_t  predict_target_f1,

  // ---- E-stage update port (sequential, 1-cycle pulse) ----
  input  logic   update_valid_e,
  input  word_t  update_pc_e,
  input  logic   update_taken_e,
  input  word_t  update_target_e,

  // accepted-dispatch return-site hint. The frontend supplies the
  // target from the live RAS, so the BTB stores only site identity/kind.
  input  logic   return_update_valid,
  input  word_t  return_update_pc
);

  // ==================================================================
  // Geometry guards reject configurations that cannot be represented by
  // the PC slices. Arrays must contain exactly 2**BTB_IDX_W rows.
  if (BTB_DEPTH != (1 << BTB_IDX_W)) begin : g_btb_depth_guard
    $fatal(1, "rv32i_ss_bp: BTB_DEPTH must equal 2**BTB_IDX_W (idx_* are plain PC slices)");
  end

  // {tag,index} must fit inside the PC above the word-align bits.
  if ((BTB_IDX_W + BTB_TAG_W + 1) > 31) begin : g_btb_pc_span_guard
    $fatal(1, "rv32i_ss_bp: BTB_IDX_W+BTB_TAG_W+1 exceeds the 32-bit PC");
  end

  // ==================================================================
  // BTB storage — one block of arrays rather than a packed struct so the
  // tools (Yosys, Verilator) infer BTB_DEPTH small DFFs per field cleanly.
  // ==================================================================
  logic                 btb_valid_q  [BTB_DEPTH];
  logic [BTB_TAG_W-1:0] btb_tag_q    [BTB_DEPTH];
  logic [29:0]          btb_target_q [BTB_DEPTH];   // word-aligned, [31:2]
  logic [1:0]           btb_state_q  [BTB_DEPTH];
  logic                 btb_return_q [BTB_DEPTH];

  logic [BTB_IDX_W-1:0] idx_f0, idx_f1, idx_e, idx_r;
  logic [BTB_TAG_W-1:0] tag_f0, tag_f1, tag_e, tag_r;

  // PC slices derive from index and tag widths; bits [1:0] are word alignment.
  assign idx_f0 = pc_f0[BTB_IDX_W+1:2];
  assign tag_f0 = pc_f0[BTB_IDX_W+BTB_TAG_W+1:BTB_IDX_W+2];
  assign idx_f1 = pc_f1[BTB_IDX_W+1:2];
  assign tag_f1 = pc_f1[BTB_IDX_W+BTB_TAG_W+1:BTB_IDX_W+2];
  assign idx_e  = update_pc_e[BTB_IDX_W+1:2];
  assign tag_e  = update_pc_e[BTB_IDX_W+BTB_TAG_W+1:BTB_IDX_W+2];
  assign idx_r  = return_update_pc[BTB_IDX_W+1:2];
  assign tag_r  = return_update_pc[BTB_IDX_W+BTB_TAG_W+1:BTB_IDX_W+2];

  logic hit_f0, hit_f1;
  logic match_e;

  assign match_e = btb_valid_q[idx_e] && (btb_tag_q[idx_e] == tag_e);

  // 2-bit saturating counter step.
  function automatic logic [1:0] sat2_update(input logic [1:0] s,
                                             input logic       t);
    unique case (s)
      2'b00: sat2_update = t ? 2'b01 : 2'b00;
      2'b01: sat2_update = t ? 2'b10 : 2'b00;
      2'b10: sat2_update = t ? 2'b11 : 2'b01;
      2'b11: sat2_update = t ? 2'b11 : 2'b10;
    endcase
  endfunction

  // ==================================================================
  // Dual predict (combinational). A miss reports not-taken and a zero
  // target; the frontend then keeps sequential fetch.
  // ==================================================================
  always_comb begin
    hit_f0            = btb_valid_q[idx_f0] && (btb_tag_q[idx_f0] == tag_f0);
    predict_taken_f0  = hit_f0 && !btb_return_q[idx_f0]
                      && btb_state_q[idx_f0][1];
    predict_return_f0 = hit_f0 && btb_return_q[idx_f0];
    predict_target_f0 = hit_f0 ? {btb_target_q[idx_f0], 2'b00} : '0;

    hit_f1            = btb_valid_q[idx_f1] && (btb_tag_q[idx_f1] == tag_f1);
    predict_taken_f1  = hit_f1 && !btb_return_q[idx_f1]
                      && btb_state_q[idx_f1][1];
    predict_return_f1 = hit_f1 && btb_return_q[idx_f1];
    predict_target_f1 = hit_f1 ? {btb_target_q[idx_f1], 2'b00} : '0;
  end

  // ==================================================================
  // Update (sequential, posedge clk).
  // ==================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < BTB_DEPTH; i++) begin
        btb_valid_q[i]  <= 1'b0;
        btb_tag_q[i]    <= '0;
        btb_target_q[i] <= '0;
        btb_state_q[i]  <= 2'b01;        // weakly NT — neutral start
        btb_return_q[i] <= 1'b0;
      end
    end else begin
      if (update_valid_e) begin
        if (match_e) begin
          btb_return_q[idx_e] <= 1'b0;
          btb_state_q[idx_e] <= sat2_update(btb_state_q[idx_e], update_taken_e);
          if (update_taken_e) begin
            btb_target_q[idx_e] <= update_target_e[31:2];
          end
        end
        else if (update_taken_e) begin
          btb_valid_q[idx_e]  <= 1'b1;
          btb_tag_q[idx_e]    <= tag_e;
          btb_target_q[idx_e] <= update_target_e[31:2];
          btb_state_q[idx_e]  <= 2'b10;
          btb_return_q[idx_e] <= 1'b0;
        end
      end

      // A different row may learn a return site in the same cycle. On a
      // same-row collision the existing conditional update wins, preserving
      // the predictor-training contract exactly.
      if (return_update_valid &&
          !(update_valid_e && (idx_r == idx_e))) begin
        btb_valid_q[idx_r]  <= 1'b1;
        btb_tag_q[idx_r]    <= tag_r;
        btb_return_q[idx_r] <= 1'b1;
        btb_state_q[idx_r]  <= 2'b11;
      end
    end
  end

endmodule
