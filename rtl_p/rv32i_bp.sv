`timescale 1ns/1ps

// rv32i_bp — 2-bit bimodal branch predictor with tagged direct-mapped BTB.
//
// External contract:
//
//   F-stage predict port (combinational, single-cycle):
//     pc_f               : current fetch PC.
//     predict_taken_f    : 1 if BTB hit AND saturating counter says "taken".
//     predict_target_f   : the cached target PC for that entry.
//     predict_valid_f    : 1 if BTB tag matches (i.e. we trust this prediction).
//                          On predict_valid_f == 0, the F stage falls back to
//                          predict-not-taken (= pc_f + 4).
//
//   E-stage update port (sequential, 1-cycle pulse):
//     update_valid_e     : 1 when a conditional branch resolves this cycle.
//     update_pc_e        : the PC of that branch.
//     update_taken_e     : the actual taken outcome.
//     update_target_e    : the actual target (meaningful only when taken).
//
// Algorithm (2-bit bimodal + tagged direct-mapped BTB):
//
//   Predict at F:
//     idx  = pc_f[5:2]      // 4-bit index into 16-entry BTB
//     tag  = pc_f[11:6]     // 6-bit tag for collision detection
//     hit  = btb_valid[idx] && (btb_tag[idx] == tag)
//     taken? = hit && (btb_state[idx][1] == 1)
//     target = {btb_target[idx], 2'b00}
//
//   Update at E (when update_valid_e):
//     idx, tag computed from update_pc_e the same way.
//     match  = btb_valid[idx] && (btb_tag[idx] == tag)
//
//     if (match):
//       btb_state[idx]  <= sat2_update(btb_state[idx], update_taken_e);
//       if (update_taken_e) btb_target[idx] <= update_target_e[31:2];
//     else if (update_taken_e):                 // allocate on miss + taken
//       btb_valid[idx]  <= 1
//       btb_tag[idx]    <= tag
//       btb_target[idx] <= update_target_e[31:2]
//       btb_state[idx]  <= 2'b10               // weakly taken on allocation
//     // miss + not-taken: do nothing (don't pollute BTB with NT entries)
//
// 2-bit saturating counter encoding:
//   2'b00  strongly NT      predict not-taken
//   2'b01  weakly NT        predict not-taken
//   2'b10  weakly T         predict taken
//   2'b11  strongly T       predict taken
//   Predict-taken when state[1] == 1.
//
// Scope: conditional branches only. Jumps are resolved by the core;
// this predictor has no return stack, global history or indirect-target table.

module rv32i_bp
  import fyp_cpu_pkg::*;
#(
  parameter int BTB_DEPTH = 16,
  parameter int BTB_IDX_W = 4,    // = $clog2(BTB_DEPTH)
  parameter int BTB_TAG_W = 6
) (
  input  logic   clk,
  input  logic   rst_n,

  // ---- F-stage predict port (combinational) ----
  input  word_t  pc_f,
  output logic   predict_taken_f,
  output word_t  predict_target_f,
  output logic   predict_valid_f,

  // ---- E-stage update port (sequential, 1-cycle pulse) ----
  input  logic   update_valid_e,
  input  word_t  update_pc_e,
  input  logic   update_taken_e,
  input  word_t  update_target_e
);

  // ==================================================================
  // BTB storage — one block of arrays rather than a packed struct so the
  // tools (Yosys, Verilator) infer 16 small DFFs per field cleanly.
  // ==================================================================
  logic                 btb_valid_q  [BTB_DEPTH];
  logic [BTB_TAG_W-1:0] btb_tag_q    [BTB_DEPTH];
  logic [29:0]          btb_target_q [BTB_DEPTH];   // word-aligned, [31:2]
  logic [1:0]           btb_state_q  [BTB_DEPTH];

  // ==================================================================
  // PC partition helpers (combinational, used by both predict and update)
  //   pc[1:0] == 00 always (word-aligned RV32)
  //   pc[5:2]   -> 4-bit index    (picks 1 of 16 BTB slots)
  //   pc[11:6]  -> 6-bit tag      (collision distinguisher)
  //   pc[31:12] -> discarded      (BTB is small; aliasing happens & is
  //                                detected by the tag compare)
  // ==================================================================
  logic [BTB_IDX_W-1:0] idx_f, idx_e;
  logic [BTB_TAG_W-1:0] tag_f, tag_e;

  assign idx_f = pc_f[5:2];
  assign tag_f = pc_f[11:6];
  assign idx_e = update_pc_e[5:2];
  assign tag_e = update_pc_e[11:6];

  // ==================================================================
  // Live combinational helpers (used by predict + update)
  // ==================================================================
  logic        hit_f;
  logic        match_e;

  assign match_e = btb_valid_q[idx_e] && (btb_tag_q[idx_e] == tag_e);

  // ==================================================================
  // 2-bit saturating counter step.
  //   t = 1  ->  increment, clamp at 2'b11
  //   t = 0  ->  decrement, clamp at 2'b00
  // ==================================================================
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
  // Predict (combinational). A miss reports no valid prediction and a zero
  // target; the F stage should then fall back to pc_f + 4.
  // ==================================================================
  always_comb begin
    hit_f            = btb_valid_q[idx_f] && (btb_tag_q[idx_f] == tag_f);
    predict_valid_f  = hit_f;
    predict_taken_f  = hit_f && btb_state_q[idx_f][1];
    predict_target_f = hit_f ? {btb_target_q[idx_f], 2'b00} : '0;
  end

  // ==================================================================
  // Update (sequential, posedge clk). Existing entries train the 2-bit
  // counter; a taken miss allocates a new BTB entry; a not-taken miss is
  // ignored to avoid polluting the small BTB.
  // ==================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < BTB_DEPTH; i++) begin
        btb_valid_q[i]  <= 1'b0;
        btb_tag_q[i]    <= '0;
        btb_target_q[i] <= '0;
        btb_state_q[i]  <= 2'b01;        // weakly NT — neutral start
      end
    end else begin
      if (update_valid_e) begin
        if (match_e) begin
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
        end
      end
    end
  end

endmodule
