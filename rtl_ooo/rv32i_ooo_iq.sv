`timescale 1ns/1ps

// =============================================================================
// rv32i_ooo_iq.sv -- decoupled scalar issue queue
// =============================================================================
// The IQ is a separate structure from the ROB. A uop
// occupies one IQ slot from dispatch until it ISSUES, then frees the slot; the
// matching ROB entry lives on until COMMIT. So this 16-deep IQ feeds the
// 32-deep ROB, and the IQ recycles slots far faster than the ROB.
//
//   * wakeup  = centralized physical-register ready table (ready_vec, from the
//               PRF). A slot is ready when both its physical sources read
//               ready. Accepted writeback updates the registered table;
//               selection observes it the next cycle, with no same-cycle bypass.
//   * select  = oldest-ready. Age is the ROB ring-distance of the slot's
//               rob_idx from the current ROB head (0 == head == oldest). Among
//               ready slots, the smallest age wins.
//
// Recovery compares each slot's ROB ring age to the recovering branch's age.
// The branch and older instructions survive, independent of checkpoint-ID reuse.
//
// Loads are head-gated (issue only when rob_idx == rob_head_idx) and a
// store waits for its DATA operand (prs2) even though its src2_sel is IMM
// (address = rs1+imm).
// =============================================================================

import rv32i_ooo_pkg::OOO_IQ_DEPTH;
import rv32i_ooo_pkg::OOO_IQ_BITS;
import rv32i_ooo_pkg::OOO_PHYS_REGS;
import rv32i_ooo_pkg::iq_entry_t;
import rv32i_ooo_pkg::rob_idx_t;
import rv32i_ooo_pkg::ooo_src_sel_e;
import rv32i_ooo_pkg::OOO_SRC_REG;
import rv32i_ooo_pkg::OOO_FU_ALU;
import rv32i_ooo_pkg::OOO_FU_LSU;

module rv32i_ooo_iq (
    input  logic clk,
    input  logic rst_n,
    input  logic trap_flush,
    input  logic branch_recover_req,
    // Recovery kills by ROB ring distance, not by checkpoint mask, so the
    // IQ takes the recovering branch's rob_idx (not its checkpoint id).
    input  rob_idx_t recover_rob_idx,

    // ---- Allocate: dispatch writes one renamed uop into a free IQ slot ----
    input  logic      alloc_valid,
    input  iq_entry_t alloc_entry,
    output logic      alloc_ready,    // ~iq_full: a free slot exists this cycle

    // ---- Wakeup: centralized physreg ready table (PRF) ----
    // ready_vec[p] == 1 means physreg p holds a produced value. ready_vec[0] is
    // forced 1 (x0). NOTE: a slot's prs1/prs2 are only real dependencies when
    // src1_sel/src2_sel == OOO_SRC_REG -- non-register operands carry a stale
    // prs and must not be awaited. Stores separately require their data source.
    input  logic [OOO_PHYS_REGS-1:0] ready_vec,

    // ---- Age reference: current ROB head index (oldest in-flight) ----
    input  rob_idx_t rob_head_idx,

    // ---- Issue: the selected oldest-ready uop ----
    output logic      issue_valid,
    output iq_entry_t issue_entry,
    input  logic      issue_ready,     // downstream accepts the issued uop

    // FU availability
    input logic alu_fu_ready,
    input logic muldiv_fu_ready,
    input logic lsu_fu_ready
);

    // -------------------------------------------------------------------------
    // State: one valid bit + one payload per slot. Slots form a free pool,
    // independent of ROB row indices.
    // -------------------------------------------------------------------------
    iq_entry_t entry_q [OOO_IQ_DEPTH];
    logic      valid_q [OOO_IQ_DEPTH];

    // Combinational scheduling signals (driven by the scheduler always_comb).
    rob_idx_t                slot_age   [OOO_IQ_DEPTH];
    logic [OOO_IQ_DEPTH-1:0] slot_ready;
    logic [OOO_IQ_BITS-1:0]  issue_idx;
    logic                    issue_hit;

    // Free-slot pick and fire handshakes.
    logic [OOO_IQ_BITS-1:0]  free_idx;
    logic                    have_free;
    logic                    alloc_fire;
    logic                    issue_fire;

    integer rst_i;

    // -------------------------------------------------------------------------
    // Scheduler: per-slot age + wakeup eligibility (one loop, both outputs).
    //   age   : entry.rob_idx - rob_head_idx, mod-32 ring distance (0 = oldest);
    //           don't-care for invalid slots (slot_ready gates them out).
    //   ready : valid & both sources ready & the entry's FU free. A source is a
    //           real dependency ONLY when src*_sel == OOO_SRC_REG -- non-register
    //           operands (addi/lui/auipc/jal) carry a stale prs the executor
    //           never reads. x0 works (OOO_SRC_REG, rs==x0 -> p0, ready_vec[0]=1).
    //
    // e_i copies the WHOLE entry first: Icarus aborts (elab_expr.cc:2677) on a
    // struct field-select of an array element (entry_q[i].field) -- even with a
    // genvar -- so the slot MUST be read as a unit. lint_off silences Verilator
    // UNUSEDSIGNAL on the e_i fields this block does not read.
    // -------------------------------------------------------------------------
    /* verilator lint_off UNUSEDSIGNAL */
    always_comb begin
        iq_entry_t e_i;
        logic src1_ready;
        logic src2_ready;
        logic store_data_ready;
        logic fu_ready;
        logic load_at_head;

        for (int i = 0; i < OOO_IQ_DEPTH; i++) begin
            e_i = entry_q[i];
            load_at_head = !e_i.is_load || (e_i.rob_idx == rob_head_idx);
            src1_ready = (e_i.src1_sel != OOO_SRC_REG) || ready_vec[e_i.prs1];
            src2_ready = (e_i.src2_sel != OOO_SRC_REG) || ready_vec[e_i.prs2];
            // Store address uses rs1+imm, but store data still comes from rs2.
            store_data_ready = !e_i.is_store || ready_vec[e_i.prs2];
            fu_ready = (e_i.fu_class == OOO_FU_LSU) ? lsu_fu_ready : ((e_i.fu_class == OOO_FU_ALU)
                        ? alu_fu_ready : muldiv_fu_ready);

            slot_age[i] = e_i.rob_idx - rob_head_idx;
            slot_ready[i] = valid_q[i]
                & src1_ready
                & src2_ready
                & store_data_ready
                & fu_ready
                & load_at_head;
        end
    end
    /* verilator lint_on UNUSEDSIGNAL */

    // =========================================================================
    // Oldest-ready selection combines ring age with operand/FU eligibility.
    //   Among slots with slot_ready[i], pick the SMALLEST slot_age. Set
    //   issue_hit = (any ready slot exists) and issue_idx = its slot index.
    //   No rob_seq tie-break is needed: live entries have distinct ages.
    //   Scan slots from low to high, retaining the first ready slot until a
    //   strictly smaller age is encountered. Index assignment width-casts
    //   the loop variable (issue_idx = OOO_IQ_BITS'(i)) rather than i[..] -- Icarus
    //   rejects constant part-selects of a loop variable inside always_*.
    // =========================================================================
    always_comb begin
        rob_idx_t best_age;   // declaration must precede statements in the block
        issue_hit = 1'b0;
        issue_idx = '0;
        best_age = '0;
        for (int i = 0; i < OOO_IQ_DEPTH; i ++) begin
            if (slot_ready[i]) begin
                if (!issue_hit || (slot_age[i] < best_age)) begin
                    issue_hit = 1'b1;
                    issue_idx = OOO_IQ_BITS'(i);
                    best_age = slot_age[i];
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Free-slot pick: lowest-index slot with valid_q == 0.
    // Descending scan so the lowest free index wins.
    // -------------------------------------------------------------------------
    always_comb begin
        have_free = 1'b0;
        free_idx  = '0;
        for (int j = OOO_IQ_DEPTH - 1; j >= 0; j--) begin
            if (!valid_q[j]) begin
                have_free = 1'b1;
                free_idx  = OOO_IQ_BITS'(j);   // width-cast, not j[..]: Icarus dislikes
            end                                //   constant part-selects of a loop var
        end
    end

    // -------------------------------------------------------------------------
    // Outputs and fire handshakes.
    // alloc_ready is a plain ~full: when the IQ is exactly full we stall
    // dispatch for one cycle even if a slot is issuing this cycle (a free slot
    // appears next cycle). No same-cycle dequeue credit is used.
    // -------------------------------------------------------------------------
    assign alloc_ready = have_free;
    assign issue_valid = issue_hit;
    assign issue_entry = entry_q[issue_idx];

    assign alloc_fire  = alloc_valid & alloc_ready;
    assign issue_fire  = issue_valid & issue_ready;

    // -------------------------------------------------------------------------
    // Slot update: alloc writes a free slot; issue frees the selected slot;
    // trap_flush/reset clear all valids. alloc_fire and issue_fire never target the
    // same slot (free_idx is invalid, issue_idx is valid -> disjoint).
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (rst_i = 0; rst_i < OOO_IQ_DEPTH; rst_i++) begin
                valid_q[rst_i] <= 1'b0;
                entry_q[rst_i] <= '0;
            end
        end else if (trap_flush) begin
            for (rst_i = 0; rst_i < OOO_IQ_DEPTH; rst_i++) begin
                valid_q[rst_i] <= 1'b0;
                entry_q[rst_i] <= '0;
            end
        end else if (branch_recover_req) begin
            // Kill every slot younger than the recovering branch by ROB
            // ring distance -- exactly the ROB rollback predicate. slot_age is
            // entry.rob_idx - rob_head_idx (computed by the scheduler above); the
            // branch's own age is recover_rob_idx - rob_head_idx. age-based
            // kill uses distinct ROB rows within the in-flight window, avoiding
            // aliases from checkpoint identifiers that may be freed and reused.
            // The issue gate (issue_accept) blocks issue during recovery, so no
            // issuing slot needs separate handling here.
            for (rst_i = 0; rst_i < OOO_IQ_DEPTH; rst_i++) begin
                if (valid_q[rst_i] &&
                    (slot_age[rst_i] > (recover_rob_idx - rob_head_idx))) begin
                    valid_q[rst_i] <= 1'b0;
                    entry_q[rst_i] <= '0;
                end
            end
        end else begin
            if (alloc_fire) begin
                entry_q[free_idx] <= alloc_entry;
                valid_q[free_idx] <= 1'b1;
            end
            if (issue_fire) begin
                valid_q[issue_idx] <= 1'b0;
            end
        end
    end

endmodule
