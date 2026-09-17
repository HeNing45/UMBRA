// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// =============================================================================
// rv32i_ss_iq.sv -- 16-entry decoupled dual-grant issue queue
// =============================================================================
// The IQ is separate from the ROB. Each instruction occupies an IQ entry
// from dispatch until downstream accepts its selected bundle. Its ROB entry
// remains live until retirement. The default capacities are 16 IQ and 32 ROB
// entries, with IQ storage reusable while the instruction awaits execution.
//
// * Wakeup reads the centralized physical-register ready table. A final
// non-trapping ALU execution can set readiness at the edge; selection
// uses it the following cycle and a registered live holder supplies
// the matching value. There is no combinational completion-to-select path.
// * Selection uses a shared pairwise (age, index) ordering matrix. Each
// grant picks the unique oldest candidate. Position 0 binds with full
// FU capacity; position 1 uses capacity left after the first binding.
// Position 0 is older, and jump/CSR operations issue alone on ALU0.
//
// Recovery kills younger entries by ROB ring distance, avoiding aliasing
// when checkpoint identifiers are released and reused.
// Loads issue to the AGU on operand readiness. The LQ checks memory order
// before launch. Store address generation waits for its address source;
// the SQ can capture late store data independently.

import rv32i_ss_pkg::OOO_IQ_DEPTH;
import rv32i_ss_pkg::OOO_IQ_BITS;
import rv32i_ss_pkg::OOO_PHYS_REGS;
import rv32i_ss_pkg::iq_entry_t;
import rv32i_ss_pkg::rob_idx_t;
import rv32i_ss_pkg::ooo_src_sel_e;
import rv32i_ss_pkg::OOO_SRC_REG;
import rv32i_ss_pkg::OOO_FU_ALU;
import rv32i_ss_pkg::OOO_FU_MULDIV;
import rv32i_ss_pkg::OOO_FU_LSU;
import rv32i_ss_pkg::ooo_fu_class_e;
import rv32i_ss_pkg::issue_unit_e;
import rv32i_ss_pkg::ISSUE_UNIT_ALU0;
import rv32i_ss_pkg::ISSUE_UNIT_ALU1;
import rv32i_ss_pkg::ISSUE_UNIT_MULDIV;
import rv32i_ss_pkg::ISSUE_UNIT_AGEN;
import rv32i_ss_pkg::OOO_OP_JUMP;

module rv32i_ss_iq (
    input  logic clk,
    input  logic rst_n,
    input  logic trap_flush,
    input  logic branch_recover_req,
    // recovery kills by ROB ring distance, not by checkpoint mask, so the
    // IQ takes the recovering branch's rob_idx (not its checkpoint id).
    input  rob_idx_t recover_rob_idx,

    // ---- Allocate: dispatch writes 0-2 renamed uops into free IQ entries ----
    input logic [1:0] bundle_size,
    input logic [1:0] iq_alloc_slot_valid,
    input logic       bundle_fire,
    input  iq_entry_t[1:0] iq_alloc_entry,
    output logic      iq_alloc_ready,    // need-aware: free_count >= bundle_size

    // ---- Wakeup: centralized physreg ready table (PRF) ----
    // ready_vec[p] == 1 means p's value is available in the PRF or bypass. ready_vec[0] is
    // forced 1 (x0). NOTE: an entry's prs1/prs2 are real dependencies only when
    // src1_sel/src2_sel == OOO_SRC_REG -- non-register operands carry a stale
    // prs and must NOT be awaited by the readiness calculation below.
    input  logic [OOO_PHYS_REGS-1:0] ready_vec,

    // ---- Age reference: current ROB head index (oldest in-flight) ----
    input  rob_idx_t rob_head_idx,

    // ---- Issue: age-ordered grant positions (position 0 is older) ----
    output logic        [1:0] issue_valid,
    output iq_entry_t   [1:0] issue_entry,
    output issue_unit_e [1:0] issue_unit,
    input  logic              issue_accept, // downstream accepts the whole grant set

    // FU availability
    input logic alu0_fu_ready,
    input logic alu1_fu_ready,
    input logic muldiv_fu_ready,
    input logic lsu_fu_ready
);

    typedef logic [OOO_IQ_BITS:0] iq_count_t;

    // -------------------------------------------------------------------------
    // State: one valid bit + one payload per entry. IQ entries are not addressed
    // by rob_idx; unlike the ROB, this structure is a free pool.
    // -------------------------------------------------------------------------
    iq_entry_t entry_q [OOO_IQ_DEPTH];
    logic      valid_q [OOO_IQ_DEPTH];

    // Combinational scheduling signals (driven by the scheduler always_comb).
    rob_idx_t                entry_age   [OOO_IQ_DEPTH];
    logic [OOO_IQ_DEPTH-1:0] entry_operand_ready;
    // Jump/CSR operations issue alone on ALU0. Classify each entry once
    // and reuse that result for both grant positions.
    logic [OOO_IQ_DEPTH-1:0] entry_issue_solo;
    logic                    select_issue_solo [1:0];
    issue_unit_e             select_unit [1:0];
    logic [OOO_IQ_BITS-1:0]  select_idx [1:0];
    logic [1:0]              select_hit;

    // Combinational dominance-selection signals, driven by the scheduler.
    logic [OOO_IQ_DEPTH-1:0] beats_col [OOO_IQ_DEPTH];
    logic [OOO_IQ_DEPTH-1:0] cand0;
    logic [OOO_IQ_DEPTH-1:0] cand1;
    logic [OOO_IQ_DEPTH-1:0] win0_onehot;
    logic [OOO_IQ_DEPTH-1:0] win1_onehot;
    logic [OOO_IQ_DEPTH-1:0] bindable_full;
    logic [OOO_IQ_DEPTH-1:0] bindable_shadow;
    issue_unit_e             unit_full   [OOO_IQ_DEPTH];
    issue_unit_e             unit_shadow [OOO_IQ_DEPTH];

    logic shadow_alu0;
    logic shadow_alu1;
    logic shadow_muldiv;
    logic shadow_agen;

    // -------------------------------------------------------------------------
    // ONE bindability policy for both grant positions. The single deliberate
    // asymmetry between the positions is the solo guard, passed EXPLICITLY as
    // solo_blocks_alu1 instead of baked into two near-identical case blocks:
    // grant 0 passes entry_issue_solo[i] -- a solo op is ALU0-only; with
    // ALU0 taken it is simply unbindable and never blocks a younger bindable
    // candidate from the age competition;
    // grant 1 passes 1'b0 -- NOT because a solo may bind ALU1 there, but
    // because cand1 already masks solos out at the candidate-set level,
    // so a per-entry guard here would be dead logic whose deadness depends
    // NON-LOCALLY on that cand1 term. If the cand1 solo mask is ever
    // removed, passing the entry's solo bit here DOES NOT restore the
    // invariant: solo_blocks_alu1 gates only the ALU1 arm, so when grant 0
    // takes MULDIV/AGEN and ALU0 survives into the shadow capacity, a solo
    // would still bind ALU0 at position 1 through the ungated sh_alu0 arm.
    // The exclusion must be re-established at the CANDIDATE-SET level (or
    // by refusing to bind a solo to ANY unit at grant 1). The architectural
    // invariant is pinned at the output: "jump/CSR granted at position 1".
    // -------------------------------------------------------------------------
    typedef struct packed {
        logic        bindable;
        issue_unit_e unit;
    } iq_bind_t;

    function automatic iq_bind_t iq_bind_unit(
        input ooo_fu_class_e fu_class,
        input logic          sh_alu0,
        input logic          sh_alu1,
        input logic          sh_muldiv,
        input logic          sh_agen,
        input logic          solo_blocks_alu1);
        iq_bind_t r;
        r.bindable = 1'b0;
        r.unit     = ISSUE_UNIT_ALU0;
        unique case (fu_class)
            OOO_FU_ALU : begin
                if (sh_alu0) begin
                    r.bindable = 1'b1;
                    r.unit     = ISSUE_UNIT_ALU0;
                end else if (!solo_blocks_alu1 && sh_alu1) begin
                    r.bindable = 1'b1;
                    r.unit     = ISSUE_UNIT_ALU1;
                end
            end
            OOO_FU_MULDIV : begin
                if (sh_muldiv) begin
                    r.bindable = 1'b1;
                    r.unit     = ISSUE_UNIT_MULDIV;
                end
            end
            OOO_FU_LSU : begin
                if (sh_agen) begin
                    r.bindable = 1'b1;
                    r.unit     = ISSUE_UNIT_AGEN;
                end
            end
            default : r.bindable = 1'b0;
        endcase
        iq_bind_unit = r;
    endfunction

    // Free-entry pick + fire handshakes.
    logic [OOO_IQ_BITS-1:0] free_idx [1:0];
    iq_count_t              free_count;
    logic [1:0]              issue_fire;

    integer rst_i;

    // -------------------------------------------------------------------------
    // Scheduler: per-entry age + wakeup eligibility (one loop, all per-entry
    // terms), then a PARALLEL DOMINANCE SELECT for both grant positions.
    // age : entry.rob_idx - rob_head_idx, mod-32 ring distance (0 = oldest);
    // don't-care for invalid entries (entry_operand_ready gates them).
    // ready : valid & both sources ready & the entry's FU free. A source is a
    // real dependency ONLY when src*_sel == OOO_SRC_REG -- non-register
    // operands (addi/lui/auipc/jal) carry a stale prs the executor
    // never reads. x0 works (OOO_SRC_REG, rs==x0 -> p0, ready_vec[0]=1).
    //
    // One pairwise matrix defines beats_col[i][j]: entry j precedes entry i
    // in strict (age, index) order. Independent ring-distance comparisons feed
    // each grant's reduction: win[i] = cand[i] & ~|(cand & beats_col[i]).
    // The reductions have no data feedback and can be balanced. Both grants
    // share the matrix; grant 1 adds the dependency on remaining FU capacity.
    // Strict index tie-breaking guarantees a unique winner for each candidate set.
    //
    // e_i copies the whole entry because Icarus cannot elaborate a field select
    // through a struct-array element. The lint scope covers unused copied fields.
    /* verilator lint_off UNUSEDSIGNAL */
    always_comb begin
        iq_entry_t e_i;
        logic src1_ready;
        logic src2_ready;
        iq_bind_t bind_r;
        select_hit = '0;
        select_idx[0] = '0;
        select_idx[1] = '0;
        select_unit[0] = ISSUE_UNIT_ALU0;
        select_unit[1] = ISSUE_UNIT_ALU0;
        // Position 1 can never carry a solo (cand1 excludes them); [1] is kept
        // in the family shape and stays 0.
        select_issue_solo[0] = 1'b0;
        select_issue_solo[1] = 1'b0;

        shadow_alu0 = alu0_fu_ready;
        shadow_alu1 = alu1_fu_ready;
        shadow_muldiv = muldiv_fu_ready;
        shadow_agen = lsu_fu_ready;

        // Per-entry terms: age, operand readiness, solo class and full-capacity
        // unit binding for grant 0. Shadow capacity changes only after grant 0 is
        // chosen and is then used by grant 1.
        for (int i = 0; i < OOO_IQ_DEPTH; i++) begin
            e_i = entry_q[i];
            src1_ready = (e_i.src1_sel != OOO_SRC_REG) || ready_vec[e_i.prs1];
            src2_ready = (e_i.src2_sel != OOO_SRC_REG) || ready_vec[e_i.prs2];

            entry_age[i] = e_i.rob_idx - rob_head_idx;
            entry_operand_ready[i] = valid_q[i]
                & src1_ready
                & src2_ready;
            entry_issue_solo[i] = valid_q[i]
                && ((e_i.op_class == OOO_OP_JUMP)
                    || (e_i.csr_op != rv32i_pipeline_pkg::CSR_NONE));

            // one policy, solo guard passed per call site (see iq_bind_unit).
            bind_r = iq_bind_unit(e_i.fu_class, shadow_alu0, shadow_alu1,
                                  shadow_muldiv, shadow_agen,
                                  entry_issue_solo[i]);
            bindable_full[i] = bind_r.bindable;
            unit_full[i]     = bind_r.unit;
        end

        // Pairwise order matrix. Strict (age, index) lexicographic order is a
        // strict TOTAL order over the 16 positions, so every nonempty
        // candidate set has exactly one un-dominated member. Live entries
        // never share a rob_idx (one ROB slot per in-flight uop), so among
        // real candidates the index tie-break is dead weight, carried only so
        // the order stays total in any state a TB can drive; (j < i) is an
        // elaboration constant, not logic.
        for (int i = 0; i < OOO_IQ_DEPTH; i++) begin
            for (int j = 0; j < OOO_IQ_DEPTH; j++) begin
                beats_col[i][j] = (entry_age[j] < entry_age[i])
                               || ((entry_age[j] == entry_age[i]) && (j < i));
            end
        end

        // Grant 0: un-dominated member of {operand-ready & bindable at full
        // capacity}. beats_col[i][i] is identically 0 (nothing precedes
        // itself), so no self-mask is needed.
        cand0 = entry_operand_ready & bindable_full;
        for (int i = 0; i < OOO_IQ_DEPTH; i++) begin
            win0_onehot[i] = cand0[i] && !(|(cand0 & beats_col[i]));
        end
        select_hit[0] = |cand0;
        for (int i = 0; i < OOO_IQ_DEPTH; i++) begin
            if (win0_onehot[i]) select_idx[0] |= OOO_IQ_BITS'(i);
        end
        if (select_hit[0]) begin
            select_unit[0]       = unit_full[select_idx[0]];
            select_issue_solo[0] = entry_issue_solo[select_idx[0]];
        end

        // Consume grant 0's unit: grant 1 competes for the shadow capacity.
        if (select_hit[0]) begin
            unique case (select_unit[0])
                ISSUE_UNIT_ALU0 : shadow_alu0 = 1'b0;
                ISSUE_UNIT_ALU1 : shadow_alu1 = 1'b0;
                ISSUE_UNIT_MULDIV : shadow_muldiv = 1'b0;
                ISSUE_UNIT_AGEN : shadow_agen = 1'b0;
            endcase
        end

        // Shadow-capacity bindability + unit for grant 1. solo_blocks_alu1 is
        // 1'b0 HERE ONLY: solos are excluded from cand1 itself, so the
        // per-entry guard would be dead -- and passing it would NOT survive a
        // cand1 change anyway (it gates only the ALU1 arm; a solo could still
        // bind ALU0 here when grant 0 consumed MULDIV/AGEN). See the
        // iq_bind_unit header for the non-local dependency and the correct
        // remediation; the output pin "jump/CSR granted at position 1" holds
        // the architectural invariant.
        for (int i = 0; i < OOO_IQ_DEPTH; i++) begin
            e_i = entry_q[i];
            bind_r = iq_bind_unit(e_i.fu_class, shadow_alu0, shadow_alu1,
                                  shadow_muldiv, shadow_agen, 1'b0);
            bindable_shadow[i] = bind_r.bindable;
            unit_shadow[i]     = bind_r.unit;
        end

        // Grant 1: un-dominated member of {operand-ready, non-solo, not the
        // grant-0 entry, bindable at shadow capacity}; competes only when
        // grant 0 landed and is not a solo (a solo winner suppresses the
        // second grant entirely). win0_onehot IS the grant-0 entry mask
        // whenever select_hit[0] holds, which cand1 requires.
        if (select_hit[0] && !select_issue_solo[0]) begin
            cand1 = entry_operand_ready & ~entry_issue_solo
                  & ~win0_onehot & bindable_shadow;
        end else begin
            cand1 = '0;
        end
        for (int i = 0; i < OOO_IQ_DEPTH; i++) begin
            win1_onehot[i] = cand1[i] && !(|(cand1 & beats_col[i]));
        end
        select_hit[1] = |cand1;
        for (int i = 0; i < OOO_IQ_DEPTH; i++) begin
            if (win1_onehot[i]) select_idx[1] |= OOO_IQ_BITS'(i);
        end
        if (select_hit[1]) begin
            select_unit[1] = unit_shadow[select_idx[1]];
        end
    end

    // -------------------------------------------------------------------------
    // Free-entry pick: lowest-index entry with valid_q == 0.
    // Descending scan so the lowest free index wins.
    // -------------------------------------------------------------------------
    always_comb begin
        free_idx[0] = '0;
        free_idx[1] = '0;
        free_count  = '0;
        for (int j = OOO_IQ_DEPTH - 1; j >= 0; j--) begin
            if (!valid_q[j]) begin
                free_count = free_count + 1'b1;
                free_idx[1] = free_idx[0];
                free_idx[0] = OOO_IQ_BITS'(j);   // width-cast, not j[..]: Icarus dislikes
            end                                // constant part-selects of a loop var
        end
    end

    // -------------------------------------------------------------------------
    // Outputs + fire handshakes.
    // iq_alloc_ready is need-aware from the registered entry occupancy. Entries
    // freed by issue this edge become available to dispatch on the next cycle.
    // -------------------------------------------------------------------------
    assign iq_alloc_ready = free_count >= iq_count_t'(bundle_size);
    assign issue_valid[0] = select_hit[0];
    assign issue_valid[1] = select_hit[1];
    assign issue_entry[0] = entry_q[select_idx[0]];
    assign issue_entry[1] = entry_q[select_idx[1]];
    assign issue_unit[0] = select_unit[0];
    assign issue_unit[1] = select_unit[1];

    assign issue_fire = issue_accept ? issue_valid : 2'b00;

    // -------------------------------------------------------------------------
    // Entry update: bundle_fire writes free entries; issue frees the selected
    // entries; trap_flush/reset clear all valids. Allocation and issue cannot
    // target one entry (free_idx is invalid, select_idx is valid -> disjoint).
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
            // Kill every entry younger than the recovering branch by ROB ring
            // distance. Ages identify the current in-flight window even when released
            // checkpoint IDs are reused. Older entries retain readiness and payload.
            for (rst_i = 0; rst_i < OOO_IQ_DEPTH; rst_i++) begin
                if (valid_q[rst_i] &&
                    (entry_age[rst_i] > (recover_rob_idx - rob_head_idx))) begin
                    valid_q[rst_i] <= 1'b0;
                    entry_q[rst_i] <= '0;
                end
            end
        end else begin
            if (bundle_fire) begin
                for (int i = 0; i < 2; i++) begin
                    if (iq_alloc_slot_valid[i]) begin
                        entry_q[free_idx[i]] <= iq_alloc_entry[i];
                        valid_q[free_idx[i]] <= 1'b1;
                    end
                end
            end
            if (issue_fire[0]) begin
                valid_q[select_idx[0]] <= 1'b0;
            end
            // Without this second removal arm, a position-1 issue would stay
            // resident and become re-issuable.
            if (issue_fire[1]) begin
                valid_q[select_idx[1]] <= 1'b0;
            end
        end
    end


`ifndef SYNTHESIS
    /* verilator lint_off SYNCASYNCNET */
    // Allocation event integrity: alloc trusts bundle_fire to include iq_alloc_ready.
    always @(posedge clk) begin
        if (rst_n === 1'b1) begin
            if (bundle_fire && !iq_alloc_ready) begin
                $fatal(1, "rv32i_ss_iq: bundle_fire with no free IQ entry (fire-honesty)");
            end

            // Jump/CSR are issue-solo and ALU0-only.
            if (issue_valid[0] && select_issue_solo[0] && issue_valid[1]) begin
                $fatal(1, "rv32i_ss_iq: second grant issued alongside a solo winner");
            end
            if (issue_valid[1] &&
                ((issue_entry[1].op_class == OOO_OP_JUMP) ||
                 (issue_entry[1].csr_op != rv32i_pipeline_pkg::CSR_NONE))) begin
                $fatal(1, "rv32i_ss_iq: jump/CSR granted at position 1");
            end
            if (issue_valid[0] && select_issue_solo[0] &&
                (issue_unit[0] == ISSUE_UNIT_ALU1)) begin
                $fatal(1, "rv32i_ss_iq: solo op bound to ALU1");
            end

            // Two-grant integrity:
            // a younger grant needs an older one, and the two grants can
            // never name one IQ entry.
            if (issue_valid[1] && !issue_valid[0]) begin
                $fatal(1, "rv32i_ss_iq: position 1 granted without position 0");
            end
            if (issue_valid[0] && issue_valid[1] &&
                (select_idx[0] == select_idx[1])) begin
                $fatal(1, "rv32i_ss_iq: both grant positions name one IQ entry");
            end

            // Position 0 is the oldest bindable candidate. Position 1 excludes its
            // winner and can use only remaining FU capacity, so it cannot precede
            // position 0 in the same strict age order.
            if (issue_valid[0] && issue_valid[1] &&
                !((issue_entry[0].rob_idx - rob_head_idx) <
                  (issue_entry[1].rob_idx - rob_head_idx))) begin
                $fatal(1, "rv32i_ss_iq: grant positions violate age order");
            end
        end
    end
    /* verilator lint_on SYNCASYNCNET */
`endif

endmodule
