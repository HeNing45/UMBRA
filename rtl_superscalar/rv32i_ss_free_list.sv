`timescale 1ns/1ps

import rv32i_ss_pkg::OOO_ARCH_REGS;
import rv32i_ss_pkg::OOO_PHYS_REGS;
import rv32i_ss_pkg::phys_reg_t;

module rv32i_ss_free_list (
    input  logic                     clk,
    input  logic                     rst_n,

    // Allocation side: rename asks for 0-2 new destination physregs.
    input  logic [1:0]               preg_slot_need,
    output logic                     preg_avail,
    output phys_reg_t [1:0]          preg_alloc_reg,

    //bundle
    input  logic                     bundle_fire,

    // Commit side: up to two architectural destination updates per cycle,
    // applied in program order (slot 0 then slot 1).
    input  logic [1:0]                    commit_fire,
    input  logic [1:0]                    commit_rd_we,
    input  phys_reg_t [1:0]               commit_pdst,
    input  phys_reg_t [1:0]               commit_stale_pdst,

    // Branch recovery: return all physical regs allocated on the wrong path.
    input  logic                     branch_recover_req,
    input  logic [OOO_PHYS_REGS-1:0] rename_branch_recover_alloc_list,

    // Full precise flush: restore the committed free-list image.
    input  logic                     trap_flush
);

    localparam int PREG_COUNT_BITS = $clog2(OOO_PHYS_REGS + 1);
    typedef logic [PREG_COUNT_BITS-1:0] preg_count_t;

    typedef logic [OOO_PHYS_REGS-1:0] free_bits_t;

    free_bits_t free_bits_q;
    free_bits_t free_bits_d;
    free_bits_t committed_free_bits_q;
    free_bits_t committed_free_bits_d;

    logic [1:0] commit_rd_fire;

    integer reset_i;
    integer free_i;

    assign commit_rd_fire [0] = commit_fire [0] && commit_rd_we [0];
    assign commit_rd_fire [1] = commit_fire [1] && commit_rd_we [1];

    // pop count
    logic [1:0] preg_need;
    preg_count_t free_count;
    logic [1:0] found_alloc;

    assign preg_need = {1'b0, preg_slot_need[0]} + {1'b0, preg_slot_need[1]};

    always_comb begin
        free_bits_d           = free_bits_q;
        committed_free_bits_d = committed_free_bits_q;
        preg_avail            = 1'b0;
        preg_alloc_reg        = '0;
        free_count            = '0;
        found_alloc           = '0;

        for (free_i = 1; free_i < OOO_PHYS_REGS; free_i++) begin
            if (free_bits_q[free_i]) begin
                if (preg_slot_need[0] && !found_alloc[0]) begin
                    preg_alloc_reg[0] = phys_reg_t'(free_i);
                    found_alloc[0]    = 1'b1;
                end else if (preg_slot_need[1] && !found_alloc[1]) begin
                    preg_alloc_reg[1] = phys_reg_t'(free_i);
                    found_alloc[1]    = 1'b1;
                end
                free_count = free_count + 1'b1;
            end
        end

        preg_avail = (free_count >= preg_count_t'(preg_need));

        if (bundle_fire) begin
            if (preg_slot_need[0]) begin
                free_bits_d[preg_alloc_reg[0]] = 1'b0;
            end
            if (preg_slot_need[1]) begin
                free_bits_d[preg_alloc_reg[1]] = 1'b0;
            end
        end

        if (commit_rd_fire[0]) begin
            if (commit_pdst[0] != '0) begin
                free_bits_d[commit_pdst[0]]           = 1'b0;
                committed_free_bits_d[commit_pdst[0]] = 1'b0;
            end

            if (commit_stale_pdst[0] != '0) begin
                free_bits_d[commit_stale_pdst[0]]           = 1'b1;
                committed_free_bits_d[commit_stale_pdst[0]] = 1'b1;
            end
        end
        if (commit_rd_fire[1]) begin
            if (commit_pdst[1] != '0) begin
                free_bits_d[commit_pdst[1]]           = 1'b0;
                committed_free_bits_d[commit_pdst[1]] = 1'b0;
            end

            if (commit_stale_pdst[1] != '0) begin
                free_bits_d[commit_stale_pdst[1]]           = 1'b1;
                committed_free_bits_d[commit_stale_pdst[1]] = 1'b1;
            end
        end

        if (branch_recover_req) begin
            free_bits_d = free_bits_d | rename_branch_recover_alloc_list;
        end

        if (trap_flush) begin
            free_bits_d = committed_free_bits_q;
        end

        free_bits_d[0]           = 1'b0;
        committed_free_bits_d[0] = 1'b0;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            free_bits_q           <= '0;
            committed_free_bits_q <= '0;

            for (reset_i = OOO_ARCH_REGS; reset_i < OOO_PHYS_REGS; reset_i++) begin
                free_bits_q[reset_i]           <= 1'b1;
                committed_free_bits_q[reset_i] <= 1'b1;
            end
        end else begin
            free_bits_q           <= free_bits_d;
            committed_free_bits_q <= committed_free_bits_d;
        end
    end

`ifndef SYNTHESIS
    /* verilator lint_off SYNCASYNCNET */
    // Allocation invariants. twa_ = tripwire, allocation.
    integer twa_i;
    integer twa_count;
    always @(posedge clk) begin
        if (rst_n === 1'b1) begin
            if (free_bits_q[0] || committed_free_bits_q[0]) begin
                $fatal(1, "rv32i_ss_free_list: p0 must never be free");
            end

            // the bundle may only fire when the presented need
            // is satisfiable — a fire without avail is an upstream bug.
            if (bundle_fire && !preg_avail) begin
                $fatal(1, "rv32i_ss_free_list: bundle_fire without preg_avail");
            end

            // slot-indexed allocation integrity, per consuming slot
            for (twa_i = 0; twa_i < 2; twa_i = twa_i + 1) begin
                if (bundle_fire && preg_slot_need[twa_i]) begin
                    if (preg_alloc_reg[twa_i] == '0) begin
                        $fatal(1, "rv32i_ss_free_list: allocated p0");
                    end
                    if (!free_bits_q[preg_alloc_reg[twa_i]]) begin
                        $fatal(1, "rv32i_ss_free_list: allocated a non-free preg");
                    end
                end
            end

            // dual-pop aliasing: two slots must never receive one preg
            if (bundle_fire && (preg_slot_need == 2'b11) &&
                (preg_alloc_reg[0] == preg_alloc_reg[1])) begin
                $fatal(1, "rv32i_ss_free_list: dual-pop aliasing");
            end

            // avail honesty vs an independent count of the free image
            twa_count = 0;
            for (twa_i = 0; twa_i < OOO_PHYS_REGS; twa_i = twa_i + 1) begin
                if (free_bits_q[twa_i]) twa_count = twa_count + 1;
            end
            if (preg_avail !== (twa_count >= {30'b0, preg_need})) begin
                $fatal(1, "rv32i_ss_free_list: preg_avail disagrees with independent count");
            end

            // PER-SLOT commit integrity. Both pregs were claimed at
            // dispatch and neither is released until this edge, so both slots
            // check against the registered pre-update image free_bits_q --
            // including the same-rd WAW case where slot 1's stale IS slot 0's
            // pdst (that preg is allocated, so neither check false-fires).
            for (twa_i = 0; twa_i < 2; twa_i = twa_i + 1) begin
                if (commit_rd_fire[twa_i]) begin
                    if ((commit_pdst[twa_i] != '0) &&
                        free_bits_q[commit_pdst[twa_i]]) begin
                        $fatal(1, "rv32i_ss_free_list: committing pdst was already free (slot %0d)",
                               twa_i);
                    end
                    if ((commit_stale_pdst[twa_i] != '0) &&
                        free_bits_q[commit_stale_pdst[twa_i]]) begin
                        $fatal(1, "rv32i_ss_free_list: double-free of stale_pdst (slot %0d)",
                               twa_i);
                    end
                end
            end

            // Dual-commit aliasing, the retire-side twin of the dual-pop pin
            // above: two retiring slots must never name one preg. Equal pdsts
            // means two live instructions claimed one register; equal stales
            // means one register is returned to the free list twice.
            if (commit_rd_fire[0] && commit_rd_fire[1]) begin
                if ((commit_pdst[0] != '0) &&
                    (commit_pdst[0] == commit_pdst[1])) begin
                    $fatal(1, "rv32i_ss_free_list: dual-commit pdst aliasing");
                end
                if ((commit_stale_pdst[0] != '0) &&
                    (commit_stale_pdst[0] == commit_stale_pdst[1])) begin
                    $fatal(1, "rv32i_ss_free_list: dual-commit stale aliasing");
                end
            end

            // Allocation-versus-return disjointness: a
            // preg handed out this cycle must not also be returned this cycle,
            // or the commit's set would override the allocation's clear and
            // the register would be live in two places at once.
            for (twa_i = 0; twa_i < 2; twa_i = twa_i + 1) begin
                if (bundle_fire && preg_slot_need[twa_i]) begin
                    if (commit_rd_fire[0] && (commit_stale_pdst[0] != '0) &&
                        (preg_alloc_reg[twa_i] == commit_stale_pdst[0])) begin
                        $fatal(1, "rv32i_ss_free_list: alloc/return collision (slot 0 stale)");
                    end
                    if (commit_rd_fire[1] && (commit_stale_pdst[1] != '0) &&
                        (preg_alloc_reg[twa_i] == commit_stale_pdst[1])) begin
                        $fatal(1, "rv32i_ss_free_list: alloc/return collision (slot 1 stale)");
                    end
                end
            end

            // An older trap and a younger branch recovery can coincide. Full flush
            // has priority: restoring committed_free_bits_q also supersedes the
            // younger branch's reclaim, because the trap squashes that branch.
        end
    end
    /* verilator lint_on SYNCASYNCNET */
`endif

endmodule
