`timescale 1ns/1ps

import rv32i_ooo_pkg::OOO_ARCH_REGS;
import rv32i_ooo_pkg::OOO_PHYS_REGS;
import rv32i_ooo_pkg::phys_reg_t;

module rv32i_ooo_free_list (
    input  logic                     clk,
    input  logic                     rst_n,

    // Allocation side: rename asks for one new destination physreg.
    input  logic                     preg_alloc_req,
    output logic                     preg_avail,
    output phys_reg_t                preg_alloc_reg,

    // Commit side: ROB retires one architectural destination update.
    input  logic                     commit_fire,
    input  logic                     commit_rd_we,
    input  phys_reg_t                commit_pdst,
    input  phys_reg_t                commit_stale_pdst,

    // Branch recovery: return all physical regs allocated on the wrong path.
    input  logic                     branch_recover_req,
    input  logic [OOO_PHYS_REGS-1:0] branch_recover_alloc_list,

    // Full precise flush: restore the committed free-list image.
    input  logic                     trap_flush
);

    typedef logic [OOO_PHYS_REGS-1:0] free_bits_t;

    free_bits_t free_bits_q;
    free_bits_t free_bits_d;
    free_bits_t committed_free_bits_q;
    free_bits_t committed_free_bits_d;

    logic commit_rd_fire;

    integer reset_i;
    integer free_i;
    logic found_alloc;

    assign commit_rd_fire = commit_fire && commit_rd_we;

    always_comb begin
        free_bits_d           = free_bits_q;
        committed_free_bits_d = committed_free_bits_q;
        preg_avail           = 1'b0;
        preg_alloc_reg         = '0;
        found_alloc           = 1'b0;

        for (free_i = 1; free_i < OOO_PHYS_REGS; free_i++) begin
            if (free_bits_q[free_i] && !found_alloc) begin
                preg_avail   = 1'b1;
                preg_alloc_reg = phys_reg_t'(free_i);
                found_alloc   = 1'b1;
            end
        end

        if (preg_alloc_req && preg_avail) begin
            free_bits_d[preg_alloc_reg] = 1'b0;
        end

        if (commit_rd_fire) begin
            if (commit_pdst != '0) begin
                free_bits_d[commit_pdst]           = 1'b0;
                committed_free_bits_d[commit_pdst] = 1'b0;
            end

            if (commit_stale_pdst != '0) begin
                free_bits_d[commit_stale_pdst]           = 1'b1;
                committed_free_bits_d[commit_stale_pdst] = 1'b1;
            end
        end

        if (branch_recover_req) begin
            free_bits_d = free_bits_d | branch_recover_alloc_list;
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
    always @(posedge clk) begin
        if (rst_n === 1'b1) begin
            if (free_bits_q[0] || committed_free_bits_q[0]) begin
                $fatal(1, "rv32i_ooo_free_list: p0 must never be free");
            end

            if (preg_alloc_req && preg_avail) begin
                if (preg_alloc_reg == '0) begin
                    $fatal(1, "rv32i_ooo_free_list: allocated p0");
                end
                if (!free_bits_q[preg_alloc_reg]) begin
                    $fatal(1, "rv32i_ooo_free_list: preg_alloc_reg was not free");
                end
            end

            if (commit_rd_fire && (commit_pdst != '0) && free_bits_q[commit_pdst]) begin
                $fatal(1, "rv32i_ooo_free_list: committing pdst was already free");
            end

            if (commit_rd_fire && (commit_stale_pdst != '0) &&
                free_bits_q[commit_stale_pdst]) begin
                $fatal(1, "rv32i_ooo_free_list: double-free of stale_pdst");
            end

            // branch_recover_req and trap_flush CAN legally co-assert
            // (an older trap reaches the head the same cycle a younger branch
            // mispredicts). The full flush wins -- committed_free_bits_q is written
            // last (above), squashing the branch's reclaim too, which is correct
            // since the trap squashes the branch as well.
        end
    end
    /* verilator lint_on SYNCASYNCNET */
`endif

endmodule
