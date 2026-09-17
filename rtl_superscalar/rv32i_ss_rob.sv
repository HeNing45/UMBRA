// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

import rv32i_ss_pkg::OOO_ROB_BITS;
import rv32i_ss_pkg::OOO_ROB_DEPTH;
import rv32i_ss_pkg::arch_reg_t;
import rv32i_ss_pkg::phys_reg_t;
import rv32i_ss_pkg::rob_idx_t;
import rv32i_ss_pkg::rob_seq_t;
import rv32i_ss_pkg::word_t;
import rv32i_ss_pkg::commit_order_t;
import rv32i_ss_pkg::csr_addr_t;   // ROB stashes the committing CSR's addr

module rv32i_ss_rob (
    input  logic      clk,
    input  logic      rst_n,

    // Dispatch allocates 0-2 ROB entries at tail.
    input  logic       bundle_fire,
    input  logic [1:0] rob_alloc_slot_valid,
    input  logic [1:0] bundle_size,
    output logic       rob_alloc_ready,
    input  word_t [1:0]     rob_alloc_pc,
    input  word_t [1:0]     rob_alloc_instr,
    input  logic [1:0]      rob_alloc_rd_we,
    input  arch_reg_t [1:0] rob_alloc_rd,
    input  phys_reg_t [1:0] rob_alloc_pdst,
    input  phys_reg_t [1:0] rob_alloc_stale_pdst,
    input  logic      rob_alloc_trap_valid,   // decode-detected trap (ecall/ebreak)
    input  word_t     rob_alloc_trap_cause,
    input  word_t     rob_alloc_trap_tval,
    input  logic      rob_alloc_is_mret,
    output rob_idx_t [1:0] rob_alloc_idx,
    output rob_seq_t [1:0] rob_alloc_seq,
    input  logic       rob_alloc_is_csr,
    input  logic [1:0] rob_alloc_is_store,
    input  logic [1:0] rob_alloc_is_load,
    input  csr_addr_t  rob_alloc_csr_addr,

    // Writeback marks 0-2 existing ROB entries complete.
    input  logic [1:0]     wb_valid,
    input  rob_idx_t [1:0] wb_rob_idx,
    input  rob_seq_t [1:0] wb_rob_seq,

    input  word_t [1:0]    wb_result,
    input  logic [1:0]     wb_csr_we,
    input  word_t [1:0]    wb_csr_wdata,
    output logic [1:0]     wb_accept,
    input  logic [1:0]     wb_trap_valid,
    input  word_t [1:0]    wb_trap_cause,
    input  word_t [1:0]    wb_trap_tval,

    // Commit exposes the two oldest rows ({head, head+1}) as per-slot facts;
    // the core owns the fire decision and the ROB consumes it.
    output logic [1:0]     commit_valid,
    input  logic [1:0]     commit_fire,
    output word_t [1:0]    commit_pc,
    output word_t [1:0]    commit_instr,
    output logic [1:0]     commit_rd_we,
    output arch_reg_t [1:0] commit_rd,
    output phys_reg_t [1:0] commit_pdst,
    output phys_reg_t [1:0] commit_stale_pdst,
    output word_t [1:0]    commit_result,
    output logic [1:0]     commit_is_csr,
    output logic [1:0]     commit_is_store,
    output logic [1:0]     commit_is_load,
    output logic [1:0]     commit_csr_we,
    output csr_addr_t [1:0] commit_csr_addr,
    output word_t [1:0]    commit_csr_wdata,

    //branch and trap_flush
    input logic trap_flush,
    input logic branch_recover_req,
    input rob_idx_t recover_rob_idx,

    // Head/age facts (the IQ + CDB age reference and the liveness
    // boundary) plus the commit-trace taps.
    output rob_idx_t rob_head_idx,
    output rob_seq_t rob_head_seq,
    output logic rob_head_valid,
    output logic rob_head_done,
    output commit_order_t commit_order,
    output logic [1:0]  commit_trap_valid,
    output word_t [1:0] commit_trap_cause,
    output word_t [1:0] commit_trap_tval,
    output logic [1:0]  commit_is_mret
);

    // geometry guard (elaboration-time): every age/kill compare in this
    // machine is rob_idx_t subtraction, i.e. mod 2**OOO_ROB_BITS -- which
    // equals ring distance mod OOO_ROB_DEPTH ONLY for a power-of-two depth.
    // Any other depth silently corrupts every age predicate. Fatal here so
    // the build dies loudly instead.
    if (OOO_ROB_DEPTH != (1 << $clog2(OOO_ROB_DEPTH))) begin : g_rob_pow2_guard
        $fatal(1, "rv32i_ss_rob: OOO_ROB_DEPTH must be a power of two (ring-distance arithmetic)");
    end

    localparam int ROB_COUNT_BITS = OOO_ROB_BITS + 1;
    typedef logic [ROB_COUNT_BITS-1:0] rob_count_t;

    localparam rob_count_t ROB_DEPTH_COUNT = rob_count_t'(OOO_ROB_DEPTH);

    logic      valid_q      [OOO_ROB_DEPTH];
    logic      done_q       [OOO_ROB_DEPTH];
    word_t     pc_q         [OOO_ROB_DEPTH];
    word_t     instr_q      [OOO_ROB_DEPTH];
    logic      rd_we_q      [OOO_ROB_DEPTH];
    arch_reg_t rd_q         [OOO_ROB_DEPTH];
    phys_reg_t pdst_q       [OOO_ROB_DEPTH];
    phys_reg_t stale_pdst_q [OOO_ROB_DEPTH];
    word_t     result_q     [OOO_ROB_DEPTH];
    rob_seq_t  seq_q        [OOO_ROB_DEPTH];
    logic      trap_valid_q [OOO_ROB_DEPTH];   // per-entry trap (decode-detected)
    word_t     trap_cause_q [OOO_ROB_DEPTH];
    word_t     trap_tval_q  [OOO_ROB_DEPTH];
    logic      is_mret_q    [OOO_ROB_DEPTH];
    logic      is_csr_q     [OOO_ROB_DEPTH];
    csr_addr_t csr_addr_q   [OOO_ROB_DEPTH];
    logic      csr_we_q     [OOO_ROB_DEPTH];
    word_t     csr_wdata_q  [OOO_ROB_DEPTH];
    logic      is_store_q   [OOO_ROB_DEPTH];
    logic      is_load_q    [OOO_ROB_DEPTH];

    commit_order_t commit_order_q;
    rob_seq_t next_seq_q;

    rob_idx_t head_q;
    rob_idx_t tail_q;
    rob_count_t count_q;
    logic [1:0] commit_count;
    rob_idx_t recover_tail;
    rob_count_t recover_count;
    rob_idx_t recover_age;

    logic [1:0] wb_match;

    integer rob_i;
    integer wb_i;
    integer wb_apply_i;
    integer commit_i;         // combinational export loop ONLY
    integer commit_apply_i;   // sequential retire loop ONLY (never share one
                              // loop variable across two procedural blocks --
                              // same reason wb_i and wb_apply_i are separate)

    assign commit_count = {1'b0, commit_fire[0]} + {1'b0, commit_fire[1]};

    assign rob_alloc_ready  = ((ROB_DEPTH_COUNT - count_q) >= rob_count_t'(bundle_size));
    assign rob_alloc_idx[0] = tail_q;
    assign rob_alloc_idx[1] = tail_q + rob_idx_t'(1);
    assign rob_alloc_seq[0] = next_seq_q;
    assign rob_alloc_seq[1] = next_seq_q + rob_seq_t'(1);

    always_comb begin
        wb_match  = '0;
        wb_accept = '0;

        for (wb_i = 0; wb_i < 2; wb_i++) begin
            wb_match[wb_i] =
                wb_valid[wb_i] &&
                valid_q[wb_rob_idx[wb_i]] &&
                !done_q[wb_rob_idx[wb_i]] &&
                (seq_q[wb_rob_idx[wb_i]] == wb_rob_seq[wb_i]);

            wb_accept[wb_i] =
                wb_match[wb_i] &&
                !trap_flush &&
                (!branch_recover_req ||
                ((wb_rob_idx[wb_i] - head_q) <= recover_age));
        end
    end

    assign recover_tail = recover_rob_idx + 1'b1;
    assign recover_count = rob_count_t'({1'b0, (recover_rob_idx - head_q)})
                            + rob_count_t'(1'b1);
    assign recover_age = recover_rob_idx - head_q;

    // Commit slots read head and head+1. Facts are qualified by valid && done;
    // raw payloads require qualification by the consuming commit event. Trap and
    // mret facts likewise wait for completion, and the core acts on them only
    // when their instruction is the oldest entry.
    always_comb begin
        rob_idx_t row;
        for (commit_i = 0; commit_i < 2; commit_i++) begin
            row = head_q + rob_idx_t'(commit_i);

            commit_valid[commit_i]      = valid_q[row] && done_q[row];
            commit_trap_valid[commit_i] = commit_valid[commit_i] && trap_valid_q[row];
            commit_is_mret[commit_i]    = commit_valid[commit_i] && is_mret_q[row];
            commit_is_csr[commit_i]     = commit_valid[commit_i] && is_csr_q[row];
            commit_is_store[commit_i]   = commit_valid[commit_i] && is_store_q[row];
            commit_is_load[commit_i]    = commit_valid[commit_i] && is_load_q[row];

            commit_pc[commit_i]         = pc_q[row];
            commit_instr[commit_i]      = instr_q[row];
            commit_rd_we[commit_i]      = rd_we_q[row];
            commit_rd[commit_i]         = rd_q[row];
            commit_pdst[commit_i]       = pdst_q[row];
            commit_stale_pdst[commit_i] = stale_pdst_q[row];
            commit_result[commit_i]     = result_q[row];
            commit_csr_we[commit_i]     = csr_we_q[row];
            commit_csr_addr[commit_i]   = csr_addr_q[row];
            commit_csr_wdata[commit_i]  = csr_wdata_q[row];
            commit_trap_cause[commit_i] = trap_cause_q[row];
            commit_trap_tval[commit_i]  = trap_tval_q[row];
        end
    end

    assign rob_head_idx   = head_q;
    assign rob_head_seq   = seq_q[head_q];
    assign rob_head_valid = valid_q[head_q];
    assign rob_head_done  = done_q[head_q];
    // commit_order is a single retirement counter. Slot p's trace order is
    // commit_order + p under ordered-prefix retirement.
    assign commit_order = commit_order_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (rob_i = 0; rob_i < OOO_ROB_DEPTH; rob_i = rob_i + 1) begin
                valid_q[rob_i]      <= 1'b0;
                done_q[rob_i]       <= 1'b0;
                pc_q[rob_i]         <= '0;
                instr_q[rob_i]      <= '0;
                rd_we_q[rob_i]      <= 1'b0;
                rd_q[rob_i]         <= '0;
                pdst_q[rob_i]       <= '0;
                stale_pdst_q[rob_i] <= '0;
                result_q[rob_i]     <= '0;
                seq_q[rob_i]        <= '0;
                trap_valid_q[rob_i] <= 1'b0;
                trap_cause_q[rob_i] <= '0;
                trap_tval_q[rob_i]  <= '0;
                is_mret_q[rob_i]    <= 1'b0;
                is_csr_q[rob_i]     <= 1'b0;
                is_store_q[rob_i]   <= 1'b0;
                is_load_q[rob_i]    <= 1'b0;
                csr_addr_q[rob_i]   <= '0;
                csr_we_q[rob_i]     <= 1'b0;
                csr_wdata_q[rob_i]  <= '0;
            end
            head_q  <= '0;
            tail_q  <= '0;
            count_q <= '0;
            commit_order_q <= '0;
            next_seq_q     <= '0;
        end else if (trap_flush) begin
            for (rob_i = 0; rob_i < OOO_ROB_DEPTH; rob_i = rob_i + 1) begin
                valid_q[rob_i]      <= 1'b0;
                done_q[rob_i]       <= 1'b0;
                pc_q[rob_i]         <= '0;
                instr_q[rob_i]      <= '0;
                rd_we_q[rob_i]      <= 1'b0;
                rd_q[rob_i]         <= '0;
                pdst_q[rob_i]       <= '0;
                stale_pdst_q[rob_i] <= '0;
                result_q[rob_i]     <= '0;
                seq_q[rob_i]        <= '0;
                trap_valid_q[rob_i] <= 1'b0;
                trap_cause_q[rob_i] <= '0;
                trap_tval_q[rob_i]  <= '0;
                is_mret_q[rob_i]    <= 1'b0;
                is_csr_q[rob_i]     <= 1'b0;
                is_store_q[rob_i]   <= 1'b0;
                is_load_q[rob_i]    <= 1'b0;
                csr_addr_q[rob_i]   <= '0;
                csr_we_q[rob_i]     <= 1'b0;
                csr_wdata_q[rob_i]  <= '0;
            end
            head_q  <= '0;
            tail_q  <= '0;
            count_q <= '0;
        end else if (branch_recover_req) begin
            for (rob_i = 0; rob_i < OOO_ROB_DEPTH; rob_i = rob_i + 1) begin
                if (valid_q[rob_i] &&
                    ((rob_idx_t'(rob_i) - head_q) > recover_age)) begin
                    valid_q[rob_i]      <= 1'b0;
                    done_q[rob_i]       <= 1'b0;
                    pc_q[rob_i]         <= '0;
                    instr_q[rob_i]      <= '0;
                    rd_we_q[rob_i]      <= 1'b0;
                    rd_q[rob_i]         <= '0;
                    pdst_q[rob_i]       <= '0;
                    stale_pdst_q[rob_i] <= '0;
                    result_q[rob_i]     <= '0;
                    seq_q[rob_i]        <= '0;
                    trap_valid_q[rob_i] <= 1'b0;
                    trap_cause_q[rob_i] <= '0;
                    trap_tval_q[rob_i]  <= '0;
                    is_mret_q[rob_i]    <= 1'b0;
                    is_csr_q[rob_i]     <= 1'b0;
                    is_store_q[rob_i]   <= 1'b0;
                    is_load_q[rob_i]    <= 1'b0;
                    csr_addr_q[rob_i]   <= '0;
                    csr_we_q[rob_i]     <= 1'b0;
                    csr_wdata_q[rob_i]  <= '0;
                end
            end
            tail_q  <= recover_tail;
            count_q <= recover_count;
            // a writeback landing on the recovery cycle must
            // still complete a SURVIVING entry. The CDB beat is fire-and-forget,
            // so dropping it would strand the older entry not-done forever. The
            // wrong-path target (younger than the branch) is being killed this
            // same cycle, so its writeback is correctly ignored: wb_accept
            // carries the age qualification, and it is the SAME verdict the
            // core uses for the PRF write and ready-table set.
            for (wb_apply_i = 0; wb_apply_i < 2; wb_apply_i++) begin
                if (wb_accept[wb_apply_i]) begin
                    result_q[wb_rob_idx[wb_apply_i]] <= wb_result[wb_apply_i];
                    done_q[wb_rob_idx[wb_apply_i]]   <= 1'b1;
                    csr_we_q[wb_rob_idx[wb_apply_i]]
                        <= wb_csr_we[wb_apply_i];
                    csr_wdata_q[wb_rob_idx[wb_apply_i]]
                        <= wb_csr_wdata[wb_apply_i];
                    if (wb_trap_valid[wb_apply_i]) begin
                        trap_valid_q[wb_rob_idx[wb_apply_i]] <= 1'b1;
                        trap_cause_q[wb_rob_idx[wb_apply_i]]
                            <= wb_trap_cause[wb_apply_i];
                        trap_tval_q[wb_rob_idx[wb_apply_i]]
                            <= wb_trap_tval[wb_apply_i];
                    end
                end
            end
        end else begin
            for (wb_apply_i = 0; wb_apply_i < 2; wb_apply_i++) begin
                if (wb_accept[wb_apply_i]) begin
                    result_q[wb_rob_idx[wb_apply_i]] <= wb_result[wb_apply_i];
                    done_q[wb_rob_idx[wb_apply_i]]   <= 1'b1;
                    csr_we_q[wb_rob_idx[wb_apply_i]]
                        <= wb_csr_we[wb_apply_i];
                    csr_wdata_q[wb_rob_idx[wb_apply_i]]
                        <= wb_csr_wdata[wb_apply_i];
                    if (wb_trap_valid[wb_apply_i]) begin
                        trap_valid_q[wb_rob_idx[wb_apply_i]] <= 1'b1;
                        trap_cause_q[wb_rob_idx[wb_apply_i]]
                            <= wb_trap_cause[wb_apply_i];
                        trap_tval_q[wb_rob_idx[wb_apply_i]]
                            <= wb_trap_tval[wb_apply_i];
                    end
                end
            end

            // retire the ORDERED PREFIX of fired slots. Each fired slot
            // clears its own row at head + slot, and head/order advance by the
            // popcount, so zero, one, or two entries retire on one edge. The
            // casts are type-exact per slot: head is a ROB index, commit_order
            // is the 64-bit architectural counter -- they are different types
            // carrying different meanings and must not share a cast.
            for (commit_apply_i = 0; commit_apply_i < 2; commit_apply_i++) begin
                if (commit_fire[commit_apply_i]) begin
                    valid_q[head_q + rob_idx_t'(commit_apply_i)] <= 1'b0;
                    done_q[head_q + rob_idx_t'(commit_apply_i)]  <= 1'b0;
                end
            end
            if (|commit_fire) begin
                head_q          <= head_q + rob_idx_t'(commit_count);
                commit_order_q  <= commit_order_q + commit_order_t'(commit_count);
            end

            if (bundle_fire) begin
                for (rob_i = 0; rob_i < 2; rob_i ++) begin
                    if (rob_alloc_slot_valid[rob_i]) begin
                        valid_q[rob_alloc_idx[rob_i]]      <= 1'b1;
                        done_q[rob_alloc_idx[rob_i]]       <= 1'b0;
                        pc_q[rob_alloc_idx[rob_i]]         <= rob_alloc_pc[rob_i];
                        instr_q[rob_alloc_idx[rob_i]]      <= rob_alloc_instr[rob_i];
                        rd_we_q[rob_alloc_idx[rob_i]]      <= rob_alloc_rd_we[rob_i];
                        rd_q[rob_alloc_idx[rob_i]]         <= rob_alloc_rd[rob_i];
                        pdst_q[rob_alloc_idx[rob_i]]       <= rob_alloc_pdst[rob_i];
                        stale_pdst_q[rob_alloc_idx[rob_i]] <= rob_alloc_stale_pdst[rob_i];
                        result_q[rob_alloc_idx[rob_i]]     <= '0;
                        seq_q[rob_alloc_idx[rob_i]]        <= rob_alloc_seq[rob_i];
                        csr_we_q[rob_alloc_idx[rob_i]]     <= 1'b0;
                        csr_wdata_q[rob_alloc_idx[rob_i]]  <= '0;
                        is_store_q[rob_alloc_idx[rob_i]]   <= rob_alloc_is_store[rob_i];
                        is_load_q[rob_alloc_idx[rob_i]]    <= rob_alloc_is_load[rob_i];

                        // Trap/CSR/mret bundles are formation-guaranteed SOLO,
                        // so their scalar payload is routed only to slot 0.
                        if (rob_i == 0) begin
                            trap_valid_q[rob_alloc_idx[rob_i]] <= rob_alloc_trap_valid;
                            trap_cause_q[rob_alloc_idx[rob_i]] <= rob_alloc_trap_cause;
                            trap_tval_q[rob_alloc_idx[rob_i]]  <= rob_alloc_trap_tval;
                            is_mret_q[rob_alloc_idx[rob_i]]    <= rob_alloc_is_mret;
                            is_csr_q[rob_alloc_idx[rob_i]]     <= rob_alloc_is_csr;
                            csr_addr_q[rob_alloc_idx[rob_i]]   <= rob_alloc_csr_addr;
                        end else begin
                            trap_valid_q[rob_alloc_idx[rob_i]] <= 1'b0;
                            trap_cause_q[rob_alloc_idx[rob_i]] <= '0;
                            trap_tval_q[rob_alloc_idx[rob_i]]  <= '0;
                            is_mret_q[rob_alloc_idx[rob_i]]    <= 1'b0;
                            is_csr_q[rob_alloc_idx[rob_i]]     <= 1'b0;
                            csr_addr_q[rob_alloc_idx[rob_i]]   <= '0;
                        end
                    end
                end
                tail_q     <= tail_q     + rob_idx_t'(bundle_size);
                next_seq_q <= next_seq_q + rob_seq_t'(bundle_size);
            end

            count_q <= count_q + (bundle_fire ? rob_count_t'(bundle_size) : rob_count_t'(0))
                         - rob_count_t'(commit_count);
        end
    end


`ifndef SYNTHESIS
    /* verilator lint_off SYNCASYNCNET */
    // Allocation comes from the core's bundle event. Every fired bundle
    // must fit the free ROB capacity advertised by rob_alloc_ready.
    always @(posedge clk) begin
        if (rst_n === 1'b1) begin
            if (bundle_fire && !rob_alloc_ready) begin
                $fatal(1, "rv32i_ss_rob: bundle_fire with ROB full (fire-honesty)");
            end

            // bundle_size is CARRIED beside the shape, not derived —
            // pin the consistency so a future producer refactor cannot skew
            // tail/count advancement against the per-slot writes.
            if (bundle_fire &&
                (bundle_size != ({1'b0, rob_alloc_slot_valid[0]} +
                                 {1'b0, rob_alloc_slot_valid[1]}))) begin
                $fatal(1, "rv32i_ss_rob: bundle_size != popcount(rob_alloc_slot_valid)");
            end

            // The core owns commit_fire. Every fired slot must name a valid, completed
            // ROB row; otherwise architectural consumers could advance prematurely.
            if (commit_fire[0] && !commit_valid[0]) begin
                $fatal(1, "rv32i_ss_rob: commit_fire[0] without commit_valid[0]");
            end
            if (commit_fire[1] && !commit_valid[1]) begin
                $fatal(1, "rv32i_ss_rob: commit_fire[1] without commit_valid[1]");
            end

            // Retirement is an ordered prefix: slot 1 requires slot 0 to fire.
            if (commit_fire[1] && !commit_fire[0]) begin
                $fatal(1, "rv32i_ss_rob: commit_fire[1] without commit_fire[0]");
            end

            // The two wb lanes must never accept one ROB entry in
            // the same cycle — both accepts on one idx means both lanes
            // matched the same live {rob_idx, rob_seq} generation, i.e. an
            // exact duplicate was in flight on two transport lanes.
            if (wb_accept[0] && wb_accept[1] &&
                (wb_rob_idx[0] == wb_rob_idx[1])) begin
                $fatal(1, "rv32i_ss_rob: dual writeback accepted one ROB entry");
            end
        end
    end
    /* verilator lint_on SYNCASYNCNET */
`endif

endmodule
