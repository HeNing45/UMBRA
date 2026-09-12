`timescale 1ns/1ps

import rv32i_ooo_pkg::OOO_ROB_BITS;
import rv32i_ooo_pkg::OOO_ROB_DEPTH;
import rv32i_ooo_pkg::arch_reg_t;
import rv32i_ooo_pkg::phys_reg_t;
import rv32i_ooo_pkg::rob_idx_t;
import rv32i_ooo_pkg::rob_seq_t;
import rv32i_ooo_pkg::word_t;
import rv32i_ooo_pkg::commit_order_t;
import rv32i_ooo_pkg::csr_addr_t;   // ROB retains the committing CSR's address.

module rv32i_ooo_rob (
    input  logic      clk,
    input  logic      rst_n,

    // Dispatch allocates a new ROB entry at tail.
    input  logic      rob_alloc_valid,
    output logic      rob_alloc_ready,
    input  word_t     rob_alloc_pc,
    input  word_t     rob_alloc_instr,
    input  logic      rob_alloc_rd_we,
    input  arch_reg_t rob_alloc_rd,
    input  phys_reg_t rob_alloc_pdst,
    input  phys_reg_t rob_alloc_stale_pdst,
    input  logic      rob_alloc_trap_valid,   // decode-detected trap
    input  word_t     rob_alloc_trap_cause,
    input  word_t     rob_alloc_trap_tval,
    input  logic      rob_alloc_is_mret,
    output rob_idx_t  rob_alloc_idx,
    output rob_seq_t  rob_alloc_seq,
    input  logic       rob_alloc_is_csr,
    input  logic       rob_alloc_is_store,
    input  csr_addr_t  rob_alloc_csr_addr,

    // Writeback marks an existing ROB entry complete.
    input  logic     wb_valid,
    input  rob_idx_t wb_rob_idx,
    input  rob_seq_t wb_rob_seq,
    input  word_t    wb_result,
    input  logic     wb_csr_we,
    input  word_t    wb_csr_wdata,
    output logic     wb_accept,
    input logic  wb_trap_valid,
    input word_t wb_trap_cause,
    input word_t wb_trap_tval,

    // Commit exposes the oldest complete entry at head.
    output logic      commit_valid,
    input  logic      commit_ready,
    output word_t     commit_pc,
    output word_t     commit_instr,
    output logic      commit_rd_we,
    output arch_reg_t commit_rd,
    output phys_reg_t commit_pdst,
    output phys_reg_t commit_stale_pdst,
    output word_t     commit_result,
    output logic      commit_is_csr,
    output logic      commit_is_store,
    output logic      commit_csr_we,
    output csr_addr_t commit_csr_addr,
    output word_t     commit_csr_wdata,

    //branch and trap_flush
    input logic trap_flush,
    input logic branch_recover_req,
    input rob_idx_t recover_rob_idx,

    //traces
    output rob_idx_t rob_head_idx,
    output rob_seq_t rob_head_seq,
    output logic rob_head_valid,
    output logic rob_head_done,
    output commit_order_t commit_order,
    output logic commit_trap_valid,
    output word_t commit_trap_cause,
    output word_t commit_trap_tval,
    output logic commit_is_mret
);

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
    logic      trap_valid_q [OOO_ROB_DEPTH];   // per-entry trap validity
    word_t     trap_cause_q [OOO_ROB_DEPTH];
    word_t     trap_tval_q  [OOO_ROB_DEPTH];
    logic      is_mret_q    [OOO_ROB_DEPTH];
    logic      is_csr_q     [OOO_ROB_DEPTH];
    csr_addr_t csr_addr_q   [OOO_ROB_DEPTH];
    logic      csr_we_q     [OOO_ROB_DEPTH];
    word_t     csr_wdata_q  [OOO_ROB_DEPTH];
    logic      is_store_q   [OOO_ROB_DEPTH];

    commit_order_t commit_order_q;
    rob_seq_t next_seq_q;

    rob_idx_t head_q;
    rob_idx_t tail_q;
    rob_count_t count_q;
    rob_idx_t recover_tail;
    rob_count_t recover_count;
    rob_idx_t recover_age;

    logic rob_alloc_fire;
    logic wb_fire;
    logic commit_fire;

    integer rob_i;

    assign rob_alloc_ready   = (count_q < ROB_DEPTH_COUNT);
    assign rob_alloc_fire    = rob_alloc_valid && rob_alloc_ready;
    assign rob_alloc_idx = tail_q;
    assign rob_alloc_seq = next_seq_q;

    assign wb_fire = wb_valid && valid_q[wb_rob_idx] && !done_q[wb_rob_idx] &&
                     (seq_q[wb_rob_idx] == wb_rob_seq);
    assign wb_accept = wb_fire;

    assign recover_tail = recover_rob_idx + 1'b1;
    assign recover_count = rob_count_t'({1'b0, (recover_rob_idx - head_q)})
                            + rob_count_t'(1'b1);
    assign recover_age = recover_rob_idx - head_q;

    assign commit_valid      = valid_q[head_q] && done_q[head_q];
    assign commit_fire       = commit_valid && commit_ready;
    assign commit_rd_we      = rd_we_q[head_q];
    assign commit_pc         = pc_q[head_q];
    assign commit_instr      = instr_q[head_q];
    assign commit_pdst       = pdst_q[head_q];
    assign commit_stale_pdst = stale_pdst_q[head_q];
    assign commit_rd         = rd_q[head_q];
    assign commit_result     = result_q[head_q];
    assign commit_is_csr     = commit_valid && is_csr_q[head_q];
    assign commit_is_store   = commit_valid && is_store_q[head_q];
    assign commit_csr_we     = csr_we_q[head_q];
    assign commit_csr_addr   = csr_addr_q[head_q];
    assign commit_csr_wdata  = csr_wdata_q[head_q];

    assign rob_head_idx   = head_q;
    assign rob_head_seq   = seq_q[head_q];
    assign rob_head_valid = valid_q[head_q];
    assign rob_head_done  = done_q[head_q];
    assign commit_order = commit_order_q;
    // Expose the head's trap, gated by commit_valid (valid && done) -- the
    // SAME complete-head boundary as a normal commit. Trap ops are ALU-class:
    // they execute through the ALU and get done, so this fires only when the
    // trapping uop is the oldest AND complete. The core then registers its flush.
    assign commit_trap_valid = commit_valid && trap_valid_q[head_q];
    assign commit_trap_cause = trap_cause_q[head_q];
    assign commit_trap_tval  = trap_tval_q[head_q];
    assign commit_is_mret    = commit_valid && is_mret_q[head_q];

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
                    is_store_q[rob_i]     <= 1'b0;
                    csr_addr_q[rob_i]   <= '0;
                    csr_we_q[rob_i]     <= 1'b0;
                    csr_wdata_q[rob_i]  <= '0;
                end
            end
            tail_q  <= recover_tail;
            count_q <= recover_count;
            // A writeback landing on the recovery cycle must
            // still complete a SURVIVING entry. The CDB beat is fire-and-forget,
            // so dropping it would strand the older entry not-done forever. The
            // wrong-path target (younger than the branch) is being killed this
            // same cycle, so its writeback is correctly ignored (guard fails).
            if (wb_fire && ((wb_rob_idx - head_q) <= recover_age)) begin
                result_q[wb_rob_idx] <= wb_result;
                done_q[wb_rob_idx]   <= 1'b1;
                csr_we_q[wb_rob_idx]    <= wb_csr_we;
                csr_wdata_q[wb_rob_idx] <= wb_csr_wdata;
                if (wb_trap_valid) begin
                    trap_valid_q[wb_rob_idx] <= 1'b1;
                    trap_cause_q[wb_rob_idx] <= wb_trap_cause;
                    trap_tval_q[wb_rob_idx]  <= wb_trap_tval;
                end
            end
        end else begin
            if (wb_fire) begin
                result_q[wb_rob_idx] <= wb_result;
                done_q[wb_rob_idx]   <= 1'b1;
                csr_we_q[wb_rob_idx]    <= wb_csr_we;
                csr_wdata_q[wb_rob_idx] <= wb_csr_wdata;
                if (wb_trap_valid) begin
                    trap_valid_q[wb_rob_idx] <= 1'b1;
                    trap_cause_q[wb_rob_idx] <= wb_trap_cause;
                    trap_tval_q[wb_rob_idx]  <= wb_trap_tval;
                end
            end

            if (commit_fire) begin
                valid_q[head_q] <= 1'b0;
                done_q[head_q]  <= 1'b0;
                head_q          <= head_q + 1'b1;
                commit_order_q  <= commit_order_q + 64'd1;
            end

            if (rob_alloc_fire) begin
                valid_q[tail_q]      <= 1'b1;
                done_q[tail_q]       <= 1'b0;
                pc_q[tail_q]         <= rob_alloc_pc;
                instr_q[tail_q]      <= rob_alloc_instr;
                rd_we_q[tail_q]      <= rob_alloc_rd_we;
                rd_q[tail_q]         <= rob_alloc_rd;
                pdst_q[tail_q]       <= rob_alloc_pdst;
                stale_pdst_q[tail_q] <= rob_alloc_stale_pdst;
                result_q[tail_q]     <= '0;
                csr_we_q[tail_q]     <= 1'b0;
                csr_wdata_q[tail_q]  <= '0;
                seq_q[tail_q]        <= next_seq_q;
                trap_valid_q[tail_q] <= rob_alloc_trap_valid;
                trap_cause_q[tail_q] <= rob_alloc_trap_cause;
                trap_tval_q[tail_q]  <= rob_alloc_trap_tval;
                is_mret_q[tail_q]    <= rob_alloc_is_mret;
                next_seq_q           <= next_seq_q + 64'd1;
                tail_q               <= tail_q + 1'b1;
                is_csr_q[tail_q]     <= rob_alloc_is_csr;
                is_store_q[tail_q]   <= rob_alloc_is_store;
                csr_addr_q[tail_q]   <= rob_alloc_csr_addr;
            end

            case ({rob_alloc_fire, commit_fire})
                2'b10: count_q <= count_q + 1'b1;
                2'b01: count_q <= count_q - 1'b1;
                default: count_q <= count_q;
            endcase
        end
    end

endmodule
