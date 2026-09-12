`timescale 1ns/1ps

import rv32i_ooo_pkg::OOO_ARCH_REGS;
import rv32i_ooo_pkg::OOO_PHYS_REGS;
import rv32i_ooo_pkg::OOO_BRANCH_CKPTS;
import rv32i_ooo_pkg::arch_reg_t;
import rv32i_ooo_pkg::phys_reg_t;
import rv32i_ooo_pkg::ckpt_idx_t;
import rv32i_ooo_pkg::branch_mask_t;

module rv32i_ooo_rename (
    input  logic      clk,
    input  logic      rst_n,

    // Decoded architectural register names.
    input  logic      decoded_valid,
    output logic      decoded_ready,
    input  arch_reg_t decoded_rs1,
    input  arch_reg_t decoded_rs2,
    input  arch_reg_t decoded_rd,
    input  logic      decoded_rd_we,
    input  logic      decoded_needs_checkpoint,

    // Free-list allocation side.
    output logic      preg_alloc_req,
    input  logic      preg_avail,
    input  phys_reg_t preg_alloc_reg,

    // Renamed physical register names.
    output logic      rename_valid,
    input  logic      rename_ready,
    output phys_reg_t rename_prs1,
    output phys_reg_t rename_prs2,
    output phys_reg_t rename_pdst,
    output phys_reg_t rename_stale_pdst,
    output arch_reg_t rename_rd,
    output logic      rename_rd_we,

    // ROB commit updates the committed architectural map.
    input  logic      commit_fire,
    input  logic      commit_rd_we,
    input  arch_reg_t commit_rd,
    input  phys_reg_t commit_pdst,

    // Full precise flush restores speculative map from committed map.
    input  logic      trap_flush,

    //branch recover
    input  logic                     branch_recover_req,
    input  ckpt_idx_t                branch_recover_id,

    output logic [OOO_PHYS_REGS-1:0] branch_recover_alloc_list,
    output branch_mask_t rename_branch_mask,
    output logic rename_checkpoint_valid,
    output ckpt_idx_t rename_checkpoint_id,

    //branch resolve
    input logic branch_resolve_valid,
    input ckpt_idx_t branch_resolve_id

);
    //spec and committed states
    phys_reg_t spec_map_q      [OOO_ARCH_REGS];
    phys_reg_t spec_map_d      [OOO_ARCH_REGS];
    phys_reg_t committed_map_q [OOO_ARCH_REGS];
    //checkpoint states
    logic checkpoint_valid_q [OOO_BRANCH_CKPTS];
    logic checkpoint_valid_d [OOO_BRANCH_CKPTS];
    phys_reg_t checkpoint_map_q [OOO_BRANCH_CKPTS][OOO_ARCH_REGS];
    phys_reg_t checkpoint_map_d [OOO_BRANCH_CKPTS][OOO_ARCH_REGS];
    logic [OOO_PHYS_REGS-1:0] checkpoint_alloc_list_q [OOO_BRANCH_CKPTS];
    logic [OOO_PHYS_REGS-1:0] checkpoint_alloc_list_d [OOO_BRANCH_CKPTS];
    //mask states
    branch_mask_t checkpoint_older_mask_q [OOO_BRANCH_CKPTS];
    branch_mask_t checkpoint_older_mask_d [OOO_BRANCH_CKPTS];



    logic needs_alloc;
    logic rename_fire;
    logic commit_rd_fire;
    logic found_free_ckpt;
    logic checkpoint_available;
    logic checkpoint_ok;
    integer reset_i;
    integer ckpt_i;
    integer avail_ckpt_i;
    integer arch_i;
    integer ff_ckpt_i;
    integer ff_arch_i;

    assign needs_alloc = decoded_rd_we && (decoded_rd != '0);
    assign checkpoint_ok = !decoded_needs_checkpoint || checkpoint_available;
    assign decoded_ready = rename_ready && (!needs_alloc || preg_avail) &&
                        checkpoint_ok;
    assign rename_valid = decoded_valid && (!needs_alloc || preg_avail) &&
                        checkpoint_ok;
    assign preg_alloc_req = decoded_valid && rename_ready && needs_alloc && checkpoint_ok;

    assign rename_fire = decoded_valid && decoded_ready;
    assign commit_rd_fire = commit_fire && commit_rd_we && (commit_rd != '0);

    //compute checkpoint availability
    always_comb begin
        checkpoint_available = 1'b0;
        for (avail_ckpt_i = 0; avail_ckpt_i < OOO_BRANCH_CKPTS; avail_ckpt_i++) begin
            if (!checkpoint_valid_q[avail_ckpt_i]) begin
                checkpoint_available = 1'b1;
            end
        end
    //compute the branch mask = the set of outstanding (valid) checkpoints.
    //driven straight onto the rename output; reused as a new checkpoint's older_mask.
        rename_branch_mask = '0;
        for (avail_ckpt_i = 0; avail_ckpt_i < OOO_BRANCH_CKPTS; avail_ckpt_i++) begin
            if (checkpoint_valid_q[avail_ckpt_i] &&
                !(branch_resolve_valid &&
                  (rv32i_ooo_pkg::ckpt_idx_t'(avail_ckpt_i) == branch_resolve_id))) begin
                rename_branch_mask[avail_ckpt_i] = 1'b1;
            end
        end
    end

    // branch_recover_alloc_list lives in its OWN always_comb on purpose. If it is
    // produced inside the big rename block below -- which also CONSUMES preg_alloc_reg
    // (<- free_list) -- then since free_list's allocator block consumes this list and
    // produces preg_alloc_reg, the two blocks form a cycle at always_comb granularity.
    // It is broken bit-wise by free_bits_q (so Verilator levelizes it, UNOPTFLAT-clean),
    // but Icarus evaluates each block atomically and oscillates -> zero-time hang on any
    // recovery. Isolating this output (reads only registered checkpoint state) cuts it.
    always_comb begin
        if (branch_recover_req && !trap_flush)
            branch_recover_alloc_list = checkpoint_alloc_list_q[branch_recover_id];
        else
            branch_recover_alloc_list = '0;
    end

    always_comb begin
        // Default next state holds current state. Use loops for Icarus unpacked-array support.
        // rename_branch_mask is driven by the branch-mask block above.
        for (arch_i = 0; arch_i < OOO_ARCH_REGS; arch_i++) begin
            spec_map_d[arch_i] = spec_map_q[arch_i];
        end
        rename_prs1 = spec_map_q[decoded_rs1];
        rename_prs2 = spec_map_q[decoded_rs2];

        //checkpoint relay
        for (ckpt_i = 0; ckpt_i < OOO_BRANCH_CKPTS; ckpt_i++) begin
            checkpoint_valid_d[ckpt_i] = checkpoint_valid_q[ckpt_i];
            checkpoint_alloc_list_d[ckpt_i] = checkpoint_alloc_list_q[ckpt_i];
            checkpoint_older_mask_d[ckpt_i] = checkpoint_older_mask_q[ckpt_i];
            for (arch_i = 0; arch_i < OOO_ARCH_REGS; arch_i++) begin
                checkpoint_map_d[ckpt_i][arch_i] = checkpoint_map_q[ckpt_i][arch_i];
            end
        end

        rename_checkpoint_id = '0;
        rename_checkpoint_valid = '0;
        found_free_ckpt = 1'b0;
        if (needs_alloc) begin
            rename_pdst = preg_alloc_reg;
            rename_stale_pdst = spec_map_q[decoded_rd];
        end else begin
            rename_pdst = '0;
            rename_stale_pdst = '0;
        end
        rename_rd = decoded_rd;
        rename_rd_we = needs_alloc;

        if (trap_flush) begin
            for (arch_i = 0; arch_i < OOO_ARCH_REGS; arch_i++) begin
                spec_map_d[arch_i] = committed_map_q[arch_i];
            end
            for (ckpt_i = 0; ckpt_i < OOO_BRANCH_CKPTS; ckpt_i++) begin
                checkpoint_valid_d[ckpt_i] = 1'b0;
                checkpoint_alloc_list_d[ckpt_i] = '0;
                checkpoint_older_mask_d[ckpt_i] = '0;
            end
        end else if (branch_recover_req) begin
            for (arch_i = 0; arch_i < OOO_ARCH_REGS; arch_i++) begin
                spec_map_d[arch_i] = checkpoint_map_q[branch_recover_id][arch_i];
            end

            for (ckpt_i = 0; ckpt_i < OOO_BRANCH_CKPTS; ckpt_i++) begin
                if ((rv32i_ooo_pkg::ckpt_idx_t'(ckpt_i) == branch_recover_id) ||
                    checkpoint_older_mask_q[ckpt_i][branch_recover_id]) begin
                    checkpoint_valid_d[ckpt_i] = 1'b0;
                    checkpoint_alloc_list_d[ckpt_i] = '0;
                    checkpoint_older_mask_d[ckpt_i] = '0;
                end
            end
        end else begin
            //resolve branch
            if (branch_resolve_valid) begin
                // Correctly resolved branch leaves the speculation set.
                checkpoint_valid_d[branch_resolve_id] = 1'b0;
                checkpoint_alloc_list_d[branch_resolve_id] = '0;
                checkpoint_older_mask_d[branch_resolve_id] = '0;
                for (ckpt_i = 0; ckpt_i < OOO_BRANCH_CKPTS; ckpt_i++) begin
                    checkpoint_older_mask_d[ckpt_i][branch_resolve_id] = 1'b0;
                end
            end
            //set checkpoint alloc list
            if (rename_fire && needs_alloc) begin
                spec_map_d[decoded_rd] = preg_alloc_reg;
                for (ckpt_i = 0; ckpt_i < OOO_BRANCH_CKPTS; ckpt_i++) begin
                    if (checkpoint_valid_d[ckpt_i]) begin
                        checkpoint_alloc_list_d[ckpt_i][preg_alloc_reg] = 1'b1;
                    end
                end
            end
            //normal rename fire updates checkpoint
            if (decoded_needs_checkpoint && rename_fire && checkpoint_available) begin
                for (ckpt_i = 0; ckpt_i < OOO_BRANCH_CKPTS; ckpt_i ++) begin
                    if (!checkpoint_valid_q[ckpt_i] && !found_free_ckpt) begin
                        checkpoint_valid_d[ckpt_i] = 1'b1;
                        checkpoint_alloc_list_d[ckpt_i] = '0;
                        checkpoint_older_mask_d[ckpt_i] = rename_branch_mask;
                        for (arch_i = 0; arch_i < OOO_ARCH_REGS; arch_i++) begin
                            checkpoint_map_d[ckpt_i][arch_i] = spec_map_d[arch_i];
                        end
                        rename_checkpoint_id = rv32i_ooo_pkg::ckpt_idx_t'(ckpt_i);
                        rename_checkpoint_valid = 1'b1;
                        found_free_ckpt = 1'b1;
                    end
                end
            end
        end

    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Clear architectural maps back to the direct xN -> pN mapping.
            for (reset_i = 0; reset_i < OOO_ARCH_REGS; reset_i++) begin
                spec_map_q[reset_i]      <= phys_reg_t'(reset_i);
                committed_map_q[reset_i] <= phys_reg_t'(reset_i);
            end

            // Only valid needs reset; checkpoint payload arrays are valid-gated.
            for (ff_ckpt_i = 0; ff_ckpt_i < OOO_BRANCH_CKPTS; ff_ckpt_i++) begin
                checkpoint_valid_q[ff_ckpt_i] <= 1'b0;
            end

        end else begin
            if (commit_rd_fire) begin
                committed_map_q[commit_rd] <= commit_pdst;
            end

            for (ff_arch_i = 0; ff_arch_i < OOO_ARCH_REGS; ff_arch_i++) begin
                spec_map_q[ff_arch_i] <= spec_map_d[ff_arch_i];
            end

            for (ff_ckpt_i = 0; ff_ckpt_i < OOO_BRANCH_CKPTS; ff_ckpt_i++) begin
                checkpoint_valid_q[ff_ckpt_i] <= checkpoint_valid_d[ff_ckpt_i];
                checkpoint_alloc_list_q[ff_ckpt_i] <= checkpoint_alloc_list_d[ff_ckpt_i];
                checkpoint_older_mask_q[ff_ckpt_i] <= checkpoint_older_mask_d[ff_ckpt_i];

                for (ff_arch_i = 0; ff_arch_i < OOO_ARCH_REGS; ff_arch_i++) begin
                    checkpoint_map_q[ff_ckpt_i][ff_arch_i] <=
                        checkpoint_map_d[ff_ckpt_i][ff_arch_i];
                end
            end
        end
    end

`ifndef SYNTHESIS
    /* verilator lint_off SYNCASYNCNET */
    always @(posedge clk) begin
        if (rst_n === 1'b1) begin
            if ((spec_map_q[0] != '0) || (committed_map_q[0] != '0)) begin
                $fatal(1, "rv32i_ooo_rename: x0 must always map to p0");
            end

            if (rename_fire && needs_alloc && (preg_alloc_reg == '0)) begin
                $fatal(1, "rv32i_ooo_rename: allocated p0 for real destination");
            end

            if (commit_rd_fire && trap_flush) begin
                $fatal(1, "rv32i_ooo_rename: commit map update and flush co-asserted");
            end

            if (branch_resolve_valid && branch_recover_req) begin
                $fatal(1, "rv32i_ooo_rename: branch resolve and recover co-asserted");
            end

            if (branch_resolve_valid && !checkpoint_valid_q[branch_resolve_id]) begin
                $fatal(1, "rv32i_ooo_rename: resolved invalid branch checkpoint");
            end

            if (branch_recover_req && !checkpoint_valid_q[branch_recover_id]) begin
                $fatal(1, "rv32i_ooo_rename: recovered invalid branch checkpoint");
            end
        end
    end
    /* verilator lint_on SYNCASYNCNET */
`endif

endmodule
