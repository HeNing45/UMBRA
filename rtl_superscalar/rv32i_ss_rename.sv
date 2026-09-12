`timescale 1ns/1ps

import rv32i_ss_pkg::OOO_ARCH_REGS;
import rv32i_ss_pkg::OOO_PHYS_REGS;
import rv32i_ss_pkg::OOO_BRANCH_CKPTS;
import rv32i_ss_pkg::arch_reg_t;
import rv32i_ss_pkg::phys_reg_t;
import rv32i_ss_pkg::ckpt_idx_t;
import rv32i_ss_pkg::branch_mask_t;

module rv32i_ss_rename (
    input  logic      clk,
    input  logic      rst_n,

    // Decoded architectural register names. decoded_valid is the scalar
    // bundle-offer level; decoded_slot_valid is the bundle-shape payload
    // (2'b01 one-instruction, 2'b11 two; 2'b10 unreachable — slot 1
    // cannot exist without older slot 0; invariant tripwires below).
    input  logic      decoded_valid,
    input  logic [1:0] decoded_slot_valid,
    output logic      decoded_ready,
    input  arch_reg_t [1:0] decoded_rs1,
    input  arch_reg_t [1:0] decoded_rs2,
    input  arch_reg_t [1:0] decoded_rd,
    input  logic      [1:0] decoded_rd_we,
    input  logic      [1:0] decoded_needs_checkpoint,

    // Free-list allocation side.
    output logic [1:0]      preg_slot_need,
    input  logic      preg_avail,
    input  phys_reg_t [1:0] preg_alloc_reg,

    // Renamed physical register names.
    input  logic      rename_ready,
    input  logic      bundle_fire,
    output phys_reg_t [1:0] rename_prs1,
    output phys_reg_t [1:0] rename_prs2,
    output phys_reg_t [1:0] rename_pdst,
    output phys_reg_t [1:0] rename_stale_pdst,
    output arch_reg_t [1:0] rename_rd,
    output logic      [1:0] rename_rd_we,

    // ROB commit updates the committed architectural map.
    input  logic [1:0]     commit_fire,
    input  logic [1:0]     commit_rd_we,
    input  arch_reg_t [1:0] commit_rd,
    input  phys_reg_t [1:0] commit_pdst,

    // Full precise flush restores speculative map from committed map.
    input  logic      trap_flush,

    //branch recover
    input  logic                     branch_recover_req,
    input  ckpt_idx_t                branch_recover_id,

    output logic [OOO_PHYS_REGS-1:0] rename_branch_recover_alloc_list,
    output branch_mask_t rename_branch_mask,
    output logic rename_checkpoint_valid,
    output ckpt_idx_t rename_checkpoint_id,

    // Checkpoint release set: the branch combiner may release 0-2 rows/cycle.
    input branch_mask_t checkpoint_release_mask

);
    // geometry guard (elaboration-time): OOO_ARCH_REGS is ISA-FIXED, not
    // a knob -- RV32I's 5-bit register fields, x0 semantics, and this map's
    // indexing all assume exactly 32 architectural registers.
    if (OOO_ARCH_REGS != 32) begin : g_arch_regs_guard
        $fatal(1, "rv32i_ss_rename: OOO_ARCH_REGS is ISA-fixed at 32 (RV32I)");
    end

    //speculative and committed map state
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



    logic [1:0] needs_alloc;
    logic [1:0] commit_rd_fire;
    logic found_free_ckpt;
    logic checkpoint_available;
    logic checkpoint_ok;
    integer reset_i;
    integer ckpt_i;
    integer rel_i;
    integer tw_i;
    integer tw_j;
    integer avail_ckpt_i;
    integer arch_i;
    integer ff_ckpt_i;
    integer ff_arch_i;

    assign checkpoint_ok = (!(|decoded_needs_checkpoint)) || checkpoint_available;
    assign decoded_ready = rename_ready && ((needs_alloc == 2'b00) || preg_avail)
                        && checkpoint_ok;
    // Pure demand: no ready/checkpoint/fire term is embedded —
    // slots are accepted together only on the fire edge; a held bundle
    // presents demand and pops nothing.
    assign preg_slot_need = needs_alloc;

    assign commit_rd_fire[0] = commit_fire[0] && commit_rd_we[0] && (commit_rd[0] != '0);
    assign commit_rd_fire[1] = commit_fire[1] && commit_rd_we[1] && (commit_rd[1] != '0);

    assign needs_alloc[0] = decoded_slot_valid[0] && decoded_rd_we[0] && (decoded_rd[0] != '0);
    assign needs_alloc[1] = decoded_slot_valid[1] && decoded_rd_we[1] && (decoded_rd[1] != '0);

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
                !checkpoint_release_mask[avail_ckpt_i]) begin
                rename_branch_mask[avail_ckpt_i] = 1'b1;
            end
        end
    end

    // rename_branch_recover_alloc_list lives in its OWN always_comb on purpose. If it is
    // produced inside the big rename block below -- which also CONSUMES preg_alloc_reg
    // (<- free_list) -- then since free_list's allocator block consumes this list and
    // produces preg_alloc_reg, the two blocks form a cycle at always_comb granularity.
    // It is broken bit-wise by free_bits_q (so Verilator levelizes it, UNOPTFLAT-clean),
    // but Icarus evaluates each block atomically and oscillates -> zero-time hang on any
    // recovery. Isolating this output (reads only registered checkpoint state) cuts it.
    always_comb begin
        if (branch_recover_req && !trap_flush)
            rename_branch_recover_alloc_list = checkpoint_alloc_list_q[branch_recover_id];
        else
            rename_branch_recover_alloc_list = '0;
    end

    // Build the per-slot rename payload. Destination mappings are produced
    // before the slot-1 RAW/WAW bypasses consume slot 0's new mapping.
    always_comb begin
        if (needs_alloc[0]) begin
            rename_pdst[0] = preg_alloc_reg[0];
        end else begin
            rename_pdst[0] = '0;
        end

        if (needs_alloc[1]) begin
            rename_pdst[1] = preg_alloc_reg[1];
        end else begin
            rename_pdst[1] = '0;
        end

        if (needs_alloc[0]) begin
            rename_stale_pdst[0] = spec_map_q[decoded_rd[0]];
        end else begin
            rename_stale_pdst[0] = '0;
        end

        // Rename WAW: stale1 = (rd1 == rd0 && rd0_we) ? pdst0 : map[rd1].
        if (needs_alloc[1]) begin
            rename_stale_pdst[1] = (decoded_rd[1] == decoded_rd[0]
                     && decoded_rd_we[0]) ? rename_pdst[0] : spec_map_q[decoded_rd[1]];
        end else begin
            rename_stale_pdst[1] = '0;
        end

        rename_prs1[0] = spec_map_q[decoded_rs1[0]];
        rename_prs2[0] = spec_map_q[decoded_rs2[0]];
        rename_prs1[1] = (decoded_rs1[1] == decoded_rd[0] && decoded_rd_we[0])
                        ? rename_pdst[0] : spec_map_q[decoded_rs1[1]];
        rename_prs2[1] = (decoded_rs2[1] == decoded_rd[0] && decoded_rd_we[0])
                        ? rename_pdst[0] : spec_map_q[decoded_rs2[1]];

        rename_rd[0] = decoded_rd[0];
        rename_rd[1] = decoded_rd[1];
        rename_rd_we[0] = needs_alloc[0];
        rename_rd_we[1] = needs_alloc[1];
    end

    always_comb begin
        // Default next state holds current state. Use loops for Icarus unpacked-array support.
        // rename_branch_mask is driven by the branch-mask block above.
        for (arch_i = 0; arch_i < OOO_ARCH_REGS; arch_i++) begin
            spec_map_d[arch_i] = spec_map_q[arch_i];
        end

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
                if ((rv32i_ss_pkg::ckpt_idx_t'(ckpt_i) == branch_recover_id) ||
                    checkpoint_older_mask_q[ckpt_i][branch_recover_id]) begin
                    checkpoint_valid_d[ckpt_i] = 1'b0;
                    checkpoint_alloc_list_d[ckpt_i] = '0;
                    checkpoint_older_mask_d[ckpt_i] = '0;
                end
            end
        end else begin
            //release set: every masked row leaves the speculation set
            for (rel_i = 0; rel_i < OOO_BRANCH_CKPTS; rel_i++) begin
                if (checkpoint_release_mask[rel_i]) begin
                    checkpoint_valid_d[rel_i] = 1'b0;
                    checkpoint_alloc_list_d[rel_i] = '0;
                    checkpoint_older_mask_d[rel_i] = '0;
                    for (ckpt_i = 0; ckpt_i < OOO_BRANCH_CKPTS; ckpt_i++) begin
                        checkpoint_older_mask_d[ckpt_i][rel_i] = 1'b0;
                    end
                end
            end
            //set checkpoint alloc list
            if (bundle_fire) begin
                if (needs_alloc[0]) begin
                    spec_map_d[decoded_rd[0]] = preg_alloc_reg[0];
                end
                if (needs_alloc[1]) begin
                    spec_map_d[decoded_rd[1]] = preg_alloc_reg[1];
                end

                for (ckpt_i = 0; ckpt_i < OOO_BRANCH_CKPTS; ckpt_i++) begin
                    if (checkpoint_valid_d[ckpt_i]) begin
                        if (needs_alloc[0]) begin
                            checkpoint_alloc_list_d[ckpt_i][preg_alloc_reg[0]] = 1'b1;
                        end
                        if (needs_alloc[1]) begin
                            checkpoint_alloc_list_d[ckpt_i][preg_alloc_reg[1]] = 1'b1;
                        end
                    end
                end
            end
            //normal rename fire updates checkpoint
            if ((|decoded_needs_checkpoint) && bundle_fire && checkpoint_available) begin
                for (ckpt_i = 0; ckpt_i < OOO_BRANCH_CKPTS; ckpt_i ++) begin
                    if (!checkpoint_valid_q[ckpt_i] && !found_free_ckpt) begin
                        checkpoint_valid_d[ckpt_i] = 1'b1;
                        checkpoint_alloc_list_d[ckpt_i] = '0;
                        checkpoint_older_mask_d[ckpt_i] = rename_branch_mask;
                        // Slot 0's checkpoint will reclaim slot 1's allocated preg.
                        if (decoded_needs_checkpoint[0] && needs_alloc[1]) begin
                            checkpoint_alloc_list_d[ckpt_i][preg_alloc_reg[1]] = 1'b1;
                        end

                        for (arch_i = 0; arch_i < OOO_ARCH_REGS; arch_i++) begin
                            checkpoint_map_d[ckpt_i][arch_i] = spec_map_q[arch_i];
                        end
                        if (decoded_needs_checkpoint[1] && needs_alloc[0]) begin
                            checkpoint_map_d[ckpt_i][decoded_rd[0]] = preg_alloc_reg[0];
                        end
                        rename_checkpoint_id = rv32i_ss_pkg::ckpt_idx_t'(ckpt_i);
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
            //sequential overwrite to bypass commit stale
            if (commit_rd_fire[0]) begin
                committed_map_q[commit_rd[0]] <= commit_pdst[0];
            end
            if (commit_rd_fire[1]) begin
                committed_map_q[commit_rd[1]] <= commit_pdst[1];
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
                $fatal(1, "rv32i_ss_rename: x0 must always map to p0");
            end

            if (bundle_fire && (|needs_alloc) && (preg_alloc_reg == '0)) begin
                $fatal(1, "rv32i_ss_rename: allocated p0 for real destination");
            end

            // Checkpoint creation assumes its instruction allocates no destination.
            // Conditional branches and predicted return forms satisfy this;
            // the snapshot and reclaim-list initialization rely on that fact.
            if (bundle_fire &&
                ((decoded_needs_checkpoint[0] && needs_alloc[0]) ||
                 (decoded_needs_checkpoint[1] && needs_alloc[1]))) begin
                $fatal(1, "rv32i_ss_rename: checkpointing slot allocates a dest");
            end

            // Bundle-shape invariants:
            // the offer level and the shape payload must agree, and slot 1
            // cannot exist without older slot 0.
            if (decoded_valid != (|decoded_slot_valid)) begin
                $fatal(1, "rv32i_ss_rename: bundle-offer level disagrees with slot shape");
            end

            if (decoded_slot_valid[1] && !decoded_slot_valid[0]) begin
                $fatal(1, "rv32i_ss_rename: slot 1 offered without older slot 0");
            end

            // Payload-cleanliness pin the checkpoint logic relies on: an
            // INVALID slot must not carry a checkpoint request (the create
            // gate and arm mux consume decoded_needs_checkpoint raw).
            if (decoded_valid &&
                (|(decoded_needs_checkpoint & ~decoded_slot_valid))) begin
                $fatal(1, "rv32i_ss_rename: checkpoint request from an invalid slot");
            end

            // ANY committing slot during a full flush is the error, so
            // reduce explicitly -- a bare 2-bit vector in a logical AND is an
            // implicit non-zero test and lints as a width truncation.
            if ((|commit_rd_fire) && trap_flush) begin
                $fatal(1, "rv32i_ss_rename: commit map update and flush co-asserted");
            end

            if ((|checkpoint_release_mask) && (branch_recover_req || trap_flush)) begin
                $fatal(1, "rv32i_ss_rename: release set nonzero on a broadcast cycle");
            end

            for (tw_i = 0; tw_i < OOO_BRANCH_CKPTS; tw_i++) begin
                if (checkpoint_release_mask[tw_i] && !checkpoint_valid_q[tw_i]) begin
                    $fatal(1, "rv32i_ss_rename: released an invalid branch checkpoint");
                end
            end

            // Checkpoint nesting: a younger
            // checkpoint's alloc list is a subset of every older live
            // checkpoint's list — allocations after the younger's creation
            // land in both, and the older additionally carries the span
            // between the two creations. Covering-recovery idempotence
            // rests on this.
            for (tw_i = 0; tw_i < OOO_BRANCH_CKPTS; tw_i++) begin
                for (tw_j = 0; tw_j < OOO_BRANCH_CKPTS; tw_j++) begin
                    if (checkpoint_valid_q[tw_i] && checkpoint_valid_q[tw_j] &&
                        checkpoint_older_mask_q[tw_j][tw_i] &&
                        (|(checkpoint_alloc_list_q[tw_j] &
                           ~checkpoint_alloc_list_q[tw_i]))) begin
                        $fatal(1, "rv32i_ss_rename: nesting lemma violated (younger list not a subset)");
                    end
                end
            end

            if (branch_recover_req && !checkpoint_valid_q[branch_recover_id]) begin
                $fatal(1, "rv32i_ss_rename: recovered invalid branch checkpoint");
            end
        end
    end
    /* verilator lint_on SYNCASYNCNET */
`endif

endmodule
