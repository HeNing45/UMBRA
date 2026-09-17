// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

import fyp_cpu_pkg::mem_size_e;
import rv32i_ss_pkg::SS_LQ_DEPTH;
import rv32i_ss_pkg::SS_SQ_DEPTH;
import rv32i_ss_pkg::completion_packet_t;
import rv32i_ss_pkg::lq_entry_t;
import rv32i_ss_pkg::lq_idx_t;
import rv32i_ss_pkg::phys_reg_t;
import rv32i_ss_pkg::rob_idx_t;
import rv32i_ss_pkg::rob_seq_t;
import rv32i_ss_pkg::sq_entry_t;
import rv32i_ss_pkg::sq_idx_t;
import rv32i_ss_pkg::word_t;

// Split-transaction load/store queue with conservative memory ordering and
// full-cover forwarding from the youngest older overlapping store.
//
// Progress depends on the memory environment eventually accepting requests
// and returning read responses. Loads wait only on older stores; older
// operations do not depend on younger results. IQ selection favors older
// ready operations. The oldest store drains from the SQ head, while queue
// fullness blocks dispatch allocation without blocking commit or drain.
module rv32i_ss_lsq (
    input  logic clk,
    input  logic rst_n,

    // Load allocation at dispatch. The returned entry index travels with the
    // IQ entry so the later AGU deposit updates the reserved LQ entry.
    input  logic       lq_alloc_fire,
    output logic       lq_alloc_ready,
    output lq_idx_t    lq_alloc_idx,
    input  rob_idx_t   lq_alloc_rob_idx,
    input  rob_seq_t   lq_alloc_rob_seq,
    input  phys_reg_t  lq_alloc_pdst,
    input  logic       lq_alloc_rd_wen,
    input  mem_size_e  lq_alloc_mem_size,
    input  logic       lq_alloc_mem_unsigned,

    // Store allocation at dispatch. The data-source tag is retained so a
    // store whose base wins first can capture its payload from accepted CDB
    // writeback later.
    input  logic       sq_alloc_fire,
    output logic       sq_alloc_ready,
    output sq_idx_t    sq_alloc_idx,
    input  rob_idx_t   sq_alloc_rob_idx,
    input  rob_seq_t   sq_alloc_rob_seq,
    input  phys_reg_t  sq_alloc_data_preg,

    // AGU deposit into the load entry reserved at dispatch.
    input  logic       lq_deposit_fire,
    input  lq_idx_t    lq_deposit_idx,
    input  word_t      lq_deposit_addr,
    input  logic       lq_deposit_inert,

    // AGU deposit into the store entry reserved at dispatch. Address and byte
    // enables always arrive; data is meaningful only when data_valid is set.
    // deferred_pending records the one case that still needs the SQ completion
    // seam: an aligned store whose data was not ready at address deposit.
    input  logic       sq_deposit_fire,
    input  sq_idx_t    sq_deposit_idx,
    input  word_t      sq_deposit_addr,
    input  word_t      sq_deposit_data,
    input  logic       sq_deposit_data_valid,
    input  logic [3:0] sq_deposit_be,
    input  logic       sq_deposit_inert,
    input  logic       sq_deposit_deferred_pending,

    // Accepted CDB values capture late store data by physical source tag. The
    // fire is ROB-liveness-qualified by the core; stale/killed CDB packets are
    // never allowed to populate a live SQ row.
    input  logic [1:0]      sq_data_wb_fire,
    input  phys_reg_t [1:0] sq_data_wb_pdst,
    input  word_t [1:0]     sq_data_wb_value,

    // A data-late store emits one deferred completion after both halves are
    // present. The packet is held from SQ state until the shared AGU CDB
    // client accepts it.
    output completion_packet_t sq_complete,
    input  logic               sq_complete_accept,

    // The committing store drains from the SQ head. Queue-full state must not
    // participate in this interface's enable path.
    input  logic [1:0]    commit_fire,
    input  logic [1:0]    commit_is_store,
    input  logic [1:0]    commit_is_load,

    // the per-slot commit WANT for a store — the fire
    // equation minus the recovery pulse and minus the acceptance gate — feeds
    // the presentation intent, so the store is presented through a stall.
    // Acceptance and the deferred-acceptance record flow back for the core's
    // commit gate.
    input  logic [1:0]    store_commit_want,
    output logic          sq_mem_accept,
    output logic          sq_accept_deferred,

    // Registered branch recovery kills younger entries by ROB ring distance;
    // a taken precise trap clears both queues.
    input logic     branch_recover_req,
    input rob_idx_t recover_rob_idx,
    input rob_idx_t rob_head_idx,
    input logic     trap_flush,

    // Single data-memory port owned and arbitrated by the LSQ. SQ commit
    // drain has priority over an LQ launch.
    output logic       dmem_valid,
    output logic       dmem_we,
    output logic [3:0] dmem_be,
    output word_t      dmem_addr,
    output word_t      dmem_wdata,
    input  logic       dmem_ready,
    input  word_t      dmem_rdata,
    input  logic        dmem_rvalid,

    // A completed load is one of the five CDB clients. Hold lq_complete.valid until
    // the CDB arbiter grants this client.
    output completion_packet_t lq_complete,
    input  logic               cdb_grant_lq
);

    // geometry guards (elaboration-time): both queues are rings whose
    // head/tail advance wraps in lq_idx_t/sq_idx_t width (mod 2**BITS).
    // That equals mod-DEPTH wrap ONLY for power-of-two depths; anything else
    // walks the pointers off the live window silently.
    if (SS_LQ_DEPTH != (1 << $clog2(SS_LQ_DEPTH))) begin : g_lq_pow2_guard
        $fatal(1, "rv32i_ss_lsq: SS_LQ_DEPTH must be a power of two (ring wrap arithmetic)");
    end
    if (SS_SQ_DEPTH != (1 << $clog2(SS_SQ_DEPTH))) begin : g_sq_pow2_guard
        $fatal(1, "rv32i_ss_lsq: SS_SQ_DEPTH must be a power of two (ring wrap arithmetic)");
    end

    localparam int LQ_COUNT_BITS = $clog2(SS_LQ_DEPTH + 1);
    localparam int SQ_COUNT_BITS = $clog2(SS_SQ_DEPTH + 1);

    typedef logic [LQ_COUNT_BITS-1:0] lq_count_t;
    typedef logic [SQ_COUNT_BITS-1:0] sq_count_t;

    // Circular queue state. The widened counts distinguish empty from full
    // when head_q == tail_q.
    // The executed bit has an independent write enable. Launch must not
    // recirculate an entire selected row (especially rob_seq) just to set it.
    typedef struct packed {
        logic      valid;
        rob_idx_t  rob_idx;
        logic      addr_valid;
        word_t     addr;
        mem_size_e mem_size;
        logic      mem_unsigned;
        phys_reg_t pdst;
        logic      rd_wen;
        rob_seq_t  rob_seq;
        logic      inert;
    } lq_metadata_t;

    lq_metadata_t lq_metadata_q [SS_LQ_DEPTH];
    logic [SS_LQ_DEPTH-1:0] lq_executed_q;
    // Wire-only composite of registered bits, not another register bank.
    // Keep the complete entry view for existing consumers and snapshots.
    wire lq_entry_t lq_entry_q [SS_LQ_DEPTH];

    function automatic lq_metadata_t pack_lq_metadata(input lq_entry_t entry);
        lq_metadata_t metadata;
        metadata.valid        = entry.valid;
        metadata.rob_idx      = entry.rob_idx;
        metadata.addr_valid   = entry.addr_valid;
        metadata.addr         = entry.addr;
        metadata.mem_size     = entry.mem_size;
        metadata.mem_unsigned = entry.mem_unsigned;
        metadata.pdst         = entry.pdst;
        metadata.rd_wen       = entry.rd_wen;
        metadata.rob_seq      = entry.rob_seq;
        metadata.inert        = entry.inert;
        return metadata;
    endfunction

    function automatic lq_entry_t join_lq_entry(
        input lq_metadata_t metadata, input logic executed
    );
        lq_entry_t entry;
        entry.valid        = metadata.valid;
        entry.rob_idx      = metadata.rob_idx;
        entry.addr_valid   = metadata.addr_valid;
        entry.addr         = metadata.addr;
        entry.mem_size     = metadata.mem_size;
        entry.mem_unsigned = metadata.mem_unsigned;
        entry.pdst         = metadata.pdst;
        entry.rd_wen       = metadata.rd_wen;
        entry.rob_seq      = metadata.rob_seq;
        entry.executed     = executed;
        entry.inert        = metadata.inert;
        return entry;
    endfunction

    for (genvar lq_view_i = 0; lq_view_i < SS_LQ_DEPTH; lq_view_i++) begin : g_lq_view
        assign lq_entry_q[lq_view_i] =
            join_lq_entry(lq_metadata_q[lq_view_i], lq_executed_q[lq_view_i]);
    end
    lq_idx_t   lq_head_q;
    lq_idx_t   lq_tail_q;
    lq_count_t lq_count_q;

    sq_entry_t sq_entry_q [SS_SQ_DEPTH];
    sq_idx_t   sq_head_q;
    sq_idx_t   sq_tail_q;
    sq_count_t sq_count_q;

    // Build allocation payloads as whole packed structs. Icarus cannot
    // elaborate a field-select through an indexed unpacked array element.
    lq_entry_t lq_alloc_entry;
    sq_entry_t sq_alloc_entry;
    lq_entry_t lq_head_entry;
    sq_entry_t sq_head_entry;


    // Launch-candidate classification (scan verdicts) and byte enables.
    logic       lq_mem_safe;
    logic [3:0] load_be;
    rob_idx_t   lq_select_age;

    sq_entry_t  sq_order_entry;
    rob_idx_t   sq_order_age;
    logic       sq_order_any_overlap;
    integer     sq_order_scan_i;

    logic       sq_unknown_present;
    logic       sq_overlap_winner_valid;
    rob_idx_t   sq_overlap_winner_age;
    logic [3:0] sq_overlap_winner_be;
    logic       sq_overlap_winner_data_valid;
    word_t      sq_overlap_winner_data;
    sq_idx_t    sq_overlap_winner_idx;

    // Handshake events.
    logic [1:0] lq_pop_fire;
    logic [1:0] lq_pop_count;
    lq_idx_t [1:0] lq_pop_idx;
    logic sq_drain_fire;
    logic [1:0] sq_drain_req;

    integer lq_reset_i;
    integer sq_reset_i;
    integer lq_head_select_i;
    integer sq_head_select_i;
    integer sq_data_wb_i;
    integer sq_data_wb_lane_i;
    integer sq_data_wb_apply_i;
    sq_entry_t sq_data_wb_entry;
    logic [SS_SQ_DEPTH-1:0] sq_data_wb_match [1:0];

    // Store AGU deposit read-modify-write payload.
    sq_entry_t  sq_deposit_entry_next;
    integer     sq_deposit_select_i;
    integer     sq_deposit_wb_lane_i;
    logic [1:0] sq_deposit_wb_match;

    // Deferred store-completion selector. SQ order is program order, so the
    // first eligible row from head is the oldest completion waiting to emit.
    logic               sq_complete_candidate_valid;
    sq_idx_t            sq_complete_candidate_idx;
    sq_idx_t            sq_complete_scan_idx;
    sq_entry_t          sq_complete_candidate_entry;
    sq_entry_t          sq_complete_scan_entry;
    integer             sq_complete_offset_i;
    integer             sq_complete_entry_i;
    completion_packet_t sq_complete_q;
    sq_idx_t            sq_complete_idx_q;

    // Load AGU deposit
    lq_entry_t lq_deposit_entry_next;
    integer    lq_deposit_select_i;

    // Recovery reconstruction stays in queue-index/count space after the ROB
    // age predicate identifies the survivor set.
    lq_idx_t   lq_recover_tail;
    lq_count_t lq_recover_count;
    sq_idx_t   sq_recover_tail;
    sq_count_t sq_recover_count;
    rob_idx_t  recover_age;
    lq_entry_t lq_recover_entry;
    sq_entry_t sq_recover_entry;
    logic [SS_LQ_DEPTH-1:0] lq_recover_kill;
    logic [SS_SQ_DEPTH-1:0] sq_recover_kill;
    integer lq_recover_scan_i;
    integer sq_recover_scan_i;
    integer lq_recover_apply_i;
    integer sq_recover_apply_i;

    // Launch selector output: the oldest QUALIFYING LQ entry
    // (skipping ineligible rows). Ordering safety is judged separately by the
    // SQ scan; this nomination is never itself a fire.
    logic       lq_select_valid;
    lq_idx_t    lq_select_idx;
    lq_entry_t  lq_select_entry;

    lq_idx_t    lq_select_scan_idx;
    lq_entry_t  lq_select_scan_entry;
    integer     lq_select_offset_i;
    integer     lq_select_entry_i;

    // Two completion slots present the head to the CDB client. Read responses
    // are single-cycle events without backpressure, so accepted reads reserve
    // completion storage: outstanding reads plus buffered completions must fit
    // within two slots, including consecutive responses while the CDB stalls.
    completion_packet_t lq_compl_q [0:1];
    logic               lq_compl_head_q;
    logic [1:0]         lq_compl_count_q;
    logic [1:0]         lq_compl_count_after_grant;
    logic               lq_compl_pop;
    logic               lq_compl_tail;
    completion_packet_t lq_compl_new;

    // Accepted LQ memory request.
    logic lq_mem_req_fire;
    logic lq_forward_fire;
    logic lq_complete_set;
    logic lq_complete_ready;

    // A response is meaningful only when paired with either an older
    // outstanding request or the request accepted in this same cycle.
    logic      lq_mem_resp_fire;
    lq_entry_t lq_complete_entry;
    word_t       load_raw_data;
    logic [15:0] load_half_raw;
    logic [7:0]  load_byte_raw;
    word_t       load_result;

    // The outstanding-read FIFO holds two launch-time metadata snapshots.
    // A response consumes the oldest accepted snapshot, never a possibly reused
    // LQ entry. The tail is head ^ count[0]. At count == 1, simultaneous pop
    // and push advance the head and write the other slot.
    lq_entry_t  lq_out_entry_q [0:1];
    lq_idx_t    lq_out_idx_q   [0:1];
    logic       lq_out_head_q;
    logic [1:0] lq_out_count_q;
    logic       lq_out_push;
    logic       lq_out_pop;
    logic       lq_out_tail;
    lq_entry_t  lq_out_push_entry;
    lq_idx_t    lq_out_push_idx;
    lq_entry_t  lq_out_head_entry;

    // Presented requests remain producer-owned until acceptance. Loads hold
    // payload and identity because recovery can kill or replace their selection.
    // Stores need only a held flag: the uncommitted SQ head cannot move, and a
    // presenting store is older than every branch that can recover. Its commit
    // intent therefore remains valid through branch recovery.
    logic       dreq_held_q;
    word_t      dreq_addr_q;
    lq_entry_t  dreq_load_entry_q;
    lq_idx_t    dreq_load_idx_q;
    logic       sq_held_q;
    logic       sq_accept_deferred_q;
    logic       sq_drain_intent;
    logic       lq_req_intent_new;
    logic       lq_launch_held;
    lq_entry_t  unexec_held_entry;
    logic       unexec_held_match;
    logic       lq_held_slot_changes_hands;
    integer     unexec_i;

    // forwarding source selected from the youngest older overlapping SQ
    // entry; post-scan classification permits only a full-cover winner.
    logic     sq_forward_valid;
    sq_idx_t  sq_forward_idx;
    word_t    sq_forward_data;

    // Convert a raw architectural store operand into the byte-lane-aligned
    // bus payload described by the address-derived byte enable already stored
    // in the SQ row.
    function automatic word_t format_store_data(
        input word_t raw_data,
        input logic [3:0] byte_enable
    );
        unique case (byte_enable)
            4'b0001: format_store_data = {24'h000000, raw_data[7:0]};
            4'b0010: format_store_data = {16'h0000, raw_data[7:0], 8'h00};
            4'b0100: format_store_data = {8'h00, raw_data[7:0], 16'h0000};
            4'b1000: format_store_data = {raw_data[7:0], 24'h000000};
            4'b0011: format_store_data = {16'h0000, raw_data[15:0]};
            4'b1100: format_store_data = {raw_data[15:0], 16'h0000};
            default: format_store_data = raw_data;
        endcase
    endfunction

    function automatic sq_entry_t capture_store_data(
        input sq_entry_t entry,
        input word_t raw_data
    );
        sq_entry_t updated;
        updated = entry;
        updated.data_valid = 1'b1;
        updated.data = format_store_data(raw_data, entry.be);
        capture_store_data = updated;
    endfunction

    function automatic sq_entry_t clear_store_deferred_pending(
        input sq_entry_t entry
    );
        sq_entry_t updated;
        updated = entry;
        updated.deferred_pending = 1'b0;
        clear_store_deferred_pending = updated;
    endfunction

    assign lq_alloc_ready = lq_count_q != lq_count_t'(SS_LQ_DEPTH);
    assign lq_alloc_idx = lq_tail_q;
    assign sq_alloc_ready = sq_count_q != sq_count_t'(SS_SQ_DEPTH);
    assign sq_alloc_idx = sq_tail_q;
    assign recover_age = recover_rob_idx - rob_head_idx;
    // Launch admission requires the completion buffer to be empty after this
    // cycle's grant. This is stricter than checking for one free slot and fixes
    // the scheduling policy when a buffered completion is waiting.
    assign lq_compl_pop = cdb_grant_lq && (lq_compl_count_q != 2'd0);
    assign lq_compl_count_after_grant =
        lq_compl_count_q - (lq_compl_pop ? 2'd1 : 2'd0);
    assign lq_complete_ready = (lq_compl_count_after_grant == 2'd0);
    assign lq_compl_tail = lq_compl_head_q ^ lq_compl_count_q[0];
    assign lq_complete = lq_compl_head_q ? lq_compl_q[1] : lq_compl_q[0];

    // Outstanding-FIFO pop consumes the head in acceptance order. A read
    // answered on its own acceptance edge while the FIFO is empty completes
    // directly without creating an outstanding record.
    assign lq_out_head_entry = lq_out_head_q ? lq_out_entry_q[1]
                                             : lq_out_entry_q[0];
    assign lq_out_tail = lq_out_head_q ^ lq_out_count_q[0];
    assign lq_out_pop  = dmem_rvalid && (lq_out_count_q != 2'd0);
    assign lq_out_push = (lq_mem_req_fire || lq_launch_held) &&
                         !(dmem_rvalid && (lq_out_count_q == 2'd0));
    assign lq_out_push_entry = lq_launch_held ? dreq_load_entry_q
                                              : lq_select_entry;
    assign lq_out_push_idx   = lq_launch_held ? dreq_load_idx_q
                                              : lq_select_idx;

    // One accepted producer value may feed several stores. Only rows whose
    // address is already deposited need the snoop: if data arrives first, the
    // PRF ready bit retains it and later operand capture obtains it directly.
    always_comb begin
        sq_data_wb_entry = '0;
        for (sq_data_wb_lane_i = 0; sq_data_wb_lane_i < 2;
             sq_data_wb_lane_i = sq_data_wb_lane_i + 1) begin
            sq_data_wb_match[sq_data_wb_lane_i] = '0;
        end
        for (sq_data_wb_i = 0; sq_data_wb_i < SS_SQ_DEPTH;
             sq_data_wb_i = sq_data_wb_i + 1) begin
            sq_data_wb_entry = sq_entry_q[sq_data_wb_i];
            for (sq_data_wb_lane_i = 0; sq_data_wb_lane_i < 2;
                 sq_data_wb_lane_i = sq_data_wb_lane_i + 1) begin
                sq_data_wb_match[sq_data_wb_lane_i][sq_data_wb_i] =
                    sq_data_wb_fire[sq_data_wb_lane_i] &&
                    sq_data_wb_entry.valid &&
                    sq_data_wb_entry.addr_valid &&
                    !sq_data_wb_entry.data_valid &&
                    sq_data_wb_entry.deferred_pending &&
                    (sq_data_wb_entry.data_preg ==
                     sq_data_wb_pdst[sq_data_wb_lane_i]);
            end
        end
    end

    // Presentation derives from intent without dmem_ready, keeping valid
    // asserted through stalls. New requests pause during recovery; an existing
    // unaccepted request remains stable and carries its original identity.
    // Load and store intents are mutually exclusive, and each yields to the
    // other's held request. Port stability checks enforce this arbitration lock.
    assign sq_drain_intent = (|store_commit_want)     &&
                             !sq_accept_deferred_q    &&
                             !dreq_held_q             &&
                             (sq_held_q || !(branch_recover_req || trap_flush));
    assign lq_req_intent_new = lq_select_valid                &&
                               lq_mem_safe                    &&
                               !sq_drain_intent               &&
                               !sq_held_q                     &&
                               !dreq_held_q                   &&
                               (lq_out_count_q < 2'd2)        &&
                               lq_complete_ready              &&
                               !branch_recover_req            &&
                               !trap_flush;
    assign lq_launch_held  = dreq_held_q && dmem_ready;
    assign lq_mem_req_fire = lq_req_intent_new && dmem_ready;
    assign sq_mem_accept   = sq_drain_intent && dmem_ready;
    assign sq_accept_deferred = sq_accept_deferred_q;

    // Forwarding requires an empty outstanding-read FIFO. It also waits for
    // the completion buffer to drain, preventing a forward/response collision.
    assign lq_forward_fire = lq_select_valid              &&
                             sq_forward_valid             &&
                             (lq_out_count_q == 2'd0)     &&
                             !dreq_held_q                 &&
                             lq_complete_ready            &&
                             !branch_recover_req          &&
                             !trap_flush;

    assign lq_mem_resp_fire = dmem_rvalid &&
                              ((lq_out_count_q != 2'd0) || lq_mem_req_fire ||
                               lq_launch_held);

    // Responses complete with launch-time metadata. The downstream ROB rejects
    // stale identities. Held requests use their own snapshots through this same
    // completion path, including responses arriving after recovery or slot reuse.
    assign lq_complete_set = lq_mem_resp_fire || lq_forward_fire;

    // Select the whole entry first; Icarus cannot elaborate a field-select
    // directly through a variable-indexed unpacked array element.
    always_comb begin
        lq_head_entry = '0;
        for (lq_head_select_i = 0; lq_head_select_i < SS_LQ_DEPTH;
             lq_head_select_i = lq_head_select_i + 1) begin
            if (lq_head_q == lq_idx_t'(lq_head_select_i)) begin
                lq_head_entry = lq_entry_q[lq_head_select_i];
            end
        end

        sq_head_entry = '0;
        for (sq_head_select_i = 0; sq_head_select_i < SS_SQ_DEPTH;
             sq_head_select_i = sq_head_select_i + 1) begin
            if (sq_head_q == sq_idx_t'(sq_head_select_i)) begin
                sq_head_entry = sq_entry_q[sq_head_select_i];
            end
        end

        sq_deposit_entry_next = '0;
        sq_deposit_wb_match = '0;
        for (sq_deposit_select_i = 0;
            sq_deposit_select_i < SS_SQ_DEPTH;
            sq_deposit_select_i = sq_deposit_select_i + 1) begin
            if (sq_deposit_idx == sq_idx_t'(sq_deposit_select_i)) begin
                sq_deposit_entry_next = sq_entry_q[sq_deposit_select_i];
            end
        end
        sq_deposit_entry_next.addr_valid = 1'b1;
        sq_deposit_entry_next.data_valid = sq_deposit_data_valid;
        sq_deposit_entry_next.deferred_pending =
            sq_deposit_deferred_pending;
        sq_deposit_entry_next.addr       = sq_deposit_addr;
        sq_deposit_entry_next.data       = sq_deposit_data_valid
                                         ? sq_deposit_data : '0;
        sq_deposit_entry_next.be         = sq_deposit_be;
        sq_deposit_entry_next.inert      = sq_deposit_inert;

        // Compose the simultaneous-event corner: an address-only AGU deposit
        // and the accepted producer CDB beat may reach the SQ on the same
        // edge. The ordinary row snoop intentionally requires addr_valid in
        // PRE-edge state, so this separate path folds the raw producer value
        // into the newly deposited row instead of losing the one-shot beat.
        for (sq_deposit_wb_lane_i = 0; sq_deposit_wb_lane_i < 2;
             sq_deposit_wb_lane_i = sq_deposit_wb_lane_i + 1) begin
            sq_deposit_wb_match[sq_deposit_wb_lane_i] =
                sq_deposit_fire && !sq_deposit_inert &&
                !sq_deposit_data_valid &&
                sq_data_wb_fire[sq_deposit_wb_lane_i] &&
                sq_deposit_entry_next.valid &&
                (sq_deposit_entry_next.data_preg ==
                 sq_data_wb_pdst[sq_deposit_wb_lane_i]);
            if (sq_deposit_wb_match[sq_deposit_wb_lane_i]) begin
                sq_deposit_entry_next = capture_store_data(
                    sq_deposit_entry_next,
                    sq_data_wb_value[sq_deposit_wb_lane_i]);
            end
        end

        lq_deposit_entry_next = '0;
        for (lq_deposit_select_i = 0;
            lq_deposit_select_i < SS_LQ_DEPTH;
            lq_deposit_select_i = lq_deposit_select_i + 1) begin

            if (lq_deposit_idx == lq_idx_t'(lq_deposit_select_i)) begin
                lq_deposit_entry_next = lq_entry_q[lq_deposit_select_i];
            end
        end
        lq_deposit_entry_next.addr_valid = 1'b1;
        lq_deposit_entry_next.addr       = lq_deposit_addr;
        lq_deposit_entry_next.inert      = lq_deposit_inert;

    end

    // Oldest deferred store completion. A row becomes eligible only after its
    // address and data are both resident. Directly-completed and inert rows
    // never enter this seam.
    always_comb begin
        sq_complete_candidate_valid = 1'b0;
        sq_complete_candidate_idx   = '0;
        sq_complete_candidate_entry = '0;
        sq_complete_scan_idx   = '0;
        sq_complete_scan_entry = '0;

        for (sq_complete_offset_i = 0;
             sq_complete_offset_i < SS_SQ_DEPTH;
             sq_complete_offset_i = sq_complete_offset_i + 1) begin
            sq_complete_scan_idx =
                sq_head_q + sq_idx_t'(sq_complete_offset_i);
            sq_complete_scan_entry = '0;
            for (sq_complete_entry_i = 0;
                 sq_complete_entry_i < SS_SQ_DEPTH;
                 sq_complete_entry_i = sq_complete_entry_i + 1) begin
                if (sq_complete_scan_idx ==
                    sq_idx_t'(sq_complete_entry_i)) begin
                    sq_complete_scan_entry =
                        sq_entry_q[sq_complete_entry_i];
                end
            end

            if (!sq_complete_candidate_valid &&
                sq_complete_scan_entry.valid &&
                sq_complete_scan_entry.addr_valid &&
                sq_complete_scan_entry.data_valid &&
                !sq_complete_scan_entry.inert &&
                sq_complete_scan_entry.deferred_pending) begin
                sq_complete_candidate_valid = 1'b1;
                sq_complete_candidate_idx   = sq_complete_scan_idx;
                sq_complete_candidate_entry = sq_complete_scan_entry;
            end
        end
    end

    assign sq_complete = sq_complete_q;

    // A deferred completion is a real CDB producer, so once offered its
    // packet may not change while arbitration back-pressures it. Register
    // both the packet and the exact SQ row it represents; selecting directly
    // from live queue state would let a newly-ready older store pre-empt a
    // held younger packet.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sq_complete_q     <= '0;
            sq_complete_idx_q <= '0;
        end else if (trap_flush) begin
            sq_complete_q     <= '0;
            sq_complete_idx_q <= '0;
        end else if (branch_recover_req) begin
            if (sq_complete_accept ||
                (sq_complete_q.valid &&
                 !((sq_complete_q.rob_idx - rob_head_idx) < recover_age))) begin
                sq_complete_q     <= '0;
                sq_complete_idx_q <= '0;
            end
        end else if (sq_complete_accept) begin
            sq_complete_q     <= '0;
            sq_complete_idx_q <= '0;
        end else if (!sq_complete_q.valid &&
                     sq_complete_candidate_valid) begin
            sq_complete_q.valid      <= 1'b1;
            sq_complete_q.rob_idx    <= sq_complete_candidate_entry.rob_idx;
            sq_complete_q.rob_seq    <= sq_complete_candidate_entry.rob_seq;
            sq_complete_q.pdst       <= '0;
            sq_complete_q.rd_wen     <= 1'b0;
            sq_complete_q.result     <= sq_complete_candidate_entry.addr;
            sq_complete_q.trap_valid <= 1'b0;
            sq_complete_q.trap_cause <= '0;
            sq_complete_q.trap_tval  <= '0;
            sq_complete_q.csr_we     <= 1'b0;
            sq_complete_q.csr_wdata  <= '0;
            sq_complete_idx_q        <= sq_complete_candidate_idx;
        end
    end

    // determine whether the nominated load may access memory.
    always_comb begin
        // Defaults
        load_be     = 4'b1111;
        lq_select_age  = lq_select_entry.rob_idx - rob_head_idx;
        lq_mem_safe    = 1'b0;
        sq_order_entry = '0;
        sq_order_age   = '0;
        sq_order_any_overlap = 1'b0;

        sq_unknown_present     = 1'b0;
        sq_overlap_winner_valid = 1'b0;
        sq_overlap_winner_age   = '0;
        sq_overlap_winner_be    = '0;
        sq_overlap_winner_data_valid = 1'b0;
        sq_overlap_winner_data  = '0;
        sq_overlap_winner_idx   = '0;

        sq_forward_valid = '0;
        sq_forward_idx   = '0;
        sq_forward_data  = '0;

        // Derive the selected load's byte lanes.
        unique case (lq_select_entry.mem_size)
            fyp_cpu_pkg::MEM_W: begin
                load_be = 4'b1111;
            end

            fyp_cpu_pkg::MEM_H: begin
                load_be = lq_select_entry.addr[1]
                       ? 4'b1100
                       : 4'b0011;
            end

            fyp_cpu_pkg::MEM_B: begin
                unique case (lq_select_entry.addr[1:0])
                    2'b00:   load_be = 4'b0001;
                    2'b01:   load_be = 4'b0010;
                    2'b10:   load_be = 4'b0100;
                    default: load_be = 4'b1000;
                endcase
            end

            default: begin
                load_be = 4'b1111;
            end
        endcase

        // Examine every SQ entry.
        for (sq_order_scan_i = 0;
            sq_order_scan_i < SS_SQ_DEPTH;
            sq_order_scan_i = sq_order_scan_i + 1) begin

            // Icarus-safe whole-entry read.
            sq_order_entry = sq_entry_q[sq_order_scan_i];
            sq_order_age = sq_order_entry.rob_idx - rob_head_idx;

            sq_order_any_overlap = |(sq_order_entry.be & load_be);

            if (lq_select_valid &&
                    sq_order_entry.valid && !sq_order_entry.inert
                    && (sq_order_age < lq_select_age)) begin
                if (!sq_order_entry.addr_valid) begin
                    sq_unknown_present = 1'b1;
                end else if ((sq_order_entry.addr[31:2] ==
                            lq_select_entry.addr[31:2]) &&
                            sq_order_any_overlap) begin
                    if (!sq_overlap_winner_valid ||
                            (sq_order_age > sq_overlap_winner_age)) begin
                        sq_overlap_winner_valid = 1'b1;
                        sq_overlap_winner_age   = sq_order_age;
                        sq_overlap_winner_be    = sq_order_entry.be;
                        sq_overlap_winner_data_valid =
                            sq_order_entry.data_valid;
                        sq_overlap_winner_data  = sq_order_entry.data;
                        sq_overlap_winner_idx   = sq_idx_t'(sq_order_scan_i);
                    end
                end
            end
        end

        // Classify the youngest older overlapping store only after the scan.
        // A partial winner waits; older partial matches are shadowed by a
        // younger full-cover winner. Unknown older addresses always block.
        lq_mem_safe = lq_select_valid &&
                      !sq_unknown_present &&
                      !sq_overlap_winner_valid;
        sq_forward_valid = lq_select_valid &&
                           !sq_unknown_present &&
                           sq_overlap_winner_valid &&
                           sq_overlap_winner_data_valid &&
                           ((sq_overlap_winner_be & load_be) == load_be);
        sq_forward_idx  = sq_overlap_winner_idx;
        sq_forward_data = sq_overlap_winner_data;
    end


    //load launch select
    always_comb begin
        lq_select_valid      = '0;
        lq_select_idx        = '0;
        lq_select_entry      = '0;
        lq_select_scan_idx   = '0;
        lq_select_scan_entry = '0;

        for (lq_select_offset_i = 0;
            lq_select_offset_i < SS_LQ_DEPTH;
            lq_select_offset_i = lq_select_offset_i + 1) begin

            lq_select_scan_idx = lq_head_q + lq_idx_t'(lq_select_offset_i);

            // Icarus-safe whole-entry selection.
            lq_select_scan_entry = '0;
            for (lq_select_entry_i = 0;
                lq_select_entry_i < SS_LQ_DEPTH;
                lq_select_entry_i = lq_select_entry_i + 1) begin
                if (lq_select_scan_idx == lq_idx_t'(lq_select_entry_i)) begin
                    lq_select_scan_entry = lq_entry_q[lq_select_entry_i];
                end
            end

            if (!lq_select_valid &&
                lq_select_scan_entry.valid &&
                lq_select_scan_entry.addr_valid &&
                !lq_select_scan_entry.inert &&
                !lq_select_scan_entry.executed) begin
                lq_select_valid = 1'b1;
                lq_select_idx   = lq_select_scan_idx;
                lq_select_entry = lq_select_scan_entry;
            end
        end
    end


    // Branch-recovery reconstruction
    always_comb begin
        lq_recover_entry = '0;
        lq_recover_kill  = '0;
        lq_recover_count = '0;
        lq_recover_tail  = lq_head_q;

        sq_recover_entry = '0;
        sq_recover_kill  = '0;
        sq_recover_count = '0;
        sq_recover_tail  = sq_head_q;

        // LQ survivor scan. Invariant: KILL iff
        // entry_age > recover_age, i.e. survive iff <=. The `<` below is
        // equivalent because age-EQUALITY is impossible: recover_rob_idx is
        // the branch's own ROB index and a branch is never a mem op, so no
        // queue entry can share it.
        for (lq_recover_scan_i = 0; lq_recover_scan_i < SS_LQ_DEPTH;
             lq_recover_scan_i = lq_recover_scan_i + 1) begin
            // Select the whole entry before field access for Icarus.
            lq_recover_entry = lq_entry_q[lq_recover_scan_i];
            if (lq_recover_entry.valid) begin
                if ((lq_recover_entry.rob_idx - rob_head_idx) < recover_age) begin
                    lq_recover_kill[lq_recover_scan_i] = 1'b0;
                    lq_recover_count = lq_recover_count + 1'b1;
                end else begin
                    lq_recover_kill[lq_recover_scan_i] = 1'b1;
                end
            end


        end

        // SQ survivor scan.
        for (sq_recover_scan_i = 0; sq_recover_scan_i < SS_SQ_DEPTH;
             sq_recover_scan_i = sq_recover_scan_i + 1) begin
            // Select the whole entry before field access for Icarus.
            sq_recover_entry = sq_entry_q[sq_recover_scan_i];
            if (sq_recover_entry.valid) begin
                if ((sq_recover_entry.rob_idx - rob_head_idx) < recover_age) begin
                    sq_recover_kill[sq_recover_scan_i] = 1'b0;
                    sq_recover_count = sq_recover_count + 1'b1;
                end else begin
                    sq_recover_kill[sq_recover_scan_i] = 1'b1;
                end
            end
        end

        lq_recover_tail =
            lq_head_q + lq_idx_t'(lq_recover_count);

        sq_recover_tail =
            sq_head_q + sq_idx_t'(sq_recover_count);
    end
    // command-and-check: the pop/drain enables below are COMMANDS
    // derived only from commit_fire[p] and the committed op class. Queue
    // validity, occupancy, and ROB-index correspondence deliberately do NOT
    // gate them — a mismatch is a fatal contract break (tripwires below),
    // never a silent suppression that would strand the queue head.
    assign lq_pop_fire[0] = commit_fire[0] && commit_is_load[0];
    assign lq_pop_fire[1] = commit_fire[1] && commit_is_load[1];
    assign lq_pop_count   = {1'b0, lq_pop_fire[0]} + {1'b0, lq_pop_fire[1]};
    assign lq_pop_idx[0]  = lq_head_q;
    assign lq_pop_idx[1]  = lq_head_q + lq_idx_t'(lq_pop_fire[0]);
    assign sq_drain_req[0] = commit_fire[0] && commit_is_store[0];
    assign sq_drain_req[1] = commit_fire[1] && commit_is_store[1];
    // sq_drain_fire denotes retirement. The memory write and SQ pop instead
    // occur at acceptance, which may precede retirement across branch recovery.
    assign sq_drain_fire   = sq_drain_req[0] || sq_drain_req[1];

    // Store drain has priority over a new ordering-safe load request. Once
    // either request is held, it retains the port until acceptance; a newly
    // eligible request cannot replace its payload during a stall.
    assign dmem_valid = sq_drain_intent || dreq_held_q || lq_req_intent_new;
    assign dmem_we    = sq_drain_intent;
    assign dmem_addr  = sq_drain_intent ? sq_head_entry.addr
                      : dreq_held_q     ? dreq_addr_q
                                        : lq_select_entry.addr;
    assign dmem_wdata = sq_drain_intent ? sq_head_entry.data : '0;
    assign dmem_be    = sq_drain_intent ? sq_head_entry.be   : 4'b0000;

    // The LQ CDB client sees the completion buffer's HEAD (assigned with the
    // buffer views above); the second slot is invisible to it and drains in
    // order behind the head.

    // Completion metadata follows accepted-request order. With outstanding
    // reads, use the FIFO head regardless of current selection or acceptance.
    // With an empty FIFO, a held launch uses its snapshot; a new launch or
    // forward uses the selected LQ entry. Forwarded data is byte-lane aligned
    // and passes through the same load extractor.
    // Choosing current selection for a held request could attach data to a
    // different live identity, so both response-identity cases are checked.
    always_comb begin
        lq_complete_entry = (lq_out_count_q != 2'd0) ? lq_out_head_entry
                          : lq_launch_held           ? dreq_load_entry_q
                          :                            lq_select_entry;
        load_raw_data = lq_forward_fire
                         ? sq_forward_data
                         : dmem_rdata;

        load_half_raw = lq_complete_entry.addr[1]
                         ? load_raw_data[31:16]
                         : load_raw_data[15:0];

        unique case (lq_complete_entry.addr[1:0])
            2'b00:   load_byte_raw = load_raw_data[7:0];
            2'b01:   load_byte_raw = load_raw_data[15:8];
            2'b10:   load_byte_raw = load_raw_data[23:16];
            default: load_byte_raw = load_raw_data[31:24];
        endcase

        unique case (lq_complete_entry.mem_size)
            fyp_cpu_pkg::MEM_W: begin
                load_result = load_raw_data;
            end
            fyp_cpu_pkg::MEM_H: begin
                load_result = lq_complete_entry.mem_unsigned
                               ? {16'h0000, load_half_raw}
                               : {{16{load_half_raw[15]}},
                                  load_half_raw};
            end
            fyp_cpu_pkg::MEM_B: begin
                load_result = lq_complete_entry.mem_unsigned
                               ? {24'h000000, load_byte_raw}
                               : {{24{load_byte_raw[7]}},
                                  load_byte_raw};
            end
            default: begin
                load_result = load_raw_data;
            end
        endcase
    end

    always_comb begin
        lq_alloc_entry              = '0;
        lq_alloc_entry.valid        = 1'b1;
        lq_alloc_entry.rob_idx      = lq_alloc_rob_idx;
        lq_alloc_entry.rob_seq      = lq_alloc_rob_seq;
        lq_alloc_entry.pdst         = lq_alloc_pdst;
        lq_alloc_entry.rd_wen       = lq_alloc_rd_wen;
        lq_alloc_entry.mem_size     = lq_alloc_mem_size;
        lq_alloc_entry.mem_unsigned = lq_alloc_mem_unsigned;

        sq_alloc_entry         = '0;
        sq_alloc_entry.valid   = 1'b1;
        sq_alloc_entry.rob_idx = sq_alloc_rob_idx;
        sq_alloc_entry.rob_seq = sq_alloc_rob_seq;
        sq_alloc_entry.data_preg = sq_alloc_data_preg;
    end

    // Push each accepted read unless an empty-FIFO request completes on its
    // own edge. A response pops the oldest outstanding snapshot. At count=1,
    // simultaneous pop and push advance the head and write the other slot.
    // Recovery and trap leave these records intact: killed reads still return,
    // and downstream identity checks reject stale completions.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lq_out_entry_q[0] <= '0;
            lq_out_entry_q[1] <= '0;
            lq_out_idx_q[0]   <= '0;
            lq_out_idx_q[1]   <= '0;
            lq_out_head_q     <= 1'b0;
            lq_out_count_q    <= 2'd0;
        end else begin
            if (lq_out_push) begin
                if (lq_out_tail) begin
                    lq_out_entry_q[1] <= lq_out_push_entry;
                    lq_out_idx_q[1]   <= lq_out_push_idx;
                end else begin
                    lq_out_entry_q[0] <= lq_out_push_entry;
                    lq_out_idx_q[0]   <= lq_out_push_idx;
                end
            end
            if (lq_out_pop) begin
                lq_out_head_q <= !lq_out_head_q;
            end
            unique case ({lq_out_push, lq_out_pop})
                2'b10:   lq_out_count_q <= lq_out_count_q + 2'd1;
                2'b01:   lq_out_count_q <= lq_out_count_q - 2'd1;
                default: lq_out_count_q <= lq_out_count_q;
            endcase
        end
    end

    // Hold a presented load's payload and identity until acceptance, even if
    // recovery removes its original queue entry. Stores hold their SQ head
    // using a flag. Acceptance during branch recovery sets the deferred-store
    // record, which clears when the store later retires.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dreq_held_q          <= 1'b0;
            dreq_addr_q          <= '0;
            dreq_load_entry_q    <= '0;
            dreq_load_idx_q      <= '0;
            sq_held_q            <= 1'b0;
            sq_accept_deferred_q <= 1'b0;
        end else begin
            if (lq_launch_held) begin
                dreq_held_q <= 1'b0;
            end else if (lq_req_intent_new && !dmem_ready) begin
                dreq_held_q       <= 1'b1;
                dreq_addr_q       <= lq_select_entry.addr;
                dreq_load_entry_q <= lq_select_entry;
                dreq_load_idx_q   <= lq_select_idx;
            end

            if (sq_mem_accept) begin
                sq_held_q <= 1'b0;
            end else if (sq_drain_intent && !dmem_ready) begin
                sq_held_q <= 1'b1;
            end

            if (sq_drain_fire) begin
                sq_accept_deferred_q <= 1'b0;
            end else if (sq_mem_accept) begin
                sq_accept_deferred_q <= 1'b1;
            end
        end
    end

    // Completed loads remain in the two-slot buffer until CDB grant.
    // Each write stores one complete packet. Grant clears and advances the
    // head; simultaneous refill preserves ordered presentation and keeps an
    // ungranted head packet stable.
    always_comb begin
        lq_compl_new            = '0;
        lq_compl_new.valid      = 1'b1;
        lq_compl_new.rob_idx    = lq_complete_entry.rob_idx;
        lq_compl_new.rob_seq    = lq_complete_entry.rob_seq;
        lq_compl_new.pdst       = lq_complete_entry.pdst;
        lq_compl_new.rd_wen     = lq_complete_entry.rd_wen;
        lq_compl_new.result     = load_result;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lq_compl_q[0]    <= '0;
            lq_compl_q[1]    <= '0;
            lq_compl_head_q  <= 1'b0;
            lq_compl_count_q <= 2'd0;
        end else begin
            // Whole-slot clear on pop (Icarus cannot elaborate a member NBA
            // on a struct-array element): equivalent, because a popped slot
            // is never read again until its next whole-slot write, and the
            // same-cycle pop+write case targets the OTHER slot by the tail
            // equation.
            if (lq_compl_pop) begin
                if (lq_compl_head_q) begin
                    lq_compl_q[1] <= '0;
                end else begin
                    lq_compl_q[0] <= '0;
                end
                lq_compl_head_q <= !lq_compl_head_q;
            end
            if (lq_complete_set) begin
                if (lq_compl_tail) begin
                    lq_compl_q[1] <= lq_compl_new;
                end else begin
                    lq_compl_q[0] <= lq_compl_new;
                end
            end
            unique case ({lq_complete_set, lq_compl_pop})
                2'b10:   lq_compl_count_q <= lq_compl_count_q + 2'd1;
                2'b01:   lq_compl_count_q <= lq_compl_count_q - 2'd1;
                default: lq_compl_count_q <= lq_compl_count_q;
            endcase
        end
    end

    // queue lifecycle. Trap flush and branch recovery take priority over
    // normal allocation/pop, matching the established ROB control structure.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lq_head_q  <= '0;
            lq_tail_q  <= '0;
            lq_count_q <= '0;
            lq_executed_q <= '0;
            sq_head_q  <= '0;
            sq_tail_q  <= '0;
            sq_count_q <= '0;

            for (lq_reset_i = 0; lq_reset_i < SS_LQ_DEPTH;
                 lq_reset_i = lq_reset_i + 1) begin
                lq_metadata_q[lq_reset_i] <= '0;
            end
            for (sq_reset_i = 0; sq_reset_i < SS_SQ_DEPTH;
                 sq_reset_i = sq_reset_i + 1) begin
                sq_entry_q[sq_reset_i] <= '0;
            end
        end else begin
        // Keep all non-reset queue updates within the reset branch's else.
        // This gives synthesis one asynchronous-reset priority tree, including
        // the acceptance-driven SQ update below.
        if (trap_flush) begin
            lq_head_q  <= '0;
            lq_tail_q  <= '0;
            lq_count_q <= '0;
            lq_executed_q <= '0;
            sq_head_q  <= '0;
            sq_tail_q  <= '0;
            sq_count_q <= '0;

            for (lq_reset_i = 0; lq_reset_i < SS_LQ_DEPTH;
                 lq_reset_i = lq_reset_i + 1) begin
                lq_metadata_q[lq_reset_i] <= '0;
            end
            for (sq_reset_i = 0; sq_reset_i < SS_SQ_DEPTH;
                 sq_reset_i = sq_reset_i + 1) begin
                sq_entry_q[sq_reset_i] <= '0;
            end
        end else if (branch_recover_req) begin
            // Apply the precomputed kill maps, then install each queue's
            // independently rebuilt tail/count in queue space.
            for (lq_recover_apply_i = 0; lq_recover_apply_i < SS_LQ_DEPTH;
                    lq_recover_apply_i = lq_recover_apply_i + 1) begin
                        if (lq_recover_kill[lq_recover_apply_i]) begin
                            lq_metadata_q[lq_recover_apply_i] <= '0;
                            lq_executed_q[lq_recover_apply_i] <= 1'b0;
                        end else if (lq_launch_held && unexec_held_match &&
                                     !lq_held_slot_changes_hands &&
                                     (dreq_load_idx_q == lq_idx_t'(lq_recover_apply_i))) begin
                            // A presented request may be accepted during recovery.
                            // Compose that irreversible acceptance into the surviving
                            // owner's issue state, or it will launch again afterwards.
                            // Kill wins above; identity and ownership guards exclude
                            // a retired/reused row. Response data still uses its snapshot.
                            lq_executed_q[lq_recover_apply_i] <= 1'b1;
                        end
            end
            for (sq_recover_apply_i = 0; sq_recover_apply_i < SS_SQ_DEPTH;
                    sq_recover_apply_i = sq_recover_apply_i + 1) begin
                if (sq_recover_kill[sq_recover_apply_i]) begin
                    sq_entry_q[sq_recover_apply_i] <= '0;
                end else begin
                    // A surviving producer writeback is fire-and-forget, so
                    // recovery must compose it into the surviving SQ row just
                    // as the ROB composes survivor writeback into done_q.
                    if (sq_data_wb_match[0][sq_recover_apply_i]) begin
                        sq_entry_q[sq_recover_apply_i] <=
                            capture_store_data(
                                sq_entry_q[sq_recover_apply_i],
                                sq_data_wb_value[0]);
                    end
                    if (sq_data_wb_match[1][sq_recover_apply_i]) begin
                        sq_entry_q[sq_recover_apply_i] <=
                            capture_store_data(
                                sq_entry_q[sq_recover_apply_i],
                                sq_data_wb_value[1]);
                    end
                    if (sq_complete_accept &&
                        (sq_complete_idx_q ==
                         sq_idx_t'(sq_recover_apply_i))) begin
                        sq_entry_q[sq_recover_apply_i] <=
                            clear_store_deferred_pending(
                                sq_entry_q[sq_recover_apply_i]);
                    end
                end
            end

            lq_tail_q <= lq_recover_tail;
            sq_tail_q <= sq_recover_tail;
            lq_count_q <= lq_recover_count;
            sq_count_q <= sq_recover_count;



        end else begin
            if (lq_alloc_fire) begin
                lq_metadata_q[lq_alloc_idx] <= pack_lq_metadata(lq_alloc_entry);
                lq_executed_q[lq_alloc_idx] <= 1'b0;

                lq_tail_q <= lq_tail_q + 1'b1;
            end

            if (lq_deposit_fire) begin
                lq_metadata_q[lq_deposit_idx] <= pack_lq_metadata(lq_deposit_entry_next);
                // Deposit preserves executed: it only fills a valid,
                // previously addressless slot, which cannot have launched.
            end

            if (lq_pop_fire[0]) begin
                lq_metadata_q[lq_pop_idx[0]] <= '0;
                lq_executed_q[lq_pop_idx[0]] <= 1'b0;
            end
            if (lq_pop_fire[1]) begin
                lq_metadata_q[lq_pop_idx[1]] <= '0;
                lq_executed_q[lq_pop_idx[1]] <= 1'b0;
            end
            lq_head_q <= lq_head_q + lq_pop_count;

            if (sq_alloc_fire) begin
                sq_entry_q[sq_alloc_idx] <= sq_alloc_entry;

                sq_tail_q <= sq_tail_q + 1'b1;
            end

            if (sq_deposit_fire) begin
                sq_entry_q[sq_deposit_idx] <= sq_deposit_entry_next;
            end

            // Capture accepted producer values into every matching late-data
            // row. Accepted lanes carry distinct real pdsts; the contract pin
            // below makes the assignment order immaterial.
            for (sq_data_wb_apply_i = 0;
                 sq_data_wb_apply_i < SS_SQ_DEPTH;
                 sq_data_wb_apply_i = sq_data_wb_apply_i + 1) begin
                if (sq_data_wb_match[0][sq_data_wb_apply_i]) begin
                    sq_entry_q[sq_data_wb_apply_i] <=
                        capture_store_data(sq_entry_q[sq_data_wb_apply_i],
                                           sq_data_wb_value[0]);
                end
                if (sq_data_wb_match[1][sq_data_wb_apply_i]) begin
                    sq_entry_q[sq_data_wb_apply_i] <=
                        capture_store_data(sq_entry_q[sq_data_wb_apply_i],
                                           sq_data_wb_value[1]);
                end
            end

            if (sq_complete_accept) begin
                sq_entry_q[sq_complete_idx_q] <=
                    clear_store_deferred_pending(
                        sq_entry_q[sq_complete_idx_q]);
            end

            // The acceptance-driven block below pops the SQ. Keeping it outside
            // this normal-operation branch allows a surviving store to be accepted
            // on a branch-recovery edge.

            if (lq_mem_req_fire || lq_forward_fire) begin
                lq_executed_q[lq_select_idx] <= 1'b1;
            end else if (lq_launch_held && unexec_held_match &&
                         !lq_held_slot_changes_hands) begin
                // Held-load acceptance marks executed only when its snapshot still
                // identifies the live row. Recovery or reuse may invalidate that match.
                // Also exclude a same-edge retirement or allocation on the row: registered
                // identity alone cannot see those ownership changes, and a later write
                // could otherwise mark a fresh load executed without launching it.
                // Completion still uses the request snapshot; downstream acceptance
                // rejects retired or reused identities.
                lq_executed_q[dreq_load_idx_q] <= 1'b1;
            end

            case ({lq_alloc_fire, lq_pop_fire})
                3'b100: lq_count_q <= lq_count_q + 1'b1;
                3'b111: lq_count_q <= lq_count_q - 1'b1;
                3'b010: lq_count_q <= lq_count_q - 1'b1;
                3'b001: lq_count_q <= lq_count_q - 1'b1;
                3'b011: lq_count_q <= lq_count_q - lq_count_t'(2);
                default: lq_count_q <= lq_count_q;
            endcase

            // Allocation increments the count here; the acceptance block below
            // subtracts a drained store after recovery and allocation are accounted for.
            if (sq_alloc_fire) begin
                sq_count_q <= sq_count_q + 1'b1;
            end
        end

        // Accepting a store pops the SQ even on a branch-recovery edge. The store
        // is older than the recovering branch and survives recovery. Apply this
        // after the recovery/normal updates so occupancy includes survivors and any
        // allocation, then subtracts the accepted head. Retirement may follow later
        // using the deferred-acceptance record.
        if (sq_mem_accept) begin   // rst_n guaranteed by the enclosing else
            sq_entry_q[sq_head_q] <= '0;
            sq_head_q             <= sq_head_q + 1'b1;
            sq_count_q <= (branch_recover_req ? sq_recover_count
                          : (sq_alloc_fire ? sq_count_q + 1'b1
                                           : sq_count_q)) - 1'b1;
        end

        end
    end

    // Select the held launch's complete target entry before member access
    // to avoid variable-indexed struct-array field selection in Icarus.
    always_comb begin
        unexec_held_entry = '0;
        for (unexec_i = 0; unexec_i < SS_LQ_DEPTH;
             unexec_i = unexec_i + 1) begin
            if (dreq_load_idx_q == lq_idx_t'(unexec_i)) begin
                unexec_held_entry = lq_entry_q[unexec_i];
            end
        end
        unexec_held_match = unexec_held_entry.valid &&
                            (unexec_held_entry.rob_seq ==
                             dreq_load_entry_q.rob_seq);
        // Same-cycle ownership change of the held slot: a retirement taking
        // it out of the queue, or an allocation handing it to a new load.
        // Both are written earlier in the lifecycle block than the executed
        // mark, so without this the mark would silently win over them.
        lq_held_slot_changes_hands =
            (lq_pop_fire[0] && (lq_pop_idx[0] == dreq_load_idx_q)) ||
            (lq_pop_fire[1] && (lq_pop_idx[1] == dreq_load_idx_q)) ||
            (lq_alloc_fire  && (lq_alloc_idx  == dreq_load_idx_q));
    end



`ifndef SYNTHESIS
    // Queue and interface invariants. Checks validate allocation, deposit,
    // recovery and architectural retirement correspondence independently of
    // the control equations that drive those events.
    lq_entry_t lq_chk;
    // commit-correspondence checker locals
    integer    twc_i;
    lq_entry_t twc_lq;
    logic      twc_sq_slot;
    sq_entry_t sq_chk;
    always @(posedge clk) begin
        if (rst_n === 1'b1) begin
            // a dispatched op is load XOR store — the parallel
            // alloc/deposit ifs in the lifecycle block rely on it.
            if (lq_alloc_fire && sq_alloc_fire)
                $fatal(1, "rv32i_ss_lsq: LQ and SQ allocated in one cycle");
            if (lq_deposit_fire && sq_deposit_fire)
                $fatal(1, "rv32i_ss_lsq: LQ and SQ deposits in one cycle");

            // Deposit contract: the AGU deposits exactly once, into the entry
            // reserved at dispatch — the target must be a VALID entry not yet
            // deposited. Catches ticket corruption the cycle it happens
            // (rob_idx integrity itself is pinned by the unit TB's
            // metadata-preservation checks).
            lq_chk = lq_entry_q[lq_deposit_idx];
            if (lq_deposit_fire && !(lq_chk.valid && !lq_chk.addr_valid))
                $fatal(1, "rv32i_ss_lsq: LQ deposit to invalid/already-deposited entry");
            sq_chk = sq_entry_q[sq_deposit_idx];
            if (sq_deposit_fire && !(sq_chk.valid && !sq_chk.addr_valid))
                $fatal(1, "rv32i_ss_lsq: SQ deposit to invalid/already-deposited entry");
            if (sq_deposit_fire &&
                (sq_deposit_deferred_pending !==
                 (!sq_deposit_inert && !sq_deposit_data_valid)))
                $fatal(1, "rv32i_ss_lsq: SQ deposit deferred-completion contract broken");

            // A memory write accepts only a complete, non-inert SQ head with no
            // deferred completion owed. Check acceptance because a deferred retirement
            // can occur after the SQ head has already advanced.
            if (sq_mem_accept &&
                !(sq_head_entry.addr_valid && sq_head_entry.data_valid &&
                  !sq_head_entry.deferred_pending))
                $fatal(1, "rv32i_ss_lsq: drain of an incomplete SQ head");
            if (sq_mem_accept && sq_head_entry.inert)
                $fatal(1, "rv32i_ss_lsq: drain of an INERT SQ head");

            // Allocation, AGU deposit and architectural retirement are excluded
            // from recovery and trap-flush cycles. The upstream gates must enforce
            // this; a violation would require additional survivor arbitration here.
            if ((branch_recover_req || trap_flush) &&
                (lq_alloc_fire || sq_alloc_fire))
                $fatal(1, "rv32i_ss_lsq: allocation during recovery/flush");
            if ((branch_recover_req || trap_flush) &&
                (lq_deposit_fire || sq_deposit_fire))
                $fatal(1, "rv32i_ss_lsq: deposit during recovery/flush");
            if ((branch_recover_req || trap_flush) && (|commit_fire))
                $fatal(1, "rv32i_ss_lsq: commit pop during recovery/flush");

            // Commit correspondence. Pop commands depend on commit events and op
            // class; count, validity and ROB-index agreement are checked here instead
            // of silently suppressing a malformed pop and stranding the queue head.
            for (twc_i = 0; twc_i < 2; twc_i = twc_i + 1) begin
                if (lq_pop_fire[twc_i]) begin
                    twc_lq = lq_entry_q[lq_pop_idx[twc_i]];
                    if (!twc_lq.valid)
                        $fatal(1, "rv32i_ss_lsq: LQ pop of an invalid entry (slot %0d)",
                               twc_i);
                    if (twc_lq.rob_idx !== (rob_head_idx + rob_idx_t'(twc_i)))
                        $fatal(1, "rv32i_ss_lsq: LQ pop correspondence broken (slot %0d)",
                               twc_i);
                end
            end

            // Exact pop count: the queue must actually hold what is retiring.
            if ((lq_pop_count != '0) &&
                (lq_count_t'(lq_pop_count) > lq_count_q))
                $fatal(1, "rv32i_ss_lsq: LQ pop count exceeds occupancy");

            // Two LQ pops must name two DIFFERENT entries.
            if ((lq_pop_fire == 2'b11) && (lq_pop_idx[0] == lq_pop_idx[1]))
                $fatal(1, "rv32i_ss_lsq: dual LQ pop aliasing");

            // At most one store retires per group because dmem has one write
            // port. Check the core's commit restriction again at this boundary.
            if (sq_drain_req == 2'b11)
                $fatal(1, "rv32i_ss_lsq: two stores in one commit group");

            // Store drain correspondence is checked at acceptance. Retirement may
            // lag during a deferred window. The accepting store is the SQ head by
            // program order; its commit position follows store_commit_want.
            if (sq_mem_accept) begin
                if (sq_count_q == '0)
                    $fatal(1, "rv32i_ss_lsq: SQ drain from an empty queue");
                if (!sq_head_entry.valid)
                    $fatal(1, "rv32i_ss_lsq: SQ drain of an invalid head");
                twc_sq_slot = store_commit_want[1];
                if (sq_head_entry.rob_idx !== (rob_head_idx + rob_idx_t'(twc_sq_slot)))
                    $fatal(1, "rv32i_ss_lsq: SQ drain correspondence broken (slot %0b)",
                           twc_sq_slot);
            end

            // A committed store must REACH memory: retirement implies a real
            // write beat this cycle OR the accept-defer record — the beat
            // that already happened on a recovery edge. A store can
            // never retire without one of the two.
            if (sq_drain_fire &&
                !((dmem_valid && dmem_we) || sq_accept_deferred_q))
                $fatal(1, "rv32i_ss_lsq: SQ drain without a dmem write beat");

        end
    end

    // Forwarding invariants. Independently recompute the youngest older
    // overlapping store from queue storage. That store must fully cover the
    // load, and unknown older addresses block all progress.
    logic       twf_unknown;
    logic       twf_winner_valid;
    logic       twf_winner_data_valid;
    rob_idx_t   twf_winner_age;
    logic [3:0] twf_winner_be;
    sq_idx_t    twf_winner_idx;
    rob_idx_t   twf_cand_age;
    logic       twf_full_cover;
    logic [3:0] twf_load_be;   // derived INDEPENDENTLY of the design's load_be
    // The checker combines memory-safe and forwardable cases into one progress
    // verdict; the design consumes the separate cases directly.
    logic       twf_can_progress;
    sq_entry_t  twf_sq;
    integer     twf_i;

    always_comb begin
        twf_unknown      = 1'b0;
        twf_winner_valid = 1'b0;
        twf_winner_data_valid = 1'b0;
        twf_winner_age   = '0;
        twf_winner_be    = '0;
        twf_winner_idx   = '0;
        twf_sq           = '0;
        twf_cand_age     = lq_select_entry.rob_idx - rob_head_idx;
        unique case (lq_select_entry.mem_size)
            fyp_cpu_pkg::MEM_W: twf_load_be = 4'b1111;
            fyp_cpu_pkg::MEM_H: twf_load_be = lq_select_entry.addr[1]
                                            ? 4'b1100 : 4'b0011;
            fyp_cpu_pkg::MEM_B: begin
                unique case (lq_select_entry.addr[1:0])
                    2'b00:   twf_load_be = 4'b0001;
                    2'b01:   twf_load_be = 4'b0010;
                    2'b10:   twf_load_be = 4'b0100;
                    default: twf_load_be = 4'b1000;
                endcase
            end
            default: twf_load_be = 4'b1111;
        endcase
        for (twf_i = 0; twf_i < SS_SQ_DEPTH; twf_i = twf_i + 1) begin
            twf_sq = sq_entry_q[twf_i];
            if (lq_select_valid && twf_sq.valid && !twf_sq.inert &&
                ((twf_sq.rob_idx - rob_head_idx) < twf_cand_age)) begin
                if (!twf_sq.addr_valid) begin
                    twf_unknown = 1'b1;
                end else if ((twf_sq.addr[31:2] == lq_select_entry.addr[31:2])
                             && (|(twf_sq.be & twf_load_be))) begin
                    if (!twf_winner_valid ||
                        ((twf_sq.rob_idx - rob_head_idx) > twf_winner_age)) begin
                        twf_winner_valid = 1'b1;
                        twf_winner_age   = twf_sq.rob_idx - rob_head_idx;
                        twf_winner_be    = twf_sq.be;
                        twf_winner_data_valid = twf_sq.data_valid;
                        twf_winner_idx   = sq_idx_t'(twf_i);
                    end
                end
            end
        end
        twf_full_cover   = twf_winner_valid &&
                           ((twf_winner_be & twf_load_be) == twf_load_be);
        twf_can_progress = lq_select_valid && !twf_unknown &&
                           (!twf_winner_valid ||
                            (twf_full_cover && twf_winner_data_valid));
    end

    // Previous-cycle sampling for the rise-discipline checks.
    logic [1:0] twf_out_count_prev_q  = 2'd0;
    logic      twf_rvalid_prev_q      = 1'b0;
    logic      twf_req_fire_prev_q    = 1'b0;
    logic      twf_any_fire_prev_q    = 1'b0;
    logic      twf_held_launch_prev_q = 1'b0;
    logic      twf_launch_prev_q      = 1'b0;
    lq_idx_t   twf_held_idx_prev_q    = '0;
    logic      twm_presented_prev_q   = 1'b0;
    logic      twm_accepted_prev_q    = 1'b0;
    logic      twm_we_prev_q          = 1'b0;
    word_t     twm_addr_prev_q        = '0;
    word_t     twm_wdata_prev_q       = '0;
    logic [3:0] twm_be_prev_q         = '0;
    lq_idx_t   twf_sel_idx_prev_q     = '0;
    logic [SS_LQ_DEPTH-1:0] twf_executed_prev_q = '0;
    completion_packet_t     twf_lq_complete_prev_q;
    logic                   twf_lq_held_prev_q = 1'b0;
    lq_entry_t twf_lq;
    integer    twf_j;

    // Late-store-data lifetime checker. These snapshots make the legal
    // transition causes independent of the queue update equations above:
    // address arrives only at AGU deposit; data arrives at that deposit or
    // through an accepted CDB beat; deferred_pending rises only at an
    // address-only deposit and clears only when its registered packet is sent.
    logic [SS_SQ_DEPTH-1:0] tws_addr_valid_prev_q = '0;
    logic [SS_SQ_DEPTH-1:0] tws_data_valid_prev_q = '0;
    logic [SS_SQ_DEPTH-1:0] tws_deferred_pending_prev_q = '0;
    logic                   tws_deposit_fire_prev_q = 1'b0;
    sq_idx_t                tws_deposit_idx_prev_q = '0;
    logic                   tws_deposit_data_valid_prev_q = 1'b0;
    logic                   tws_deposit_deferred_pending_prev_q = 1'b0;
    logic [1:0]             tws_deposit_wb_match_prev_q = '0;
    logic [SS_SQ_DEPTH-1:0] tws_wb_match0_prev_q = '0;
    logic [SS_SQ_DEPTH-1:0] tws_wb_match1_prev_q = '0;
    word_t                  tws_wb_value0_prev_q = '0;
    word_t                  tws_wb_value1_prev_q = '0;
    logic                   tws_complete_accept_prev_q = 1'b0;
    sq_idx_t                tws_complete_idx_prev_q = '0;
    completion_packet_t     tws_complete_prev_q;
    logic                   tws_complete_held_prev_q = 1'b0;
    sq_entry_t              tws_sq;
    integer                 tws_i;

    always @(posedge clk) begin
        if (rst_n === 1'b1) begin
            // a forward's source is the youngest older overlapping
            // match, and it fully covers the load.
            if (lq_forward_fire &&
                !(twf_winner_valid && twf_full_cover &&
                  twf_winner_data_valid && !twf_unknown))
                $fatal(1, "rv32i_ss_lsq: forward without a full-cover youngest-older winner");
            if (lq_forward_fire && (sq_forward_idx != twf_winner_idx))
                $fatal(1, "rv32i_ss_lsq: forward source is not the youngest older overlapping match");

            // an unknown older store address forbids ALL progress.
            if ((lq_mem_req_fire || lq_forward_fire) && twf_unknown)
                $fatal(1, "rv32i_ss_lsq: progress past an unknown older store address");

            // memory launch and forward are mutually exclusive.
            if (lq_mem_req_fire && lq_forward_fire)
                $fatal(1, "rv32i_ss_lsq: memory launch and forward in one cycle");

            // A forward never drives a memory read. Outstanding count rises only
            // after an accepted read and falls only after a response; each event
            // changes the count by at most one.
            if (lq_forward_fire && dmem_valid && !dmem_we)
                $fatal(1, "rv32i_ss_lsq: forward drove a dmem read");
            if ((lq_out_count_q > twf_out_count_prev_q) &&
                !twf_launch_prev_q)
                $fatal(1, "rv32i_ss_lsq: outstanding count rose without an accepted read");
            if ((lq_out_count_q < twf_out_count_prev_q) &&
                !twf_rvalid_prev_q)
                $fatal(1, "rv32i_ss_lsq: outstanding count fell without a response");

            // The outstanding-read limit is two. A third accepted read is an error,
            // not a condition that may silently saturate the count.
            if ((lq_mem_req_fire || lq_launch_held) &&
                (lq_out_count_q == 2'd2))
                $fatal(1, "rv32i_ss_lsq: third outstanding read accepted");

            // Whenever reads are outstanding, completion metadata must equal the
            // FIFO head snapshot. The held-request check below covers direct completion
            // when the outstanding FIFO is empty.
            if (lq_complete_set && (lq_out_count_q != 2'd0) &&
                ((lq_complete_entry.rob_idx !== lq_out_head_entry.rob_idx) ||
                 (lq_complete_entry.rob_seq !== lq_out_head_entry.rob_seq)))
                $fatal(1, "rv32i_ss_lsq: completion identity diverged from the outstanding head");

            // The completion buffer cannot overflow — the
            // launch gate reserves its capacity at acceptance, so a write
            // into a full, undraining buffer means the reservation broke.
            if (lq_complete_set && (lq_compl_count_q == 2'd2) &&
                !lq_compl_pop)
                $fatal(1, "rv32i_ss_lsq: completion buffer overflow");

            // A held request answered on its own acceptance edge must complete under
            // its snapshotted identity. Using the current selection could corrupt a
            // different live load, which downstream stale rejection cannot detect.
            // Responses with a nonempty outstanding FIFO belong to its head instead.
            if (lq_complete_set && lq_launch_held &&
                (lq_out_count_q == 2'd0) &&
                ((lq_complete_entry.rob_idx !== dreq_load_entry_q.rob_idx) ||
                 (lq_complete_entry.rob_seq !== dreq_load_entry_q.rob_seq)))
                $fatal(1, "rv32i_ss_lsq: held launch completed under a foreign identity");

            // A forward cannot coincide with outstanding reads or buffered
            // completions. Fully draining the buffer through this cycle's grant permits
            // a new forwarded completion.
            if (lq_forward_fire &&
                ((lq_out_count_q != 2'd0) ||
                 (lq_compl_count_after_grant != 2'd0)))
                $fatal(1, "rv32i_ss_lsq: forward collides with outstanding/completion buffer");

            // flow-and-reject: an ungranted held completion is never
            // scrubbed or mutated -- it leaves only through a CDB grant.
            // pinned on the PRESENTED VIEW (the buffer head), which is
            // exactly what the CDB client sees; the second slot is invisible
            // to it and is free to fill behind the head.
            if (twf_lq_held_prev_q &&
                (lq_complete !== twf_lq_complete_prev_q))
                $fatal(1, "rv32i_ss_lsq: held LQ completion scrubbed or mutated");

            // Executed rises ONLY via a launch/forward
            // fire on the selected entry, or via a held launch on the
            // launch-time snapshot index.
            for (twf_j = 0; twf_j < SS_LQ_DEPTH; twf_j = twf_j + 1) begin
                twf_lq = lq_entry_q[twf_j];
                if (twf_lq.executed && !twf_executed_prev_q[twf_j] &&
                    !(twf_any_fire_prev_q &&
                      (twf_sel_idx_prev_q == lq_idx_t'(twf_j))) &&
                    !(twf_held_launch_prev_q &&
                      (twf_held_idx_prev_q == lq_idx_t'(twf_j))))
                    $fatal(1, "rv32i_ss_lsq: executed set outside a launch/forward fire");
            end

            // A presented request retains valid and its complete payload until
            // acceptance. The deferred-store record permits only one accepted store
            // in its window and exists only while that store still wants to commit.
            if (twm_presented_prev_q && !twm_accepted_prev_q &&
                (!dmem_valid ||
                 (dmem_we    !== twm_we_prev_q)   ||
                 (dmem_addr  !== twm_addr_prev_q) ||
                 (dmem_wdata !== twm_wdata_prev_q) ||
                 (dmem_be    !== twm_be_prev_q)))
                $fatal(1, "rv32i_ss_lsq: presented request changed before acceptance");
            if (sq_accept_deferred_q && sq_mem_accept)
                $fatal(1, "rv32i_ss_lsq: second store acceptance inside the accept-defer window");
            if (sq_accept_deferred_q && !(|store_commit_want))
                $fatal(1, "rv32i_ss_lsq: accept-defer record without a committing store");

            // no fire while the contract recompute says no progress.
            if ((lq_mem_req_fire || lq_forward_fire) && !twf_can_progress)
                $fatal(1, "rv32i_ss_lsq: progress fired against the contract recompute");
        end
        twf_out_count_prev_q   <= lq_out_count_q;
        twf_rvalid_prev_q      <= dmem_rvalid;
        twf_req_fire_prev_q    <= lq_mem_req_fire;
        twf_any_fire_prev_q    <= lq_mem_req_fire || lq_forward_fire;
        twf_lq_held_prev_q     <= lq_complete.valid && !cdb_grant_lq;
        twf_lq_complete_prev_q <= lq_complete;
        twf_sel_idx_prev_q     <= lq_select_idx;
        twf_held_launch_prev_q <= lq_launch_held && unexec_held_match;
        twf_launch_prev_q      <= lq_mem_req_fire || lq_launch_held;
        twf_held_idx_prev_q    <= dreq_load_idx_q;
        twm_presented_prev_q   <= dmem_valid;
        twm_accepted_prev_q    <= dmem_valid && dmem_ready;
        twm_we_prev_q          <= dmem_we;
        twm_addr_prev_q        <= dmem_addr;
        twm_wdata_prev_q       <= dmem_wdata;
        twm_be_prev_q          <= dmem_be;
        for (twf_j = 0; twf_j < SS_LQ_DEPTH; twf_j = twf_j + 1) begin
            twf_lq = lq_entry_q[twf_j];
            twf_executed_prev_q[twf_j] <= twf_lq.executed;
        end
    end

    always @(posedge clk) begin
        if (rst_n === 1'b1) begin
            if (sq_data_wb_fire[0] && sq_data_wb_fire[1] &&
                (sq_data_wb_pdst[0] == sq_data_wb_pdst[1]))
                $fatal(1, "rv32i_ss_lsq: accepted CDB lanes alias one pdst");

            if (sq_complete_accept && !sq_complete_q.valid)
                $fatal(1, "rv32i_ss_lsq: deferred store completion accepted while invalid");

            if (tws_complete_held_prev_q &&
                (sq_complete_q !== tws_complete_prev_q))
                $fatal(1, "rv32i_ss_lsq: held SQ completion scrubbed or mutated");

            for (tws_i = 0; tws_i < SS_SQ_DEPTH; tws_i = tws_i + 1) begin
                tws_sq = sq_entry_q[tws_i];
                if (tws_sq.valid) begin
                    if (tws_sq.addr_valid &&
                        !tws_addr_valid_prev_q[tws_i] &&
                        !(tws_deposit_fire_prev_q &&
                          (tws_deposit_idx_prev_q == sq_idx_t'(tws_i))))
                        $fatal(1, "rv32i_ss_lsq: SQ addr_valid rose outside AGU deposit");

                    if (tws_sq.data_valid &&
                        !tws_data_valid_prev_q[tws_i] &&
                        !((tws_deposit_fire_prev_q &&
                           tws_deposit_data_valid_prev_q &&
                           (tws_deposit_idx_prev_q == sq_idx_t'(tws_i))) ||
                          (tws_deposit_fire_prev_q &&
                           (|tws_deposit_wb_match_prev_q) &&
                           (tws_deposit_idx_prev_q == sq_idx_t'(tws_i))) ||
                          tws_wb_match0_prev_q[tws_i] ||
                          tws_wb_match1_prev_q[tws_i]))
                        $fatal(1, "rv32i_ss_lsq: SQ data_valid rose outside deposit/accepted CDB capture");

                    if (tws_sq.deferred_pending &&
                        !tws_deferred_pending_prev_q[tws_i] &&
                        !(tws_deposit_fire_prev_q &&
                          tws_deposit_deferred_pending_prev_q &&
                          (tws_deposit_idx_prev_q == sq_idx_t'(tws_i))))
                        $fatal(1, "rv32i_ss_lsq: SQ deferred_pending rose outside address-only deposit");

                    if (!tws_sq.deferred_pending &&
                        tws_deferred_pending_prev_q[tws_i] &&
                        !(tws_complete_accept_prev_q &&
                          (tws_complete_idx_prev_q == sq_idx_t'(tws_i))))
                        $fatal(1, "rv32i_ss_lsq: SQ deferred_pending cleared without completion acceptance");

                    if (tws_sq.deferred_pending &&
                        (!tws_sq.addr_valid || tws_sq.inert))
                        $fatal(1, "rv32i_ss_lsq: deferred completion pending on an unaddressed/inert store");

                    if (tws_wb_match0_prev_q[tws_i] &&
                        (tws_sq.data !==
                         format_store_data(tws_wb_value0_prev_q, tws_sq.be)))
                        $fatal(1, "rv32i_ss_lsq: lane-0 late store data captured with wrong byte alignment");
                    if (tws_wb_match1_prev_q[tws_i] &&
                        (tws_sq.data !==
                         format_store_data(tws_wb_value1_prev_q, tws_sq.be)))
                        $fatal(1, "rv32i_ss_lsq: lane-1 late store data captured with wrong byte alignment");
                    if (tws_deposit_fire_prev_q &&
                        (tws_deposit_idx_prev_q == sq_idx_t'(tws_i)) &&
                        tws_deposit_wb_match_prev_q[0] &&
                        (tws_sq.data !==
                         format_store_data(tws_wb_value0_prev_q, tws_sq.be)))
                        $fatal(1, "rv32i_ss_lsq: lane-0 deposit/CDB store data captured with wrong byte alignment");
                    if (tws_deposit_fire_prev_q &&
                        (tws_deposit_idx_prev_q == sq_idx_t'(tws_i)) &&
                        tws_deposit_wb_match_prev_q[1] &&
                        (tws_sq.data !==
                         format_store_data(tws_wb_value1_prev_q, tws_sq.be)))
                        $fatal(1, "rv32i_ss_lsq: lane-1 deposit/CDB store data captured with wrong byte alignment");
                end
            end

            if (sq_complete_q.valid) begin
                tws_sq = sq_entry_q[sq_complete_idx_q];
                if (!(tws_sq.valid && tws_sq.addr_valid &&
                      tws_sq.data_valid && !tws_sq.inert &&
                      tws_sq.deferred_pending &&
                      (sq_complete_q.rob_idx == tws_sq.rob_idx) &&
                      (sq_complete_q.rob_seq == tws_sq.rob_seq) &&
                      (sq_complete_q.result == tws_sq.addr) &&
                      !sq_complete_q.rd_wen &&
                      !sq_complete_q.trap_valid && !sq_complete_q.csr_we))
                    $fatal(1, "rv32i_ss_lsq: deferred completion does not name one ready SQ row");
            end
        end

        tws_deposit_fire_prev_q <= sq_deposit_fire;
        tws_deposit_idx_prev_q <= sq_deposit_idx;
        tws_deposit_data_valid_prev_q <= sq_deposit_data_valid;
        tws_deposit_deferred_pending_prev_q <=
            sq_deposit_deferred_pending;
        tws_deposit_wb_match_prev_q <= sq_deposit_wb_match;
        tws_wb_match0_prev_q <= sq_data_wb_match[0];
        tws_wb_match1_prev_q <= sq_data_wb_match[1];
        tws_wb_value0_prev_q <= sq_data_wb_value[0];
        tws_wb_value1_prev_q <= sq_data_wb_value[1];
        tws_complete_accept_prev_q <= sq_complete_accept;
        tws_complete_idx_prev_q <= sq_complete_idx_q;
        tws_complete_held_prev_q <= sq_complete_q.valid &&
                                      !sq_complete_accept &&
                                      !branch_recover_req && !trap_flush;
        tws_complete_prev_q <= sq_complete_q;
        for (tws_i = 0; tws_i < SS_SQ_DEPTH; tws_i = tws_i + 1) begin
            tws_sq = sq_entry_q[tws_i];
            tws_addr_valid_prev_q[tws_i] <= tws_sq.addr_valid;
            tws_data_valid_prev_q[tws_i] <= tws_sq.data_valid;
            tws_deferred_pending_prev_q[tws_i] <= tws_sq.deferred_pending;
        end
    end
`endif


`ifndef SYNTHESIS
    /* verilator lint_off SYNCASYNCNET */
    // Allocation event integrity: the alloc fires are core-derived events and must
    // include the queue's own capacity answer.
    always @(posedge clk) begin
        if (rst_n === 1'b1) begin
            if (lq_alloc_fire && !lq_alloc_ready) begin
                $fatal(1, "rv32i_ss_lsq: lq_alloc_fire with LQ full (fire-honesty)");
            end
            if (sq_alloc_fire && !sq_alloc_ready) begin
                $fatal(1, "rv32i_ss_lsq: sq_alloc_fire with SQ full (fire-honesty)");
            end
        end
    end
    /* verilator lint_on SYNCASYNCNET */
`endif

endmodule
