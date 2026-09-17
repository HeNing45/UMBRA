// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// Full IQ picker -> select_q -> PRF/bypass -> existing execute inputs.
// select_q owns a selected bundle, NOT a promise of downstream free space.
// It is an elastic two-wide register: all surviving lanes transfer together,
// or the entire bundle waits. IQ removal happens only on iq_select_accept.
// Its capacity is reserved by accepting that bundle; a full stage may refill
// only when its OLD registered bundle transfers. No current IQ winner, PRF
// data, execute pop or CDB grant participates in the acceptance equation.
//
// Registered identity includes ROB generation, physical sources/destination,
// checkpoint/prediction and LSQ tickets. No operand is read before this flop.
// Sources remain owned while this instruction is uncompleted in the ROB;
// recovery kills its younger consumers before physical-register reuse.
module rv32i_ss_select_pipe
  import rv32i_ss_pkg::*;
(
    input logic clk,
    input logic rst_n,
    input logic trap_flush,
    input logic branch_recover_req,
    input rob_idx_t recover_rob_idx,
    input rob_idx_t rob_head_idx,

    input logic [1:0] iq_select_valid,
    input iq_entry_t [1:0] iq_select_entry,
    input issue_unit_e [1:0] iq_select_unit,
    output logic iq_select_accept,

    input logic alu0_fu_ready,
    input logic alu1_fu_ready,
    input logic muldiv_fu_ready,
    input logic lsu_fu_ready,
    output logic [1:0] issue_valid,
    output iq_entry_t [1:0] issue_entry,
    output issue_unit_e [1:0] issue_unit,
    output logic issue_accept
);
    typedef struct packed {
        logic valid;
        iq_entry_t uop;
        issue_unit_e unit;
    } select_slot_t;
    select_slot_t [1:0] select_q;
    logic [1:0] unit_ready;
    rob_idx_t [1:0] select_age;
    rob_idx_t recover_age;

    assign recover_age = recover_rob_idx - rob_head_idx;
    for (genvar i = 0; i < 2; i++) begin : g_select_lane
        always_comb begin
            issue_valid[i] = select_q[i].valid;
            issue_entry[i] = select_q[i].uop;
            issue_unit[i] = select_q[i].unit;
            select_age[i] = select_q[i].uop.rob_idx - rob_head_idx;
            case (select_q[i].unit)
                ISSUE_UNIT_ALU0: unit_ready[i] = alu0_fu_ready;
                ISSUE_UNIT_ALU1: unit_ready[i] = alu1_fu_ready;
                ISSUE_UNIT_MULDIV: unit_ready[i] = muldiv_fu_ready;
                ISSUE_UNIT_AGEN: unit_ready[i] = lsu_fu_ready;
                default: unit_ready[i] = 1'b0;
            endcase
        end

        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                select_q[i] <= '0;
            end else if (trap_flush) begin
                select_q[i].valid <= 1'b0;
            end else if (branch_recover_req) begin
                if (select_q[i].valid && (select_age[i] > recover_age))
                    select_q[i].valid <= 1'b0;
            end else if (iq_select_accept) begin
                select_q[i].valid <= iq_select_valid[i];
                if (iq_select_valid[i]) begin
                    select_q[i] <= {1'b1, iq_select_entry[i], iq_select_unit[i]};
                end
            end
        end
    end

    // Both units were distinct at IQ binding. No partial transfer, rebinding,
    // or borrowing of same-cycle execute/result drains is permitted.
    assign issue_accept = !trap_flush && !branch_recover_req
                        && (&(unit_ready | ~issue_valid));
    assign iq_select_accept = !trap_flush && !branch_recover_req
                            && (!(|issue_valid) || issue_accept);

endmodule
