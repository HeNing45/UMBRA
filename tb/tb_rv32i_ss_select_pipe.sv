`timescale 1ns/1ps
// Contract-derived scoreboard for the full-picker register. No DUT-private
// state is read. A transaction carries distinct full payload/ROB generation.
// Directed: no fallthrough; blocked second lane holds BOTH; refill/drain;
// wraparound older survivor/younger kill; flush and same-index/pdst reuse.
module tb_rv32i_ss_select_pipe;
    import rv32i_ss_pkg::*;
    logic clk = 0;
    always #5 clk = ~clk;
    logic rst_n, trap_flush, branch_recover_req;
    rob_idx_t recover_rob_idx, rob_head_idx;
    logic [1:0] iq_select_valid, issue_valid;
    iq_entry_t [1:0] iq_select_entry, issue_entry;
    issue_unit_e [1:0] iq_select_unit, issue_unit;
    logic iq_select_accept, issue_accept;
    logic alu0_fu_ready, alu1_fu_ready, muldiv_fu_ready, lsu_fu_ready;
    rv32i_ss_select_pipe dut (.*);
    int checks = 0;
    int held = 0, replaced = 0, recovered = 0;
    logic [1:0] model_valid;
    iq_entry_t [1:0] model_entry;
    issue_unit_e [1:0] model_unit;
    int transfers = 0;

    function automatic iq_entry_t packet(input int idx, input int seq);
        iq_entry_t e;
        e = '0;
        e.rob_idx = rob_idx_t'(idx);
        e.rob_seq = rob_seq_t'(seq);
        e.pc = 32'h1000 + 4*word_t'(seq);
        e.imm = 32'h76543210 ^ word_t'(seq);
        e.prs1 = phys_reg_t'(seq);
        e.prs2 = phys_reg_t'(seq+1);
        e.pdst = phys_reg_t'(40);
        e.rd_wen = 1'b1;
        packet = e;
    endfunction
    task automatic check(input logic okay, input string why);
        checks++;
        if (okay !== 1'b1) $fatal(1, "%s", why);
    endtask
    task automatic step;
        logic available, take, accept;
        int a, r;
        #1;
        available = 1'b1;
        for (int i = 0; i < 2; i++) begin
            if (model_valid[i]) begin
                case (model_unit[i])
                    ISSUE_UNIT_ALU0: available &= alu0_fu_ready;
                    ISSUE_UNIT_ALU1: available &= alu1_fu_ready;
                    ISSUE_UNIT_MULDIV: available &= muldiv_fu_ready;
                    ISSUE_UNIT_AGEN: available &= lsu_fu_ready;
                endcase
            end
            check(issue_valid[i] === model_valid[i], "registered validity");
            if (model_valid[i]) begin
                check(issue_entry[i] === model_entry[i], "full identity/payload held");
                check(issue_unit[i] === model_unit[i], "binding held");
            end
        end
        take = available && !trap_flush && !branch_recover_req;
        accept = (!(|model_valid) || take) && !trap_flush && !branch_recover_req;
        check(issue_accept === take, "atomic downstream acceptance");
        check(iq_select_accept === accept, "owned select capacity");
        if ((|model_valid) && !take && !trap_flush && !branch_recover_req) held++;
        if ((|model_valid) && take && (|iq_select_valid)) replaced++;
        if (take) transfers += int'(model_valid[0]) + int'(model_valid[1]);
        @(posedge clk);
        if (trap_flush) model_valid = '0;
        else if (branch_recover_req) begin
            recovered++;
            r = (int'(recover_rob_idx) - int'(rob_head_idx)) & 31;
            for (int i = 0; i < 2; i++) begin
                a = (int'(model_entry[i].rob_idx) - int'(rob_head_idx)) & 31;
                if (model_valid[i] && (a > r)) model_valid[i] = 1'b0;
            end
        end else if (accept) begin
            model_valid = iq_select_valid;
            model_entry = iq_select_entry;
            model_unit = iq_select_unit;
        end
        @(negedge clk);
    endtask
    initial begin
        #20000;
        $fatal(1, "WATCHDOG select_pipe");
    end
    initial begin
        rst_n = 0; trap_flush = 0; branch_recover_req = 0;
        rob_head_idx = 30; recover_rob_idx = 0;
        iq_select_valid = 0; iq_select_entry = '0; iq_select_unit = '0;
        model_valid = 0; model_entry = '0; model_unit = '0;
        alu0_fu_ready = 1; alu1_fu_ready = 1; muldiv_fu_ready = 1; lsu_fu_ready = 1;
        repeat (2) @(negedge clk);
        rst_n = 1;
        // Select even though nothing may bypass the empty register this cycle.
        iq_select_valid = 3;
        iq_select_entry[0] = packet(31, 10);
        iq_select_entry[1] = packet(1, 11);
        iq_select_unit[0] = ISSUE_UNIT_ALU0;
        iq_select_unit[1] = ISSUE_UNIT_ALU1;
        step();
        // Block lane 1; adversarial new offers must not replace either lane.
        alu1_fu_ready = 0;
        iq_select_entry[0] = packet(2, 99);
        repeat (3) step();
        // Recovery on the stalled cycle: idx31 older survives, idx1 younger dies.
        branch_recover_req = 1; recover_rob_idx = 0;
        step();
        check(model_valid == 1, "wrapped older survivor required");
        branch_recover_req = 0;
        iq_select_entry[0] = packet(1, 111); // same killed index/pdst, fresh generation
        iq_select_entry[1] = packet(2, 112);
        iq_select_unit[0] = ISSUE_UNIT_MULDIV;
        iq_select_unit[1] = ISSUE_UNIT_AGEN;
        step(); // older survivor transfers; fresh pair replaces it
        // Sweep all downstream capacity patterns while the pair remains blocked.
        for (int k = 0; k < 15; k++) begin
            {lsu_fu_ready,muldiv_fu_ready,alu1_fu_ready,alu0_fu_ready} = 4'(k);
            iq_select_valid = 0;
            step();
        end
        {lsu_fu_ready,muldiv_fu_ready,alu1_fu_ready,alu0_fu_ready} = 4'b1111;
        iq_select_valid = 3;
        iq_select_entry[0] = packet(3, 120);
        iq_select_entry[1] = packet(4, 121);
        step();
        trap_flush = 1;
        step();
        trap_flush = 0; iq_select_valid = 0;
        step();
        // Full-rate replacement: two transfers each cycle, no fill/drain bubble.
        iq_select_unit[0] = ISSUE_UNIT_ALU0; iq_select_unit[1] = ISSUE_UNIT_ALU1;
        for (int n = 0; n < 12; n++) begin
            iq_select_valid = 3;
            iq_select_entry[0] = packet(5, 200+2*n);
            iq_select_entry[1] = packet(6, 201+2*n);
            step();
        end
        iq_select_valid = 0; step(); step();
        check(held >= 3 && recovered == 1 && replaced >= 11, "reachability floors");
        check(transfers >= 24, "sustained dual transfer witness");
        $display("[tb_rv32i_ss_select_pipe] PASS checks=%0d held=%0d replaced=%0d recovery=%0d",
                 checks, held, replaced, recovered);
        $finish;
    end
endmodule
