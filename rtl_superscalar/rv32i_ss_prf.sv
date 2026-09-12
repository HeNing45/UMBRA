`timescale 1ns/1ps

import rv32i_ss_pkg::OOO_PHYS_REGS;
import rv32i_ss_pkg::OOO_ARCH_REGS;
import rv32i_ss_pkg::phys_reg_t;
import rv32i_ss_pkg::word_t;

module rv32i_ss_prf (
    input logic clk,
    input logic rst_n,
    //read
    input phys_reg_t raddr1,
    output word_t rdata1,
    input phys_reg_t raddr2,
    output word_t rdata2,
    // Ports 3/4 serve grant position 1.
    input phys_reg_t raddr3,
    output word_t rdata3,
    input phys_reg_t raddr4,
    output word_t rdata4,
    //write (on accepted CDB beat): writes the value AND marks the dest ready
    input logic [1:0] write_en,
    input phys_reg_t [1:0] waddr,
    input word_t [1:0] wdata,

    // ---- ready/busy table ----
    // alloc: a new pdst was allocated this cycle -> mark it busy (not ready)
    input  logic [1:0]                  alloc_fire,
    input  phys_reg_t [1:0]             alloc_phys,
    // Ready-only ALU execute wakeup: the core ties ports 0/1 inactive and
    // uses ports 2/3 when a final result enters a live ALU result holder.
    // Non-ALU readiness uses accepted write_en, together with the value.
    // Allocation's busy-clear is ordered LAST, so physical-tag reuse wins
    // any legal same-edge collision with an old early set.
    input  logic [3:0]                  early_set,
    input  phys_reg_t [3:0]             early_pdst,
    // Per-physreg readiness consumed by the IQ; p0 is always ready.
    output logic [OOO_PHYS_REGS-1:0] ready_vec
);

    word_t                    regs_q[OOO_PHYS_REGS];
    logic [OOO_PHYS_REGS-1:0] ready_q;
    integer reg_rst;
    integer write_i;
    integer alloc_i;

    always_comb begin
        if (raddr1 == '0) begin
            rdata1 = '0;
        end else begin
            rdata1 = regs_q[raddr1];
        end

        if (raddr2 == '0) begin
            rdata2 = '0;
        end else begin
            rdata2 = regs_q[raddr2];
        end

        if (raddr3 == '0) begin
            rdata3 = '0;
        end else begin
            rdata3 = regs_q[raddr3];
        end

        if (raddr4 == '0) begin
            rdata4 = '0;
        end else begin
            rdata4 = regs_q[raddr4];
        end
    end

    // p0 is the zero register: always ready, regardless of the table.
    assign ready_vec = {ready_q[OOO_PHYS_REGS-1:1], 1'b1};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (reg_rst = 1; reg_rst < OOO_PHYS_REGS; reg_rst++) begin
                regs_q[reg_rst] <= '0;
            end
            // Reset readiness: the committed arch physregs p0..p31 hold valid 0s
            // (the reset architectural state), so they are ready; the free pool
            // p32..p63 holds no produced value yet, so it is not ready.
            for (reg_rst = 0; reg_rst < OOO_PHYS_REGS; reg_rst++) begin
                ready_q[reg_rst] <= (reg_rst < OOO_ARCH_REGS) ? 1'b1 : 1'b0;
            end
        end else begin
            // Accepted writeback: store the value and mark the dest ready.
            for (write_i = 0; write_i < 2; write_i++) begin
                if (write_en[write_i] && (waddr[write_i] != '0)) begin
                    regs_q[waddr[write_i]] <= wdata[write_i];
                    ready_q[waddr[write_i]] <= 1'b1;
                end
            end
            // Early wakeup is ready-only; core ports 2/3 carry ALU execute
            // sets, backed by a live result holder or accepted transit.
            for (write_i = 0; write_i < 4; write_i++) begin
                if (early_set[write_i] && (early_pdst[write_i] != '0)) begin
                    ready_q[early_pdst[write_i]] <= 1'b1;
                end
            end
            // Allocation clears readiness until an early ALU result or accepted
            // writeback supplies the value. Busy-clear wins on physical-tag reuse.
            for (alloc_i = 0; alloc_i < 2; alloc_i++) begin
                if (alloc_fire[alloc_i] && alloc_phys[alloc_i] != '0) begin
                    ready_q[alloc_phys[alloc_i]] <= 1'b0;
                end
            end
        end
    end


`ifndef SYNTHESIS
    /* verilator lint_off SYNCASYNCNET */
    integer pin_alloc_i;
    integer pin_write_i;

    // Ready-table pins: same-cycle clears and same-cycle sets each target
    // distinct pregs, and every clear is disjoint from every accepted set.
    always @(posedge clk) begin
        if (rst_n === 1'b1) begin
            if (alloc_fire[0] && alloc_fire[1] &&
                (alloc_phys[0] == alloc_phys[1]) && (alloc_phys[0] != '0)) begin
                $fatal(1, "rv32i_ss_prf: dual busy-clear targets one preg");
            end
            if (write_en[0] && write_en[1] &&
                (waddr[0] == waddr[1]) && (waddr[0] != '0)) begin
                $fatal(1, "rv32i_ss_prf: dual writeback targets one preg");
            end
            for (pin_alloc_i = 0; pin_alloc_i < 2; pin_alloc_i++) begin
                for (pin_write_i = 0; pin_write_i < 2; pin_write_i++) begin
                    if (alloc_fire[pin_alloc_i] && write_en[pin_write_i] &&
                        (waddr[pin_write_i] == alloc_phys[pin_alloc_i]) &&
                        (waddr[pin_write_i] != '0)) begin
                        $fatal(1, "rv32i_ss_prf: busy-clear collides with same-cycle writeback");
                    end
                end
            end
        end
    end
    /* verilator lint_on SYNCASYNCNET */
`endif

endmodule
