// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

import rv32i_ooo_pkg::OOO_PHYS_REGS;
import rv32i_ooo_pkg::OOO_ARCH_REGS;
import rv32i_ooo_pkg::phys_reg_t;
import rv32i_ooo_pkg::word_t;

module rv32i_ooo_prf (
    input logic clk,
    input logic rst_n,
    //read
    input phys_reg_t raddr1,
    output word_t rdata1,
    input phys_reg_t raddr2,
    output word_t rdata2,
    //write (on accepted CDB beat): writes the value AND marks the dest ready
    input logic write_en,
    input phys_reg_t waddr,
    input word_t wdata,

    // ---- ready/busy table ----
    // alloc: a new pdst was allocated this cycle -> mark it busy (not ready)
    input  logic                     alloc_en,
    input  phys_reg_t                alloc_phys,
    // Per-physreg ready bits consumed by the IQ; p0 is forced ready.
    output logic [OOO_PHYS_REGS-1:0] ready_vec
);

    word_t                    regs_q[OOO_PHYS_REGS];
    logic [OOO_PHYS_REGS-1:0] ready_q;
    integer reg_rst;

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
            if (write_en && waddr != '0) begin
                regs_q[waddr]  <= wdata;
                ready_q[waddr] <= 1'b1;
            end
            // Newly allocated destination is busy until its producer writes back.
            // (alloc_phys and waddr are never the same physreg in the same cycle:
            // a reg is allocated at dispatch and written back many cycles later.)
            if (alloc_en && alloc_phys != '0) begin
                ready_q[alloc_phys] <= 1'b0;
            end
        end
    end

endmodule
