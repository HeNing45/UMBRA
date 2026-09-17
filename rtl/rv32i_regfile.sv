// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// 32x32 RV32 register file.
//   - Synchronous write on posedge clk when write_en && rd_addr != 0.
//   - Asynchronous reads on rs1_addr / rs2_addr.
//   - x0 is hardwired to zero.
// The pipeline core handles same-cycle W -> D dependence with an external
// bypass mux on the D-stage read ports, so this regfile stays read-from-
// storage and is safe to reuse from the single-cycle core too.
module rv32i_regfile
  import fyp_cpu_pkg::*;
(
  input  logic      clk,
  input  logic      rst_n,
  input  logic      write_en,
  input  reg_addr_t rs1_addr,
  input  reg_addr_t rs2_addr,
  input  reg_addr_t rd_addr,
  input  word_t     rd_data,
  output word_t     rs1_data,
  output word_t     rs2_data
);
  word_t regs [32];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < 32; i++) begin
        regs[i] <= 32'b0;
      end
    end else if (write_en && (rd_addr != 5'd0)) begin
      regs[rd_addr] <= rd_data;
    end
  end

  always_comb begin
    rs1_data = (rs1_addr == 5'd0) ? '0 : regs[rs1_addr];
    rs2_data = (rs2_addr == 5'd0) ? '0 : regs[rs2_addr];
  end
endmodule
