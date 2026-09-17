// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// ooo_dmem_model — reusable data-memory model for the OoO core testbenches.
//
// Combinational single-cycle read, no req/ready/resp
// handshake. A clocked byte-enable write, plus a tohost watch for riscv-tests.
//
// Deliberately package-import-free so it drops into any OoO harness. Its
// apply_be, raw-word reads and byte-enable writes provide a consistent
// memory model across the core testbenches.
//
// Division of labour (head-only LSU contract):
//   - The OoO LSU owns sub-word alignment: it lane-shifts store data + forms the
//     4-bit byte mask on the store side, and lane-extracts + sign/zero-extends on
//     the load side. This model only ever sees raw 32-bit words + a byte mask.
//   - `addr` is a BYTE address; the word index is addr[MEM_MSB:2]. addr[1:0] is
//     ignored here (the LSU already reflected it into `be` / the load extract).
//
// tohost: `tohost_we` pulses for exactly one cycle on any store that lands on
// tohost_addr (or its alias tohost_full_addr); `tohost_val` is the resulting
// word AFTER the byte-enable merge. The harness — not this model — decides
// PASS (==1) / FAIL (!=0) so the model stays judgement-free and reusable.

module ooo_dmem_model #(
  parameter int MEM_WORDS = 65536,       // 256 KiB default (matches in-order TB)
  parameter int MEM_MSB   = 17           // byte-addr MSB; word index = addr[MEM_MSB:2]
)(
  input  logic        clk,
  input  logic        rst_n,

  // combinational read (raw word at addr[MEM_MSB:2])
  input  logic [31:0] addr,
  output logic [31:0] rdata,

  // clocked byte-enable write
  input  logic        we,
  input  logic [3:0]  be,
  input  logic [31:0] wdata,

  // tohost oracle
  input  logic [31:0] tohost_addr,
  input  logic [31:0] tohost_full_addr,
  output logic        tohost_we,         // 1-cycle pulse on a store to tohost
  output logic [31:0] tohost_val         // post-byte-merge word written to tohost
);

  logic [31:0] mem [0:MEM_WORDS-1];

  // ---- combinational raw-word read ----
  assign rdata = (!$isunknown(addr[MEM_MSB:2])) ? mem[addr[MEM_MSB:2]]
                                                : 32'h0000_0000;

  // ---- byte-enable merge (identical to the in-order harness apply_be) ----
  function automatic logic [31:0] apply_be(input logic [31:0] cur,
                                           input logic [31:0] data,
                                           input logic [3:0]  msk);
    apply_be = cur;
    if (msk[0]) apply_be[7:0]   = data[7:0];
    if (msk[1]) apply_be[15:8]  = data[15:8];
    if (msk[2]) apply_be[23:16] = data[23:16];
    if (msk[3]) apply_be[31:24] = data[31:24];
  endfunction

  // ---- clocked write + tohost watch ----
  always_ff @(posedge clk) begin
    logic [31:0] merged;
    tohost_we <= 1'b0;                    // default: no tohost store this cycle
    if (rst_n && we && !$isunknown(addr[MEM_MSB:2])) begin
      merged = apply_be(mem[addr[MEM_MSB:2]], wdata, be);
      mem[addr[MEM_MSB:2]] <= merged;
      if ((addr == tohost_addr) || (addr == tohost_full_addr)) begin
        tohost_we  <= 1'b1;
        tohost_val <= merged;
      end
    end
  end

  // ---- init: zero-fill so untouched reads are defined (harness may $readmemh over it) ----
  integer i;
  initial begin
    for (i = 0; i < MEM_WORDS; i = i + 1) mem[i] = 32'h0000_0000;
    tohost_we  = 1'b0;
    tohost_val = 32'h0000_0000;
  end

endmodule
