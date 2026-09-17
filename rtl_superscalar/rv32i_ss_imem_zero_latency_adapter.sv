// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// rv32i_ss_imem_zero_latency_adapter - compatibility environment for the
// instruction request/response port.
//
// The adapter is stateless. It accepts every request and returns the
// backing-store line on the same cycle. The CPU owns request identity,
// queue storage and redirect handling.

module rv32i_ss_imem_zero_latency_adapter
  import rv32i_ss_pkg::*;
(
  input  logic        imem_req_valid,
  output logic        imem_req_ready,
  input  word_t       imem_req_addr,
  output logic        imem_resp_valid,
  input  logic        imem_resp_ready,
  output word_t [1:0] imem_resp_data,

  output word_t       line_addr,
  input  word_t [1:0] line_data
);

  assign imem_req_ready  = 1'b1;
  assign imem_resp_valid = imem_req_valid;
  assign imem_resp_data  = line_data;
  assign line_addr       = imem_req_addr;

  // The frontend reserves a queue slot before request fire, so the
  // same-cycle response is accepted whenever this adapter accepts a request.
  // Keep the port referenced in synthesis/lint without adding adapter state.
  logic unused_resp_ready;
  assign unused_resp_ready = imem_resp_ready;

endmodule
