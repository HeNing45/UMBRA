`timescale 1ns/1ps

// rv32i_ss_imem_scratchpad — instruction-memory timing environment.
// This simulation model sits outside the CPU synthesis boundary and reads
// aligned lines from a backing store.
//
// LATENCY=0 accepts every request and returns its line combinationally.
// LATENCY=1 captures line_data at acceptance, presents it the next cycle,
// and holds response valid and data until the CPU accepts them.
// At most one accepted request is represented. With PIPELINED=0, a pending
// response blocks acceptance. With PIPELINED=1, legal only at LATENCY=1,
// response retirement and replacement acceptance may share an edge.
//
// The CPU owns redirect/kill semantics and drains killed responses. Memory
// therefore has no flush input and never cancels an accepted transaction.

module rv32i_ss_imem_scratchpad
  import rv32i_ss_pkg::*;
#(
  parameter int LATENCY   = 0,
  // Permit response retirement and request acceptance on the same edge.
  parameter int PIPELINED = 0
) (
  input  logic        clk,
  input  logic        rst_n,

  input  logic        imem_req_valid,
  output logic        imem_req_ready,
  input  word_t       imem_req_addr,
  output logic        imem_resp_valid,
  input  logic        imem_resp_ready,
  output word_t [1:0] imem_resp_data,

  output word_t       line_addr,
  input  word_t [1:0] line_data
);

  // Contract fixes the legal values at 0 and 1. Without this an
  // out-of-range LATENCY would silently elaborate into the registered leg and
  // then misreport its own timing, which is the one failure mode a timing
  // model must not have.
  initial begin
    if (LATENCY != 0 && LATENCY != 1) begin
      $fatal(1, "rv32i_ss_imem_scratchpad: LATENCY must be 0 or 1, got %0d",
             LATENCY);
    end
    // PIPELINED=1 is legal ONLY with LATENCY=1, for the same
    // reason the LATENCY check exists — a timing model must not silently
    // misreport its own timing.
    if (PIPELINED != 0 && PIPELINED != 1) begin
      $fatal(1, "rv32i_ss_imem_scratchpad: PIPELINED must be 0 or 1, got %0d",
             PIPELINED);
    end
    if (PIPELINED == 1 && LATENCY != 1) begin
      $fatal(1, "rv32i_ss_imem_scratchpad: PIPELINED=1 is legal only with LATENCY=1");
    end
  end

  // The addressed line is presented combinationally in both legs. At
  // LATENCY = 1 that is the acceptance-cycle read whose result is captured
  // below; the address is not held afterwards because the DATA is held.
  assign line_addr = imem_req_addr;

  generate
    if (LATENCY == 0) begin : g_zero_latency
      // Accept every request and return its line on the request-fire cycle.
      assign imem_req_ready  = 1'b1;
      assign imem_resp_valid = imem_req_valid;
      assign imem_resp_data  = line_data;

      // The frontend reserves a queue slot before request fire, so the
      // same-cycle response is accepted whenever this leg accepts a request.
      // Keep the ports this leg does not use referenced, without adding state.
      logic unused_zero_latency;
      assign unused_zero_latency = imem_resp_ready ^ clk ^ rst_n;

    end else begin : g_registered
      logic        resp_valid_q;
      word_t [1:0] resp_data_q;

      if (PIPELINED == 0) begin : g_serial_ready
        // One-deep, non-pipelined: a presented response blocks request acceptance.
        assign imem_req_ready = !resp_valid_q;
      end else begin : g_pipelined_ready
        // ready stays asserted while a response is presented
        // PROVIDED that response is being accepted this cycle — one
        // acceptance and one retirement handed over on the same edge, a
        // sustained rate of one per cycle. Under backpressure, an
        // unaccepted response forces ready low and throttles the loop.
        assign imem_req_ready = !resp_valid_q || imem_resp_ready;
      end

      assign imem_resp_valid = resp_valid_q;
      assign imem_resp_data  = resp_data_q;

      // Acceptance takes priority over response clear. In pipelined mode,
      // both events may coincide: the outgoing response is consumed while its
      // storage is replaced by the newly accepted line. A stalled response
      // blocks acceptance and retains its payload.
      always_ff @(posedge clk or negedge rst_n) begin : p_response
        if (!rst_n) begin
          resp_valid_q <= 1'b0;
          resp_data_q  <= '0;
        end else if (imem_req_valid && imem_req_ready) begin
          resp_valid_q <= 1'b1;
          resp_data_q  <= line_data;
        end else if (resp_valid_q && imem_resp_ready) begin
          // Backpressure: while a presented response is unaccepted, both the
          // valid and the data hold and no new request is accepted.
          resp_valid_q <= 1'b0;
        end
      end
    end
  endgenerate

endmodule
