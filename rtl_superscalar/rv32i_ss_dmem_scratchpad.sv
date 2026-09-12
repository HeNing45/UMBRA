`timescale 1ns/1ps

// rv32i_ss_dmem_scratchpad — data-memory timing environment.
// This simulation model connects the CPU to a backing store. It is outside
// the CPU synthesis boundary. mem_en pulses only when a request is accepted,
// ensuring a stalled store writes exactly once.
//
// READY_STALL controls request backpressure; RESP_LATENCY controls read
// response delay. Separate parameters exercise payload stability and response
// identity independently. Zero for both selects always-ready, combinational
// reads. Delayed responses capture data at request acceptance and return in
// acceptance order. MAX_OUTSTANDING checks the chosen CPU read-capacity limit.

module rv32i_ss_dmem_scratchpad
  import rv32i_ss_pkg::*;
#(
  // 0 = always ready (identity). N > 0 = after each acceptance, withhold
  // dmem_ready for N cycles before accepting again.
  parameter int READY_STALL  = 0,
  // Read-response delay: zero returns combinationally; one returns a captured
  // value next cycle; larger values use a pipeline of that depth.
  parameter int RESP_LATENCY = 0,
  // Maximum accepted reads awaiting responses. The model reports a fatal
  // error if the configured limit is exceeded.
  parameter int MAX_OUTSTANDING = 1,
  // Request class subject to stalls: 0 = all, 1 = reads, 2 = writes.
  // Class selection isolates load and store handshake behavior.
  parameter int STALL_SEL    = 0
) (
  input  logic        clk,
  input  logic        rst_n,

  // CPU side - handshake, producer-owned request and read response.
  input  logic        dmem_valid,
  input  logic        dmem_we,
  input  logic [3:0]  dmem_be,
  input  word_t       dmem_addr,
  input  word_t       dmem_wdata,
  output logic        dmem_ready,
  output logic        dmem_rvalid,
  output word_t       dmem_rdata,

  // Backing-store side. mem_en is an ACCEPTANCE strobe, not a request mirror.
  output logic        mem_en,
  output logic        mem_we,
  output logic [3:0]  mem_be,
  output word_t       mem_addr,
  output word_t       mem_wdata,
  input  word_t       mem_rdata
);

  initial begin
    if (READY_STALL < 0) begin
      $fatal(1, "rv32i_ss_dmem_scratchpad: READY_STALL must be >= 0, got %0d",
             READY_STALL);
    end
    // Response delay must be nonnegative. Runtime tracking separately checks
    // outstanding-read capacity against MAX_OUTSTANDING.
    if (RESP_LATENCY < 0) begin
      $fatal(1, "rv32i_ss_dmem_scratchpad: RESP_LATENCY must be >= 0, got %0d",
             RESP_LATENCY);
    end
    if (MAX_OUTSTANDING < 1) begin
      $fatal(1, "rv32i_ss_dmem_scratchpad: MAX_OUTSTANDING must be >= 1, got %0d",
             MAX_OUTSTANDING);
    end
  end

  logic accept;
  assign accept = dmem_valid && dmem_ready;

  // The store-side request is the presented request, strobed at acceptance.
  assign mem_en    = accept;
  assign mem_we    = dmem_we;
  assign mem_be    = dmem_be;
  assign mem_addr  = dmem_addr;
  assign mem_wdata = dmem_wdata;

  // ------------------------------------------------------------------ ready
  generate
    if (READY_STALL == 0) begin : g_always_ready
      assign dmem_ready = 1'b1;
    end else begin : g_stalled_ready
      // Only the selected class can be stalled; every other request is
      // accepted immediately, so the two halves of the port can be diagnosed
      // independently.
      logic stall_applies;
      assign stall_applies = (STALL_SEL == 0) ||
                             ((STALL_SEL == 1) && !dmem_we) ||
                             ((STALL_SEL == 2) &&  dmem_we);
      // Withhold ready for READY_STALL cycles after each acceptance. Simple and
      // deterministic on purpose: a directed environment should be reproducible,
      // and randomized backpressure belongs in the battery's stimulus, not in
      // the model's own timing.
      logic [15:0] stall_ctr_q;
      assign dmem_ready = (stall_ctr_q == 16'd0) || !stall_applies;
      always_ff @(posedge clk or negedge rst_n) begin : p_stall
        // Reload only for the selected request class. Other accepted requests
        // must not extend that class's configured stall window.
        if (!rst_n)                        stall_ctr_q <= 16'd0;
        else if (accept && stall_applies)  stall_ctr_q <= 16'(READY_STALL);
        else if (|stall_ctr_q)             stall_ctr_q <= stall_ctr_q - 16'd1;
      end
    end
  endgenerate

  // --------------------------------------------------------------- response
  generate
    if (RESP_LATENCY == 0) begin : g_same_cycle_resp
      // rvalid is always asserted; read data is combinational from the backing store.
      assign dmem_rvalid = 1'b1;
      assign dmem_rdata  = mem_rdata;
    end else if (RESP_LATENCY == 1) begin : g_delayed_resp
      // A read accepted at T returns at T+1, with the line CAPTURED AT
      // ACCEPTANCE so a later change to the backing store cannot alter a
      // response already in flight.
      logic  read_pending_q;
      word_t rdata_q;
      assign dmem_rvalid = read_pending_q;
      assign dmem_rdata  = rdata_q;
      always_ff @(posedge clk or negedge rst_n) begin : p_resp
        if (!rst_n) begin
          read_pending_q <= 1'b0;
          rdata_q        <= '0;
        end else begin
          read_pending_q <= accept && !dmem_we;
          if (accept && !dmem_we) rdata_q <= mem_rdata;
        end
      end
    end else begin : g_pipe_resp
      // RESP_LATENCY >= 2. A read accepted at T returns at exactly T+L, data
      // CAPTURED AT ACCEPTANCE and carried through the pipe, so a later change
      // to the backing store cannot alter a response already in flight.
      //
      // Depth L is what makes K_cpu > 1 testable: a single pending-response
      // register is sufficient ONLY under one-outstanding. Against a K=2 CPU
      // two reads are legitimately in flight and a depth-1 model would drop
      // one. Each stage is independent, so the pipe holds up to L responses
      // and every entry emerges EXACTLY ONCE.
      logic  pend_v_q [RESP_LATENCY];
      word_t pend_d_q [RESP_LATENCY];
      logic  read_accept;
      int    inflight;

      assign read_accept  = accept && !dmem_we;
      assign dmem_rvalid  = pend_v_q[RESP_LATENCY-1];
      assign dmem_rdata   = pend_d_q[RESP_LATENCY-1];

      always_comb begin : p_inflight
        inflight = 0;
        for (int i = 0; i < RESP_LATENCY; i++)
          if (pend_v_q[i]) inflight++;
      end

      always_ff @(posedge clk or negedge rst_n) begin : p_pipe
        if (!rst_n) begin
          // Reset cancellation: every in-flight response is discarded.
          for (int i = 0; i < RESP_LATENCY; i++) begin
            pend_v_q[i] <= 1'b0;
            pend_d_q[i] <= '0;
          end
        end else begin
          for (int i = RESP_LATENCY - 1; i > 0; i--) begin
            pend_v_q[i] <= pend_v_q[i-1];
            pend_d_q[i] <= pend_d_q[i-1];
          end
          pend_v_q[0] <= read_accept;
          if (read_accept) pend_d_q[0] <= mem_rdata;

          // Capacity pin. `inflight` counts entries already in the pipe; the
          // read accepted this cycle joins them, and the one leaving on this
          // edge does not. A violation means the CPU exceeded its contract.
          if ((inflight + (read_accept ? 1 : 0)
               - (pend_v_q[RESP_LATENCY-1] ? 1 : 0)) > MAX_OUTSTANDING) begin
            $fatal(1,
              "rv32i_ss_dmem_scratchpad: %0d reads in flight exceeds MAX_OUTSTANDING=%0d",
              inflight + (read_accept ? 1 : 0)
                - (pend_v_q[RESP_LATENCY-1] ? 1 : 0), MAX_OUTSTANDING);
          end
        end
      end
    end
  endgenerate

  // ------------------------------------------------- producer obligation pin
  // "producer-owned request valid and payload remain stable until
  // acceptance." That is a CPU obligation, and the environment is the natural
  // place to pin it — a violation is otherwise invisible until it corrupts a
  // location. Inert when ready is never withheld, which is why it costs the
  // identity leg nothing.
  logic        held_valid_q;
  logic        held_we_q;
  logic [3:0]  held_be_q;
  word_t       held_addr_q;
  word_t       held_wdata_q;

  always_ff @(posedge clk or negedge rst_n) begin : p_stability
    if (!rst_n) begin
      held_valid_q <= 1'b0;
      held_we_q    <= 1'b0;
      held_be_q    <= 4'd0;
      held_addr_q  <= '0;
      held_wdata_q <= '0;
    end else begin
      if (held_valid_q) begin
        if (dmem_we    !== held_we_q   || dmem_be    !== held_be_q ||
            dmem_addr  !== held_addr_q || dmem_wdata !== held_wdata_q ||
            dmem_valid !== 1'b1) begin
          $fatal(1, "rv32i_ss_dmem_scratchpad: request payload changed before acceptance");
        end
      end
      // Capture the first unaccepted presentation. Compare every later cycle
      // against that snapshot so the first stalled cycle cannot hide a payload
      // change by replacing the reference value.
      if (accept) begin
        held_valid_q <= 1'b0;
      end else if (dmem_valid && !held_valid_q) begin
        held_valid_q <= 1'b1;
        held_we_q    <= dmem_we;
        held_be_q    <= dmem_be;
        held_addr_q  <= dmem_addr;
        held_wdata_q <= dmem_wdata;
      end
    end
  end

  // Responses carry read data. With zero latency, rvalid is continuously
  // asserted and the CPU qualifies it by read identity. Delayed modes pulse
  // responses for accepted reads only. Writes take effect at acceptance and
  // have no separate response. Request stalls and response timing are independent.

endmodule
