`timescale 1ns/1ps

// Instruction scratchpad identity, latency and pipelining tests.
//
// SECTION A - the identity gate. rv32i_ss_imem_scratchpad #(.LATENCY(0))
// is run side by side with the rv32i_ss_imem_zero_latency_adapter it replaces,
// under one shared stimulus, and all four outputs are compared at two stable
// sample points per cycle for the WHOLE simulation. The comparison uses ===,
// so X behaviour has to match too: a candidate that resolved an X the adapter
// propagates would be a different environment, not an equivalent one. Each DUT
// reads the backing store through its OWN line_addr, so an address difference
// shows up twice - directly, and through the data it fetches.
//
// SECTION B - LATENCY = 1: ready when idle, response at T+1,
// hold until accepted, data captured AT
// acceptance, no second acceptance while a response is live, and the
// two-cycle acceptance period under continuous traffic. Section A's
// checker keeps running underneath section B, so the identity pair is also
// held to its contract while idle.
//
// The backing store is a function of address AND a mutable epoch. That makes
// the capture obligation testable - the epoch is stepped while a response is
// outstanding - and it keeps section A honest, because a candidate that
// registered the line instead of passing it through would diverge the first
// time the epoch moved under a held address.

module tb_rv32i_ss_imem_scratchpad;
  import rv32i_ss_pkg::*;

  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  integer checks    = 0;
  integer mismatch  = 0;
  integer mem_epoch = 0;

  function automatic word_t [1:0] mem_line(input word_t a);
    word_t base;
    begin
      base     = {a[31:3], 3'b000};
      mem_line = {base ^ 32'hA5A5_5A5A ^ word_t'(mem_epoch),
                  base + 32'h0000_0004 + word_t'(mem_epoch)};
    end
  endfunction

  // ---------------------------------------------------------------- section A
  logic  a_req_valid  = 1'b0;
  word_t a_req_addr   = '0;
  logic  a_resp_ready = 1'b1;

  logic        ref_req_ready, ref_resp_valid;
  word_t [1:0] ref_resp_data;
  word_t       ref_line_addr;
  word_t [1:0] ref_line_data;

  logic        dut_req_ready, dut_resp_valid;
  word_t [1:0] dut_resp_data;
  word_t       dut_line_addr;
  word_t [1:0] dut_line_data;

  assign ref_line_data = mem_line(ref_line_addr);
  assign dut_line_data = mem_line(dut_line_addr);

  rv32i_ss_imem_zero_latency_adapter u_ref (
    .imem_req_valid(a_req_valid), .imem_req_ready(ref_req_ready),
    .imem_req_addr(a_req_addr), .imem_resp_valid(ref_resp_valid),
    .imem_resp_ready(a_resp_ready), .imem_resp_data(ref_resp_data),
    .line_addr(ref_line_addr), .line_data(ref_line_data)
  );

  rv32i_ss_imem_scratchpad #(.LATENCY(0)) u_dut0 (
    .clk(clk), .rst_n(rst_n),
    .imem_req_valid(a_req_valid), .imem_req_ready(dut_req_ready),
    .imem_req_addr(a_req_addr), .imem_resp_valid(dut_resp_valid),
    .imem_resp_ready(a_resp_ready), .imem_resp_data(dut_resp_data),
    .line_addr(dut_line_addr), .line_data(dut_line_data)
  );

  task automatic identity_check(input string where);
    begin
      checks = checks + 1;
      if (ref_req_ready  !== dut_req_ready  ||
          ref_resp_valid !== dut_resp_valid ||
          ref_resp_data  !== dut_resp_data  ||
          ref_line_addr  !== dut_line_addr) begin
        mismatch = mismatch + 1;
        $display("IDENTITY MISMATCH @%0t (%s) addr=%h valid=%b rready=%b",
                 $time, where, a_req_addr, a_req_valid, a_resp_ready);
        $display("  ref  ready=%b rvalid=%b rdata=%h laddr=%h",
                 ref_req_ready, ref_resp_valid, ref_resp_data, ref_line_addr);
        $display("  dut  ready=%b rvalid=%b rdata=%h laddr=%h",
                 dut_req_ready, dut_resp_valid, dut_resp_data, dut_line_addr);
      end
    end
  endtask

  // Two stable sample points per cycle, one in each clock phase, so a
  // difference that exists only while the clock is low cannot slip through.
  // Stimulus is driven at negedge; the +1 offset lets it settle first.
  always @(posedge clk) identity_check("posedge");
  always @(negedge clk) begin
    #1;
    identity_check("negedge+1");
  end

  // ---------------------------------------------------------------- section B
  logic  b_req_valid  = 1'b0;
  word_t b_req_addr   = '0;
  logic  b_resp_ready = 1'b1;

  logic        l1_req_ready, l1_resp_valid;
  word_t [1:0] l1_resp_data;
  word_t       l1_line_addr;
  word_t [1:0] l1_line_data;

  assign l1_line_data = mem_line(l1_line_addr);

  rv32i_ss_imem_scratchpad #(.LATENCY(1)) u_dut1 (
    .clk(clk), .rst_n(rst_n),
    .imem_req_valid(b_req_valid), .imem_req_ready(l1_req_ready),
    .imem_req_addr(b_req_addr), .imem_resp_valid(l1_resp_valid),
    .imem_resp_ready(b_resp_ready), .imem_resp_data(l1_resp_data),
    .line_addr(l1_line_addr), .line_data(l1_line_data)
  );

  // ---------------------------------------------------------------- section C
  // Pipelined acceptance is checked alongside the serial PIPELINED=0
  // instance, whose independent checks remain active throughout.
  logic  c_req_valid  = 1'b0;
  word_t c_req_addr   = '0;
  logic  c_resp_ready = 1'b1;

  logic        lp_req_ready, lp_resp_valid;
  word_t [1:0] lp_resp_data;
  word_t       lp_line_addr;
  word_t [1:0] lp_line_data;

  assign lp_line_data = mem_line(lp_line_addr);

  rv32i_ss_imem_scratchpad #(.LATENCY(1), .PIPELINED(1)) u_dutp (
    .clk(clk), .rst_n(rst_n),
    .imem_req_valid(c_req_valid), .imem_req_ready(lp_req_ready),
    .imem_req_addr(c_req_addr), .imem_resp_valid(lp_resp_valid),
    .imem_resp_ready(c_resp_ready), .imem_resp_data(lp_resp_data),
    .line_addr(lp_line_addr), .line_data(lp_line_data)
  );

  integer lp_accepts = 0;
  always @(posedge clk) begin
    if (rst_n && c_req_valid && lp_req_ready) lp_accepts = lp_accepts + 1;
  end

  task automatic expect_eq(input string what, input logic [63:0] got,
                           input logic [63:0] exp);
    begin
      checks = checks + 1;
      if (got !== exp) begin
        mismatch = mismatch + 1;
        $display("CHECK FAIL @%0t %s: got %h expected %h", $time, what,
                 got, exp);
      end
    end
  endtask

  // Count latency-1 acceptances to measure the serial model's two-cycle
  // fire-to-fire period under continuous offered traffic.
  integer l1_accepts = 0;
  always @(posedge clk) begin
    if (rst_n && b_req_valid && l1_req_ready) l1_accepts = l1_accepts + 1;
  end

  // --------------------------------------------------------------- stimulus
  word_t [1:0] captured;
  integer      i;
  integer      period_start, period_cycles;

  initial begin
    // Reset is held across several edges with traffic offered, so the identity
    // pair is compared inside reset too: the adapter has no reset, and a
    // candidate that gated its outputs on rst_n would diverge here.
    a_req_valid = 1'b1;
    a_req_addr  = 32'h0000_0100;
    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);

    // --- Section A: directed corners -------------------------------------
    // Idle.
    a_req_valid = 1'b0; a_req_addr = 32'h0000_0000; a_resp_ready = 1'b1;
    repeat (2) @(negedge clk);

    // Steady request, walking address, including the aligned-line boundary
    // bit and the unaligned low bits the adapter simply forwards.
    a_req_valid = 1'b1;
    for (i = 0; i < 16; i = i + 1) begin
      a_req_addr = word_t'(i * 4);
      @(negedge clk);
    end

    // Address extremes.
    a_req_addr = 32'hFFFF_FFFF; @(negedge clk);
    a_req_addr = 32'h0000_0000; @(negedge clk);
    a_req_addr = 32'h8000_0004; @(negedge clk);

    // resp_ready deasserted: the adapter ignores it, so the candidate must
    // ignore it identically at LATENCY = 0.
    a_resp_ready = 1'b0;
    repeat (4) @(negedge clk);
    a_resp_ready = 1'b1;

    // Backing store moves under a held address. A registered candidate would
    // diverge on the next sample; a pass-through one cannot.
    a_req_addr = 32'h0000_2000;
    @(negedge clk);
    mem_epoch = 1; @(negedge clk);
    mem_epoch = 2; @(negedge clk);
    mem_epoch = 0; @(negedge clk);

    // X propagation, on both the address and the valid.
    a_req_addr = 'x;  @(negedge clk);
    a_req_valid = 1'bx; @(negedge clk);
    a_req_addr = 32'h0000_0040; @(negedge clk);
    a_req_valid = 1'b1; @(negedge clk);

    // Randomized traffic, with occasional X injection.
    for (i = 0; i < 512; i = i + 1) begin
      a_req_valid  = ($random % 4 == 0) ? 1'b0 : 1'b1;
      a_resp_ready = ($random % 3 == 0) ? 1'b0 : 1'b1;
      a_req_addr   = ($random % 32 == 0) ? 'x : word_t'($random);
      if ($random % 64 == 0) mem_epoch = mem_epoch + 1;
      @(negedge clk);
    end
    a_req_valid = 1'b1; a_resp_ready = 1'b1; a_req_addr = 32'h0000_0080;
    mem_epoch = 0;

    // --- Section B: LATENCY = 1 conformance ------------------------------
    // Idle: ready asserted, nothing presented.
    expect_eq("L1 idle ready", l1_req_ready, 1'b1);
    expect_eq("L1 idle resp_valid", l1_resp_valid, 1'b0);

    // Single transaction. Accepted at T, presented at T+1.
    b_req_addr  = 32'h0000_1000;
    b_req_valid = 1'b1;
    b_resp_ready = 1'b1;
    captured = mem_line(32'h0000_1000);
    @(negedge clk);                       // T+1: response cycle
    expect_eq("L1 T+1 resp_valid", l1_resp_valid, 1'b1);
    expect_eq("L1 T+1 resp_data", l1_resp_data, captured);
    expect_eq("L1 T+1 req_ready low", l1_req_ready, 1'b0);
    b_req_valid = 1'b0;
    @(negedge clk);                       // T+2: retired
    expect_eq("L1 T+2 resp_valid", l1_resp_valid, 1'b0);
    expect_eq("L1 T+2 req_ready", l1_req_ready, 1'b1);

    // Backpressure: the response holds, the data holds, and no second request
    // is accepted, for as long as resp_ready stays low.
    b_req_addr   = 32'h0000_3000;
    b_req_valid  = 1'b1;
    b_resp_ready = 1'b0;
    captured     = mem_line(32'h0000_3000);
    @(negedge clk);                       // T+1
    b_req_valid = 1'b1;                   // keep offering a second request
    b_req_addr  = 32'h0000_5000;
    for (i = 0; i < 5; i = i + 1) begin
      expect_eq("L1 stall resp_valid", l1_resp_valid, 1'b1);
      expect_eq("L1 stall resp_data", l1_resp_data, captured);
      expect_eq("L1 stall req_ready low", l1_req_ready, 1'b0);
      // Capture obligation: the store moves under the outstanding response.
      mem_epoch = mem_epoch + 1;
      @(negedge clk);
    end
    expect_eq("L1 held data survives store change", l1_resp_data, captured);
    b_resp_ready = 1'b1;
    @(negedge clk);                       // accepted this cycle
    expect_eq("L1 post-accept resp_valid", l1_resp_valid, 1'b0);
    expect_eq("L1 post-accept req_ready", l1_req_ready, 1'b1);
    b_req_valid = 1'b0;
    mem_epoch   = 0;
    @(negedge clk);

    // Acceptance period with the request held high and responses accepted
    // immediately: the environment must accept exactly every second cycle.
    b_req_addr   = 32'h0000_7000;
    b_resp_ready = 1'b1;
    b_req_valid  = 1'b1;
    @(negedge clk);
    period_start = l1_accepts;
    for (i = 0; i < 20; i = i + 1) @(negedge clk);
    period_cycles = l1_accepts - period_start;
    expect_eq("L1 acceptances in 20 cycles", period_cycles, 10);
    b_req_valid = 1'b0;
    @(negedge clk);

    // Randomized L1 traffic. The invariant that must never break is the
    // one-transaction rule: ready and resp_valid are never both asserted.
    for (i = 0; i < 512; i = i + 1) begin
      b_req_valid  = ($random % 3 == 0) ? 1'b0 : 1'b1;
      b_resp_ready = ($random % 3 == 0) ? 1'b0 : 1'b1;
      b_req_addr   = word_t'($random);
      @(negedge clk);
      checks = checks + 1;
      if (l1_req_ready === 1'b1 && l1_resp_valid === 1'b1) begin
        mismatch = mismatch + 1;
        $display("CHECK FAIL @%0t L1 accepted a request while a response was live",
                 $time);
      end
    end
    b_req_valid = 1'b0;
    repeat (4) @(negedge clk);

    // --- Section C: LATENCY=1, PIPELINED=1 conformance ------
    // idle shape identical to the serial leg.
    expect_eq("P1 idle ready", lp_req_ready, 1'b1);
    expect_eq("P1 idle resp_valid", lp_resp_valid, 1'b0);

    // a single transaction still has latency exactly one — nothing is
    // presented in the acceptance cycle, the line arrives at T+1.
    c_req_addr  = 32'h0000_9000;
    c_req_valid = 1'b1;
    captured = mem_line(32'h0000_9000);
    expect_eq("P1 nothing presented at acceptance", lp_resp_valid, 1'b0);
    @(negedge clk);                       // T+1
    c_req_valid = 1'b0;
    expect_eq("P1 T+1 resp_valid", lp_resp_valid, 1'b1);
    expect_eq("P1 T+1 resp_data", lp_resp_data, captured);
    @(negedge clk);                       // retired
    expect_eq("P1 retired resp_valid", lp_resp_valid, 1'b0);

    // Back-to-back acceptance: with the
    // request held and responses accepted, the leg must accept EVERY cycle
    // (the serial leg accepts every second), ready staying high while each
    // response retires on the same edge; and every presented line must be
    // the one captured at the PREVIOUS cycle's acceptance. Drive convention:
    // an address driven at negedge k is accepted at the following posedge
    // and its line is presented in the next cycle, checked at negedge k+1.
    c_resp_ready = 1'b1;
    c_req_valid  = 1'b1;
    c_req_addr   = 32'h0000_A000;
    captured     = mem_line(32'h0000_A000);
    @(negedge clk);
    period_start = lp_accepts;
    for (i = 0; i < 20; i = i + 1) begin
      expect_eq("P1 handover resp_valid", lp_resp_valid, 1'b1);
      expect_eq("P1 handover presents last capture", lp_resp_data, captured);
      expect_eq("P1 handover ready stays high", lp_req_ready, 1'b1);
      c_req_addr = c_req_addr + 32'd8;
      captured   = mem_line(c_req_addr);
      @(negedge clk);
    end
    period_cycles = lp_accepts - period_start;
    expect_eq("P1 acceptances in 20 cycles", period_cycles, 20);

    // backpressure unchanged — the response and data hold, and because
    // ready is conditioned on the response retiring, no new request is
    // accepted while one is unaccepted.
    c_resp_ready = 1'b0;
    captured = lp_resp_data;
    #1;   // let ready settle on the deasserted resp_ready before checking
    for (i = 0; i < 5; i = i + 1) begin
      expect_eq("P1 stall resp_valid holds", lp_resp_valid, 1'b1);
      expect_eq("P1 stall data holds", lp_resp_data, captured);
      expect_eq("P1 stall ready low", lp_req_ready, 1'b0);
      // capture at acceptance unchanged — the store moves under the
      // held response and the presented line must not follow it.
      mem_epoch = mem_epoch + 1;
      @(negedge clk);
    end
    expect_eq("P1 held data survives store change", lp_resp_data, captured);
    c_resp_ready = 1'b1;
    @(negedge clk);
    c_req_valid = 1'b0;
    mem_epoch   = 0;
    repeat (2) @(negedge clk);
    expect_eq("P1 drained after release", lp_resp_valid, 1'b0);

    if (mismatch != 0) begin
      $display("TB_FAIL mismatches=%0d checks=%0d", mismatch, checks);
      $fatal(1, "tb_rv32i_ss_imem_scratchpad FAILED");
    end
    $display("M3 STEP A: LATENCY=0 identity holds against the adapter");
    $display("PASS checks=%0d", checks);
    $finish;
  end

  initial begin
    #500000;
    $display("TB_FAIL timeout checks=%0d", checks);
    $fatal(1, "tb_rv32i_ss_imem_scratchpad TIMEOUT");
  end

endmodule
