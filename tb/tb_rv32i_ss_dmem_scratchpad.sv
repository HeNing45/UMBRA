`timescale 1ns/1ps

// data-side environment battery. Three obligations, one simulation.
//
// SECTION A - the identity gate. rv32i_ss_dmem_scratchpad #(0, 0) is compared
// against the hand-tied environment every existing testbench uses today
// (dmem_ready = 1, dmem_rvalid = 1, rdata combinational, backing store written
// on dmem_valid && dmem_we). All five observable outputs plus the store-side
// strobe are compared with === at two stable sample points per cycle for the
// whole simulation. This is what lets the model be dropped into a CPU-level
// testbench with zero behavioural change.
//
// SECTION B - withheld ready. Ready deasserts for READY_STALL cycles after each
// acceptance, and the WRITE-ONCE law holds: a request held up across several
// stalled cycles must strobe the backing store exactly once, on the acceptance
// cycle. A model keyed off dmem_valid instead of acceptance writes once per
// stalled cycle and silently corrupts memory; that is the single most important
// property in this file.
//
// SECTION C - delayed response. A read accepted at T returns at T+1 with the
// data CAPTURED AT ACCEPTANCE. The backing store is mutated under the pending
// response to distinguish capture from a live read — the I-side battery needed
// exactly this and learned that a function-based live read is invisible under
// Icarus's operand-only sensitivity, so the store here is a real array whose
// reads are genuinely re-evaluated.
//
// The producer-obligation pin inside the model (payload stable until accepted)
// is exercised throughout section B and RED-proven by the mutation harness
// rather than by a protocol violation committed here.

module tb_rv32i_ss_dmem_scratchpad;
  import rv32i_ss_pkg::*;

  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  integer checks = 0, errors = 0;
  task automatic chk(input string name, input logic cond);
    begin
      checks = checks + 1;
      if (cond !== 1'b1) begin
        errors = errors + 1;
        $display("  FAIL [%0d] @%0t %s", checks, $time, name);
      end
    end
  endtask

  // ------------------------------------------------------- backing store
  localparam int MEM_WORDS = 256;
  word_t mem [0:MEM_WORDS-1];
  integer i;

  function automatic int widx(input word_t a);
    widx = int'(a[9:2]);
  endfunction

  // ------------------------------------------------------------ stimulus
  logic  d_valid = 1'b0, d_we = 1'b0;
  logic [3:0] d_be = 4'hF;
  word_t d_addr = '0, d_wdata = '0;

  // -------------------------------------------------- A: identity instance
  logic  a_ready, a_rvalid, a_en, a_we;
  logic [3:0] a_be;
  word_t a_rdata, a_addr, a_wdata;

  rv32i_ss_dmem_scratchpad #(.READY_STALL(0), .RESP_LATENCY(0)) u_id (
    .clk(clk), .rst_n(rst_n),
    .dmem_valid(d_valid), .dmem_we(d_we), .dmem_be(d_be),
    .dmem_addr(d_addr), .dmem_wdata(d_wdata),
    .dmem_ready(a_ready), .dmem_rvalid(a_rvalid), .dmem_rdata(a_rdata),
    .mem_en(a_en), .mem_we(a_we), .mem_be(a_be),
    .mem_addr(a_addr), .mem_wdata(a_wdata),
    .mem_rdata(mem[widx(a_addr)])
  );

  // The hand-tied reference, as an expression set rather than a module: this
  // is literally what every CPU-level testbench ties today.
  wire   ref_ready  = 1'b1;
  wire   ref_rvalid = 1'b1;
  wire   ref_en     = d_valid;
  word_t ref_rdata; assign ref_rdata = mem[widx(d_addr)];

  task automatic identity_check(input string where);
    begin
      checks = checks + 1;
      if (a_ready  !== ref_ready  || a_rvalid !== ref_rvalid ||
          a_rdata  !== ref_rdata  || a_en     !== ref_en     ||
          a_addr   !== d_addr     || a_we     !== d_we       ||
          a_be     !== d_be       || a_wdata  !== d_wdata) begin
        errors = errors + 1;
        $display("IDENTITY MISMATCH @%0t (%s) ready %b/%b rvalid %b/%b en %b/%b",
                 $time, where, a_ready, ref_ready, a_rvalid, ref_rvalid,
                 a_en, ref_en);
      end
    end
  endtask
  always @(posedge clk) if (rst_n) identity_check("posedge");
  always @(negedge clk) begin #1; if (rst_n) identity_check("negedge+1"); end

  // ------------------------------------------------ B: withheld-ready instance
  // Driven by its OWN stimulus, not section A's. A stalling environment imposes
  // a producer obligation that an always-ready one does not — the request must
  // hold stable until accepted — so the two legs cannot share a driver. This
  // separation is not tidiness: the model's stability pin caught section A's
  // free-running stimulus the first time this battery ran, which is the same
  // class of finding as the pipelined discovery that constant-ready testbench
  // environments became contract-illegal.
  logic  b_valid = 1'b0, b_we_i = 1'b0;
  logic [3:0] b_be_i = 4'hF;
  word_t b_addr_i = '0, b_wdata_i = '0;

  logic  b_ready, b_rvalid, b_en, b_we;
  logic [3:0] b_be;
  word_t b_rdata, b_addr, b_wdata;

  rv32i_ss_dmem_scratchpad #(.READY_STALL(2), .RESP_LATENCY(0)) u_stall (
    .clk(clk), .rst_n(rst_n),
    .dmem_valid(b_valid), .dmem_we(b_we_i), .dmem_be(b_be_i),
    .dmem_addr(b_addr_i), .dmem_wdata(b_wdata_i),
    .dmem_ready(b_ready), .dmem_rvalid(b_rvalid), .dmem_rdata(b_rdata),
    .mem_en(b_en), .mem_we(b_we), .mem_be(b_be),
    .mem_addr(b_addr), .mem_wdata(b_wdata),
    .mem_rdata(mem[widx(b_addr)])
  );

  // The write-once witness: count store-side strobes for the stalled model.
  integer b_strobes = 0;
  always @(posedge clk) if (rst_n && b_en) b_strobes = b_strobes + 1;

  // ---------------------------------------------- C: delayed-response instance
  logic  c_valid = 1'b0, c_we = 1'b0;
  logic [3:0] c_be = 4'hF;
  word_t c_addr = '0, c_wdata = '0;
  logic  c_ready, c_rvalid, c_en, c_we_o;
  logic [3:0] c_be_o;
  word_t c_rdata, c_addr_o, c_wdata_o;

  rv32i_ss_dmem_scratchpad #(.READY_STALL(0), .RESP_LATENCY(1)) u_delay (
    .clk(clk), .rst_n(rst_n),
    .dmem_valid(c_valid), .dmem_we(c_we), .dmem_be(c_be),
    .dmem_addr(c_addr), .dmem_wdata(c_wdata),
    .dmem_ready(c_ready), .dmem_rvalid(c_rvalid), .dmem_rdata(c_rdata),
    .mem_en(c_en), .mem_we(c_we_o), .mem_be(c_be_o),
    .mem_addr(c_addr_o), .mem_wdata(c_wdata_o),
    .mem_rdata(mem[widx(c_addr_o)])
  );

  // ------------------------------------------------------- store-side writes
  // The backing store is written from the IDENTITY model's acceptance strobe,
  // so section A's traffic maintains real memory contents for every leg to
  // read. Sections B and C write through their own strobes where they need to.
  always @(posedge clk) begin
    if (rst_n && a_en && a_we) mem[widx(a_addr)] <= a_wdata;
    if (rst_n && b_en && b_we) mem[widx(b_addr)] <= b_wdata;
  end

  // ------------------------------------------------------ entry-proof counters
  integer n_stall_cycles = 0, n_stall_accepts = 0, n_delayed_resp = 0;
  always @(posedge clk) if (rst_n) begin
    if (!b_ready)                 n_stall_cycles = n_stall_cycles + 1;
    if (b_valid && b_ready)       n_stall_accepts = n_stall_accepts + 1;
    if (c_rvalid)                 n_delayed_resp = n_delayed_resp + 1;
  end

  // A protocol-correct producer for the stalling leg: present the request and
  // hold valid AND payload absolutely stable until the edge that accepts it,
  // then release. Written as a task rather than open-coded cycle counts because
  // the first version of this battery asserted where acceptance "should" land
  // and was wrong; observing the handshake cannot be wrong.
  task automatic b_request(input logic we, input word_t addr, input word_t wd);
    begin
      @(negedge clk);
      b_we_i = we; b_addr_i = addr; b_wdata_i = wd; b_be_i = 4'hF;
      b_valid = 1'b1;
      forever begin
        @(posedge clk);
        if (b_ready === 1'b1) break;   // accepted on this edge
      end
      @(negedge clk);
      b_valid = 1'b0;
    end
  endtask

  // ---------------------------------------------------------------- stimulus
  word_t captured;
  integer strobes_before;
  integer stall_len;

  initial begin
    for (i = 0; i < MEM_WORDS; i = i + 1) mem[i] = word_t'(32'hA000_0000 + i);
    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);

    // ---- Section A: identity traffic, reads and writes -------------------
    for (i = 0; i < 24; i = i + 1) begin
      d_valid = (i % 5 != 0);
      d_we    = (i % 3 == 0);
      d_addr  = word_t'(32'h0000_0040 + (i * 4));
      d_wdata = word_t'(32'hC0DE_0000 + i);
      d_be    = 4'hF;
      @(negedge clk);
    end
    d_valid = 1'b0;
    repeat (2) @(negedge clk);

    // ---- Section B: withheld ready, and the write-once law ---------------
    // The property that matters: a request held across a stall window strobes
    // the backing store EXACTLY ONCE, on the acceptance edge. A model keyed off
    // dmem_valid rather than acceptance writes once per stalled cycle.
    strobes_before = b_strobes;
    b_request(1'b1, 32'h0000_0080, 32'hBEEF_0001);
    chk("B held write strobes the store exactly once",
        (b_strobes - strobes_before) == 1);

    // Back-to-back requests each strobe exactly once, and the stall window
    // separates them.
    strobes_before = b_strobes;
    b_request(1'b1, 32'h0000_0084, 32'hBEEF_0002);
    b_request(1'b1, 32'h0000_0088, 32'hBEEF_0003);
    chk("B two requests strobe exactly twice",
        (b_strobes - strobes_before) == 2);

    // A read through the stalling leg returns the stored value.
    b_request(1'b0, 32'h0000_0080, 32'h0);
    chk("B stalled read sees the value its own write stored",
        mem[widx(32'h0000_0080)] === 32'hBEEF_0001);

    // Pin the stall window LENGTH, not merely that ready was withheld. A model
    // that stalls for the wrong number of cycles still satisfies "ready went
    // low", which is why the entry proof alone is not enough.
    @(negedge clk);
    b_we_i = 1'b1; b_addr_i = 32'h0000_008C; b_wdata_i = 32'hBEEF_0004;
    b_be_i = 4'hF; b_valid = 1'b1;
    forever begin
      @(posedge clk);
      if (b_ready === 1'b1) break;          // accepted on this edge
    end
    @(negedge clk);
    b_valid = 1'b0;
    stall_len = 0;
    #1;
    while (b_ready === 1'b0) begin
      stall_len = stall_len + 1;
      @(negedge clk); #1;
    end
    chk("B stall window is exactly READY_STALL cycles long", stall_len == 2);

    // Idle: no request, no strobe, and ready recovers to high.
    strobes_before = b_strobes;
    repeat (6) @(negedge clk); #1;
    chk("B idle produces no store strobes", (b_strobes - strobes_before) == 0);
    chk("B ready is high when idle", b_ready === 1'b1);

    // ---- Section C: delayed response, captured at acceptance -------------
    // Timed by observation, in phase with the drive. The response must not
    // appear before the acceptance edge, must appear on the next one, and must
    // carry the value the store held AT ACCEPTANCE even though the store is
    // mutated underneath the pending response.
    c_addr   = 32'h0000_00C0;
    c_we     = 1'b0;
    captured = mem[widx(32'h0000_00C0)];
    @(negedge clk);
    c_valid = 1'b1;
    #1;
    chk("C no response before the acceptance edge", c_rvalid === 1'b0);
    @(posedge clk);                       // accepted on this edge
    @(negedge clk);
    c_valid = 1'b0;
    mem[widx(32'h0000_00C0)] = 32'hDEAD_BEEF;   // moved under the response
    #1;
    chk("C response presented one cycle after acceptance", c_rvalid === 1'b1);
    chk("C data is the value captured AT ACCEPTANCE, not a live read",
        c_rdata === captured);
    @(negedge clk); #1;
    chk("C response deasserts after one cycle", c_rvalid === 1'b0);
    mem[widx(32'h0000_00C0)] = captured;

    // A write must not raise the read response.
    @(negedge clk);
    c_addr = 32'h0000_00C4; c_we = 1'b1; c_wdata = 32'h5555_0001;
    c_valid = 1'b1;
    @(posedge clk);
    @(negedge clk);
    c_valid = 1'b0;
    #1;
    chk("C a write raises no read response", c_rvalid === 1'b0);

    repeat (3) @(negedge clk);

    chk("B entered: ready was actually withheld", n_stall_cycles >= 2);
    if (n_stall_cycles < 2 || n_stall_accepts < 1 || n_delayed_resp < 1) begin
      $display("TB_FAIL leg not entered: stall=%0d accepts=%0d delayed=%0d",
               n_stall_cycles, n_stall_accepts, n_delayed_resp);
      $fatal(1, "tb_rv32i_ss_dmem_scratchpad: a leg failed to enter its state");
    end
    if (errors != 0) begin
      $display("TB_FAIL errors=%0d checks=%0d", errors, checks);
      $fatal(1, "tb_rv32i_ss_dmem_scratchpad FAILED");
    end
    $display("M4 environment: identity holds, write-once holds, capture holds");
    $display("PASS checks=%0d entered: stall=%0d accepts=%0d delayed=%0d",
             checks, n_stall_cycles, n_stall_accepts, n_delayed_resp);
    $finish;
  end

  initial begin
    #200000;
    $display("TB_FAIL timeout checks=%0d errors=%0d", checks, errors);
    $fatal(1, "tb_rv32i_ss_dmem_scratchpad TIMEOUT");
  end

endmodule
