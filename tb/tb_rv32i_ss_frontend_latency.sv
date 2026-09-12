`timescale 1ns/1ps

// Directed latency battery for the frontend request/response boundary.
// The frontend may present its next offer while an accepted request remains
// in flight. Counters require this overlap to occur, and architectural checks
// verify that every decoded slot matches the instruction image at its PC.
//
// A testbench responder drives the ports with per-request timing:
//   - Delayed: accept at T, present at T+1, and hold until accepted.
//   - Pipelined: accept a new request on the edge that retires the preceding
//     response, allowing a distinct-transaction handover every cycle.
//
// Scenarios cover delayed streaming, data captured before backing-store
// changes, redirect coincident with a response, redirect while a response is
// withheld, mixed immediate/delayed responses, address wraparound, and full
// queue backpressure. Pipelined streaming, redirect during a handover, and
// stalled decode under pipelined supply exercise the same obligations when
// requests arrive back to back.
//
// Continuous invariants:
//   - A presented request holds valid and address until acceptance.
//   - At most one accepted transaction remains outstanding; a new acceptance
//     may coincide with the previous response retiring.
//   - Every delivered instruction matches the addressed program word.
//   - Under pure delayed timing, the responder matches
//     rv32i_ss_imem_scratchpad #(.LATENCY(1)) on ready, response valid and data,
//     including the moving-backing-store scenario.
//
// Spurious responses violate an environment assumption and intentionally
// trigger the frontend's response-identity assertion; they are not treated
// as inputs that the frontend promises to ignore.

module tb_rv32i_ss_frontend_latency;
  import rv32i_ss_pkg::*;

  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  // ------------------------------------------------------------------ DUT
  logic  imem_req_valid, imem_req_ready;
  word_t imem_req_addr;
  logic  imem_resp_valid, imem_resp_ready;
  word_t [1:0] imem_resp_data;
  logic  redirect_valid = 1'b0;
  word_t redirect_target = '0;
  logic decoded_valid, decoded_ready;
  logic [1:0] decoded_slot_valid;
  word_t [1:0] decoded_pc, decoded_instr, decoded_imm;
  logic [1:0] decoded_pred_taken;
  word_t decoded_pred_target;
  arch_reg_t [1:0] d_rs1, d_rs2, d_rd;
  logic [1:0] d_we, d_ck, d_ld, d_st, d_mu;
  ooo_op_class_e [1:0] d_op; ooo_fu_class_e [1:0] d_fu;
  fyp_cpu_pkg::alu_op_e [1:0] d_alu;
  rv32i_pipeline_pkg::muldiv_op_e [1:0] d_md;
  rv32i_pipeline_pkg::br_type_e [1:0] d_br;
  ooo_src_sel_e [1:0] d_s1, d_s2;
  fyp_cpu_pkg::mem_size_e [1:0] d_ms;
  decoded_trap_t d_trap;
  rv32i_pipeline_pkg::csr_op_e d_csrop;
  csr_addr_t d_csra; csr_zimm_t d_csrz;

  rv32i_ss_frontend #(.RESET_PC(32'h0)) dut (
    .clk(clk), .rst_n(rst_n),
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr),
    .imem_resp_valid(imem_resp_valid), .imem_resp_ready(imem_resp_ready),
    .imem_resp_data(imem_resp_data),
    .bp_update_valid(1'b0),  // predictor inert, as in tb_rv32i_ss_frontend
    .bp_update_pc('0), .bp_update_taken(1'b0), .bp_update_target('0),
    .bp_return_update_valid(1'b0), .bp_return_update_pc('0),
    .ras_fetch_valid(1'b0), .ras_fetch_target('0),
    .redirect_valid(redirect_valid), .redirect_target(redirect_target),
    .decoded_valid(decoded_valid), .decoded_slot_valid(decoded_slot_valid),
    .decoded_ready(decoded_ready),
    .decoded_pc(decoded_pc), .decoded_instr(decoded_instr),
    .decoded_rs1(d_rs1), .decoded_rs2(d_rs2), .decoded_rd(d_rd),
    .decoded_rd_we(d_we), .decoded_needs_checkpoint(d_ck),
    .decoded_op_class(d_op), .decoded_fu_class(d_fu),
    .decoded_muldiv_op(d_md), .decoded_alu_op(d_alu),
    .decoded_branch_op(d_br), .decoded_src1_sel(d_s1), .decoded_src2_sel(d_s2),
    .decoded_imm(decoded_imm), .decoded_trap(d_trap),
    .decoded_csr_op(d_csrop), .decoded_csr_addr(d_csra), .decoded_csr_zimm(d_csrz),
    .decoded_is_load(d_ld), .decoded_is_store(d_st),
    .decoded_mem_size(d_ms), .decoded_mem_unsigned(d_mu),
    .decoded_pred_taken(decoded_pred_taken),
    .decoded_pred_target(decoded_pred_target)
  );

  // -------------------------------------------------------- program image
  // Every address decodes as ADDI x1, x0, imm with imm = addr[13:2] ^ epoch,
  // so any line forms a plain dual-ALU bundle and the whole 32-bit space is
  // a defined program. mem_epoch makes the image MUTABLE, exactly like the
  // scratchpad unit battery's store: capture-at-acceptance is only a
  // testable property if the store can move underneath a pending response.
  // It is nonzero only in the capture test, which restores it before the
  // affected line can reach decode.
  logic [11:0] mem_epoch = '0;

  function automatic word_t instr_at(input word_t a);
    instr_at = {a[13:2] ^ mem_epoch, 5'd0, 3'b000, 5'd1, 7'b0010011};
  endfunction

  function automatic word_t [1:0] line_at(input word_t a);
    word_t base;
    begin
      base    = {a[31:3], 3'b000};
      line_at = {instr_at(base + 32'd4), instr_at(base)};
    end
  endfunction

  // ------------------------------------------------------- TB responder
  // Delayed mode is the ratified LATENCY=1 shape (one-deep, capture at
  // acceptance, hold under backpressure). Same-cycle mode is the adapter
  // shape; env_alternate flips per request. env_block_resp withholds a
  // pending delayed response for exactly the cycles the kill legs need.
  //
  // Pipelined mode: a new request is accepted in the
  // same cycle the presented response retires. Its ready reads the DUT's
  // resp_ready output; the apparent combinational cycle
  // (ready -> fire -> resp_ready) is value-convergent — whenever a response
  // is presented the DUT has its transaction in flight, so resp_ready's
  // fire term is short-circuited by req_inflight_q — and settles in one
  // delta. One accepted transaction outstanding is preserved by
  // construction: a fire requires the presented response to be retiring.
  // The accept -> present -> retire record is ONE machine for every mode;
  // the modes differ only in the ready and present muxes. A single record
  // also removes an entire hazard class: a transaction accepted moments
  // before a mode switch stays tracked, instead of freezing in the state of
  // a machine the mux no longer selects.
  logic  env_same_cycle  = 1'b0;
  logic  env_alternate   = 1'b0;
  logic  env_block_resp  = 1'b0;
  logic  env_pipelined   = 1'b0;
  logic  env_drain       = 1'b0;  // refuse new requests; let responses finish
  logic  pend_q          = 1'b0;
  word_t pend_addr_q     = '0;
  word_t [1:0] pend_data_q = '0;
  logic  alt_flip_q      = 1'b0;

  wire eff_same_cycle = env_alternate ? alt_flip_q : env_same_cycle;
  wire req_fire  = imem_req_valid && imem_req_ready;
  wire resp_fire = imem_resp_valid && imem_resp_ready;

  // Pipelined mode accepts in the retiring cycle: the apparent combinational
  // cycle (ready -> fire -> resp_ready) is value-convergent — whenever a
  // response is presented the DUT has its transaction in flight, so
  // resp_ready's fire term is short-circuited by req_inflight_q — and
  // settles in one delta. A fire that overwrites the record is legal only
  // because it required imem_resp_ready: the old transaction is retiring on
  // the same edge.
  assign imem_req_ready  = !env_drain &&
                           (!pend_q || (env_pipelined && imem_resp_ready));
  assign imem_resp_valid = (pend_q && !env_block_resp)
                         || (eff_same_cycle && imem_req_valid && !pend_q);
  assign imem_resp_data  = pend_q ? pend_data_q
                                  : line_at(imem_req_addr);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pend_q     <= 1'b0;
      alt_flip_q <= 1'b0;
    end else begin
      if (req_fire && !eff_same_cycle) begin
        pend_q      <= 1'b1;
        pend_addr_q <= imem_req_addr;
        pend_data_q <= line_at(imem_req_addr);   // capture at acceptance
      end else if (resp_fire && pend_q) begin
        pend_q <= 1'b0;
      end
      if (req_fire && env_alternate) alt_flip_q <= ~alt_flip_q;
    end
  end

  // ------------------------------------------------- environment equivalence
  // The delayed responder must match LATENCY=1. A shadow scratchpad observes
  // the same request
  // stream and the same resp_ready; in pure delayed shape, after both models
  // re-sync on a fire with neither presenting a response, all three
  // environment outputs must match ===. (The idle-at-resync condition
  // matters: a disturbance window can retire the shadow's response while the
  // responder's is withheld, and re-arming while either side still holds
  // state would compare two machines mid-divergence.)
  logic        eqs_req_ready, eqs_resp_valid;
  word_t [1:0] eqs_resp_data;
  word_t       eqs_line_addr;
  word_t [1:0] eqs_line_data;
  assign eqs_line_data = line_at(eqs_line_addr);

  // The shadow accepts on ITS OWN ready, so while env_drain deliberately
  // deviates from the scratchpad shape (refusing requests the shadow would
  // take), the request must be hidden from it — otherwise it ingests a
  // phantom transaction the real port never fired and stays out of phase.
  rv32i_ss_imem_scratchpad #(.LATENCY(1)) u_eq_shadow (
    .clk(clk), .rst_n(rst_n),
    .imem_req_valid(imem_req_valid && !env_drain),
    .imem_req_ready(eqs_req_ready),
    .imem_req_addr(imem_req_addr),
    .imem_resp_valid(eqs_resp_valid), .imem_resp_ready(imem_resp_ready),
    .imem_resp_data(eqs_resp_data),
    .line_addr(eqs_line_addr), .line_data(eqs_line_data)
  );

  wire  eq_armable = !env_block_resp && !env_alternate && !env_same_cycle &&
                     !env_pipelined && !env_drain;
  logic eq_sync_q = 1'b0;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)                 eq_sync_q <= 1'b0;
    else if (!eq_armable)       eq_sync_q <= 1'b0;
    else if (req_fire && !imem_resp_valid && !eqs_resp_valid)
                                eq_sync_q <= 1'b1;
  end

  // Apply the same equivalence check to the PIPELINED shape. While the responder
  // is in pipelined mode, a shadow #(.LATENCY(1), .PIPELINED(1)) instance
  // must agree === on ready, resp_valid and presented data — the battery's
  // pipelined environment model is checked against the scratchpad instance.
  logic        eqp_req_ready, eqp_resp_valid;
  word_t [1:0] eqp_resp_data;
  word_t       eqp_line_addr;
  word_t [1:0] eqp_line_data;
  assign eqp_line_data = line_at(eqp_line_addr);

  // Alignment by construction rather than by sync. The shadow's request
  // input is masked with the RESPONDER's ready, so it ingests only real
  // fires and can never phantom-accept an offer the responder refused.
  // It is also cleared at every quiesce, so each pipelined
  // leg starts both models idle; from the first real fire the two are
  // locked to the same accept and retire wires, and any contract
  // divergence — a ready rule that would have refused a fire the responder
  // took, a wrong presentation, wrong data — is a direct mismatch.
  logic eqp_shadow_clear = 1'b0;

  rv32i_ss_imem_scratchpad #(.LATENCY(1), .PIPELINED(1)) u_eqp_shadow (
    .clk(clk), .rst_n(rst_n && !eqp_shadow_clear),
    .imem_req_valid(imem_req_valid && imem_req_ready),
    .imem_req_ready(eqp_req_ready),
    .imem_req_addr(imem_req_addr),
    .imem_resp_valid(eqp_resp_valid), .imem_resp_ready(imem_resp_ready),
    .imem_resp_data(eqp_resp_data),
    .line_addr(eqp_line_addr), .line_data(eqp_line_data)
  );

  wire  eqp_armable = env_pipelined && !env_block_resp && !env_alternate &&
                      !env_same_cycle && !env_drain;
  logic eqp_live_q = 1'b0;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)                       eqp_live_q <= 1'b0;
    else if (eqp_shadow_clear)        eqp_live_q <= 1'b0;
    else if (req_fire && eqp_armable) eqp_live_q <= 1'b1;
  end

  // ------------------------------------------------------------ scoreboard
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

  integer n_eq_samples = 0;
  task automatic eq_check(input string where);
    begin
      if (eq_sync_q && eq_armable) begin
        checks = checks + 1;
        n_eq_samples = n_eq_samples + 1;
        if (eqs_req_ready  !== imem_req_ready  ||
            eqs_resp_valid !== imem_resp_valid ||
            (imem_resp_valid === 1'b1 && (eqs_resp_data !== imem_resp_data))) begin
          errors = errors + 1;
          $display("  FAIL EQ @%0t (%s) responder != scratchpad: ready %b/%b valid %b/%b",
                   $time, where, imem_req_ready, eqs_req_ready,
                   imem_resp_valid, eqs_resp_valid);
        end
      end
    end
  endtask

  integer n_eqp_samples = 0;
  task automatic eqp_check(input string where);
    begin
      if (eqp_live_q && eqp_armable) begin
        checks = checks + 1;
        n_eqp_samples = n_eqp_samples + 1;
        if (eqp_req_ready  !== imem_req_ready  ||
            eqp_resp_valid !== imem_resp_valid ||
            (imem_resp_valid === 1'b1 && (eqp_resp_data !== imem_resp_data))) begin
          errors = errors + 1;
          $display("  FAIL EQP @%0t (%s) responder != pipelined scratchpad: ready %b/%b valid %b/%b",
                   $time, where, imem_req_ready, eqp_req_ready,
                   imem_resp_valid, eqp_resp_valid);
        end
      end
    end
  endtask
  always @(posedge clk) if (rst_n) eq_check("posedge");
  always @(negedge clk) begin #2; if (rst_n) eq_check("negedge+2"); end
  always @(posedge clk) if (rst_n) eqp_check("posedge");
  always @(negedge clk) begin #2; if (rst_n) eqp_check("negedge+2"); end

  // Entry-proof counters: a leg that never entered its state is a FAIL even
  // if every check passed vacuously.
  integer n_delayed_resp        = 0;  // responses presented from the pending record
  integer n_same_cycle          = 0;  // responses consumed on the fire edge
  integer n_coincident          = 0;  // redirect in the exact response cycle
  integer n_kill_drain          = 0;  // redirect with request in flight, no resp
  integer n_wrap_fire           = 0;  // fire at the wrap line, then at zero
  integer n_qfull_quiet         = 0;  // stalled cycles with no request presented
  integer n_capture             = 0;  // capture window observed a moved store
  integer n_offer_while_inflight = 0; // THE pipelined state: presented offer + accepted outstanding
  integer n_b2b_fire            = 0;  // fires in consecutive cycles (pipelined)
  integer n_handover            = 0;  // distinct-transaction response+fire cycles
  integer n_flush_handover      = 0;  // redirect landing on a handover cycle
  integer n_resp_backpressure   = 0;  // live response held at the boundary
  logic   wrap_seen_hi          = 1'b0;
  logic   fire_prev_q           = 1'b0;

  // Killed-path exclusion window: after a redirect, no line base in the
  // armed set may ever decode. Two bases suffice: the in-flight transaction
  // and the presented prefetch offer at the moment of the kill.
  logic  excl_armed = 1'b0;
  word_t excl_base_a = '0;
  word_t excl_base_b = '0;

  wire env_accepted_pending = pend_q;

  // Producer-ownership tracking (law 1).
  logic  prev_stalled_q = 1'b0;
  word_t prev_addr_q    = '0;

  always @(posedge clk) if (rst_n) begin
    // Law 1 — producer ownership at the request port.
    if (prev_stalled_q) begin
      chk("GLOBAL stalled request held valid", imem_req_valid === 1'b1);
      chk("GLOBAL stalled request held addr",  imem_req_addr === prev_addr_q);
    end
    prev_stalled_q <= imem_req_valid && !imem_req_ready;
    prev_addr_q    <= imem_req_addr;

    // Law 2 — one accepted transaction outstanding: a fire may coincide with
    // the previous response retiring, never with it still pending.
    chk("GLOBAL one accepted outstanding",
        !(req_fire && env_accepted_pending && !resp_fire));

    // Law 3 — the queue delivers the addressed program.
    if (decoded_valid && decoded_ready) begin
      if (decoded_slot_valid[0])
        chk("GLOBAL slot0 instr matches image",
            decoded_instr[0] === instr_at(decoded_pc[0]));
      if (decoded_slot_valid[1])
        chk("GLOBAL slot1 instr matches image",
            decoded_instr[1] === instr_at(decoded_pc[1]));
      if (excl_armed) begin
        chk("GLOBAL killed line A never decodes",
            {decoded_pc[0][31:3], 3'b000} !== excl_base_a);
        chk("GLOBAL killed line B never decodes",
            {decoded_pc[0][31:3], 3'b000} !== excl_base_b);
      end
    end

    // Entry census, port-level only.
    if (resp_fire && !env_pipelined && pend_q && !req_fire)
      n_delayed_resp = n_delayed_resp + 1;
    if (resp_fire && !env_pipelined && !pend_q && req_fire)
      n_same_cycle = n_same_cycle + 1;
    if (redirect_valid && env_accepted_pending &&  imem_resp_valid)
      n_coincident = n_coincident + 1;
    if (redirect_valid && env_accepted_pending && !imem_resp_valid)
      n_kill_drain = n_kill_drain + 1;
    if (req_fire && imem_req_addr === 32'hFFFF_FFF8) wrap_seen_hi <= 1'b1;
    if (req_fire && imem_req_addr === 32'h0000_0000 && wrap_seen_hi)
      n_wrap_fire = n_wrap_fire + 1;
    if (decoded_valid === 1'b1 && !decoded_ready && !imem_req_valid)
      n_qfull_quiet = n_qfull_quiet + 1;

    // The pipelined state itself, required rather than forbidden.
    if (imem_req_valid && env_accepted_pending)
      n_offer_while_inflight = n_offer_while_inflight + 1;
    if (req_fire && fire_prev_q) n_b2b_fire = n_b2b_fire + 1;
    fire_prev_q <= req_fire;
    if (resp_fire && req_fire && env_pipelined) begin
      n_handover = n_handover + 1;
      if (redirect_valid) n_flush_handover = n_flush_handover + 1;
    end
    if (imem_resp_valid && !imem_resp_ready)
      n_resp_backpressure = n_resp_backpressure + 1;
  end

  // ------------------------------------------------------------- helpers
  task automatic expect_bundle(input string name, input word_t pc);
    begin
      // A packet may already be presented at call time (a stalled head when
      // ready was low, or the next bundle mid-stream). Check it before the
      // next posedge can consume it.
      decoded_ready = 1'b1;
      #1;
      while (decoded_valid !== 1'b1) begin
        @(negedge clk); #1;
      end
      chk({name, ": pc"},    decoded_pc[0]     === pc);
      chk({name, ": pc1"},   decoded_pc[1]     === pc + 32'd4);
      chk({name, ": shape"}, decoded_slot_valid === 2'b11);
      @(posedge clk);
    end
  endtask

  task automatic wait_fire(output word_t addr);
    begin
      while (1) begin
        @(posedge clk);
        if (req_fire) begin addr = imem_req_addr; break; end
      end
    end
  endtask

  task automatic quiesce();
    begin
      // Drain to a true idle before any mode change — with DECODE ENABLED.
      // The frontend can hold an overcommitted response at the boundary until a
      // consume frees its slot, so draining with decode stalled deadlocks
      // (the exact state leg F pins on purpose). env_drain freezes new
      // acceptances while every already-accepted response completes; a
      // presented offer stays presented (producer-owned) and is simply
      // accepted later, under whatever mode the next leg selects.
      @(negedge clk);
      redirect_valid = 1'b0;
      env_block_resp = 1'b0;
      env_drain      = 1'b1;
      decoded_ready  = 1'b1;
      repeat (2) @(negedge clk);
      while (pend_q) @(negedge clk);
      eqp_shadow_clear = 1'b1;   // both models idle: restart the shadow here
      env_pipelined  = 1'b0;
      env_alternate  = 1'b0;
      env_same_cycle = 1'b0;
      env_drain      = 1'b0;
      @(negedge clk);
      eqp_shadow_clear = 1'b0;
      decoded_ready  = 1'b0;
      repeat (2) @(negedge clk);
    end
  endtask

  task automatic leg(input string name);
    $display("[tb_rv32i_ss_frontend_latency] %s @%0t", name, $time);
  endtask

  // Deterministic two-outstanding setup for the kill legs: from a drained
  // machine, redirect to T0 with decode stalled; the T0 request fires and
  // the T0+8 prefetch offer is created in the same evaluation, so
  // the state {T0 accepted in flight, T0+8 presented} is entered by
  // construction, and proven by the offer-while-in-flight census.
  task automatic setup_two_outstanding(input word_t t0, output word_t fired);
    begin
      @(negedge clk);
      decoded_ready = 1'b0;
      redirect_valid = 1'b1; redirect_target = t0;
      @(negedge clk); redirect_valid = 1'b0;
      wait_fire(fired);
      chk("setup fired the redirect line", fired === {t0[31:3], 3'b000});
      @(negedge clk); #1;
      chk("setup offer presented while in flight",
          imem_req_valid === 1'b1 && pend_q === 1'b1);
      chk("setup offer is the next sequential line",
          imem_req_addr === {t0[31:3], 3'b000} + 32'd8);
    end
  endtask

  // ------------------------------------------------------------- stimulus
  integer i;
  integer fires_a, last_fire_t;
  word_t  tgt, fired_addr, seen_addr;
  word_t [1:0] cap_expect;

  initial begin
    decoded_ready = 1'b0;
    rst_n = 1'b0;
    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);

    // ---- Leg A: delayed streaming --------------------------------------
    // Delayed responses impose a two-cycle acceptance gap. The frontend
    // still presents its next offer while a request is outstanding, and
    // the overlap counter must observe that state.
    leg("leg A");
    fires_a = 0; last_fire_t = 0;
    fork
      begin : leg_a_walk
        expect_bundle("A pc=00", 32'h0000_0000);
        expect_bundle("A pc=08", 32'h0000_0008);
        expect_bundle("A pc=10", 32'h0000_0010);
        expect_bundle("A pc=18", 32'h0000_0018);
        expect_bundle("A pc=20", 32'h0000_0020);
        expect_bundle("A pc=28", 32'h0000_0028);
      end
      begin : leg_a_period
        forever begin
          @(posedge clk);
          if (req_fire) begin
            if (fires_a != 0)
              chk("A fire-to-fire gap is exactly 2 cycles",
                  ($time - last_fire_t) == 20);
            last_fire_t = $time;
            fires_a = fires_a + 1;
          end
        end
      end
    join_any
    disable leg_a_period;
    chk("A entered: >=6 delayed responses", n_delayed_resp >= 6);

    // ---Leg : capture-at-acceptance under a moving store -------------
    // Decode pauses, a request fires, the store moves immediately after the
    // acceptance edge — so it holds its moved value across every instant of
    // the presentation window — and the presented data must still be the
    // captured line. The epoch is restored before the (correctly captured)
    // line can reach decode, so the image oracle never disagrees.
    leg("leg A2");
    @(negedge clk);
    decoded_ready = 1'b0;
    // Enter through a flush: pipelined prefetch may already have the queue
    // full with the overcommitted response held, in which case no further
    // fire would ever come with decode stalled. The redirect drains
    // everything and guarantees a deterministic first fire.
    redirect_valid = 1'b1; redirect_target = 32'h0000_2000;
    @(negedge clk); redirect_valid = 1'b0;
    wait_fire(fired_addr);
    cap_expect = line_at(fired_addr);      // the accepted line, epoch 0
    #1;
    mem_epoch = 12'h0A5;
    @(negedge clk); #2;
    chk("A2 response presented while store moved", imem_resp_valid === 1'b1);
    chk("A2 presented data is the captured line",
        imem_resp_data === cap_expect);
    if (imem_resp_valid === 1'b1) n_capture = n_capture + 1;
    @(negedge clk);
    mem_epoch = '0;
    decoded_ready = 1'b1;
    quiesce();

    // ---- Leg B: kill with a coincident drain ---------------------------
    // {T0 in flight, T0+8 presented}; the redirect lands in the exact cycle
    // T0's response is presented. T0 drains killed on that edge, T0+8 stays
    // presented (producer-owned), fires killed, drains — and only then does
    // the redirect line fire. Neither killed line ever decodes.
    leg("leg B");
    setup_two_outstanding(32'h0000_0400, fired_addr);
    excl_base_a = fired_addr;
    excl_base_b = fired_addr + 32'd8;
    excl_armed  = 1'b1;
    tgt = 32'h0000_0600;
    // setup returned inside T0's response-presentation cycle — act NOW; one
    // more negedge and the response would already have drained live.
    chk("B response presented at kill", imem_resp_valid === 1'b1);
    redirect_valid = 1'b1; redirect_target = tgt;
    @(posedge clk); #1;
    chk("B coincidence occurred", n_coincident >= 1);
    @(negedge clk);
    redirect_valid = 1'b0;
    wait_fire(seen_addr);
    chk("B killed offer fires first", seen_addr === excl_base_b);
    wait_fire(seen_addr);
    chk("B redirect line fires after the drain",
        seen_addr === {tgt[31:3], 3'b000});
    expect_bundle("B target decodes", tgt);
    expect_bundle("B target+8 decodes", tgt + 32'h8);
    excl_armed = 1'b0;
    quiesce();

    // ---- Leg C: kill with the response withheld ------------------------
    // Same two-outstanding state, but the in-flight response is blocked in
    // the kill cycle: the presented offer is visibly outstanding through the
    // kill. The withheld response drains killed one cycle later, then the
    // killed offer, then the redirect.
    leg("leg C");
    setup_two_outstanding(32'h0000_0800, fired_addr);
    excl_base_a = fired_addr;
    excl_base_b = fired_addr + 32'd8;
    excl_armed  = 1'b1;
    tgt = 32'h0000_0A00;
    // setup returned inside T0's response-presentation cycle: withhold the
    // response and kill in this same cycle, before it can drain live.
    env_block_resp  = 1'b1;                // kill cycle: no response
    redirect_valid  = 1'b1;
    redirect_target = tgt;
    #1;
    chk("C offer still presented through the kill",
        imem_req_valid === 1'b1 && imem_req_addr === excl_base_b);
    @(posedge clk); #1;
    chk("C kill-while-in-flight occurred", n_kill_drain >= 1);
    @(negedge clk);
    redirect_valid = 1'b0;
    env_block_resp = 1'b0;                 // drain cycle: killed response
    #1;
    chk("C killed response presented for drain",
        imem_resp_valid === 1'b1 && pend_q === 1'b1);
    @(posedge clk); #1;
    chk("C drain retired the transaction", pend_q === 1'b0);
    chk("C nothing enqueued from the drain", decoded_valid === 1'b0);
    wait_fire(seen_addr);
    chk("C killed offer fires after the drain", seen_addr === excl_base_b);
    wait_fire(seen_addr);
    chk("C redirect line fires last", seen_addr === {tgt[31:3], 3'b000});
    expect_bundle("C target decodes", tgt);
    expect_bundle("C target+8 decodes", tgt + 32'h8);
    excl_armed = 1'b0;
    quiesce();

    // ---- Leg D: same-cycle and delayed responses alternating -----------
    leg("leg D");
    env_alternate = 1'b1;
    decoded_ready = 1'b1;
    tgt = 32'h0000_1000;
    redirect_valid = 1'b1; redirect_target = tgt;
    @(negedge clk); redirect_valid = 1'b0;
    for (i = 0; i < 8; i = i + 1)
      expect_bundle($sformatf("D pc=%0h", tgt + 8*i), tgt + word_t'(8*i));
    chk("D entered: same-cycle responses seen", n_same_cycle >= 3);
    chk("D entered: delayed responses interleaved", n_delayed_resp >= 9);
    quiesce();

    // ---- Leg E: wraparound ---------------------------------------------
    leg("leg E");
    decoded_ready = 1'b1;
    redirect_valid = 1'b1; redirect_target = 32'hFFFF_FFF8;
    @(negedge clk); redirect_valid = 1'b0;
    expect_bundle("E last line decodes", 32'hFFFF_FFF8);
    expect_bundle("E wraps to zero",     32'h0000_0000);
    expect_bundle("E continues at 8",    32'h0000_0008);
    chk("E entered: wrap fire observed", n_wrap_fire >= 1);
    quiesce();

    // ---- Leg F: queue-full backpressure under the delayed shape --------
    // Backend stalled. pipelined fetches ahead: two lines fill, the third
    // (overcommitted) request fires and its response is HELD at the
    // boundary — never dropped — while the request channel goes quiet
    // because creation is blocked at two residents. Consume releases it.
    leg("leg F");
    decoded_ready = 1'b0;
    tgt = 32'h0000_3000;
    redirect_valid = 1'b1; redirect_target = tgt;
    @(negedge clk); redirect_valid = 1'b0;
    repeat (14) @(negedge clk);   // fills + the held third response settle
    for (i = 0; i < 5; i = i + 1) begin
      @(posedge clk);
      chk("F request channel quiet at queue-full", !imem_req_valid);
      chk("F queue is full and presenting, not dead", decoded_valid === 1'b1);
      chk("F overcommitted response is held, not dropped",
          imem_resp_valid === 1'b1 && imem_resp_ready === 1'b0);
    end
    chk("F entered: quiet cycles observed", n_qfull_quiet >= 5);
    chk("F entered: backpressure observed", n_resp_backpressure >= 3);
    @(negedge clk);
    expect_bundle("F drain resumes stream", tgt);
    expect_bundle("F refill continues", tgt + 32'h8);
    expect_bundle("F held response's line arrives", tgt + 32'h10);
    quiesce();

    // ---Leg : pipelined streaming — the pipelined capability ----------
    // Under the pipelined-shaped environment the machine must sustain a fire
    // every cycle, every one a distinct-transaction handover: the response
    // of request N retiring in the same cycle request N+1 is accepted.
    leg("leg S1");
    env_pipelined = 1'b1;
    decoded_ready = 1'b1;
    tgt = 32'h0000_4000;
    redirect_valid = 1'b1; redirect_target = tgt;
    @(negedge clk); redirect_valid = 1'b0;
    for (i = 0; i < 10; i = i + 1)
      expect_bundle($sformatf("S1 pc=%0h", tgt + 8*i), tgt + word_t'(8*i));
    chk("S1 entered: back-to-back fires", n_b2b_fire >= 8);
    chk("S1 entered: distinct-transaction handovers", n_handover >= 8);
    quiesce();

    // ---- Kill landing on a handover cycle ------------------------------
    // The redirect lands on a cycle that is BOTH the drain of request A and
    // the fire of request B. A drains killed on the flush edge, B
    // enters the in-flight record killed and drains next, and the redirect
    // line fires immediately after: the flush costs the drain, nothing more.
    leg("leg S2");
    env_pipelined = 1'b1;
    decoded_ready = 1'b1;
    tgt = 32'h0000_5000;
    redirect_valid = 1'b1; redirect_target = tgt;
    @(negedge clk); redirect_valid = 1'b0;
    // Let the stream reach steady handovers, then kill mid-stream.
    for (i = 0; i < 4; i = i + 1) expect_bundle("S2 stream", tgt + word_t'(8*i));
    excl_armed = 1'b0;
    tgt = 32'h0000_6000;
    @(negedge clk); #1;
    // Snapshot the two live transactions at the kill edge: the accepted one
    // (presenting or about to present) and the presented offer.
    excl_base_a = {pend_addr_q[31:3], 3'b000};
    excl_base_b = imem_req_valid ? {imem_req_addr[31:3], 3'b000}
                                 : excl_base_a;
    excl_armed  = 1'b1;
    redirect_valid = 1'b1; redirect_target = tgt;
    @(posedge clk); #1;
    @(negedge clk);
    redirect_valid = 1'b0;
    wait_fire(seen_addr);
    chk("S2 redirect line fires right after the killed drain",
        seen_addr === {tgt[31:3], 3'b000});
    expect_bundle("S2 target decodes", tgt);
    expect_bundle("S2 target+8 decodes", tgt + 32'h8);
    chk("S2 entered: a flush landed on a handover", n_flush_handover >= 1);
    excl_armed = 1'b0;
    quiesce();

    // ---Leg : pipelined exhaustion ----------------------------------
    // Stalled decode under pipelined supply: the window fills, the last
    // accepted response is held at the boundary, the request channel goes
    // quiet, and consume releases everything in order.
    leg("leg S3");
    env_pipelined = 1'b1;
    decoded_ready = 1'b0;
    tgt = 32'h0000_7000;
    redirect_valid = 1'b1; redirect_target = tgt;
    @(negedge clk); redirect_valid = 1'b0;
    repeat (14) @(negedge clk);
    for (i = 0; i < 5; i = i + 1) begin
      @(posedge clk);
      chk("S3 request channel quiet when exhausted", !imem_req_valid);
      chk("S3 held response stays presented",
          imem_resp_valid === 1'b1 && imem_resp_ready === 1'b0);
    end
    @(negedge clk);
    expect_bundle("S3 drain resumes stream", tgt);
    expect_bundle("S3 refill continues", tgt + 32'h8);
    expect_bundle("S3 held response's line arrives", tgt + 32'h10);
    quiesce();

    // ---- entry-proof closure -------------------------------------------
    if (n_delayed_resp < 9 || n_same_cycle < 3 || n_coincident < 1 ||
        n_kill_drain < 1 || n_wrap_fire < 1 || n_qfull_quiet < 5 ||
        n_eq_samples < 100 || n_capture < 1 ||
        n_offer_while_inflight < 20 || n_b2b_fire < 8 || n_handover < 8 ||
        n_flush_handover < 1 || n_resp_backpressure < 3 ||
        n_eqp_samples < 60) begin
      $display("TB_FAIL leg not entered: delayed=%0d same=%0d coinc=%0d kill=%0d wrap=%0d qfull=%0d eq=%0d cap=%0d owi=%0d b2b=%0d ho=%0d fho=%0d bp=%0d eqp=%0d",
               n_delayed_resp, n_same_cycle, n_coincident, n_kill_drain,
               n_wrap_fire, n_qfull_quiet, n_eq_samples, n_capture,
               n_offer_while_inflight, n_b2b_fire, n_handover,
               n_flush_handover, n_resp_backpressure, n_eqp_samples);
      $fatal(1, "tb_rv32i_ss_frontend_latency: a directed leg failed to enter its state");
    end

    if (errors != 0) begin
      $display("TB_FAIL errors=%0d checks=%0d", errors, checks);
      $fatal(1, "tb_rv32i_ss_frontend_latency FAILED");
    end
    $display("[tb_rv32i_ss_frontend_latency] PASS checks=%0d entered: delayed=%0d same=%0d coinc=%0d kill=%0d wrap=%0d qfull=%0d eq=%0d cap=%0d owi=%0d b2b=%0d ho=%0d fho=%0d bp=%0d eqp=%0d",
             checks, n_delayed_resp, n_same_cycle, n_coincident, n_kill_drain,
             n_wrap_fire, n_qfull_quiet, n_eq_samples, n_capture,
             n_offer_while_inflight, n_b2b_fire, n_handover,
             n_flush_handover, n_resp_backpressure, n_eqp_samples);
    $finish;
  end

  initial begin
    #600000;
    $display("TB_FAIL timeout checks=%0d errors=%0d", checks, errors);
    $fatal(1, "tb_rv32i_ss_frontend_latency TIMEOUT");
  end

endmodule
