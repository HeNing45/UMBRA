// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// fetch-queue control battery.
//
// Unlike the compatibility-adapter tests, this test drives the final
// request/response channel directly. It enters the queue and request states
// that a combinational always-ready memory cannot expose: request stall,
// delayed response, a full two-line queue, word-granular partial consumption,
// an accepted request killed by redirect, and redirect coincident with a
// response. Payload checks make every discard/consume decision observable.
module tb_rv32i_ss_fetch_queue;
  import rv32i_ss_pkg::*;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  logic  imem_req_valid;
  logic  imem_req_ready;
  word_t imem_req_addr;
  logic  imem_resp_valid;
  logic  imem_resp_ready;
  word_t [1:0] imem_resp_data;

  logic  redirect_valid;
  word_t redirect_target;
  logic  decoded_valid;
  logic  decoded_ready;
  logic [1:0] decoded_slot_valid;
  word_t [1:0] decoded_pc;
  word_t [1:0] decoded_instr;

  arch_reg_t [1:0] decoded_rs1, decoded_rs2, decoded_rd;
  logic [1:0] decoded_rd_we, decoded_needs_checkpoint;
  ooo_op_class_e [1:0] decoded_op_class;
  ooo_fu_class_e [1:0] decoded_fu_class;
  rv32i_pipeline_pkg::muldiv_op_e [1:0] decoded_muldiv_op;
  fyp_cpu_pkg::alu_op_e [1:0] decoded_alu_op;
  rv32i_pipeline_pkg::br_type_e [1:0] decoded_branch_op;
  ooo_src_sel_e [1:0] decoded_src1_sel, decoded_src2_sel;
  word_t [1:0] decoded_imm;
  decoded_trap_t decoded_trap;
  rv32i_pipeline_pkg::csr_op_e decoded_csr_op;
  csr_addr_t decoded_csr_addr;
  csr_zimm_t decoded_csr_zimm;
  logic [1:0] decoded_is_load, decoded_is_store;
  fyp_cpu_pkg::mem_size_e [1:0] decoded_mem_size;
  logic [1:0] decoded_mem_unsigned;
  logic [1:0] decoded_pred_taken;
  word_t decoded_pred_target;

  rv32i_ss_frontend #(.RESET_PC(32'h0000_0000)) dut (
    .clk                      (clk),
    .rst_n                    (rst_n),
    .imem_req_valid           (imem_req_valid),
    .imem_req_ready           (imem_req_ready),
    .imem_req_addr            (imem_req_addr),
    .imem_resp_valid          (imem_resp_valid),
    .imem_resp_ready          (imem_resp_ready),
    .imem_resp_data           (imem_resp_data),
    .redirect_valid           (redirect_valid),
    .redirect_target          (redirect_target),
    .bp_update_valid          (1'b0),
    .bp_update_pc             ('0),
    .bp_update_taken          (1'b0),
    .bp_update_target         ('0),
    .bp_return_update_valid   (1'b0),
    .bp_return_update_pc      ('0),
    .ras_fetch_valid          (1'b0),
    .ras_fetch_target         ('0),
    .decoded_valid            (decoded_valid),
    .decoded_slot_valid       (decoded_slot_valid),
    .decoded_ready            (decoded_ready),
    .decoded_pc               (decoded_pc),
    .decoded_instr            (decoded_instr),
    .decoded_rs1              (decoded_rs1),
    .decoded_rs2              (decoded_rs2),
    .decoded_rd               (decoded_rd),
    .decoded_rd_we            (decoded_rd_we),
    .decoded_needs_checkpoint (decoded_needs_checkpoint),
    .decoded_op_class         (decoded_op_class),
    .decoded_fu_class         (decoded_fu_class),
    .decoded_muldiv_op        (decoded_muldiv_op),
    .decoded_alu_op           (decoded_alu_op),
    .decoded_branch_op        (decoded_branch_op),
    .decoded_src1_sel         (decoded_src1_sel),
    .decoded_src2_sel         (decoded_src2_sel),
    .decoded_imm              (decoded_imm),
    .decoded_trap             (decoded_trap),
    .decoded_csr_op           (decoded_csr_op),
    .decoded_csr_addr         (decoded_csr_addr),
    .decoded_csr_zimm         (decoded_csr_zimm),
    .decoded_is_load          (decoded_is_load),
    .decoded_is_store         (decoded_is_store),
    .decoded_mem_size         (decoded_mem_size),
    .decoded_mem_unsigned     (decoded_mem_unsigned),
    .decoded_pred_taken       (decoded_pred_taken),
    .decoded_pred_target      (decoded_pred_target)
  );

  localparam word_t LW_X2       = 32'h0000_2103; // lw   x2,0(x0)
  localparam word_t SW_X2       = 32'h0020_2023; // sw   x2,0(x0)
  localparam word_t ADDI_X3_3   = 32'h0030_0193; // addi x3,x0,3
  localparam word_t ADDI_X4_4   = 32'h0040_0213; // addi x4,x0,4
  localparam word_t STALE_X9_99 = 32'h0630_0493; // addi x9,x0,99
  localparam word_t STALE_X10   = 32'h0580_0513; // addi x10,x0,88
  localparam word_t ADDI_X5_5   = 32'h0050_0293; // addi x5,x0,5
  localparam word_t TARGET_X6_6 = 32'h0060_0313; // addi x6,x0,6
  localparam word_t DROP_X11    = 32'h04d0_0593; // addi x11,x0,77
  localparam word_t DROP_X12    = 32'h0420_0613; // addi x12,x0,66
  localparam word_t ADDI_X13_13 = 32'h00d0_0693; // addi x13,x0,13
  localparam word_t ADDI_X14_14 = 32'h00e0_0713; // addi x14,x0,14

  int checks = 0;
  int errors = 0;
  int waited;

  bit request_stall_seen;
  bit delayed_response_seen;
  bit full_queue_seen;
  bit partial_head_seen;
  bit killed_response_seen;
  bit upper_redirect_seen;
  bit same_edge_redirect_response_seen;
  bit stalled_upper_lock_seen;

  task automatic check(input string name, input logic condition);
    checks++;
    if (!condition) begin
      $error("[%s]", name);
      errors++;
    end
  endtask

  task automatic wait_request(input word_t expected_addr);
    waited = 0;
    while (!(imem_req_valid && (imem_req_addr == expected_addr))) begin
      @(negedge clk);
      waited++;
      if (waited > 40) begin
        $fatal(1, "request %08h did not appear: valid=%0b addr=%08h",
               expected_addr, imem_req_valid, imem_req_addr);
      end
    end
  endtask

  task automatic accept_without_response(input word_t expected_addr);
    wait_request(expected_addr);
    imem_req_ready = 1'b1;
    @(posedge clk);
    @(negedge clk);
    imem_req_ready = 1'b0;
    check("accepted request leaves response identity live", imem_resp_ready);
  endtask

  task automatic send_delayed_response(input word_t lower, input word_t upper);
    @(negedge clk);
    imem_resp_data  = {upper, lower};
    imem_resp_valid = 1'b1;
    check("delayed response sees ready", imem_resp_ready);
    @(posedge clk);
    @(negedge clk);
    imem_resp_valid = 1'b0;
    imem_resp_data  = '0;
  endtask

  task automatic accept_same_cycle_response(
    input word_t expected_addr,
    input word_t lower,
    input word_t upper
  );
    wait_request(expected_addr);
    imem_req_ready  = 1'b1;
    imem_resp_data  = {upper, lower};
    imem_resp_valid = 1'b1;
    #1;
    check("same-cycle request fire makes response ready", imem_resp_ready);
    @(posedge clk);
    @(negedge clk);
    imem_req_ready  = 1'b0;
    imem_resp_valid = 1'b0;
    imem_resp_data  = '0;
  endtask

  task automatic consume_one;
    decoded_ready = 1'b1;
    @(posedge clk);
    @(negedge clk);
    decoded_ready = 1'b0;
  endtask

  initial begin
    repeat (1000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_fetch_queue exceeded 1000 cycles");
  end

  initial begin
    $display("[tb_rv32i_ss_fetch_queue] starting");
    imem_req_ready  = 1'b0;
    imem_resp_valid = 1'b0;
    imem_resp_data  = '0;
    redirect_valid  = 1'b0;
    redirect_target = '0;
    decoded_ready   = 1'b0;
    request_stall_seen = 1'b0;
    delayed_response_seen = 1'b0;
    full_queue_seen = 1'b0;
    partial_head_seen = 1'b0;
    killed_response_seen = 1'b0;
    upper_redirect_seen = 1'b0;
    same_edge_redirect_response_seen = 1'b0;
    stalled_upper_lock_seen = 1'b0;

    repeat (4) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;

    // the registered request is a stable valid/ready offer. Hold it for
    // three cycles, then accept it without a response.
    wait_request(32'h0000_0000);
    repeat (3) begin
      @(posedge clk);
      @(negedge clk);
      check("stalled request remains valid", imem_req_valid);
      check("stalled request address remains stable", imem_req_addr == 32'h0);
    end
    request_stall_seen = 1'b1;
    accept_without_response(32'h0000_0000);

    // Keep the accepted request outstanding for two cycles, then return a
    // memory-pair line. Its lower word must be retained independently.
    repeat (2) begin
      @(posedge clk);
      @(negedge clk);
      check("accepted request retains response readiness", imem_resp_ready);
      check("no decode before delayed response", !decoded_valid);
    end
    send_delayed_response(LW_X2, SW_X2);
    delayed_response_seen = 1'b1;
    check("delayed response fills first queue line", dut.fq_count_q == 2'd1);
    check("first line lower word presented", decoded_valid &&
          (decoded_pc[0] == 32'h0) && (decoded_instr[0] == LW_X2));

    // fill the second line with a zero-latency response. Backpressure at
    // decode then leaves a physically full queue and no third request.
    accept_same_cycle_response(32'h0000_0008, ADDI_X3_3, ADDI_X4_4);
    check("two line queue reached full state", dut.fq_count_q == 2'd2);
    check("full queue has both entries valid", dut.fq_valid_q == 2'b11);
    check("full queue issues no unreserved request", !imem_req_valid);
    full_queue_seen = 1'b1;

    // the memory-pair veto consumes only the lower word. With both lines
    // already registered, then forms {current.upper, follower.lower} and
    // retains the follower upper word exactly once.
    check("memory pair is lower-only", decoded_slot_valid == 2'b01);
    consume_one();
    check("lower-only consume retains queue count", dut.fq_count_q == 2'd2);
    check("lower-only consume marks partial head", dut.fq_upper_q[dut.fq_head_q]);
    check("registered cross-line pair is offered", decoded_valid &&
          (decoded_slot_valid == 2'b11) &&
          (decoded_pc[0] == 32'h4) && (decoded_instr[0] == SW_X2) &&
          (decoded_pc[1] == 32'h8) && (decoded_instr[1] == ADDI_X3_3));
    partial_head_seen = 1'b1;
    consume_one();
    check("cross-line consume removes exactly one physical line",
          dut.fq_count_q == 2'd1);
    check("follower upper becomes the partial head", decoded_valid &&
          dut.fq_upper_q[dut.fq_head_q] &&
          (decoded_pc[0] == 32'hc) && (decoded_instr[0] == ADDI_X4_4));
    check("cross-line event counter advanced", dut.q1_cross_line_count == 1);

    // accept the newly-opened line-0x10 request, then redirect while it is
    // outstanding. Its later response must be accepted and discarded before
    // the target request can issue.
    accept_without_response(32'h0000_0010);
    @(negedge clk);
    redirect_target = 32'h0000_0024;
    redirect_valid  = 1'b1;
    @(posedge clk);
    @(negedge clk);
    redirect_valid  = 1'b0;
    check("redirect flushes queued fall-through line", dut.fq_count_q == 2'd0);
    check("redirect leaves no stale decode offer", !decoded_valid);
    check("killed in-flight request still accepts its response", imem_resp_ready);
    send_delayed_response(STALE_X9_99, STALE_X10);
    check("killed response did not refill queue", dut.fq_count_q == 2'd0);
    check("killed response payload never decoded", !decoded_valid);
    killed_response_seen = 1'b1;

    // The 0x18 prefetch offer was created at 0x10's acceptance and
    // is producer-held through the redirect, marked killed. It cannot be
    // withdrawn — drain it exactly like the killed in-flight request before
    // the target can issue.
    accept_without_response(32'h0000_0018);
    send_delayed_response(STALE_X9_99, STALE_X10);
    check("killed offer response did not refill queue", dut.fq_count_q == 2'd0);
    check("killed offer payload never decoded", !decoded_valid);

    // Redirect target 0x24 is the upper word of line 0x20. A same-cycle target
    // response must preserve that word identity and offer shape 01.
    accept_same_cycle_response(32'h0000_0020, ADDI_X5_5, TARGET_X6_6);
    check("upper-word redirect target retained", decoded_valid &&
          (decoded_pc[0] == 32'h24) &&
          (decoded_instr[0] == TARGET_X6_6) &&
          (decoded_slot_valid == 2'b01));
    upper_redirect_seen = 1'b1;

    // with the next request accepted, present its response on the exact
    // redirect edge. Redirect wins: both the queued target word and the
    // coincident old-path response disappear.
    accept_without_response(32'h0000_0028);
    @(negedge clk);
    redirect_target = 32'h0000_0040;
    redirect_valid  = 1'b1;
    imem_resp_data  = {DROP_X12, DROP_X11};
    imem_resp_valid = 1'b1;
    check("coincident response is accepted", imem_resp_ready);
    @(posedge clk);
    @(negedge clk);
    redirect_valid  = 1'b0;
    imem_resp_valid = 1'b0;
    imem_resp_data  = '0;
    check("redirect beats same-edge response", (dut.fq_count_q == 2'd0) &&
          !decoded_valid && !dut.req_inflight_q);
    same_edge_redirect_response_seen = 1'b1;

    // pipelined: drain the killed 0x30 prefetch offer left presented by the
    // same-edge redirect before the new target can issue.
    accept_without_response(32'h0000_0030);
    send_delayed_response(DROP_X11, DROP_X12);
    check("killed offer after same-edge redirect drained clean",
          (dut.fq_count_q == 2'd0) && !decoded_valid);

    accept_same_cycle_response(32'h0000_0040, ADDI_X13_13, ADDI_X14_14);
    check("post-priority target is first visible payload", decoded_valid &&
          (decoded_pc[0] == 32'h40) &&
          (decoded_instr[0] == ADDI_X13_13));

    // /: an upper-word 01 packet begins while its follower request is
    // delayed. Once valid is held against ready=0, the later response may not
    // expand that packet to shape 11 before acceptance.
    @(negedge clk);
    rst_n = 1'b0;
    imem_req_ready = 1'b0;
    imem_resp_valid = 1'b0;
    decoded_ready = 1'b0;
    redirect_valid = 1'b0;
    repeat (3) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    accept_same_cycle_response(32'h0000_0000, LW_X2, SW_X2);
    check("Q1 lock setup starts at lower word", decoded_valid &&
          (decoded_pc[0] == 32'h0) && (decoded_slot_valid == 2'b01));
    consume_one();
    check("Q1 lock setup exposes upper 01 packet", decoded_valid &&
          (decoded_pc[0] == 32'h4) && (decoded_instr[0] == SW_X2) &&
          (decoded_slot_valid == 2'b01));
    @(posedge clk);
    @(negedge clk);
    check("Q1 upper 01 packet locked under backpressure",
          dut.upper_solo_locked_q);
    accept_without_response(32'h0000_0008);
    send_delayed_response(ADDI_X3_3, ADDI_X4_4);
    @(posedge clk);
    @(negedge clk);
    check("late follower cannot expand stalled upper packet", decoded_valid &&
          (decoded_pc[0] == 32'h4) && (decoded_instr[0] == SW_X2) &&
          (decoded_slot_valid == 2'b01));
    stalled_upper_lock_seen = 1'b1;

    check("request-stall state entered", request_stall_seen);
    check("delayed-response state entered", delayed_response_seen);
    check("full-queue state entered", full_queue_seen);
    check("partial-head state entered", partial_head_seen);
    check("killed-response state entered", killed_response_seen);
    check("upper-word redirect state entered", upper_redirect_seen);
    check("same-edge redirect/response state entered",
          same_edge_redirect_response_seen);
    check("stalled upper-packet lock state entered", stalled_upper_lock_seen);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_fetch_queue] PASS checks=%0d", checks);
      $finish;
    end
    $fatal(1, "[tb_rv32i_ss_fetch_queue] FAIL errors=%0d checks=%0d",
           errors, checks);
  end
endmodule
