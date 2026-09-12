`timescale 1ns/1ps

// tb_rv32i_ss_btb -- directed battery: dual-slot BTB prediction.
//
// One continuous program through umbra_ss_cpu_top (real frontend + real
// rv32i_ss_bp + core), so the predict -> steer -> verify -> train loop closes
// end-to-end. Four proof obligations, each provably entered AND
// consequential (a verdict or steering error changes committed values):
//
//   slot-0 loop slot-0 loop learning + exit misprediction. A 6-iteration countdown
//      loop's backward branch: first taken resolution mispredicts (BTB
//      cold), trains, then iterations 2-5 are TAKEN-AND-CORRECT -- a
//      release-without-flush case -- and
//      the loop exit is predicted-taken-but-not-taken, recovering to the
//      FALLTHROUGH. Accumulator x1 = 21 pins the exact
//      iteration count against double-dispatch or lost iterations.
//
//   slot-1 loop slot-1 (dual-slot) prediction. A two-instruction aligned loop
//      {addi; bne} keeps the branch at the ODD word: the prediction rides
//      slot 1 of a dual bundle (entered-pin) and steers fetch back to the
//      bundle start.
//
//   branch-alias case BTB aliasing / decode-exact target repair. Branch A at 0x040 trains its
//      target; branch B at 0x10040 (same idx AND partial tag -- the BTB ignores
//      pc[31:16]) hits A's entry and is steered to A's stale target. B
//      is recognized as a conditional branch at dequeue, repairs to its exact
//      target before dispatch completes, and carries that corrected target to
//      execute; the wrong-path re-fetch of A's region (including an addi
//      that would corrupt x7) is rolled back. This is the leg that makes
//      small-BTB aliasing safe.
//
//   backpressure case predicted-bundle-under-backpressure. A countdown loop whose body is
//      {addi; csrrw mscratch; bne}: from iteration 2 the trained bne
//      presents PREDICTED-TAKEN while csr_inflight still blocks dispatch --
//      the exact window where an ungated pc steer would walk fetch away
//      from a bundle that never fired, vanishing the branch from the
//      instruction stream. Entered-pin: a predicted bundle observed with
//      decoded_ready low. (The frontend's own tripwires --
//      prediction-on-non-branch, both-slots-predicted, predicted-taken
//      slot-0 in a dual bundle -- stay armed through every leg.)
//
// Mutation consequences (one mutation per run):
//   mispredict without the target-compare term -> the candidate
//     coherence tripwire fatals (independent recompute); with that
//     recompute weakened identically, the aliasing case's arch checks catch the
//     wrong path (x7 corrupted to -1, alias pin unreached)
//   correct_valid without !mispredict (release every resolve) ->
//     the combiner's standing correct&&recover-exclusivity tripwire
//     fatals on the first mispredicted resolution
//   formation_dual_legal without !pred_taken0 -> the frontend's
//     formation tripwire fatals on slot-0 loop's first predicted-taken iteration
//     (slot-0 predicted-taken branch offered in a dual bundle)
//   pc steer without the bundle_fire gate -> backpressure case: fetch walks away
//     from the csr-blocked predicted bne, the branch never dispatches,
//     the loop unravels: arch checks catch x13/x14

module tb_rv32i_ss_btb;
  import rv32i_ss_pkg::*;

  localparam time CLK_PERIOD = 10ns;

  logic clk;
  logic rst_n;

  // 256 KiB imem: branch-alias case/non-branch-alias case use three sites 64 KiB apart (TAG_W=8 alias
  // distance), so the repair proof remains entered as tag width grows.
  word_t imem_addr;
  word_t [1:0] imem_rdata;
  logic imem_req_valid, imem_req_ready;
  word_t imem_req_addr;
  logic imem_resp_valid, imem_resp_ready;
  word_t [1:0] imem_resp_data;
  logic [31:0] imem [0:65535];
  assign imem_rdata = {imem[{imem_addr[17:3], 1'b1}], imem[{imem_addr[17:3], 1'b0}]};

  rv32i_ss_imem_zero_latency_adapter u_imem_adapter (
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr), .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready), .imem_resp_data(imem_resp_data),
    .line_addr(imem_addr), .line_data(imem_rdata)
  );

  int errors = 0;
  int checks = 0;

  umbra_ss_cpu_top u_cpu (
    .clk        (clk),
    .rst_n      (rst_n),
    .imem_req_valid  (imem_req_valid),
    .imem_req_ready  (imem_req_ready),
    .imem_req_addr   (imem_req_addr),
    .imem_resp_valid (imem_resp_valid),
    .imem_resp_ready (imem_resp_ready),
    .imem_resp_data  (imem_resp_data)
  );

  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;

  initial begin
    repeat (8000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_btb exceeded 8000 cycles");
  end

  `define CORE u_cpu.u_core
  `define ROB  u_cpu.u_core.u_rob
  `define RN   u_cpu.u_core.u_rename
  `define PRF  u_cpu.u_core.u_prf

  task automatic check_bit(input string name, input logic got, input logic exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got=%0b exp=%0b", name, got, exp);
      errors++;
    end
  endtask

  task automatic check_arch(input string name, input int r, input word_t exp);
    automatic phys_reg_t p = `RN.committed_map_q[r];
    checks++;
    if (`PRF.regs_q[p] !== exp) begin
      $error("[%s] x%0d=%08h (via p%0d) expected %08h",
             name, r, `PRF.regs_q[p], p, exp);
      errors++;
    end
  endtask

  // ---------------- monitors ----------------
  int  n_pred_steer;              // bundles dispatched with a live prediction
  bit  taken_correct_seen;        // release-without-flush on a TAKEN branch
  bit  fallthrough_recover_seen;  // recovery to pc+4 (pred-taken, actually NT)
  bit  alias_target_repair_seen;  // accepted-dequeue stale-target repair
  bit  pred_slot1_seen;           // prediction riding slot 1 of a dual bundle
  bit  pred_stall_seen;           // predicted bundle presented while blocked
  bit  marker_seen;               // x14 <- 14 committed (program end)
  bit  final_marker_seen;         // x16 <- 16 after the false-type leg

  always @(posedge clk) begin
    if (rst_n === 1'b1) begin
      if (`CORE.bundle_fire && (|`CORE.decoded_pred_taken))
        n_pred_steer++;
      if (`CORE.bundle_fire && (`CORE.decoded_slot_valid == 2'b11) &&
          `CORE.decoded_pred_taken[1])
        pred_slot1_seen = 1'b1;
      if (`CORE.decoded_valid && !`CORE.decoded_ready &&
          (|`CORE.decoded_pred_taken))
        pred_stall_seen = 1'b1;

      if ((`CORE.branch_candidate[0].correct_valid && `CORE.branch_taken[0]) ||
          (`CORE.branch_candidate[1].correct_valid && `CORE.branch_taken[1]))
        taken_correct_seen = 1'b1;
      if ((`CORE.branch_candidate[0].recover_valid && !`CORE.branch_taken[0]) ||
          (`CORE.branch_candidate[1].recover_valid && !`CORE.branch_taken[1]))
        fallthrough_recover_seen = 1'b1;
      if (u_cpu.u_fe.steer_repair_fire &&
          u_cpu.u_fe.head_pred_target_mismatch)
        alias_target_repair_seen = 1'b1;

      if ((`CORE.commit_fire[0] && `CORE.commit_rd_wen[0] &&
           (`CORE.commit_rd[0] == 14) && (`CORE.commit_wdata[0] == 32'd14)) ||
          (`CORE.commit_fire[1] && `CORE.commit_rd_wen[1] &&
           (`CORE.commit_rd[1] == 14) && (`CORE.commit_wdata[1] == 32'd14)))
        marker_seen = 1'b1;
      if ((`CORE.commit_fire[0] && `CORE.commit_rd_wen[0] &&
           (`CORE.commit_rd[0] == 16) && (`CORE.commit_wdata[0] == 32'd16)) ||
          (`CORE.commit_fire[1] && `CORE.commit_rd_wen[1] &&
           (`CORE.commit_rd[1] == 16) && (`CORE.commit_wdata[1] == 32'd16)))
        final_marker_seen = 1'b1;
    end
  end

  integer i;
  int wait_i;

  initial begin
    $display("[tb_rv32i_ss_btb] starting");
    n_pred_steer = 0;
    taken_correct_seen = 1'b0;
    fallthrough_recover_seen = 1'b0;
    alias_target_repair_seen = 1'b0;
    pred_slot1_seen = 1'b0;
    pred_stall_seen = 1'b0;
    marker_seen = 1'b0;
    final_marker_seen = 1'b0;

    for (i = 0; i < 65536; i = i + 1) imem[i] = 32'h0000_0013;

    // ---- slot-0 loop: countdown loop, branch at 0x10 (slot 0 after redirects) ----
    imem[0]  = 32'h0000_0093;  // 0x00 addi x1,x0,0
    imem[1]  = 32'h0060_0113;  // 0x04 addi x2,x0,6
    imem[2]  = 32'h0020_80b3;  // 0x08 add  x1,x1,x2
    imem[3]  = 32'hfff1_0113;  // 0x0c addi x2,x2,-1
    imem[4]  = 32'hfe01_1ce3;  // 0x10 bne  x2,x0,-8 -> 0x08
    imem[5]  = 32'h0210_0193;  // 0x14 addi x3,x0,33
    // ---- slot-1 loop: two-instruction aligned loop, branch at the ODD word 0x24 ----
    imem[6]  = 32'h0050_0293;  // 0x18 addi x5,x0,5
    imem[7]  = 32'h0000_0013;  // 0x1c nop (align the loop to 0x20)
    imem[8]  = 32'hfff2_8293;  // 0x20 addi x5,x5,-1   (slot 0)
    imem[9]  = 32'hfe02_9ee3;  // 0x24 bne  x5,x0,-4 -> 0x20  (slot 1)
    imem[10] = 32'h0420_0313;  // 0x28 addi x6,x0,66
    // ---- branch-alias case: branch A at 0x40 (idx 16, tag 0), then jump to the alias ----
    imem[11] = 32'h0030_0393;  // 0x2c addi x7,x0,3
    imem[15] = 32'hfff3_8393;  // 0x3c addi x7,x7,-1   (A-loop body)
    imem[16] = 32'hfe03_9ee3;  // 0x40 bne  x7,x0,-4 -> 0x3c  (branch A)
    imem[17] = 32'h7bd0_f4ef;  // 0x44 jal x9,+0xffbc -> 0x10000
    // ---- branch-alias case alias region: branch B at 0x10040 (same idx 16, tag 0) ----
    imem[16384] = 32'h0010_0513;  // 0x10000 addi x10,x0,1
    imem[16400] = 32'h0205_1063;  // 0x10040 bne x10,x0,+0x20 -> 0x10060
    imem[16401] = 32'h0630_0593;  // 0x10044 POISON addi x11,x0,99
    imem[16408] = 32'h0016_0613;  // 0x10060 addi x12,x12,1
    // ---- backpressure case: predicted bne under csr_inflight backpressure ----
    imem[16412] = 32'h0040_0693;  // 0x10070 addi x13,x0,4
    imem[16416] = 32'hfff6_8693;  // 0x10080 addi x13,x13,-1 (loop top)
    imem[16417] = 32'h3400_1073;  // 0x10084 csrrw x0,mscratch,x0
    imem[16418] = 32'hfe06_9ce3;  // 0x10088 bne x13,x0,-8 -> 0x10080
    imem[16419] = 32'h00e0_0713;  // 0x1008c addi x14,x0,14
    imem[16420] = 32'h7b10_f06f;  // 0x10090 jal x0,+0xffb0 -> 0x20040
    // ---- non-branch-alias case: non-branch alias at 0x20040. Branch B's still-taken row points
    // to 0x10060. Decode must repair at accepted dequeue before that old
    //      target can execute a second time. ----
    imem[32784] = 32'h00f0_0793;  // 0x20040 addi x15,x0,15 (false-type site)
    imem[32785] = 32'h0110_0893;  // 0x20044 addi x17,x0,17
    imem[32786] = 32'h0100_0813;  // 0x20048 addi x16,x0,16 (final marker)
    imem[32787] = 32'h0000_006f;  // 0x2004c jal x0,0

    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;

    wait_i = 0;
    while (!final_marker_seen) begin
      @(posedge clk);
      wait_i++;
      if (wait_i > 4000) $fatal(1, "stuck: end marker never committed (order=%0d)",
                                `ROB.commit_order_q);
    end
    repeat (4) @(posedge clk);

    // ---- entered pins ----
    check_bit("L1/L2: predictions steered fetch (>=6 bundles)",
              n_pred_steer >= 6, 1'b1);
    check_bit("L1: taken-and-correct release entered", taken_correct_seen, 1'b1);
    check_bit("L1/L2: fallthrough recovery entered", fallthrough_recover_seen, 1'b1);
    check_bit("L2: prediction rode slot 1 of a dual bundle", pred_slot1_seen, 1'b1);
    check_bit("L3: alias stale-target repair entered",
              alias_target_repair_seen, 1'b1);
    check_bit("L4: predicted bundle observed under backpressure",
              pred_stall_seen, 1'b1);
    check_bit("D-015: lower-half request steering entered",
              u_cpu.u_fe.d015_req_branch_lower_count > 0, 1'b1);
    check_bit("D-015: upper-half request steering entered",
              u_cpu.u_fe.d015_req_branch_upper_count > 0, 1'b1);
    check_bit("L5: non-branch false-type repair entered",
              u_cpu.u_fe.d015_false_type_count > 0, 1'b1);

    // ---- architectural consequence ----
    check_arch("L1 x1 loop sum",        1,  32'd21);
    check_arch("L1 x2 counter drained", 2,  32'd0);
    check_arch("L1 x3 exit marker",     3,  32'd33);
    check_arch("L2 x5 counter drained", 5,  32'd0);
    check_arch("L2 x6 exit marker",     6,  32'd66);
    check_arch("L3 x7 wrong-path decrement rolled back", 7, 32'd0);
    check_arch("L3 x9 jal link",        9,  32'h0000_0048);
    check_arch("L3 x10 B condition",    10, 32'd1);
    check_arch("L3 x11 B fallthrough never committed", 11, 32'd0);
    check_arch("L3/L5 x12 old target executed once", 12, 32'd1);
    check_arch("L4 x13 counter drained", 13, 32'd0);
    check_arch("L4 x14 end marker",     14, 32'd14);
    check_arch("L5 x15 false-site lower committed", 15, 32'd15);
    check_arch("L5 x16 final marker", 16, 32'd16);
    check_arch("L5 x17 false-site upper committed", 17, 32'd17);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_btb] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_btb] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
