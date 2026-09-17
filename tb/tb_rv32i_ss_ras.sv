// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// tb_rv32i_ss_ras — return-address-stack and pop-JALR speculation tests.
// A continuous program through umbra_ss_cpu_top checks push, predict, steer,
// verify, recover and release, with real branch recovery and trap flushes.
// Each scenario has entry counters and architectural consequences.
//
// Coverage:
//   - Three nested calls prime three correctly predicted returns.
//   - A wrong-path push/pop pair checks content survival across recovery.
//     Because the pair is pointer-balanced, it alone cannot detect an omitted
//     pointer restore. A second case leaves the wrong-path push unbalanced;
//     the true return must use the restored stack rather than the stale entry.
//   - Nine nested calls overflow the eight-entry stack. Eight returns predict,
//     then the saturated-empty stack takes the serialized fallback.
//   - A clobbered return address forces recovery. The next return uses the
//     restored post-pop empty state; a pre-pop snapshot would predict again.
//   - A trap between call and return clears the stack, so the post-mret return
//     is serialized. Nop padding prevents wrong-path returns from obscuring
//     the trap-clear effect. mret redirects at commit, so its fallthrough
//     shadow may dispatch an ecall that is flushed before it commits.
//   - A predicted return with a misaligned actual target suppresses normal
//     verdicts, takes IADDR_MISALIGN at commit and reclaims its checkpoint.
//
// Exact counters (n_pred=16, n_jump_alloc=3, n_jump_recover=1,
// n_branch_recover=2) distinguish missing recovery restore, pre-pop snapshots,
// missing trap clear and prediction from an empty stack. Architectural checks
// also reject committed wrong-path effects.

module tb_rv32i_ss_ras;
  import rv32i_ss_pkg::*;

  localparam time CLK_PERIOD = 10ns;

  logic clk;
  logic rst_n;

  word_t imem_addr;
  word_t [1:0] imem_rdata;
  logic imem_req_valid, imem_req_ready;
  word_t imem_req_addr;
  logic imem_resp_valid, imem_resp_ready;
  word_t [1:0] imem_resp_data;
  logic [31:0] imem [0:255];
  assign imem_rdata = {imem[{imem_addr[9:3], 1'b1}], imem[{imem_addr[9:3], 1'b0}]};

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
    repeat (6000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_ras exceeded 6000 cycles");
  end

  `define CORE u_cpu.u_core
  `define RN   u_cpu.u_core.u_rename
  `define PRF  u_cpu.u_core.u_prf

  task automatic check_bit(input string name, input logic got, input logic exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got=%0b exp=%0b", name, got, exp);
      errors++;
    end
  endtask

  task automatic check_int(input string name, input int got, input int exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got=%0d exp=%0d", name, got, exp);
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
  int n_pred;            // ras_pred_fire count (predicted-return dispatches)
  int n_jump_alloc;      // serialized-jump freezes
  int n_jump_recover;    // jump-class recoveries on the candidate-0 seam
  int n_branch_recover;  // branch-class recoveries (either candidate)
  bit pred_under_rem;    // wrong-path prediction inside the rem shadow
  bit ecall_trap_seen;   // entered
  bit misalign_trap_seen;// entered
  bit marker_seen;

  always @(posedge clk) begin
    if (rst_n === 1'b1) begin
      if (`CORE.ras_pred_fire) begin
        n_pred++;
        // note: the flag anchors on the wrong-path sled ret's PC
        // (0x0c4) rather than muldiv_busy overlap -- early wakeup shifts
        // when the rem ISSUES relative to the shadow's dispatch, but the
        // sled ret is wrong-path by construction (only reachable while
        // the mispredicted bne is unresolved), so the pc IS the window.
        if (`CORE.decoded_pc[0] == 32'h0000_00c4) pred_under_rem = 1'b1;
      end
      if (`CORE.jump_alloc_fire) n_jump_alloc++;
      if (`CORE.branch_candidate[0].recover_valid) begin
        if (`CORE.alu0_exec_entry.op_class == OOO_OP_JUMP) n_jump_recover++;
        else n_branch_recover++;
      end
      if (`CORE.branch_candidate[1].recover_valid) n_branch_recover++;
      if (`CORE.trap_q_valid && !`CORE.trap_q_is_mret) begin
        if (`CORE.trap_q_cause == OOO_CAUSE_ECALL_M)        ecall_trap_seen = 1'b1;
        if (`CORE.trap_q_cause == OOO_CAUSE_IADDR_MISALIGN) misalign_trap_seen = 1'b1;
      end
      if ((`CORE.commit_fire[0] && `CORE.commit_rd_wen[0] &&
           (`CORE.commit_rd[0] == 14) && (`CORE.commit_wdata[0] == 32'd14)) ||
          (`CORE.commit_fire[1] && `CORE.commit_rd_wen[1] &&
           (`CORE.commit_rd[1] == 14) && (`CORE.commit_wdata[1] == 32'd14)))
        marker_seen = 1'b1;
    end
  end

  integer i;
  int wait_i;

  initial begin
    $display("[tb_rv32i_ss_ras] starting");
    n_pred = 0; n_jump_alloc = 0; n_jump_recover = 0; n_branch_recover = 0;
    pred_under_rem = 1'b0; ecall_trap_seen = 1'b0; misalign_trap_seen = 1'b0;
    marker_seen = 1'b0;

    for (i = 0; i < 256; i = i + 1) imem[i] = 32'h0000_0013;

    // Program generated by the mini-assembler (expected x7 = 607).
    imem[  0] = 32'h00000393;  // 0x000 addi x7,x0,0
    imem[  1] = 32'h00700B93;  // 0x004 addi x23,x0,7   (rem dividend)
    imem[  2] = 32'h00300C13;  // 0x008 addi x24,x0,3   (rem divisor)
    imem[  3] = 32'h034000EF;  // 0x00c jal x1,A call (push 0x010)
    imem[  4] = 32'h04038393;  // 0x010 addi x7,x7,64 return-here
    imem[  5] = 32'h07C0006F;  // 0x014 jal x0,main2
    imem[ 16] = 32'h00008A13;  // 0x040 A: save ra -> x20
    imem[ 17] = 32'h00138393;  // 0x044 addi x7,x7,1
    imem[ 18] = 32'h018000EF;  // 0x048 jal x1,B (push)
    imem[ 19] = 32'h00238393;  // 0x04c addi x7,x7,2
    imem[ 20] = 32'h000A0093;  // 0x050 restore ra
    imem[ 21] = 32'h00008067;  // 0x054 ret  (predicted)
    imem[ 24] = 32'h00008A93;  // 0x060 B: save ra -> x21
    imem[ 25] = 32'h00438393;  // 0x064 addi x7,x7,4
    imem[ 26] = 32'h018000EF;  // 0x068 jal x1,C (push)
    imem[ 27] = 32'h00838393;  // 0x06c addi x7,x7,8
    imem[ 28] = 32'h000A8093;  // 0x070 restore ra
    imem[ 29] = 32'h00008067;  // 0x074 ret  (predicted)
    imem[ 32] = 32'h01038393;  // 0x080 C: addi x7,x7,16
    imem[ 33] = 32'h00008067;  // 0x084 ret  (predicted)
    imem[ 36] = 32'h010000EF;  // 0x090 main2: jal x1, (push 0x094)
    imem[ 37] = 32'h04038393;  // 0x094 addi x7,x7,64  true return-here
    imem[ 38] = 32'h0300006F;  // 0x098 jal x0,main2b
    imem[ 40] = 32'h038BEB33;  // 0x0a0 : rem x22,x23,x24 (=1, 32-cycle)
    imem[ 41] = 32'h000B1663;  // 0x0a4 bne x22,x0, TAKEN, cold-mispredicted
    imem[ 42] = 32'h018000EF;  // 0x0a8 WRONG-PATH: jal x1,sled (push)
    imem[ 43] = 32'h0000006F;  // 0x0ac WRONG-PATH: self-loop (sled-ret lands here)
    imem[ 44] = 32'h02038393;  // 0x0b0 : addi x7,x7,32
    imem[ 45] = 32'h00008067;  // 0x0b4 true ret (must still predict after recovery)
    imem[ 48] = 32'h00100C93;  // 0x0c0 sled: x25 poison (wrong-path only)
    imem[ 49] = 32'h00008067;  // 0x0c4 WRONG-PATH ret (pops its own push, predicts)
    // unbalanced shadow (push, never popped) -- the kill. The
    // bne shares BTB index 9 with the preceding scenario but differs in tag -> miss ->
    // mispredicts the same way. main2b falls through into main3.
    imem[ 50] = 32'h018000EF;  // 0x0c8 main2b: jal x1, (push 0x0cc)
    imem[ 51] = 32'h04038393;  // 0x0cc addi x7,x7,64 (falls into main3)
    imem[ 56] = 32'h038BEB33;  // 0x0e0 : rem x22,x23,x24 (slow again)
    imem[ 57] = 32'h000B1663;  // 0x0e4 bne x22,x0, TAKEN, tag-miss-mispredicted
    imem[ 58] = 32'h010000EF;  // 0x0e8 WRONG-PATH: jal x1,sled2 (push, NEVER popped)
    imem[ 60] = 32'h02038393;  // 0x0f0 : addi x7,x7,32
    imem[ 61] = 32'h00008067;  // 0x0f4 true ret (post-restore: 0x0cc; : sled2's 0x0ec)
    imem[ 62] = 32'h00100C93;  // 0x0f8 sled2: x25 poison (wrong-path only)
    imem[ 63] = 32'h0000006F;  // 0x0fc self-loop -- the push is never popped
    imem[ 52] = 32'h030000EF;  // 0x0d0 main3: jal x1,f1 (push 0x0d4)
    imem[ 53] = 32'h04038393;  // 0x0d4 addi x7,x7,64
    imem[ 54] = 32'h1380006F;  // 0x0d8 jal x0,main4
    imem[ 64] = 32'h00008413;  // 0x100 f1: save ra -> x8
    imem[ 65] = 32'h01C000EF;  // 0x104 jal x1,f2 (push)
    imem[ 66] = 32'h00040093;  // 0x108 restore ra
    imem[ 67] = 32'h00008067;  // 0x10c ret
    imem[ 72] = 32'h00008493;  // 0x120 f2: save ra -> x9
    imem[ 73] = 32'h01C000EF;  // 0x124 jal x1,f3 (push)
    imem[ 74] = 32'h00048093;  // 0x128 restore ra
    imem[ 75] = 32'h00008067;  // 0x12c ret
    imem[ 80] = 32'h00008513;  // 0x140 f3: save ra -> x10
    imem[ 81] = 32'h01C000EF;  // 0x144 jal x1,f4 (push)
    imem[ 82] = 32'h00050093;  // 0x148 restore ra
    imem[ 83] = 32'h00008067;  // 0x14c ret
    imem[ 88] = 32'h00008593;  // 0x160 f4: save ra -> x11
    imem[ 89] = 32'h01C000EF;  // 0x164 jal x1,f5 (push)
    imem[ 90] = 32'h00058093;  // 0x168 restore ra
    imem[ 91] = 32'h00008067;  // 0x16c ret
    imem[ 96] = 32'h00008613;  // 0x180 f5: save ra -> x12
    imem[ 97] = 32'h01C000EF;  // 0x184 jal x1,f6 (push)
    imem[ 98] = 32'h00060093;  // 0x188 restore ra
    imem[ 99] = 32'h00008067;  // 0x18c ret
    imem[104] = 32'h00008793;  // 0x1a0 f6: save ra -> x15
    imem[105] = 32'h01C000EF;  // 0x1a4 jal x1,f7 (push)
    imem[106] = 32'h00078093;  // 0x1a8 restore ra
    imem[107] = 32'h00008067;  // 0x1ac ret
    imem[112] = 32'h00008813;  // 0x1c0 f7: save ra -> x16
    imem[113] = 32'h01C000EF;  // 0x1c4 jal x1,f8 (push)
    imem[114] = 32'h00080093;  // 0x1c8 restore ra
    imem[115] = 32'h00008067;  // 0x1cc ret
    imem[120] = 32'h00008893;  // 0x1e0 f8: save ra -> x17
    imem[121] = 32'h01C000EF;  // 0x1e4 jal x1,f9 (push)
    imem[122] = 32'h00088093;  // 0x1e8 restore ra
    imem[123] = 32'h00008067;  // 0x1ec ret
    imem[128] = 32'h08038393;  // 0x200 f9: addi x7,x7,128
    imem[129] = 32'h00008067;  // 0x204 ret (deepest)
    imem[132] = 32'h010000EF;  // 0x210 main4: jal x1,G (push 0x214)
    imem[133] = 32'h06300693;  // 0x214 POISON x13 (predicted-path, must roll back)
    imem[134] = 32'h0000006F;  // 0x218 wrong-path self-loop
    imem[136] = 32'h00000097;  // 0x220 G: x1 = 0x220
    imem[137] = 32'h02008093;  // 0x224 x1 = Z
    imem[138] = 32'h00008067;  // 0x228 ret: predicted 0x214, actual Z -> JUMP RECOVERY
    imem[144] = 32'h08038393;  // 0x240 Z: addi x7,x7,128
    imem[145] = 32'h00000097;  // 0x244 x1 = 0x244
    imem[146] = 32'h01408093;  // 0x248 x1 = W
    imem[147] = 32'h00008067;  // 0x24c forced ret #2: empty stack -> SERIALIZED
    imem[150] = 32'h0080006F;  // 0x258 W: jal x0,main5
    imem[152] = 32'h00000E17;  // 0x260 main5: x28 = 0x260
    imem[153] = 32'h020E0E13;  // 0x264 x28 = handler
    imem[154] = 32'h305E1073;  // 0x268 csrw mtvec,x28
    imem[155] = 32'h034000EF;  // 0x26c jal x1,H (push 0x270)
    imem[156] = 32'h04038393;  // 0x270 addi x7,x7,64  post-return
    imem[157] = 32'h06C0006F;  // 0x274 jal x0,main6
    imem[160] = 32'h34102E73;  // 0x280 handler: x28 = mepc
    imem[161] = 32'h004E0E13;  // 0x284 x28 += 4
    imem[162] = 32'h341E1073;  // 0x288 csrw mepc,x28
    imem[163] = 32'h30200073;  // 0x28c mret
    // H: the ecall's fallthrough shadow (and the mret shadows that
    // re-enter H) burn in the default-nop field 0x2a4..0x2c8 -- no ret is
    // reachable before a flush lands, so the stack going empty across the
    // trap is attributable ONLY to the trap-clear (kills ). The
    // real ret sits past the shadow, reached by the true post-mret path.
    imem[168] = 32'h00000073;  // 0x2a0 H: ecall (trap; stack cleared)
    imem[179] = 32'h00008067;  // 0x2cc post-mret ret (past the shadow): SERIALIZED
    imem[184] = 32'h020000EF;  // 0x2e0 main6: jal x1,K (push 0x2e4)
    imem[185] = 32'h06300693;  // 0x2e4 POISON x13 (predicted-path, flushed)
    imem[186] = 32'h0000006F;  // 0x2e8 wrong-path self-loop
    imem[192] = 32'h00208093;  // 0x300 K: ra += 2 (misaligned)
    imem[193] = 32'h00008067;  // 0x304 ret: predicted 0x2e4, actual 0x2e6 -> IADDR trap
    imem[194] = 32'h02038393;  // 0x308 continuation via mret (mepc+4)
    imem[195] = 32'h0140006F;  // 0x30c jal x0,fin
    imem[200] = 32'h00E00713;  // 0x320 fin: end marker x14=14
    imem[201] = 32'h0000006F;  // 0x324 self-loop

    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;

    wait_i = 0;
    while (!marker_seen) begin
      @(posedge clk);
      wait_i++;
      if (wait_i > 5000) $fatal(1, "stuck: end marker never committed");
    end
    repeat (4) @(posedge clk);

    // ---- exact prediction-outcome totals (derived in the header) ----
    // n_pred: 3, 2 (wrong-path sled ret + post-recovery true ret),
    // 1 (post-recovery true ret), 8, 1 (recovered), 0
    // (shadow is ret-free BY DESIGN), 1.
    // n_jump_alloc: underflow ret, second forced ret, post-mret
    // ret (empty stack, the pin).
    check_int("n_pred (predicted-return dispatches)", n_pred, 16);
    check_int("n_jump_alloc (serialized fallbacks)",  n_jump_alloc, 3);
    check_int("n_jump_recover (P4 forced mispredict)", n_jump_recover, 1);
    check_int("n_branch_recover (P2a+P2b bnes)",       n_branch_recover, 2);
    check_bit("P2: wrong-path sled-ret prediction entered",
              pred_under_rem, 1'b1);
    check_bit("P5: ecall trap entered", ecall_trap_seen, 1'b1);
    check_bit("P6: misaligned-return trap entered", misalign_trap_seen, 1'b1);

    // ---- architectural consequence ----
    check_arch("x7 phase accumulator", 7,  32'd703);
    check_arch("x22 rem result committed", 22, 32'd1);
    check_arch("x13 predicted-path poisons rolled back", 13, 32'd0);
    check_arch("x25 branch-shadow poison rolled back",   25, 32'd0);
    check_arch("x14 end marker", 14, 32'd14);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_ras] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_ras] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
