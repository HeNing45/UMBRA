// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// tb_rv32i_ss_jal_deser -- directed battery: JAL de-serialization.
//
// Drives umbra_ss_cpu_top (real frontend + core) with real instruction
// encodings so the fetch -> decode -> dispatch-redirect -> fetch loop closes
// end-to-end. Three phases, each provably entered AND consequential
// (mishandling changes committed architectural state):
//
// Scenario 1 -- de-serialization proper. Three aligned JALs (two forward, one
//     backward, distinct link registers), a j (rd=x0) self-loop terminator,
//     and a serialized JALR leg in one control chain. Every JAL shadow
//     carries a poison addi that must never commit. Entered-pins:
//     jal_deser_fire observed at each JAL's pc; the TARGET bundle dispatches
//     on the very next cycle (the de-serialization property itself -- a
//     serialized design scores 0 here); jump_alloc_fire and
//     jump_resolve_fire fire exactly ONCE each (the JALR only -- a
//     double-redirecting JAL scores >1 resolve and re-dispatches its target
//     path, which the arch checks catch as duplicated commits).
//
// Scenario 2 -- wrong-path JAL under a mispredicted branch. A REM-delayed
//     taken branch shadows a speculative JAL; the JAL's dispatch redirect
//     fires on the wrong path (entered-pin), its target op dispatches, then
//     the registered recovery must roll back the JAL, its link write, and
//     its target path, and win the redirect priority. Consequence: the
//     wrong-path link/target registers must read 0 at the end.
//
// Scenario 3 -- misaligned-target JAL keeps the serialized fallback. jal
//     x11,+2 has target pc+2 (bit[1] set): the de-serialization predicate
//     must EXCLUDE it (no jal_deser_fire), the serialized path must engage
//     (jump_alloc_fire exactly once), and the proven execute-side
//     IADDR_MISALIGN trap must land: mcause==0, mepc==the JAL's pc, no link
//     write, handler reached through mtvec.
//
// Mutation consequences:
//   revert jump_alloc_fire to all-jumps (serialize JALs again)
// -> Scenario 1 "target dispatched next cycle" pins score 0
//   jump_resolve_fire widened to all jumps (deser JAL redirects at
// issue too) -> Scenario 1 resolve-count pin (==1) fails
// tripwire allowed-set without jal_deser_fire
//     -> the DUT's own redirect tripwire $fatals on the first JAL dispatch
//   dispatch_accept without ~jump_inflight_q -> with a JAL placed in
// the JALR's dispatch shadow (scratch stimulus variant), the DUT's
//     mutual-exclusion tripwire $fatals; with the stock program the leak
//     shows as committed shadow poison (x7=96) in the arch checks

module tb_rv32i_ss_jal_deser;
  import rv32i_ss_pkg::*;

  localparam time CLK_PERIOD = 10ns;

  logic clk;
  logic rst_n;

  // instruction memory (line shape, as tb_rv32i_ss_core_prog)
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
    $fatal(1, "WATCHDOG: tb_rv32i_ss_jal_deser exceeded 6000 cycles");
  end

  `define CORE u_cpu.u_core
  `define ROB  u_cpu.u_core.u_rob
  `define RN   u_cpu.u_core.u_rename
  `define PRF  u_cpu.u_core.u_prf
  `define CSRF u_cpu.u_core.u_csr_file

  task automatic check_bit(input string name, input logic got, input logic exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got=%0b exp=%0b", name, got, exp);
      errors++;
    end
  endtask

  task automatic check_word(input string name, input word_t got, input word_t exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got=%08h exp=%08h", name, got, exp);
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

  // ---------------- monitors (per-phase, cleared by mon_clear) -------------
  bit    mon_on;
  int    n_deser_fire;
  int    n_jump_alloc;
  int    n_jump_resolve;
  int    n_recover;
  int    n_trap;
  // per-pc entered flags for Scenario 1/2 JALs
  bit    deser_at_08, deser_at_28, deser_at_54, deser_at_0c;
  bit    next_ok_08, next_ok_28, next_ok_54;
  // keeps the dispatch-time JAL redirect but inserts queue flush/refill.
  // Pin that the first subsequent dispatch is the exact target; cycle N+1 is
  // no longer the contract once instruction delivery is registered.
  bit    deser_target_pending;
  word_t pending_deser_pc, pending_deser_tgt;
  bit    wrong_dispatch_after_deser;

  task automatic mon_clear();
    n_deser_fire  = 0;
    n_jump_alloc  = 0;
    n_jump_resolve = 0;
    n_recover     = 0;
    n_trap        = 0;
    deser_at_08 = 1'b0; deser_at_28 = 1'b0; deser_at_54 = 1'b0;
    deser_at_0c = 1'b0;
    next_ok_08 = 1'b0; next_ok_28 = 1'b0; next_ok_54 = 1'b0;
    deser_target_pending = 1'b0;
    wrong_dispatch_after_deser = 1'b0;
    mon_on = 1'b1;
  endtask

  always @(posedge clk) begin
    if (rst_n === 1'b1 && mon_on) begin
      if (`CORE.jal_deser_fire) begin
        n_deser_fire++;
        if (`CORE.decoded_pc[0] == 32'h0000_0008) deser_at_08 = 1'b1;
        if (`CORE.decoded_pc[0] == 32'h0000_0028) deser_at_28 = 1'b1;
        if (`CORE.decoded_pc[0] == 32'h0000_0054) deser_at_54 = 1'b1;
        if (`CORE.decoded_pc[0] == 32'h0000_000c) deser_at_0c = 1'b1;
      end
      if (`CORE.jump_alloc_fire)      n_jump_alloc++;
      if (`CORE.jump_resolve_fire)    n_jump_resolve++;
      if (`CORE.branch_recover_req)   n_recover++;
      if (`CORE.trap_q_valid)         n_trap++;

      if (deser_target_pending && `CORE.bundle_fire) begin
        if (`CORE.decoded_pc[0] == pending_deser_tgt) begin
          if (pending_deser_pc == 32'h0000_0008) next_ok_08 = 1'b1;
          if (pending_deser_pc == 32'h0000_0028) next_ok_28 = 1'b1;
          if (pending_deser_pc == 32'h0000_0054) next_ok_54 = 1'b1;
        end else begin
          wrong_dispatch_after_deser = 1'b1;
        end
        deser_target_pending = 1'b0;
      end
      if (`CORE.jal_deser_fire) begin
        deser_target_pending = 1'b1;
        pending_deser_pc     = `CORE.decoded_pc[0];
        pending_deser_tgt    = `CORE.jal_deser_target;
      end
    end
  end

  task automatic fill_nops();
    for (int i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  endtask

  task automatic pulse_reset();
    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
  endtask

  int wait_i;

  task automatic wait_commit_order(input longint unsigned target, input int limit);
    wait_i = 0;
    while (`ROB.commit_order_q < target) begin
      @(posedge clk);
      wait_i++;
      if (wait_i > limit)
        $fatal(1, "stuck waiting commit_order>=%0d (at %0d)",
               target, `ROB.commit_order_q);
    end
  endtask

  initial begin
    $display("[tb_rv32i_ss_jal_deser] starting");

    // ================= Scenario 1: de-serialization proper =================
    // 0x00 addi x9,x0,0x60   0x04 addi x1,x0,1
    // 0x08 jal x5,+0x1C ->0x24   [0x0c/0x10 poison]
    // 0x24 addi x2,x0,2      0x28 jal x4,+0x28 ->0x50   [0x2c poison]
    // 0x50 addi x3,x0,3      0x54 jal x6,-0x24 ->0x30   [0x58 poison]
    // 0x30 addi x7,x0,7      0x34 jalr x8,0(x9) ->0x60  [0x38 poison]
    // 0x60 addi x10,x0,10    0x64 jal x0,0 (self-loop terminator)
    fill_nops();
    imem[0]  = 32'h0600_0493;  // addi x9,x0,0x60
    imem[1]  = 32'h0010_0093;  // addi x1,x0,1
    imem[2]  = 32'h01c0_02ef;  // jal  x5,+0x1C -> 0x24
    imem[3]  = 32'h0630_0093;  // POISON addi x1,x0,99
    imem[4]  = 32'h0630_0113;  // POISON addi x2,x0,99
    imem[9]  = 32'h0020_0113;  // addi x2,x0,2
    imem[10] = 32'h0280_026f;  // jal  x4,+0x28 -> 0x50
    imem[11] = 32'h0620_0113;  // POISON addi x2,x0,98
    imem[20] = 32'h0030_0193;  // addi x3,x0,3
    imem[21] = 32'hfddf_f36f;  // jal  x6,-0x24 -> 0x30
    imem[22] = 32'h0610_0193;  // POISON addi x3,x0,97
    imem[12] = 32'h0070_0393;  // addi x7,x0,7
    imem[13] = 32'h0004_8467;  // jalr x8,0(x9) -> 0x60
    imem[14] = 32'h0600_0393;  // POISON addi x7,x0,96
    imem[24] = 32'h00a0_0513;  // addi x10,x0,10
    imem[25] = 32'h0000_006f;  // jal  x0,0 (self-loop)
    pulse_reset();
    mon_clear();

    // 10 real instructions, then at least one self-loop commit.
    wait_commit_order(64'd11, 2000);
    repeat (2) @(posedge clk);
    mon_on = 1'b0;

    // entered pins
    check_bit("P1: deser fired at 0x08", deser_at_08, 1'b1);
    check_bit("P1: deser fired at 0x28", deser_at_28, 1'b1);
    check_bit("P1: deser fired at 0x54 (backward)", deser_at_54, 1'b1);
    check_bit("P1: 0x08 target was first dispatch after refill", next_ok_08, 1'b1);
    check_bit("P1: 0x28 target was first dispatch after refill", next_ok_28, 1'b1);
    check_bit("P1: 0x54 target was first dispatch after refill", next_ok_54, 1'b1);
    check_bit("P1: no stale queued word dispatched after JAL", wrong_dispatch_after_deser, 1'b0);
    check_word("P1: exactly one serialized alloc (the JALR)",
               word_t'(n_jump_alloc), 32'd1);
    check_word("P1: exactly one serialized resolve (the JALR)",
               word_t'(n_jump_resolve), 32'd1);
    check_bit("P1: no recovery in a jump-only program", n_recover == 0, 1'b1);
    check_bit("P1: no trap", n_trap == 0, 1'b1);

    // architectural consequence (poisons would overwrite these)
    check_arch("P1 x9  jalr base",   9,  32'h0000_0060);
    check_arch("P1 x1",              1,  32'h0000_0001);
    check_arch("P1 x5  link of 0x08", 5, 32'h0000_000c);
    check_arch("P1 x2",              2,  32'h0000_0002);
    check_arch("P1 x4  link of 0x28", 4, 32'h0000_002c);
    check_arch("P1 x3",              3,  32'h0000_0003);
    check_arch("P1 x6  link of 0x54", 6, 32'h0000_0058);
    check_arch("P1 x7",              7,  32'h0000_0007);
    check_arch("P1 x8  link of jalr", 8, 32'h0000_0038);
    check_arch("P1 x10 jalr target",  10, 32'h0000_000a);

    // ================= Scenario 2: wrong-path JAL under recovery ============
    // 0x00 addi x28,x0,7    0x04 rem x1,x0,x28 (32-cycle producer, x1=0)
    // 0x08 beq x1,x0,+0x1C -> 0x24 (TAKEN; predicted not-taken)
    // 0x0c jal x2,+0x14 -> 0x20 (WRONG PATH; deser redirect fires)
    // 0x10 addi x3,x0,99 (never)   0x20 addi x4,x0,88 (wrong-path target)
    // 0x24 addi x5,x0,55 (correct path)   0x28 jal x0,0
    fill_nops();
    imem[0]  = 32'h0070_0e13;  // addi x28,x0,7
    imem[1]  = 32'h03c0_60b3;  // rem  x1,x0,x28
    imem[2]  = 32'h0000_8e63;  // beq  x1,x0,+0x1C -> 0x24
    imem[3]  = 32'h0140_016f;  // jal  x2,+0x14 -> 0x20 (wrong path)
    imem[4]  = 32'h0630_0193;  // POISON addi x3,x0,99
    imem[8]  = 32'h0580_0213;  // addi x4,x0,88 (wrong-path target)
    imem[9]  = 32'h0370_0293;  // addi x5,x0,55 (correct path)
    imem[10] = 32'h0000_006f;  // jal x0,0 (self-loop)
    pulse_reset();
    mon_clear();

    // 5 correct-path commits (addi x28, rem, beq, addi x5, >=1 self-loop jal)
    wait_commit_order(64'd5, 2000);
    repeat (2) @(posedge clk);
    mon_on = 1'b0;

    check_bit("P2: wrong-path JAL deser fired at 0x0c", deser_at_0c, 1'b1);
    check_bit("P2: recovery broadcast entered", n_recover > 0, 1'b1);
    check_bit("P2: no serialized alloc", n_jump_alloc == 0, 1'b1);
    check_bit("P2: no trap", n_trap == 0, 1'b1);
    check_arch("P2 x28 divisor", 28, 32'h0000_0007);
    check_arch("P2 x1 rem result", 1, 32'h0000_0000);
    check_arch("P2 x5 correct path committed", 5, 32'h0000_0037);
    check_arch("P2 x2 wrong-path link never committed", 2, 32'h0000_0000);
    check_arch("P2 x4 wrong-path target rolled back", 4, 32'h0000_0000);
    check_arch("P2 x3 shadow poison never committed", 3, 32'h0000_0000);

    // ================= Scenario 3: misaligned JAL serialized fallback =======
    // 0x00 addi x30,x0,0x80   0x04 csrrw x0,mtvec,x30
    // 0x08 jal x11,+2 (target 0x0a: MISALIGNED -> serialized -> trap)
    // 0x0c addi x12,x0,77 (never commits)
    // 0x80 csrrs x13,mcause,x0   0x84 csrrs x14,mepc,x0   0x88 jal x0,0
    fill_nops();
    imem[0]  = 32'h0800_0f13;  // addi x30,x0,0x80
    imem[1]  = 32'h305f_1073;  // csrrw x0,mtvec,x30
    imem[2]  = 32'h0020_05ef;  // jal x11,+2 -> 0x0a (misaligned)
    imem[3]  = 32'h04d0_0613;  // POISON addi x12,x0,77
    imem[32] = 32'h3420_26f3;  // csrrs x13,mcause,x0
    imem[33] = 32'h3410_2773;  // csrrs x14,mepc,x0
    imem[34] = 32'h0000_006f;  // jal x0,0 (self-loop)
    pulse_reset();
    mon_clear();

    // Wait for the trap to land, then the handler to commit.
    wait_i = 0;
    while (`CSRF.mepc_q !== 32'h0000_0008) begin
      @(posedge clk);
      wait_i++;
      if (wait_i > 2000) $fatal(1, "P3: misalign trap never landed (mepc=%08h)",
                                `CSRF.mepc_q);
    end
    wait_commit_order(64'd5, 2000);
    repeat (2) @(posedge clk);
    mon_on = 1'b0;

    // The handler's own aligned jal x0,0 terminator deser-fires legitimately,
    // so pin the exclusion to the misaligned JAL's pc, not the global count.
    check_bit("P3: misaligned JAL was NOT de-serialized", deser_at_08 == 1'b0, 1'b1);
    check_word("P3: serialized fallback engaged (one alloc)",
               word_t'(n_jump_alloc), 32'd1);
    check_bit("P3: trap entered", n_trap > 0, 1'b1);
    check_word("P3: mcause is IADDR_MISALIGN", `CSRF.mcause_q, 32'h0000_0000);
    check_word("P3: mepc is the JAL's pc", `CSRF.mepc_q, 32'h0000_0008);
    check_arch("P3 x13 handler-read mcause", 13, 32'h0000_0000);
    check_arch("P3 x14 handler-read mepc",   14, 32'h0000_0008);
    check_arch("P3 x11 no link write on the trapping JAL", 11, 32'h0000_0000);
    check_arch("P3 x12 shadow poison never committed", 12, 32'h0000_0000);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_jal_deser] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_jal_deser] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
