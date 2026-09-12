`timescale 1ns/1ps

// tb_rv32i_ss_core_m4_dside — the contract at system level: the SAME
// consequential program as tb_rv32i_ss_core_store, driven through
// rv32i_ss_dmem_scratchpad at READY_STALL=2 with RESP_LATENCY=1 — every
// property that held under always-ready memory must hold under withheld
// ready and delayed responses, composed. (The two knobs are exercised
// separately in the unit batteries; this leg is their composition through
// the full machine.) Entry proofs pin the commit-hold, the held-load
// window and the outstanding-response window, and the write-once law rides
// the acceptance strobe.
//
// Proves the invariant — a store's memory write happens at ACCEPTANCE,
// retirement waits for it, and wrong-path/trap-shadow stores never reach
// memory — consequential three ways:
//
//  1. RAW through memory: sw then lw the same address; the load receives a
//     full-cover forward from the youngest older SQ entry before store commit.
//  2. WRONG-PATH store: a store shadowed by a mispredicted branch (branch
//     waits ~33 cycles on a mul operand, so the younger store ISSUES, writes
//     its payload, and COMPLETES before recovery kills it) must never write.
//     Provably-entered: store issue count > dmem_we count.
//  3. Store-behind-TRAP: a store dispatched after an ecall completes, then the
//     taken trap flushes it — must never write.
//
// Plus SB/SH byte-lane placement + byte-enable merge read back through real loads,
// and the port-level invariant monitor: dmem_we only within a committing group.
//
// Program (RESET_PC = 0):
//   0x00 addi x1, x0, 0x100
//   0x04 addi x9, x0, -1          # x9 = FFFFFFFF (all bytes consequential)
//   0x08 sw   x9, 0(x1)           # mem[100] = FFFFFFFF
//   0x0c lw   x5, 0(x1)           # RAW: expect FFFFFFFF
//   0x10 addi x6, x0, 0x77
//   0x14 mul  x20, x6, x6         # slow producer for the branch
//   0x18 beq  x20, x20, 0x80      # TAKEN when mul resolves -> mispredict
//   0x1c sw   x9, 4(x1)           # WRONG PATH: mem[104] must stay 0
//   ...  nops
//   0x80 addi x22, x0, 0x4D2
//   0x84 sb   x9, 5(x1)           # mem[104] lane1  -> 0000FF00
//   0x88 sh   x22, 6(x1)          # mem[104] high   -> 04D2FF00
//   0x8c lw   x7, 4(x1)           # expect 04D2FF00 (also proves #2!)
//   0x90 sb   x9, 8(x1)           # mem[108] lane0  -> 000000FF
//   0x94 sb   x22, 11(x1)         # mem[108] lane3  -> D20000FF
//   0x98 lw   x8, 8(x1)           # expect D20000FF
//   0x9c addi x23, x0, 0xC0
//   0xa0 csrrw x0, mtvec, x23     # handler = 0xC0
//   0xa4 ecall                    # trap -> flush -> redirect 0xC0
//   0xa8 sw   x9, 12(x1)          # TRAP SHADOW: mem[10C] must stay 0
//   ...  nops
//   0xc0 lw   x11, 12(x1)         # handler: expect 0
//   0xc4 addi x18, x0, 0x55       # tail marker
//   0xc8 jal  x0, 0               # spin

module tb_rv32i_ss_core_m4_dside;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;

  logic clk;
  logic rst_n;

  logic          dec_valid, dec_ready;
  logic [1:0] dec_slot_valid;
  word_t [1:0] dec_pc, dec_instr;
  arch_reg_t [1:0] dec_rs1, dec_rs2, dec_rd;
  logic [1:0] dec_rd_we, dec_needs_checkpoint;
  ooo_op_class_e [1:0] dec_op_class;
  ooo_fu_class_e [1:0] dec_fu_class;
  muldiv_op_e [1:0]    dec_muldiv_op;
  alu_op_e [1:0]       dec_alu_op;
  br_type_e [1:0]      dec_branch_op;
  ooo_src_sel_e [1:0] dec_src1_sel, dec_src2_sel;
  word_t [1:0]         dec_imm;
  decoded_trap_t dec_trap;
  csr_op_e       dec_csr_op;
  csr_addr_t     dec_csr_addr;
  csr_zimm_t     dec_csr_zimm;
  logic [1:0] dec_is_load, dec_is_store;
  mem_size_e [1:0]     dec_mem_size;
  logic [1:0]          dec_mem_unsigned;

  logic [1:0]          commit_fire;
  commit_order_t commit_order;
  word_t [1:0]         commit_pc, commit_inst, commit_wdata;
  arch_reg_t [1:0]     commit_rd;
  logic [1:0]          commit_rd_wen;

  logic          redirect_valid;
  word_t         redirect_target;

  logic        dmem_valid, dmem_we;
  logic [3:0]  dmem_be;
  word_t       dmem_addr, dmem_wdata, dmem_rdata;
  logic        dmem_ready, dmem_rvalid;
  logic        sp_en, sp_we;
  logic [3:0]  sp_be;
  word_t       sp_addr, sp_wdata, sp_rdata;

  word_t       imem_addr;
  word_t [1:0]       imem_rdata;
  logic imem_req_valid, imem_req_ready;
  word_t imem_req_addr;
  logic imem_resp_valid, imem_resp_ready;
  word_t [1:0] imem_resp_data;
  logic [31:0] imem [256];
  assign imem_rdata = !$isunknown(imem_addr[9:3])
      ? {imem[{imem_addr[9:3], 1'b1}], imem[{imem_addr[9:3], 1'b0}]}
      : {32'h0000_0013, 32'h0000_0013};

  rv32i_ss_imem_zero_latency_adapter u_imem_adapter (
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr), .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready), .imem_resp_data(imem_resp_data),
    .line_addr(imem_addr), .line_data(imem_rdata)
  );

  rv32i_ss_frontend #(.RESET_PC(32'h0)) u_fe (
    .clk(clk), .rst_n(rst_n),
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr), .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready), .imem_resp_data(imem_resp_data),
    .bp_update_valid(1'b0),  // tie-off: predictor never trains (inert)
    .bp_update_pc('0), .bp_update_taken(1'b0), .bp_update_target('0),
    .redirect_valid(redirect_valid), .redirect_target(redirect_target),
    .decoded_valid(dec_valid), .decoded_slot_valid(dec_slot_valid), .decoded_ready(dec_ready),
    .decoded_pc(dec_pc), .decoded_instr(dec_instr),
    .decoded_rs1(dec_rs1), .decoded_rs2(dec_rs2), .decoded_rd(dec_rd),
    .decoded_rd_we(dec_rd_we), .decoded_needs_checkpoint(dec_needs_checkpoint),
    .decoded_op_class(dec_op_class), .decoded_fu_class(dec_fu_class),
    .decoded_muldiv_op(dec_muldiv_op), .decoded_alu_op(dec_alu_op),
    .decoded_branch_op(dec_branch_op),
    .decoded_src1_sel(dec_src1_sel), .decoded_src2_sel(dec_src2_sel),
    .decoded_imm(dec_imm),
    .decoded_trap(dec_trap),
    .decoded_csr_op(dec_csr_op),
    .decoded_csr_addr(dec_csr_addr),
    .decoded_csr_zimm(dec_csr_zimm),
    .decoded_is_load(dec_is_load),
    .decoded_is_store(dec_is_store),
    .decoded_mem_size(dec_mem_size),
    .decoded_mem_unsigned(dec_mem_unsigned)
  );

  rv32i_ss_core u_core (
    .clk(clk), .rst_n(rst_n),
    .decoded_valid(dec_valid), .decoded_slot_valid(dec_slot_valid), .decoded_ready(dec_ready),
    .decoded_pc(dec_pc), .decoded_instr(dec_instr),
    .decoded_rs1(dec_rs1), .decoded_rs2(dec_rs2), .decoded_rd(dec_rd),
    .decoded_rd_we(dec_rd_we), .decoded_needs_checkpoint(dec_needs_checkpoint),
    .decoded_op_class(dec_op_class), .decoded_fu_class(dec_fu_class),
    .decoded_muldiv_op(dec_muldiv_op), .decoded_alu_op(dec_alu_op),
    .decoded_branch_op(dec_branch_op),
    .decoded_src1_sel(dec_src1_sel), .decoded_src2_sel(dec_src2_sel),
    .decoded_imm(dec_imm),
    .decoded_trap(dec_trap),
    .decoded_csr_op(dec_csr_op),
    .decoded_csr_addr(dec_csr_addr),
    .decoded_csr_zimm(dec_csr_zimm),
    .decoded_is_load(dec_is_load),
    .decoded_is_store(dec_is_store),
    .decoded_mem_size(dec_mem_size),
    .decoded_mem_unsigned(dec_mem_unsigned),
    .decoded_pred_taken('0),  // tie-off: static not-taken prediction
    .decoded_pred_target('0),
    .redirect_valid(redirect_valid), .redirect_target(redirect_target),
    .commit_fire(commit_fire), .commit_order(commit_order),
    .commit_pc(commit_pc), .commit_inst(commit_inst),
    .commit_rd(commit_rd), .commit_rd_wen(commit_rd_wen),
    .commit_wdata(commit_wdata),
    .dmem_valid(dmem_valid), .dmem_we(dmem_we), .dmem_be(dmem_be),
    .dmem_addr(dmem_addr), .dmem_wdata(dmem_wdata),
    .dmem_ready(dmem_ready), .dmem_rvalid(dmem_rvalid),
    .dmem_rdata(dmem_rdata)
  );

  // environment: withheld ready + delayed response, composed. The
  // backing model keys off the ACCEPTANCE strobe, never dmem_we.
  rv32i_ss_dmem_scratchpad #(.READY_STALL(2), .RESP_LATENCY(1)) u_dmem_env (
    .clk(clk), .rst_n(rst_n),
    .dmem_valid(dmem_valid), .dmem_we(dmem_we), .dmem_be(dmem_be),
    .dmem_addr(dmem_addr), .dmem_wdata(dmem_wdata),
    .dmem_ready(dmem_ready), .dmem_rvalid(dmem_rvalid),
    .dmem_rdata(dmem_rdata),
    .mem_en(sp_en), .mem_we(sp_we), .mem_be(sp_be),
    .mem_addr(sp_addr), .mem_wdata(sp_wdata), .mem_rdata(sp_rdata)
  );

  ooo_dmem_model #(.MEM_WORDS(256), .MEM_MSB(9)) u_dmem (
    .clk(clk), .rst_n(rst_n),
    .addr(sp_addr), .rdata(sp_rdata),
    .we(sp_en && sp_we), .be(sp_be), .wdata(sp_wdata),
    .tohost_addr(32'hFFFF_FFFC), .tohost_full_addr(32'hFFFF_FFFC),
    .tohost_we(), .tohost_val()
  );

  initial clk = 1'b0;
  always #5 clk = ~clk;

  int checks, fails;
  task automatic chk(input logic cond, input string name,
                     input logic [31:0] got, input logic [31:0] exp);
    checks++;
    if (!cond) begin
      fails++;
      $display("  FAIL [%0d] %s got=%08h exp=%08h", checks, name, got, exp);
    end
  endtask

  // ---- observation ----
  word_t got [64];
  logic  seen_tail;
  int    store_issues;        // lsu_issue_fire && is_store (incl. ALL wrong-path:
                              // the shadow stores AND any prefetched battery
                              // stores — prefetch depth is ROB-bounded, so no
                              // exact count is asserted, only the flags below)
  logic  branch_shadow_issued;  // the 0x1c store ISSUED before being killed
  logic  trap_shadow_issued;    // the 0xa8 store ISSUED before being flushed
  int    we_pulses;           // actual memory writes
  // per-slot capture scratch (Icarus rejects a part-select chained
  // onto a variable array-element select, so copy the word first)
  integer cap_slot;
  word_t  cap_pc;
  word_t  cap_wdata;
  word_t  pair_inst0, pair_inst1;
  logic   pair_load0, pair_load1, pair_store0, pair_store1;
  int     pair_alu_store, pair_store_alu;
  int     pair_load_store, pair_store_load, pair_load_load;
  int     stopped_store_store;
  int    we_outside_commit;   // accepted write, no commit, no recovery gate
  int    n_def_accept;        // accepted writes without same-cycle commit
  int    n_commit_hold;       // store commit-hold cycles (want vs low ready)
  int    n_load_hold;         // held-load presentation cycles
  int    n_out_wait;          // outstanding-response cycles
  always @(posedge clk) begin
    if (rst_n) begin
      // capture every fired commit position in program order.
      for (cap_slot = 0; cap_slot < 2; cap_slot++) begin
        if (commit_fire[cap_slot]) begin
          cap_pc    = commit_pc[cap_slot];
          cap_wdata = commit_wdata[cap_slot];
          if (cap_pc[31:8] == '0) got[cap_pc[7:2]] <= cap_wdata;
          if (cap_pc == 32'hc4)   seen_tail <= 1'b1;
        end
      end
      // Store-issue event in AGU-unit space: store_agen_fire is the routed
      // store's own fire, and agen_issue_entry carries ITS pc regardless of
      // which grant position bound the AGU (position 1 is reachable at 3b).
      if (u_core.store_agen_fire) begin
        store_issues <= store_issues + 1;
        if (u_core.agen_issue_entry.pc == 32'h1c) branch_shadow_issued <= 1'b1;
        if (u_core.agen_issue_entry.pc == 32'ha8) trap_shadow_issued   <= 1'b1;
      end
      // dmem_we PRESENTS through stall windows; only the acceptance
      // is a write. An accepted write without a same-cycle commit is legal
      // ONLY on a recovery-gated edge (3.3a accept-defer) — anything else
      // is the invariant break the always-ready form used to catch.
      if (dmem_we && dmem_ready) begin
        we_pulses <= we_pulses + 1;
        if (!(|commit_fire)) begin
          n_def_accept <= n_def_accept + 1;
          if (!u_core.branch_recover_req)
            we_outside_commit <= we_outside_commit + 1;
        end
      end
      if ((|u_core.store_commit_want) && !dmem_ready)
        n_commit_hold <= n_commit_hold + 1;
      if (u_core.u_lsq.dreq_held_q)
        n_load_hold <= n_load_hold + 1;
      if ((u_core.u_lsq.lq_out_count_q != 2'd0))
        n_out_wait <= n_out_wait + 1;

      // commit-group census. Decode from the independent architectural
      // instruction records, not the core's class bits, so these counters are
      // an external oracle for the legal memory orientations.
      pair_inst0  = commit_inst[0];
      pair_inst1  = commit_inst[1];
      pair_load0  = (pair_inst0[6:0] == 7'b0000011);
      pair_load1  = (pair_inst1[6:0] == 7'b0000011);
      pair_store0 = (pair_inst0[6:0] == 7'b0100011);
      pair_store1 = (pair_inst1[6:0] == 7'b0100011);
      if (commit_fire == 2'b11) begin
        if (!pair_load0 && !pair_store0 && pair_store1)
          pair_alu_store = pair_alu_store + 1;
        if (pair_store0 && !pair_load1 && !pair_store1)
          pair_store_alu = pair_store_alu + 1;
        if (pair_load0 && pair_store1)
          pair_load_store = pair_load_store + 1;
        if (pair_store0 && pair_load1)
          pair_store_load = pair_store_load + 1;
        if (pair_load0 && pair_load1)
          pair_load_load = pair_load_load + 1;
      end
      if ((commit_fire == 2'b01) &&
          u_core.rob_commit_valid[1] &&
          u_core.rob_commit_is_store[0] && u_core.rob_commit_is_store[1])
        stopped_store_store = stopped_store_store + 1;
    end
  end

  task automatic reset_dut();
    rst_n = 1'b0;
    repeat (4) @(posedge clk);
    rst_n = 1'b1;
  endtask

  integer i;
  initial begin
    checks = 0; fails = 0; seen_tail = 1'b0;
    store_issues = 0; we_pulses = 0; we_outside_commit = 0;
    n_def_accept = 0; n_commit_hold = 0; n_load_hold = 0; n_out_wait = 0;
    pair_alu_store = 0; pair_store_alu = 0;
    pair_load_store = 0; pair_store_load = 0; pair_load_load = 0;
    stopped_store_store = 0;
    branch_shadow_issued = 1'b0; trap_shadow_issued = 1'b0;
    for (i = 0; i < 64; i++) got[i] = '0;

    for (i = 0; i < 256; i++) imem[i] = 32'h00000013;  // NOP fill
    imem[ 0] = 32'h10000093;  // addi x1, x0, 0x100
    imem[ 1] = 32'hFFF00493;  // addi x9, x0, -1
    imem[ 2] = 32'h0090A023;  // sw   x9, 0(x1)
    imem[ 3] = 32'h0000A283;  // lw   x5, 0(x1)
    imem[ 4] = 32'h07700313;  // addi x6, x0, 0x77
    imem[ 5] = 32'h02630A33;  // mul  x20, x6, x6
    imem[ 6] = 32'h074A0463;  // beq  x20, x20, +0x68 (-> 0x80; TAKEN = mispredict)
    imem[ 7] = 32'h0090A223;  // sw   x9, 4(x1)   WRONG PATH
    imem[32] = 32'h4D200B13;  // addi x22, x0, 0x4D2
    imem[33] = 32'h009082A3;  // sb   x9, 5(x1)
    imem[34] = 32'h01609323;  // sh   x22, 6(x1)
    imem[35] = 32'h0040A383;  // lw   x7, 4(x1)
    imem[36] = 32'h00908423;  // sb   x9, 8(x1)
    imem[37] = 32'h016085A3;  // sb   x22, 11(x1)
    imem[38] = 32'h0080A403;  // lw   x8, 8(x1)
    imem[39] = 32'h0C000B93;  // addi x23, x0, 0xC0
    imem[40] = 32'h305B9073;  // csrrw x0, mtvec, x23
    imem[41] = 32'h00000073;  // ecall
    imem[42] = 32'h0090A623;  // sw   x9, 12(x1)  TRAP SHADOW
    imem[48] = 32'h00C0A583;  // 0xc0: lw x11, 12(x1)   (handler)
    imem[49] = 32'h05500913;  // 0xc4: addi x18, x0, 0x55
    imem[50] = 32'h0000006F;  // 0xc8: jal x0, 0

    reset_dut();
    for (i = 0; i < 9000 && !seen_tail; i++) @(posedge clk);
    repeat (4) @(posedge clk);

    chk(seen_tail,                        "liveness: handler tail committed", {31'b0, seen_tail}, 32'h1);
    chk(got['h0c>>2] === 32'hFFFF_FFFF,   "RAW through memory: lw after sw",  got['h0c>>2], 32'hFFFF_FFFF);
    chk(got['h8c>>2] === 32'h04D2_FF00,   "sb lane1 + sh high merged (and wrong-path sw suppressed)", got['h8c>>2], 32'h04D2_FF00);
    chk(got['h98>>2] === 32'hD200_00FF,   "sb lane0 + sb lane3 merged",       got['h98>>2], 32'hD200_00FF);
    chk(got['hc0>>2] === 32'h0000_0000,   "trap-shadow sw suppressed (handler lw reads 0)", got['hc0>>2], 32'h0);
    chk(got['hc4>>2] === 32'h0000_0055,   "tail value",                       got['hc4>>2], 32'h55);
    chk(u_dmem.mem[32'h100>>2] === 32'hFFFF_FFFF, "mem[100] final", u_dmem.mem[32'h100>>2], 32'hFFFF_FFFF);
    chk(u_dmem.mem[32'h104>>2] === 32'h04D2_FF00, "mem[104] final", u_dmem.mem[32'h104>>2], 32'h04D2_FF00);
    chk(u_dmem.mem[32'h108>>2] === 32'hD200_00FF, "mem[108] final", u_dmem.mem[32'h108>>2], 32'hD200_00FF);
    chk(u_dmem.mem[32'h10C>>2] === 32'h0000_0000, "mem[10C] never written",   u_dmem.mem[32'h10C>>2], 32'h0);
    chk(branch_shadow_issued,             "PROVABLY-ENTERED: branch-shadow store issued before kill", {31'b0, branch_shadow_issued}, 32'h1);
    chk(trap_shadow_issued,               "PROVABLY-ENTERED: trap-shadow store issued before flush", {31'b0, trap_shadow_issued}, 32'h1);
    chk(store_issues > we_pulses,         "wrong-path store issues exceeded memory writes", store_issues[31:0], we_pulses[31:0]);
    chk(we_pulses == 5,                   "exactly 5 memory writes (committed stores only)", we_pulses[31:0], 32'd5);
    chk(we_outside_commit == 0,           "INVARIANT: no-commit acceptance only under recovery gating", we_outside_commit[31:0], 32'h0);
    chk(n_commit_hold >= 3,               "M4 entered: store commit-hold cycles", n_commit_hold[31:0], 32'd3);
    chk(n_load_hold >= 1,                 "M4 entered: held-load presentation", n_load_hold[31:0], 32'd1);
    chk(n_out_wait >= 3,                  "M4 entered: outstanding-response cycles", n_out_wait[31:0], 32'd3);
    chk(pair_alu_store > 0,               "S4 entered fetched ALU+store dual-commit group", pair_alu_store[31:0], 32'd1);
    chk(pair_load_store > 0,              "S4 entered fetched load+store dual-commit group", pair_load_store[31:0], 32'd1);
    $display("S4_MEM_GROUPS alu+store=%0d store+alu=%0d load+store=%0d store+load=%0d load+load=%0d store+store-stop=%0d",
             pair_alu_store, pair_store_alu, pair_load_store, pair_store_load,
             pair_load_load, stopped_store_store);

    if (fails == 0)
      $display("tb_rv32i_ss_core_m4_dside PASS checks=%0d", checks);
    else
      $display("tb_rv32i_ss_core_m4_dside FAIL fails=%0d checks=%0d", fails, checks);
    $finish;
  end

  initial begin
    #900000;
    $display("tb_rv32i_ss_core_m4_dside FAIL watchdog");
    $finish;
  end

endmodule
