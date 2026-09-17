// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// tb_rv32i_ss_core_lsu — LSU/LQ directed proof (frontend + core + dmem
// model, program-driven through the REAL decoder).
//
// Proves the load-completion ownership split and early-launch behavior:
//
//  1. MISROUTE: a real `lw` decoded by the golden decode wrapper executes in
//     the LSU and retires the MEMORY WORD — not its effective address, which
//     is exactly what it would retire if fu_class still mapped mem ops to the
//     ALU (decode sets ALU_ADD + IMM for address calc, so the failure is
//     silent). dmem is preloaded with values != their addresses.
//  2. EARLY LAUNCH: a ready load behind an older multi-cycle MUL accesses dmem
//     before becoming the ROB head, but still commits after that older MUL.
//     The loaded architectural value must remain correct.
//
// Also pins the store path: the store writes memory only at commit.
//
// Program (RESET_PC = 0):
//   0x00 addi x1, x0, 0x100    # base
//   0x04 lw   x5, 8(x1)        # -> mem[0x108] = DEADBEEF (misroute: 0x108)
//   0x08 addi x6, x0, 0x77
//   0x0c mul  x7, x6, x6       # multi-cycle; occupies the head for ~33 cycles
//   0x10 lw   x8, 12(x1)       # launches before MUL commits; retires after it
//   0x14 sw   x9, 0(x1)        # writes zero at commit
//   0x18 addi x10, x0, 0x55    # tail marker
//   0x1c jal  x0, 0            # spin

module tb_rv32i_ss_core_lsu;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;

  logic clk;
  logic rst_n;

  // frontend <-> core decoded packet
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

  // commit channel
  logic [1:0]          commit_fire;
  commit_order_t commit_order;
  word_t [1:0]         commit_pc, commit_inst, commit_wdata;
  arch_reg_t [1:0]     commit_rd;
  logic [1:0]          commit_rd_wen;

  // redirect
  logic          redirect_valid;
  word_t         redirect_target;

  // dmem interface (handshake shape; TB ties ready/rvalid high)
  logic        dmem_valid, dmem_we;
  logic [3:0]  dmem_be;
  word_t       dmem_addr, dmem_wdata, dmem_rdata;

  // instruction memory
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
    .dmem_ready(1'b1), .dmem_rvalid(1'b1),      // tied for bring-up
    .dmem_rdata(dmem_rdata)
  );

  // data memory model (shared TB infra; raw words + byte-enable writes)
  ooo_dmem_model #(.MEM_WORDS(256), .MEM_MSB(9)) u_dmem (
    .clk(clk), .rst_n(rst_n),
    .addr(dmem_addr), .rdata(dmem_rdata),
    .we(dmem_we), .be(dmem_be), .wdata(dmem_wdata),
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

  // ---- observation state ----
  int          cycle_cnt;
  int          dmem_read_pulses;
  int          nonhead_load_launches;
  int          dmem_we_pulses;
  int          mul_commit_cycle;
  int          load2_fire_cycle;
  int          load2_commit_cycle;
  logic        seen_sw_commit;
  logic        seen_tail_commit;
  word_t       got_x1, got_x5, got_x6, got_x7, got_x8, got_x10;
  // per-slot capture scratch (Icarus rejects a part-select chained
  // onto a variable array-element select, so copy the word first)
  integer      cap_slot;
  // program order between two retirements in ONE cycle is carried by
  // the SLOT index, not the cycle number, so the ordering oracle needs both.
  integer      mul_commit_slot;
  integer      load2_commit_slot;
  word_t       cap_pc;
  word_t       cap_wdata;

  always @(posedge clk) begin
    if (rst_n) begin
      cycle_cnt <= cycle_cnt + 1;

      // The LQ, not the live IQ issue entry, owns the memory request. Count
      // launches that intentionally occur before the load reaches ROB head.
      if (u_core.u_lsq.lq_mem_req_fire) begin
        dmem_read_pulses <= dmem_read_pulses + 1;
        if (u_core.u_lsq.lq_select_entry.rob_idx !== u_core.rob_head_idx)
          nonhead_load_launches <= nonhead_load_launches + 1;
        if (dmem_read_pulses == 1)           // this pulse is the SECOND load
          load2_fire_cycle <= cycle_cnt;
      end
      if (dmem_we) dmem_we_pulses <= dmem_we_pulses + 1;

      // per-position capture keeps the oracle honest when one cycle
      // retires two records.
      for (cap_slot = 0; cap_slot < 2; cap_slot++) begin
        if (commit_fire[cap_slot]) begin
          cap_pc    = commit_pc[cap_slot];
          cap_wdata = commit_wdata[cap_slot];
          case (cap_pc)
            32'h00: got_x1  <= cap_wdata;
            32'h04: got_x5  <= cap_wdata;
            32'h08: got_x6  <= cap_wdata;
            32'h0c: begin got_x7 <= cap_wdata; mul_commit_cycle <= cycle_cnt;
                          mul_commit_slot <= cap_slot; end
            32'h10: begin got_x8 <= cap_wdata; load2_commit_cycle <= cycle_cnt;
                          load2_commit_slot <= cap_slot; end
            32'h14: seen_sw_commit <= 1'b1;
            32'h18: begin got_x10 <= cap_wdata; seen_tail_commit <= 1'b1; end
            default: ;                          // jal spin
          endcase
        end
      end
    end
  end

  task automatic reset_dut();
    rst_n = 1'b0;
    repeat (4) @(posedge clk);
    rst_n = 1'b1;
  endtask

  integer i;
  initial begin
    checks = 0; fails = 0;
    cycle_cnt = 0; dmem_read_pulses = 0; nonhead_load_launches = 0;
    dmem_we_pulses = 0; mul_commit_cycle = -1; load2_fire_cycle = -1;
    load2_commit_cycle = -1;
    seen_sw_commit = 1'b0; seen_tail_commit = 1'b0;

    for (i = 0; i < 256; i++) imem[i] = 32'h00000013;   // NOP fill
    imem[0] = 32'h10000093;   // addi x1, x0, 0x100
    imem[1] = 32'h0080A283;   // lw   x5, 8(x1)
    imem[2] = 32'h07700313;   // addi x6, x0, 0x77
    imem[3] = 32'h026303B3;   // mul  x7, x6, x6
    imem[4] = 32'h00C0A403;   // lw   x8, 12(x1)
    imem[5] = 32'h0090A023;   // sw   x9, 0(x1)
    imem[6] = 32'h05500513;   // addi x10, x0, 0x55
    imem[7] = 32'h0000006F;   // jal  x0, 0 (spin)

    #1;  // let the dmem model's zero-fill initial run first
    u_dmem.mem[32'h108 >> 2] = 32'hDEADBEEF;
    u_dmem.mem[32'h10C >> 2] = 32'hCAFE0123;
    u_dmem.mem[32'h100 >> 2] = 32'h00000000;  // sw target: x9 is zero in this smoke

    reset_dut();

    // run until the tail marker commits (bounded)
    for (i = 0; i < 2000 && !seen_tail_commit; i++) @(posedge clk);
    repeat (4) @(posedge clk);

    chk(seen_tail_commit,                "liveness: tail addi committed", {31'b0, seen_tail_commit}, 32'h1);
    chk(got_x1  === 32'h0000_0100,       "x1 base",                got_x1,  32'h0000_0100);
    chk(got_x5  === 32'hDEAD_BEEF,       "MISROUTE: lw x5 = mem, not addr", got_x5, 32'hDEAD_BEEF);
    chk(got_x6  === 32'h0000_0077,       "x6 independent op",      got_x6,  32'h0000_0077);
    chk(got_x7  === 32'h0000_3751,       "x7 mul result",          got_x7,  32'h0000_3751);
    chk(got_x8  === 32'hCAFE_0123,       "lw x8 behind mul",       got_x8,  32'hCAFE_0123);
    chk(seen_sw_commit,                  "sw committed",           {31'b0, seen_sw_commit}, 32'h1);
    chk(got_x10 === 32'h0000_0055,       "x10 tail value",         got_x10, 32'h0000_0055);
    chk(nonhead_load_launches > 0,       "EARLY-LAUNCH: load accessed dmem before ROB head",
                                         nonhead_load_launches[31:0], 32'h1);
    chk(dmem_read_pulses == 2,           "exactly two dmem reads (loads only)", dmem_read_pulses[31:0], 32'h2);
    chk((load2_fire_cycle > 0) && (load2_fire_cycle < mul_commit_cycle),
                                         "EARLY-LAUNCH: lw x8 fired before older mul committed",
                                         load2_fire_cycle[31:0], mul_commit_cycle[31:0]);
    // the pre-dual proxy was a strict CYCLE inequality, valid only
    // while one instruction retired per cycle. Under dual commit the mul and
    // this load are ROB-adjacent and retire TOGETHER (mul slot 0, load slot 1),
    // which explicitly permits -- memory is legal in either slot. Program
    // order is then carried by the slot index, so assert that instead.
    chk(((load2_commit_cycle > mul_commit_cycle) ||
         ((load2_commit_cycle == mul_commit_cycle) &&
          (load2_commit_slot > mul_commit_slot))) && (mul_commit_cycle > 0),
                                         "IN-ORDER COMMIT: lw x8 retired after older mul",
                                         load2_commit_cycle[31:0], mul_commit_cycle[31:0]);
    chk(dmem_we_pulses == 1,             "one commit-time store write", dmem_we_pulses[31:0], 32'h1);
    chk(u_dmem.mem[32'h100 >> 2] === 32'h0, "sw x9 writes zero in this smoke", u_dmem.mem[32'h100 >> 2], 32'h0);

    if (fails == 0)
      $display("tb_rv32i_ss_core_lsu PASS checks=%0d", checks);
    else
      $display("tb_rv32i_ss_core_lsu FAIL fails=%0d checks=%0d", fails, checks);
    $finish;
  end

  // watchdog
  initial begin
    #300000;
    $display("tb_rv32i_ss_core_lsu FAIL watchdog (no tail commit)");
    $finish;
  end

endmodule
