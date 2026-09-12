`timescale 1ns/1ps

// tb_rv32i_ss_core_misalign — misalign battery (frontend + core
// + dmem model, program-driven; traps are TAKEN through a real handler).
//
// Six execute-detected misalign traps ride the trap machinery end-to-end:
//   lw  @0x202 -> cause 4, mtval 0x202     lh @0x201 -> cause 4, mtval 0x201
//   sw  @0x206 -> cause 6, mtval 0x206     sh @0x203 -> cause 6, mtval 0x203
// beq taken to pc+6 (0x2e) -> cause 0, mtval 0x2e (branch)
// jalr to (x20+0x22)&~1 -> cause 0, mtval target (jalr)
// The handler records mcause/mtval/mepc per trap (captured from the commit
// channel by occurrence), bumps mepc+4, and mrets.
//
// Consequential side checks: faulting stores never write (dmem_we count == 1,
// only the aligned control store; memory image unchanged); faulting loads
// never write rd (x5+x6 still 0); machine stays live through all six traps.
//
// Program (RESET_PC=0; handler at 0x100):
//   0x00 addi  x23, x0, 0x100
//   0x04 csrrw x0, mtvec, x23
//   0x08 addi  x1, x0, 0x200
//   0x0c addi  x9, x0, -1
//   0x10 addi  x22, x0, 0x4D2
//   0x14 sw    x9, 0(x1)          # aligned control store
//   0x18 lw    x5, 2(x1)          # TRAP cause 4
//   0x1c lh    x6, 1(x1)          # TRAP cause 4
//   0x20 sw    x22, 6(x1)         # TRAP cause 6 (must not write)
//   0x24 sh    x22, 3(x1)         # TRAP cause 6 (must not write)
//   0x28 beq   x0, x0, +6         # TRAP cause 0 (taken, target 0x2e)
//   0x2c auipc x20, 0
//   0x30 jalr  x0, x20, 0x22      # TRAP cause 0 (target (0x2c+0x22)&~1=0x4e)
//   0x34 lw    x7, 0(x1)          # FFFFFFFF (mem survived the faulting sh)
//   0x38 lw    x8, 4(x1)          # 0 (faulting sw never wrote)
//   0x3c add   x13, x5, x6        # 0 (faulting loads never wrote rd)
//   0x40 addi  x18, x0, 0x55      # tail
//   0x44 jal   x0, 0
//   0x100 csrr x10, mcause / 0x104 csrr x11, mtval / 0x108 csrr x12, mepc
//   0x10c addi x12, x12, 4 / 0x110 csrw mepc, x12 / 0x114 mret

module tb_rv32i_ss_core_misalign;
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
    .dmem_ready(1'b1), .dmem_rvalid(1'b1),
    .dmem_rdata(dmem_rdata)
  );

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

  // ---- per-trap CSR capture (handler commits, by occurrence) ----
  word_t mcause_log [8];
  word_t mtval_log  [8];
  word_t mepc_log   [8];
  int    n_cause, n_tval, n_epc;
  word_t got [32];
  logic  seen_tail;
  int    we_pulses;
  // per-slot capture scratch (Icarus rejects a part-select chained
  // onto a variable array-element select, so copy the word first)
  integer cap_slot;
  word_t  cap_pc;
  word_t  cap_wdata;

  always @(posedge clk) begin
    if (rst_n) begin
      // per-slot capture. The log counters use BLOCKING assignment so
      // two records retiring in one cycle append twice -- an NBA counter would
      // read the stale index and the second record would overwrite the first.
      // can retire both positions, so append both records in order.
      for (cap_slot = 0; cap_slot < 2; cap_slot++) begin
        if (commit_fire[cap_slot]) begin
          cap_pc    = commit_pc[cap_slot];
          cap_wdata = commit_wdata[cap_slot];
          case (cap_pc)
            32'h100: begin mcause_log[n_cause] = cap_wdata; n_cause = n_cause + 1; end
            32'h104: begin mtval_log[n_tval]   = cap_wdata; n_tval  = n_tval + 1;  end
            32'h108: begin mepc_log[n_epc]     = cap_wdata; n_epc   = n_epc + 1;   end
            default: if (cap_pc[31:7] == '0) got[cap_pc[6:2]] = cap_wdata;
          endcase
          if (cap_pc == 32'h40) seen_tail <= 1'b1;
        end
      end
      if (dmem_we) we_pulses <= we_pulses + 1;
    end
  end

  task automatic reset_dut();
    rst_n = 1'b0;
    repeat (4) @(posedge clk);
    rst_n = 1'b1;
  endtask

  integer i;
  initial begin
    checks = 0; fails = 0; seen_tail = 1'b0; we_pulses = 0;
    n_cause = 0; n_tval = 0; n_epc = 0;
    for (i = 0; i < 32; i++) got[i] = '0;
    for (i = 0; i < 8; i++) begin mcause_log[i]='0; mtval_log[i]='0; mepc_log[i]='0; end

    for (i = 0; i < 256; i++) imem[i] = 32'h00000013;
    imem[ 0] = 32'h10000B93;  // addi  x23, x0, 0x100
    imem[ 1] = 32'h305B9073;  // csrrw x0, mtvec, x23
    imem[ 2] = 32'h20000093;  // addi  x1, x0, 0x200
    imem[ 3] = 32'hFFF00493;  // addi  x9, x0, -1
    imem[ 4] = 32'h4D200B13;  // addi  x22, x0, 0x4D2
    imem[ 5] = 32'h0090A023;  // sw    x9, 0(x1)        aligned control
    imem[ 6] = 32'h0020A283;  // lw    x5, 2(x1)        TRAP 4 @0x202
    imem[ 7] = 32'h00109303;  // lh    x6, 1(x1)        TRAP 4 @0x201
    imem[ 8] = 32'h0160A323;  // sw    x22, 6(x1)       TRAP 6 @0x206
    imem[ 9] = 32'h016091A3;  // sh    x22, 3(x1)       TRAP 6 @0x203
    imem[10] = 32'h00000363;  // beq   x0, x0, +6       TRAP 0 -> 0x2e
    imem[11] = 32'h00000A17;  // auipc x20, 0           (x20 = 0x2c)
    imem[12] = 32'h022A0067;  // jalr  x0, x20, 0x22    TRAP 0 -> 0x4e
    imem[13] = 32'h0000A383;  // lw    x7, 0(x1)
    imem[14] = 32'h0040A403;  // lw    x8, 4(x1)
    imem[15] = 32'h006286B3;  // add   x13, x5, x6
    imem[16] = 32'h05500913;  // addi  x18, x0, 0x55    tail
    imem[17] = 32'h0000006F;  // jal   x0, 0
    // handler @ 0x100 (imem[64])
    imem[64] = 32'h34202573;  // csrr  x10, mcause
    imem[65] = 32'h343025F3;  // csrr  x11, mtval
    imem[66] = 32'h34102673;  // csrr  x12, mepc
    imem[67] = 32'h00460613;  // addi  x12, x12, 4
    imem[68] = 32'h34161073;  // csrw  mepc, x12
    imem[69] = 32'h30200073;  // mret

    reset_dut();
    for (i = 0; i < 8000 && !seen_tail; i++) @(posedge clk);
    repeat (4) @(posedge clk);

    chk(seen_tail,                     "liveness through six taken traps", {31'b0, seen_tail}, 32'h1);
    chk(n_cause == 6,                  "exactly six traps", n_cause[31:0], 32'd6);
    // trap 1: misaligned lw
    chk(mcause_log[0] === 32'd4,       "lw  cause 4",  mcause_log[0], 32'd4);
    chk(mtval_log[0]  === 32'h202,     "lw  mtval",    mtval_log[0],  32'h202);
    chk(mepc_log[0]   === 32'h18,      "lw  mepc = faulting pc", mepc_log[0], 32'h18);
    // trap 2: misaligned lh
    chk(mcause_log[1] === 32'd4,       "lh  cause 4",  mcause_log[1], 32'd4);
    chk(mtval_log[1]  === 32'h201,     "lh  mtval",    mtval_log[1],  32'h201);
    // trap 3: misaligned sw
    chk(mcause_log[2] === 32'd6,       "sw  cause 6",  mcause_log[2], 32'd6);
    chk(mtval_log[2]  === 32'h206,     "sw  mtval",    mtval_log[2],  32'h206);
    // trap 4: misaligned sh
    chk(mcause_log[3] === 32'd6,       "sh  cause 6",  mcause_log[3], 32'd6);
    chk(mtval_log[3]  === 32'h203,     "sh  mtval",    mtval_log[3],  32'h203);
    // trap 5: taken branch to pc+6
    chk(mcause_log[4] === 32'd0,       "beq cause 0",  mcause_log[4], 32'd0);
    chk(mtval_log[4]  === 32'h2e,      "beq mtval",    mtval_log[4],  32'h2e);
    chk(mepc_log[4]   === 32'h28,      "beq mepc = faulting pc", mepc_log[4], 32'h28);
    // trap 6: jalr to (0x2c+0x22)&~1 = 0x4e
    chk(mcause_log[5] === 32'd0,       "jalr cause 0", mcause_log[5], 32'd0);
    chk(mtval_log[5]  === 32'h4e,      "jalr mtval",   mtval_log[5],  32'h4e);
    // consequences
    chk(got['h34>>2] === 32'hFFFF_FFFF, "mem[200] survived faulting sh", got['h34>>2], 32'hFFFF_FFFF);
    chk(got['h38>>2] === 32'h0,         "mem[204] never written by faulting sw", got['h38>>2], 32'h0);
    chk(got['h3c>>2] === 32'h0,         "faulting loads never wrote rd (x5+x6)", got['h3c>>2], 32'h0);
    chk(got['h40>>2] === 32'h55,        "tail value", got['h40>>2], 32'h55);
    chk(we_pulses == 1,                 "exactly one memory write (the aligned control sw)", we_pulses[31:0], 32'd1);
    chk(u_dmem.mem[32'h204>>2] === 32'h0, "mem[204] final image", u_dmem.mem[32'h204>>2], 32'h0);

    if (fails == 0)
      $display("tb_rv32i_ss_core_misalign PASS checks=%0d", checks);
    else
      $display("tb_rv32i_ss_core_misalign FAIL fails=%0d checks=%0d", fails, checks);
    $finish;
  end

  initial begin
    #800000;
    $display("tb_rv32i_ss_core_misalign FAIL watchdog");
    $finish;
  end

endmodule
