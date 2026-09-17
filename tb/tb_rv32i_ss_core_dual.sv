// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// =============================================================================
// tb_rv32i_ss_core_dual — shape-2'b11 atomic dispatch at the core seam.
// This battery injects dual bundles directly at the decoded-packet
// seam and proves the mandatory allocation/dispatch cases end-to-end:
// dual-ALU smoke — one fire, two ROB entries, both architectural
//                        results correct in commit order
// same-bundle WAW — same rd in both slots; younger wins the map
//                        (a later reader sees slot 1's value)
// mem in slot 1 — the sole memory op carries SLOT 1's metadata into
//                        the LQ
// partial-block — IQ has ONE hole, ROB has room: a dual offer holds
//                        with ZERO state change anywhere,
//                        then completes after the queue drains
// trap broadcast — trap_q flush cycle forbids bundle_fire while a
//                        dual offer is presented
// =============================================================================

module tb_rv32i_ss_core_dual;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;

  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  logic        decoded_valid;
  logic [1:0]  decoded_slot_valid;
  logic        decoded_ready;
  word_t         [1:0] decoded_pc, decoded_instr, decoded_imm;
  arch_reg_t     [1:0] decoded_rs1, decoded_rs2, decoded_rd;
  logic          [1:0] decoded_rd_we, decoded_needs_checkpoint;
  ooo_op_class_e [1:0] decoded_op_class;
  ooo_fu_class_e [1:0] decoded_fu_class;
  alu_op_e       [1:0] decoded_alu_op;
  muldiv_op_e    [1:0] decoded_muldiv_op;
  br_type_e      [1:0] decoded_branch_op;
  ooo_src_sel_e  [1:0] decoded_src1_sel, decoded_src2_sel;
  decoded_trap_t decoded_trap;
  logic [1:0] decoded_is_load, decoded_is_store, decoded_mem_unsigned;
  mem_size_e [1:0] decoded_mem_size;

  logic redirect_valid;  word_t redirect_target;
  logic [1:0] commit_fire, commit_rd_wen;
  commit_order_t commit_order;
  word_t [1:0] commit_pc, commit_inst, commit_wdata;
  arch_reg_t [1:0] commit_rd;

  logic dmem_valid, dmem_we;
  logic [3:0] dmem_be;
  word_t dmem_addr, dmem_wdata, dmem_rdata;

  int checks = 0, errors = 0;

  rv32i_ss_core dut (
    .clk(clk), .rst_n(rst_n),
    .decoded_valid(decoded_valid), .decoded_slot_valid(decoded_slot_valid),
    .decoded_ready(decoded_ready),
    .decoded_pc(decoded_pc), .decoded_instr(decoded_instr),
    .decoded_rs1(decoded_rs1), .decoded_rs2(decoded_rs2),
    .decoded_rd(decoded_rd), .decoded_rd_we(decoded_rd_we),
    .decoded_needs_checkpoint(decoded_needs_checkpoint),
    .decoded_op_class(decoded_op_class), .decoded_fu_class(decoded_fu_class),
    .decoded_alu_op(decoded_alu_op), .decoded_muldiv_op(decoded_muldiv_op),
    .decoded_branch_op(decoded_branch_op),
    .decoded_src1_sel(decoded_src1_sel), .decoded_src2_sel(decoded_src2_sel),
    .decoded_imm(decoded_imm), .decoded_trap(decoded_trap),
    .decoded_csr_op(rv32i_pipeline_pkg::CSR_NONE),
    .decoded_csr_addr('0), .decoded_csr_zimm('0),
    .redirect_valid(redirect_valid), .redirect_target(redirect_target),
    .commit_fire(commit_fire), .commit_order(commit_order),
    .commit_pc(commit_pc), .commit_inst(commit_inst),
    .commit_rd(commit_rd), .commit_rd_wen(commit_rd_wen),
    .commit_wdata(commit_wdata),
    .decoded_is_load(decoded_is_load), .decoded_is_store(decoded_is_store),
    .decoded_mem_size(decoded_mem_size),
    .decoded_mem_unsigned(decoded_mem_unsigned),
    .decoded_pred_taken('0),  // tie-off: static not-taken prediction
    .decoded_pred_target('0),
    .dmem_valid(dmem_valid), .dmem_we(dmem_we), .dmem_be(dmem_be),
    .dmem_addr(dmem_addr), .dmem_wdata(dmem_wdata),
    .dmem_ready(1'b1), .dmem_rvalid(1'b1), .dmem_rdata(dmem_rdata)
  );

  ooo_dmem_model #(.MEM_WORDS(256), .MEM_MSB(9)) u_dmem (
    .clk(clk), .rst_n(rst_n),
    .addr(dmem_addr), .rdata(dmem_rdata),
    .we(dmem_we), .be(dmem_be), .wdata(dmem_wdata),
    .tohost_addr(32'hFFFF_FFFC), .tohost_full_addr(32'hFFFF_FFFC),
    .tohost_we(), .tohost_val()
  );

  // ---- commit monitor: records every fired commit position ----
  word_t     cm_pc   [0:63];
  arch_reg_t cm_rd   [0:63];
  word_t     cm_wd   [0:63];
  int        cm_cycle [0:63];
  int        cm_n = 0;
  int        cycle_n = 0;
  int        two_store_stop_n = 0;
  integer    commit_slot;
  always @(posedge clk) begin
    if (rst_n) begin
      cycle_n = cycle_n + 1;
      for (commit_slot = 0; commit_slot < 2; commit_slot++) begin
        if (commit_fire[commit_slot]) begin
          cm_pc[cm_n] = commit_pc[commit_slot];
          cm_rd[cm_n] = commit_rd[commit_slot];
          cm_wd[cm_n] = commit_wdata[commit_slot];
          cm_cycle[cm_n] = cycle_n;
          // Blocking update preserves both program-ordered records when two
          // positions retire on one edge.
          cm_n = cm_n + 1;
        end
      end
      if ((commit_fire == 2'b01) && dut.rob_commit_valid[1] &&
          dut.rob_commit_is_store[0] && dut.rob_commit_is_store[1])
        two_store_stop_n = two_store_stop_n + 1;
    end
  end

  initial begin
    repeat (3000) @(posedge clk);
    $display("DBG cm_n=%0d rob_count=%0d ready=%b fire=%b psn=%b avail=%b",
             cm_n, dut.u_rob.count_q, decoded_ready,
             dut.bundle_fire, dut.preg_slot_need, dut.preg_avail);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_dual exceeded 3000 cycles");
  end

  task automatic chk(input string name, input logic cond);
    checks++;
    if (!cond) begin errors++; $display("  FAIL [%0d] %s", checks, name); end
  endtask

  function automatic int iq_count();
    int n; n = 0;
    for (int i = 0; i < OOO_IQ_DEPTH; i++) if (dut.u_iq.valid_q[i]) n++;
    iq_count = n;
  endfunction

  task automatic clear_inputs();
    decoded_valid = 0; decoded_slot_valid = '0;
    decoded_pc = '0; decoded_instr = '0; decoded_imm = '0;
    decoded_rs1 = '0; decoded_rs2 = '0; decoded_rd = '0;
    decoded_rd_we = '0; decoded_needs_checkpoint = '0;
    decoded_op_class[0] = OOO_OP_ALU; decoded_op_class[1] = OOO_OP_ALU;
    decoded_fu_class[0] = OOO_FU_ALU; decoded_fu_class[1] = OOO_FU_ALU;
    decoded_alu_op[0] = fyp_cpu_pkg::ALU_ADD;
    decoded_alu_op[1] = fyp_cpu_pkg::ALU_ADD;
    decoded_muldiv_op[0] = rv32i_pipeline_pkg::MD_MUL;
    decoded_muldiv_op[1] = rv32i_pipeline_pkg::MD_MUL;
    decoded_branch_op[0] = rv32i_pipeline_pkg::BR_NONE;
    decoded_branch_op[1] = rv32i_pipeline_pkg::BR_NONE;
    decoded_src1_sel[0] = OOO_SRC_REG; decoded_src1_sel[1] = OOO_SRC_REG;
    decoded_src2_sel[0] = OOO_SRC_IMM; decoded_src2_sel[1] = OOO_SRC_IMM;
    decoded_trap = '0;
    decoded_is_load = '0; decoded_is_store = '0; decoded_mem_unsigned = '0;
    decoded_mem_size[0] = fyp_cpu_pkg::MEM_W;
    decoded_mem_size[1] = fyp_cpu_pkg::MEM_W;
  endtask

  // dual ALU bundle: rd[i] = x0 + imm[i]  (src1 x0, src2 IMM)
  task automatic bundle2_addi(
      input word_t pc0, input arch_reg_t rd0, input word_t imm0,
      input arch_reg_t rd1, input word_t imm1);
    @(negedge clk);
    decoded_valid = 1; decoded_slot_valid = 2'b11;
    decoded_pc = {word_t'(pc0 + 32'd4), pc0};
    decoded_rd = {rd1, rd0}; decoded_rd_we = 2'b11;
    decoded_imm = {imm1, imm0};
    decoded_rs1 = '0; decoded_rs2 = '0;
    wait (decoded_ready === 1'b1); @(posedge clk); @(negedge clk);
    decoded_valid = 0; decoded_slot_valid = '0; decoded_rd_we = '0;
  endtask

  // Two older ALUs held on x3. Memory operations dispatched behind them may
  // complete, but cannot retire; releasing x3 exposes the following adjacent
  // memory rows as one natural commit group on the next cycle.
  task automatic dispatch_held_alu_pair(input word_t pc0);
    @(negedge clk);
    decoded_valid = 1; decoded_slot_valid = 2'b11;
    decoded_pc = {word_t'(pc0 + 32'd4), pc0};
    decoded_instr = {32'h00118f93, 32'h00118f13}; // addi x31/x30,x3,1
    decoded_rd = {5'd31, 5'd30}; decoded_rd_we = 2'b11;
    decoded_rs1 = {5'd3, 5'd3}; decoded_rs2 = '0;
    decoded_imm = {32'd1, 32'd1};
    wait (decoded_ready === 1'b1); @(posedge clk); @(negedge clk);
    clear_inputs();
  endtask

  task automatic dispatch_mem_one(
      input word_t pc, input word_t addr,
      input logic is_load, input logic is_store,
      input arch_reg_t rd, output rob_idx_t row_idx);
    @(negedge clk);
    decoded_valid = 1; decoded_slot_valid = 2'b01;
    decoded_pc = {32'h0, pc};
    decoded_instr = {32'h0, is_load ? 32'h00002003 : 32'h00002023};
    decoded_rd = {5'd0, rd}; decoded_rd_we = {1'b0, is_load};
    decoded_rs1 = '0; decoded_rs2 = '0;
    decoded_imm = {32'h0, addr};
    decoded_fu_class[0] = OOO_FU_LSU;
    decoded_is_load = {1'b0, is_load};
    decoded_is_store = {1'b0, is_store};
    // The AGEN adder always takes the immediate. Store data is an independent
    // prs2 read in the physical-unit router, not operand_b.
    decoded_src2_sel[0] = OOO_SRC_IMM;
    wait (decoded_ready === 1'b1);
    row_idx = dut.rob_alloc_idx[0];
    @(posedge clk); @(negedge clk);
    clear_inputs();
  endtask

  int n0;
  int total_before;
  rob_idx_t d3_load_rob_idx;
  rob_idx_t d3_store_rob_idx;
  rob_idx_t mem_row0, mem_row1;
  int mem_base;
  int stop_before;

  initial begin
    clear_inputs();
    rst_n = 0; repeat (4) @(posedge clk); @(negedge clk); rst_n = 1;
    @(negedge clk);

    // ================= dual-ALU smoke =================
    bundle2_addi(32'h0000_1000, 5'd5, 32'd111, 5'd6, 32'd222);
    // Wait for both program-ordered commit records; may produce both in
    // the same cycle.
    wait (cm_n >= 2); @(negedge clk);
    chk("D1 two commits, program order (pc)",
        (cm_pc[0] == 32'h0000_1000) && (cm_pc[1] == 32'h0000_1004));
    chk("D1 slot-0 result", (cm_rd[0] == 5'd5) && (cm_wd[0] == 32'd111));
    chk("D1 slot-1 result", (cm_rd[1] == 5'd6) && (cm_wd[1] == 32'd222));
    chk("D1 both ALUs retire in one commit group", cm_cycle[0] == cm_cycle[1]);
    chk("D1 ROB drained", dut.u_rob.count_q == '0);

    // ============= same-bundle WAW — younger wins the map =============
    bundle2_addi(32'h0000_2000, 5'd7, 32'd333, 5'd7, 32'd444);
    wait (cm_n >= 4); @(negedge clk);
    chk("D2 both WAW commits in order",
        (cm_wd[2] == 32'd333) && (cm_wd[3] == 32'd444));
    chk("D2 WAW pair retires in one commit group", cm_cycle[2] == cm_cycle[3]);
    // reader: x9 = x7 + 0 -> must see slot 1's 444
    @(negedge clk);
    decoded_valid = 1; decoded_slot_valid = 2'b01;
    decoded_pc = {32'h0, 32'h0000_2008};
    decoded_rd = {5'd0, 5'd9}; decoded_rd_we = 2'b01;
    decoded_rs1 = {5'd0, 5'd7}; decoded_imm = '0;
    wait (decoded_ready === 1'b1); @(posedge clk); @(negedge clk);
    decoded_valid = 0; decoded_slot_valid = '0; decoded_rd_we = '0; decoded_rs1 = '0;
    wait (cm_n >= 5); @(negedge clk);
    chk("D2 reader sees the younger mapping", cm_wd[4] == 32'd444);

    // ================= memory op in slot 1 (case 5) =================
    u_dmem.mem[32'h40 >> 2] = 32'hfeed_beef;
    // Hold the older ALU on x3 while the independent younger load completes.
    // Releasing x3 after the load's ROB row is done creates the consequential
    // ALU+load dual-retire state instead of letting the older ALU escape alone.
    force dut.u_prf.ready_q[3] = 1'b0;
    @(negedge clk);
    decoded_valid = 1; decoded_slot_valid = 2'b11;
    decoded_pc = {32'h0000_3004, 32'h0000_3000};
    decoded_rd = {5'd11, 5'd10}; decoded_rd_we = 2'b11;
    decoded_imm = {32'h0000_0040, 32'd555};       // slot1: LW x11, 0x40(x0)
    decoded_rs1 = {5'd0, 5'd3}; decoded_rs2 = '0;
    decoded_is_load = 2'b10;
    decoded_fu_class[1] = OOO_FU_LSU;
    wait (decoded_ready === 1'b1);
    d3_load_rob_idx = dut.rob_alloc_idx[1];
    @(posedge clk); @(negedge clk);
    decoded_valid = 0; decoded_slot_valid = '0; decoded_rd_we = '0;
    decoded_is_load = '0; decoded_fu_class[1] = OOO_FU_ALU;
    decoded_rs1 = '0;
    wait (dut.u_rob.done_q[d3_load_rob_idx] === 1'b1);
    force dut.u_prf.ready_q[3] = 1'b1;
    @(posedge clk); @(negedge clk);
    release dut.u_prf.ready_q[3];
    wait (cm_n >= 7); @(negedge clk);
    chk("D3 slot-0 ALU committed first", (cm_rd[5] == 5'd10) && (cm_wd[5] == 32'd555));
    chk("D3 slot-1 LOAD got slot-1 metadata (rd + loaded value)",
        (cm_rd[6] == 5'd11) && (cm_wd[6] == 32'hfeed_beef));
    chk("D3 ALU+load dual-commits", cm_cycle[5] == cm_cycle[6]);
    chk("D3 committing load popped its LQ row", dut.u_lsq.lq_count_q == '0);

    // ================ memory op in position 0 ======================
    // Reverse the orientation so the load is the older position and the ALU
    // is younger. Both must retire together and the one load must pop exactly
    // one LQ entry.
    u_dmem.mem[32'h44 >> 2] = 32'hcafe_babe;
    total_before = cm_n;
    @(negedge clk);
    decoded_valid = 1; decoded_slot_valid = 2'b11;
    decoded_pc = {32'h0000_300c, 32'h0000_3008};
    decoded_rd = {5'd13, 5'd12}; decoded_rd_we = 2'b11;
    decoded_imm = {32'd666, 32'h0000_0044};
    decoded_rs1 = '0; decoded_rs2 = '0;
    decoded_is_load = 2'b01;
    decoded_fu_class[0] = OOO_FU_LSU;
    wait (decoded_ready === 1'b1); @(posedge clk); @(negedge clk);
    decoded_valid = 0; decoded_slot_valid = '0; decoded_rd_we = '0;
    decoded_is_load = '0; decoded_fu_class[0] = OOO_FU_ALU;
    wait (cm_n >= total_before + 2); @(negedge clk);
    chk("D3b position-0 LOAD value",
        (cm_rd[total_before] == 5'd12) &&
        (cm_wd[total_before] == 32'hcafe_babe));
    chk("D3b position-1 ALU value",
        (cm_rd[total_before+1] == 5'd13) &&
        (cm_wd[total_before+1] == 32'd666));
    chk("D3b load+ALU dual-commits",
        cm_cycle[total_before] == cm_cycle[total_before+1]);
    chk("D3b committing load popped its LQ row", dut.u_lsq.lq_count_q == '0);

    // ================ ALU then store ================================
    // Hold the older ALU until the younger store has deposited its payload,
    // then prove both retire together and the position-1 store owns the sole
    // dmem write.
    u_dmem.mem[32'ha0 >> 2] = 32'hffff_ffff;
    force dut.u_prf.ready_q[3] = 1'b0;
    total_before = cm_n;
    @(negedge clk);
    decoded_valid = 1; decoded_slot_valid = 2'b11;
    decoded_pc = {32'h0000_3014, 32'h0000_3010};
    decoded_instr = {32'h00002023, 32'h30900713};
    decoded_rd = {5'd0, 5'd14}; decoded_rd_we = 2'b01;
    decoded_rs1 = {5'd0, 5'd3}; decoded_rs2 = '0;
    decoded_imm = {32'h0000_00a0, 32'd777};
    decoded_is_store = 2'b10;
    decoded_fu_class[1] = OOO_FU_LSU;
    wait (decoded_ready === 1'b1);
    d3_store_rob_idx = dut.rob_alloc_idx[1];
    @(posedge clk); @(negedge clk);
    clear_inputs();
    wait (dut.u_rob.done_q[d3_store_rob_idx] === 1'b1);
    force dut.u_prf.ready_q[3] = 1'b1;
    @(posedge clk); @(negedge clk); release dut.u_prf.ready_q[3];
    wait (cm_n >= total_before + 2); @(negedge clk);
    chk("D3g ALU+store dual-commits",
        cm_cycle[total_before] == cm_cycle[total_before+1]);
    chk("D3g ALU value and position-1 store effect land",
        (cm_wd[total_before] == 32'd777) &&
        (u_dmem.mem[32'ha0 >> 2] == 32'h0000_0000));

    // ================ store then ALU ================================
    // The older store naturally holds the head until its payload is ready;
    // the younger ALU must retire beside it when the drain fires.
    u_dmem.mem[32'ha4 >> 2] = 32'hffff_ffff;
    total_before = cm_n;
    @(negedge clk);
    decoded_valid = 1; decoded_slot_valid = 2'b11;
    decoded_pc = {32'h0000_301c, 32'h0000_3018};
    decoded_instr = {32'h37800793, 32'h00002023};
    decoded_rd = {5'd15, 5'd0}; decoded_rd_we = 2'b10;
    decoded_rs1 = '0; decoded_rs2 = '0;
    decoded_imm = {32'd888, 32'h0000_00a4};
    decoded_is_store = 2'b01;
    decoded_fu_class[0] = OOO_FU_LSU;
    wait (decoded_ready === 1'b1); @(posedge clk); @(negedge clk);
    clear_inputs();
    wait (cm_n >= total_before + 2); @(negedge clk);
    chk("D3h store+ALU dual-commits",
        cm_cycle[total_before] == cm_cycle[total_before+1]);
    chk("D3h position-0 store effect and ALU value land",
        (u_dmem.mem[32'ha4 >> 2] == 32'h0000_0000) &&
        (cm_wd[total_before+1] == 32'd888));

    // ============== adjacent loads from separate bundles ===========
    // Frontend formation forbids two memory ops in one bundle. Dispatch them
    // as adjacent singleton bundles behind a held older ALU pair, let both
    // complete, then release the pair. The loads must become the next natural
    // two-retire group and pop two consecutive LQ rows.
    u_dmem.mem[32'h80 >> 2] = 32'h1111_aaaa;
    u_dmem.mem[32'h84 >> 2] = 32'h2222_bbbb;
    force dut.u_prf.ready_q[3] = 1'b0;
    dispatch_held_alu_pair(32'h0000_3100);
    dispatch_mem_one(32'h0000_3108, 32'h80, 1'b1, 1'b0, 5'd22, mem_row0);
    dispatch_mem_one(32'h0000_310c, 32'h84, 1'b1, 1'b0, 5'd23, mem_row1);
    wait (dut.u_rob.done_q[mem_row0] && dut.u_rob.done_q[mem_row1]);
    mem_base = cm_n;
    force dut.u_prf.ready_q[3] = 1'b1;
    @(posedge clk); @(negedge clk); release dut.u_prf.ready_q[3];
    wait (cm_n >= mem_base + 4); @(negedge clk);
    chk("D3c held ALU prefix dual-commits",
        cm_cycle[mem_base] == cm_cycle[mem_base+1]);
    chk("D3c adjacent loads retire in program order",
        (cm_pc[mem_base+2] == 32'h0000_3108) &&
        (cm_pc[mem_base+3] == 32'h0000_310c));
    chk("D3c load+load dual-commits",
        cm_cycle[mem_base+2] == cm_cycle[mem_base+3]);
    chk("D3c both load values are architectural",
        (cm_wd[mem_base+2] == 32'h1111_aaaa) &&
        (cm_wd[mem_base+3] == 32'h2222_bbbb));
    chk("D3c dual load popped both LQ rows", dut.u_lsq.lq_count_q == '0);

    // ============== load then store, both at commit =================
    u_dmem.mem[32'h88 >> 2] = 32'h3333_cccc;
    u_dmem.mem[32'h8c >> 2] = 32'hffff_ffff;
    force dut.u_prf.ready_q[3] = 1'b0;
    dispatch_held_alu_pair(32'h0000_3200);
    dispatch_mem_one(32'h0000_3208, 32'h88, 1'b1, 1'b0, 5'd24, mem_row0);
    dispatch_mem_one(32'h0000_320c, 32'h8c, 1'b0, 1'b1, 5'd0,  mem_row1);
    wait (dut.u_rob.done_q[mem_row0] && dut.u_rob.done_q[mem_row1]);
    mem_base = cm_n;
    force dut.u_prf.ready_q[3] = 1'b1;
    @(posedge clk); @(negedge clk); release dut.u_prf.ready_q[3];
    wait (cm_n >= mem_base + 4); @(negedge clk);
    chk("D3d load+store dual-commits",
        cm_cycle[mem_base+2] == cm_cycle[mem_base+3]);
    chk("D3d load result and store effect both land",
        (cm_wd[mem_base+2] == 32'h3333_cccc) &&
        (u_dmem.mem[32'h8c >> 2] == 32'h0000_0000));
    chk("D3d load pop plus store drain empty both queues",
        (dut.u_lsq.lq_count_q == '0) && (dut.u_lsq.sq_count_q == '0));

    // ============== store then load, reverse orientation ============
    u_dmem.mem[32'h90 >> 2] = 32'hffff_ffff;
    u_dmem.mem[32'h94 >> 2] = 32'h4444_dddd;
    force dut.u_prf.ready_q[3] = 1'b0;
    dispatch_held_alu_pair(32'h0000_3300);
    dispatch_mem_one(32'h0000_3308, 32'h90, 1'b0, 1'b1, 5'd0,  mem_row0);
    dispatch_mem_one(32'h0000_330c, 32'h94, 1'b1, 1'b0, 5'd25, mem_row1);
    wait (dut.u_rob.done_q[mem_row0] && dut.u_rob.done_q[mem_row1]);
    mem_base = cm_n;
    force dut.u_prf.ready_q[3] = 1'b1;
    @(posedge clk); @(negedge clk); release dut.u_prf.ready_q[3];
    wait (cm_n >= mem_base + 4); @(negedge clk);
    chk("D3e store+load dual-commits",
        cm_cycle[mem_base+2] == cm_cycle[mem_base+3]);
    chk("D3e store effect and load result both land",
        (u_dmem.mem[32'h90 >> 2] == 32'h0000_0000) &&
        (cm_wd[mem_base+3] == 32'h4444_dddd));
    chk("D3e store drain plus load pop empty both queues",
        (dut.u_lsq.lq_count_q == '0) && (dut.u_lsq.sq_count_q == '0));

    // ============== two stores stop the ordered prefix ==============
    u_dmem.mem[32'h98 >> 2] = 32'hffff_ffff;
    u_dmem.mem[32'h9c >> 2] = 32'hffff_ffff;
    force dut.u_prf.ready_q[3] = 1'b0;
    dispatch_held_alu_pair(32'h0000_3400);
    dispatch_mem_one(32'h0000_3408, 32'h98, 1'b0, 1'b1, 5'd0, mem_row0);
    dispatch_mem_one(32'h0000_340c, 32'h9c, 1'b0, 1'b1, 5'd0, mem_row1);
    wait (dut.u_rob.done_q[mem_row0] && dut.u_rob.done_q[mem_row1]);
    mem_base = cm_n;
    stop_before = two_store_stop_n;
    force dut.u_prf.ready_q[3] = 1'b1;
    @(posedge clk); @(negedge clk); release dut.u_prf.ready_q[3];
    wait (cm_n >= mem_base + 4); @(negedge clk);
    chk("D3f store+store exclusion was actually challenged",
        two_store_stop_n > stop_before);
    chk("D3f stores retire on consecutive cycles, not together",
        cm_cycle[mem_base+2] != cm_cycle[mem_base+3]);
    chk("D3f first store remains older",
        (cm_pc[mem_base+2] == 32'h0000_3408) &&
        (cm_pc[mem_base+3] == 32'h0000_340c));
    chk("D3f both stores eventually reach memory",
        (u_dmem.mem[32'h98 >> 2] == 32'h0000_0000) &&
        (u_dmem.mem[32'h9c >> 2] == 32'h0000_0000));
    chk("D3f SQ drains completely", dut.u_lsq.sq_count_q == '0);

    // ================= IQ one hole + dual offer -> zero delta (case 1) ==
    // Setup: x1/x2 nonzero so the iterative divider runs FULL LENGTH (a
    // zero divisor early-outs and the IQ can never fill behind it).
    bundle2_addi(32'h0000_3f00, 5'd1, 32'd61440, 5'd2, 32'd3);
    wait (cm_n >= 9); @(negedge clk);
    // div x12 = x1/x2 occupies the muldiv unit; dependents on x12 pile into
    // the IQ until exactly ONE hole remains, then a dual offer must hold.
    @(negedge clk);
    decoded_valid = 1; decoded_slot_valid = 2'b01;
    decoded_pc = {32'h0, 32'h0000_4000};
    decoded_rd = {5'd0, 5'd12}; decoded_rd_we = 2'b01;
    decoded_rs1 = {5'd0, 5'd1}; decoded_rs2 = {5'd0, 5'd2};
    decoded_fu_class[0] = OOO_FU_MULDIV;
    decoded_muldiv_op[0] = rv32i_pipeline_pkg::MD_DIV;
    decoded_src2_sel[0] = OOO_SRC_REG;
    wait (decoded_ready === 1'b1); @(posedge clk); @(negedge clk);
    decoded_valid = 0; decoded_slot_valid = '0; decoded_rd_we = '0;
    decoded_fu_class[0] = OOO_FU_ALU;
    decoded_muldiv_op[0] = rv32i_pipeline_pkg::MD_MUL;
    decoded_src2_sel[0] = OOO_SRC_IMM;
    // dependents: x13 = x12 + k, fill the IQ to depth-1 (guarded)
    n0 = 0;
    while (iq_count() < OOO_IQ_DEPTH - 1) begin
      n0++;
      if (n0 > 40) $fatal(1, "D4: IQ never filled — div completed too fast?");
      @(negedge clk);
      decoded_valid = 1; decoded_slot_valid = 2'b01;
      decoded_pc = {32'h0, 32'h0000_4100};
      decoded_rd = {5'd0, 5'd13}; decoded_rd_we = 2'b01;
      decoded_rs1 = {5'd0, 5'd12}; decoded_imm = {32'h0, 32'd1};
      wait (decoded_ready === 1'b1); @(posedge clk); @(negedge clk);
      decoded_valid = 0; decoded_slot_valid = '0; decoded_rd_we = '0;
      decoded_rs1 = '0;
    end
    chk("D4 IQ at depth-1", iq_count() == OOO_IQ_DEPTH - 1);
    // present the dual offer: ROB has room, IQ does not (needs 2, has 1)
    @(negedge clk);
    decoded_valid = 1; decoded_slot_valid = 2'b11;
    decoded_pc = {32'h0000_5004, 32'h0000_5000};
    decoded_rd = {5'd15, 5'd14}; decoded_rd_we = 2'b11;
    decoded_imm = {32'd777, 32'd666}; decoded_rs1 = '0;
    #1;
    n0 = int'(dut.u_rob.count_q);
    total_before = cm_n;
    chk("D4 dual offer held (ready low)", decoded_ready === 1'b0);
    chk("D4 no fire under partial block", dut.bundle_fire === 1'b0);
    repeat (2) @(negedge clk);
    chk("D4 zero delta while held (ROB count, IQ count)",
        (int'(dut.u_rob.count_q) == n0) && (iq_count() == OOO_IQ_DEPTH - 1));
    // keep the offer up; the div completes, the IQ drains, the bundle fires
    wait (decoded_ready === 1'b1); @(posedge clk); @(negedge clk);
    decoded_valid = 0; decoded_slot_valid = '0; decoded_rd_we = '0;
    wait (dut.u_rob.count_q == '0); @(negedge clk);
    chk("D4 held bundle completed after drain (both results)",
        (cm_wd[cm_n-2] == 32'd666) && (cm_wd[cm_n-1] == 32'd777));

    // ================= trap broadcast forbids the fire (case 6) =========
    @(negedge clk);
    decoded_valid = 1; decoded_slot_valid = 2'b01;
    decoded_pc = {32'h0, 32'h0000_6000};
    decoded_rd = '0; decoded_rd_we = '0;
    decoded_trap.valid = 1'b1;
    decoded_trap.cause = OOO_CAUSE_ECALL_M;
    wait (decoded_ready === 1'b1); @(posedge clk); @(negedge clk);
    decoded_trap = '0;
    // present a dual offer and hold it through the trap flush window
    decoded_valid = 1; decoded_slot_valid = 2'b11;
    decoded_pc = {32'h0000_7004, 32'h0000_7000};
    decoded_rd = {5'd17, 5'd16}; decoded_rd_we = 2'b11;
    decoded_imm = {32'd999, 32'd888};
    n0 = cm_n;
    wait (dut.trap_q_valid === 1'b1);
    #1;
    chk("D5 no fire on the trap broadcast cycle", dut.bundle_fire === 1'b0);
    chk("D5 decoded_ready low under trap flush", decoded_ready === 1'b0);
    @(negedge clk);
    decoded_valid = 0; decoded_slot_valid = '0; decoded_rd_we = '0;
    repeat (4) @(negedge clk);
    chk("D5 flush emptied the ROB", dut.u_rob.count_q == '0);


    // ===== taken-branch decision in N +
    // younger shape-11 fire in the decision window + registered recovery
    // in N+1 wipes BOTH younger slots (-reachable — dual DISPATCH is
    // live; only ONE branch execution is needed).
    begin
      int nf; word_t om20, om21;
      phys_reg_t u0p, u1p;
      repeat (4) @(negedge clk);
      nf = 0;
      for (int i = 0; i < OOO_PHYS_REGS; i++) if (dut.u_free_list.free_bits_q[i]) nf++;
      om20 = word_t'(dut.u_rename.spec_map_q[20]);
      om21 = word_t'(dut.u_rename.spec_map_q[21]);
      // branch A: beq x0,x0,+16 (taken), checkpointed, no rd
      @(negedge clk);
      decoded_valid = 1; decoded_slot_valid = 2'b01;
      decoded_pc = {32'h0, 32'h0000_8000};
      decoded_rd = '0; decoded_rd_we = '0;
      decoded_rs1 = '0; decoded_rs2 = '0;   // explicit: BEQ x0,x0 (leftover-state lesson)
      decoded_needs_checkpoint = 2'b01;
      decoded_op_class[0] = OOO_OP_BRANCH;
      decoded_branch_op[0] = rv32i_pipeline_pkg::BR_BEQ;
      decoded_src2_sel[0] = OOO_SRC_REG;
      decoded_imm = {32'h0, 32'd16};
      wait (decoded_ready === 1'b1); @(posedge clk); @(negedge clk);
      decoded_needs_checkpoint = '0;
      decoded_op_class[0] = OOO_OP_ALU;
      decoded_branch_op[0] = rv32i_pipeline_pkg::BR_NONE;
      decoded_src2_sel[0] = OOO_SRC_IMM;
      // younger dual fires in A's decision window
      decoded_valid = 1; decoded_slot_valid = 2'b11;
      decoded_pc = {32'h0000_8008, 32'h0000_8004};
      decoded_rd = {5'd21, 5'd20}; decoded_rd_we = 2'b11;
      decoded_imm = {32'd212, 32'd211};
      #1;
      u0p = dut.rename_pdst[0]; u1p = dut.rename_pdst[1];
      wait (decoded_ready === 1'b1); @(posedge clk); @(negedge clk);
      decoded_valid = 0; decoded_slot_valid = '0; decoded_rd_we = '0;
      // let the recovery broadcast land and the machine quiesce
      repeat (10) @(negedge clk);
      chk("D6 map x20 rolled back", word_t'(dut.u_rename.spec_map_q[20]) == om20);
      chk("D6 map x21 rolled back", word_t'(dut.u_rename.spec_map_q[21]) == om21);
      chk("D6 BOTH younger pregs reclaimed (no leak)",
          dut.u_free_list.free_bits_q[u0p] && dut.u_free_list.free_bits_q[u1p]);
      begin
        int nf2; nf2 = 0;
        for (int i = 0; i < OOO_PHYS_REGS; i++) if (dut.u_free_list.free_bits_q[i]) nf2++;
        chk("D6 free-list conservation", nf2 == nf);
      end
      chk("D6 ROB drained after recovery + A commit", dut.u_rob.count_q == '0);
    end

    if (errors == 0) begin
      $display("[tb_rv32i_ss_core_dual] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_core_dual] FAIL errors=%0d checks=%0d",
             errors, checks);
    end
  end

endmodule
