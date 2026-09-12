`timescale 1ns/1ps

// =============================================================================
// tb_rv32i_ss_core_t36_natural -- three natural end-to-end programs through
// the REAL frontend (imem-driven): dual-correct case dual-correct release, dual-recovery case dual-taken
// recovery with a wrong-path S_DONE MUL holder surviving the broadcast and
// draining as a rejected ghost, lane-1 CSR case natural lane-1 CSR beside an older MUL.
// No completion/recovery/release pulse is forced; alignment comes from
// delaying the external dmem response against observed internal state.
//
// =============================================================================
module tb_rv32i_ss_core_t36_natural;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;

  logic clk, rst_n;

  logic dec_valid, dec_ready;
  logic [1:0] dec_slot_valid;
  word_t [1:0] dec_pc, dec_instr, dec_imm;
  arch_reg_t [1:0] dec_rs1, dec_rs2, dec_rd;
  logic [1:0] dec_rd_we, dec_needs_checkpoint;
  ooo_op_class_e [1:0] dec_op_class;
  ooo_fu_class_e [1:0] dec_fu_class;
  muldiv_op_e [1:0] dec_muldiv_op;
  alu_op_e [1:0] dec_alu_op;
  br_type_e [1:0] dec_branch_op;
  ooo_src_sel_e [1:0] dec_src1_sel, dec_src2_sel;
  decoded_trap_t dec_trap;
  csr_op_e dec_csr_op;
  csr_addr_t dec_csr_addr;
  csr_zimm_t dec_csr_zimm;
  logic [1:0] dec_is_load, dec_is_store, dec_mem_unsigned;
  mem_size_e [1:0] dec_mem_size;

  word_t imem_addr;
  word_t [1:0] imem_rdata;
  logic imem_req_valid, imem_req_ready;
  word_t imem_req_addr;
  logic imem_resp_valid, imem_resp_ready;
  word_t [1:0] imem_resp_data;
  word_t imem [0:255];
  assign imem_rdata = {
    imem[{imem_addr[9:3], 1'b1}],
    imem[{imem_addr[9:3], 1'b0}]
  };

  logic redirect_valid;
  word_t redirect_target;
  logic [1:0] commit_fire, commit_rd_wen;
  commit_order_t commit_order;
  word_t [1:0] commit_pc, commit_inst, commit_wdata;
  arch_reg_t [1:0] commit_rd;

  logic dmem_valid, dmem_we, dmem_ready, dmem_rvalid;
  logic [3:0] dmem_be;
  word_t dmem_addr, dmem_wdata, dmem_rdata;

  int shape11_fires, recovery_broadcasts, csr_stall_cycles;
  bit dual_correct_seen, release_two_seen, dual_recover_seen;
  bit broadcast_shape11_seen, mixed_wb_seen, marker_seen;
  integer commit_slot;
  bit csr_lane1_seen, refill_commit_seen;
  word_t selected_target_seen, marker_value;
  int checks, scenarios;

  rv32i_ss_imem_zero_latency_adapter u_imem_adapter (
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr), .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready), .imem_resp_data(imem_resp_data),
    .line_addr(imem_addr), .line_data(imem_rdata)
  );

  rv32i_ss_frontend u_fe (
    .clk(clk), .rst_n(rst_n),
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr), .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready), .imem_resp_data(imem_resp_data),
    .bp_update_valid(1'b0),  // tie-off: predictor never trains (inert)
    .bp_update_pc('0), .bp_update_taken(1'b0), .bp_update_target('0), .redirect_valid(redirect_valid),
    .redirect_target(redirect_target), .decoded_valid(dec_valid),
    .decoded_slot_valid(dec_slot_valid), .decoded_ready(dec_ready),
    .decoded_pc(dec_pc), .decoded_instr(dec_instr),
    .decoded_rs1(dec_rs1), .decoded_rs2(dec_rs2), .decoded_rd(dec_rd),
    .decoded_rd_we(dec_rd_we),
    .decoded_needs_checkpoint(dec_needs_checkpoint),
    .decoded_op_class(dec_op_class), .decoded_fu_class(dec_fu_class),
    .decoded_muldiv_op(dec_muldiv_op), .decoded_alu_op(dec_alu_op),
    .decoded_branch_op(dec_branch_op), .decoded_src1_sel(dec_src1_sel),
    .decoded_src2_sel(dec_src2_sel), .decoded_imm(dec_imm),
    .decoded_trap(dec_trap), .decoded_csr_op(dec_csr_op),
    .decoded_csr_addr(dec_csr_addr), .decoded_csr_zimm(dec_csr_zimm),
    .decoded_is_load(dec_is_load), .decoded_is_store(dec_is_store),
    .decoded_mem_size(dec_mem_size),
    .decoded_mem_unsigned(dec_mem_unsigned)
  );

  rv32i_ss_core u_core (
    .clk(clk), .rst_n(rst_n), .decoded_valid(dec_valid),
    .decoded_slot_valid(dec_slot_valid), .decoded_ready(dec_ready),
    .decoded_pc(dec_pc), .decoded_instr(dec_instr),
    .decoded_rs1(dec_rs1), .decoded_rs2(dec_rs2), .decoded_rd(dec_rd),
    .decoded_rd_we(dec_rd_we),
    .decoded_needs_checkpoint(dec_needs_checkpoint),
    .decoded_op_class(dec_op_class), .decoded_fu_class(dec_fu_class),
    .decoded_muldiv_op(dec_muldiv_op), .decoded_alu_op(dec_alu_op),
    .decoded_branch_op(dec_branch_op), .decoded_src1_sel(dec_src1_sel),
    .decoded_src2_sel(dec_src2_sel), .decoded_imm(dec_imm),
    .decoded_trap(dec_trap), .decoded_csr_op(dec_csr_op),
    .decoded_csr_addr(dec_csr_addr), .decoded_csr_zimm(dec_csr_zimm),
    .decoded_is_load(dec_is_load), .decoded_is_store(dec_is_store),
    .decoded_mem_size(dec_mem_size),
    .decoded_mem_unsigned(dec_mem_unsigned),
    .decoded_pred_taken('0),  // tie-off: static not-taken prediction
    .decoded_pred_target('0),
    .redirect_valid(redirect_valid), .redirect_target(redirect_target),
    .commit_fire(commit_fire), .commit_order(commit_order),
    .commit_pc(commit_pc), .commit_inst(commit_inst), .commit_rd(commit_rd),
    .commit_rd_wen(commit_rd_wen), .commit_wdata(commit_wdata),
    .dmem_valid(dmem_valid), .dmem_we(dmem_we), .dmem_be(dmem_be),
    .dmem_addr(dmem_addr), .dmem_wdata(dmem_wdata),
    .dmem_ready(dmem_ready), .dmem_rvalid(dmem_rvalid),
    .dmem_rdata(dmem_rdata)
  );

  initial clk = 1'b0;
  always #5 clk = ~clk;
  initial begin
    repeat (12000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_t36_natural exceeded 12000 cycles");
  end

  function automatic int free_count_now();
    int n;
    n = 0;
    for (int i = 0; i < OOO_PHYS_REGS; i++)
      if (u_core.u_free_list.free_bits_q[i]) n++;
    return n;
  endfunction

  task automatic check(input string label, input logic condition);
    checks++;
    if (condition !== 1'b1)
      $fatal(1, "NATURAL check failed: %s", label);
  endtask

  task automatic check_arch(input string label, input int regno,
                            input word_t expected);
    phys_reg_t p;
    p = u_core.u_rename.committed_map_q[regno];
    checks++;
    if (u_core.u_prf.regs_q[p] !== expected)
      $fatal(1, "NATURAL %s x%0d=%08h via p%0d exp=%08h",
             label, regno, u_core.u_prf.regs_q[p], p, expected);
  endtask

  task automatic fill_nops();
    for (int i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  endtask

  task automatic clear_observers();
    shape11_fires = 0;
    recovery_broadcasts = 0;
    csr_stall_cycles = 0;
    dual_correct_seen = 1'b0;
    release_two_seen = 1'b0;
    dual_recover_seen = 1'b0;
    broadcast_shape11_seen = 1'b0;
    mixed_wb_seen = 1'b0;
    marker_seen = 1'b0;
    csr_lane1_seen = 1'b0;
    refill_commit_seen = 1'b0;
    selected_target_seen = '0;
    marker_value = '0;
  endtask

  task automatic reset_dut();
    rst_n = 1'b0;
    dmem_ready = 1'b1;
    dmem_rvalid = 1'b0;
    dmem_rdata = '0;
    repeat (4) @(posedge clk);
    @(negedge clk);
    clear_observers();
    rst_n = 1'b1;
  endtask

  task automatic wait_marker(input word_t expected, input int limit);
    int waited;
    waited = 0;
    while (!marker_seen && (waited < limit)) begin
      @(posedge clk);
      waited++;
    end
    if (!marker_seen)
      $fatal(1, "marker timeout expected=%0d commit_order=%0d pc=%08h",
             expected, commit_order, commit_pc[0]);
    @(negedge clk);
    check("marker value", marker_value == expected);
  endtask

  task automatic respond_when_mul_count(input word_t data,
                                        input int desired_count);
    int waited;
    waited = 0;
    while (!((u_core.u_lsq.lq_out_count_q != 2'd0) && u_core.muldiv_busy &&
             (u_core.u_muldiv.cnt_q == desired_count))) begin
      @(negedge clk);
      waited++;
      if (waited > 500)
        $fatal(1, "could not align dmem response: outstanding=%0b busy=%0b cnt=%0d",
               (u_core.u_lsq.lq_out_count_q != 2'd0),
               u_core.muldiv_busy, u_core.u_muldiv.cnt_q);
    end
    dmem_rdata = data;
    dmem_rvalid = 1'b1;
    @(posedge clk);
    @(negedge clk);
    dmem_rvalid = 1'b0;
    dmem_rdata = '0;
  endtask

  always @(posedge clk) begin
    if (rst_n) begin
      if (u_core.bundle_fire && (dec_slot_valid == 2'b11))
        shape11_fires <= shape11_fires + 1;
      if (u_core.branch_recover_req) begin
        recovery_broadcasts <= recovery_broadcasts + 1;
        if ((dec_slot_valid == 2'b11) && !u_core.bundle_fire)
          broadcast_shape11_seen <= 1'b1;
      end
      if (u_core.branch_candidate[0].correct_valid &&
          u_core.branch_candidate[1].correct_valid) begin
        dual_correct_seen <= 1'b1;
        $display("TRACE N1 dual-correct t=%0t head=%0d release=%b",
                 $time, u_core.rob_head_idx, u_core.checkpoint_release_mask);
      end
      if ((u_core.checkpoint_release_mask != 0) &&
          ((u_core.checkpoint_release_mask &
            (u_core.checkpoint_release_mask - 1'b1)) != 0))
        release_two_seen <= 1'b1;
      if (u_core.branch_candidate[0].recover_valid &&
          u_core.branch_candidate[1].recover_valid) begin
        dual_recover_seen <= 1'b1;
        selected_target_seen <= u_core.selected_recovery.target;
      end
      if (u_core.cdb_q[0].valid && u_core.cdb_q[1].valid &&
          ((u_core.rob_wb_accept == 2'b01) ||
           (u_core.rob_wb_accept == 2'b10)))
        mixed_wb_seen <= 1'b1;
      if (u_core.cdb_q[1].valid && u_core.cdb_q[1].csr_we)
        csr_lane1_seen <= 1'b1;
      if (u_core.csr_inflight_q && !dec_ready)
        csr_stall_cycles <= csr_stall_cycles + 1;
      for (commit_slot = 0; commit_slot < 2; commit_slot++) begin
        if (commit_fire[commit_slot]) begin
          if ((commit_pc[commit_slot] == 32'h0000_002c) &&
              commit_rd_wen[commit_slot] &&
              (commit_rd[commit_slot] == 14) &&
              (commit_wdata[commit_slot] == 32'd12321))
            refill_commit_seen <= 1'b1;
          if (commit_rd_wen[commit_slot] &&
              (commit_rd[commit_slot] == 31)) begin
            marker_seen <= 1'b1;
            marker_value <= commit_wdata[commit_slot];
            $display("TRACE marker t=%0t order=%0d pc=%08h value=%0d",
                     $time, commit_order + commit_order_t'(commit_slot),
                     commit_pc[commit_slot], commit_wdata[commit_slot]);
          end
        end
      end
    end
  end

  initial begin : campaign
    completion_packet_t held_mul;
    phys_reg_t ghost_pdst;
    word_t ghost_old_data;
    int wait_i;

    checks = 0;
    scenarios = 0;

    // Program 1: two correct branches wake from a real MUL producer and
    // release both live rows in one decision cycle.
    fill_nops();
    imem[0] = 32'h0070_0113; // addi x2,x0,7
    imem[1] = 32'h0030_0193; // addi x3,x0,3
    imem[2] = 32'h0231_00b3; // mul  x1,x2,x3
    imem[3] = 32'h0010_9863; // bne  x1,x1,+16 (correct)
    imem[4] = 32'h0010_9863; // bne  x1,x1,+16 (correct)
    // Depend on the MUL so this addi cannot occupy an ALU while the two
    // correct branches are waking; occupancy-only admission would otherwise
    // serialize the branches across a drain-idle and miss dual-correct.
    imem[5] = 32'hff50_8513; // addi x10,x1,-11 -> 10
    imem[6] = 32'h0015_0593; // addi x11,x10,1
    imem[7] = 32'h04d0_0f93; // addi x31,x0,77
    imem[8] = 32'h0000_006f; // jal x0,0
    reset_dut();
    wait_marker(32'd77, 1200);
    repeat (2) @(posedge clk);
    check("natural dual-correct decision entered", dual_correct_seen);
    check("natural two-bit release entered", release_two_seen);
    check("correct-only program has no recovery broadcast", recovery_broadcasts == 0);
    check("multiple real shape11 fires", shape11_fires >= 4);
    check("all checkpoints released", u_core.u_rename.checkpoint_valid_q[0] == 0 &&
                                      u_core.u_rename.checkpoint_valid_q[1] == 0);
    check_arch("mul result", 1, 32'd21);
    check_arch("post-release x10", 10, 32'd10);
    check_arch("dependent post-release x11", 11, 32'd11);
    check("free-list conserved after all-correct drain", free_count_now() == 32);
    scenarios++;

    // Program 2: delayed load wakes two taken branches one cycle before a
    // younger real REM reaches S_DONE. At the recovery broadcast, the two
    // branch holders consume both CDB lanes and the wrong-path REM holder must
    // stay bit-exact; it drains later and is rejected by generation/age state.
    // (REM, not MUL: the count-down alignment below needs the DIV family's
    // 32-cycle FSM now that the multiplier is 2-stage; the between-checkpoint
    // addi x12 doubles as its nonzero divisor so the div-by-zero fast path
    // cannot skip the occupancy.)
    fill_nops();
    imem[0]  = 32'h0000_2083; // lw   x1,0(x0)
    imem[1]  = 32'h0200_9263; // bne  x1,x0,+36 -> 0x28 (older)
    imem[2]  = 32'h00c0_0613; // addi x12,x0,12 (between checkpoints)
    imem[3]  = 32'h0e00_9a63; // bne  x1,x0,+244 -> 0x100 (younger)
    imem[4]  = 32'h02c0_62b3; // rem  x5,x0,x12 (wrong path)
    imem[5]  = 32'h00d0_0693; // addi x13,x0,13
    imem[6]  = 32'h0420_0313; // addi x6,x0,66
    imem[7]  = 32'h04d0_0393; // addi x7,x0,77
    imem[8]  = 32'h0580_0413; // addi x8,x0,88
    imem[9]  = 32'h0630_0493; // addi x9,x0,99
    imem[10] = 32'h06f0_0513; // target A: addi x10,x0,111
    imem[11] = 32'h02a5_0733; // mul x14,x10,x10 (post-recovery refill)
    imem[12] = 32'h05b0_0f93; // addi x31,x0,91
    imem[64] = 32'h0de0_0513; // target B: addi x10,x0,222
    imem[65] = 32'h05c0_0f93; // addi x31,x0,92
    reset_dut();

    // Wait until both nested checkpoints and their strict allocation-list
    // relation are established before releasing the load response.
    wait_i = 0;
    while (!(u_core.u_rename.checkpoint_valid_q[0] &&
             u_core.u_rename.checkpoint_valid_q[1] &&
             u_core.muldiv_busy)) begin
      @(negedge clk);
      wait_i++;
      if (wait_i > 200) $fatal(1, "nested checkpoints not reached");
    end
    check("younger checkpoint records older ancestry",
          u_core.u_rename.checkpoint_older_mask_q[1][0]);
    check("nesting subset holds independently",
          (u_core.u_rename.checkpoint_alloc_list_q[1] &
           ~u_core.u_rename.checkpoint_alloc_list_q[0]) == 0);
    check("nesting is strict due between-branch allocation",
          (u_core.u_rename.checkpoint_alloc_list_q[0] &
           ~u_core.u_rename.checkpoint_alloc_list_q[1]) != 0);

    // Accepted non-ALU wakeup adds one edge to load->branch readiness.
    // Release the external response one countdown earlier (4 -> 5), keeping
    // the SAME three-holder/recovery state and every oracle below unchanged.
    respond_when_mul_count(32'd1, 5);

    wait_i = 0;
    while (!u_core.branch_recover_req) begin
      @(posedge clk); #1;
      wait_i++;
      if (wait_i > 20) $fatal(1, "dual recovery did not latch");
    end
    #1;
    check("dual taken decision observed", dual_recover_seen);
    check("oldest recovery target selected", selected_target_seen == 32'h0000_0028);
    check("three real holders coexist at broadcast",
          u_core.alu0_complete.valid && u_core.alu1_complete.valid &&
          u_core.muldiv_complete.valid);
    check("branch holders consume both CDB lanes",
          (u_core.cdb_lane_select[0] == 5'b00001) &&
          (u_core.cdb_lane_select[1] == 5'b00010));
    check("wrong-path S_DONE holder is not granted yet", !u_core.cdb_grant_muldiv);
    check("release pulse is zero on broadcast", u_core.checkpoint_release_mask == 0);
    check("issue and dispatch blocked on broadcast",
          (u_core.issue_fire == 0) && !u_core.bundle_fire);
    check("shape11 is presented but excluded on broadcast", dec_slot_valid == 2'b11);
    held_mul = u_core.muldiv_complete;
    ghost_pdst = held_mul.pdst;
    ghost_old_data = u_core.u_prf.regs_q[ghost_pdst];
    $display("TRACE N2 broadcast t=%0t head=%0d target=%08h release=%b select={%b,%b} holders=%b shape=%b",
             $time, u_core.rob_head_idx, selected_target_seen,
             u_core.checkpoint_release_mask, u_core.cdb_lane_select[0],
             u_core.cdb_lane_select[1], u_core.holder_valid, dec_slot_valid);

    @(posedge clk); #1; // recovery applies; branches drain, MUL remains
    check("S_DONE holder survives recovery bit-exact",
          u_core.muldiv_complete === held_mul);
    check("S_DONE unit kill is distinct from holder hold", !u_core.muldiv_kill);
    check("mixed branch writeback verdict is independent", u_core.rob_wb_accept == 2'b01);
    check("mul holder becomes next real grant", u_core.cdb_grant_muldiv);
    $display("TRACE N2 apply t=%0t wb_accept=%b mul_held=%0b mul_grant=%0b",
             $time, u_core.rob_wb_accept, u_core.muldiv_complete.valid,
             u_core.cdb_grant_muldiv);

    @(posedge clk); #1; // held MUL reaches registered CDB after ROB rollback
    check("ghost MUL transported after recovery", u_core.cdb_q[0].valid &&
          (u_core.cdb_q[0].rob_idx == held_mul.rob_idx) &&
          (u_core.cdb_q[0].rob_seq == held_mul.rob_seq));
    check("ghost MUL writeback rejected", u_core.rob_wb_accept == 2'b00);
    check("ghost MUL cannot alter PRF data", u_core.u_prf.regs_q[ghost_pdst] == ghost_old_data);
    check("ghost destination remains not-ready at rejection",
          !u_core.u_prf.ready_q[ghost_pdst]);
    $display("TRACE N2 ghost t=%0t idx=%0d seq=%0d wb_accept=%b pdst=%0d data=%08h ready=%0b",
             $time, held_mul.rob_idx, held_mul.rob_seq, u_core.rob_wb_accept,
             ghost_pdst, u_core.u_prf.regs_q[ghost_pdst],
             u_core.u_prf.ready_q[ghost_pdst]);

    wait_marker(32'd91, 1600);
    repeat (2) @(posedge clk);
    check("one registered recovery broadcast", recovery_broadcasts == 1);
    check("shape11 broadcast variant observed", broadcast_shape11_seen);
    check("mixed survivor/victim writeback observed", mixed_wb_seen);
    check("post-recovery mul refill committed", refill_commit_seen);
    check_arch("oldest recovery target won", 10, 32'd111);
    check_arch("refill result", 14, 32'd12321);
    check_arch("wrong-path mul did not commit", 5, 32'd0);
    check_arch("between-checkpoint wrong path did not commit", 12, 32'd0);
    check_arch("younger wrong path did not commit", 13, 32'd0);
    check("all checkpoints cleared by covering recovery",
          !(u_core.u_rename.checkpoint_valid_q[0] ||
            u_core.u_rename.checkpoint_valid_q[1] ||
            u_core.u_rename.checkpoint_valid_q[2] ||
            u_core.u_rename.checkpoint_valid_q[3]));
    check("free-list conserved after recovery/refill", free_count_now() == 32);
    $display("TRACE N2 committed target_x10=%0d refill_x14=%0d wrong_x5=%0d free=%0d",
             u_core.u_prf.regs_q[u_core.u_rename.committed_map_q[10]],
             u_core.u_prf.regs_q[u_core.u_rename.committed_map_q[14]],
             u_core.u_prf.regs_q[u_core.u_rename.committed_map_q[5]],
             free_count_now());
    scenarios++;

    // Program 3: a delayed load aligns a real CSR completion with a real REM
    // completion. REM (older) takes lane 0, CSR takes lane 1; the lane-1 CSR
    // metadata reaches the ROB and commits before a read-only CSR interleave.
    // (REM for the same count-down-alignment reason as program 2; the leading
    // addi x8 supplies its nonzero divisor, shifting the body by one word --
    // safe, this program is straight-line with no PC-relative encodings.)
    fill_nops();
    imem[0] = 32'h0090_0413; // addi x8,x0,9 (divisor for the REM)
    imem[1] = 32'h0000_2083; // lw x1,0(x0)
    imem[2] = 32'h0280_6133; // rem x2,x0,x8
    imem[3] = 32'h3410_91f3; // csrrw x3,mepc,x1
    imem[4] = 32'h0011_8213; // addi x4,x3,1
    imem[5] = 32'h3410_22f3; // csrrs x5,mepc,x0 (read-only)
    imem[6] = 32'h0012_8313; // addi x6,x5,1
    imem[7] = 32'h05d0_0f93; // addi x31,x0,93
    reset_dut();
    // Same one-edge load-wakeup alignment adjustment as program 2.
    respond_when_mul_count(32'h0000_0055, 5);

    wait_i = 0;
    while (!(u_core.muldiv_complete.valid && u_core.alu0_complete.valid)) begin
      @(posedge clk); #1;
      wait_i++;
      if (wait_i > 20) $fatal(1, "CSR/MUL natural alignment failed");
    end
    check("natural MUL/CSR holders coexist",
          u_core.muldiv_complete.valid && u_core.alu0_complete.valid);
    check("age order puts MUL lane0 CSR lane1",
          (u_core.cdb_lane_select[0] == 5'b00100) &&
          (u_core.cdb_lane_select[1] == 5'b00001));
    $display("TRACE N3 select t=%0t select={%b,%b} holders=%b",
             $time, u_core.cdb_lane_select[0], u_core.cdb_lane_select[1],
             u_core.holder_valid);
    @(posedge clk); #1;
    check("CSR metadata transported on lane1",
          u_core.cdb_q[1].valid && u_core.cdb_q[1].csr_we &&
          (u_core.cdb_q[1].csr_wdata == 32'h0000_0055));
    check("both natural CDB lanes independently accepted",
          u_core.rob_wb_accept == 2'b11);
    $display("TRACE N3 cdb t=%0t lanes_valid=%b wb_accept=%b lane1_csr=%0b wdata=%08h",
             $time, {u_core.cdb_q[1].valid, u_core.cdb_q[0].valid},
             u_core.rob_wb_accept, u_core.cdb_q[1].csr_we,
             u_core.cdb_q[1].csr_wdata);

    wait_marker(32'd93, 1200);
    repeat (2) @(posedge clk);
    check("CSR lane1 event observed", csr_lane1_seen);
    check("CSR serialization held younger fetch", csr_stall_cycles > 0);
    check("architectural mepc updated", u_core.u_csr_file.mepc_q == 32'h55);
    check_arch("csrrw returns old mepc", 3, 32'd0);
    check_arch("dependent after csrrw", 4, 32'd1);
    check_arch("csrrs reads new mepc", 5, 32'h55);
    check_arch("dependent after csrrs", 6, 32'h56);
    check("free-list conserved after CSR interleave", free_count_now() == 32);
    $display("TRACE N3 committed mepc=%08h x3=%0d x4=%0d x5=%0d x6=%0d free=%0d",
             u_core.u_csr_file.mepc_q,
             u_core.u_prf.regs_q[u_core.u_rename.committed_map_q[3]],
             u_core.u_prf.regs_q[u_core.u_rename.committed_map_q[4]],
             u_core.u_prf.regs_q[u_core.u_rename.committed_map_q[5]],
             u_core.u_prf.regs_q[u_core.u_rename.committed_map_q[6]],
             free_count_now());
    scenarios++;

    if (scenarios != 3)
      $fatal(1, "NATURAL scenario count got=%0d exp=3", scenarios);
    $display("[tb_rv32i_ss_core_t36_natural] PASS checks=%0d (scenarios=%0d)", checks, scenarios);
    $finish;
  end
endmodule
