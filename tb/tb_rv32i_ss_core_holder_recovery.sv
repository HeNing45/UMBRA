`timescale 1ns/1ps

// tb_rv32i_ss_core_holder_recovery -- the REAL wrong-path-holder recovery
// test (program-driven; no forced CDB registers).
//
// Trajectory proven here, all through real machine flow:
//   a wrong-path LOAD completes into the REAL LQ mailbox (dmem response
//   timed by the TB) -> the mailbox loses CDB arbitration to the OLDER
//   recovering branch's own completion -> the packet is observed bit-exact
//   in the holder ON the recovery broadcast -> a REAL cdb_grant_lq drains it
//   either on that cycle or later -> the registered CDB transports it ->
//   wb_accept = 0 and ROB done / PRF data / ready bit are ALL unchanged.
//
// This battery deliberately targets the LQ mailbox because its fill time rides
// the DMEM response, which the TB controls. The corresponding wrong-path
// muldiv S_DONE flow-and-reject trajectory is covered by
// tb_rv32i_ss_core_stale_wb.sv.
//
// Method: the branch's resolve cycle is deterministic (it waits on a real
// 32-cycle REM, independent of the load), so iteration 0 measures the
// broadcast cycle B0 with an immediate response; the sweep then lands the
// response across the configured offset sweep and requires the holder-at-broadcast
// observation on at least one iteration (provably entered), with ZERO
// side-effect leakage on every iteration whose completion had not been
// legally accepted before the broadcast.
//
// Program per iteration (slot-0 dispatch beats):
//   rob0: addi x1, x0, 0x100      (load base)
//   rob1: addi x6, x0, 6
//   rob2: addi x7, x0, 7
//   rob3: rem  x3, x6, x7          (32-cycle producer; 6 % 7 = 6, keeping
//                                   the branch unresolved while the load launches)
//   rob4: bne  x3, x0, +0x40       (waits on x3; 6 != 0 -> TAKEN ->
//                                   mispredict -> recovery; checkpoint 0)
//   rob5: lw   x9, 0(x1)           (WRONG PATH; the holder under test)

module tb_rv32i_ss_core_holder_recovery;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;

  localparam time CLK_PERIOD = 10ns;

  logic clk;
  logic rst_n;

  logic           decoded_valid;
  logic           decoded_ready;
  word_t [1:0]          decoded_pc;
  word_t [1:0]          decoded_instr;
  arch_reg_t [1:0]      decoded_rs1;
  arch_reg_t [1:0]      decoded_rs2;
  arch_reg_t [1:0]      decoded_rd;
  logic [1:0]           decoded_rd_we;
  logic [1:0]           decoded_needs_checkpoint;
  ooo_op_class_e [1:0]  decoded_op_class;
  alu_op_e [1:0]        decoded_alu_op;
  br_type_e [1:0]       decoded_branch_op;
  ooo_src_sel_e [1:0]   decoded_src1_sel;
  ooo_src_sel_e [1:0]   decoded_src2_sel;
  word_t [1:0]          decoded_imm;
  ooo_fu_class_e [1:0]  decoded_fu_class;
  muldiv_op_e [1:0]     decoded_muldiv_op;
  decoded_trap_t  decoded_trap;
  logic [1:0]           decoded_is_load;
  logic [1:0]           decoded_is_store;
  mem_size_e [1:0]      decoded_mem_size;
  logic [1:0]           decoded_mem_unsigned;
  logic           redirect_valid;
  word_t          redirect_target;
  logic [1:0]           commit_fire;
  commit_order_t  commit_order;
  word_t [1:0]          commit_pc;
  word_t [1:0]          commit_inst;
  arch_reg_t [1:0]      commit_rd;
  logic [1:0]           commit_rd_wen;
  word_t [1:0]          commit_wdata;

  logic  dmem_valid, dmem_we;
  logic [3:0] dmem_be;
  word_t dmem_addr, dmem_wdata;
  logic  dmem_rvalid;
  word_t dmem_rdata;

  int errors = 0;
  int checks = 0;

  rv32i_ss_core dut (
    .clk                      (clk),
    .rst_n                    (rst_n),
    .decoded_valid            (decoded_valid),
    .decoded_slot_valid       ({1'b0, decoded_valid}),
    .decoded_ready            (decoded_ready),
    .decoded_pc               (decoded_pc),
    .decoded_instr            (decoded_instr),
    .decoded_rs1              (decoded_rs1),
    .decoded_rs2              (decoded_rs2),
    .decoded_rd               (decoded_rd),
    .decoded_rd_we            (decoded_rd_we),
    .decoded_needs_checkpoint (decoded_needs_checkpoint),
    .decoded_op_class         (decoded_op_class),
    .decoded_alu_op           (decoded_alu_op),
    .decoded_branch_op        (decoded_branch_op),
    .decoded_src1_sel         (decoded_src1_sel),
    .decoded_src2_sel         (decoded_src2_sel),
    .decoded_imm              (decoded_imm),
    .decoded_fu_class         (decoded_fu_class),
    .decoded_muldiv_op        (decoded_muldiv_op),
    .decoded_trap             (decoded_trap),
    .decoded_is_load          (decoded_is_load),
    .decoded_is_store         (decoded_is_store),
    .decoded_mem_size         (decoded_mem_size),
    .decoded_mem_unsigned     (decoded_mem_unsigned),
    .decoded_pred_taken('0),  // tie-off: static not-taken prediction
    .decoded_pred_target('0),
    .decoded_csr_op           ('0),
    .decoded_csr_addr         ('0),
    .decoded_csr_zimm         ('0),
    .redirect_valid           (redirect_valid),
    .redirect_target          (redirect_target),
    .commit_fire              (commit_fire),
    .commit_order             (commit_order),
    .commit_pc                (commit_pc),
    .commit_inst              (commit_inst),
    .commit_rd                (commit_rd),
    .commit_rd_wen            (commit_rd_wen),
    .commit_wdata             (commit_wdata),
    .dmem_valid               (dmem_valid),
    .dmem_we                  (dmem_we),
    .dmem_be                  (dmem_be),
    .dmem_addr                (dmem_addr),
    .dmem_wdata               (dmem_wdata),
    .dmem_ready               (1'b1),
    .dmem_rvalid              (dmem_rvalid),
    .dmem_rdata               (dmem_rdata)
  );

  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;

  initial begin
    repeat (4000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_holder_recovery exceeded 4000 cycles");
  end

  `define ROB dut.u_rob
  `define PRF dut.u_prf
  `define LSQ dut.u_lsq

  function automatic arch_reg_t areg(input int v);
    areg = arch_reg_t'(v);
  endfunction

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
      $error("[%s] got=%h exp=%h", name, got, exp);
      errors++;
    end
  endtask

  task automatic clear_inputs();
    decoded_valid            = 1'b0;
    decoded_pc               = '0;
    decoded_instr            = '0;
    decoded_rs1              = '0;
    decoded_rs2              = '0;
    decoded_rd               = '0;
    decoded_rd_we            = 1'b0;
    decoded_needs_checkpoint = 1'b0;
    decoded_op_class         = OOO_OP_ALU;
    decoded_alu_op           = fyp_cpu_pkg::ALU_ADD;
    decoded_branch_op        = rv32i_pipeline_pkg::BR_NONE;
    decoded_src1_sel         = OOO_SRC_REG;
    decoded_src2_sel         = OOO_SRC_REG;
    decoded_imm              = '0;
    decoded_fu_class         = OOO_FU_ALU;
    decoded_muldiv_op        = rv32i_pipeline_pkg::MD_MUL;
    decoded_trap             = '0;
    decoded_is_load          = 1'b0;
    decoded_is_store         = 1'b0;
    decoded_mem_size         = fyp_cpu_pkg::MEM_W;
    decoded_mem_unsigned     = 1'b0;
  endtask

  // dmem read-response control. A request landing at or after the release
  // cycle is answered same-cycle (the combinational form); a request
  // landing earlier goes PENDING and is answered exactly when the release
  // cycle arrives (dmem_valid is long gone by then — the LSQ pairs the
  // response with the outstanding FIFO).
  int   cyc;
  int   release_cyc;
  logic pend_q;
  always @(posedge clk) begin
    if (rst_n) cyc <= cyc + 1;
  end
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pend_q <= 1'b0;
    end else if (dmem_valid && !dmem_we && (cyc < release_cyc)) begin
      pend_q <= 1'b1;
    end else if (pend_q && (cyc >= release_cyc)) begin
      pend_q <= 1'b0;
    end
  end
  always_comb begin
    dmem_rvalid = 1'b0;
    dmem_rdata  = '0;
    if ((dmem_valid && !dmem_we && (cyc >= release_cyc)) ||
        (pend_q && (cyc >= release_cyc))) begin
      dmem_rvalid = 1'b1;
      dmem_rdata  = 32'hFEED_FACE;
    end
  end

  task automatic reset_dut();
    clear_inputs();
    rst_n = 1'b0;
    cyc   = 0;
    repeat (3) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);
  endtask

  // One slot-0 dispatch beat.
  task automatic disp(input word_t pc,
                      input arch_reg_t rs1, input arch_reg_t rs2,
                      input arch_reg_t rd, input logic rd_we,
                      input ooo_op_class_e opc, input ooo_fu_class_e fuc,
                      input br_type_e brop, input muldiv_op_e mdop,
                      input ooo_src_sel_e s1, input ooo_src_sel_e s2,
                      input word_t imm,
                      input logic is_ld, input logic ckpt);
    @(negedge clk);
    decoded_valid               = 1'b1;
    decoded_pc[0]               = pc;
    decoded_rs1[0]              = rs1;
    decoded_rs2[0]              = rs2;
    decoded_rd[0]               = rd;
    decoded_rd_we[0]            = rd_we;
    decoded_needs_checkpoint[0] = ckpt;
    decoded_op_class[0]         = opc;
    decoded_fu_class[0]         = fuc;
    decoded_branch_op[0]        = brop;
    decoded_muldiv_op[0]        = mdop;
    decoded_src1_sel[0]         = s1;
    decoded_src2_sel[0]         = s2;
    decoded_imm[0]              = imm;
    decoded_is_load[0]          = is_ld;
    decoded_mem_size[0]         = fyp_cpu_pkg::MEM_W;
    #1;
    while (decoded_ready !== 1'b1) @(negedge clk);
    @(posedge clk);
    @(negedge clk);
    decoded_valid = 1'b0;
    decoded_is_load[0] = 1'b0;
    decoded_needs_checkpoint[0] = 1'b0;
  endtask

  task automatic run_program();
    disp(32'h0000, areg(0), areg(0), areg(1), 1'b1, OOO_OP_ALU, OOO_FU_ALU,
         rv32i_pipeline_pkg::BR_NONE, rv32i_pipeline_pkg::MD_MUL,
         OOO_SRC_REG, OOO_SRC_IMM, 32'h100, 1'b0, 1'b0);
    disp(32'h0004, areg(0), areg(0), areg(6), 1'b1, OOO_OP_ALU, OOO_FU_ALU,
         rv32i_pipeline_pkg::BR_NONE, rv32i_pipeline_pkg::MD_MUL,
         OOO_SRC_REG, OOO_SRC_IMM, 32'd6, 1'b0, 1'b0);
    disp(32'h0008, areg(0), areg(0), areg(7), 1'b1, OOO_OP_ALU, OOO_FU_ALU,
         rv32i_pipeline_pkg::BR_NONE, rv32i_pipeline_pkg::MD_MUL,
         OOO_SRC_REG, OOO_SRC_IMM, 32'd7, 1'b0, 1'b0);
    disp(32'h000c, areg(6), areg(7), areg(3), 1'b1, OOO_OP_ALU, OOO_FU_MULDIV,
         rv32i_pipeline_pkg::BR_NONE, rv32i_pipeline_pkg::MD_REM,
         OOO_SRC_REG, OOO_SRC_REG, '0, 1'b0, 1'b0);
    disp(32'h0010, areg(3), areg(0), areg(0), 1'b0, OOO_OP_BRANCH, OOO_FU_ALU,
         rv32i_pipeline_pkg::BR_BNE, rv32i_pipeline_pkg::MD_MUL,
         OOO_SRC_REG, OOO_SRC_REG, 32'h40, 1'b0, 1'b1);
    disp(32'h0014, areg(1), areg(0), areg(9), 1'b1, OOO_OP_ALU, OOO_FU_LSU,
         rv32i_pipeline_pkg::BR_NONE, rv32i_pipeline_pkg::MD_MUL,
         OOO_SRC_REG, OOO_SRC_IMM, '0, 1'b1, 1'b0);
    clear_inputs();
  endtask

  // Per-iteration observation state.
  phys_reg_t          pd_victim;
  word_t              prf_before;
  logic               ready_before;
  bit                 done_at_bcast;
  bit                 held_across_bcast;
  completion_packet_t held_pkt, pkt_after;
  bit                 grant_after_seen;
  bit                 reject_seen;
  bit                 victim_early_grant;  // victim beat in transit at
                                           // B or B+1 = granted at B-1/B,
                                           // while its ROB row was still
                                           // pre-rollback (seq-live)
  int                 bcast_cyc;
  int                 wait_i;

  // Sweep bookkeeping.
  int b0;
  int t;
  int total_held_iters;
  int total_rejected_early_grant_iters;

  task automatic run_iteration(input int rel, output int bcast_out);
    reset_dut();
    release_cyc = rel;
    run_program();

    // The victim load sits at ROB index 5.
    pd_victim    = `ROB.pdst_q[5];
    prf_before   = `PRF.regs_q[pd_victim];
    ready_before = `PRF.ready_q[pd_victim];
    check_bit("victim pdst starts not-ready", ready_before, 1'b0);

    held_across_bcast = 1'b0;
    grant_after_seen  = 1'b0;
    reject_seen       = 1'b0;
    victim_early_grant = 1'b0;
    done_at_bcast     = 1'b0;
    bcast_out         = -1;

    // Watch until the recovery broadcast, then a drain window after it.
    for (wait_i = 0; wait_i < 200; wait_i++) begin
      @(negedge clk);
      if (dut.branch_recover_req === 1'b1 && bcast_out < 0) begin
        bcast_out = cyc;
        // A response released early enough is LEGALLY accepted before the
        // branch even resolves (normal OoO) — those iterations prove
        // nothing about the holder and skip the unchanged checks.
        done_at_bcast = `ROB.done_q[5];
        // THE observation: the real LQ mailbox holds the wrong-path
        // packet, bit-captured, on the broadcast cycle itself.
        if (`LSQ.lq_complete.valid === 1'b1) begin
          held_across_bcast = 1'b1;
          held_pkt = `LSQ.lq_complete;
        end
      end
      if (bcast_out >= 0 && held_across_bcast && !grant_after_seen &&
          dut.cdb_grant_lq === 1'b1 && cyc >= bcast_out) begin
        grant_after_seen = 1'b1;
        pkt_after = `LSQ.lq_complete;
      end
      if (bcast_out >= 0 && cyc <= bcast_out + 1 &&
          ((dut.cdb_q[0].valid === 1'b1 && dut.cdb_q[0].rob_idx === rob_idx_t'(5)) ||
           (dut.cdb_q[1].valid === 1'b1 && dut.cdb_q[1].rob_idx === rob_idx_t'(5)))) begin
        victim_early_grant = 1'b1;
      end
      if (grant_after_seen && !reject_seen) begin
        if (dut.cdb_q[0].valid === 1'b1 &&
            dut.cdb_q[0].rob_idx === rob_idx_t'(5)) begin
          reject_seen = 1'b1;
          check_bit("stale lane-0 beat rejected", dut.rob_wb_accept[0], 1'b0);
        end else if (dut.cdb_q[1].valid === 1'b1 &&
                     dut.cdb_q[1].rob_idx === rob_idx_t'(5)) begin
          reject_seen = 1'b1;
          check_bit("stale lane-1 beat rejected", dut.rob_wb_accept[1], 1'b0);
        end
      end
      if (bcast_out >= 0 && cyc > bcast_out + 12) break;
    end

    check_bit("recovery broadcast observed", bcast_out >= 0, 1'b1);

    if (held_across_bcast) begin
      total_held_iters++;
      // Holder integrity: bit-identical between broadcast observation and
      // its grant, whether lane capacity drains it immediately or later.
      check_bit("held packet grant seen on/after broadcast", grant_after_seen, 1'b1);
      if (grant_after_seen) begin
        check_bit("held packet bit-identical through recovery grant",
                  pkt_after === held_pkt, 1'b1);
        check_bit("stale beat observed and rejected", reject_seen, 1'b1);
      end
    end

    // Side effects: for every iteration where the completion had NOT been
    // legally accepted before the broadcast, nothing may change after it.
    if (!done_at_bcast) begin
      check_bit("victim ROB entry not done", `ROB.done_q[5], 1'b0);
      check_word("victim PRF data unchanged", `PRF.regs_q[pd_victim], prf_before);
      // Accepted-wakeup invariant: even a seq-live grant at B-1/B cannot
      // set ready. The younger packet is rejected at acceptance, protecting
      // BOTH readiness and value. Pin entry into that former exception.
      if (victim_early_grant)
        total_rejected_early_grant_iters++;
      check_bit("victim ready bit never set", `PRF.ready_q[pd_victim], 1'b0);
    end

    // Liveness: the machine drains and the correct path commits (5 older
    // instructions: 3 addi + mul + the recovering branch itself).
    for (wait_i = 0; wait_i < 200 && `ROB.count_q != 0; wait_i++) @(posedge clk);
    check_word("correct path fully committed",
               word_t'(`ROB.commit_order_q), 32'd5);
  endtask

  initial begin
    $display("[tb_rv32i_ss_core_holder_recovery] starting");
    total_held_iters = 0;
    total_rejected_early_grant_iters = 0;

    // Iteration 0: immediate response measures the broadcast cycle B0.
    run_iteration(0, b0);
    check_bit("baseline broadcast cycle measured", b0 > 0, 1'b1);

    // Sweep the response landing across the broadcast window.
    // At least one offset in [B0-8 .. B0+2] must hold the real packet in
    // the real holder on the broadcast cycle.
    for (t = b0 - 8; t <= b0 + 2; t++) begin
      run_iteration(t, bcast_cyc);
    end

    // Provably entered: at least one sweep point held the real packet in
    // the real holder on the broadcast cycle.
    check_bit("holder-at-broadcast reached in the sweep",
              total_held_iters > 0, 1'b1);
    check_bit("live-at-grant but rejected-at-accept window entered",
              total_rejected_early_grant_iters > 0, 1'b1);
    $display("[tb_rv32i_ss_core_holder_recovery] held iterations = %0d (B0=%0d)",
             total_held_iters, b0);

    if (errors == 0)
      $display("[tb_rv32i_ss_core_holder_recovery] PASS checks=%0d", checks);
    else
      $display("[tb_rv32i_ss_core_holder_recovery] FAIL checks=%0d errors=%0d",
               checks, errors);
    $finish;
  end

endmodule
