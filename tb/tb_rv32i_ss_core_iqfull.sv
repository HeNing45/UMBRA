`timescale 1ns/1ps

// =============================================================================
// tb_rv32i_ss_core_iqfull.sv -- directed IQ-full backpressure test.
// =============================================================================
// Enters the iq_full dispatch stall and proves:
//   1. IQ occupancy NEVER exceeds OOO_IQ_DEPTH        (no overflow, invariant)
//   2. when the IQ is full, dispatch stalls           (decoded_ready == 0)
//   3. iq_full -- not rob_full -- is the binding stall (ROB still has room)
//   4. the stall is actually reached                  (IQ fills to depth)
//   5. it recovers                                     (IQ drains, dispatch resumes)
//   6. the core keeps retiring in order                (commits make progress)
//
// Stimulus: a back-to-back stream of DIVU uops. The single-occupancy 32-cycle
// MULDIV unit makes one-wide dispatch outpace execution without assuming any
// particular wakeup latency, so the 16-deep IQ fills before the 32-deep
// ROB/free-list boundary. No branches are used, isolating IQ backpressure from
// recovery and jump serialization.
// =============================================================================

module tb_rv32i_ss_core_iqfull;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;

  localparam time CLK_PERIOD = 10ns;

  logic           clk, rst_n;
  logic           decoded_valid;
  logic           decoded_ready;
  word_t [1:0]          decoded_pc;
  word_t [1:0]          decoded_instr;
  arch_reg_t [1:0] decoded_rs1, decoded_rs2, decoded_rd;
  logic [1:0]           decoded_rd_we;
  logic [1:0]           decoded_needs_checkpoint;
  ooo_op_class_e [1:0]  decoded_op_class;
  alu_op_e [1:0]        decoded_alu_op;
  br_type_e [1:0]       decoded_branch_op;
  ooo_src_sel_e [1:0] decoded_src1_sel, decoded_src2_sel;
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
  word_t [1:0]          commit_pc, commit_inst;
  arch_reg_t [1:0]      commit_rd;
  logic [1:0]           commit_rd_wen;
  word_t [1:0]          commit_wdata;

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
    .commit_wdata             (commit_wdata)
  );

  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;

  initial begin
    repeat (2000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_iqfull exceeded 2000 cycles");
  end

  `define IQ dut.u_iq

  function automatic arch_reg_t areg(input int v); areg = arch_reg_t'(v); endfunction

  // Live IQ occupancy (popcount of the entry valid bits).
  function automatic int iq_count();
    int n; n = 0;
    for (int i = 0; i < OOO_IQ_DEPTH; i++) if (`IQ.valid_q[i]) n++;
    iq_count = n;
  endfunction

  task automatic check_bit(input string name, input logic got, input logic exp);
    checks++;
    if (got !== exp) begin $error("[%s] got=%0b exp=%0b", name, got, exp); errors++; end
  endtask

  task automatic set_divu(input int k);
    decoded_valid            = 1'b1;
    decoded_pc               = word_t'(k << 2);
    decoded_instr            = '0;
    // drain-or-empty drains ALU uops at 1/cycle. The 32-cycle DIVU
    // occupancy is the structural blocker, so this remains valid across
    // registered, grant-time, and issue-driven wakeup implementations.
    decoded_rs1              = areg((k == 1) ? 31 : k - 1);
    decoded_rs2              = areg(0);
    decoded_rd               = areg(k);          // distinct dest, chained via rs1
    decoded_rd_we            = 1'b1;
    decoded_needs_checkpoint = 1'b0;
    decoded_op_class         = OOO_OP_ALU;
    decoded_alu_op           = fyp_cpu_pkg::ALU_ADD;
    decoded_branch_op        = rv32i_pipeline_pkg::BR_NONE;
    decoded_src1_sel         = OOO_SRC_REG;
    decoded_src2_sel         = OOO_SRC_IMM;       // operand 2 from imm -> prs2 don't-care
    decoded_imm              = word_t'(k | 1); // nonzero divisor: no fast path
    decoded_fu_class         = OOO_FU_MULDIV;
    decoded_muldiv_op        = rv32i_pipeline_pkg::MD_DIVU;
    decoded_trap             = '0;
    decoded_is_load          = 1'b0;
    decoded_is_store         = 1'b0;
    decoded_mem_size         = fyp_cpu_pkg::MEM_W;
    decoded_mem_unsigned     = 1'b0;
  endtask

  task automatic reset_dut();
    decoded_valid = 1'b0;
    set_divu(1); decoded_valid = 1'b0;
    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    @(negedge clk);
  endtask

  int  rdk;
  int  occ, robcnt;
  int  max_occ;
  int  commits;
  integer commit_slot;
  bit  saw_full, saw_rob_had_room, saw_drain_resume;
  bit  saw_atomic_snapshot;
  bit  rdy;
  bit  check_atomic_this_cycle;
  rob_idx_t  tail_before;
  rob_seq_t  next_seq_before;
  phys_reg_t spec_before;

  initial begin
    $display("[tb_rv32i_ss_core_iqfull] starting");
    reset_dut();

    max_occ = 0; commits = 0;
    saw_full = 1'b0; saw_rob_had_room = 1'b0; saw_drain_resume = 1'b0;
    saw_atomic_snapshot = 1'b0;

    rdk = 1;
    @(negedge clk);
    set_divu(rdk);                       // present the first uop

    for (int c = 0; c < 240; c++) begin
      @(negedge clk);
      #1;
      occ    = iq_count();
      robcnt = int'(dut.u_rob.count_q);
      rdy    = decoded_ready;
      check_atomic_this_cycle = 1'b0;

      // (1) invariant: never overflow the IQ
      if (occ > OOO_IQ_DEPTH) begin
        $error("IQ overflow: occ=%0d > depth=%0d", occ, OOO_IQ_DEPTH);
        errors++;
      end
      if (occ > max_occ) max_occ = occ;

      // (2)+(3): when the IQ is full and we are offering a uop, dispatch must
      // stall, and the ROB must still have room (iq_full is the binding stall).
      if (occ == OOO_IQ_DEPTH && decoded_valid) begin
        saw_full = 1'b1;
        if (rdy) begin
          $error("IQ full but decoded_ready high (dispatch not stalled)");
          errors++;
        end
        if (robcnt < OOO_ROB_DEPTH) saw_rob_had_room = 1'b1;

        // Stall atomicity: while decoded_valid is held at the
        // boundary and iq_full is the binding backpressure, the offered uop
        // must not partially allocate in rename/free-list/ROB. Commit/writeback
        // may still make progress, so only check allocation-owned state.
        if (!saw_atomic_snapshot && !rdy) begin
          tail_before = dut.u_rob.tail_q;
          next_seq_before = dut.u_rob.next_seq_q;
          spec_before = dut.u_rename.spec_map_q[decoded_rd];
          check_atomic_this_cycle = 1'b1;
        end
      end

      // (5): after being full, occupancy drops and dispatch fires again
      if (saw_full && occ < OOO_IQ_DEPTH && rdy) saw_drain_resume = 1'b1;

      // (6): forward progress, counted per retired instruction rather than
      // per clock containing at least one retirement.
      for (commit_slot = 0; commit_slot < 2; commit_slot++) begin
        if (commit_fire[commit_slot]) commits++;
      end

      @(posedge clk);                    // dispatch edge: consume the uop if ready
      #1;
      if (check_atomic_this_cycle) begin
        check_bit("iq_full stall leaves ROB tail unchanged",
                  dut.u_rob.tail_q == tail_before, 1'b1);
        check_bit("iq_full stall leaves ROB seq unchanged",
                  dut.u_rob.next_seq_q == next_seq_before, 1'b1);
        check_bit("iq_full stall leaves offered spec map unchanged",
                  dut.u_rename.spec_map_q[decoded_rd] == spec_before, 1'b1);
        saw_atomic_snapshot = 1'b1;
      end
      if (rdy) begin
        rdk = (rdk == 31) ? 1 : rdk + 1; // next independent dest
        set_divu(rdk);
      end
    end

    // ---- summary assertions ----
    check_bit("IQ fills to depth (stall reachable)",  (max_occ == OOO_IQ_DEPTH), 1'b1);
    check_bit("dispatch stalls when IQ full",          saw_full,                 1'b1);
    check_bit("iq_full binds before rob_full",         saw_rob_had_room,         1'b1);
    check_bit("IQ drains and dispatch resumes",        saw_drain_resume,         1'b1);
    check_bit("core keeps committing (forward progress)", (commits > 0),         1'b1);
    check_bit("iq_full atomicity snapshot taken",      saw_atomic_snapshot,      1'b1);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_core_iqfull] PASS checks=%0d (max_occ=%0d commits=%0d)",
               checks, max_occ, commits);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_core_iqfull] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
