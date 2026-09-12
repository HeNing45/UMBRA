`timescale 1ns/1ps

// tb_rv32i_ss_core_cdb_top2 -- top-2 CDB select battery.
//
// The arbiter ranks the five completion holders by ring age with the
// total key (age, then client position on the reachable ghost-tie) and
// emits two live lane selects. Each selected holder drains on the same edge,
// and the registered CDB lanes preserve the rank-0/rank-1 ordering.
//
// natural dual ALU completion (x15-park, wake together) -> lane 0
//       carries the ring-older holder, lane 1 the younger; expectations
//       self-calibrated from the holders' own rob_idx (no binding-order
//       assumption).
// live dual drain -- both selected holders clear together, both packets
//       appear on their respective CDB lanes, and both eventually commit.
// ghost-tie (force): two holders with EQUAL rob_idx, different
//       rob_seq (the reachable flow-and-reject + recovery-tail-reuse state).
//       Tie resolves to the lower client, one-hot per lane, no false fatal
//       from the lane-0 one-hot pin. A variant adds a strictly-older LQ client:
//       the tied clients fill lane 1 only.
// five-holder exactness spots (force): anti-position age order and
//       muldiv/agen lane identity on both lanes.
//
// Force mechanics: the vvp backend
// silently NO-OPS a force on a multi-bit struct member (single-bit member
// forces work), so holders are forced as WHOLE structs tracking static f_*
// packet variables. A forced holder may only be REMOVED after it has won one
// of the two lanes (held-sample 0): the holder-stability pins correctly treat
// any other held-packet change as a scrub -- hence the drain choreography.
//
// This TB proves the IN-CORE wiring: holder -> age index -> select bit ->
// grant signal, the tie arm on live holder state, and dual-lane drain.

module tb_rv32i_ss_core_cdb_top2;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;

  localparam time CLK_PERIOD = 10ns;

  logic clk;
  logic rst_n;

  logic                 decoded_valid;
  logic [1:0]           decoded_slot_valid;
  logic                 decoded_ready;
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
  decoded_trap_t        decoded_trap;
  logic [1:0]           decoded_is_load;
  logic [1:0]           decoded_is_store;
  mem_size_e [1:0]      decoded_mem_size;
  logic [1:0]           decoded_mem_unsigned;
  logic                 redirect_valid;
  word_t                redirect_target;
  logic [1:0]                 commit_fire;
  commit_order_t        commit_order;
  word_t [1:0]                commit_pc;
  word_t [1:0]                commit_inst;
  arch_reg_t [1:0]            commit_rd;
  logic [1:0]                 commit_rd_wen;
  word_t [1:0]                commit_wdata;

  int errors = 0;
  int checks = 0;

  rv32i_ss_core dut (
    .clk                      (clk),
    .rst_n                    (rst_n),
    .decoded_valid            (decoded_valid),
    .decoded_slot_valid       (decoded_slot_valid),
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
    repeat (1000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_cdb_top2 exceeded 1000 cycles");
  end

  `define PRF dut.u_prf
  `define ROB dut.u_rob

  int commits;
  integer commit_slot;
  always @(posedge clk) begin
    if (rst_n) begin
      for (commit_slot = 0; commit_slot < 2; commit_slot++) begin
        if (commit_fire[commit_slot]) commits = commits + 1;
      end
    end
  end

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

  task automatic check_sel(input string name, input logic [4:0] got,
                           input logic [4:0] exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got=%b exp=%b", name, got, exp);
      errors++;
    end
  endtask

  task automatic clear_inputs();
    decoded_valid            = 1'b0;
    decoded_slot_valid       = 2'b00;
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

  task automatic reset_dut();
    clear_inputs();
    commits = 0;
    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);
  endtask

  // Slot-0 dispatch of one ALU imm-add reading rs1 (park on x15).
  task automatic disp(input word_t pc, input arch_reg_t rs1,
                      input arch_reg_t rd, input word_t imm);
    @(negedge clk);
    decoded_valid       = 1'b1;
    decoded_slot_valid  = 2'b01;
    decoded_pc[0]       = pc;
    decoded_rs1[0]      = rs1;
    decoded_rs2[0]      = areg(0);
    decoded_rd[0]       = rd;
    decoded_rd_we[0]    = 1'b1;
    decoded_op_class[0] = OOO_OP_ALU;
    decoded_src1_sel[0] = OOO_SRC_REG;
    decoded_src2_sel[0] = OOO_SRC_IMM;
    decoded_imm[0]      = imm;
    decoded_fu_class[0] = OOO_FU_ALU;
    #1;
    while (decoded_ready !== 1'b1) @(negedge clk);
    @(posedge clk);
    @(negedge clk);
    decoded_valid      = 1'b0;
    decoded_slot_valid = 2'b00;
  endtask

  // Static forced-packet variables: `force a = f_x` tracks later f_x edits,
  // which the drain choreography uses (f_x.valid = 0 removes a client on a
  // granted-winner cycle). Static tasks: Icarus rejects force statements
  // referencing automatic variables (house rule, wb_accept TB).
  // lq_complete at core level is a module-OUTPUT-driven variable
  // (unforceable); its internal flop is forced instead (lsq: assign
  // lq_complete = the buffer head). The muldiv completion is a comb decode of
  // state_q and stays un-forced: its lane-0 wiring is covered by
  // tb_rv32i_ss_core_arbiter, its rank algebra by the scratch sweep, and
  // the live runtime pins police both transport lanes.
  completion_packet_t f_alu0, f_alu1, f_agen, f_lq;

  task automatic pkt_set(output completion_packet_t p,
                         input rob_idx_t idx, input rob_seq_t seq);
    p         = '0;
    p.valid   = 1'b1;
    p.rob_idx = idx;
    p.rob_seq = seq;
  endtask

  task force_alu0(input rob_idx_t idx, input rob_seq_t seq);
    pkt_set(f_alu0, idx, seq);
    force dut.alu0_complete = f_alu0;
  endtask
  task force_alu1(input rob_idx_t idx, input rob_seq_t seq);
    pkt_set(f_alu1, idx, seq);
    force dut.alu1_complete = f_alu1;
  endtask
  task force_agen(input rob_idx_t idx, input rob_seq_t seq);
    pkt_set(f_agen, idx, seq);
    force dut.agen_complete = f_agen;
  endtask
  // lq_complete is the assign-driven completion-buffer head, and
  // Icarus statically rejects a force on an assign-driven variable INSIDE a
  // task body (the inline form is accepted — the arbiter TB's pattern). The
  // force therefore lives in a macro expanded at the call sites; the packet
  // setup stays a task. Assign-driven is also the better force target: a
  // release re-drives the mux immediately, so there is no retention hazard
  // at all.
  task setup_lq_pkt(input rob_idx_t idx, input rob_seq_t seq);
    pkt_set(f_lq, idx, seq);
  endtask
`define T2_FORCE_LQ(idx_, seq_) \
  begin setup_lq_pkt((idx_), (seq_)); force dut.u_lsq.lq_complete = f_lq; end

  // Pin-safe removal: drop a client only while it is the granted lane-0
  // winner (held-sample 0 lets its packet change on the next edge). The
  // underlying flops were washed by the real grants during the force, so
  // the final release reads back 0 with no retention hazard.
  task automatic drop_alu0(); @(negedge clk); f_alu0.valid = 1'b0; endtask
  task automatic drop_alu1(); @(negedge clk); f_alu1.valid = 1'b0; endtask
  task automatic drop_agen(); @(negedge clk); f_agen.valid = 1'b0; endtask
  task automatic drop_lq();   @(negedge clk); f_lq.valid   = 1'b0; endtask

  task release_holders();
    release dut.alu0_complete;
    release dut.alu1_complete;
    release dut.agen_complete;
  endtask
`define T2_RELEASE_LQ release dut.u_lsq.lq_complete

  logic [4:0] sel0_snap, sel1_snap;
  task automatic snap_selects();
    sel0_snap = dut.cdb_lane_select[0];
    sel1_snap = dut.cdb_lane_select[1];
  endtask

  int wait_i;
  logic alu0_is_older;
  rob_idx_t head_snap;
  rob_idx_t alu0_idx_snap;
  rob_idx_t alu1_idx_snap;

  initial begin
    reset_dut();

    // ---------------- natural dual ALU completion ----------------
    // Park both ops on x15, wake together -> dual issue -> both ALU holders
    // load on the same edge.
    force `PRF.ready_q[15] = 1'b0;
    disp(32'h0000_1000, areg(15), areg(5), 32'd5);
    disp(32'h0000_1004, areg(15), areg(6), 32'd6);
    release `PRF.ready_q[15];
    force `PRF.ready_q[15] = 1'b1;

    wait_i = 0;
    while (!(dut.alu0_complete.valid && dut.alu1_complete.valid) &&
           (wait_i < 20)) begin
      @(negedge clk);
      wait_i++;
    end
    check_bit("s1.both_holders_valid",
              dut.alu0_complete.valid && dut.alu1_complete.valid, 1'b1);

    // Self-calibrate: which ALU holder carries the ring-older entry.
    alu0_idx_snap = dut.alu0_complete.rob_idx;
    alu1_idx_snap = dut.alu1_complete.rob_idx;
    alu0_is_older = ((dut.alu0_complete.rob_idx - `ROB.head_q) <
                     (dut.alu1_complete.rob_idx - `ROB.head_q));
    snap_selects();
    check_sel("s1.lane0_is_older_alu",
              sel0_snap, alu0_is_older ? 5'b00001 : 5'b00010);
    check_sel("s1.lane1_is_younger_alu",
              sel1_snap, alu0_is_older ? 5'b00010 : 5'b00001);
    check_bit("s1.alu0_granted_on_one_lane", dut.cdb_grant_alu0, 1'b1);
    check_bit("s1.alu1_granted_on_one_lane", dut.cdb_grant_alu1, 1'b1);

    // ---------------- live dual drain ----------------
    // Both selected holders drain together and retain rank order in cdb_q.
    @(posedge clk);
    @(negedge clk);
    check_bit("s2.older_holder_cleared",
              alu0_is_older ? dut.alu0_complete.valid
                            : dut.alu1_complete.valid, 1'b0);
    check_bit("s2.younger_holder_cleared",
              alu0_is_older ? dut.alu1_complete.valid
                            : dut.alu0_complete.valid, 1'b0);
    check_bit("s2.both_cdb_lanes_valid",
              dut.cdb_q[0].valid && dut.cdb_q[1].valid, 1'b1);
    check_bit("s2.lane0_carries_older",
              dut.cdb_q[0].rob_idx ==
                  (alu0_is_older ? alu0_idx_snap : alu1_idx_snap), 1'b1);
    check_bit("s2.lane1_carries_younger",
              dut.cdb_q[1].rob_idx ==
                  (alu0_is_older ? alu1_idx_snap : alu0_idx_snap), 1'b1);

    wait_i = 0;
    while ((commits < 2) && (wait_i < 40)) begin
      @(negedge clk);
      wait_i++;
    end
    check_bit("s2.machine_drained_two_commits", (commits == 2), 1'b1);

    // ---------------- ghost-tie (equal rob_idx, distinct rob_seq) ----
    // The reachable flow-and-reject state: a wrong-path ghost's ROB entry was
    // rewound and reused, so two holders carry the SAME rob_idx. The tie
    // arm must pick the lower client; the lane-0 one-hot pin must stay
    // silent (surviving posedges below proves it).
    @(negedge clk);
    head_snap = `ROB.head_q;
    force_alu0(head_snap + rob_idx_t'(11), 64'd101);
    force_alu1(head_snap + rob_idx_t'(11), 64'd202);
    @(negedge clk);
    // Readback guard: if the whole-struct force no-ops, fail here loudly.
    check_bit("s3.force_readback_alu0_valid", dut.alu0_complete.valid, 1'b1);
    check_bit("s3.force_readback_alu1_valid", dut.alu1_complete.valid, 1'b1);
    check_bit("s3.force_readback_idx_equal",
              (dut.alu0_complete.rob_idx == dut.alu1_complete.rob_idx), 1'b1);
    check_bit("s3.force_readback_seq_differs",
              (dut.alu0_complete.rob_seq != dut.alu1_complete.rob_seq), 1'b1);
    snap_selects();
    check_sel("s3a.tie_lane0_lower_client", sel0_snap, 5'b00001);
    check_sel("s3a.tie_lane1_other_client", sel1_snap, 5'b00010);
    @(posedge clk);          // lane-0 one-hot pin evaluates this edge
    @(negedge clk);

    // a strictly-older LQ client displaces the tied clients to lane 1.
    `T2_FORCE_LQ(head_snap + rob_idx_t'(3), 64'd303)
    @(negedge clk);
    check_bit("s3b.force_readback_lq_valid", dut.lq_complete.valid, 1'b1);
    snap_selects();
    check_sel("s3b.older_lq_takes_lane0", sel0_snap, 5'b10000);
    check_sel("s3b.tie_pair_lower_takes_lane1", sel1_snap, 5'b00001);
    // Drain pin-safe: LQ is the current winner -> drop it; the tied clients
    // re-promotes (alu0 wins) -> drop alu0; alu1 wins alone -> drop alu1.
    drop_lq();
    @(posedge clk);
    drop_alu0();
    @(posedge clk);
    drop_alu1();
    @(posedge clk);
    @(negedge clk);
    begin release_holders(); `T2_RELEASE_LQ; end
    @(negedge clk);
    check_bit("s3.holders_wash_after_release",
              dut.alu0_complete.valid | dut.alu1_complete.valid |
              dut.lq_complete.valid, 1'b0);

    // ---------------- multi-holder exactness spots ----------------
    // Anti-position order (highest client index carries the oldest age):
    // any static-priority regression flips these.
    @(negedge clk);
    head_snap = `ROB.head_q;
    force_alu0(head_snap + rob_idx_t'(25), 64'd1);
    force_alu1(head_snap + rob_idx_t'(20), 64'd2);
    force_agen(head_snap + rob_idx_t'(10), 64'd4);
    `T2_FORCE_LQ(head_snap + rob_idx_t'(5), 64'd5)
    @(negedge clk);
    snap_selects();
    check_sel("s4a.lane0_lq_oldest",   sel0_snap, 5'b10000);
    check_sel("s4a.lane1_agen_second", sel1_snap, 5'b01000);
    drop_lq();
    @(posedge clk);
    drop_agen();
    @(posedge clk);
    drop_alu1();
    @(posedge clk);
    drop_alu0();
    @(posedge clk);
    @(negedge clk);
    begin release_holders(); `T2_RELEASE_LQ; end
    @(negedge clk);

    // AGEN takes lane 0 with LQ on lane 1 (LQ's select[1] bit identity).
    force_agen(head_snap + rob_idx_t'(2), 64'd6);
    `T2_FORCE_LQ(head_snap + rob_idx_t'(4), 64'd7)
    @(negedge clk);
    snap_selects();
    check_sel("s4b.lane0_agen", sel0_snap, 5'b01000);
    check_sel("s4b.lane1_lq",   sel1_snap, 5'b10000);
    drop_agen();
    @(posedge clk);
    drop_lq();
    @(posedge clk);
    @(negedge clk);
    begin release_holders(); `T2_RELEASE_LQ; end
    @(negedge clk);
    check_bit("s4.holders_wash_after_release",
              dut.alu0_complete.valid | dut.alu1_complete.valid |
              dut.agen_complete.valid | dut.lq_complete.valid, 1'b0);

    if (errors == 0) begin
      $display("PASS checks=%0d", checks);
    end else begin
      $fatal(1, "tb_rv32i_ss_core_cdb_top2 FAILED with %0d error(s)", errors);
    end
    $finish;
  end

endmodule
