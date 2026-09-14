`timescale 1ns/1ps

// tb_rv32i_ss_core_wb_dual -- dual-writeback close battery.
//
// The arbiter already proved lane SELECTION; this battery proves the
// second SINK: two live transport lanes into dual ROB wb ports, per-lane
// independent acceptance, per-lane PRF data + ready effects, and lossless
// flow-and-reject holding when producers outnumber lanes.
//
// natural dual drain -- two same-bundle ALU uops complete in one
//       cycle, drains on both lanes, both results/ready bits land, 2 commits,
//       then the beat bus is QUIESCENT (no rebroadcast).
// mixed recovery verdict -- recovery-coincident cycle with a SURVIVOR
//       on one lane and a VICTIM on the other, BOTH orientations: the two
// lanes return different verdicts in the same cycle (per-lane).
// trap-flush rejection -- both lanes valid on the trap-flush cycle,
//       both rejected, no PRF effects.
// ghost-tie adjudication -- same rob_idx, different rob_seq on the two
//       lanes (real recovery + tail-reuse generation), BOTH orientations:
//       exactly the live-generation lane accepts, wherever it rides.
// natural lane-1 CSR -- an older in-flight REM completes the same
//       cycle as a CSR (self-calibrated), so the CSR packet rides lane 1;
//       its csr_we/csr_wdata metadata flows through the ROB to the 1-wide
//       commit CSR write (mepc readback).
// lane-1 trap capture -- a forced trap packet accepted on lane 1
//       lands trap metadata in the ROB; the head commit takes the trap
//       (mcause/mtval/mepc readback + full flush).
// three producers, two lanes -- REM + two ALU completions arrive in one
//       cycle (self-calibrated): two drain, the third is HELD with its packet
//       byte-stable, drains next cycle, all three results land, 3 commits.
//
// Mutation kill map (RED set, four families):
// rob.sv wb seq-match term deleted -> accepts pattern +
// dual-accept pin
// core lane-1 drain term dropped -> quiescence checks
// core lane-1 capture chain cross-wired -> pin via duplicate
//                                                    {idx,seq} on both lanes
// core PRF write gate cross-laned -> victim-side ready/data
//                                                    checks (accept[0] round)
//
// Force idiom: Icarus silently NO-OPS a force on a multi-bit member of a
// packed struct element, so beats are forced as WHOLE cdb_q[n] elements
// tracking static f_l0/f_l1 packet variables.
// A forced-valid beat present at a posedge TAKES EFFECT (release retention),
// so every force bracket accounts for exactly one apply edge.

module tb_rv32i_ss_core_wb_dual;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;

  localparam time CLK_PERIOD = 10ns;

  logic clk;
  logic rst_n;

  logic           decoded_valid;
  logic [1:0]     decoded_slot_valid;
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
  csr_op_e              decoded_csr_op;
  csr_addr_t            decoded_csr_addr;
  csr_zimm_t            decoded_csr_zimm;
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
    .decoded_csr_op           (decoded_csr_op),
    .decoded_csr_addr         (decoded_csr_addr),
    .decoded_csr_zimm         (decoded_csr_zimm),
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
    repeat (3000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_wb_dual exceeded 3000 cycles");
  end

  `define ROB  dut.u_rob
  `define RN   dut.u_rename
  `define PRF  dut.u_prf
  `define CSRF dut.u_csr_file

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

  task automatic check_accepts(input string name, input logic [1:0] got,
                               input logic [1:0] exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] rob_wb_accept got=%b exp=%b (lane0 cdb={v=%0b idx=%0d seq=%0d} lane1 cdb={v=%0b idx=%0d seq=%0d})",
             name, got, exp,
             dut.cdb_q[0].valid, dut.cdb_q[0].rob_idx, dut.cdb_q[0].rob_seq,
             dut.cdb_q[1].valid, dut.cdb_q[1].rob_idx, dut.cdb_q[1].rob_seq);
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
    decoded_rd_we            = '0;
    decoded_needs_checkpoint = '0;
    decoded_op_class[0]      = OOO_OP_ALU;
    decoded_op_class[1]      = OOO_OP_ALU;
    decoded_alu_op[0]        = fyp_cpu_pkg::ALU_ADD;
    decoded_alu_op[1]        = fyp_cpu_pkg::ALU_ADD;
    decoded_branch_op[0]     = rv32i_pipeline_pkg::BR_NONE;
    decoded_branch_op[1]     = rv32i_pipeline_pkg::BR_NONE;
    decoded_src1_sel[0]      = OOO_SRC_REG;
    decoded_src1_sel[1]      = OOO_SRC_REG;
    decoded_src2_sel[0]      = OOO_SRC_REG;
    decoded_src2_sel[1]      = OOO_SRC_REG;
    decoded_imm              = '0;
    decoded_fu_class[0]      = OOO_FU_ALU;
    decoded_fu_class[1]      = OOO_FU_ALU;
    decoded_muldiv_op[0]     = rv32i_pipeline_pkg::MD_MUL;
    decoded_muldiv_op[1]     = rv32i_pipeline_pkg::MD_MUL;
    decoded_csr_op           = rv32i_pipeline_pkg::CSR_NONE;
    decoded_csr_addr         = '0;
    decoded_csr_zimm         = '0;
    decoded_trap             = '0;
    decoded_is_load          = '0;
    decoded_is_store         = '0;
    decoded_mem_size[0]      = fyp_cpu_pkg::MEM_W;
    decoded_mem_size[1]      = fyp_cpu_pkg::MEM_W;
    decoded_mem_unsigned     = '0;
  endtask

  task automatic reset_dut();
    clear_inputs();
    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);
    // Park register: ops that read x1 (phys p1, never rewritten) stay
    // un-issued while ready_q[1] is forced low; force it high to wake them.
    force `PRF.ready_q[1] = 1'b0;
  endtask

  // Complete one dispatch handshake for whatever the caller staged.
  task automatic disp_go();
    decoded_valid = 1'b1;
    #1;
    while (decoded_ready !== 1'b1) @(negedge clk);
    @(posedge clk);
    @(negedge clk);
    decoded_valid      = 1'b0;
    decoded_slot_valid = 2'b00;
    decoded_csr_op     = rv32i_pipeline_pkg::CSR_NONE;
  endtask

  // Slot-0 ALU op: rd <- x1 + imm (x1 parked; result is 0 + imm).
  task automatic disp_alu_parked(input arch_reg_t rd, input word_t imm);
    @(negedge clk);
    clear_inputs();
    decoded_slot_valid  = 2'b01;
    decoded_pc[0]       = 32'h0000_1000;
    decoded_rs1[0]      = areg(1);
    decoded_rd[0]       = rd;
    decoded_rd_we[0]    = 1'b1;
    decoded_src2_sel[0] = OOO_SRC_IMM;
    decoded_imm[0]      = imm;
    disp_go();
  endtask

  // Slot-0 branch reading x1 (parked); allocates a checkpoint.
  task automatic disp_branch_parked();
    @(negedge clk);
    clear_inputs();
    decoded_slot_valid          = 2'b01;
    decoded_pc[0]               = 32'h0000_1000;
    decoded_rs1[0]              = areg(1);
    decoded_needs_checkpoint[0] = 1'b1;
    decoded_op_class[0]         = OOO_OP_BRANCH;
    decoded_branch_op[0]        = rv32i_pipeline_pkg::BR_BEQ;
    disp_go();
  endtask

  // Dual-ALU bundle, both parked on x1, distinct immediates.
  task automatic disp_alu_pair_parked(input arch_reg_t rd_a, input word_t imm_a,
                                      input arch_reg_t rd_b, input word_t imm_b);
    @(negedge clk);
    clear_inputs();
    decoded_slot_valid  = 2'b11;
    decoded_pc[0]       = 32'h0000_1000;
    decoded_pc[1]       = 32'h0000_1004;
    decoded_rs1[0]      = areg(1);
    decoded_rs1[1]      = areg(1);
    decoded_rd[0]       = rd_a;
    decoded_rd[1]       = rd_b;
    decoded_rd_we       = 2'b11;
    decoded_src2_sel[0] = OOO_SRC_IMM;
    decoded_src2_sel[1] = OOO_SRC_IMM;
    decoded_imm[0]      = imm_a;
    decoded_imm[1]      = imm_b;
    disp_go();
  endtask

  // Slot-0 MUL x0*x0 (sources always ready -> issues immediately).
  // Slow muldiv occupant: REM rd, x0, x7. The / two-pass alignment
  // requires the muldiv completion path LONGER than the ALU/CSR wake path,
  // which only the DIV family's 32-cycle count-down provides now that the
  // multiplier is 2-stage pipelined. The divisor value is deposited here so
  // every call site is covered after its own reset (p7 is never FF-written;
  // a zero divisor would take the fast path and defeat the occupancy).
  task automatic disp_rem_slow(input arch_reg_t rd);
    @(negedge clk);
    `PRF.regs_q[7] = 32'd7;
    clear_inputs();
    decoded_slot_valid   = 2'b01;
    decoded_pc[0]        = 32'h0000_1000;
    decoded_rs2[0]       = areg(7);
    decoded_rd[0]        = rd;
    decoded_rd_we[0]     = 1'b1;
    decoded_fu_class[0]  = OOO_FU_MULDIV;
    decoded_muldiv_op[0] = rv32i_pipeline_pkg::MD_REM;
    disp_go();
  endtask

  // Slot-0 CSRRW rd, mepc, x1 (x1 parked; solo bundle by).
  task automatic disp_csr_parked(input arch_reg_t rd);
    @(negedge clk);
    clear_inputs();
    decoded_slot_valid = 2'b01;
    decoded_pc[0]      = 32'h0000_1000;
    decoded_rs1[0]     = areg(1);
    decoded_rd[0]      = rd;
    decoded_rd_we[0]   = 1'b1;
    decoded_csr_op     = rv32i_pipeline_pkg::CSR_RW;
    decoded_csr_addr   = 12'h341;   // mepc: plain read/write CSR
    disp_go();
  endtask

  // ---- whole-element beat forcing (static vars; see header) ----
  completion_packet_t f_l0, f_l1;

  function automatic completion_packet_t mk_pkt(
      input rob_idx_t idx, input rob_seq_t seq, input phys_reg_t pd,
      input logic wen, input word_t res);
    completion_packet_t p;
    p            = '0;
    p.valid      = 1'b1;
    p.rob_idx    = idx;
    p.rob_seq    = seq;
    p.pdst       = pd;
    p.rd_wen     = wen;
    p.result     = res;
    mk_pkt       = p;
  endfunction

  function automatic completion_packet_t mk_trap_pkt(
      input rob_idx_t idx, input rob_seq_t seq,
      input word_t cause, input word_t tval);
    completion_packet_t p;
    p            = '0;
    p.valid      = 1'b1;
    p.rob_idx    = idx;
    p.rob_seq    = seq;
    p.trap_valid = 1'b1;
    p.trap_cause = cause;
    p.trap_tval  = tval;
    mk_trap_pkt  = p;
  endfunction

  task force_lane0();
    force dut.cdb_q[0] = f_l0;
  endtask
  task force_lane1();
    force dut.cdb_q[1] = f_l1;
  endtask
  task automatic release_lanes();
    release dut.cdb_q[0];
    release dut.cdb_q[1];
  endtask

  // Recovery-broadcast force bracket (wb_accept house idiom, incl. the
  // release-retention teardown).
  task recover_begin(input rob_idx_t br_idx);
    force dut.recover_q_valid   = 1'b1;
    force dut.recover_q_rob_idx = br_idx;
    force dut.recover_q_ckpt_id = ckpt_idx_t'(0);
    force dut.recover_q_target  = 32'h0000_2000;
  endtask
  task recover_end();
    // Retention hazard: a released force on a flop keeps the forced value
    // until the flop's next assignment; drive valid low for one edge first.
    force dut.recover_q_valid = 1'b0;
    release dut.recover_q_rob_idx;
    release dut.recover_q_ckpt_id;
    release dut.recover_q_target;
    release_lanes();
    @(posedge clk);
    @(negedge clk);
    release dut.recover_q_valid;
  endtask

  // Scenario-scope bookkeeping (module scope: static tasks + Icarus).
  phys_reg_t pd_a, pd_b, pd_mul, pd_csr, pd_surv, pd_vict;
  rob_seq_t  seq_a, seq_b, seq_surv, seq_vict, stale_seq, fresh_seq;
  phys_reg_t stale_pd, fresh_pd;
  word_t     old_surv, old_vict, old_ghost;
  commit_order_t co_before;
  int        m_lat, a_lat, wait_n, i;
  completion_packet_t held_snap;

  initial begin
    $display("[tb_rv32i_ss_core_wb_dual] starting");

    // =================== natural dual drain ===================
    reset_dut();
    disp_alu_pair_parked(areg(5), 32'h0000_0111, areg(6), 32'h0000_0222);
    #1;
    pd_a      = `ROB.pdst_q[0];
    pd_b      = `ROB.pdst_q[1];
    co_before = `ROB.commit_order_q;

    @(negedge clk);
    force `PRF.ready_q[1] = 1'b1;   // wake both together

    // Bounded wait: both ALU holders valid in the same cycle.
    wait_n = 0;
    while (!(dut.alu0_complete.valid && dut.alu1_complete.valid)) begin
      @(posedge clk); #1;
      wait_n++;
      if (wait_n > 10)
        $fatal(1, "D1: dual completion never landed (alu0.v=%0b alu1.v=%0b)",
               dut.alu0_complete.valid, dut.alu1_complete.valid);
    end
    check_bit ("D1 alu0 holds the older op", dut.alu0_complete.rob_idx == 0, 1'b1);
    check_bit ("D1 alu1 holds the younger op", dut.alu1_complete.rob_idx == 1, 1'b1);
    check_word("D1 lane0 selects alu0", word_t'(dut.cdb_lane_select[0]), word_t'(5'b00001));
    check_word("D1 lane1 selects alu1", word_t'(dut.cdb_lane_select[1]), word_t'(5'b00010));

    @(posedge clk); #1;   // capture edge: both packets on the bus
    check_bit ("D1 lane0 beat valid", dut.cdb_q[0].valid, 1'b1);
    check_bit ("D1 lane1 beat valid", dut.cdb_q[1].valid, 1'b1);
    check_word("D1 lane0 carries entry 0 result", dut.cdb_q[0].result, 32'h0000_0111);
    check_word("D1 lane1 carries entry 1 result", dut.cdb_q[1].result, 32'h0000_0222);
    check_accepts("D1 both lanes accepted", dut.rob_wb_accept, 2'b11);
    check_bit ("D1 holders drained on grant",
               dut.alu0_complete.valid | dut.alu1_complete.valid, 1'b0);

    @(posedge clk); #1;   // apply edge done: ROB/PRF effects
    check_bit ("D1 entry0 done", `ROB.done_q[0], 1'b1);
    check_bit ("D1 entry1 done", `ROB.done_q[1], 1'b1);
    check_word("D1 PRF lane0 data", `PRF.regs_q[pd_a], 32'h0000_0111);
    check_word("D1 PRF lane1 data", `PRF.regs_q[pd_b], 32'h0000_0222);
    check_bit ("D1 PRF lane0 ready", `PRF.ready_q[pd_a], 1'b1);
    check_bit ("D1 PRF lane1 ready", `PRF.ready_q[pd_b], 1'b1);
    // Quiescence: nothing may rebroadcast a drained completion.
    check_bit ("D1 beat bus quiescent (lane0)", dut.cdb_q[0].valid, 1'b0);
    check_bit ("D1 beat bus quiescent (lane1)", dut.cdb_q[1].valid, 1'b0);
    check_accepts("D1 no residual accept", dut.rob_wb_accept, 2'b00);

    wait_n = 0;
    while (`ROB.commit_order_q < (co_before + 64'd2)) begin
      @(posedge clk); #1;
      wait_n++;
      if (wait_n > 8)
        $fatal(1, "D1: expected 2 commits, commit_order=%0d (started %0d)",
               `ROB.commit_order_q, co_before);
    end
    check_bit("D1 both retired", 1'b1, 1'b1);

    // =================== mixed recovery verdict ===================
    // Layout: entry0 = survivor ALU, entry1 = branch (ckpt 0), entry2 =
    // victim ALU. Recovery at idx1 -> recover_age 1: age(0)=0 accept,
    // age(2)=2 reject -- in the SAME cycle, one verdict per lane.
    // Orientation A: victim on lane 0, survivor on lane 1.
    reset_dut();
    disp_alu_parked(areg(5), 32'h0000_0500);
    disp_branch_parked();
    disp_alu_parked(areg(6), 32'h0000_0600);
    #1;
    check_bit("D2A checkpoint 0 live", `RN.checkpoint_valid_q[0], 1'b1);
    pd_surv  = `ROB.pdst_q[0];  seq_surv = `ROB.seq_q[0];
    pd_vict  = `ROB.pdst_q[2];  seq_vict = `ROB.seq_q[2];
    old_surv = `PRF.regs_q[pd_surv];
    old_vict = `PRF.regs_q[pd_vict];

    @(negedge clk);
    recover_begin(rob_idx_t'(1));
    f_l0 = mk_pkt(rob_idx_t'(2), seq_vict, pd_vict, 1'b1, 32'hBAD0_00A0);
    f_l1 = mk_pkt(rob_idx_t'(0), seq_surv, pd_surv, 1'b1, 32'hCAFE_00A1);
    force_lane0();
    force_lane1();
    #1;
    check_accepts("D2A mixed verdict (victim@0 survivor@1)", dut.rob_wb_accept, 2'b10);
    @(posedge clk);
    @(negedge clk);
    recover_end();
    #1;
    check_bit ("D2A victim entry dead",      `ROB.valid_q[2], 1'b0);
    check_word("D2A victim PRF unchanged",   `PRF.regs_q[pd_vict], old_vict);
    check_bit ("D2A victim ready not set",   `PRF.ready_q[pd_vict], 1'b0);
    check_word("D2A survivor PRF written",   `PRF.regs_q[pd_surv], 32'hCAFE_00A1);
    check_bit ("D2A survivor ready set",     `PRF.ready_q[pd_surv], 1'b1);

    // Orientation B: survivor on lane 0, victim on lane 1 (fresh staging).
    reset_dut();
    disp_alu_parked(areg(5), 32'h0000_0500);
    disp_branch_parked();
    disp_alu_parked(areg(6), 32'h0000_0600);
    #1;
    check_bit("D2B checkpoint 0 live", `RN.checkpoint_valid_q[0], 1'b1);
    pd_surv  = `ROB.pdst_q[0];  seq_surv = `ROB.seq_q[0];
    pd_vict  = `ROB.pdst_q[2];  seq_vict = `ROB.seq_q[2];
    old_vict = `PRF.regs_q[pd_vict];

    @(negedge clk);
    recover_begin(rob_idx_t'(1));
    f_l0 = mk_pkt(rob_idx_t'(0), seq_surv, pd_surv, 1'b1, 32'hCAFE_00B0);
    f_l1 = mk_pkt(rob_idx_t'(2), seq_vict, pd_vict, 1'b1, 32'hBAD0_00B1);
    force_lane0();
    force_lane1();
    #1;
    check_accepts("D2B mixed verdict (survivor@0 victim@1)", dut.rob_wb_accept, 2'b01);
    @(posedge clk);
    @(negedge clk);
    recover_end();
    #1;
    check_bit ("D2B victim entry dead",    `ROB.valid_q[2], 1'b0);
    check_word("D2B victim PRF unchanged", `PRF.regs_q[pd_vict], old_vict);
    check_bit ("D2B victim ready not set", `PRF.ready_q[pd_vict], 1'b0);
    check_word("D2B survivor PRF written", `PRF.regs_q[pd_surv], 32'hCAFE_00B0);
    check_bit ("D2B survivor ready set",   `PRF.ready_q[pd_surv], 1'b1);

    // =================== trap-flush rejection ===================
    reset_dut();
    disp_alu_parked(areg(5), 32'h0000_0700);
    disp_alu_parked(areg(6), 32'h0000_0800);
    #1;
    pd_a  = `ROB.pdst_q[0];  seq_a = `ROB.seq_q[0];
    pd_b  = `ROB.pdst_q[1];  seq_b = `ROB.seq_q[1];
    old_surv = `PRF.regs_q[pd_a];
    old_vict = `PRF.regs_q[pd_b];

    @(negedge clk);
    force dut.trap_q_valid = 1'b1;
    f_l0 = mk_pkt(rob_idx_t'(0), seq_a, pd_a, 1'b1, 32'hBAD0_0301);
    f_l1 = mk_pkt(rob_idx_t'(1), seq_b, pd_b, 1'b1, 32'hBAD0_0302);
    force_lane0();
    force_lane1();
    #1;
    check_accepts("D3 trap-flush rejects both lanes", dut.rob_wb_accept, 2'b00);
    @(posedge clk);
    @(negedge clk);
    force dut.trap_q_valid = 1'b0;
    release_lanes();
    @(posedge clk);
    @(negedge clk);
    release dut.trap_q_valid;
    #1;
    check_bit ("D3 entry0 flushed",       `ROB.valid_q[0], 1'b0);
    check_bit ("D3 entry1 flushed",       `ROB.valid_q[1], 1'b0);
    check_word("D3 PRF lane0 unchanged",  `PRF.regs_q[pd_a], old_surv);
    check_word("D3 PRF lane1 unchanged",  `PRF.regs_q[pd_b], old_vict);
    check_bit ("D3 ready lane0 not set",  `PRF.ready_q[pd_a], 1'b0);
    check_bit ("D3 ready lane1 not set",  `PRF.ready_q[pd_b], 1'b0);

    // =================== ghost-tie adjudication ===================
    // Real recovery + tail reuse manufactures the tie: idx1 holds a fresh
    // generation while the killed generation's {idx1, stale_seq} ghost is
    // presented on the other lane. Round A: ghost@0, fresh@1.
    reset_dut();
    disp_branch_parked();                          // idx0, ckpt 0
    disp_alu_parked(areg(5), 32'h0000_0900);       // idx1: generation 1
    #1;
    check_bit("D4A checkpoint 0 live", `RN.checkpoint_valid_q[0], 1'b1);
    stale_seq = `ROB.seq_q[1];
    stale_pd  = `ROB.pdst_q[1];
    @(negedge clk);
    recover_begin(rob_idx_t'(0));                  // kills idx1, tail -> 1
    @(posedge clk);
    @(negedge clk);
    recover_end();
    disp_alu_parked(areg(6), 32'h0000_0A00);       // idx1 again: generation 2
    #1;
    fresh_seq = `ROB.seq_q[1];
    fresh_pd  = `ROB.pdst_q[1];
    check_bit ("D4A generations differ", fresh_seq != stale_seq, 1'b1);
    check_bit ("D4A fresh entry live/not-done",
               `ROB.valid_q[1] && !`ROB.done_q[1], 1'b1);
    old_ghost = `PRF.regs_q[stale_pd];

    @(negedge clk);
    f_l0 = mk_pkt(rob_idx_t'(1), stale_seq, stale_pd, 1'b1, 32'hDEAD_04A0);
    f_l1 = mk_pkt(rob_idx_t'(1), fresh_seq, fresh_pd, 1'b1, 32'hC0DE_04A1);
    force_lane0();
    force_lane1();
    #1;
    check_accepts("D4A ghost@0 rejected, fresh@1 accepted", dut.rob_wb_accept, 2'b10);
    @(posedge clk);
    @(negedge clk);
    release_lanes();
    #1;
    check_bit ("D4A fresh generation done", `ROB.done_q[1], 1'b1);
    check_word("D4A fresh PRF written", `PRF.regs_q[fresh_pd], 32'hC0DE_04A1);
    check_bit ("D4A fresh ready set",   `PRF.ready_q[fresh_pd], 1'b1);
    if (stale_pd != fresh_pd) begin
      check_word("D4A ghost PRF untouched", `PRF.regs_q[stale_pd], old_ghost);
    end

    // Round B: fresh@0, ghost@1 (fresh staging; lane roles swapped).
    reset_dut();
    disp_branch_parked();
    disp_alu_parked(areg(5), 32'h0000_0900);
    #1;
    check_bit("D4B checkpoint 0 live", `RN.checkpoint_valid_q[0], 1'b1);
    stale_seq = `ROB.seq_q[1];
    stale_pd  = `ROB.pdst_q[1];
    @(negedge clk);
    recover_begin(rob_idx_t'(0));
    @(posedge clk);
    @(negedge clk);
    recover_end();
    disp_alu_parked(areg(6), 32'h0000_0A00);
    #1;
    fresh_seq = `ROB.seq_q[1];
    fresh_pd  = `ROB.pdst_q[1];
    check_bit("D4B generations differ", fresh_seq != stale_seq, 1'b1);

    @(negedge clk);
    f_l0 = mk_pkt(rob_idx_t'(1), fresh_seq, fresh_pd, 1'b1, 32'hC0DE_04B0);
    f_l1 = mk_pkt(rob_idx_t'(1), stale_seq, stale_pd, 1'b1, 32'hDEAD_04B1);
    force_lane0();
    force_lane1();
    #1;
    check_accepts("D4B fresh@0 accepted, ghost@1 rejected", dut.rob_wb_accept, 2'b01);
    @(posedge clk);
    @(negedge clk);
    release_lanes();
    #1;
    check_bit ("D4B fresh generation done", `ROB.done_q[1], 1'b1);
    check_word("D4B fresh PRF written", `PRF.regs_q[fresh_pd], 32'hC0DE_04B0);
    check_bit ("D4B fresh ready set",   `PRF.ready_q[fresh_pd], 1'b1);

    // =================== natural lane-1 CSR ===================
    // Pass 1 measures both completion paths from a staging point IDENTICAL
    // to pass 2's; pass 2 wakes the CSR so its alu0 completion lands the
    // same cycle the older MUL completes: MUL takes lane 0, the CSR packet
    // rides lane 1. Deterministic sim makes the measured counts exact.
    reset_dut();
    `PRF.regs_q[1] = 32'h5A5A_0123;   // deposit x1's value (p1; ready still 0)
    disp_rem_slow(areg(7));             // idx0 (older); issues during CSR disp
    disp_csr_parked(areg(8));         // idx1 (younger, parked)
    #1;                               // <- measurement point P
    m_lat = 0;
    while (!dut.muldiv_complete.valid) begin
      @(posedge clk); #1;
      m_lat++;
      if (m_lat > 200) $fatal(1, "D5 pass1: muldiv op never completed");
    end
    // MUL drains alone; then measure the CSR wake->completion path.
    @(negedge clk);
    force `PRF.ready_q[1] = 1'b1;
    a_lat = 0;
    while (!dut.alu0_complete.valid) begin
      @(posedge clk); #1;
      a_lat++;
      if (a_lat > 20) $fatal(1, "D5 pass1: CSR completion never landed");
    end
    if (m_lat <= a_lat)
      $fatal(1, "D5: mul path (%0d) not longer than csr path (%0d)", m_lat, a_lat);

    // Pass 2: identical staging to point P, then the aligned wake.
    reset_dut();
    `PRF.regs_q[1] = 32'h5A5A_0123;
    disp_rem_slow(areg(7));
    disp_csr_parked(areg(8));
    #1;                               // <- the same point P
    pd_mul    = `ROB.pdst_q[0];
    pd_csr    = `ROB.pdst_q[1];
    co_before = `ROB.commit_order_q;
    repeat (m_lat - a_lat) @(posedge clk);
    @(negedge clk);
    force `PRF.ready_q[1] = 1'b1;
    wait_n = 0;
    while (!(dut.muldiv_complete.valid && dut.alu0_complete.valid)) begin
      @(posedge clk); #1;
      wait_n++;
      if (wait_n > a_lat + 4)
        $fatal(1, "D5 pass2: alignment failed (mul.v=%0b alu0.v=%0b m_lat=%0d a_lat=%0d)",
               dut.muldiv_complete.valid, dut.alu0_complete.valid, m_lat, a_lat);
    end
    check_word("D5 lane0 selects muldiv", word_t'(dut.cdb_lane_select[0]), word_t'(5'b00100));
    check_word("D5 lane1 selects alu0 (CSR)", word_t'(dut.cdb_lane_select[1]), word_t'(5'b00001));

    @(posedge clk); #1;
    check_bit ("D5 lane1 beat is the CSR entry", dut.cdb_q[1].rob_idx == 1, 1'b1);
    check_bit ("D5 lane1 csr_we metadata", dut.cdb_q[1].csr_we, 1'b1);
    check_word("D5 lane1 csr_wdata metadata", dut.cdb_q[1].csr_wdata, 32'h5A5A_0123);
    check_accepts("D5 both lanes accepted", dut.rob_wb_accept, 2'b11);

    @(posedge clk); #1;
    check_bit ("D5 ROB captured csr_we via lane 1", `ROB.csr_we_q[1], 1'b1);
    check_word("D5 ROB captured csr_wdata via lane 1", `ROB.csr_wdata_q[1], 32'h5A5A_0123);

    wait_n = 0;
    while (`ROB.commit_order_q < (co_before + 64'd2)) begin
      @(posedge clk); #1;
      wait_n++;
      if (wait_n > 10)
        $fatal(1, "D5: expected 2 commits, commit_order=%0d", `ROB.commit_order_q);
    end
    @(posedge clk); #1;
    check_word("D5 architectural CSR write landed (aligned mepc)", `CSRF.mepc_q, 32'h5A5A_0120);
    check_word("D5 CSR rd got old mepc (0)", `PRF.regs_q[pd_csr], 32'h0000_0000);
    check_bit ("D5 mul rd ready", `PRF.ready_q[pd_mul], 1'b1);

    // =================== lane-1 trap capture ===================
    reset_dut();
    disp_alu_parked(areg(9), 32'h0000_0D00);   // idx0 at head (parked)
    #1;
    seq_a = `ROB.seq_q[0];

    @(negedge clk);
    f_l1 = mk_trap_pkt(rob_idx_t'(0), seq_a, 32'd5, 32'hBEEF_0004);
    force_lane1();                              // lane 0 stays RTL-idle
    #1;
    check_accepts("D6 trap packet accepted on lane 1", dut.rob_wb_accept, 2'b10);
    @(posedge clk);
    @(negedge clk);
    release_lanes();
    #1;
    check_bit ("D6 ROB trap_valid captured", `ROB.trap_valid_q[0], 1'b1);
    check_word("D6 ROB trap cause captured", `ROB.trap_cause_q[0], 32'd5);
    check_word("D6 ROB trap tval captured",  `ROB.trap_tval_q[0], 32'hBEEF_0004);
    check_bit ("D6 entry done",              `ROB.done_q[0], 1'b1);

    wait_n = 0;
    while (dut.trap_q_valid !== 1'b1) begin
      @(posedge clk); #1;
      wait_n++;
      if (wait_n > 8)
        $fatal(1, "D6: trap never taken (commit_trap_valid=%0b head done=%0b)",
               dut.commit_trap_valid[0], `ROB.done_q[0]);
    end
    @(posedge clk); #1;
    check_bit ("D6 ROB flushed by taken trap", `ROB.valid_q[0], 1'b0);
    check_word("D6 mcause from lane-1 packet", `CSRF.mcause_q, 32'd5);
    check_word("D6 mtval from lane-1 packet",  `CSRF.mtval_q, 32'hBEEF_0004);
    check_word("D6 mepc is the trap PC",       `CSRF.mepc_q, 32'h0000_1000);

    // =================== three producers, two lanes ===================
    // Same two-pass scheme as : measure from a staging point identical to
    // pass 2's, then wake both ALU uops so all THREE completions land in one
    // cycle: 3 valid holders, 2 lanes -- the youngest holds.
    reset_dut();
    disp_rem_slow(areg(10));
    disp_alu_pair_parked(areg(11), 32'h0000_0333, areg(12), 32'h0000_0444);
    #1;                               // <- measurement point P
    m_lat = 0;
    while (!dut.muldiv_complete.valid) begin
      @(posedge clk); #1;
      m_lat++;
      if (m_lat > 200) $fatal(1, "D7 pass1: muldiv op never completed");
    end
    @(negedge clk);
    force `PRF.ready_q[1] = 1'b1;
    a_lat = 0;
    while (!(dut.alu0_complete.valid && dut.alu1_complete.valid)) begin
      @(posedge clk); #1;
      a_lat++;
      if (a_lat > 20) $fatal(1, "D7 pass1: two ALU completions never landed");
    end
    if (m_lat <= a_lat)
      $fatal(1, "D7: mul path (%0d) not longer than alu path (%0d)", m_lat, a_lat);

    // Pass 2: identical staging to point P, then the aligned wake.
    reset_dut();
    disp_rem_slow(areg(10));                                       // idx0 oldest
    disp_alu_pair_parked(areg(11), 32'h0000_0333, areg(12), 32'h0000_0444); // idx1/idx2
    #1;                               // <- the same point P
    pd_mul    = `ROB.pdst_q[0];
    pd_a      = `ROB.pdst_q[1];
    pd_b      = `ROB.pdst_q[2];
    co_before = `ROB.commit_order_q;
    repeat (m_lat - a_lat) @(posedge clk);
    @(negedge clk);
    force `PRF.ready_q[1] = 1'b1;
    wait_n = 0;
    while (!(dut.muldiv_complete.valid && dut.alu0_complete.valid
             && dut.alu1_complete.valid)) begin
      @(posedge clk); #1;
      wait_n++;
      if (wait_n > a_lat + 4)
        $fatal(1, "D7 pass2: alignment failed (mul.v=%0b alu0.v=%0b alu1.v=%0b m_lat=%0d a_lat=%0d)",
               dut.muldiv_complete.valid, dut.alu0_complete.valid,
               dut.alu1_complete.valid, m_lat, a_lat);
    end
    check_word("D7 lane0 selects muldiv (oldest)",
               word_t'(dut.cdb_lane_select[0]), word_t'(5'b00100));
    check_word("D7 lane1 selects alu0 (middle)",
               word_t'(dut.cdb_lane_select[1]), word_t'(5'b00001));
    held_snap = dut.alu1_complete;   // the youngest: granted on NEITHER lane

    @(posedge clk); #1;
    check_bit ("D7 third producer still held", dut.alu1_complete.valid, 1'b1);
    check_bit ("D7 held packet byte-stable", dut.alu1_complete === held_snap, 1'b1);
    check_accepts("D7 first two-packet drain accepted", dut.rob_wb_accept, 2'b11);
    check_word("D7 holdover wins lane 0 next",
               word_t'(dut.cdb_lane_select[0]), word_t'(5'b00010));

    @(posedge clk); #1;
    check_bit ("D7 holdover beat on lane 0", dut.cdb_q[0].rob_idx == 2, 1'b1);
    check_word("D7 holdover result intact", dut.cdb_q[0].result, 32'h0000_0444);
    check_accepts("D7 holdover accepted alone", dut.rob_wb_accept, 2'b01);

    @(posedge clk); #1;
    check_word("D7 PRF mid result",  `PRF.regs_q[pd_a], 32'h0000_0333);
    check_word("D7 PRF held result", `PRF.regs_q[pd_b], 32'h0000_0444);
    check_bit ("D7 mul rd ready",    `PRF.ready_q[pd_mul], 1'b1);
    check_bit ("D7 beat bus quiescent (lane0)", dut.cdb_q[0].valid, 1'b0);
    check_bit ("D7 beat bus quiescent (lane1)", dut.cdb_q[1].valid, 1'b0);

    wait_n = 0;
    while (`ROB.commit_order_q < (co_before + 64'd3)) begin
      @(posedge clk); #1;
      wait_n++;
      if (wait_n > 10)
        $fatal(1, "D7: expected 3 commits, commit_order=%0d", `ROB.commit_order_q);
    end
    check_bit("D7 all three retired losslessly", 1'b1, 1'b1);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_core_wb_dual] PASS checks=%0d", checks);
      $finish;
    end else begin
      $display("[tb_rv32i_ss_core_wb_dual] FAIL checks=%0d errors=%0d",
               checks, errors);
      $fatal(1, "tb_rv32i_ss_core_wb_dual failed");
    end
  end

endmodule
