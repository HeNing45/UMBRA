// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// Dispatch-integration TB for rv32i_ss_core.
//
// The core may issue and commit while later packets are dispatched, so every
// check keys on the *bundle_fire event itself* (sampled when a packet is
// accepted), not on standing ROB occupancy.
//
// What it proves:
//   - each accepted packet lands in the ROB under the correct rob_idx, with the
//     correct renamed fields (sampled right after the dispatch edge, before
//     commit can clear it);
//   - RAW flows through rename (consumer prs == producer pdst);
//   - rd=x0 allocates no physreg;
//   - dispatch is gated by backend accept: when the IQ is full, decoded_ready
//     drops and the ROB tail freezes (the bundle_fire gate, not decoded_valid).

module tb_rv32i_ss_core_dispatch;
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
    .redirect_target          (redirect_target)
  );

  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;

  initial begin
    repeat (3000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_dispatch exceeded 3000 cycles");
  end

  `define ROB dut.u_rob
  `define RN  dut.u_rename
  `define IQ  dut.u_iq

  function automatic arch_reg_t areg(input int v); areg = arch_reg_t'(v); endfunction
  function automatic phys_reg_t preg(input int v); preg = phys_reg_t'(v); endfunction

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

  task automatic check_bool(input string name, input logic got, input logic exp);
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

  task automatic check_preg(input string name, input phys_reg_t got, input phys_reg_t exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got=p%0d exp=p%0d", name, got, exp);
      errors++;
    end
  endtask

  task automatic reset_dut();
    clear_inputs();
    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    @(negedge clk);
  endtask

  function automatic int iq_count();
    int n;
    n = 0;
    for (int i = 0; i < OOO_IQ_DEPTH; i++) begin
      if (`IQ.valid_q[i]) n++;
    end
    iq_count = n;
  endfunction

  // Drive one packet, confirm it is accepted this cycle, sample the renamed
  // outputs at the dispatch point, then deassert.
  task automatic rob_alloc_packet(
    input  string         name,
    input  word_t         pc,
    input  word_t         instr,
    input  arch_reg_t     rs1,
    input  arch_reg_t     rs2,
    input  arch_reg_t     rd,
    input  logic          rd_we,
    input  ooo_op_class_e op_class,
    input  alu_op_e       alu_op,
    input  ooo_src_sel_e  src1_sel,
    input  ooo_src_sel_e  src2_sel,
    input  word_t         imm,
    output rob_idx_t      out_rob_idx,
    output phys_reg_t     out_pdst,
    output phys_reg_t     out_stale,
    output phys_reg_t     out_prs1,
    output phys_reg_t     out_prs2
  );
    @(negedge clk);
    decoded_valid            = 1'b1;
    decoded_pc               = pc;
    decoded_instr            = instr;
    decoded_rs1              = rs1;
    decoded_rs2              = rs2;
    decoded_rd               = rd;
    decoded_rd_we            = rd_we;
    decoded_needs_checkpoint = 1'b0;
    decoded_op_class         = op_class;
    decoded_alu_op           = alu_op;
    decoded_branch_op        = rv32i_pipeline_pkg::BR_NONE;
    decoded_src1_sel         = src1_sel;
    decoded_src2_sel         = src2_sel;
    decoded_imm              = imm;
    #1;
    checks++;
    if (decoded_ready !== 1'b1) begin
      $error("[%s] expected decoded_ready=1 (ROB has room), got %0b",
             name, decoded_ready);
      errors++;
    end
    out_rob_idx = `ROB.rob_alloc_idx;
    out_pdst    = `RN.rename_pdst;
    out_stale   = `RN.rename_stale_pdst;
    out_prs1    = `RN.rename_prs1;
    out_prs2    = `RN.rename_prs2;
    @(posedge clk);            // dispatch latched here
    @(negedge clk);
    decoded_valid = 1'b0;
    #1;
    $display("[%s] rob[%0d] pdst=p%0d stale=p%0d prs1=p%0d prs2=p%0d",
             name, out_rob_idx, out_pdst, out_stale, out_prs1, out_prs2);
  endtask

  // Checks the just-dispatched entry. Called immediately after rob_alloc_packet,
  // i.e. < 1 cycle after the dispatch edge: valid=1, done not yet set, fields
  // fresh (commit cannot have cleared it yet).
  task automatic check_rob_entry(
    input string     name,
    input rob_idx_t  idx,
    input word_t     pc,
    input arch_reg_t rd,
    input logic      rd_we,
    input phys_reg_t pdst,
    input phys_reg_t stale
  );
    check_bool({name, " rob valid"}, `ROB.valid_q[idx], 1'b1);
    check_bool({name, " rob done"},  `ROB.done_q[idx],  1'b0);
    check_word({name, " rob pc"},    `ROB.pc_q[idx],    pc);
    check_preg({name, " rob rd"},    phys_reg_t'(`ROB.rd_q[idx]),
                                     phys_reg_t'(rd));
    check_bool({name, " rob rd_we"}, `ROB.rd_we_q[idx], rd_we);
    check_preg({name, " rob pdst"},  `ROB.pdst_q[idx],  pdst);
    check_preg({name, " rob stale"}, `ROB.stale_pdst_q[idx], stale);
  endtask

  rob_idx_t  r0, r1, r2;
  phys_reg_t pdst0, pdst1, pdst2;
  phys_reg_t st0,   st1,   st2;
  phys_reg_t prs1_0, prs2_0, prs1_1, prs2_1, prs1_2, prs2_2;

  int        fill_i;
  rob_idx_t  tail_before;

  initial begin
    $display("[tb_rv32i_ss_core_dispatch] starting");

    reset_dut();

    // ---------------- Test 1: dispatch lands + RAW through rename ----------
    // add x3, x1, x2 -> pdst=p32, stale = reset spec_map[x3] = p3
    rob_alloc_packet("t1.a add x3,x1,x2",
      32'h0000_1000, 32'h002080b3,
      areg(1), areg(2), areg(3), 1'b1,
      OOO_OP_ALU, fyp_cpu_pkg::ALU_ADD,
      OOO_SRC_REG, OOO_SRC_REG, 32'h0,
      r0, pdst0, st0, prs1_0, prs2_0);

    check_preg("t1.a prs1=p1",  prs1_0, preg(1));
    check_preg("t1.a prs2=p2",  prs2_0, preg(2));
    check_preg("t1.a pdst=p32", pdst0,  preg(32));
    check_preg("t1.a stale=p3", st0,    preg(3));
    check_rob_entry("t1.a", r0, 32'h0000_1000, areg(3), 1'b1, pdst0, preg(3));

    // add x4, x3, x1 -> RAW: prs1 must equal pdst0; pdst distinct
    rob_alloc_packet("t1.b add x4,x3,x1",
      32'h0000_1004, 32'h00118233,
      areg(3), areg(1), areg(4), 1'b1,
      OOO_OP_ALU, fyp_cpu_pkg::ALU_ADD,
      OOO_SRC_REG, OOO_SRC_REG, 32'h0,
      r1, pdst1, st1, prs1_1, prs2_1);

    check_preg("t1.b prs1=pdst0 (RAW)", prs1_1, pdst0);
    check_preg("t1.b prs2=p1",          prs2_1, preg(1));
    checks++;
    if (pdst1 === pdst0) begin
      $error("t1.b pdst must differ from t1.a pdst (both p%0d)", pdst1);
      errors++;
    end
    check_rob_entry("t1.b", r1, 32'h0000_1004, areg(4), 1'b1, pdst1, st1);

    // ---------------- Test 2: rd=x0 must not allocate ---------------------
    rob_alloc_packet("t2 add x0,x1,x2",
      32'h0000_1008, 32'h00208033,
      areg(1), areg(2), areg(0), 1'b1,
      OOO_OP_ALU, fyp_cpu_pkg::ALU_ADD,
      OOO_SRC_REG, OOO_SRC_REG, 32'h0,
      r2, pdst2, st2, prs1_2, prs2_2);

    check_preg("t2 pdst=p0  (no alloc)", pdst2, preg(0));
    check_preg("t2 stale=p0 (no alloc)", st2,   preg(0));
    check_bool("t2 rob.rd_we=0",         `ROB.rd_we_q[r2], 1'b0);

    // let everything drain before the backpressure test
    clear_inputs();
    fill_i = 0;
    while (`ROB.count_q !== '0) begin
      @(posedge clk);
      fill_i++;
      if (fill_i > 100) $fatal(1, "drain stuck before t3: count=%0d", `ROB.count_q);
    end

    // ---------------- Test 3: IQ-full backpressure freezes dispatch -------
    // Stream DIVU uops into the single-occupancy 32-cycle MULDIV unit so
    // dispatch out-paces execution independently of the wakeup latency. The
    // 16-entry IQ fills before the 32-entry ROB/free-list boundary. While
    // decoded_ready=0 due to IQ-full, present a marker: bundle_fire must be 0,
    // so the ROB tail must NOT advance.
    @(negedge clk);
    decoded_valid    = 1'b1;
    decoded_pc       = 32'h0000_2000;
    decoded_instr    = 32'h00000013;
    // drain-or-empty: ALU fillers drain at the same 1/cycle rate as this
    // direct-core source. DIVU makes the structural one-at-a-time FU limit the
    // stable cause of occupancy rather than an assumed wakeup interval.
    decoded_rs1      = areg(3);
    decoded_rs2      = areg(2);
    decoded_rd       = areg(3);
    decoded_rd_we    = 1'b1;
    decoded_op_class = OOO_OP_ALU;
    decoded_alu_op   = fyp_cpu_pkg::ALU_ADD;
    decoded_src1_sel = OOO_SRC_REG;
    decoded_src2_sel = OOO_SRC_IMM;
    decoded_imm      = 32'd3;
    decoded_fu_class = OOO_FU_MULDIV;
    decoded_muldiv_op = rv32i_pipeline_pkg::MD_DIVU;

    fill_i = 0;
    while (decoded_ready === 1'b1) begin
      @(negedge clk);
      fill_i++;
      if (fill_i > 256) begin
        $fatal(1, "IQ never filled: iq_count=%0d rob_count=%0d",
               iq_count(), `ROB.count_q);
      end
    end

    // decoded_ready==0 here: IQ is full, and the ROB still has room.
    check_word("t3 iq.count==DEPTH at full",
               word_t'(iq_count()), word_t'(OOO_IQ_DEPTH));
    checks++;
    if (`ROB.count_q >= OOO_ROB_DEPTH) begin
      $error("t3 expected IQ-full before ROB-full, rob_count=%0d", `ROB.count_q);
      errors++;
    end
    tail_before = `ROB.tail_q;

    // present the marker during the full (ready=0) cycle
    decoded_pc = 32'hdead_beef;
    @(posedge clk);                 // iq_alloc_ready was 0 -> no ROB write
    @(negedge clk);
    decoded_valid = 1'b0;
    #1;

    check_preg("t3 tail frozen under full",
               phys_reg_t'(`ROB.tail_q), phys_reg_t'(tail_before));

    // sanity: the burst drains back to empty once we stop driving
    fill_i = 0;
    while (`ROB.count_q !== '0) begin
      @(posedge clk);
      fill_i++;
      if (fill_i > 1200) $fatal(1, "post-t3 drain stuck: count=%0d", `ROB.count_q);
    end
    check_word("t3 drains to empty", word_t'({26'b0, `ROB.count_q}), 32'd0);

    // ============ dispatch_accept holds the frontend (case 6) ============
    // A JUMP dispatch sets jump_inflight_q; a following valid bundle must be
    // HELD: decoded_ready low, bundle_fire forbidden, zero ROB/IQ deltas.
    // This is the directed kill for dropping dispatch_accept from the common
    // bundle-fire product. Recovery/trap broadcast exactness is covered by the
    // shape-11 battery; this arm proves the jump-serialization predicate.
    clear_inputs();
    @(negedge clk);
    decoded_valid    = 1'b1;
    decoded_op_class = OOO_OP_JUMP;      // jal-class: serializes dispatch
    decoded_pc       = 32'h0000_3000;
    @(posedge clk); @(negedge clk);      // jump dispatches on this edge
    decoded_op_class = OOO_OP_ALU;       // next bundle: plain ALU, still valid
    decoded_pc       = 32'h0000_3004;
    #1;
    check_bool("B1 jump in flight after dispatch", dut.jump_inflight_q, 1'b1);
    check_bool("B1 accept holds decoded_ready low", decoded_ready, 1'b0);
    check_bool("B1 no bundle_fire under serialization", dut.bundle_fire, 1'b0);
    tail_before = `ROB.tail_q;
    repeat (2) @(negedge clk);
    // the jump may resolve in background; the held bundle must not have
    // dispatched during any serialized cycle: tail moved at most 0 while
    // jump_inflight_q stayed high (probe again before releasing)
    if (dut.jump_inflight_q === 1'b1) begin
      check_preg("B1 tail frozen while serialized",
                 phys_reg_t'(`ROB.tail_q), phys_reg_t'(tail_before));
    end else begin
      checks++;  // jump already resolved; hold window closed legally
    end
    clear_inputs();
    repeat (8) @(negedge clk);           // drain

    if (errors == 0) begin
      $display("[tb_rv32i_ss_core_dispatch] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_core_dispatch] FAIL errors=%0d checks=%0d",
             errors, checks);
    end
  end

endmodule
