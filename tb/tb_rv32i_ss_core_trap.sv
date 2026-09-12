`timescale 1ns/1ps

// =============================================================================
// tb_rv32i_ss_core_trap.sv -- directed proof of take-the-trap + mret.
//
// At the ROB head a decode-detected trap (ecall/ebreak/illegal) or mret drives,
// one cycle later (registered trap_q), the CSR trap write + a FULL flush
// (ROB/IQ/rename/free-list back to committed) + a redirect to mtvec / mepc.
//
// ecall : mepc<-pc, mcause<-11, mstatus push, redirect to mtvec, ROB empty,
//               and the redirect is a ONE-cycle pulse (the trap_latch_fire fix).
// mret : mstatus pop, redirect to mepc.
// precise: an OLDER op commits, a YOUNGER op behind the trap is squashed.
// ebreak : mcause<-3, mtval<-pc carried through.
//
// CSR addrs: mstatus 0x300, mtvec 0x305, mepc 0x341, mcause 0x342, mtval 0x343.
// mstatus bits: MIE=3, MPIE=7, MPP=[12:11]=2'b11. Pushed mstatus(MIE=1) = 0x1880;
// popped = 0x1888.
// =============================================================================

module tb_rv32i_ss_core_trap;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;
  import rv32i_pipeline_pkg::CSR_NONE;
  import rv32i_pipeline_pkg::CSR_RW;

  localparam time CLK_PERIOD = 10ns;
  localparam logic [11:0] MSTATUS = 12'h300;
  localparam logic [11:0] MTVEC   = 12'h305;

  logic clk, rst_n;
  logic           decoded_valid, decoded_ready;
  word_t [1:0] decoded_pc, decoded_instr;
  arch_reg_t [1:0] decoded_rs1, decoded_rs2, decoded_rd;
  logic [1:0] decoded_rd_we, decoded_needs_checkpoint;
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
  csr_op_e        decoded_csr_op;
  csr_addr_t      decoded_csr_addr;
  csr_zimm_t      decoded_csr_zimm;
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
    .clk(clk), .rst_n(rst_n),
    .decoded_valid(decoded_valid), .decoded_slot_valid({1'b0, decoded_valid}), .decoded_ready(decoded_ready),
    .decoded_pc(decoded_pc), .decoded_instr(decoded_instr),
    .decoded_rs1(decoded_rs1), .decoded_rs2(decoded_rs2), .decoded_rd(decoded_rd),
    .decoded_rd_we(decoded_rd_we), .decoded_needs_checkpoint(decoded_needs_checkpoint),
    .decoded_op_class(decoded_op_class), .decoded_alu_op(decoded_alu_op),
    .decoded_branch_op(decoded_branch_op),
    .decoded_src1_sel(decoded_src1_sel), .decoded_src2_sel(decoded_src2_sel),
    .decoded_imm(decoded_imm),
    .decoded_fu_class(decoded_fu_class), .decoded_muldiv_op(decoded_muldiv_op),
    .decoded_trap(decoded_trap),
    .decoded_is_load(decoded_is_load),
    .decoded_is_store(decoded_is_store),
    .decoded_mem_size(decoded_mem_size),
    .decoded_mem_unsigned(decoded_mem_unsigned),
    .decoded_pred_taken('0),  // tie-off: static not-taken prediction
    .decoded_pred_target('0),
    .decoded_csr_op(decoded_csr_op), .decoded_csr_addr(decoded_csr_addr),
    .decoded_csr_zimm(decoded_csr_zimm),
    .redirect_valid(redirect_valid), .redirect_target(redirect_target),
    .commit_fire(commit_fire), .commit_order(commit_order),
    .commit_pc(commit_pc), .commit_inst(commit_inst), .commit_rd(commit_rd),
    .commit_rd_wen(commit_rd_wen), .commit_wdata(commit_wdata)
  );

  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;
  initial begin
    repeat (3000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_trap exceeded 3000 cycles");
  end

  function automatic arch_reg_t areg(input int v); areg = arch_reg_t'(v); endfunction
  function automatic word_t creg(input int n);
    creg = dut.u_prf.regs_q[dut.u_rename.committed_map_q[n]];
  endfunction
  function automatic int iq_count();
    int n; n = 0;
    for (int i = 0; i < OOO_IQ_DEPTH; i++) if (dut.u_iq.valid_q[i]) n++;
    iq_count = n;
  endfunction

  task automatic check_word(input string name, input word_t got, input word_t exp);
    checks++;
    if (got !== exp) begin $error("[%s] got %08h exp %08h", name, got, exp); errors++; end
  endtask

  // ---- redirect-pulse monitor: capture target + the widest contiguous run ----
  int    redir_run = 0;
  int    redir_max_run = 0;
  int    slot1_trap_stop_n = 0;
  int    slot1_mret_stop_n = 0;
  word_t redir_target_cap = '0;
  always @(posedge clk) begin
    if (rst_n && redirect_valid) begin
      redir_run        <= redir_run + 1;
      redir_target_cap <= redirect_target;
      if (redir_run + 1 > redir_max_run) redir_max_run <= redir_run + 1;
    end else begin
      redir_run <= 0;
    end
    if (rst_n && (commit_fire == 2'b01) && dut.commit_trap_valid[1])
      slot1_trap_stop_n = slot1_trap_stop_n + 1;
    if (rst_n && (commit_fire == 2'b01) && dut.commit_is_mret[1])
      slot1_mret_stop_n = slot1_mret_stop_n + 1;
  end

  task automatic clear_inputs();
    decoded_valid = 1'b0; decoded_pc = '0; decoded_instr = '0;
    decoded_rs1 = '0; decoded_rs2 = '0; decoded_rd = '0;
    decoded_rd_we = 1'b0; decoded_needs_checkpoint = 1'b0;
    decoded_op_class = OOO_OP_ALU; decoded_alu_op = fyp_cpu_pkg::ALU_ADD;
    decoded_branch_op = rv32i_pipeline_pkg::BR_NONE;
    decoded_src1_sel = OOO_SRC_REG; decoded_src2_sel = OOO_SRC_REG; decoded_imm = '0;
    decoded_fu_class = OOO_FU_ALU; decoded_muldiv_op = rv32i_pipeline_pkg::MD_MUL;
    decoded_trap = '0;
    decoded_is_load = 1'b0;
    decoded_is_store = 1'b0;
    decoded_mem_size = fyp_cpu_pkg::MEM_W;
    decoded_mem_unsigned = 1'b0;
    decoded_csr_op = CSR_NONE; decoded_csr_addr = '0; decoded_csr_zimm = '0;
  endtask

  task automatic reset_dut();
    clear_inputs(); rst_n = 1'b0;
    repeat (3) @(posedge clk); rst_n = 1'b1; @(negedge clk);
  endtask

  task automatic fire();
    #1;
    while (decoded_ready !== 1'b1) @(negedge clk);
    @(posedge clk); @(negedge clk);
    clear_inputs();
  endtask

  task automatic disp_addi(input word_t pc, input int rd, input int rs1, input word_t imm);
    @(negedge clk);
    clear_inputs();
    decoded_valid = 1'b1; decoded_pc = pc;
    decoded_rs1 = areg(rs1); decoded_rd = areg(rd); decoded_rd_we = (rd != 0);
    decoded_src1_sel = OOO_SRC_REG; decoded_src2_sel = OOO_SRC_IMM; decoded_imm = imm;
    fire();
  endtask

  task automatic disp_csr(input word_t pc, input csr_op_e cop, input logic [11:0] caddr,
                          input int rd, input int rs1);
    @(negedge clk);
    clear_inputs();
    decoded_valid = 1'b1; decoded_pc = pc;
    decoded_rs1 = areg(rs1); decoded_rd = areg(rd); decoded_rd_we = (rd != 0);
    decoded_src1_sel = OOO_SRC_REG; decoded_src2_sel = OOO_SRC_IMM;
    decoded_csr_op = cop; decoded_csr_addr = csr_addr_t'(caddr);
    decoded_csr_zimm = csr_zimm_t'(rs1);
    fire();
  endtask

  // a trapping/mret uop: ALU-class (executes -> done -> reaches head), no rd.
  task automatic disp_trap(input word_t pc, input word_t cause, input word_t tval,
                           input bit is_mret);
    @(negedge clk);
    clear_inputs();
    decoded_valid = 1'b1; decoded_pc = pc;
    decoded_src1_sel = OOO_SRC_REG; decoded_src2_sel = OOO_SRC_IMM; decoded_imm = '0;
    decoded_trap.valid   = ~is_mret;
    decoded_trap.cause   = cause;
    decoded_trap.tval    = tval;
    decoded_trap.is_mret = is_mret;
    fire();
  endtask

  int w;
  // wait for the trap-take: the ROB drains to empty via the full flush.
  task automatic wait_trap();
    w = 0;
    while (dut.u_rob.count_q !== '0 && w < 80) begin @(posedge clk); w++; end
    if (dut.u_rob.count_q !== '0) $fatal(1, "trap never taken: ROB not flushed (count=%0d)", dut.u_rob.count_q);
    repeat (2) @(posedge clk);
  endtask

  task automatic drain();
    w = 0;
    while (dut.u_rob.count_q !== '0 && w < 200) begin @(posedge clk); w++; end
    repeat (2) @(posedge clk);
  endtask

  // set mstatus = MIE(0x8) and mtvec = 0x200 (committed CSR state before a trap)
  task automatic setup_csr();
    disp_addi(32'h0100, 1, 0, 32'h0000_0008);   // x1 = 0x8 (MIE)
    disp_addi(32'h0104, 2, 0, 32'h0000_0200);   // x2 = 0x200 (mtvec)
    disp_csr (32'h0108, CSR_RW, MSTATUS, 0, 1);  // mstatus <- x1
    disp_csr (32'h010C, CSR_RW, MTVEC,   0, 2);  // mtvec   <- x2
    drain();
  endtask

  // ---reproduction helpers for the -class trap-flush deadlocks ----------
  // a trapping uop that depends on a (force-held) register so it lingers at the
  // head while a YOUNGER serializing/long-latency uop dispatches behind it.
  task automatic disp_trap_held(input word_t pc, input word_t cause, input int rs1);
    @(negedge clk); clear_inputs();
    decoded_valid = 1'b1; decoded_pc = pc;
    decoded_rs1 = areg(rs1); decoded_src1_sel = OOO_SRC_REG; decoded_src2_sel = OOO_SRC_IMM;
    decoded_trap.valid = 1'b1; decoded_trap.cause = cause;
    fire();
  endtask
  task automatic disp_jalr(input word_t pc, input int rd, input int rs1, input word_t off);
    @(negedge clk); clear_inputs();
    decoded_valid = 1'b1; decoded_pc = pc;
    decoded_rs1 = areg(rs1); decoded_rd = areg(rd); decoded_rd_we = (rd != 0);
    decoded_op_class = OOO_OP_JUMP; decoded_fu_class = OOO_FU_ALU;
    decoded_src1_sel = OOO_SRC_REG; decoded_src2_sel = OOO_SRC_IMM; decoded_imm = off;
    fire();
  endtask
  // REM, not MUL: needs the muldiv FSM still busy when the trap fires,
  // and only the DIV family still runs 32 cycles now that the multiplier is
  // 2-stage pipelined. The divisor register must hold a nonzero value or the
  // div-by-zero fast path skips the FSM occupancy entirely.
  task automatic disp_rem(input word_t pc, input int rd, input int rs1, input int rs2);
    @(negedge clk); clear_inputs();
    decoded_valid = 1'b1; decoded_pc = pc;
    decoded_rs1 = areg(rs1); decoded_rs2 = areg(rs2); decoded_rd = areg(rd); decoded_rd_we = (rd != 0);
    decoded_fu_class = OOO_FU_MULDIV; decoded_muldiv_op = rv32i_pipeline_pkg::MD_REM;
    decoded_src1_sel = OOO_SRC_REG; decoded_src2_sel = OOO_SRC_REG;
    fire();
  endtask
  // post-trap liveness: the handler's first op must be able to dispatch (a stuck
  // inflight gate hangs forever -- bounded wait so the failure is reported, not a
  // 3000-cycle watchdog).
  task automatic post_trap_addi(input word_t pc, input int rd, input word_t imm);
    @(negedge clk); clear_inputs();
    decoded_valid = 1'b1; decoded_pc = pc;
    decoded_rd = areg(rd); decoded_rd_we = 1'b1;
    decoded_src1_sel = OOO_SRC_REG; decoded_src2_sel = OOO_SRC_IMM; decoded_imm = imm;
    #1; w = 0;
    while (decoded_ready !== 1'b1 && w < 80) begin @(negedge clk); w++; end
    checks++;
    if (decoded_ready !== 1'b1) begin
      $error("POST-TRAP DEADLOCK: handler cannot dispatch (inflight gate stuck after trap)");
      errors++; decoded_valid = 1'b0; clear_inputs();
    end else begin
      @(posedge clk); @(negedge clk); clear_inputs(); drain();
      check_word("post-trap handler committed", creg(rd), imm);
    end
  endtask

  initial begin
    $display("[tb_rv32i_ss_core_trap] starting");

    // ===== ecall takes the trap =========================================
    reset_dut();
    setup_csr();
    check_word("setup: mstatus==MIE", dut.u_csr_file.mstatus_q, 32'h0000_0008);
    check_word("setup: mtvec==0x200", dut.u_csr_file.mtvec_q,   32'h0000_0200);

    disp_trap(32'h0000_1000, 32'd11, 32'd0, 1'b0);   // ecall
    wait_trap();
    check_word("T1 ecall: mepc<-pc",        dut.u_csr_file.mepc_q,    32'h0000_1000);
    check_word("T1 ecall: mcause<-11",      dut.u_csr_file.mcause_q,  32'd11);
    check_word("T1 ecall: mstatus push",    dut.u_csr_file.mstatus_q, 32'h0000_1880);
    check_word("T1 ecall: redirect to mtvec", redir_target_cap,       32'h0000_0200);
    check_word("T1 ecall: ROB flushed",     word_t'(dut.u_rob.count_q), 32'd0);
    check_word("T1 ecall: IQ flushed",      word_t'(iq_count()),         32'd0);
    check_word("T1 ecall: redirect is a 1-cycle pulse", word_t'(redir_max_run), 32'd1);

    // ===== position-1 mret stops the prefix, then returns ===============
    // Hold an older ordinary instruction until mret is complete at head+1.
    // Releasing it must commit position 0 alone; mret acts only after becoming
    // the head on the following cycle.
    force dut.u_prf.ready_q[3] = 1'b0;
    disp_addi(32'h0000_1ffc, 13, 3, 32'h0000_0013);
    disp_trap(32'h0000_2000, 32'd0, 32'd0, 1'b1);    // mret
    wait (dut.commit_is_mret[1] === 1'b1);
    force dut.u_prf.ready_q[3] = 1'b1;
    wait_trap();
    release dut.u_prf.ready_q[3];
    check_word("T2 position-1 mret challenged the prefix",
               word_t'(slot1_mret_stop_n), 32'd1);
    check_word("T2 older instruction committed before mret",
               creg(13), 32'h0000_0013);
    check_word("T2 mret: mstatus pop",      dut.u_csr_file.mstatus_q, 32'h0000_1888);
    check_word("T2 mret: redirect to mepc", redir_target_cap,         32'h0000_1000);

    // ===== position-1 trap stops prefix; younger is squashed ============
    reset_dut();
    setup_csr();
    force dut.u_prf.ready_q[3] = 1'b0;
    disp_addi(32'h0000_3000, 4, 3, 32'h0000_0044);   // OLDER held at position 0
    disp_trap(32'h0000_3004, 32'd11, 32'd0, 1'b0);   // ecall
    disp_addi(32'h0000_3008, 5, 0, 32'h0000_0055);   // YOUNGER: x5=0x55, must be squashed
    wait (dut.commit_trap_valid[1] === 1'b1);
    force dut.u_prf.ready_q[3] = 1'b1;
    wait_trap();
    release dut.u_prf.ready_q[3];
    check_word("T3 position-1 trap challenged the prefix",
               word_t'(slot1_trap_stop_n), 32'd1);
    check_word("T3 precise: older x4 committed",   creg(4), 32'h0000_0044);
    check_word("T3 precise: younger x5 squashed",  creg(5), 32'h0000_0000);
    check_word("T3 precise: mcause<-11",           dut.u_csr_file.mcause_q, 32'd11);

    // ===== ebreak carries cause 3 + mtval ===============================
    reset_dut();
    setup_csr();
    disp_trap(32'h0000_4000, 32'd3, 32'h0000_4000, 1'b0);  // ebreak (tval=pc)
    wait_trap();
    check_word("T4 ebreak: mcause<-3",  dut.u_csr_file.mcause_q, 32'd3);
    check_word("T4 ebreak: mtval<-pc",  dut.u_csr_file.mtval_q,  32'h0000_4000);
    check_word("T4 ebreak: mepc<-pc",   dut.u_csr_file.mepc_q,   32'h0000_4000);

    // ===== a YOUNGER CSR flushed by a trap must not stick csr_inflight =====
    // (dispatch deadlock). Hold the ecall at the head; a younger csrrw
    //   dispatches behind it (csr_inflight=1) and is flushed without committing.
    reset_dut();
    force dut.u_prf.ready_q[3] = 1'b0;                  // hold ecall (depends on x3)
    disp_trap_held(32'h0000_5000, 32'd11, 3);
    disp_csr(32'h0000_5004, CSR_RW, MTVEC, 0, 0);       // younger csrrw -> csr_inflight=1
    repeat (3) @(posedge clk);
    force dut.u_prf.ready_q[3] = 1'b1;                  // release -> ecall traps -> flush
    wait_trap();
    release dut.u_prf.ready_q[3];
    post_trap_addi(32'h0000_5008, 9, 32'h0000_0099);   // hangs if csr_inflight stuck

    // ===== a YOUNGER jump flushed (unissued) by a trap must not stick =======
    //   jump_inflight. Hold both the ecall and a jalr; the jalr lingers in the IQ
    //   (jump_inflight=1) and is flushed before it ever resolves.
    reset_dut();
    force dut.u_prf.ready_q[3] = 1'b0;                  // hold ecall (x3)
    force dut.u_prf.ready_q[4] = 1'b0;                  // hold jalr  (x4) -> stays in IQ
    disp_trap_held(32'h0000_6000, 32'd11, 3);
    disp_jalr(32'h0000_6004, 0, 4, 32'd0);             // younger jalr -> jump_inflight=1
    repeat (3) @(posedge clk);
    force dut.u_prf.ready_q[3] = 1'b1;                  // release ecall -> traps -> flush jalr
    wait_trap();
    release dut.u_prf.ready_q[3];
    release dut.u_prf.ready_q[4];
    post_trap_addi(32'h0000_6008, 10, 32'h0000_00AA);  // hangs if jump_inflight stuck

    // ===== a muldiv in flight when a trap fires must be KILLED ==============
    reset_dut();
    dut.u_prf.regs_q[7] = 32'd7;                        // nonzero divisor for the REM
                                                        // (p7 is never FF-written)
    force dut.u_prf.ready_q[3] = 1'b0;                  // hold ecall
    disp_trap_held(32'h0000_7000, 32'd11, 3);
    disp_rem(32'h0000_7004, 11, 0, 7);                 // younger muldiv -> 32-cycle FSM
    repeat (4) @(posedge clk);
    check_word("T7 pre: muldiv running under held trap", word_t'(dut.muldiv_busy), 32'd1);
    force dut.u_prf.ready_q[3] = 1'b1;                  // release ecall -> traps -> kill muldiv
    wait_trap();
    release dut.u_prf.ready_q[3];
    check_word("T7: muldiv killed by trap", word_t'(dut.muldiv_busy), 32'd0);
    post_trap_addi(32'h0000_7008, 12, 32'h0000_00CC);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_core_trap] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_core_trap] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
