`timescale 1ns/1ps

// =============================================================================
// tb_rv32i_ss_core_csr.sv -- directed proof of CSR architectural behavior.
//
// Exercises the full CSR path end to end through the core: rd <- OLD csr value
// (read at execute), csr <- NEW value (RMW, written at COMMIT), and the x0/zimm
// write-suppress rule. Uses mtvec (0x305) as the scratch CSR -- it is fully
// read/write in csr_file (no WARL masking), unlike mstatus.
//
// csrrw : write csr = rs1, rd = old csr.
// csrrs x0 : pure read (write SUPPRESSED), rd = old, csr unchanged.
// csrrs : csr |= rs1, rd = old.
// csrrc : csr &= ~rs1, rd = old.
//
// The csr_file write is commit-gated, so each group drains the ROB before
// checking dut.u_csr_file.mtvec_q (architectural state) and the committed rd.
// =============================================================================

module tb_rv32i_ss_core_csr;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;
  import rv32i_pipeline_pkg::CSR_NONE;
  import rv32i_pipeline_pkg::CSR_RW;
  import rv32i_pipeline_pkg::CSR_RS;
  import rv32i_pipeline_pkg::CSR_RC;

  localparam time CLK_PERIOD = 10ns;
  localparam logic [11:0] MTVEC = 12'h305;

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
  int slot1_csr_stop_n = 0;

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
  always @(posedge clk) begin
    if (rst_n && (commit_fire == 2'b01) &&
        dut.rob_commit_valid[1] && dut.rob_commit_is_csr[1])
      slot1_csr_stop_n = slot1_csr_stop_n + 1;
  end
  initial begin
    repeat (3000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_csr exceeded 3000 cycles");
  end

  function automatic arch_reg_t areg(input int v); areg = arch_reg_t'(v); endfunction
  // committed architectural register value (through the committed map -> PRF).
  function automatic word_t creg(input int n);
    creg = dut.u_prf.regs_q[dut.u_rename.committed_map_q[n]];
  endfunction

  task automatic check_word(input string name, input word_t got, input word_t exp);
    checks++;
    if (got !== exp) begin $error("[%s] got %08h exp %08h", name, got, exp); errors++; end
  endtask

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

  // ---- one dispatch handshake (drives the decoded packet for one uop) -------
  task automatic fire();
    int fire_wait;
    #1;
    fire_wait = 0;
    while ((decoded_ready !== 1'b1) && (fire_wait < 100)) begin
      @(negedge clk);
      fire_wait++;
    end
    if (decoded_ready !== 1'b1)
      $fatal(1, "CSR_DISPATCH_TIMEOUT: ready stayed low (csr_inflight=%0b rob_count=%0d)",
             dut.csr_inflight_q, dut.u_rob.count_q);
    @(posedge clk); @(negedge clk);
    clear_inputs();
  endtask

  // addi rd, rs1, imm   (rd = rs1 + imm)
  task automatic disp_addi(input word_t pc, input int rd, input int rs1, input word_t imm);
    @(negedge clk);
    clear_inputs();
    decoded_valid = 1'b1; decoded_pc = pc;
    decoded_rs1 = areg(rs1); decoded_rd = areg(rd); decoded_rd_we = (rd != 0);
    decoded_alu_op = fyp_cpu_pkg::ALU_ADD;
    decoded_src1_sel = OOO_SRC_REG; decoded_src2_sel = OOO_SRC_IMM; decoded_imm = imm;
    fire();
  endtask

  // CSR rd, csr, rs1   (rs1 field doubles as the zimm bits, used for write-suppress)
  task automatic disp_csr(input word_t pc, input csr_op_e cop, input logic [11:0] caddr,
                          input int rd, input int rs1);
    @(negedge clk);
    clear_inputs();
    decoded_valid = 1'b1; decoded_pc = pc;
    decoded_rs1 = areg(rs1); decoded_rd = areg(rd); decoded_rd_we = (rd != 0);
    decoded_op_class = OOO_OP_ALU; decoded_fu_class = OOO_FU_ALU;
    decoded_src1_sel = OOO_SRC_REG;   // operand_a = rs1 value = csr_src
    decoded_src2_sel = OOO_SRC_IMM;   // no rs2 dependency
    decoded_csr_op = cop; decoded_csr_addr = csr_addr_t'(caddr);
    decoded_csr_zimm = csr_zimm_t'(rs1);   // zimm == rs1 field (instr[19:15])
    fire();
  endtask

  int drain;
  task automatic drain_rob();
    drain = 0;
    while (dut.u_rob.count_q !== '0 && drain < 400) begin @(posedge clk); drain++; end
    if (dut.u_rob.count_q !== '0)
      $fatal(1, "drain stuck: ROB never emptied (count=%0d)", dut.u_rob.count_q);
    repeat (2) @(posedge clk);
  endtask

  initial begin
    $display("[tb_rv32i_ss_core_csr] starting");
    reset_dut();
    check_word("reset: mtvec==0", dut.u_csr_file.mtvec_q, 32'd0);

    // ---- csrrw x2, mtvec, x1 (x1=0x123) -> mtvec=0x123, x2=old(0) --------
    disp_addi(32'h1000, 1, 0, 32'h0000_0123);
    disp_csr (32'h1004, CSR_RW, MTVEC, 2, 1);
    drain_rob();
    check_word("T1 csrrw: mtvec <- x1",        dut.u_csr_file.mtvec_q, 32'h0000_0123);
    check_word("T1 csrrw: x2 <- old mtvec(0)", creg(2),                32'h0000_0000);

    // ---- csrrs x3, mtvec, x0 -> pure read, WRITE SUPPRESSED ---------------
    disp_csr (32'h1008, CSR_RS, MTVEC, 3, 0);   // rs1=x0 -> zimm=0 -> no write
    drain_rob();
    check_word("T2 csrrs x0: x3 <- old mtvec", creg(3),                32'h0000_0123);
    check_word("T2 csrrs x0: mtvec UNCHANGED", dut.u_csr_file.mtvec_q, 32'h0000_0123);

    // ---- csrrs x5, mtvec, x4 (x4=0x0F0) -> mtvec |= 0x0F0, x5=old ---------
    disp_addi(32'h100C, 4, 0, 32'h0000_00F0);
    disp_csr (32'h1010, CSR_RS, MTVEC, 5, 4);
    drain_rob();
    check_word("T3 csrrs: mtvec |= x4",        dut.u_csr_file.mtvec_q, 32'h0000_01F3);
    check_word("T3 csrrs: x5 <- old mtvec",    creg(5),                32'h0000_0123);

    // ---- csrrc x7, mtvec, x6 (x6=0x0F0) -> mtvec &= ~0x0F0, x7=old --------
    disp_addi(32'h1014, 6, 0, 32'h0000_00F0);
    disp_csr (32'h1018, CSR_RC, MTVEC, 7, 6);
    drain_rob();
    check_word("T4 csrrc: mtvec &= ~x6",       dut.u_csr_file.mtvec_q, 32'h0000_0103);
    check_word("T4 csrrc: x7 <- old mtvec",    creg(7),                32'h0000_01F3);

    // ---- position-1 CSR stops the prefix and clears only at own commit --
    // Hold an older ordinary instruction on x8 while the CSR completes at
    // head+1. Releasing x8 must commit only the older instruction; the CSR
    // remains serialized for that edge, then commits from the head next cycle.
    force dut.u_prf.ready_q[8] = 1'b0;
    disp_addi(32'h101C, 9, 8, 32'h0000_0009);
    disp_csr (32'h1020, CSR_RW, MTVEC, 10, 1);
    drain = 0;
    while (!(dut.rob_commit_valid[1] && dut.rob_commit_is_csr[1]) &&
           (drain < 100)) begin
      @(posedge clk);
      drain++;
    end
    if (!(dut.rob_commit_valid[1] && dut.rob_commit_is_csr[1]))
      $fatal(1, "T5 CSR never reached completed position 1 (rob_count=%0d inflight=%0b)",
             dut.u_rob.count_q, dut.csr_inflight_q);
    force dut.u_prf.ready_q[8] = 1'b1;
    drain = 0;
    while ((slot1_csr_stop_n != 1) && (drain < 100)) begin
      @(posedge clk);
      drain++;
    end
    if (slot1_csr_stop_n != 1)
      $fatal(1, "T5 CSR position-1 prefix stop was not observed");
    @(negedge clk);
    check_word("T5 position-1 CSR challenged the prefix",
               word_t'(slot1_csr_stop_n), 32'd1);
    check_word("T5 older instruction committed alone", creg(9), 32'h0000_0009);
    check_word("T5 CSR state unchanged on prefix-stop edge",
               dut.u_csr_file.mtvec_q, 32'h0000_0103);
    check_word("T5 csr_inflight remains set until CSR's own commit",
               word_t'(dut.csr_inflight_q), 32'd1);
    drain_rob();
    release dut.u_prf.ready_q[8];
    check_word("T5 CSR commits from head on following cycle",
               dut.u_csr_file.mtvec_q, 32'h0000_0123);
    check_word("T5 CSR rd receives prior value", creg(10), 32'h0000_0103);
    check_word("T5 csr_inflight clears at CSR commit",
               word_t'(dut.csr_inflight_q), 32'd0);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_core_csr] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_core_csr] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
