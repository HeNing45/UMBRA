`timescale 1ns/1ps

// Registered operand/execute boundary (Sky130 timing v1).
//
// Reachability:
//   Newly required: an accepted uop lives in a registered input slot for at
//     least one cycle before exec_fire. issue_fire and exec_fire of one
//     identity never share a cycle (no fall-through).
//   Newly required: two live ALU result banks; CDB offers the older
//     production, both remain bypass-eligible.
// Store pending-data coverage belongs to tb_rv32i_ss_store_data_decouple.
// CDB selection is withheld (both transport and drain) until two live ALU0
// results coexist. Commit values/counts then prove retention without loss.
// Same-cycle consumers: exec_fire (ALU/AGEN/muldiv-start/branch/CSR),
// early_set at result registration, CDB offer, IQ fu_ready (input
// occupancy only). Recovery remains registered.

module tb_rv32i_ss_exec_boundary;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;

  localparam time CLK_PERIOD = 10ns;
  logic clk, rst_n;
  logic decoded_valid;
  logic [1:0] decoded_slot_valid, decoded_rd_we, decoded_needs_checkpoint;
  logic decoded_ready;
  word_t [1:0] decoded_pc, decoded_instr, decoded_imm;
  arch_reg_t [1:0] decoded_rs1, decoded_rs2, decoded_rd;
  ooo_op_class_e [1:0] decoded_op_class;
  ooo_fu_class_e [1:0] decoded_fu_class;
  alu_op_e [1:0] decoded_alu_op;
  br_type_e [1:0] decoded_branch_op;
  ooo_src_sel_e [1:0] decoded_src1_sel, decoded_src2_sel;
  muldiv_op_e [1:0] decoded_muldiv_op;
  decoded_trap_t decoded_trap;
  logic [1:0] decoded_is_load, decoded_is_store, decoded_mem_unsigned;
  mem_size_e [1:0] decoded_mem_size;
  logic redirect_valid;
  word_t redirect_target;
  logic [1:0] commit_fire, commit_rd_wen;
  commit_order_t commit_order;
  word_t [1:0] commit_pc, commit_inst, commit_wdata;
  arch_reg_t [1:0] commit_rd;

  int errors, checks;

  rv32i_ss_core dut (
    .clk(clk), .rst_n(rst_n),
    .decoded_valid(decoded_valid), .decoded_slot_valid(decoded_slot_valid),
    .decoded_ready(decoded_ready), .decoded_pc(decoded_pc),
    .decoded_instr(decoded_instr), .decoded_rs1(decoded_rs1),
    .decoded_rs2(decoded_rs2), .decoded_rd(decoded_rd),
    .decoded_rd_we(decoded_rd_we),
    .decoded_needs_checkpoint(decoded_needs_checkpoint),
    .decoded_op_class(decoded_op_class), .decoded_alu_op(decoded_alu_op),
    .decoded_branch_op(decoded_branch_op), .decoded_src1_sel(decoded_src1_sel),
    .decoded_src2_sel(decoded_src2_sel), .decoded_imm(decoded_imm),
    .decoded_fu_class(decoded_fu_class), .decoded_muldiv_op(decoded_muldiv_op),
    .decoded_trap(decoded_trap), .decoded_is_load(decoded_is_load),
    .decoded_is_store(decoded_is_store), .decoded_mem_size(decoded_mem_size),
    .decoded_mem_unsigned(decoded_mem_unsigned),
    .decoded_pred_taken('0), .decoded_pred_target('0),
    .decoded_csr_op('0), .decoded_csr_addr('0), .decoded_csr_zimm('0),
    .redirect_valid(redirect_valid), .redirect_target(redirect_target),
    .commit_fire(commit_fire), .commit_order(commit_order),
    .commit_pc(commit_pc), .commit_inst(commit_inst), .commit_rd(commit_rd),
    .commit_rd_wen(commit_rd_wen), .commit_wdata(commit_wdata)
  );

  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;
  initial begin
    repeat (2000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_exec_boundary");
  end

  `define ROB dut.u_rob
  `define RN  dut.u_rename
  `define PRF dut.u_prf

  function automatic arch_reg_t areg(input int v);
    areg = arch_reg_t'(v);
  endfunction

  task automatic check_bit(input string n, input logic g, input logic e);
    checks++;
    if (g !== e) begin $error("[%s] got=%0b exp=%0b", n, g, e); errors++; end
  endtask
  task automatic check_word(input string n, input word_t g, input word_t e);
    checks++;
    if (g !== e) begin $error("[%s] got=%0d exp=%0d", n, g, e); errors++; end
  endtask

  task automatic clear_inputs();
    decoded_valid = 1'b0; decoded_slot_valid = 2'b00;
    decoded_pc = '0; decoded_instr = '0;
    decoded_rs1 = '0; decoded_rs2 = '0; decoded_rd = '0;
    decoded_rd_we = '0; decoded_needs_checkpoint = '0;
    decoded_op_class = OOO_OP_ALU; decoded_alu_op = fyp_cpu_pkg::ALU_ADD;
    decoded_branch_op = rv32i_pipeline_pkg::BR_NONE;
    decoded_src1_sel = OOO_SRC_REG; decoded_src2_sel = OOO_SRC_IMM;
    decoded_imm = '0; decoded_fu_class = OOO_FU_ALU;
    decoded_muldiv_op = rv32i_pipeline_pkg::MD_NONE;
    decoded_trap = '0; decoded_is_load = '0; decoded_is_store = '0;
    decoded_mem_size = fyp_cpu_pkg::MEM_W; decoded_mem_unsigned = '0;
  endtask

  task automatic disp(input word_t pc, input arch_reg_t rd, input word_t imm);
    @(negedge clk);
    decoded_valid = 1'b1; decoded_slot_valid = 2'b01;
    decoded_pc[0] = pc; decoded_instr[0] = pc;
    // Third instruction depends on the first while CDB is withheld. A stale
    // PRF read would commit 3 instead of 4, so holder bypass is consequential.
    decoded_rs1[0] = (pc == 32'h1008) ? areg(1) : areg(0);
    decoded_rd[0] = rd; decoded_rd_we[0] = 1'b1;
    decoded_imm[0] = imm; decoded_fu_class[0] = OOO_FU_ALU;
    decoded_src2_sel[0] = OOO_SRC_IMM;
    #1;
    while (decoded_ready !== 1'b1) @(negedge clk);
    @(posedge clk);
    @(negedge clk);
    decoded_valid = 1'b0; decoded_slot_valid = 2'b00;
  endtask

  bit saw_in_before_exec;
  bit saw_fall_through;
  bit saw_two_results;
  bit saw_select_stall;
  bit saw_holder_operand;
  int cycle_count;
  int selected_cycle [0:11];
  int selected_count [0:11];
  int accepted_cycle [0:11];
  int executed_count [0:11];
  int committed_count [0:11];

  always @(posedge clk) begin
    if (rst_n) begin
      cycle_count++;
      // Sample transfers before the edge updates the input banks. Record the
      // actual acceptance edge, not the next cycle's combinational fire.
      for (int n = 0; n < 12; n++) begin
        if ((dut.alu0_exec_fire && dut.alu0_exec_entry.pc == 32'h1000 + 4*n) ||
            (dut.alu1_exec_fire && dut.alu1_exec_entry.pc == 32'h1000 + 4*n)) begin
          if (accepted_cycle[n] < 0 || accepted_cycle[n] >= cycle_count)
            $fatal(1, "execution without an earlier input acceptance");
          saw_in_before_exec = 1'b1;
          executed_count[n]++;
        end
        if ((dut.alu0_issue_fire && dut.alu0_issue_entry.pc == 32'h1000 + 4*n) ||
            (dut.alu1_issue_fire && dut.alu1_issue_entry.pc == 32'h1000 + 4*n)) begin
          if (selected_cycle[n] < 0 || selected_cycle[n] >= cycle_count)
            $fatal(1, "operand capture without earlier registered selection");
          accepted_cycle[n] = cycle_count;
        end
        for (int lane = 0; lane < 2; lane++)
          if (dut.iq_select_accept && dut.iq_select_valid[lane] &&
              dut.iq_select_entry[lane].pc == 32'h1000 + 4*n) begin
            selected_cycle[n] = cycle_count;
            selected_count[n]++;
          end
        for (int lane = 0; lane < 2; lane++)
          if (commit_fire[lane] && commit_pc[lane] == 32'h1000 + 4*n) begin
            committed_count[n]++;
            check_word("commit value", commit_wdata[lane], word_t'(n+1+(n == 2)));
            check_bit("commit writes destination", commit_rd_wen[lane], 1'b1);
            check_word("commit destination", word_t'(commit_rd[lane]), word_t'(n+1));
          end
      end
      if ((|dut.issue_valid) && !dut.issue_accept && !dut.branch_recover_req)
        saw_select_stall = 1'b1;
      if (dut.alu0_issue_fire && dut.alu0_issue_entry.pc == 32'h1008 &&
          dut.alu0_operand_a == 32'd1 &&
          `PRF.regs_q[`RN.spec_map_q[1]] == 0 &&
          dut.alu0_complete.valid)
        saw_holder_operand = 1'b1;
      if (dut.alu0_issue_fire && dut.alu0_exec_fire &&
          (dut.alu0_issue_entry.rob_idx == dut.alu0_exec_entry.rob_idx) &&
          (dut.alu0_issue_entry.rob_seq == dut.alu0_exec_entry.rob_seq))
        saw_fall_through = 1'b1;
      if (dut.alu0_res_q[0].valid && dut.alu0_res_q[0].live &&
          dut.alu0_res_q[1].valid && dut.alu0_res_q[1].live)
        saw_two_results = 1'b1;
    end
  end

  initial begin
    $display("[tb_rv32i_ss_exec_boundary] starting");
    clear_inputs();
    cycle_count = 0;
    for (int n = 0; n < 12; n++) begin
      selected_cycle[n] = -1;
      selected_count[n] = 0;
      accepted_cycle[n] = -1;
      executed_count[n] = 0;
      committed_count[n] = 0;
    end
    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);
    saw_in_before_exec = 1'b0;
    saw_fall_through = 1'b0;
    saw_two_results = 1'b0;
    saw_select_stall = 1'b0;
    saw_holder_operand = 1'b0;

    force dut.holder_valid = 5'b0;
    disp(32'h1000, areg(1), 32'd1);
    @(posedge clk); // first IQ -> select_q transfer
    @(negedge clk);
    // Conservative unavailable-unit injection AFTER binding. It never
    // fabricates space; the selected instruction must wait with full identity.
    force dut.alu0_fu_ready = 1'b0;
    repeat (3) @(negedge clk);
    release dut.alu0_fu_ready;
    for (int n = 1; n < 12; n++)
      disp(32'h1000 + 4*n, areg(n+1), word_t'(n+1));
    repeat (6) @(negedge clk);
    check_bit("two live ALU0 result banks entered", saw_two_results, 1'b1);
    check_bit("input precedes execution", saw_in_before_exec, 1'b1);
    check_bit("select pair held behind unavailable bound unit", saw_select_stall, 1'b1);
    check_bit("live-holder operand used with PRF still stale", saw_holder_operand, 1'b1);
    release dut.holder_valid;
    clear_inputs();
    while (`ROB.count_q != 0) @(posedge clk);
    repeat (2) @(posedge clk);

    check_bit("no fall-through of one identity", saw_fall_through, 1'b0);
    check_word("x1", `PRF.regs_q[`RN.committed_map_q[1]], 32'd1);
    check_word("x2", `PRF.regs_q[`RN.committed_map_q[2]], 32'd2);
    check_word("x3", `PRF.regs_q[`RN.committed_map_q[3]], 32'd4);
    check_word("x4", `PRF.regs_q[`RN.committed_map_q[4]], 32'd4);
    for (int n = 0; n < 12; n++) begin
      check_word("exactly one selection", word_t'(selected_count[n]), 32'd1);
      check_word("exactly one execution", word_t'(executed_count[n]), 32'd1);
      check_word("exactly one commit", word_t'(committed_count[n]), 32'd1);
    end

    if (errors == 0) begin
      $display("[tb_rv32i_ss_exec_boundary] PASS checks=%0d two_res=%0b",
               checks, saw_two_results);
      $finish;
    end else
      $fatal(1, "[tb_rv32i_ss_exec_boundary] FAIL errors=%0d", errors);
  end
endmodule
