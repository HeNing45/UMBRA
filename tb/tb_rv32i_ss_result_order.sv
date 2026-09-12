`timescale 1ns/1ps
// Registered ALU result FIFO ordering under drain/refill then backpressure.
module tb_rv32i_ss_result_order;
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
    $fatal(1, "WATCHDOG: tb_rv32i_ss_result_order");
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


  int committed [0:15];
  int commits = 0;
  bit witnessed = 0;
  completion_packet_t held0, held1;
  always @(posedge clk) begin
    if (rst_n)
      for (int lane = 0; lane < 2; lane++) begin
        if (commit_fire[lane]) begin
          int n;
          n = int'((commit_pc[lane] - 32'h1000) >> 2);
          if (n < 0 || n >= 16) $fatal(1, "unexpected committed PC");
          committed[n]++;
          commits++;
          check_word("committed value", commit_wdata[lane], word_t'(n+1));
          check_word("committed destination", word_t'(commit_rd[lane]), word_t'(n+1));
        end
      end
  end

  initial begin
    $display("[tb_rv32i_ss_result_order] starting");
    clear_inputs();
    for (int n = 0; n < 16; n++) committed[n] = 0;
    rst_n = 0;
    repeat (3) @(negedge clk);
    rst_n = 1;
    fork
      begin : driver
        // Continuous independent pairs make both ALUs drain/refill on
        // consecutive edges, rotating the live result through both banks.
        for (int p = 0; p < 8; p++) begin
          @(negedge clk);
          decoded_valid = 1;
          decoded_slot_valid = 3;
          for (int lane = 0; lane < 2; lane++) begin
            decoded_pc[lane] = 32'h1000 + 4*word_t'(2*p+lane);
            decoded_instr[lane] = decoded_pc[lane];
            decoded_rs1[lane] = 0; decoded_rs2[lane] = 0;
            decoded_rd[lane] = areg(2*p+lane+1);
            decoded_rd_we[lane] = 1;
            decoded_src1_sel[lane] = OOO_SRC_REG;
            decoded_src2_sel[lane] = OOO_SRC_IMM;
            decoded_imm[lane] = word_t'(2*p+lane+1);
            decoded_op_class[lane] = OOO_OP_ALU;
            decoded_fu_class[lane] = OOO_FU_ALU;
            decoded_alu_op[lane] = fyp_cpu_pkg::ALU_ADD;
          end
          #1;
          while (!decoded_ready) @(negedge clk);
          @(posedge clk);
        end
        @(negedge clk);
        clear_inputs();
      end
      begin : stop_after_refill
        // Load-bearing assumption, proved at the actual transfer edge:
        // each bank1 result drains WHILE a fresh result fills bank0.
        do @(posedge clk); while (!(dut.alu0_exec_fire && dut.cdb_grant_alu0 &&
            dut.alu0_res_offer_idx && !dut.alu0_res_push_idx &&
            dut.alu1_exec_fire && dut.cdb_grant_alu1 &&
            dut.alu1_res_offer_idx && !dut.alu1_res_push_idx));
        #1;
        witnessed = 1;
        held0 = dut.alu0_complete;
        held1 = dut.alu1_complete;
        $display("RESULT_ORDER_WITNESS both drain-bank1/fill-bank0 pc=%h,%h",
                 dut.alu0_exec_entry.pc, dut.alu1_exec_entry.pc);
        @(negedge clk);
        force dut.holder_valid = 5'b0;
        // A fourth production may fill bank1 but MUST NOT overtake the
        // previously offered third production. No order-bit implementation
        // assumption in this oracle: an unaccepted offer must stay stable.
        @(posedge clk); #1;
        if (dut.alu0_complete !== held0 || dut.alu1_complete !== held1)
          $fatal(1, "RESULT_ORDER: unaccepted offer changed after other-bank fill");
        check_bit("both ALU0 result banks filled",
                  dut.alu0_res_q[0].valid && dut.alu0_res_q[1].valid, 1);
        check_bit("both ALU1 result banks filled",
                  dut.alu1_res_q[0].valid && dut.alu1_res_q[1].valid, 1);
        repeat (3) @(negedge clk);
        release dut.holder_valid;
      end
    join
    while (commits < 16) @(negedge clk);
    repeat (2) @(negedge clk);
    check_bit("drain/refill corner actually entered", witnessed, 1);
    for (int n = 0; n < 16; n++) begin
      check_word("each instruction commits exactly once", word_t'(committed[n]), 1);
      check_word("architectural result", `PRF.regs_q[`RN.committed_map_q[n+1]], word_t'(n+1));
    end
    if (errors) $fatal(1, "RESULT_ORDER: architectural checks failed");
    $display("[tb_rv32i_ss_result_order] PASS checks=%0d commits=%0d", checks, commits);
    $finish;
  end
endmodule
