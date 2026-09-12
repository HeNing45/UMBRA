`timescale 1ns/1ps

// =============================================================================
// tb_rv32i_ss_core_csr_dep -- natural proof that CSR encoding fields create
// only real register dependencies.
//
// Each case runs through the real frontend and core. An older DIV owns x3 for
// many cycles. The following CSR uses mtval (0x343), whose encoded csr[4:0]
// aliases x3 in the instruction's rs2 bit positions. Immediate CSR forms also
// use zimm=3, so their encoded rs1 bit positions alias x3 as well.
//
// The CSR must issue while the DIV result/tag is still unready:
//   * register forms consume rs1 only;
//   * immediate forms consume the separately carried zimm;
//   * no CSR instruction consumes the encoded rs2 field.
//
// Architectural checks then prove that removing the phantom dependencies did
// not weaken precise CSR behavior: DIV, CSR rd, CSR state, and a younger marker
// all commit with their expected values. The six cases cover CSRRW/S/C and all
// three immediate forms.
// =============================================================================

module tb_rv32i_ss_core_csr_dep;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;

  localparam time CLK_PERIOD = 10ns;
  localparam logic [11:0] CSR_MTVAL = 12'h343;

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

  logic redirect_valid;
  word_t redirect_target;
  logic [1:0] commit_fire, commit_rd_wen;
  commit_order_t commit_order;
  word_t [1:0] commit_pc, commit_inst, commit_wdata;
  arch_reg_t [1:0] commit_rd;

  logic dmem_valid, dmem_we, dmem_ready, dmem_rvalid;
  logic [3:0] dmem_be;
  word_t dmem_addr, dmem_wdata, dmem_rdata;

  int checks;
  int scenarios;
  int csr_issue_count;
  int marker_count;
  integer commit_slot;
  bit case_active;
  bit case_imm_form;
  bit early_issue_seen;
  bit dependency_contract_seen;
  bit encoded_alias_unready_seen;
  bit marker_seen;
  word_t marker_value;

  assign imem_rdata = {
    imem[{imem_addr[9:3], 1'b1}],
    imem[{imem_addr[9:3], 1'b0}]
  };

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
  always #(CLK_PERIOD/2) clk = ~clk;

  initial begin
    repeat (6000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_csr_dep exceeded 6000 cycles");
  end

  function automatic word_t enc_addi(input int rd, input int rs1,
                                     input int imm);
    logic [11:0] imm12;
    imm12 = imm[11:0];
    enc_addi = {imm12, rs1[4:0], 3'b000, rd[4:0], 7'b0010011};
  endfunction

  function automatic word_t enc_div(input int rd, input int rs1,
                                    input int rs2);
    enc_div = {7'b0000001, rs2[4:0], rs1[4:0], 3'b100,
               rd[4:0], 7'b0110011};
  endfunction

  function automatic word_t enc_csr(input int rd, input logic [11:0] csr,
                                    input int src_field,
                                    input logic [2:0] funct3);
    enc_csr = {csr, src_field[4:0], funct3, rd[4:0], 7'b1110011};
  endfunction

  function automatic word_t creg(input int regno);
    phys_reg_t p;
    p = u_core.u_rename.committed_map_q[regno];
    creg = u_core.u_prf.regs_q[p];
  endfunction

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
      $fatal(1, "CSR dependency check failed: %s", label);
  endtask

  task automatic fill_nops();
    for (int i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  endtask

  task automatic clear_case_observers(input bit imm_form);
    case_active = 1'b0;
    case_imm_form = imm_form;
    csr_issue_count = 0;
    marker_count = 0;
    early_issue_seen = 1'b0;
    dependency_contract_seen = 1'b0;
    encoded_alias_unready_seen = 1'b0;
    marker_seen = 1'b0;
    marker_value = '0;
  endtask

  task automatic reset_dut(input bit imm_form);
    rst_n = 1'b0;
    dmem_ready = 1'b1;
    dmem_rvalid = 1'b0;
    dmem_rdata = '0;
    clear_case_observers(imm_form);
    repeat (4) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    case_active = 1'b1;
  endtask

  task automatic wait_for_marker(input word_t expected, input int limit);
    int waited;
    waited = 0;
    while (!marker_seen && (waited < limit)) begin
      @(posedge clk);
      waited++;
    end
    if (!marker_seen)
      $fatal(1, "CSR dependency marker timeout expected=%08h", expected);
    @(negedge clk);
    check("marker value", marker_value == expected);
  endtask

  task automatic run_case(input string name,
                          input logic [2:0] funct3,
                          input bit imm_form,
                          input word_t expected_mtval,
                          input word_t marker);
    int src_field;
    src_field = imm_form ? 3 : 7;

    fill_nops();
    // The padding lets x1/x2 become ready before the DIV is dispatched. DIV
    // therefore starts immediately with x7 in the next bundle; the CSR is
    // dispatched on the following beat while the nonzero division is active.
    imem[0]  = enc_addi(1, 0, 100);                 // dividend
    imem[1]  = enc_addi(2, 0, 7);                   // divisor
    imem[2]  = 32'h0000_0013;
    imem[3]  = 32'h0000_0013;
    imem[4]  = 32'h0000_0013;
    imem[5]  = 32'h0000_0013;
    imem[6]  = 32'h0000_0013;
    imem[7]  = 32'h0000_0013;
    imem[8]  = enc_addi(7, 0, 5);                   // true CSR rs1
    imem[9]  = enc_div(3, 1, 2);                    // long owner of x3
    imem[10] = enc_csr(4, CSR_MTVAL, src_field, funct3);
    imem[11] = enc_addi(31, 0, marker);             // post-CSR marker
    imem[12] = 32'h0000_006f;                       // jal x0,0

    reset_dut(imm_form);
    wait_for_marker(marker, 800);
    case_active = 1'b0;
    repeat (2) @(posedge clk);

    check({name, ": CSR issued exactly once"}, csr_issue_count == 1);
    check({name, ": CSR issued before DIV completion"}, early_issue_seen);
    check({name, ": source-select contract"}, dependency_contract_seen);
    check({name, ": encoded x3 alias was physically unready"},
          encoded_alias_unready_seen);
    check({name, ": marker committed exactly once"}, marker_count == 1);
    check({name, ": DIV result committed"}, creg(3) == 32'd14);
    check({name, ": CSR rd receives old mtval"}, creg(4) == 32'd0);
    check({name, ": mtval commit value"},
          u_core.u_csr_file.mtval_q == expected_mtval);
    check({name, ": free-list conserved"}, free_count_now() == 32);
    scenarios++;
  endtask

  always @(posedge clk) begin
    if (rst_n && case_active) begin
      if (u_core.csr_issue_fire) begin
        csr_issue_count = csr_issue_count + 1;

        if (u_core.muldiv_busy && !u_core.muldiv_complete.valid)
          early_issue_seen = 1'b1;

        if ((u_core.alu0_exec_entry.src2_sel == OOO_SRC_ZERO) &&
            (u_core.alu0_exec_entry.src1_sel ==
             (case_imm_form ? OOO_SRC_ZERO : OOO_SRC_REG)))
          dependency_contract_seen = 1'b1;

        // Both encoded fields name architectural x3 in the immediate cases;
        // register forms use x7 as the true rs1 but still encode x3 in rs2.
        if ((u_core.alu0_exec_entry.prs2 ==
             u_core.u_rename.spec_map_q[3]) &&
            !u_core.ready_vec[u_core.alu0_exec_entry.prs2] &&
            (!case_imm_form ||
             ((u_core.alu0_exec_entry.prs1 ==
               u_core.u_rename.spec_map_q[3]) &&
              !u_core.ready_vec[u_core.alu0_exec_entry.prs1])))
          encoded_alias_unready_seen = 1'b1;
      end

      for (commit_slot = 0; commit_slot < 2; commit_slot++) begin
        if (commit_fire[commit_slot] && commit_rd_wen[commit_slot] &&
            (commit_rd[commit_slot] == arch_reg_t'(31))) begin
          marker_count = marker_count + 1;
          marker_seen = 1'b1;
          marker_value = commit_wdata[commit_slot];
        end
      end
    end
  end

  initial begin
    checks = 0;
    scenarios = 0;
    rst_n = 1'b0;
    dmem_ready = 1'b1;
    dmem_rvalid = 1'b0;
    dmem_rdata = '0;

    run_case("CSRRW",  3'b001, 1'b0, 32'd5, 32'd41);
    run_case("CSRRS",  3'b010, 1'b0, 32'd5, 32'd42);
    run_case("CSRRC",  3'b011, 1'b0, 32'd0, 32'd43);
    run_case("CSRRWI", 3'b101, 1'b1, 32'd3, 32'd44);
    run_case("CSRRSI", 3'b110, 1'b1, 32'd3, 32'd45);
    run_case("CSRRCI", 3'b111, 1'b1, 32'd0, 32'd46);

    check("all six CSR forms executed", scenarios == 6);
    $display("[tb_rv32i_ss_core_csr_dep] PASS checks=%0d scenarios=%0d",
             checks, scenarios);
    $finish;
  end

endmodule
