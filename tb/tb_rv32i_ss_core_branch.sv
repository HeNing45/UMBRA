`timescale 1ns/1ps

// Directed branch/jump TB (frontend + core + imem), self-checking.
//
// Exercises checkpointed branch resolution and serialized jump resolution end
// to end: decode, imm-gen, redirect, recovery, and frontend PC update. Programs
// are tiny inline hex (0-based PCs); each sub-test resets the DUT, runs, and
// checks the committed (pc, rd, wdata) sequence captured from the core's
// exposed commit channel.
//
//   1. Untaken branch  -> fall-through commits, no skip
//   2. Taken branch    -> target commits, fall-through (wrong path) never does
//   3. JAL             -> redirect to pc+imm, rd = pc+4 (link)
//   4. JALR            -> target (rs1+imm)&~1, rd = pc+4 (link); tests bit-0 clear

module tb_rv32i_ss_core_branch;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;

  logic clk;
  logic rst_n;

  // frontend <-> core decoded packet
  logic          dec_valid, dec_ready;
  logic [1:0] dec_slot_valid;
  word_t [1:0] dec_pc, dec_instr;
  arch_reg_t [1:0] dec_rs1, dec_rs2, dec_rd;
  logic [1:0] dec_rd_we, dec_needs_checkpoint;
  ooo_op_class_e [1:0] dec_op_class;
  ooo_fu_class_e [1:0] dec_fu_class;
  muldiv_op_e [1:0]    dec_muldiv_op;
  alu_op_e [1:0]       dec_alu_op;
  br_type_e [1:0]      dec_branch_op;
  ooo_src_sel_e [1:0] dec_src1_sel, dec_src2_sel;
  word_t [1:0]         dec_imm;
  decoded_trap_t dec_trap;
  csr_op_e       dec_csr_op;
  csr_addr_t     dec_csr_addr;
  csr_zimm_t     dec_csr_zimm;
  logic [1:0]          dec_is_load;
  logic [1:0]          dec_is_store;
  mem_size_e [1:0]     dec_mem_size;
  logic [1:0]          dec_mem_unsigned;

  // core commit channel + redirect
  logic [1:0]          commit_fire;
  commit_order_t commit_order;
  word_t [1:0]         commit_pc, commit_inst, commit_wdata;
  arch_reg_t [1:0]     commit_rd;
  logic [1:0]          commit_rd_wen;
  logic          redirect_valid;
  word_t         redirect_target;

  // instruction memory
  word_t       imem_addr;
  word_t [1:0]       imem_rdata;
  logic imem_req_valid, imem_req_ready;
  word_t imem_req_addr;
  logic imem_resp_valid, imem_resp_ready;
  word_t [1:0] imem_resp_data;
  logic [31:0] imem [1024];
  assign imem_rdata = !$isunknown(imem_addr[11:3])
      ? {imem[{imem_addr[11:3], 1'b1}], imem[{imem_addr[11:3], 1'b0}]}
      : {32'h0000_0013, 32'h0000_0013};

  int errors = 0;
  int checks = 0;

  // commit capture (declared early; used by the watchdog + capture block below)
  word_t cap_pc [0:63];
  int    cap_rd [0:63];
  word_t cap_wd [0:63];
  int    cap_n;

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
    .bp_update_pc('0), .bp_update_taken(1'b0), .bp_update_target('0),
    .redirect_valid(redirect_valid), .redirect_target(redirect_target),
    .decoded_valid(dec_valid), .decoded_slot_valid(dec_slot_valid), .decoded_ready(dec_ready),
    .decoded_pc(dec_pc), .decoded_instr(dec_instr),
    .decoded_rs1(dec_rs1), .decoded_rs2(dec_rs2), .decoded_rd(dec_rd),
    .decoded_rd_we(dec_rd_we), .decoded_needs_checkpoint(dec_needs_checkpoint),
    .decoded_op_class(dec_op_class), .decoded_fu_class(dec_fu_class),
    .decoded_muldiv_op(dec_muldiv_op), .decoded_alu_op(dec_alu_op),
    .decoded_branch_op(dec_branch_op),
    .decoded_src1_sel(dec_src1_sel), .decoded_src2_sel(dec_src2_sel),
    .decoded_imm(dec_imm),
    .decoded_trap(dec_trap),
    .decoded_csr_op(dec_csr_op),
    .decoded_csr_addr(dec_csr_addr),
    .decoded_csr_zimm(dec_csr_zimm),
    .decoded_is_load(dec_is_load),
    .decoded_is_store(dec_is_store),
    .decoded_mem_size(dec_mem_size),
    .decoded_mem_unsigned(dec_mem_unsigned)
  );

  rv32i_ss_core u_core (
    .clk(clk), .rst_n(rst_n),
    .decoded_valid(dec_valid), .decoded_slot_valid(dec_slot_valid), .decoded_ready(dec_ready),
    .decoded_pc(dec_pc), .decoded_instr(dec_instr),
    .decoded_rs1(dec_rs1), .decoded_rs2(dec_rs2), .decoded_rd(dec_rd),
    .decoded_rd_we(dec_rd_we), .decoded_needs_checkpoint(dec_needs_checkpoint),
    .decoded_op_class(dec_op_class), .decoded_fu_class(dec_fu_class),
    .decoded_muldiv_op(dec_muldiv_op), .decoded_alu_op(dec_alu_op),
    .decoded_branch_op(dec_branch_op),
    .decoded_src1_sel(dec_src1_sel), .decoded_src2_sel(dec_src2_sel),
    .decoded_imm(dec_imm),
    .decoded_trap(dec_trap),
    .decoded_csr_op(dec_csr_op),
    .decoded_csr_addr(dec_csr_addr),
    .decoded_csr_zimm(dec_csr_zimm),
    .decoded_is_load(dec_is_load),
    .decoded_is_store(dec_is_store),
    .decoded_mem_size(dec_mem_size),
    .decoded_mem_unsigned(dec_mem_unsigned),
    .decoded_pred_taken('0),  // tie-off: static not-taken prediction
    .decoded_pred_target('0),
    .redirect_valid(redirect_valid), .redirect_target(redirect_target),
    .commit_fire(commit_fire), .commit_order(commit_order),
    .commit_pc(commit_pc), .commit_inst(commit_inst),
    .commit_rd(commit_rd), .commit_rd_wen(commit_rd_wen),
    .commit_wdata(commit_wdata)
  );

  initial clk = 1'b0;
  always #5 clk = ~clk;

  initial begin
    repeat (5000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_core_branch exceeded 5000 cycles (cap_n=%0d)", cap_n);
  end

  // ---- per-position commit capture (sampled at the retirement edge) ----
  integer commit_slot;
  always @(posedge clk) begin
    if (rst_n) begin
      for (commit_slot = 0; commit_slot < 2; commit_slot++) begin
        if (commit_fire[commit_slot]) begin
          if (cap_n < 64) begin
            cap_pc[cap_n] = commit_pc[commit_slot];
            cap_rd[cap_n] = commit_rd_wen[commit_slot]
                          ? int'(commit_rd[commit_slot]) : 0;
            cap_wd[cap_n] = commit_rd_wen[commit_slot]
                          ? commit_wdata[commit_slot] : 32'd0;
          end
          // Blocking update is intentional: a second same-cycle record must
          // append after the first rather than overwrite its array element.
          cap_n = cap_n + 1;
        end
      end
    end
  end

  task automatic clear_imem();
    for (int k = 0; k < 1024; k++) imem[k] = 32'h00000013;  // NOP
  endtask

  task automatic reset_and_run(input int n);
    cap_n = 0;
    rst_n = 1'b0;
    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    wait (cap_n >= n);
    @(posedge clk);
  endtask

  task automatic chk(input string nm, input int idx,
                     input word_t epc, input int erd, input word_t ewd);
    checks++;
    if (cap_pc[idx] !== epc || cap_rd[idx] !== erd || cap_wd[idx] !== ewd) begin
      $error("[%s #%0d] got pc=%08h rd=%0d wd=%08h  exp pc=%08h rd=%0d wd=%08h",
             nm, idx, cap_pc[idx], cap_rd[idx], cap_wd[idx], epc, erd, ewd);
      errors++;
    end
  endtask

  task automatic chk_absent(input string nm, input word_t wrong_pc, input int n);
    checks++;
    for (int k = 0; k < n; k++)
      if (cap_pc[k] === wrong_pc) begin
        $error("[%s] wrong-path pc=%08h committed at #%0d", nm, wrong_pc, k);
        errors++;
      end
  endtask

  initial begin
    $display("[tb_rv32i_ss_core_branch] starting");

    // ---------------- Test 1: untaken branch ----------------
    // 0x00 addi x1,x0,5 ; 0x04 addi x2,x0,7 ; 0x08 beq x1,x2,+8 (untaken) ;
    // 0x0c addi x3,x0,1 (fall-through commits) ; 0x10 addi x4,x0,2
    clear_imem();
    imem[0] = 32'h00500093;
    imem[1] = 32'h00700113;
    imem[2] = 32'h00208463;
    imem[3] = 32'h00100193;
    imem[4] = 32'h00200213;
    reset_and_run(5);
    chk("untaken", 0, 32'h00000000, 1, 32'd5);
    chk("untaken", 1, 32'h00000004, 2, 32'd7);
    chk("untaken", 2, 32'h00000008, 0, 32'd0);   // beq, no rd
    chk("untaken", 3, 32'h0000000c, 3, 32'd1);   // fall-through
    chk("untaken", 4, 32'h00000010, 4, 32'd2);

    // ---------------- Test 2: taken branch ----------------
    // 0x00 addi x1,x0,5 ; 0x04 addi x2,x0,5 ; 0x08 beq x1,x2,+8 (taken) ;
    // 0x0c addi x3,x0,99 (WRONG PATH) ; 0x10 addi x4,x0,2 (target)
    clear_imem();
    imem[0] = 32'h00500093;
    imem[1] = 32'h00500113;
    imem[2] = 32'h00208463;
    imem[3] = 32'h06300193;
    imem[4] = 32'h00200213;
    reset_and_run(4);
    chk("taken", 0, 32'h00000000, 1, 32'd5);
    chk("taken", 1, 32'h00000004, 2, 32'd5);
    chk("taken", 2, 32'h00000008, 0, 32'd0);     // beq taken
    chk("taken", 3, 32'h00000010, 4, 32'd2);     // target (0x0c skipped)
    chk_absent("taken", 32'h0000000c, 4);

    // ---------------- Test 3: JAL ----------------
    // 0x00 addi x1,x0,5 ; 0x04 jal x5,+8 (link x5=0x08, target 0x0c) ;
    // 0x08 addi x6,x0,99 (WRONG PATH) ; 0x0c addi x7,x0,2 (target)
    clear_imem();
    imem[0] = 32'h00500093;
    imem[1] = 32'h008002ef;
    imem[2] = 32'h06300313;
    imem[3] = 32'h00200393;
    reset_and_run(3);
    chk("jal", 0, 32'h00000000, 1, 32'd5);
    chk("jal", 1, 32'h00000004, 5, 32'h00000008);  // link = pc+4
    chk("jal", 2, 32'h0000000c, 7, 32'd2);          // target = pc+imm
    chk_absent("jal", 32'h00000008, 3);

    // ---------------- Test 4: JALR (tests (rs1+imm)&~1 bit-0 clear) ----------
    // 0x00 addi x1,x0,13 ; 0x04 jalr x5,x1,0 (link x5=0x08, target=(13)&~1=0x0c) ;
    // 0x08 addi x6,x0,99 (WRONG PATH) ; 0x0c addi x7,x0,2 (target)
    clear_imem();
    imem[0] = 32'h00d00093;
    imem[1] = 32'h000082e7;
    imem[2] = 32'h06300313;
    imem[3] = 32'h00200393;
    reset_and_run(3);
    chk("jalr", 0, 32'h00000000, 1, 32'd13);
    chk("jalr", 1, 32'h00000004, 5, 32'h00000008);  // link = pc+4
    chk("jalr", 2, 32'h0000000c, 7, 32'd2);          // target = (13)&~1 = 0x0c
    chk_absent("jalr", 32'h00000008, 3);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_core_branch] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_core_branch] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
