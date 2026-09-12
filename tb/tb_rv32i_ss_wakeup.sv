`timescale 1ns/1ps

// tb_rv32i_ss_wakeup — accepted-writeback non-ALU wakeup, fixed-latency
// ALU wakeup and value-bypass tests. A wrong forwarded value must change an
// architectural result, not merely an internal timing observation.
//
// One program through umbra_ss_cpu_top and the data-memory model covers:
//   - A twelve-operation dependent ALU chain. Entry counters observe selection
//     while the producer value remains in a registered holder. The cycle pin
//     checks throughput and x5=13 makes the holder bypass consequential.
//   - ALU to store data to load-back. Store data consumes prs2 separately from
//     the immediate AGEN operand. Stale data would corrupt both memory and the
//     loaded value. Load dependents stay busy through grant and become ready
//     only after accepted writeback supplies their value.
//   - A wrong-path producer under a REM-dependent branch. Recovery kills it,
//     the correct path reuses its physical register, and the dependent must
//     receive x11=6 while the wrong-path x10 write is absent.
//
// Grant-triggered non-ALU wakeup violates the busy-after-grant check; omitting
// accepted ready updates violates ready-after-accept. The execute-boundary and
// holder-recovery testbenches separately check consequential ALU bypass.

module tb_rv32i_ss_wakeup;
  import rv32i_ss_pkg::*;

  localparam time CLK_PERIOD = 10ns;

  logic clk;
  logic rst_n;

  word_t imem_addr;
  word_t [1:0] imem_rdata;
  logic imem_req_valid, imem_req_ready;
  word_t imem_req_addr;
  logic imem_resp_valid, imem_resp_ready;
  word_t [1:0] imem_resp_data;
  logic [31:0] imem [0:63];
  assign imem_rdata = {imem[{imem_addr[7:3], 1'b1}], imem[{imem_addr[7:3], 1'b0}]};

  rv32i_ss_imem_zero_latency_adapter u_imem_adapter (
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr), .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready), .imem_resp_data(imem_resp_data),
    .line_addr(imem_addr), .line_data(imem_rdata)
  );

  word_t dmem_addr, dmem_wdata, dmem_rdata;
  logic  dmem_valid, dmem_we;
  logic [3:0] dmem_be;

  int errors = 0;
  int checks = 0;

  umbra_ss_cpu_top u_cpu (
    .clk        (clk),
    .rst_n      (rst_n),
    .imem_req_valid  (imem_req_valid),
    .imem_req_ready  (imem_req_ready),
    .imem_req_addr   (imem_req_addr),
    .imem_resp_valid (imem_resp_valid),
    .imem_resp_ready (imem_resp_ready),
    .imem_resp_data  (imem_resp_data),
    .dmem_valid (dmem_valid),
    .dmem_we    (dmem_we),
    .dmem_be    (dmem_be),
    .dmem_addr  (dmem_addr),
    .dmem_wdata (dmem_wdata),
    .dmem_ready (1'b1),
    .dmem_rvalid(1'b1),
    .dmem_rdata (dmem_rdata)
  );

  ooo_dmem_model #(.MEM_WORDS(256), .MEM_MSB(9)) u_dmem (
    .clk(clk), .rst_n(rst_n),
    .addr(dmem_addr), .rdata(dmem_rdata),
    .we(dmem_we), .be(dmem_be), .wdata(dmem_wdata),
    .tohost_addr(32'hFFFF_FFFC), .tohost_full_addr(32'hFFFF_FFFC),
    .tohost_we(), .tohost_val()
  );

  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;

  initial begin
    repeat (2000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_wakeup exceeded 2000 cycles");
  end

  `define CORE u_cpu.u_core
  `define RN   u_cpu.u_core.u_rename
  `define PRF  u_cpu.u_core.u_prf

  task automatic check_bit(input string name, input logic got, input logic exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got=%0b exp=%0b", name, got, exp);
      errors++;
    end
  endtask

  task automatic check_int(input string name, input int got, input int exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got=%0d exp=%0d", name, got, exp);
      errors++;
    end
  endtask

  task automatic check_arch(input string name, input int r, input word_t exp);
    automatic phys_reg_t p = `RN.committed_map_q[r];
    checks++;
    if (`PRF.regs_q[p] !== exp) begin
      $error("[%s] x%0d=%08h (via p%0d) expected %08h",
             name, r, `PRF.regs_q[p], p, exp);
      errors++;
    end
  endtask

  // ---------------- monitors ----------------
  int cycles;
  int n_load_grant, n_md_grant, n_load_accept, n_md_accept;
  completion_packet_t load_packet, md_packet;
  int n_byp;            // issue cycles whose position-0 REG operand was
                        // served by an accepted transit lane (bypass entered)
  int n_issue_set;      // ALU-issue ready-set edges
  int n_holder_byp;     // issued REG operands served by a live ALU holder
  int n_holder_select;  // selection while producer is still in its ALU holder
  bit early_in_shadow;  // wrong-path ALU executed before branch recovery
  int n_branch_recover;
  bit marker_seen;

  always @(posedge clk) begin
    if (rst_n === 1'b1) begin
      if (!marker_seen) cycles++;
      if ((n_branch_recover == 0) &&
          ((`CORE.alu0_exec_fire && `CORE.alu0_exec_entry.pc == 32'h50 &&
            `PRF.early_set[2]) ||
           (`CORE.alu1_exec_fire && `CORE.alu1_exec_entry.pc == 32'h50 &&
            `PRF.early_set[3]))) early_in_shadow = 1'b1;
      if (|`PRF.early_set[3:2]) n_issue_set++;
      if (`CORE.issue_fire[0] && (`CORE.issue_src1_sel[0] == OOO_SRC_REG) &&
          (((`CORE.rob_wb_accept[0] === 1'b1) && `CORE.cdb_q[0].rd_wen &&
            (`CORE.cdb_q[0].pdst == `CORE.issue_prs1[0])) ||
           ((`CORE.rob_wb_accept[1] === 1'b1) && `CORE.cdb_q[1].rd_wen &&
            (`CORE.cdb_q[1].pdst == `CORE.issue_prs1[0]))))
        n_byp++;
      for (int p = 0; p < 2; p++) begin
        // select_q delays operand capture: under unstalled CDB service the
        // producer has moved to transit/PRF by then. Pin EARLY SELECTION here;
        // actual holder-data bypass remains consequentially required by
        // tb_rv32i_ss_exec_boundary's withheld-CDB dependent ADDI.
        begin
          iq_entry_t selected;
          selected = `CORE.iq_select_entry[p];
          if (`CORE.iq_select_accept && `CORE.iq_select_valid[p] &&
              selected.src1_sel == OOO_SRC_REG &&
              ((`CORE.alu0_holder_live_q && `CORE.alu0_complete.valid &&
                `CORE.alu0_complete.rd_wen && `CORE.alu0_complete.pdst == selected.prs1) ||
               (`CORE.alu1_holder_live_q && `CORE.alu1_complete.valid &&
                `CORE.alu1_complete.rd_wen && `CORE.alu1_complete.pdst == selected.prs1)))
            n_holder_select++;
        end
        for (int s = 0; s < 2; s++) begin
          if (`CORE.issue_fire[p] &&
              ((s == 0) ? (`CORE.issue_src1_sel[p] == OOO_SRC_REG)
                        : (`CORE.issue_src2_sel[p] == OOO_SRC_REG)) &&
              (((`CORE.alu0_holder_live_q === 1'b1) &&
                `CORE.alu0_complete.valid && `CORE.alu0_complete.rd_wen &&
                (`CORE.alu0_complete.pdst ==
                 ((s == 0) ? `CORE.issue_prs1[p] : `CORE.issue_prs2[p]))) ||
               ((`CORE.alu1_holder_live_q === 1'b1) &&
                `CORE.alu1_complete.valid && `CORE.alu1_complete.rd_wen &&
                (`CORE.alu1_complete.pdst ==
                 ((s == 0) ? `CORE.issue_prs1[p] : `CORE.issue_prs2[p]))))) begin
            n_holder_byp++;
          end
        end
      end
      if (`CORE.branch_candidate[0].recover_valid ||
          `CORE.branch_candidate[1].recover_valid)
        n_branch_recover++;
      if ((`CORE.commit_fire[0] && `CORE.commit_rd_wen[0] &&
           (`CORE.commit_rd[0] == 14) && (`CORE.commit_wdata[0] == 32'd14)) ||
          (`CORE.commit_fire[1] && `CORE.commit_rd_wen[1] &&
           (`CORE.commit_rd[1] == 14) && (`CORE.commit_wdata[1] == 32'd14)))
        marker_seen = 1'b1;
    end
  end

  // Observe the two sides of each real clock edge. Capture identities before
  // NBA; checking the producer's next-cycle signals would inspect a new owner.
  // The real load/REM and their consumers below make failure consequential.
  always @(posedge clk) begin : accepted_wakeup_monitor
    logic load_grant, md_grant, load_accept, md_accept;
    completion_packet_t beat;
    if (rst_n === 1'b1) begin
      load_grant = `CORE.cdb_grant_lq;
      md_grant = `CORE.cdb_grant_muldiv;
      load_accept = 1'b0;
      md_accept = 1'b0;
      if (load_grant) begin
        load_packet = `CORE.lq_complete;
        n_load_grant++;
        check_bit("load busy before grant", `PRF.ready_vec[load_packet.pdst], 1'b0);
      end
      if (md_grant) begin
        md_packet = `CORE.muldiv_complete;
        n_md_grant++;
        check_bit("muldiv busy before grant", `PRF.ready_vec[md_packet.pdst], 1'b0);
      end
      for (int lane = 0; lane < 2; lane++) begin
        beat = `CORE.cdb_q[lane];
        if (`CORE.rob_wb_accept[lane] && beat.rd_wen) begin
          if (n_load_grant > 0 && beat.rob_idx == load_packet.rob_idx &&
              beat.rob_seq == load_packet.rob_seq) begin
            load_accept = 1'b1;
            n_load_accept++;
            check_bit("load busy until accept edge", `PRF.ready_vec[beat.pdst], 1'b0);
          end
          if (n_md_grant > 0 && beat.rob_idx == md_packet.rob_idx &&
              beat.rob_seq == md_packet.rob_seq) begin
            md_accept = 1'b1;
            n_md_accept++;
            check_bit("muldiv busy until accept edge", `PRF.ready_vec[beat.pdst], 1'b0);
          end
        end
      end
      #1;
      if (load_grant)
        check_bit("load grant must not wake", `PRF.ready_vec[load_packet.pdst], 1'b0);
      if (md_grant)
        check_bit("muldiv grant must not wake", `PRF.ready_vec[md_packet.pdst], 1'b0);
      if (load_accept) begin
        check_bit("load accepted write wakes", `PRF.ready_vec[load_packet.pdst], 1'b1);
        check_int("load accepted write value", `PRF.regs_q[load_packet.pdst], 13);
      end
      if (md_accept) begin
        check_bit("muldiv accepted write wakes", `PRF.ready_vec[md_packet.pdst], 1'b1);
        check_int("muldiv accepted write value", `PRF.regs_q[md_packet.pdst], 1);
      end
    end
  end

  integer i;
  int wait_i;

  initial begin
    $display("[tb_rv32i_ss_wakeup] starting");
    cycles = 0; n_byp = 0; n_issue_set = 0; n_holder_byp = 0;
    n_load_grant = 0; n_md_grant = 0; n_load_accept = 0; n_md_accept = 0;
    load_packet = '0; md_packet = '0;
    n_holder_select = 0;
    n_branch_recover = 0;
    early_in_shadow = 1'b0; marker_seen = 1'b0;

    for (i = 0; i < 64; i = i + 1) imem[i] = 32'h0000_0013;

    imem[ 0] = 32'h00100293;  // 0x00 addi x5,x0,1
    imem[ 1] = 32'h00128293;  // 0x04 addi x5,x5,1   (chain 1)
    imem[ 2] = 32'h00128293;  // 0x08 chain 2
    imem[ 3] = 32'h00128293;  // 0x0c chain 3
    imem[ 4] = 32'h00128293;  // 0x10 chain 4
    imem[ 5] = 32'h00128293;  // 0x14 chain 5
    imem[ 6] = 32'h00128293;  // 0x18 chain 6
    imem[ 7] = 32'h00128293;  // 0x1c chain 7
    imem[ 8] = 32'h00128293;  // 0x20 chain 8
    imem[ 9] = 32'h00128293;  // 0x24 chain 9
    imem[10] = 32'h00128293;  // 0x28 chain 10
    imem[11] = 32'h00128293;  // 0x2c chain 11
    imem[12] = 32'h00128293;  // 0x30 chain 12 -> x5 = 13
    imem[13] = 32'h10502023;  // 0x34 sw x5,0x100(x0)  (store DATA fed by chain)
    imem[14] = 32'h10002303;  // 0x38 lw x6,0x100(x0)
    imem[15] = 32'h00130393;  // 0x3c addi x7,x6,1     (load -> use)
    imem[16] = 32'h00700B93;  // 0x40 addi x23,x0,7
    imem[17] = 32'h00300C13;  // 0x44 addi x24,x0,3
    imem[18] = 32'h038BEB33;  // 0x48 rem x22,x23,x24  (slow feeder, =1)
    imem[19] = 32'h000B1A63;  // 0x4c bne x22,x0,+0x14 -> 0x60 (taken, cold-mispredicted)
    imem[20] = 32'h06300493;  // 0x50 WRONG-PATH: addi x9,x0,99 (grants while live)
    imem[21] = 32'h00148513;  // 0x54 WRONG-PATH: addi x10,x9,1 (wrong-path bypass)
    imem[22] = 32'h0000006F;  // 0x58 wrong-path self-loop
    imem[24] = 32'h00500493;  // 0x60 T: addi x9,x0,5 (TRUE producer, realloc lineage)
    imem[25] = 32'h00148593;  // 0x64 addi x11,x9,1   (must see 6)
    imem[26] = 32'h00E00713;  // 0x68 addi x14,x0,14  (marker)
    imem[27] = 32'h0000006F;  // 0x6c self-loop

    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;

    wait_i = 0;
    while (!marker_seen) begin
      @(posedge clk);
      wait_i++;
      if (wait_i > 1500) $fatal(1, "stuck: end marker never committed");
    end
    repeat (4) @(posedge clk);

    // ---- entered pins ----
    check_int("load grant entered", n_load_grant, 1);
    check_int("load accepted write entered", n_load_accept, 1);
    check_int("muldiv grant entered", n_md_grant, 1);
    check_int("muldiv accepted write entered", n_md_accept, 1);
    check_bit("transit bypass entered (operand served mid-transit)", n_byp > 0, 1'b1);
    check_bit("issue-driven ready-set entered", n_issue_set > 0, 1'b1);
    check_bit("selection wakes while ALU holder is live", n_holder_select > 0, 1'b1);
    check_bit("P3: wrong-path ALU early-set before recovery", early_in_shadow, 1'b1);
    check_int("P3: exactly one branch recovery", n_branch_recover, 1);

    // The registered selector's expected cycle count is 66.
    // Both non-ALU wake events move one edge; overlap in this out-of-order
    // program hides one of those cycles (measured total 67). Edge-event
    // checks above pin the latency contract independently of this fingerprint.
    $display("[tb_rv32i_ss_wakeup] MEASURE cycles=%0d load_accept=%0d md_accept=%0d",
             cycles, n_load_accept, n_md_accept);
    check_int("end-to-end cycles (accepted non-ALU wakeup)", cycles, 67);

    // ---- architectural consequence ----
    check_arch("P1 chain sum",            5,  32'd13);
    check_arch("P2 loaded-back store",    6,  32'd13);
    check_arch("P2 load->use",            7,  32'd14);
    check_arch("P3 rem result",           22, 32'd1);
    check_arch("P3 true producer",        9,  32'd5);
    check_arch("P3 true consumer",        11, 32'd6);
    check_arch("P3 wrong-path consumer rolled back", 10, 32'd0);
    check_arch("marker",                  14, 32'd14);
    checks++;
    if (u_dmem.mem[32'h100 >> 2] !== 32'd13) begin
      $error("[P2 memory word] got=%08h exp=0000000d", u_dmem.mem[32'h100 >> 2]);
      errors++;
    end

    if (errors == 0) begin
      $display("[tb_rv32i_ss_wakeup] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_wakeup] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
