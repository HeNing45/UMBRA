`timescale 1ns/1ps

// registered cross-line formation.
//
// A solo CSR at 0x00 consumes only the lower word of the first registered
// line. With the following line also registered, the frontend must dispatch
// {0x04,0x08}; later pairs roll the same state across two more boundaries.
// Exact packet identity plus final committed register values make a skipped,
// duplicated, or wrong-half follower architecturally consequential.
module tb_rv32i_ss_crossline;
  import rv32i_ss_pkg::*;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  logic        imem_req_valid;
  logic        imem_req_ready;
  word_t       imem_req_addr;
  logic        imem_resp_valid;
  logic        imem_resp_ready;
  word_t [1:0] imem_resp_data;
  word_t       imem_addr;
  word_t [1:0] imem_rdata;
  word_t       imem [0:255];

  logic        dmem_valid;
  logic        dmem_we;
  logic [3:0]  dmem_be;
  word_t       dmem_addr;
  word_t       dmem_wdata;
  logic [1:0]      commit_fire;
  commit_order_t   commit_order;
  word_t [1:0]     commit_pc;
  word_t [1:0]     commit_inst;
  arch_reg_t [1:0] commit_rd;
  logic [1:0]      commit_rd_wen;
  word_t [1:0]     commit_wdata;

  assign imem_rdata = {imem[{imem_addr[9:3], 1'b1}],
                       imem[{imem_addr[9:3], 1'b0}]};

  rv32i_ss_imem_zero_latency_adapter u_imem_adapter (
    .imem_req_valid (imem_req_valid),
    .imem_req_ready (imem_req_ready),
    .imem_req_addr  (imem_req_addr),
    .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready),
    .imem_resp_data (imem_resp_data),
    .line_addr      (imem_addr),
    .line_data      (imem_rdata)
  );

  umbra_ss_cpu_top u_cpu (
    .clk           (clk),
    .rst_n         (rst_n),
    .imem_req_valid(imem_req_valid),
    .imem_req_ready(imem_req_ready),
    .imem_req_addr (imem_req_addr),
    .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready),
    .imem_resp_data(imem_resp_data),
    .commit_fire   (commit_fire),
    .commit_order  (commit_order),
    .commit_pc     (commit_pc),
    .commit_inst   (commit_inst),
    .commit_rd     (commit_rd),
    .commit_rd_wen (commit_rd_wen),
    .commit_wdata  (commit_wdata),
    .dmem_valid    (dmem_valid),
    .dmem_we       (dmem_we),
    .dmem_be       (dmem_be),
    .dmem_addr     (dmem_addr),
    .dmem_wdata    (dmem_wdata),
    .dmem_ready    (1'b1),
    .dmem_rvalid   (1'b1),
    .dmem_rdata    ('0)
  );

  `define FE  u_cpu.u_fe
  `define RN  u_cpu.u_core.u_rename
  `define PRF u_cpu.u_core.u_prf

  int checks = 0;
  int errors = 0;
  int crossline_fires = 0;
  int commit_pc_count [0:6];
  bit first_pair_seen = 1'b0;
  bit marker_seen = 1'b0;

  task automatic check(input string name, input logic condition);
    checks++;
    if (!condition) begin
      errors++;
      $error("[%s]", name);
    end
  endtask

  task automatic check_arch(input string name, input int r, input word_t exp);
    automatic phys_reg_t p = `RN.committed_map_q[r];
    checks++;
    if (`PRF.regs_q[p] !== exp) begin
      errors++;
      $error("[%s] x%0d=%08h via p%0d, expected %08h",
             name, r, `PRF.regs_q[p], p, exp);
    end
  endtask

  always @(posedge clk) begin
    if (rst_n === 1'b1) begin
      if (`FE.bundle_fire && `FE.consume_cross_line) begin
        crossline_fires++;
        if ((`FE.decoded_pc[0] == 32'h0000_0004) &&
            (`FE.decoded_pc[1] == 32'h0000_0008) &&
            (`FE.decoded_instr[0] == 32'h0050_0093) &&
            (`FE.decoded_instr[1] == 32'h0070_0113))
          first_pair_seen = 1'b1;
      end

      for (int lane = 0; lane < 2; lane++) begin
        if (commit_fire[lane]) begin
          unique case (commit_pc[lane])
            32'h0000_0004: commit_pc_count[0]++;
            32'h0000_0008: commit_pc_count[1]++;
            32'h0000_000c: commit_pc_count[2]++;
            32'h0000_0010: commit_pc_count[3]++;
            32'h0000_0014: commit_pc_count[4]++;
            32'h0000_0018: commit_pc_count[5]++;
            32'h0000_001c: begin
              commit_pc_count[6]++;
              if (commit_rd_wen[lane] && (commit_rd[lane] == 5'd31) &&
                  (commit_wdata[lane] == 32'd31))
                marker_seen = 1'b1;
            end
            default: ;
          endcase
        end
      end
    end
  end

  initial begin
    repeat (3000) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_crossline exceeded 3000 cycles");
  end

  initial begin
    for (int i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
    for (int i = 0; i < 7; i++) commit_pc_count[i] = 0;

    imem[0] = 32'h3400_1073;  // 0x00 csrrw x0,mscratch,x0 -- solo
    imem[1] = 32'h0050_0093;  // 0x04 addi  x1,x0,5
    imem[2] = 32'h0070_0113;  // 0x08 addi  x2,x0,7
    imem[3] = 32'h0020_81b3;  // 0x0c add   x3,x1,x2       = 12
    imem[4] = 32'h0010_8093;  // 0x10 addi  x1,x1,1        = 6
    imem[5] = 32'h0021_0113;  // 0x14 addi  x2,x2,2        = 9
    imem[6] = 32'h0020_8233;  // 0x18 add   x4,x1,x2       = 15
    imem[7] = 32'h01f0_0f93;  // 0x1c addi  x31,x0,31      marker
    imem[8] = 32'h0000_006f;  // 0x20 jal   x0,0           park

    repeat (3) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;

    for (int wait_i = 0; !marker_seen; wait_i++) begin
      @(posedge clk);
      if (wait_i > 1000)
        $fatal(1, "stuck: marker never committed (order=%0d)", commit_order);
    end
    repeat (3) @(posedge clk);

    check("exact registered 0x04/0x08 cross-line pair entered",
          first_pair_seen);
    check("one deliberately forced cross-line pair fired", crossline_fires == 1);
    check("frontend event counter matches accepted pairs",
          `FE.q1_cross_line_count == crossline_fires);
    for (int i = 0; i < 7; i++)
      check($sformatf("architectural PC index %0d committed once", i),
            commit_pc_count[i] == 1);

    check_arch("x1 no skip or duplicate", 1, 32'd6);
    check_arch("x2 no skip or duplicate", 2, 32'd9);
    check_arch("x3 first cross-line dependency", 3, 32'd12);
    check_arch("x4 rolling cross-line dependency", 4, 32'd15);
    check_arch("x31 end marker", 31, 32'd31);

    if (errors == 0) begin
      $display("[tb_rv32i_ss_crossline] PASS checks=%0d", checks);
      $finish;
    end
    $fatal(1, "[tb_rv32i_ss_crossline] FAIL errors=%0d checks=%0d",
           errors, checks);
  end
endmodule
