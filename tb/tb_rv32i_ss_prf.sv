`timescale 1ns/1ps

module tb_rv32i_ss_prf;
  import rv32i_ss_pkg::*;

  localparam time CLK_PERIOD = 10ns;

  logic      clk;
  logic      rst_n;
  phys_reg_t raddr1;
  word_t     rdata1;
  phys_reg_t raddr2;
  word_t     rdata2;
  phys_reg_t raddr3;
  word_t     rdata3;
  phys_reg_t raddr4;
  word_t     rdata4;
  logic      write_en;
  phys_reg_t waddr;
  word_t     wdata;
  logic [1:0]               alloc_en;
  phys_reg_t [1:0]          alloc_phys;
  logic [OOO_PHYS_REGS-1:0] ready_vec;

  int errors = 0;
  int checks = 0;

  rv32i_ss_prf dut (
    .clk      (clk),
    .rst_n    (rst_n),
    .raddr1   (raddr1),
    .rdata1   (rdata1),
    .raddr2   (raddr2),
    .rdata2   (rdata2),
    .raddr3   (raddr3),
    .rdata3   (rdata3),
    .raddr4   (raddr4),
    .rdata4   (rdata4),
    .write_en (write_en),
    .waddr    (waddr),
    .wdata    (wdata),
    .alloc_fire (alloc_en),
    .alloc_phys (alloc_phys),
    .ready_vec  (ready_vec)
  );

  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;

  initial begin
    repeat (500) @(posedge clk);
    $fatal(1, "WATCHDOG: tb_rv32i_ss_prf exceeded 500 cycles");
  end

  function automatic phys_reg_t preg(input int value);
    preg = phys_reg_t'(value);
  endfunction

  task automatic clear_inputs();
    raddr3     = '0;
    raddr4     = '0;
    raddr1     = '0;
    raddr2     = '0;
    write_en   = 1'b0;
    waddr      = '0;
    wdata      = '0;
    alloc_en   = 1'b0;
    alloc_phys = '0;
  endtask

  task automatic check_bit(input string name, input logic got, input logic exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got %0b expected %0b", name, got, exp);
      errors++;
    end
  endtask

  task automatic check_word(input string name, input word_t got, input word_t exp);
    checks++;
    if (got !== exp) begin
      $error("[%s] got %08h expected %08h", name, got, exp);
      errors++;
    end
  endtask

  task automatic expect_reads(
    input string name,
    input phys_reg_t addr1,
    input word_t exp1,
    input phys_reg_t addr2,
    input word_t exp2
  );
    raddr1 = addr1;
    raddr2 = addr2;
    #1;
    check_word({name, " rdata1"}, rdata1, exp1);
    check_word({name, " rdata2"}, rdata2, exp2);
  endtask

  task automatic write_reg(input string name, input phys_reg_t addr, input word_t data);
    @(negedge clk);
    write_en = 1'b1;
    waddr    = addr;
    wdata    = data;
    @(posedge clk);
    @(negedge clk);
    write_en = 1'b0;
    waddr    = '0;
    wdata    = '0;
    #1;
    $display("[%s] wrote p%0d = %08h", name, addr, data);
  endtask

  task automatic reset_dut();
    clear_inputs();
    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    @(negedge clk);
    expect_reads("reset p0/p1", preg(0), 32'h0000_0000, preg(1), 32'h0000_0000);
    expect_reads("reset p32/p63", preg(32), 32'h0000_0000, preg(63), 32'h0000_0000);
  endtask

  initial begin
    $display("[tb_rv32i_ss_prf] starting");

    reset_dut();

    write_reg("write p32", preg(32), 32'h1234_5678);
    expect_reads("read p32 and p0", preg(32), 32'h1234_5678, preg(0), 32'h0000_0000);

    write_reg("write p33", preg(33), 32'hdead_beef);
    expect_reads("dual read p32/p33", preg(32), 32'h1234_5678, preg(33), 32'hdead_beef);

    write_reg("overwrite p32", preg(32), 32'h0bad_cafe);
    expect_reads("read overwritten p32", preg(32), 32'h0bad_cafe, preg(33), 32'hdead_beef);

    write_reg("ignored p0 write", preg(0), 32'hffff_ffff);
    expect_reads("p0 still zero", preg(0), 32'h0000_0000, preg(32), 32'h0bad_cafe);

    // 4R read side: ports 3/4 mirror 1/2, including the p0 zero case,
    // and all four ports read concurrently.
    @(negedge clk);
    raddr1 = preg(32); raddr2 = preg(33);
    raddr3 = preg(33); raddr4 = preg(32);
    #1;
    check_word("4R port3 reads p33", rdata3, 32'hdead_beef);
    check_word("4R port4 reads p32", rdata4, 32'h0bad_cafe);
    check_word("4R port1 concurrent", rdata1, 32'h0bad_cafe);
    check_word("4R port2 concurrent", rdata2, 32'hdead_beef);
    raddr3 = preg(0); raddr4 = preg(0);
    #1;
    check_word("4R port3 p0 zero", rdata3, 32'h0000_0000);
    check_word("4R port4 p0 zero", rdata4, 32'h0000_0000);
    raddr1 = '0; raddr2 = '0; raddr3 = '0; raddr4 = '0;

    @(negedge clk);
    raddr1   = preg(34);
    write_en = 1'b1;
    waddr    = preg(34);
    wdata    = 32'hc001_d00d;
    #1;
    check_word("same-cycle before edge still old", rdata1, 32'h0000_0000);
    @(posedge clk);
    #1;
    check_word("same-cycle after edge sees new", rdata1, 32'hc001_d00d);
    @(negedge clk);
    clear_inputs();

    // ============ ready/busy table ============
    // Fresh reset so readiness isn't perturbed by the value writes above
    // (those set ready via write_en, which is correct but not what we check here).
    reset_dut();
    #1;
    check_bit("ready[p0]=1 after reset",  ready_vec[0],  1'b1);
    check_bit("ready[p1]=1 after reset",  ready_vec[1],  1'b1);
    check_bit("ready[p31]=1 after reset", ready_vec[31], 1'b1);
    check_bit("ready[p32]=0 after reset", ready_vec[32], 1'b0);
    check_bit("ready[p63]=0 after reset", ready_vec[63], 1'b0);

    // accepted write sets ready[p40], registered -> visible the next cycle
    @(negedge clk);
    write_en = 1'b1; waddr = preg(40); wdata = 32'h0000_00aa;
    #1;
    check_bit("ready[p40] still 0 before write edge", ready_vec[40], 1'b0);
    @(posedge clk);
    #1;
    check_bit("ready[p40]=1 after write edge", ready_vec[40], 1'b1);
    @(negedge clk);
    write_en = 1'b0; waddr = '0; wdata = '0;

    // allocating p40 marks it busy again
    @(negedge clk);
    alloc_en = 1'b1; alloc_phys = preg(40);
    @(posedge clk);
    #1;
    check_bit("ready[p40]=0 after alloc", ready_vec[40], 1'b0);
    @(negedge clk);
    alloc_en = 1'b0; alloc_phys = '0;

    // p0 readiness is sticky even if alloc/write target p0
    @(negedge clk);
    alloc_en = 1'b1; alloc_phys = preg(0);
    write_en = 1'b1; waddr = preg(0); wdata = 32'hffff_ffff;
    @(posedge clk);
    #1;
    check_bit("ready[p0] stays ready", ready_vec[0], 1'b1);
    @(negedge clk);
    clear_inputs();


    // ============ ready-table 2W — dual same-cycle busy-clears ========
    @(negedge clk);
    alloc_en   = 2'b11;
    alloc_phys = {6'd45, 6'd44};
    @(posedge clk); @(negedge clk);
    alloc_en = 2'b00; alloc_phys = '0;
    checks++;
    if (!(ready_vec[44] === 1'b0 && ready_vec[45] === 1'b0)) begin
      errors++; $error("[B2d dual clear] p44/p45 not both busy");
    end

    if (errors == 0) begin
      $display("[tb_rv32i_ss_prf] PASS checks=%0d", checks);
      $finish;
    end else begin
      $fatal(1, "[tb_rv32i_ss_prf] FAIL errors=%0d checks=%0d", errors, checks);
    end
  end

endmodule
