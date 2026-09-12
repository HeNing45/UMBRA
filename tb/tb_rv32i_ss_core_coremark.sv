// Cycle-driven, non-timing Verilator top for UMBRA CoreMark.
//
// A C++ harness drives clk/rst_n. The image is loaded into independent
// instruction/data arrays with the same raw-word and byte-enable semantics as
// ooo_dmem_model. Three addresses form a simulation-only platform seam:
//   0x0003ff00 read  — low 32 bits of the RTL cycle counter
//   0x0003ff04 write — one output character in wdata[7:0]
//   0x0003ff08 write — 1 means target completed
//
// The instruction-side environment is rv32i_ss_imem_scratchpad. The default
// latency of 0 reproduces rv32i_ss_imem_zero_latency_adapter. A latency-1
// measurement passes +define+UMBRA_M3_IMEM_LATENCY=1. Latency is fixed at
// elaboration, so a runtime plusarg cannot select it.
`ifndef UMBRA_M3_IMEM_LATENCY
`define UMBRA_M3_IMEM_LATENCY 0
`endif
// Pipelined acceptance is selected at elaboration, like LATENCY. It defaults
// to 0 and is legal only together with LATENCY=1; the module rejects any
// other enabled combination.
`ifndef UMBRA_M3_IMEM_PIPELINED
`define UMBRA_M3_IMEM_PIPELINED 0
`endif

// Data-side defaults 0/0 reproduce the always-ready environment. A diagnostic
// stall run passes +define+UMBRA_M4_DMEM_READY_STALL=N.
`ifndef UMBRA_M4_DMEM_READY_STALL
`define UMBRA_M4_DMEM_READY_STALL 0
`endif
`ifndef UMBRA_M4_MAX_OUTSTANDING
`define UMBRA_M4_MAX_OUTSTANDING 1
`endif
`ifndef UMBRA_M4_DMEM_RESP_LATENCY
`define UMBRA_M4_DMEM_RESP_LATENCY 0
`endif
`ifndef UMBRA_M4_DMEM_STALL_SEL
`define UMBRA_M4_DMEM_STALL_SEL 0
`endif

module umbra_coremark_sim_top (
  input  logic                clk,
  input  logic                rst_n,
  output logic                uart_valid,
  output logic [7:0]          uart_char,
  output logic                done_valid,
  output logic [31:0]         done_value,
  output logic [63:0]         cycle_count_o,
  output logic [63:0]         commit_count_o
);
  import rv32i_ss_pkg::*;

  localparam int MEM_WORDS = 65536;
  localparam int MEM_MSB   = 17;

  localparam word_t MMIO_CYCLE = 32'h0003_ff00;
  localparam word_t MMIO_UART  = 32'h0003_ff04;
  localparam word_t MMIO_DONE  = 32'h0003_ff08;

  logic [1:0]          commit_fire;

  logic                dmem_valid;
  logic                dmem_we;
  logic [3:0]          dmem_be;
  word_t               dmem_addr;
  word_t               dmem_wdata;
  word_t               dmem_rdata;
  logic                dmem_ready;
  logic                dmem_rvalid;

  word_t               imem_addr;
  word_t [1:0]         imem_rdata;
  logic                imem_req_valid;
  logic                imem_req_ready;
  word_t               imem_req_addr;
  logic                imem_resp_valid;
  logic                imem_resp_ready;
  word_t [1:0]         imem_resp_data;
  logic [31:0]         imem [0:MEM_WORDS-1];
  logic [31:0]         dmem [0:MEM_WORDS-1];

  logic [63:0]         cycle_count;
  logic [63:0]         commit_count;

  assign cycle_count_o = cycle_count;
  assign commit_count_o = commit_count;

  assign imem_rdata = !$isunknown(imem_addr[MEM_MSB:3])
      ? {imem[{imem_addr[MEM_MSB:3], 1'b1}],
         imem[{imem_addr[MEM_MSB:3], 1'b0}]}
      : {32'h0000_0013, 32'h0000_0013};

  rv32i_ss_imem_scratchpad #(.LATENCY(`UMBRA_M3_IMEM_LATENCY),
                             .PIPELINED(`UMBRA_M3_IMEM_PIPELINED)) u_imem_adapter (
    .clk(clk), .rst_n(rst_n),
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr), .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready), .imem_resp_data(imem_resp_data),
    .line_addr(imem_addr), .line_data(imem_rdata)
  );

  // data-side environment. The store-side enable is an ACCEPTANCE strobe:
  // the array and all three MMIO seams below key off sp_en, never dmem_valid,
  // so a request held across a stall window acts exactly once.
  logic        sp_en, sp_we;
  logic [3:0]  sp_be;
  word_t       sp_addr, sp_wdata, sp_rdata;

  rv32i_ss_dmem_scratchpad #(.READY_STALL(`UMBRA_M4_DMEM_READY_STALL),
                             .RESP_LATENCY(`UMBRA_M4_DMEM_RESP_LATENCY),
                             .MAX_OUTSTANDING(`UMBRA_M4_MAX_OUTSTANDING),
                             .STALL_SEL(`UMBRA_M4_DMEM_STALL_SEL)) u_dmem_env (
    .clk(clk), .rst_n(rst_n),
    .dmem_valid(dmem_valid), .dmem_we(dmem_we), .dmem_be(dmem_be),
    .dmem_addr(dmem_addr), .dmem_wdata(dmem_wdata),
    .dmem_ready(dmem_ready), .dmem_rvalid(dmem_rvalid), .dmem_rdata(dmem_rdata),
    .mem_en(sp_en), .mem_we(sp_we), .mem_be(sp_be),
    .mem_addr(sp_addr), .mem_wdata(sp_wdata), .mem_rdata(sp_rdata)
  );

  // instrument: count store commits that coincide with a low ready, so the
  // benign explanation for a silent tripwire is EXCLUDED by counting rather
  // than assumed.
  integer n_store_commit_lowready = 0;
  always @(posedge clk) if (rst_n && dmem_valid && dmem_we && !dmem_ready)
    n_store_commit_lowready = n_store_commit_lowready + 1;

  always_comb begin
    if (sp_addr == MMIO_CYCLE)
      sp_rdata = cycle_count[31:0];
    else if (!$isunknown(sp_addr[MEM_MSB:2]))
      sp_rdata = dmem[sp_addr[MEM_MSB:2]];
    else
      sp_rdata = 32'h0000_0000;
  end

  function automatic logic [31:0] apply_be(input logic [31:0] cur,
                                           input logic [31:0] data,
                                           input logic [3:0]  mask);
    apply_be = cur;
    if (mask[0]) apply_be[7:0]   = data[7:0];
    if (mask[1]) apply_be[15:8]  = data[15:8];
    if (mask[2]) apply_be[23:16] = data[23:16];
    if (mask[3]) apply_be[31:24] = data[31:24];
  endfunction

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      cycle_count <= 64'd0;
      commit_count <= 64'd0;
      uart_valid <= 1'b0;
      uart_char <= 8'd0;
      done_valid <= 1'b0;
      done_value <= 32'd0;
    end else begin
      uart_valid <= 1'b0;
      done_valid <= 1'b0;
      cycle_count <= cycle_count + 64'd1;
      commit_count <= commit_count
                    + {63'd0, commit_fire[0]}
                    + {63'd0, commit_fire[1]};
      if (sp_en && sp_we) begin
        if (sp_addr == MMIO_UART) begin
          uart_valid <= 1'b1;
          uart_char <= sp_wdata[7:0];
        end else if (sp_addr == MMIO_DONE) begin
          done_valid <= 1'b1;
          done_value <= sp_wdata;
        end else if (!$isunknown(sp_addr[MEM_MSB:2])) begin
          dmem[sp_addr[MEM_MSB:2]]
            <= apply_be(dmem[sp_addr[MEM_MSB:2]], sp_wdata, sp_be);
        end
      end
    end
  end

  // Consume the public wiring-only CPU top so this benchmark cannot drift
  // from the frontend/core boundary by duplicating the decoded packet here.
  umbra_ss_cpu_top u_cpu (
    .clk           (clk),
    .rst_n         (rst_n),
    .imem_req_valid  (imem_req_valid),
    .imem_req_ready  (imem_req_ready),
    .imem_req_addr   (imem_req_addr),
    .imem_resp_valid (imem_resp_valid),
    .imem_resp_ready (imem_resp_ready),
    .imem_resp_data  (imem_resp_data),
    .commit_fire   (commit_fire),
    .commit_order  (),
    .commit_pc     (),
    .commit_inst   (),
    .commit_rd     (),
    .commit_rd_wen (),
    .commit_wdata  (),
    .dmem_valid    (dmem_valid),
    .dmem_we       (dmem_we),
    .dmem_be       (dmem_be),
    .dmem_addr     (dmem_addr),
    .dmem_wdata    (dmem_wdata),
    .dmem_ready    (dmem_ready),
    .dmem_rvalid   (dmem_rvalid),
    .dmem_rdata    (dmem_rdata)
  );

  string imem_path;
  integer i;
  initial begin
    for (i = 0; i < MEM_WORDS; i++) begin
      imem[i] = 32'h0000_0013;
      dmem[i] = 32'h0000_0000;
    end

    if (!$value$plusargs("IMEM=%s", imem_path))
      $fatal(1, "missing +IMEM=<path>");
    $readmemh(imem_path, imem);
    $readmemh(imem_path, dmem);
  end

endmodule
