`timescale 1ns/1ps

// Machine-mode CSR storage and precise trap state for the superscalar OoO core.
//
// CSR state changes two ways:
// 1. Explicit CSR instruction (csrrw/s/c + immediates) -> the we/addr/wdata
// port. The read-modify-write is computed by the core, so this port just
// commits the final value atomically at the commit boundary.
// 2. A trap or an mret -> the trap_we / mret_we ports. A trap writes mepc,
// mcause, and mtval, then pushes the mstatus interrupt-enable stack;
// mret pops it. Several CSRs move at once, which the single (addr,wdata)
// port cannot express, so traps get dedicated ports.
//
// mtvec and mepc are also exposed as direct outputs so the core's trap logic can
// form the redirect target (mtvec on entry, mepc on mret) without borrowing the
// CSR-instruction read port.
module rv32i_ss_csr_file
  import fyp_cpu_pkg::*;
(
  input  logic        clk,
  input  logic        rst_n,

  // ---- explicit CSR-instruction access (csrrw/s/c + immediates) ----
  input  logic        we,
  input  word_t       wdata,
  input  csr_addr_t   waddr,

  input  csr_addr_t   raddr,
  output word_t       rdata,

  // ---- trap entry (precise, accepted at the committing ROB head) ----
  input  logic        trap_we,     // 1 -> take a trap this cycle
  input  word_t       trap_pc,     // PC of the trapping instruction -> mepc
  input  word_t       trap_cause,  // cause code -> mcause (0/2/3/4/6/11 in this core)
  input  word_t       trap_tval,

  // ---- trap return ----
  input  logic        mret_we,     // 1 -> an mret commits this cycle

  // ---- direct outputs for the core's redirect logic ----
  output word_t       mtvec,       // trap base (direct mode; core masks low 2 bits)
  output word_t       mepc         // mret return target
);

  localparam logic [11:0] CSR_MSTATUS = 12'h300;
  localparam logic [11:0] CSR_MTVEC   = 12'h305;
  localparam logic [11:0] CSR_MEPC    = 12'h341;
  localparam logic [11:0] CSR_MCAUSE  = 12'h342;
  localparam logic [11:0] CSR_MTVAL   = 12'h343;

  // mstatus field positions (RV32, M-mode subset). All other bits read as 0.
  localparam int         MIE_BIT  = 3;     // machine interrupt enable
  localparam int         MPIE_BIT = 7;     // previous MIE (the saved copy)
  localparam int         MPP_LO   = 11;    // previous privilege, bits [12:11]
  localparam int         MPP_HI   = 12;
  localparam logic [1:0] PRIV_M   = 2'b11; // machine mode

  word_t mstatus_q;
  word_t mtvec_q;
  word_t mepc_q;
  word_t mcause_q;
  word_t mtval_q;

  function automatic word_t mstatus_warl(input word_t raw);
    mstatus_warl = 32'h0000_0000;
    mstatus_warl[MIE_BIT]  = raw[MIE_BIT];
    mstatus_warl[MPIE_BIT] = raw[MPIE_BIT];
    if (raw[MPP_HI:MPP_LO] == PRIV_M) begin
      mstatus_warl[MPP_HI:MPP_LO] = PRIV_M;
    end
  endfunction

  // ---- combinational read (old value, feeds the CSR-instruction RMW) ----
  always_comb begin
    unique case (raddr)
      CSR_MSTATUS: rdata = mstatus_q;
      CSR_MTVEC:   rdata = mtvec_q;
      CSR_MEPC:    rdata = mepc_q;
      CSR_MCAUSE:  rdata = mcause_q;
      CSR_MTVAL:   rdata = mtval_q;
      default:     rdata = 32'h0000_0000;  // unimplemented CSR -> 0
    endcase
  end

  // ---- direct outputs for the core redirect logic ----
  assign mtvec = mtvec_q;
  assign mepc  = mepc_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mstatus_q <= 32'h0000_0000;
      mtvec_q   <= 32'h0000_0000;
      mepc_q    <= 32'h0000_0000;
      mcause_q  <= 32'h0000_0000;
      mtval_q   <= 32'h0000_0000;
    end else begin
      // Priority: trap entry > mret > explicit CSR write.
      // These are mutually exclusive by construction at the commit boundary;
      // the priority ordering remains defensive.
      if (trap_we) begin
        mepc_q  <= trap_pc;
        mcause_q <= trap_cause;
        mstatus_q[MPIE_BIT] <= mstatus_q[MIE_BIT];
        mstatus_q[MIE_BIT]  <= 1'b0;
        mstatus_q[MPP_HI:MPP_LO] <= PRIV_M;
        mtval_q  <= trap_tval;
      end else if (mret_we) begin
        mstatus_q[MIE_BIT] <= mstatus_q[MPIE_BIT];
        mstatus_q[MPIE_BIT] <= 1'b1;
        mstatus_q[MPP_HI:MPP_LO] <= PRIV_M;
      end else if (we) begin
        unique case (waddr)
          CSR_MSTATUS: mstatus_q <= mstatus_warl(wdata);
          CSR_MTVEC:   mtvec_q   <= wdata;
          CSR_MEPC:    mepc_q    <= wdata;
          CSR_MCAUSE:  mcause_q  <= wdata;
          CSR_MTVAL:   mtval_q   <= wdata;
          default: begin
            // Unknown CSR writes are ignored.
          end
        endcase
      end
    end
  end

endmodule
