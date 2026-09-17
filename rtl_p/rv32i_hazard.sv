// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

`timescale 1ns/1ps

// Pure-combinational hazard / forwarding unit.
//
// Outputs per cycle:
//   - forward_a_e, forward_b_e : EX-stage operand bypass selects
//   - stall_f, stall_d         : freeze PC and IF/ID
//   - stall_e                  : hold ID/EX (used for multi-cycle muldiv so
//                                the op stays in E until it finishes, and for
//                                M-stage data-memory waits)
//   - stall_m                  : hold EX/MEM while a memory op waits
//   - flush_d                  : kill IF/ID on any E-stage redirect
//   - flush_e                  : kill ID/EX (load-use bubble or branch flush)
//   - bubble_m                 : force EX/MEM to latch a bubble (used while
//                                muldiv is still computing so the half-baked
//                                E-stage result does not enter M)
//   - bubble_w                 : force MEM/WB to latch a bubble while M waits
//   - dmem_wait_m              : M-stage memory op is waiting for handshake
//   - dmem_m_load_use_stall    : completing M-stage load feeds current E uop;
//                                hold E one extra cycle so W->E forwarding is
//                                used instead of forwarding the load address
module rv32i_hazard
  import fyp_cpu_pkg::*;
  import rv32i_pipeline_pkg::*;
(
  // D-stage source register addresses (for load-use stall detection)
  input  reg_addr_t rs1_d,
  input  reg_addr_t rs2_d,
  input  logic      valid_d,
  input  logic      imem_wait_f,

  // E-stage source register addresses (for forwarding)
  input  reg_addr_t rs1_e,
  input  reg_addr_t rs2_e,

  // M-stage / W-stage destinations and write-enables (forwarding sources)
  input  reg_addr_t rd_m,
  input  logic      reg_write_m,
  input  reg_addr_t rd_w,
  input  logic      reg_write_w,

  // E-stage info
  input  logic      valid_e,
  input  reg_addr_t rd_e,
  input  logic      mem_read_e,   // 1 if the instr in E is a load
  input  logic      redirect_any,
  input  logic      redirect_w,
  input  logic      is_muldiv_e,  // 1 if the instr in E is a muldiv op
  input  logic      muldiv_done,  // 1 the cycle the muldiv unit signals done

  // M-stage memory handshake info
  input  logic      valid_m,
  input  logic      mem_read_m,
  input  logic      mem_write_m,
  input  logic      dmem_done_m,
  input  logic      is_csr_m,     // 1 if the instr in M is a CSR op (rd value not ready until W)

  // Outputs
  output fwd_sel_e  forward_a_e,
  output fwd_sel_e  forward_b_e,
  output logic      stall_f,
  output logic      stall_d,
  output logic      stall_e,
  output logic      stall_m,
  output logic      flush_d,
  output logic      flush_e,
  output logic      bubble_m,
  output logic      bubble_w,
  output logic      dmem_wait_m,
  output logic      dmem_m_load_use_stall,
  output logic      csr_m_use_stall
);

  logic load_use_stall;
  logic muldiv_stall;
  logic m_can_forward_e;

  // A CSR op in M has reg_write=1 but its rd value (the old CSR) is not produced
  // until W (RES_CSR), exactly like a load gives its address-not-data in M. So a
  // CSR op, like a load, must be excluded as an M-forward source.
  assign m_can_forward_e = valid_m && reg_write_m && !mem_read_m && !is_csr_m;

  always_comb begin
    // ----- forwarding from M (newer) takes priority over W -----
    forward_a_e = FWD_NONE;
    forward_b_e = FWD_NONE;

    if (m_can_forward_e && (rd_m != 5'd0) && (rd_m == rs1_e)) begin
      forward_a_e = FWD_FROM_M;
    end else if (reg_write_w && (rd_w != 5'd0) && (rd_w == rs1_e)) begin
      forward_a_e = FWD_FROM_W;
    end

    if (m_can_forward_e && (rd_m != 5'd0) && (rd_m == rs2_e)) begin
      forward_b_e = FWD_FROM_M;
    end else if (reg_write_w && (rd_w != 5'd0) && (rd_w == rs2_e)) begin
      forward_b_e = FWD_FROM_W;
    end
  end

  always_comb begin
    // ----- safe defaults -----
    stall_f                  = 1'b0;
    stall_d                  = 1'b0;
    stall_e                  = 1'b0;
    stall_m                  = 1'b0;
    flush_d                  = 1'b0;
    flush_e                  = 1'b0;
    bubble_m                 = 1'b0;
    bubble_w                 = 1'b0;
    dmem_wait_m              = 1'b0;
    dmem_m_load_use_stall    = 1'b0;
    csr_m_use_stall          = 1'b0;

    //F/D wait
    if (imem_wait_f) begin
      stall_f = 1'b1;
    end
    // ----- load-use stall: a load in E feeding an op in D -----
    // The load is currently in E, so its data will not be available for the
    // D-stage consumer next cycle. Hold F/D and bubble E.
    load_use_stall = valid_d &&
                     valid_e &&
                     mem_read_e &&
                     (rd_e != 5'd0) &&
                     ((rd_e == rs1_d) || (rd_e == rs2_d));

    // ----- multi-cycle muldiv -----
    // While the muldiv op is in E and its FSM has not signalled done, hold
    // F/D/E and inject a bubble into M. On the 'done' cycle, the stall
    // releases and the result + ctrl propagate forward.
    muldiv_stall = is_muldiv_e && !muldiv_done;

    // ----- M-stage memory wait / load-release stall -----
    // dmem_wait_m holds the whole front of the pipe while the memory op in M is
    // not complete. When a load in M completes and the current E-stage uop
    // needs that rd, hold E one extra cycle and bubble M so the load enters W;
    // then normal W->E forwarding supplies the loaded data.
    dmem_wait_m = valid_m && (mem_read_m || mem_write_m) && !dmem_done_m;
    dmem_m_load_use_stall = valid_m &&
                            mem_read_m &&
                            dmem_done_m &&
                            (rd_m != 5'd0) &&
                            valid_e &&
                            ((rd_m == rs1_e) || (rd_m == rs2_e));

    // ----- CSR-use stall (mirror of dmem_m_load_use_stall) -----
    // A CSR op in M produces its rd value only at W. If the E uop needs that rd,
    // hold E one cycle and bubble M so the CSR enters W; then W->E forwarding
    // supplies the CSR value. No dmem_done gate: a CSR op has no handshake.
    csr_m_use_stall = valid_m &&
                      is_csr_m &&
                      (rd_m != 5'd0) &&
                      valid_e &&
                      ((rd_m == rs1_e) || (rd_m == rs2_e));

    if (load_use_stall && !dmem_wait_m && !dmem_m_load_use_stall) begin
      stall_f = 1'b1;
      stall_d = 1'b1;
      flush_e = 1'b1;
    end

    if (muldiv_stall) begin
      stall_f  = 1'b1;
      stall_d  = 1'b1;
      stall_e  = 1'b1;     // ID/EX holds the muldiv ctrl
      bubble_m = 1'b1;     // EX/MEM latches a bubble
    end

    if (dmem_wait_m) begin
      stall_f  = 1'b1;
      stall_d  = 1'b1;
      stall_e  = 1'b1;
      stall_m  = 1'b1;     // EX/MEM holds the waiting memory op
      bubble_w = 1'b1;     // MEM/WB retires nothing while M waits
    end

    if (dmem_m_load_use_stall) begin
      stall_f  = 1'b1;
      stall_d  = 1'b1;
      stall_e  = 1'b1;
      bubble_m = 1'b1;     // load moves M->W; bubble M behind it
    end

    if (csr_m_use_stall) begin
      stall_f  = 1'b1;
      stall_d  = 1'b1;
      stall_e  = 1'b1;
      bubble_m = 1'b1;     // CSR moves M->W; bubble M behind it
    end

    // ----- control-hazard flush on any E-stage redirect -----
    if (redirect_any) begin
      flush_d = 1'b1;
      flush_e = 1'b1;
    end
    if (redirect_w) begin
      stall_f = 1'b0;
      stall_d = 1'b0;
      stall_e = 1'b0;
      stall_m = 1'b0;

      flush_d = 1'b1;
      flush_e = 1'b1;
      bubble_m = 1'b1;
      bubble_w = 1'b1;

    end
  end
endmodule
