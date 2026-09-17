// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

// Simulation-only pipeline invariants.
//
// This file intentionally uses plain procedural if/$fatal checks instead of
// SystemVerilog assert property or bind. The project regression runs under
// Icarus, which does not support bind/concurrent SVA and is fragile around
// immediate assert statements in larger TBs.
`ifndef SYNTHESIS
`ifndef RV32I_PIPELINE_ASSERT_OFF

  `define RV32I_PIPE_CHECK(cond_, msg_) \
    if (!(cond_)) begin \
      $display("%0t %m ASSERT: %s", $time, msg_); \
      $fatal(1); \
    end

  always @(posedge clk or negedge rst_n) begin
    if (rst_n) begin
      // Invalid/bubble uops must not perform architectural side effects.
      `RV32I_PIPE_CHECK(uop_e.ctrl.valid || (!uop_e.ctrl.reg_write &&
                                             !uop_e.ctrl.mem_write &&
                                             !uop_e.ctrl.mem_read &&
                                             !uop_e.ctrl.sim_halt),
                        "invalid E-stage uop has side effects")

      `RV32I_PIPE_CHECK(uop_m.ctrl.valid || (!uop_m.ctrl.reg_write &&
                                             !uop_m.ctrl.mem_write &&
                                             !uop_m.ctrl.mem_read &&
                                             !uop_m.ctrl.sim_halt),
                        "invalid M-stage uop has side effects")

      `RV32I_PIPE_CHECK(uop_w.ctrl.valid || (!uop_w.ctrl.reg_write &&
                                             !uop_w.ctrl.mem_write &&
                                             !uop_w.ctrl.mem_read &&
                                             !uop_w.ctrl.sim_halt),
                        "invalid W-stage uop has side effects")

      // Decode/control classification checks.
      `RV32I_PIPE_CHECK(!(uop_e.ctrl.mem_read && uop_e.ctrl.mem_write),
                        "E-stage uop is both load and store")

      `RV32I_PIPE_CHECK(!(uop_m.ctrl.mem_read && uop_m.ctrl.mem_write),
                        "M-stage uop is both load and store")

      `RV32I_PIPE_CHECK(!(uop_w.ctrl.mem_read && uop_w.ctrl.mem_write),
                        "W-stage uop is both load and store")

      `RV32I_PIPE_CHECK(!uop_e.ctrl.mem_read ||
                        (uop_e.ctrl.valid &&
                         uop_e.ctrl.reg_write &&
                         (uop_e.ctrl.result_src == RES_MEM)),
                        "E-stage load has inconsistent control")

      `RV32I_PIPE_CHECK(!uop_e.ctrl.is_muldiv ||
                        (uop_e.ctrl.valid &&
                         uop_e.ctrl.reg_write &&
                         (uop_e.ctrl.result_src == RES_MULDIV)),
                        "E-stage muldiv has inconsistent control")

      `RV32I_PIPE_CHECK(!(uop_e.ctrl.is_jump &&
                          (uop_e.ctrl.branch_op != BR_NONE)),
                        "E-stage uop is both jump and branch")

      // Branch predictor update contract: train conditional branches only.
      `RV32I_PIPE_CHECK(bp_update_valid_e ==
                        (branch_is_e && e_fire && !redirect_w &&
                         !iaddr_misalign_e),
                        "BP update valid does not match branch_is_e")

      `RV32I_PIPE_CHECK(!bp_update_valid_e ||
                        (uop_e.ctrl.valid &&
                         (uop_e.ctrl.branch_op != BR_NONE)),
                        "BP update from non-branch uop")

      `RV32I_PIPE_CHECK(!bp_update_valid_e ||
                        (bp_update_pc_e == uop_e.pc),
                        "BP update PC is not the resolved branch PC")

      `RV32I_PIPE_CHECK(!bp_update_valid_e ||
                        (bp_update_target_e == branch_target_e),
                        "BP update target is not the branch target")

      `RV32I_PIPE_CHECK(branch_mispredict_e == (branch_is_e &&
                                                ((uop_e.bp_pred_taken != branch_taken_e) ||
                                                 (branch_taken_e &&
                                                  (uop_e.bp_pred_target != branch_target_e)))),
                        "branch_mispredict_e equation changed")

      // Redirect contract.
      `RV32I_PIPE_CHECK(!e_redirect_ready ||
                        !uop_e.ctrl.is_jump ||
                        iaddr_misalign_e ||
                        redirect_e,
                        "jump in E did not redirect")

      `RV32I_PIPE_CHECK(!branch_mispredict_e ||
                        !e_redirect_ready ||
                        iaddr_misalign_e ||
                        redirect_e,
                        "branch mispredict did not redirect")

      `RV32I_PIPE_CHECK(!(uop_e.ctrl.valid && !branch_is_e &&
                          !uop_e.ctrl.is_jump && uop_e.bp_pred_taken &&
                          e_redirect_ready) ||
                        (redirect_e &&
                         (redirect_target_e == uop_e.pc_plus4)),
                        "bogus predicted-taken uop did not fall through")

      // A redirect from either source flushes the younger D/E slots.
      `RV32I_PIPE_CHECK(!redirect_any || (flush_d && flush_e),
                        "redirect did not request younger-stage flush")

      // Target arbitration: W (trap/mret) outranks E (branch/jump). pc_next_f
      // carries the predicted/sequential PC; redirect_target is the selected
      // recovery PC and pc_f loads it on redirect_any.
      `RV32I_PIPE_CHECK(!(redirect_e && !redirect_w) ||
                        (redirect_target == redirect_target_e),
                        "E redirect target not selected when W idle")
      `RV32I_PIPE_CHECK(!redirect_w ||
                        (redirect_target == redirect_target_w),
                        "W redirect target not selected (W must win over E)")

      // A W redirect (trap/mret) is precise: it squashes D/E AND M, and blocks
      // the younger M op from retiring into W.
      `RV32I_PIPE_CHECK(!redirect_w ||
                        (flush_d && flush_e && bubble_m && bubble_w),
                        "W redirect did not squash D/E/M and block W retire")

      // Hazard/forwarding contract.
      `RV32I_PIPE_CHECK(!(ctrl_d_gated.valid &&
                          uop_e.ctrl.valid &&
                          uop_e.ctrl.mem_read &&
                          (uop_e.rd_addr != 5'd0) &&
                          ((uop_e.rd_addr == rs1_addr_d) ||
                           (uop_e.rd_addr == rs2_addr_d)) &&
                          !dmem_wait_m &&
                          !dmem_m_load_use_stall &&
                          !redirect_w) ||
                        (stall_f && stall_d && flush_e),
                        "load-use hazard did not stall/flush")

      `RV32I_PIPE_CHECK(!(uop_e.ctrl.is_muldiv && !muldiv_done && !redirect_w) ||
                        (stall_f && stall_d && stall_e && bubble_m),
                        "active muldiv did not hold E and bubble M")

      `RV32I_PIPE_CHECK(!(dmem_wait_m && !redirect_w) ||
                        (stall_f && stall_d && stall_e && stall_m &&
                         bubble_w),
                        "dmem wait did not hold front/M and bubble W")

      `RV32I_PIPE_CHECK(!(dmem_m_load_use_stall && !redirect_w) ||
                        (stall_f && stall_d && stall_e && bubble_m &&
                         !stall_m && !bubble_w),
                        "M-stage load release did not hold E and bubble M")

      `RV32I_PIPE_CHECK(!(csr_m_use_stall && !redirect_w) ||
                        (stall_f && stall_d && stall_e && bubble_m &&
                         !stall_m && !bubble_w),
                        "M-stage CSR release did not hold E and bubble M")

      `RV32I_PIPE_CHECK(!muldiv_start_e ||
                        (uop_e.ctrl.valid && uop_e.ctrl.is_muldiv),
                        "muldiv_start_e asserted for non-muldiv uop")

      `RV32I_PIPE_CHECK(!(uop_m.ctrl.valid &&
                          uop_m.ctrl.reg_write &&
                          !uop_m.ctrl.mem_read &&
                          (uop_m.ctrl.csr == CSR_NONE) &&
                          (uop_m.rd_addr != 5'd0) &&
                          (uop_m.rd_addr == uop_e.rs1_addr)) ||
                        (forward_a_e == FWD_FROM_M),
                        "rs1 M-stage forwarding priority broken")

      `RV32I_PIPE_CHECK(!(!(uop_m.ctrl.valid &&
                            uop_m.ctrl.reg_write &&
                            !uop_m.ctrl.mem_read &&
                            (uop_m.ctrl.csr == CSR_NONE) &&
                            (uop_m.rd_addr != 5'd0) &&
                            (uop_m.rd_addr == uop_e.rs1_addr)) &&
                          uop_w.ctrl.reg_write &&
                          (uop_w.rd_addr != 5'd0) &&
                          (uop_w.rd_addr == uop_e.rs1_addr)) ||
                        (forward_a_e == FWD_FROM_W),
                        "rs1 W-stage forwarding broken")

      `RV32I_PIPE_CHECK(!(uop_m.ctrl.valid &&
                          uop_m.ctrl.reg_write &&
                          !uop_m.ctrl.mem_read &&
                          (uop_m.ctrl.csr == CSR_NONE) &&
                          (uop_m.rd_addr != 5'd0) &&
                          (uop_m.rd_addr == uop_e.rs2_addr)) ||
                        (forward_b_e == FWD_FROM_M),
                        "rs2 M-stage forwarding priority broken")

      `RV32I_PIPE_CHECK(!(!(uop_m.ctrl.valid &&
                            uop_m.ctrl.reg_write &&
                            !uop_m.ctrl.mem_read &&
                            (uop_m.ctrl.csr == CSR_NONE) &&
                            (uop_m.rd_addr != 5'd0) &&
                            (uop_m.rd_addr == uop_e.rs2_addr)) &&
                          uop_w.ctrl.reg_write &&
                          (uop_w.rd_addr != 5'd0) &&
                          (uop_w.rd_addr == uop_e.rs2_addr)) ||
                        (forward_b_e == FWD_FROM_W),
                        "rs2 W-stage forwarding broken")

      // Memory interface should only perform stores for valid store uops.
      `RV32I_PIPE_CHECK(!dmem_we ||
                        (uop_m.ctrl.valid && uop_m.ctrl.mem_write),
                        "dmem_we asserted without a valid store")

      `RV32I_PIPE_CHECK(!uop_w.ctrl.sim_halt || uop_w.ctrl.valid,
                        "invalid W-stage uop asserted sim_halt")
    end
  end

  `undef RV32I_PIPE_CHECK

`endif
`endif
