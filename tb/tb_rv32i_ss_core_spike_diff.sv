`timescale 1ns/1ps

// Spike-diff harness for the OoO core (frontend + core + imem).
//
// Loads a word .mem (+IMEM=<path>), runs the OoO core, and emits canonical
// COMMIT lines from the core's exposed commit channel, gated on commit_fire and
// enabled by +TRACE. The line form is the four-field
//   COMMIT pc=%08h instr=%08h rd=%0d wdata=%08h
// which matches verification/normalize_spike_trace.py output, so the two traces diff
// directly.
//
// Terminates after +MAX_COMMITS=N commits,
// so the external flow diffs the first N commits against Spike's first N.
//
// External driver: the Spike comparison workflow.

`ifndef OOO_SPIKE_RESET_PC
`define OOO_SPIKE_RESET_PC 32'h0000_0000
`endif


// The instruction environment is rv32i_ss_imem_scratchpad. Both defaults
// are 0, reproducing rv32i_ss_imem_zero_latency_adapter.
// The pipelined-correctness run passes +define+UMBRA_M3_IMEM_LATENCY=1 and
// +define+UMBRA_M3_IMEM_PIPELINED=1. Both are parameters, fixed at elaboration.
`ifndef UMBRA_M3_IMEM_LATENCY
`define UMBRA_M3_IMEM_LATENCY 0
`endif
`ifndef UMBRA_M3_IMEM_PIPELINED
`define UMBRA_M3_IMEM_PIPELINED 0
`endif

module tb_rv32i_ss_core_spike_diff;
  import rv32i_ss_pkg::*;
  import fyp_cpu_pkg::alu_op_e;
  import fyp_cpu_pkg::mem_size_e;
  import rv32i_pipeline_pkg::br_type_e;
  import rv32i_pipeline_pkg::muldiv_op_e;
  import rv32i_pipeline_pkg::csr_op_e;

  localparam logic [31:0] RESET_PC = `OOO_SPIKE_RESET_PC;

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

  // core commit channel (outputs)
  logic [1:0]          commit_fire;
  commit_order_t commit_order;
  word_t [1:0]         commit_pc, commit_inst, commit_wdata;
  arch_reg_t [1:0]     commit_rd;
  logic [1:0]          commit_rd_wen;

  // core -> frontend redirect
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

  // data memory : shared raw-word model; image is mirrored into it after
  // load so programs may read initialized text-region data. tohost unused here.
  logic        dmem_valid, dmem_we;
  logic [3:0]  dmem_be;
  word_t       dmem_addr, dmem_wdata, dmem_rdata;

  ooo_dmem_model #(.MEM_WORDS(65536), .MEM_MSB(17)) u_dmem (
    .clk(clk), .rst_n(rst_n),
    .addr(dmem_addr), .rdata(dmem_rdata),
    .we(dmem_we), .be(dmem_be), .wdata(dmem_wdata),
    .tohost_addr(32'hFFFF_FFFC), .tohost_full_addr(32'hFFFF_FFFC),
    .tohost_we(), .tohost_val()
  );

  `include "tb/rv32i_ooo_trace_format.svh"
  rv32i_ooo_store_trace_t store_rec;   // canonical STORE record

  string imem_path;
  bit    trace_enable;
  int    max_commits;
  int    commit_count;

  // prediction seam + training loop (LIVE: the real machine)
  logic [1:0] dec_pred_taken;
  word_t      dec_pred_target;
  logic       bp_update_valid, bp_update_taken;
  word_t      bp_update_pc, bp_update_target;

  rv32i_ss_imem_scratchpad #(.LATENCY(`UMBRA_M3_IMEM_LATENCY),
                             .PIPELINED(`UMBRA_M3_IMEM_PIPELINED)) u_imem_adapter (
    .clk(clk), .rst_n(rst_n),
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr), .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready), .imem_resp_data(imem_resp_data),
    .line_addr(imem_addr), .line_data(imem_rdata)
  );

  rv32i_ss_frontend #(
    .RESET_PC(RESET_PC)
  ) u_fe (
    .clk(clk), .rst_n(rst_n),
    .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
    .imem_req_addr(imem_req_addr), .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready), .imem_resp_data(imem_resp_data),
    .redirect_valid(redirect_valid), .redirect_target(redirect_target),
    .bp_update_valid(bp_update_valid), .bp_update_pc(bp_update_pc),
    .bp_update_taken(bp_update_taken), .bp_update_target(bp_update_target),
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
    .decoded_pred_taken(dec_pred_taken),
    .decoded_pred_target(dec_pred_target)
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
    .decoded_pred_taken(dec_pred_taken),
    .decoded_pred_target(dec_pred_target),
    .redirect_valid(redirect_valid), .redirect_target(redirect_target),
    .bp_update_valid(bp_update_valid), .bp_update_pc(bp_update_pc),
    .bp_update_taken(bp_update_taken), .bp_update_target(bp_update_target),
    .commit_fire(commit_fire), .commit_order(commit_order),
    .commit_pc(commit_pc), .commit_inst(commit_inst),
    .commit_rd(commit_rd), .commit_rd_wen(commit_rd_wen),
    .commit_wdata(commit_wdata),
    .dmem_valid(dmem_valid), .dmem_we(dmem_we), .dmem_be(dmem_be),
    .dmem_addr(dmem_addr), .dmem_wdata(dmem_wdata),
    .dmem_ready(1'b1), .dmem_rvalid(1'b1),
    .dmem_rdata(dmem_rdata)
  );

  initial clk = 1'b0;
  always #5 clk = ~clk;

  // global watchdog (independent of the commit-progress flow)
  localparam int WATCHDOG_CYCLES = 50_000;
  int wdog;
  always_ff @(posedge clk) begin
    if (rst_n) begin
      wdog <= wdog + 1;
      if (wdog == WATCHDOG_CYCLES)
        $fatal(1, "OOO_COMMIT_WATCHDOG: no finish after %0d cycles (commits=%0d) -- commit_fire likely still stubbed",
               WATCHDOG_CYCLES, commit_count);
    end
  end

  // commit-order continuity + trace emission + termination
  //
  // PER-RECORD, not per-cycle. The harness walks the fired commit slots
  // in program order and applies the per-entry bookkeeping once per fired
  // slot, so the canonical trace stays a program-ordered line stream with no
  // cycle field -- changes only how many records a cycle produces.
  //
  // Everything that carries state ACROSS records inside one cycle
  // (prev_order, seen_commit, commit_count) is blocking-assigned, so the
  // second record sees the first record's update instead of a stale
  // NBA-deferred value.
  //
  // commit_order is the ROB's pre-advance counter, so record p's order is
  // necessarily commit_order + p (: no duplicate per-slot order seam).
  commit_order_t prev_order;
  bit            seen_commit;
  integer        emit_slot;
  int            fired_stores;
  int            dual_commit_cycles;
  commit_order_t rec_order;
  bit            rec_is_store;
  // Icarus rejects a part-select chained onto a variable array-element select
  // (commit_inst[slot][6:0]) -- copy the whole word first, same workaround the
  // IQ uses for struct-array field reads.
  word_t         rec_inst;

  always @(posedge clk) begin
    if (rst_n && (|commit_fire)) begin
      if (commit_fire == 2'b11)
        dual_commit_cycles = dual_commit_cycles + 1;
      // STORE ATTRIBUTION (addendum): the harness decodes the store
      // from the committed instruction itself rather than echoing an RTL
      // trace-only class bit -- an RTL-sourced flag would mask exactly the
      // misclassification this oracle exists to catch. At most one store may
      // commit per group, so cycle-level dmem_we must agree with exactly one
      // fired store instruction.
      fired_stores = 0;
      for (emit_slot = 0; emit_slot < 2; emit_slot++) begin
        rec_inst = commit_inst[emit_slot];
        if (commit_fire[emit_slot] &&
            (rec_inst[6:0] == fyp_cpu_pkg::OPCODE_STORE))
          fired_stores = fired_stores + 1;
      end
      if (dmem_we && (fired_stores != 1))
        $fatal(1, "STORE_ATTRIB: dmem_we with %0d fired store instructions",
               fired_stores);
      if (!dmem_we && (fired_stores != 0))
        $fatal(1, "STORE_ATTRIB: %0d fired store instructions without dmem_we",
               fired_stores);

      for (emit_slot = 0; emit_slot < 2; emit_slot++) begin
        if (commit_fire[emit_slot]) begin
          rec_order    = commit_order + commit_order_t'(emit_slot);
          rec_inst     = commit_inst[emit_slot];
          rec_is_store = (rec_inst[6:0] == fyp_cpu_pkg::OPCODE_STORE);

          if (seen_commit && (rec_order !== prev_order + 1)) begin
            $fatal(1, "COMMIT_ORDER_GAP prev=%0d now=%0d (slot %0d)",
                   prev_order, rec_order, emit_slot);
          end
          seen_commit = 1'b1;
          prev_order  = rec_order;

          if (trace_enable) begin
            if (commit_rd_wen[emit_slot] && (commit_rd[emit_slot] != 5'd0))
              $display("COMMIT pc=%08h instr=%08h rd=%0d wdata=%08h",
                       commit_pc[emit_slot], commit_inst[emit_slot],
                       commit_rd[emit_slot], commit_wdata[emit_slot]);
            else
              $display("COMMIT pc=%08h instr=%08h rd=0 wdata=00000000",
                       commit_pc[emit_slot], commit_inst[emit_slot]);
            // a committing store emits the canonical STORE record right
            // after ITS OWN COMMIT line (same commit_order;). The
            // commit-diff flow greps '^COMMIT ' so STORE lines never disturb
            // the Spike comparison.
            if (rec_is_store) begin
              // canonical record via the FMT macro; explicit fields because
              // Icarus rejects the EMIT macro's parenthesized member selects
              store_rec.addr         = dmem_addr;
              store_rec.data         = dmem_wdata;
              store_rec.wmask        = dmem_be;
              store_rec.commit_order = rec_order;
              $display(`RV32I_OOO_STORE_TRACE_FMT, store_rec.addr,
                       store_rec.data, store_rec.wmask, store_rec.commit_order);
            end
          end

          // PER-RECORD limit, not per-cycle: an odd MAX_COMMITS emits exactly
          // that many COMMIT lines even when the final cycle retires two.
          commit_count = commit_count + 1;
          if (commit_count >= max_commits) begin
            // cycles = wdog (total cycles since reset; the watchdog never
            // resets). The LSQ differential gate parses this: identical
            // traces, fewer cycles vs the frozen head-only core
            // (tools/run_s1_cycle_diff.sh).
            $display("OOO SPIKE DIFF TRACE DONE commits=%0d cycles=%0d dual_cycles=%0d",
                     commit_count, wdog, dual_commit_cycles);
            $finish;
          end
        end
      end
    end
  end

  task automatic reset_dut();
    rst_n = 1'b0;
    repeat (4) @(posedge clk);
    rst_n = 1'b1;
  endtask

  integer i;
  initial begin
    wdog = 0; commit_count = 0; dual_commit_cycles = 0;
    seen_commit = 1'b0; prev_order = '0;
    trace_enable = $test$plusargs("TRACE");
    if (!$value$plusargs("MAX_COMMITS=%d", max_commits)) max_commits = 16;
    for (i = 0; i < 1024; i++) imem[i] = 32'h00000013;   // NOP fill
    if (!$value$plusargs("IMEM=%s", imem_path))
      $fatal(1, "missing +IMEM=<path>");
    $readmemh(imem_path, imem);
    // Mirror the image into dmem so programs may load initialized data from
    // the text region (same trick as the in-order riscv-test harness).
    for (i = 0; i < 1024; i++) u_dmem.mem[i] = imem[i];
    reset_dut();
    // termination is handled in the commit always-block; watchdog backstops.
  end

endmodule
