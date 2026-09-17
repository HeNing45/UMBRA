// SPDX-FileCopyrightText: Copyright 2026 He Ning
// SPDX-License-Identifier: Apache-2.0

// t36_harness.svh — shared program-testbench harness body.
//
// Include INSIDE a module. Provides: clock/reset, frontend+core DUT pair,
// 256-word imem, a delay-controllable dmem model, X-strict standing
// request/response and recovery tripwires, check helpers,
// and an independent free-list conservation oracle.
//
// The including module must define:
//   `define T36_TB_NAME "tb_name"     (string literal, for messages)
// and may override:
//   `define T36_WATCHDOG_CYCLES 4000
//
// DUT = rv32i_ss_frontend + rv32i_ss_core.

`ifndef T36_WATCHDOG_CYCLES
`define T36_WATCHDOG_CYCLES 4000
`endif

localparam time CLK_PERIOD = 10ns;

logic clk;
logic rst_n;

// frontend <-> core decoded packet
logic                dec_valid;
logic [1:0]          dec_slot_valid;
logic                dec_ready;
word_t [1:0]         dec_pc;
word_t [1:0]         dec_instr;
arch_reg_t [1:0]     dec_rs1;
arch_reg_t [1:0]     dec_rs2;
arch_reg_t [1:0]     dec_rd;
logic [1:0]          dec_rd_we;
logic [1:0]          dec_needs_checkpoint;
ooo_op_class_e [1:0] dec_op_class;
ooo_fu_class_e [1:0] dec_fu_class;
muldiv_op_e [1:0]    dec_muldiv_op;
alu_op_e [1:0]       dec_alu_op;
br_type_e [1:0]      dec_branch_op;
ooo_src_sel_e [1:0]  dec_src1_sel;
ooo_src_sel_e [1:0]  dec_src2_sel;
word_t [1:0]         dec_imm;
decoded_trap_t       dec_trap;
csr_op_e             dec_csr_op;
csr_addr_t           dec_csr_addr;
csr_zimm_t           dec_csr_zimm;
logic [1:0]          dec_is_load;
logic [1:0]          dec_is_store;
mem_size_e [1:0]     dec_mem_size;
logic [1:0]          dec_mem_unsigned;

// instruction memory (64-bit aligned line, as tb_rv32i_ss_core_prog)
word_t       imem_addr;
word_t [1:0] imem_rdata;
logic        imem_req_valid, imem_req_ready;
word_t       imem_req_addr;
logic        imem_resp_valid, imem_resp_ready;
word_t [1:0] imem_resp_data;
logic [31:0] imem [0:255];
assign imem_rdata = {imem[{imem_addr[9:3], 1'b1}], imem[{imem_addr[9:3], 1'b0}]};

rv32i_ss_imem_zero_latency_adapter u_imem_adapter (
  .imem_req_valid(imem_req_valid), .imem_req_ready(imem_req_ready),
  .imem_req_addr(imem_req_addr), .imem_resp_valid(imem_resp_valid),
  .imem_resp_ready(imem_resp_ready), .imem_resp_data(imem_resp_data),
  .line_addr(imem_addr), .line_data(imem_rdata)
);

// redirect
logic  redirect_valid;
word_t redirect_target;

// commit trace taps
logic [1:0]    commit_fire;
commit_order_t commit_order;
word_t [1:0]   commit_pc;
word_t [1:0]   commit_inst;
arch_reg_t [1:0] commit_rd;
logic [1:0]    commit_rd_wen;
word_t [1:0]   commit_wdata;

// dmem
logic       dmem_valid, dmem_we;
logic [3:0] dmem_be;
word_t      dmem_addr, dmem_wdata;
logic       dmem_rvalid;
word_t      dmem_rdata;

int errors = 0;
int checks = 0;

rv32i_ss_frontend u_fe (
  .clk                      (clk),
  .rst_n                    (rst_n),
  .imem_req_valid           (imem_req_valid),
  .imem_req_ready           (imem_req_ready),
  .imem_req_addr            (imem_req_addr),
  .imem_resp_valid          (imem_resp_valid),
  .imem_resp_ready          (imem_resp_ready),
  .imem_resp_data           (imem_resp_data),
    .bp_update_valid(1'b0),  // tie-off: predictor never trains (inert)
    .bp_update_pc('0), .bp_update_taken(1'b0), .bp_update_target('0),
  .redirect_valid           (redirect_valid),
  .redirect_target          (redirect_target),
  .decoded_valid            (dec_valid),
  .decoded_slot_valid       (dec_slot_valid),
  .decoded_ready            (dec_ready),
  .decoded_pc               (dec_pc),
  .decoded_instr            (dec_instr),
  .decoded_rs1              (dec_rs1),
  .decoded_rs2              (dec_rs2),
  .decoded_rd               (dec_rd),
  .decoded_rd_we            (dec_rd_we),
  .decoded_needs_checkpoint (dec_needs_checkpoint),
  .decoded_op_class         (dec_op_class),
  .decoded_fu_class         (dec_fu_class),
  .decoded_muldiv_op        (dec_muldiv_op),
  .decoded_alu_op           (dec_alu_op),
  .decoded_branch_op        (dec_branch_op),
  .decoded_src1_sel         (dec_src1_sel),
  .decoded_src2_sel         (dec_src2_sel),
  .decoded_imm              (dec_imm),
  .decoded_trap             (dec_trap),
  .decoded_csr_op           (dec_csr_op),
  .decoded_csr_addr         (dec_csr_addr),
  .decoded_csr_zimm         (dec_csr_zimm),
  .decoded_is_load          (dec_is_load),
  .decoded_is_store         (dec_is_store),
  .decoded_mem_size         (dec_mem_size),
  .decoded_mem_unsigned     (dec_mem_unsigned)
);

rv32i_ss_core u_core (
  .clk                      (clk),
  .rst_n                    (rst_n),
  .decoded_valid            (dec_valid),
  .decoded_slot_valid       (dec_slot_valid),
  .decoded_ready            (dec_ready),
  .decoded_pc               (dec_pc),
  .decoded_instr            (dec_instr),
  .decoded_rs1              (dec_rs1),
  .decoded_rs2              (dec_rs2),
  .decoded_rd               (dec_rd),
  .decoded_rd_we            (dec_rd_we),
  .decoded_needs_checkpoint (dec_needs_checkpoint),
  .decoded_op_class         (dec_op_class),
  .decoded_fu_class         (dec_fu_class),
  .decoded_muldiv_op        (dec_muldiv_op),
  .decoded_alu_op           (dec_alu_op),
  .decoded_branch_op        (dec_branch_op),
  .decoded_src1_sel         (dec_src1_sel),
  .decoded_src2_sel         (dec_src2_sel),
  .decoded_imm              (dec_imm),
  .decoded_trap             (dec_trap),
  .decoded_csr_op           (dec_csr_op),
  .decoded_csr_addr         (dec_csr_addr),
  .decoded_csr_zimm         (dec_csr_zimm),
  .decoded_is_load          (dec_is_load),
  .decoded_is_store         (dec_is_store),
  .decoded_mem_size         (dec_mem_size),
  .decoded_mem_unsigned     (dec_mem_unsigned),
    .decoded_pred_taken('0),  // tie-off: static not-taken prediction
    .decoded_pred_target('0),
  .redirect_valid           (redirect_valid),
  .redirect_target          (redirect_target),
  .commit_fire              (commit_fire),
  .commit_order             (commit_order),
  .commit_pc                (commit_pc),
  .commit_inst              (commit_inst),
  .commit_rd                (commit_rd),
  .commit_rd_wen            (commit_rd_wen),
  .commit_wdata             (commit_wdata),
  .dmem_valid               (dmem_valid),
  .dmem_we                  (dmem_we),
  .dmem_be                  (dmem_be),
  .dmem_addr                (dmem_addr),
  .dmem_wdata               (dmem_wdata),
  .dmem_ready               (1'b1),
  .dmem_rvalid              (dmem_rvalid),
  .dmem_rdata               (dmem_rdata)
);

`define CORE u_core
`define ROB  u_core.u_rob
`define RN   u_core.u_rename
`define PRF  u_core.u_prf
`define FL   u_core.u_free_list
`define IQ   u_core.u_iq
`define LSQ  u_core.u_lsq
`define MD   u_core.u_muldiv

initial clk = 1'b0;
always #(CLK_PERIOD/2) clk = ~clk;

int cyc;
always @(posedge clk) begin
  if (!rst_n) cyc <= 0;        // synchronous clear: immune to the same-edge
  else        cyc <= cyc + 1;  // NBA race a task-side blocking clear loses
end

initial begin
  repeat (`T36_WATCHDOG_CYCLES) @(posedge clk);
  $fatal(1, "WATCHDOG: %s exceeded %0d cycles", `T36_TB_NAME, `T36_WATCHDOG_CYCLES);
end

// ---------------- dmem model with load-response delay control ----------------
// Stores: immediate (dmem_ready tied 1), byte-enable merge at the edge.
// Loads: answered combinationally when cyc >= load_release_cyc; an earlier
// request goes pending (address latched) and is answered exactly when the
// release cycle arrives (the LSQ pairs it via its outstanding FIFO head). Default
// release 0 = always immediate. This is the holder_recovery TB's pattern,
// generalized with a real backing array.
logic [31:0] dmem_mem [0:1023];
int    load_release_cyc  = 0;
int    load_release_cyc2 = 0;
word_t load_release2_addr = 32'hFFFF_FFFF;  // disabled unless a TB points it

function automatic int t36_rel_for(input word_t a);
  t36_rel_for = (a == load_release2_addr) ? load_release_cyc2 : load_release_cyc;
endfunction

// The responder returns data in request-acceptance order. A two-entry queue
// retains accepted reads; only the head can respond when its release cycle
// arrives. With an empty queue, an already-released read responds in the
// acceptance cycle. A third pending read is a model-conformance failure.
// Returning a younger read early would pair its data with the CPU's older
// outstanding identity and corrupt architectural state.
logic       dp_head_q;
logic [1:0] dp_count_q;
word_t      dp_addr_q [0:1];
word_t      dp_head_addr;
logic       dp_head_ready;
logic       dp_new_read;
logic       dp_same_cycle;
logic       dp_push;
logic       dp_pop;
logic       dp_tail;

always_comb begin
  dp_head_addr  = dp_head_q ? dp_addr_q[1] : dp_addr_q[0];
  dp_head_ready = (dp_count_q != 2'd0) &&
                  (cyc >= t36_rel_for(dp_head_addr));
  dp_new_read   = dmem_valid && !dmem_we;
  dp_same_cycle = dp_new_read && (dp_count_q == 2'd0) &&
                  (cyc >= t36_rel_for(dmem_addr));
  dp_pop        = dp_head_ready;
  dp_push       = dp_new_read && !dp_same_cycle;
  dp_tail       = dp_head_q ^ dp_count_q[0];
end

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    dp_head_q    <= 1'b0;
    dp_count_q   <= 2'd0;
    dp_addr_q[0] <= '0;
    dp_addr_q[1] <= '0;
  end else begin
    if (dmem_valid && dmem_we) begin
      if (dmem_be[0]) dmem_mem[dmem_addr[11:2]][7:0]   <= dmem_wdata[7:0];
      if (dmem_be[1]) dmem_mem[dmem_addr[11:2]][15:8]  <= dmem_wdata[15:8];
      if (dmem_be[2]) dmem_mem[dmem_addr[11:2]][23:16] <= dmem_wdata[23:16];
      if (dmem_be[3]) dmem_mem[dmem_addr[11:2]][31:24] <= dmem_wdata[31:24];
    end
    if (dp_push && (dp_count_q == 2'd2) && !dp_pop)
      $fatal(1, "%s: t36 dmem model: third pending read (K=2 capacity)",
             `T36_TB_NAME);
    if (dp_push) begin
      if (dp_tail) dp_addr_q[1] <= dmem_addr;
      else         dp_addr_q[0] <= dmem_addr;
    end
    if (dp_pop) dp_head_q <= !dp_head_q;
    case ({dp_push, dp_pop})
      2'b10:   dp_count_q <= dp_count_q + 2'd1;
      2'b01:   dp_count_q <= dp_count_q - 2'd1;
      default: ;
    endcase
  end
end

always_comb begin
  dmem_rvalid = 1'b0;
  dmem_rdata  = '0;
  if (dp_head_ready) begin
    dmem_rvalid = 1'b1;
    dmem_rdata  = dmem_mem[dp_head_addr[11:2]];
  end else if (dp_same_cycle) begin
    dmem_rvalid = 1'b1;
    dmem_rdata  = dmem_mem[dmem_addr[11:2]];
  end
end

// ---------------X-strict standing tripwires (protocol invariants) ----
// Any X on a sampled control signal is a hard failure, so unknowns can never
// satisfy a pass path. Broadcast exclusivity: no dispatch fire, no issue
// fire, no nonzero release set on a recovery/trap broadcast cycle.
logic t36_broadcast;
assign t36_broadcast = `CORE.branch_recover_req | `CORE.trap_q_valid;

always @(posedge clk) begin
  if (rst_n === 1'b1) begin
    if ($isunknown(`CORE.bundle_fire) || $isunknown(`CORE.branch_recover_req) ||
        $isunknown(`CORE.trap_q_valid) || $isunknown(`CORE.issue_fire) ||
        $isunknown(`CORE.rob_wb_accept) ||
        $isunknown(`CORE.cdb_q[0].valid) || $isunknown(`CORE.cdb_q[1].valid) ||
        $isunknown(`CORE.checkpoint_release_mask)) begin
      $display("X-map: bf=%b(%0d) rec=%b(%0d) trap=%b(%0d) if=%b(%0d) wba=%b(%0d) c0=%b(%0d) c1=%b(%0d) rm=%b(%0d)",
               `CORE.bundle_fire, $isunknown(`CORE.bundle_fire),
               `CORE.branch_recover_req, $isunknown(`CORE.branch_recover_req),
               `CORE.trap_q_valid, $isunknown(`CORE.trap_q_valid),
               `CORE.issue_fire, $isunknown(`CORE.issue_fire),
               `CORE.rob_wb_accept, $isunknown(`CORE.rob_wb_accept),
               `CORE.cdb_q[0].valid, $isunknown(`CORE.cdb_q[0].valid),
               `CORE.cdb_q[1].valid, $isunknown(`CORE.cdb_q[1].valid),
               `CORE.checkpoint_release_mask, $isunknown(`CORE.checkpoint_release_mask));
      $fatal(1, "%s: X on a sampled control signal at cyc=%0d", `T36_TB_NAME, cyc);
    end
    if (`CORE.bundle_fire && t36_broadcast) begin
      $fatal(1, "%s: bundle_fire on a broadcast cycle (docs/18 S3 clause 3)", `T36_TB_NAME);
    end
    if ((|`CORE.issue_fire) && t36_broadcast) begin
      $fatal(1, "%s: issue fire on a broadcast cycle (docs/18 S3 clause 3)", `T36_TB_NAME);
    end
    if ((|`CORE.checkpoint_release_mask) && t36_broadcast) begin
      $fatal(1, "%s: nonzero release set on a broadcast cycle (U4/U5)", `T36_TB_NAME);
    end
  end
end

// ---------------- shape-11 / dual-issue provably-entered counters ----------
int t36_shape11_fires  = 0;
int t36_dual_issues    = 0;
always @(posedge clk) begin
  if (rst_n === 1'b1) begin
    if (`CORE.bundle_fire && (dec_slot_valid == 2'b11)) t36_shape11_fires++;
    if (`CORE.issue_fire == 2'b11)                      t36_dual_issues++;
  end
end

// ---------------- check helpers ----------------
task automatic check_bit(input string name, input logic got, input logic exp);
  checks++;
  if (got !== exp) begin
    $error("[%s] got=%0b exp=%0b", name, got, exp);
    errors++;
  end
endtask

task automatic check_word(input string name, input word_t got, input word_t exp);
  checks++;
  if (got !== exp) begin
    $error("[%s] got=%h exp=%h", name, got, exp);
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

// Architectural register value via the committed map (X-strict).
task automatic check_arch(input string name, input int r, input word_t exp);
  automatic phys_reg_t p = `RN.committed_map_q[r[4:0]];
  checks++;
  if (`PRF.regs_q[p] !== exp) begin
    $error("[%s] x%0d=%08h (via p%0d) expected %08h",
           name, r, `PRF.regs_q[p], p, exp);
    errors++;
  end
endtask

// Data-memory word check (after commit drain).
task automatic check_dmem(input string name, input word_t addr, input word_t exp);
  checks++;
  if (dmem_mem[addr[11:2]] !== exp) begin
    $error("[%s] mem[%08h]=%08h expected %08h",
           name, addr, dmem_mem[addr[11:2]], exp);
    errors++;
  end
endtask

// Bounded wait for N total commits; a stall is a hard nonzero failure —
// this is what makes the silent-fatal direction (dropped survivor beat)
// loud under mutation.
task automatic wait_commits(input int n, input int bound);
  automatic int w = 0;
  while (commit_order < commit_order_t'(n)) begin
    @(posedge clk);
    w++;
    if (w > bound) begin
      $fatal(1, "%s: STALL waiting for commit %0d (reached %0d) after %0d cycles",
             `T36_TB_NAME, n, commit_order, bound);
    end
  end
endtask

// Independent free-list conservation oracle (bitmap scheme):
// at a quiescent point (all real writers committed; only NOPs flowing) the
// free set must hold exactly PHYS-ARCH registers, p0 never free, and every
// committed-map register must be excluded from the free set and pairwise
// distinct. Derived from the allocator CONTRACT, not from the RTL equations.
task automatic check_freelist_conservation(input string name);
  automatic int freec = 0;
  automatic int i, j;
  automatic phys_reg_t pi, pj;
  for (i = 0; i < OOO_PHYS_REGS; i++) begin
    checks++;
    if ($isunknown(`FL.free_bits_q[i])) begin
      $error("[%s] free_bits_q[%0d] is X", name, i);
      errors++;
    end
    if (`FL.free_bits_q[i] === 1'b1) freec++;
  end
  check_int({name, ": free count == PHYS-ARCH"}, freec,
            OOO_PHYS_REGS - OOO_ARCH_REGS);
  check_bit({name, ": p0 not free"}, `FL.free_bits_q[0], 1'b0);
  for (i = 0; i < OOO_ARCH_REGS; i++) begin
    pi = `RN.committed_map_q[i[4:0]];
    checks++;
    if (`FL.free_bits_q[pi] !== 1'b0) begin
      $error("[%s] committed map x%0d -> p%0d is marked free", name, i, pi);
      errors++;
    end
    for (j = i + 1; j < OOO_ARCH_REGS; j++) begin
      pj = `RN.committed_map_q[j[4:0]];
      if (pi === pj) begin
        checks++;
        $error("[%s] committed map aliases: x%0d and x%0d -> p%0d", name, i, j, pi);
        errors++;
      end
    end
  end
endtask

task automatic t36_reset();
  automatic int i;
  rst_n = 1'b0;
  load_release_cyc  = 0;
  load_release_cyc2 = 0;
  load_release2_addr = 32'hFFFF_FFFF;
  for (i = 0; i < 1024; i++) dmem_mem[i] = 32'h0000_0000;
  repeat (3) @(posedge clk);
  @(negedge clk);
  rst_n = 1'b1;
  @(negedge clk);
endtask

task automatic t36_finish();
  if (errors == 0) begin
    $display("[%s] PASS checks=%0d", `T36_TB_NAME, checks);
    $finish;
  end else begin
    $display("[%s] FAIL checks=%0d errors=%0d", `T36_TB_NAME, checks, errors);
    $fatal(1, "[%s] FAIL", `T36_TB_NAME);
  end
endtask
