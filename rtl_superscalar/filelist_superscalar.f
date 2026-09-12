rtl/fyp_cpu_pkg.sv
rtl_p/rv32i_pipeline_pkg.sv
rtl/rv32i_alu.sv
rtl/rv32i_imm_gen.sv
rtl_p/rv32i_branch_cmp.sv
rtl_p/rv32i_bp.sv
# Pipeline decoder, wrapped by rv32i_ss_decode.
rtl_p/rv32i_pipe_decode.sv

# --- Superscalar modules: two-wide dispatch and dual execute ---
rtl_superscalar/rv32i_ss_pkg.sv
rtl_superscalar/rv32i_ss_branch_combiner.sv
# Decoder wrapper over the shared pipeline decoder.
rtl_superscalar/rv32i_ss_decode.sv
rtl_superscalar/rv32i_ss_free_list.sv
rtl_superscalar/rv32i_ss_rename.sv
rtl_superscalar/rv32i_ss_rob.sv
rtl_superscalar/rv32i_ss_prf.sv
rtl_superscalar/rv32i_ss_iq.sv
rtl_superscalar/rv32i_ss_select_pipe.sv
rtl_superscalar/rv32i_ss_lsq.sv
# Checkpoint state remains folded into rv32i_ss_rename.sv; no separate module.
# IQ wakeup/select remains standalone and is instantiated by the SS core.
rtl_superscalar/rv32i_ss_muldiv.sv
rtl_superscalar/rv32i_ss_csr_file.sv
# Dual-lookup branch and return-site predictor.
rtl_superscalar/rv32i_ss_bp.sv
rtl_superscalar/rv32i_ss_core.sv
rtl_superscalar/rv32i_ss_imem_zero_latency_adapter.sv
# Instruction-memory timing model. Simulation only: outside the CPU
# synthesis set, whose boundary is umbra_ss_cpu_top's imem ports.
rtl_superscalar/rv32i_ss_imem_scratchpad.sv
# Data-memory timing model. Simulation only, same boundary rule as the
# I-side scratchpad: never added to the island synthesis set.
rtl_superscalar/rv32i_ss_dmem_scratchpad.sv
rtl_superscalar/rv32i_ss_frontend.sv
# Wiring-only CPU top: rv32i_ss_frontend + rv32i_ss_core, zero logic
# (port == net). Exposes clk/rst_n, imem, dmem and commit trace.
rtl_superscalar/umbra_ss_cpu_top.sv
