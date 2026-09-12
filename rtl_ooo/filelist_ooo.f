rtl/fyp_cpu_pkg.sv
rtl_p/rv32i_pipeline_pkg.sv
rtl/rv32i_alu.sv
rtl/rv32i_imm_gen.sv
rtl_p/rv32i_branch_cmp.sv
rtl_p/rv32i_bp.sv
# Shared pipeline decoder, wrapped by rv32i_ooo_decode.
rtl_p/rv32i_pipe_decode.sv

# --- Scalar OoO modules ---
rtl_ooo/rv32i_ooo_pkg.sv
# OoO control adaptation over the pipeline decoder.
rtl_ooo/rv32i_ooo_decode.sv
rtl_ooo/rv32i_ooo_free_list.sv
rtl_ooo/rv32i_ooo_rename.sv
rtl_ooo/rv32i_ooo_rob.sv
rtl_ooo/rv32i_ooo_prf.sv
rtl_ooo/rv32i_ooo_iq.sv
# Checkpoint state is owned by rv32i_ooo_rename.sv.
# Decoupled issue queue with registered-ready wakeup and oldest-ready select.
rtl_ooo/rv32i_ooo_muldiv.sv
rtl_ooo/rv32i_ooo_csr_file.sv
rtl_ooo/rv32i_ooo_core.sv
rtl_ooo/rv32i_ooo_frontend.sv
# (rv32i_ooo_core.sv = OoO core; rv32i_ooo_frontend.sv = fetch/decode/adapt)
