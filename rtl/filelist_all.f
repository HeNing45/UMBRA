# Editor-mode filelist: lists every SV source so the Verilator extension
# (which only knows one -f) can resolve symbols regardless of which file
# you have open. NOT used for standalone builds -- those use the
# purpose-specific filelists:
#   rtl/filelist.f            single-cycle (rtl/ only)
#   rtl_p/filelist_pipeline.f 5-stage pipelined (rtl/ leaves + rtl_p/)
# Keep this file flat. Editor Verilator integrations pass one -f file and
# treat each non-comment token as a source path, not as a nested filelist.
rtl/fyp_cpu_pkg.sv
rtl_p/rv32i_pipeline_pkg.sv
rtl_ooo/rv32i_ooo_pkg.sv
rtl_superscalar/rv32i_ss_pkg.sv
rtl/rv32i_alu.sv
rtl/rv32i_decode.sv
rtl/rv32i_imm_gen.sv
rtl/rv32i_pc_logic.sv
rtl/rv32i_regfile.sv
rtl/rv32i_single_cycle_core.sv
rtl_p/rv32i_csr_file.sv
rtl_p/rv32i_branch_cmp.sv
rtl_p/rv32i_muldiv.sv
rtl_p/rv32i_hazard.sv
rtl_p/rv32i_pipe_decode.sv
rtl_p/rv32i_bp.sv
rtl_p/rv32i_pipeline_core.sv
rtl_ooo/rv32i_ooo_decode.sv
rtl_ooo/rv32i_ooo_free_list.sv
rtl_ooo/rv32i_ooo_rename.sv
rtl_ooo/rv32i_ooo_rob.sv
rtl_ooo/rv32i_ooo_prf.sv
rtl_ooo/rv32i_ooo_iq.sv
rtl_ooo/rv32i_ooo_muldiv.sv
rtl_ooo/rv32i_ooo_csr_file.sv
rtl_ooo/rv32i_ooo_core.sv
rtl_ooo/rv32i_ooo_frontend.sv
rtl_superscalar/rv32i_ss_decode.sv
rtl_superscalar/rv32i_ss_free_list.sv
rtl_superscalar/rv32i_ss_rename.sv
rtl_superscalar/rv32i_ss_rob.sv
rtl_superscalar/rv32i_ss_prf.sv
rtl_superscalar/rv32i_ss_iq.sv
rtl_superscalar/rv32i_ss_lsq.sv
rtl_superscalar/rv32i_ss_muldiv.sv
rtl_superscalar/rv32i_ss_csr_file.sv
rtl_superscalar/rv32i_ss_core.sv
rtl_superscalar/rv32i_ss_frontend.sv
tb/tb_rv32i_single_cycle_smoke.sv
