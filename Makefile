# Small local entry points for simulation and benchmark checks.
PYTHON ?= python3
CROSS ?= riscv-none-elf-
TB ?= tb_rv32i_ss_iq
MODE ?= performance
ITERATIONS ?= 1
BENCH ?= crc32
TIMEOUT ?= 300
JOBS ?= 4
MEMORY ?= zero
PROFILE ?= o2
FILELIST := rtl_superscalar/filelist_superscalar.f
RTL := $(wildcard rtl/*.sv rtl_p/*.sv rtl_p/*.svh rtl_superscalar/*.sv)
CM := sw/coremark/upstream
CMPORT := sw/coremark/umbra
EM := sw/embench/upstream
EMPORT := sw/embench/umbra
SIM_DIR := build/obj/$(MEMORY)
SIM := $(SIM_DIR)/umbra_sim
MEMORY_DEFINES :=
ifeq ($(MEMORY),delayed)
MEMORY_DEFINES := +define+UMBRA_M3_IMEM_LATENCY=1 +define+UMBRA_M3_IMEM_PIPELINED=1 \
                  +define+UMBRA_M4_DMEM_RESP_LATENCY=1 +define+UMBRA_M4_MAX_OUTSTANDING=2
else ifneq ($(MEMORY),zero)
$(error MEMORY must be zero or delayed)
endif
ifeq ($(PROFILE),o2)
OPT_FLAGS := -O2
else ifeq ($(PROFILE),max)
OPT_FLAGS := -O3 -funroll-all-loops -finline-limit=1000 -falign-functions=8 -falign-jumps=8 -falign-loops=8
else
$(error PROFILE must be o2 or max)
endif
CM_OUT := build/benchmarks/$(MEMORY)/$(PROFILE)/coremark/$(MODE)-$(ITERATIONS)
EM_OUT := build/benchmarks/$(MEMORY)/$(PROFILE)/embench/$(BENCH)
CFLAGS := $(OPT_FLAGS) -g -march=rv32im -mabi=ilp32 -mcmodel=medlow -mstrict-align \
          -mno-relax -msmall-data-limit=0 -ffreestanding -fno-common -fno-pic \
          -fno-pie -fno-stack-protector -ffunction-sections -fdata-sections
LDFLAGS := -nostdlib -nostartfiles -static -Wl,--gc-sections -Wl,--no-relax

.PHONY: help lint test simulator coremark embench spike
help:
	@echo 'make lint | test TB=<module> | coremark MODE=performance|validation | embench BENCH=<name>|all | spike'
	@echo 'Tools: Icarus, Verilator, Python 3, a C++ compiler, RISC-V GCC/binutils, and Spike for trace comparison.'
	@echo 'Benchmarks: MEMORY=zero (default) or MEMORY=delayed (one-cycle responses, two outstanding reads).'
	@echo 'Compiler profile: PROFILE=o2 (default) or PROFILE=max. Results are stored separately.'

lint:
	@mkdir -p build/lint
	verilator --lint-only -sv --Mdir build/lint --top-module rv32i_ss_core -f $(FILELIST)

test:
	@mkdir -p build/tests
	iverilog -g2012 -s $(TB) -o build/tests/$(TB).vvp -f $(FILELIST) tb/$(TB).sv tb/ooo_dmem_model.sv
	$(PYTHON) verification/run.py test --timeout $(TIMEOUT) --log build/tests/$(TB).log vvp build/tests/$(TB).vvp

simulator: $(SIM)
$(SIM): $(RTL) $(FILELIST) tb/tb_rv32i_ss_core_coremark.sv verification/umbra_coremark_sim_main.cpp Makefile
	@mkdir -p $(SIM_DIR)
	verilator --cc --exe --build -O3 -j $(JOBS) --top-module umbra_coremark_sim_top \
	  --Mdir $(abspath $(SIM_DIR)) -o umbra_sim $(MEMORY_DEFINES) -f $(FILELIST) \
	  tb/tb_rv32i_ss_core_coremark.sv $(abspath verification/umbra_coremark_sim_main.cpp)

coremark: simulator
	@test '$(MODE)' = performance -o '$(MODE)' = validation
	@$(PYTHON) -c 'assert int("$(ITERATIONS)") > 0'
	@mkdir -p $(CM_OUT)
	$(CROSS)gcc $(CFLAGS) -fno-builtin -Wall -Wextra -Wno-unused-parameter \
	  -I$(CM) -I$(CMPORT) -DITERATIONS=$(ITERATIONS) -DTOTAL_DATA_SIZE=2000 -DSTANDALONE=1 \
	  -D$(if $(filter validation,$(MODE)),VALIDATION_RUN,PERFORMANCE_RUN)=1 \
	  '-DFLAGS_STR="$(OPT_FLAGS) -march=rv32im -mabi=ilp32 -ffreestanding"' \
	  '-DMEM_LOCATION="RTL $(MEMORY) memory profile"' \
	  $(CMPORT)/start.S $(wildcard $(CM)/*.c) $(CMPORT)/core_portme.c \
	  $(LDFLAGS) -Wl,-T,$(CMPORT)/link.ld -o $(CM_OUT)/coremark.elf -lgcc
	$(CROSS)objcopy -O verilog --verilog-data-width=4 $(CM_OUT)/coremark.elf $(CM_OUT)/coremark.mem
	$(PYTHON) verification/run.py coremark --mode $(MODE) --timeout $(TIMEOUT) \
	  --log $(CM_OUT)/rtl.log $(SIM) \
	  +IMEM=$(CM_OUT)/coremark.mem +MAX_CYCLES=100000000

ifeq ($(BENCH),all)
embench: simulator
	@set -e; for b in $(notdir $(wildcard $(EM)/src/*)); do $(MAKE) embench BENCH=$$b; done
else
embench: simulator
	@test -d $(EM)/src/$(BENCH)
	@mkdir -p $(EM_OUT)
	$(CROSS)gcc $(CFLAGS) -std=gnu17 -Wall -Wno-unused-parameter \
	  -I$(EM)/support -I$(EMPORT) -I$(EM)/src/$(BENCH) \
	  -DHAVE_CONFIG_H -DWARMUP_HEAT=1 -DGLOBAL_SCALE_FACTOR=1 \
	  $(EMPORT)/start.S $(EM)/support/main.c $(EM)/support/beebsc.c \
	  $(EM)/support/board.c $(EM)/support/chip.c $(wildcard $(EM)/src/$(BENCH)/*.c) \
	  $(LDFLAGS) -Wl,-T,$(EMPORT)/link.ld -o $(EM_OUT)/$(BENCH).elf -lm -lgcc
	$(CROSS)objcopy -O verilog --verilog-data-width=4 $(EM_OUT)/$(BENCH).elf $(EM_OUT)/$(BENCH).mem
	$(PYTHON) verification/run.py embench --timeout $(TIMEOUT) --log $(EM_OUT)/rtl.log \
	  $(SIM) +IMEM=$(EM_OUT)/$(BENCH).mem +MAX_CYCLES=200000000
endif

spike:
	@mkdir -p build/spike
	$(CROSS)gcc -march=rv32im -mabi=ilp32 -nostdlib -nostartfiles -Wl,-N -Ttext=0x80000000 -o build/spike/alu.elf sw/ooo_m2_alu.S
	$(CROSS)objcopy --change-addresses=-0x80000000 -O verilog --verilog-data-width=4 build/spike/alu.elf build/spike/alu.mem
	iverilog -g2012 -s tb_rv32i_ss_core_spike_diff -o build/spike/rtl.vvp \
	  -f $(FILELIST) tb/tb_rv32i_ss_core_spike_diff.sv tb/ooo_dmem_model.sv
	$(PYTHON) verification/run.py spike --timeout $(TIMEOUT)
