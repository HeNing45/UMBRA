# Small local entry points for simulation and benchmark checks.
PYTHON ?= python3
CROSS ?= riscv-none-elf-
TB ?= tb_rv32i_ss_iq
MODE ?= performance
ITERATIONS ?= 1
BENCH ?= crc32
TIMEOUT ?= 300
JOBS ?= 4
FILELIST := rtl_superscalar/filelist_superscalar.f
RTL := $(wildcard rtl/*.sv rtl_p/*.sv rtl_p/*.svh rtl_superscalar/*.sv)
CM := sw/coremark/upstream
CMPORT := sw/coremark/umbra
EM := sw/embench/upstream
EMPORT := sw/embench/umbra
SIM := build/obj/umbra_sim
CFLAGS := -O2 -g -march=rv32im -mabi=ilp32 -mcmodel=medlow -mstrict-align \
          -mno-relax -msmall-data-limit=0 -ffreestanding -fno-common -fno-pic \
          -fno-pie -fno-stack-protector -ffunction-sections -fdata-sections
LDFLAGS := -nostdlib -nostartfiles -static -Wl,--gc-sections -Wl,--no-relax

.PHONY: help lint test simulator coremark embench spike
help:
	@echo 'make lint | test TB=<module> | coremark MODE=performance|validation | embench BENCH=<name>|all | spike'
	@echo 'Tools: Icarus, Verilator, Python 3, a C++ compiler, RISC-V GCC/binutils, and Spike for trace comparison.'

lint:
	@mkdir -p build/lint
	verilator --lint-only -sv --Mdir build/lint --top-module rv32i_ss_core -f $(FILELIST)

test:
	@mkdir -p build/tests
	iverilog -g2012 -s $(TB) -o build/tests/$(TB).vvp -f $(FILELIST) tb/$(TB).sv tb/ooo_dmem_model.sv
	$(PYTHON) verification/run.py test --timeout $(TIMEOUT) --log build/tests/$(TB).log vvp build/tests/$(TB).vvp

simulator: $(SIM)
$(SIM): $(RTL) $(FILELIST) tb/tb_rv32i_ss_core_coremark.sv verification/umbra_coremark_sim_main.cpp
	@mkdir -p build/obj
	verilator --cc --exe --build -O3 -j $(JOBS) --top-module umbra_coremark_sim_top \
	  --Mdir $(abspath build/obj) -o umbra_sim -f $(FILELIST) \
	  tb/tb_rv32i_ss_core_coremark.sv $(abspath verification/umbra_coremark_sim_main.cpp)

coremark: simulator
	@test '$(MODE)' = performance -o '$(MODE)' = validation
	@$(PYTHON) -c 'assert int("$(ITERATIONS)") > 0'
	@mkdir -p build/coremark/$(MODE)-$(ITERATIONS)
	$(CROSS)gcc $(CFLAGS) -fno-builtin -Wall -Wextra -Wno-unused-parameter \
	  -I$(CM) -I$(CMPORT) -DITERATIONS=$(ITERATIONS) -DTOTAL_DATA_SIZE=2000 -DSTANDALONE=1 \
	  -D$(if $(filter validation,$(MODE)),VALIDATION_RUN,PERFORMANCE_RUN)=1 \
	  '-DFLAGS_STR="-O2 -march=rv32im -mabi=ilp32 -ffreestanding"' \
	  $(CMPORT)/start.S $(wildcard $(CM)/*.c) $(CMPORT)/core_portme.c \
	  $(LDFLAGS) -Wl,-T,$(CMPORT)/link.ld -o build/coremark/$(MODE)-$(ITERATIONS)/coremark.elf -lgcc
	$(CROSS)objcopy -O verilog --verilog-data-width=4 build/coremark/$(MODE)-$(ITERATIONS)/coremark.elf build/coremark/$(MODE)-$(ITERATIONS)/coremark.mem
	$(PYTHON) verification/run.py coremark --mode $(MODE) --timeout $(TIMEOUT) \
	  --log build/coremark/$(MODE)-$(ITERATIONS)/rtl.log $(SIM) \
	  +IMEM=build/coremark/$(MODE)-$(ITERATIONS)/coremark.mem +MAX_CYCLES=100000000

ifeq ($(BENCH),all)
embench: simulator
	@set -e; for b in $(notdir $(wildcard $(EM)/src/*)); do $(MAKE) embench BENCH=$$b; done
else
embench: simulator
	@test -d $(EM)/src/$(BENCH)
	@mkdir -p build/embench/$(BENCH)
	$(CROSS)gcc $(CFLAGS) -std=gnu17 -Wall -Wno-unused-parameter \
	  -I$(EM)/support -I$(EMPORT) -I$(EM)/src/$(BENCH) \
	  -DHAVE_CONFIG_H -DWARMUP_HEAT=1 -DGLOBAL_SCALE_FACTOR=1 \
	  $(EMPORT)/start.S $(EM)/support/main.c $(EM)/support/beebsc.c \
	  $(EM)/support/board.c $(EM)/support/chip.c $(wildcard $(EM)/src/$(BENCH)/*.c) \
	  $(LDFLAGS) -Wl,-T,$(EMPORT)/link.ld -o build/embench/$(BENCH)/$(BENCH).elf -lm -lgcc
	$(CROSS)objcopy -O verilog --verilog-data-width=4 build/embench/$(BENCH)/$(BENCH).elf build/embench/$(BENCH)/$(BENCH).mem
	$(PYTHON) verification/run.py embench --timeout $(TIMEOUT) --log build/embench/$(BENCH)/rtl.log \
	  $(SIM) +IMEM=build/embench/$(BENCH)/$(BENCH).mem +MAX_CYCLES=200000000
endif

spike:
	@mkdir -p build/spike
	$(CROSS)gcc -march=rv32im -mabi=ilp32 -nostdlib -nostartfiles -Wl,-N -Ttext=0x80000000 -o build/spike/alu.elf sw/ooo_m2_alu.S
	$(CROSS)objcopy --change-addresses=-0x80000000 -O verilog --verilog-data-width=4 build/spike/alu.elf build/spike/alu.mem
	iverilog -g2012 -s tb_rv32i_ss_core_spike_diff -o build/spike/rtl.vvp \
	  -f $(FILELIST) tb/tb_rv32i_ss_core_spike_diff.sv tb/ooo_dmem_model.sv
	$(PYTHON) verification/run.py spike --timeout $(TIMEOUT)
