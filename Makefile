# Makefile for dual_DIMC
# Author: Mennatalla Hassan, University of Bologna

# ── Directories ───────────────────────────────────────────────
RTL_DIR     := rtl
TB_DIR      := tb
SIM_DIR     := sim
STIM_DIR    := stimuli
DIMC_STIM_DIR := $(STIM_DIR)/dimc_tests
CLEO_TEST1_STIM_DIR := $(STIM_DIR)/cleo_test1
CLEO_TEST2_STIM_DIR := $(STIM_DIR)/cleo_test2
CLEO_TEST3_STIM_DIR := $(STIM_DIR)/cleo_test3
DOUBLE_BUFFERING_STIM_DIR := $(STIM_DIR)/double_buffering
WORK_DIR    := $(SIM_DIR)/work
COMPILE_TCL := $(SIM_DIR)/compile.tcl

# ── Bender (package manager) ──────────────────────────────────
BENDER_VERSION := 0.31.0
BENDER         := $(SIM_DIR)/bender

# ── Default target ────────────────────────────────────────────
.PHONY: hw-all stim update-ips hw-compile sim-dual sim-double-buffering sim-single sim-datapath sim-cleopatra hw-clean

hw-all: stim hw-compile

# ── Download bender if not in PATH and not already in sim/ ────
_SYSTEM_BENDER := $(shell which bender 2>/dev/null)
ifneq ($(_SYSTEM_BENDER),)
  BENDER := $(_SYSTEM_BENDER)
endif

$(SIM_DIR):
	mkdir -p $(SIM_DIR)

$(SIM_DIR)/bender: | $(SIM_DIR)
	curl -fsSL \
	    "https://github.com/pulp-platform/bender/releases/download/v$(BENDER_VERSION)/bender-$(BENDER_VERSION)-x86_64-linux-gnu-rhel8.10.tar.gz" \
	    | tar -xzf - -C $(SIM_DIR) bender
	chmod +x $@


# ── Fetch IP dependencies ─────────────────────────────────────
update-ips: $(BENDER)
	$(BENDER) update


# ── Generate stimulus ─────────────────────────────────────────
stim:
	python3 $(STIM_DIR)/generate_stim.py --outdir $(STIM_DIR)
	python3 $(STIM_DIR)/cleo_test3_stim.py
	python3 $(STIM_DIR)/double_buffering_stim.py

# ── Compile RTL + TBs ─────────────────────────────────────────
hw-compile: update-ips
	mkdir -p $(SIM_DIR)
	$(BENDER) script vsim        \
	    --vlog-arg="-sv"         \
	    -t tb                    \
	    > $(COMPILE_TCL)
	test -d $(WORK_DIR) || vlib $(WORK_DIR)
	vsim -c -do "vmap work $(WORK_DIR); source $(COMPILE_TCL); quit -f"

# ── Run simulations ───────────────────────────────────────────
# Use GUI=1 to open the QuestaSim GUI instead of batch mode

GUI ?= 0
ifeq ($(GUI),1)
VSIM_FLAGS = -voptargs=+acc
VSIM_DO    = "run -all"
else
VSIM_FLAGS = -c -voptargs=+acc
VSIM_DO    = "run -all; quit"
endif

sim-dual: stim hw-compile
	vsim $(VSIM_FLAGS) -l $(SIM_DIR)/transcript -lib $(WORK_DIR) tb_DIMC_dual -do $(VSIM_DO)

sim-double-buffering: stim hw-compile
	rm -f $(DOUBLE_BUFFERING_STIM_DIR)/double_buffering_accumulator_output.txt
	rm -f $(DOUBLE_BUFFERING_STIM_DIR)/double_buffering_final_matmul_output.txt
	vsim $(VSIM_FLAGS) -l $(SIM_DIR)/transcript -lib $(WORK_DIR) tb_double_buffering -do $(VSIM_DO)

sim-single: stim hw-compile
	vsim $(VSIM_FLAGS) -l $(SIM_DIR)/transcript -lib $(WORK_DIR) tb_DIMC -do $(VSIM_DO)

sim-datapath: stim hw-compile
	vsim $(VSIM_FLAGS) -l $(SIM_DIR)/transcript -lib $(WORK_DIR) tb_dimc_datapath -do $(VSIM_DO)

sim-cleopatra: stim hw-compile
	rm -f $(CLEO_TEST3_STIM_DIR)/test3_accumulator_output.txt
	rm -f $(CLEO_TEST3_STIM_DIR)/test3_final_matmul_output.txt
	vsim $(VSIM_FLAGS) -l $(SIM_DIR)/transcript -lib $(WORK_DIR) tb_cleopatra -do $(VSIM_DO)
	@if test -f $(CLEO_TEST3_STIM_DIR)/test3_accumulator_output.txt; then \
		python3 $(STIM_DIR)/matrix_untiling.py; \
	fi

# ── Remove all generated artefacts ────────────────────────────
hw-clean:
	rm -rf $(WORK_DIR)
	rm -f  $(COMPILE_TCL) $(SIM_DIR)/transcript $(SIM_DIR)/*.vcd
	rm -f  transcript vsim.wlf *.vcd
	rm -f  $(DIMC_STIM_DIR)/*.txt
	rm -f  $(CLEO_TEST1_STIM_DIR)/*.txt
	rm -f  $(CLEO_TEST2_STIM_DIR)/*.txt
	rm -f  $(CLEO_TEST3_STIM_DIR)/*.txt
	rm -f  $(DOUBLE_BUFFERING_STIM_DIR)/*.txt
	rm -f  etch*
