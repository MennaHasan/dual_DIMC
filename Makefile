# Makefile for dual_DIMC
# Author: Mennatalla Hassan, University of Bologna

# ── Directories ───────────────────────────────────────────────
RTL_DIR     := rtl
TB_DIR      := tb
SIM_DIR     := sim
STIM_DIR    := stimuli
WORK_DIR    := $(SIM_DIR)/work
COMPILE_TCL := $(SIM_DIR)/compile.tcl

# ── Bender (package manager) ──────────────────────────────────
BENDER_VERSION := 0.31.0
BENDER         := $(SIM_DIR)/bender

# ── Default target ────────────────────────────────────────────
.PHONY: hw-all stim update-ips hw-compile sim-dual sim-single hw-clean

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

sim-single: stim hw-compile
	vsim $(VSIM_FLAGS) -l $(SIM_DIR)/transcript -lib $(WORK_DIR) tb_DIMC -do $(VSIM_DO)

# ── Remove all generated artefacts ────────────────────────────
hw-clean:
	rm -rf $(WORK_DIR)
	rm -f  $(COMPILE_TCL) $(SIM_DIR)/transcript $(SIM_DIR)/*.vcd
	rm -f  transcript vsim.wlf *.vcd
	rm -f  $(STIM_DIR)/*.txt
	rm -f  etch*
