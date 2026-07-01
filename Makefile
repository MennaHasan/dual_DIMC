# Makefile for dual_DIMC


# Author:
# Mennatalla Hassan, University of Bologna

BENDER ?= /srv/home/mennatalla.hassan/spatz_DIMC-project/install/bender/bender

# ── Directories ───────────────────────────────────────────────
RTL_DIR  := rtl
TB_DIR   := tb
SIM_DIR  := sim
STIM_DIR := stimuli
WORK_DIR := $(SIM_DIR)/work

# ── Default target ────────────────────────────────────────────
hw-all: stim hw-compile

# ── Fetch dependencies ────────────────────────────────────────
update-ips:
	$(BENDER) update

# ── Generate stimulus ─────────────────────────────────────────
stim:
	python3 $(STIM_DIR)/generate_stim.py --outdir $(STIM_DIR)

# ── Compile RTL + TBs ─────────────────────────────────────────
hw-compile: update-ips
	mkdir -p $(SIM_DIR)
	vlib $(WORK_DIR)
	vlog -work $(WORK_DIR) -sv $(shell $(BENDER) script flist -t tb)

# ── Run simulations ───────────────────────────────────────────
# Use GUI=1 to open the QuestaSim GUI instead of batch mode:

GUI ?= 0
ifeq ($(GUI),1)
VSIM_FLAGS =
VSIM_DO    = "run -all"
else
VSIM_FLAGS = -c
VSIM_DO    = "run -all; quit"
endif

sim-dual: stim hw-compile
	vsim $(VSIM_FLAGS) -l $(SIM_DIR)/transcript -lib $(WORK_DIR) tb_DIMC_dual -do $(VSIM_DO)

sim-single: stim hw-compile
	vsim $(VSIM_FLAGS) -l $(SIM_DIR)/transcript -lib $(WORK_DIR) tb_DIMC -do $(VSIM_DO)

# ── Remove all generated artefacts ────────────────────────────
hw-clean:
	rm -rf $(WORK_DIR)
	rm -f  $(SIM_DIR)/compile.tcl $(SIM_DIR)/transcript $(SIM_DIR)/*.vcd
	rm -f  transcript vsim.wlf *.vcd
	rm -f  $(STIM_DIR)/*.txt
	rm -f  etch*


