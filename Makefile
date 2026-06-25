# Makefile for dual_DIMC


# Author:
# Mennatalla Hassan, University of Bologna

BENDER := /srv/home/mennatalla.hassan/spatz_DIMC-project/install/bender/bender


# ── Directories ───────────────────────────────────────────────
SRC_DIR  := src
TB_DIR   := tb

# ── Sources ───────────────────────────────────────────────────
RTL_SRCS := $(SRC_DIR)/spatz_DIMC.sv \
            $(SRC_DIR)/spatz_DIMC_dual.sv

TB_SRCS := $(TB_DIR)/tb_DIMC_18_fixed.sv \
           $(TB_DIR)/tb_dimc_dual.sv



# ── Default target ────────────────────────────────────────────
.PHONY: all
all: stim compile

# ── Fetch dependencies ────────────────────────────────────────
.PHONY: bender-update
bender-update:
	$(BENDER) update

# ── Compile RTL + both TBs ────────────────────────────────────
.PHONY: compile
compile: bender-update
	vlog -sv $(shell $(BENDER) script flist -t tb)

# ── Generate stimulus ─────────────────────────────────────────
.PHONY: stim
stim:
	python3 $(TB_DIR)/generate_stim.py --outdir $(TB_DIR)


