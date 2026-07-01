#!/usr/bin/env python3
"""
gen_stim.py — Stimulus and golden-output generator for tb_DIMC.sv and tb_DIMC_dual.sv

============================================================
USAGE
============================================================
    python3 generate_stim.py [--seed SEED] [--outdir DIR]

    --seed   Random seed for reproducibility (default: 42).
             The same seed produces the same kernel and feature every
             time, so golden outputs are stable across runs.
    --outdir Directory to write all six output files (default: stim).

============================================================
CHANGING BIAS OR MCT_VALS
============================================================
  1. Edit BIAS / MCT_VALS constants below.
  2. Update the matching localparams in tb_DIMC.sv.
  3. Re-run this script to regenerate the golden files.
  No simulator recompile is needed — files are loaded at runtime.
"""

import argparse
import os
import numpy as np


# Number of kernel rows.  The DUT has 32 SRAM rows.
NB_KERNEL_ROWS    = 32

# Number of 256-bit sections per kernel row (4 × 256 = 1024 bits = 128 bytes).
NUM_SECTIONS      = 4

# Number of bytes in one section (256 bits / 8 = 32 bytes).
BYTES_PER_SECTION = 32

# Total bytes in one full kernel row (128 bytes = 128 uint8 elements).
BYTES_PER_ROW     = NUM_SECTIONS * BYTES_PER_SECTION


BIAS = -2_080_000

# MCT_VALS: the six threshold values swept in Test 4.
MCT_VALS    = [0, 128, 192, 224, 240, 248]
NB_MCT_VALS = len(MCT_VALS)


# =========================================================================
# REFERENCE MODEL
# =========================================================================

def compute_mac(kernel_row: np.ndarray, feature: np.ndarray, mct: int) -> int:
    valid_bits = 1024 - int(mct) * 4
    if valid_bits > 1024:   # wrap guard (cannot actually happen for uint8 MCT)
        valid_bits = 0

    acc = 0
    for i in range(BYTES_PER_ROW):
        # Element i occupies bits [i*8+7 : i*8] of the 1024-bit row vector.
        # It is active when its LSB position (i*8) falls within the valid window.
        # Elements beyond valid_bits are treated as 0 (masked out).
        if i * 8 < valid_bits:
            acc += int(kernel_row[i]) * int(feature[i])

    return acc


def relu_quant_4bit(val: int) -> int:
    if val < 0:
        return 0    # ReLU: any negative value clamps to 0
    elif val > 15:
        return 15   # saturation: any value above 15 clamps to maximum
    else:
        return val  # already fits in 4 bits


def relu_quant_with_bias(mac_val: int, bias: int) -> int:
    psum = (mac_val + bias) & 0xFFFFFF

    # Check the sign bit (bit 23) in 24-bit two's complement.
    if psum & 0x800000:
        return 0

    # Check saturation: if any bit above bit 3 is set, the value exceeds 15.
    elif psum >> 4:
        return 15

    # Value fits in 4 bits: return the lower nibble unchanged.
    else:
        return psum & 0xF


# =========================================================================
# FILE HELPERS
# =========================================================================

def section_to_hex(section_bytes: np.ndarray) -> str:
    """
    Convert a 32-byte section to a 64-character $readmemh-compatible hex string.
    """
    return "".join(f"{b:02x}" for b in reversed(section_bytes))


def write_golden(path: str, values, width: int) -> None:
    mask = (1 << width) - 1
    chars = width // 4
    with open(path, "w") as f:
        for v in values:
            f.write(f"{v & mask:0{chars}x}\n")


# =========================================================================
# MAIN
# =========================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Generate DIMC_18_fixed testbench stimulus files"
    )
    parser.add_argument("--seed",   type=int, default=42,
                        help="Random seed for reproducibility (default: 42)")
    parser.add_argument("--outdir", type=str, default="stimuli",
                        help="Output directory for generated files (default: stimuli)")
    args = parser.parse_args()

    np.random.seed(args.seed)
    os.makedirs(args.outdir, exist_ok=True)

    
    # GENERATE RANDOM STIMULUS DATA
    kernel = np.random.randint(0, 256, size=(NB_KERNEL_ROWS, BYTES_PER_ROW), dtype=np.uint8)

    feature = np.random.randint(0, 256, size=BYTES_PER_ROW, dtype=np.uint8)

    # =========================================================================
    # FILE 1: kernel_weights.txt  
    # =========================================================================
    with open(os.path.join(args.outdir, "kernel_weights.txt"), "w") as f:
        for r in range(NB_KERNEL_ROWS):
            for s in range(NUM_SECTIONS):
                # Extract the 32-byte slice for this section of this row
                section = kernel[r, s * BYTES_PER_SECTION : (s + 1) * BYTES_PER_SECTION]
                f.write(section_to_hex(section) + "\n")

    # =========================================================================
    # FILE 2: feature_vector.txt  
    # =========================================================================
    with open(os.path.join(args.outdir, "feature_vector.txt"), "w") as f:
        for s in range(NUM_SECTIONS):
            section = feature[s * BYTES_PER_SECTION : (s + 1) * BYTES_PER_SECTION]
            f.write(section_to_hex(section) + "\n")

    # =========================================================================
    # PRE-COMPUTE MAC VALUES (MCT=0, all 128 elements active)
    # =========================================================================
    mac_full  = [compute_mac(kernel[r], feature, mct=0) for r in range(NB_KERNEL_ROWS)]

    # Pre-compute 24-bit psum = MAC + BIAS, masked to 24 bits.
    # This is what the DUT's PSOUT port outputs (before ReLU+quant).
    psum_full = [(mac + BIAS) & 0xFFFFFF for mac in mac_full]

    # =========================================================================
    # FILE 3: golden_matvec_4bit.txt
    # =========================================================================
    golden_matvec = [relu_quant_with_bias(mac, BIAS) for mac in mac_full]
    write_golden(os.path.join(args.outdir, "golden_4bit.txt"), golden_matvec, width=8)

    # =========================================================================
    # FILE 4: golden_psum_24bit.txt  (Test 3 expected 24-bit partial sums)
    # =========================================================================
    write_golden(os.path.join(args.outdir, "golden_psum_24bit.txt"), psum_full, width=24)

    # =========================================================================
    # PRE-COMPUTE MAC VALUES FOR MCT SWEEP (row 0 only, 6 MCT values)
    # =========================================================================
    mac_mct  = [compute_mac(kernel[0], feature, mct) for mct in MCT_VALS]

    # Pre-compute 24-bit psums for each MCT value (before ReLU+quant).
    psum_mct = [(mac + BIAS) & 0xFFFFFF for mac in mac_mct]

    # =========================================================================
    # FILE 5: golden_mct_4bit.txt  (Test 4 expected 4-bit outputs)
    # =========================================================================
    golden_mct = [relu_quant_with_bias(mac, BIAS) for mac in mac_mct]
    write_golden(os.path.join(args.outdir, "golden_mct_4bit.txt"), golden_mct, width=8)

    # =========================================================================
    # FILE 6: golden_psum_mct_24bit.txt  (Test 4 expected 24-bit partial sums)
    # =========================================================================
    write_golden(os.path.join(args.outdir, "golden_psum_mct_24bit.txt"), psum_mct, width=24)

    # =========================================================================
    # SUMMARY REPORT
    # =========================================================================
    out = args.outdir
    print(f"    BIAS = {BIAS},  MCT_VALS = {MCT_VALS}")
    print(f"    Files written to: {os.path.abspath(out)}")

if __name__ == "__main__":
    main()
