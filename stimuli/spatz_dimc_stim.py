#!/usr/bin/env python3
"""
spatz_dimc_stim.py — Stimulus and golden-output generator for tb_spatz_dimc.sv

============================================================
USAGE
============================================================
    python3 spatz_dimc_stim.py [--seed SEED] [--outdir DIR]

    --seed   Random seed for reproducibility (default: 42).
             The same seed produces the same kernel and feature every
             time, so golden outputs are stable across runs.
    --outdir Directory to write the output files (default: stimuli).

============================================================
CHANGING CONFIGURATION
============================================================
  1. Edit BIAS, SECTION_WIDTH, and COMPUTE_MASK_VALS below.
  2. Update the matching localparams in tb_spatz_dimc.sv.
  3. Re-run this script to regenerate the golden files.

============================================================
RUNNING THE FILE
============================================================
1. make stim
OR
2. python3 stimuli/spatz_dimc_stim.py --outdir stimuli

"""

import argparse
import os
import numpy as np

# User configuration
BIAS = 0
SECTION_WIDTH = 256

# Number of kernel rows.
NB_KERNEL_ROWS    = 32

# Number of sections per 1024-bit kernel row.
NUM_SECTIONS      = 1024 // SECTION_WIDTH

# Number of bytes in one section.
BYTES_PER_SECTION = SECTION_WIDTH // 8

# Total bytes in one full kernel row (128 bytes = 128 uint8 elements).
BYTES_PER_ROW     = NUM_SECTIONS * BYTES_PER_SECTION

# COMPUTE_MASK_VALS: the six threshold values swept in Test 4.
COMPUTE_MASK_VALS    = [0, 512, 768, 896, 960, 992]
NB_COMPUTE_MASK_VALS = len(COMPUTE_MASK_VALS)

# Signed 8-bit configurations: KS/FU, KU/FS, and KS/FS.
SIGNED_8B_MODES = [0b01, 0b10, 0b11]



# =========================================================================
# REFERENCE MODEL
# =========================================================================

# Computes the masked multiplication sum for one kernel row and feature vector.
def compute_mac(
    kernel_row: np.ndarray,
    feature: np.ndarray,
    compute_mask: int,
    sign_8b: int = 0,
) -> int:
    valid_bits = max(0, 1024 - int(compute_mask))

    acc = 0
    for i in range(BYTES_PER_ROW):
        # Element i occupies bits [i*8+7 : i*8] of the 1024-bit row vector.
        # Preserve any valid low bits in a partially masked boundary byte.
        bits_left = valid_bits - i * 8
        if bits_left > 0:
            byte_mask = (1 << min(8, bits_left)) - 1
            kernel_value = int(kernel_row[i]) & byte_mask
            feature_value = int(feature[i]) & byte_mask

            if sign_8b & 0b01 and kernel_value & 0x80:
                kernel_value -= 0x100
            if sign_8b & 0b10 and feature_value & 0x80:
                feature_value -= 0x100

            acc += kernel_value * feature_value

    return acc


# Adds the bias, applies ReLU, and clips the result to 8 bits.
def clip_with_bias(mac_val: int, bias: int) -> int:
    psum = (mac_val + bias) & 0xFFFFFFFF
    if psum & 0x80000000:
        return 0
    if psum > 0xFF:
        return 0xFF
    return psum


# =========================================================================
# FILE HELPERS
# =========================================================================

# Converts one byte section into $readmemh hexadecimal ordering.
def section_to_hex(section_bytes: np.ndarray) -> str:
    """Convert one section to a $readmemh-compatible hexadecimal string."""
    return "".join(f"{b:02x}" for b in reversed(section_bytes))


# Writes integer values as fixed-width hexadecimal lines.
def write_golden(path: str, values, width: int) -> None:
    mask = (1 << width) - 1
    chars = width // 4
    with open(path, "w") as f:
        for v in values:
            f.write(f"{v & mask:0{chars}x}\n")


# =========================================================================
# MAIN
# =========================================================================

# Parses options and generates all DIMC stimulus and golden files.
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
    spatz_dimc_stims_dir = os.path.join(args.outdir, "spatz_dimc_stims")
    os.makedirs(spatz_dimc_stims_dir, exist_ok=True)


    # GENERATE RANDOM STIMULUS DATA
    kernel = np.random.randint(
        0, 256, size=(NB_KERNEL_ROWS, BYTES_PER_ROW), dtype=np.uint8
    )
    feature = np.random.randint(0, 256, size=BYTES_PER_ROW, dtype=np.uint8)

    # =========================================================================
    # FILE 1: kernel_weights.txt
    # =========================================================================
    with open(os.path.join(spatz_dimc_stims_dir, "kernel_weights.txt"), "w") as f:
        for r in range(NB_KERNEL_ROWS):
            for s in range(NUM_SECTIONS):
                # Extract the 32-byte slice for this section of this row
                section = kernel[r, s * BYTES_PER_SECTION : (s + 1) * BYTES_PER_SECTION]
                f.write(section_to_hex(section) + "\n")

    # =========================================================================
    # FILE 2: feature_vector.txt for 1 full feature vector
    # =========================================================================
    with open(os.path.join(spatz_dimc_stims_dir, "feature_vector.txt"), "w") as f:
        for s in range(NUM_SECTIONS):
            section = feature[s * BYTES_PER_SECTION : (s + 1) * BYTES_PER_SECTION]
            f.write(section_to_hex(section) + "\n")

    # =========================================================================
    # PRE-COMPUTE MAC VALUES (compute mask=0, all 128 elements active)
    # =========================================================================
    mac_full  = [compute_mac(kernel[r], feature, compute_mask=0) for r in range(NB_KERNEL_ROWS)]

    psum_full = [(mac + BIAS) & 0xFFFFFFFF for mac in mac_full]

    # =========================================================================
    # FILE 3: golden_clipped_8bit.txt
    # =========================================================================
    golden_matvec = [clip_with_bias(mac, BIAS) for mac in mac_full]
    write_golden(os.path.join(spatz_dimc_stims_dir, "golden_clipped_8bit.txt"), golden_matvec, width=8)

    # =========================================================================
    # FILE 4: golden_psum_32bit.txt  (Test 3 expected 32-bit partial sums)
    # =========================================================================
    write_golden(os.path.join(spatz_dimc_stims_dir, "golden_psum_32bit.txt"), psum_full, width=32)

    # =========================================================================
    # PRE-COMPUTE MAC VALUES FOR compute mask SWEEP (row 0 only, 6 compute mask values)
    # =========================================================================
    mac_compute_mask  = [compute_mac(kernel[0], feature, compute_mask) for compute_mask in COMPUTE_MASK_VALS]

    # Pre-compute 32-bit psums for each compute-mask value (before clipping).
    psum_compute_mask = [(mac + BIAS) & 0xFFFFFFFF for mac in mac_compute_mask]

    # =========================================================================
    # FILE 5: golden_with_masking_8bit.txt  (Test 4 expected 8-bit outputs)
    # =========================================================================
    golden_compute_mask = [clip_with_bias(mac, BIAS) for mac in mac_compute_mask]
    write_golden(os.path.join(spatz_dimc_stims_dir, "golden_with_masking_8bit.txt"), golden_compute_mask, width=8)

    # =========================================================================
    # FILE 6: golden_psum_with_masking_32bit.txt  (Test 4 expected 32-bit partial sums)
    # =========================================================================
    write_golden(
        os.path.join(spatz_dimc_stims_dir, "golden_psum_with_masking_32bit.txt"),
        psum_compute_mask,
        width=32,
    )

    # =========================================================================
    # SIGNED 8-BIT GOLDEN FILES: sign_8b = 01, 10, and 11
    # =========================================================================
    for sign_mode in SIGNED_8B_MODES:
        signed_mac = [
            compute_mac(kernel[r], feature, compute_mask=0, sign_8b=sign_mode)
            for r in range(NB_KERNEL_ROWS)
        ]
        signed_psum = [(mac + BIAS) & 0xFFFFFFFF for mac in signed_mac]
        signed_clipped = [clip_with_bias(mac, BIAS) for mac in signed_mac]
        mode_name = f"{sign_mode:02b}"

        write_golden(
            os.path.join(spatz_dimc_stims_dir, f"golden_sign_{mode_name}_clipped_8bit.txt"),
            signed_clipped,
            width=8,
        )
        write_golden(
            os.path.join(spatz_dimc_stims_dir, f"golden_sign_{mode_name}_psum_32bit.txt"),
            signed_psum,
            width=32,
        )

    # =========================================================================
    # SUMMARY REPORT
    # =========================================================================
    out = args.outdir
    print(f"    BIAS = {BIAS}")
    print(f"    COMPUTE_MASK_VALS = {COMPUTE_MASK_VALS}")
    print(f"    Files written to: {os.path.abspath(out)}")

if __name__ == "__main__":
    main()
