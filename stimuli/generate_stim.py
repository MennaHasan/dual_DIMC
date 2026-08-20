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
CHANGING BIAS OR COMPUTE_MASK_VALS
============================================================
  1. Edit BIAS / COMPUTE_MASK_VALS constants below.
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

# COMPUTE_MASK_VALS: the six threshold values swept in Test 4.
COMPUTE_MASK_VALS    = [0, 512, 768, 896, 960, 992]
NB_COMPUTE_MASK_VALS = len(COMPUTE_MASK_VALS)

# Test 2 MACROS
TEST2_START = 0
NUM_STIM_SETS = 8  # has to be the same as value in tb_cleopatra.sv



# =========================================================================
# REFERENCE MODEL
# =========================================================================

def compute_mac(kernel_row: np.ndarray, feature: np.ndarray, compute_mask: int) -> int:
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
            acc += kernel_value * feature_value

    return acc


def clip_with_bias(mac_val: int, bias: int) -> int:
    psum = (mac_val + bias) & 0xFFFFFFFF
    if psum & 0x80000000:
        return 0
    if psum > 0xFF:
        return 0xFF
    return psum


def compute_dot_product(kernel_row: np.ndarray, feature_vec: np.ndarray) -> int:
    """Compute the scalar product of one kernel row and one feature vector."""
    return int(np.dot(kernel_row.astype(np.int64), feature_vec.astype(np.int64)))


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
    dimc_tests_dir = os.path.join(args.outdir, "dimc_tests")
    cleo_test1_dir = os.path.join(args.outdir, "cleo_test1")
    cleo_test2_dir = os.path.join(args.outdir, "cleo_test2")
    os.makedirs(dimc_tests_dir, exist_ok=True)
    os.makedirs(cleo_test1_dir, exist_ok=True)
    os.makedirs(cleo_test2_dir, exist_ok=True)


    # GENERATE RANDOM STIMULUS DATA
    kernel_sets = [
        np.random.randint(0, 256, size=(NB_KERNEL_ROWS, BYTES_PER_ROW), dtype=np.uint8)
        for _ in range(NUM_STIM_SETS)
    ]

    feature_sets_8 = [
        np.random.randint(0, 256, size=(8, BYTES_PER_ROW), dtype=np.uint8)
        for _ in range(NUM_STIM_SETS)
    ]

    # Keep the first set as the default root stimulus files used by existing tests.
    kernel = kernel_sets[0]
    feature = np.random.randint(0, 256, size=BYTES_PER_ROW, dtype=np.uint8)

    # =========================================================================
    # FILE 1: kernel_weights.txt
    # =========================================================================
    with open(os.path.join(dimc_tests_dir, "kernel_weights.txt"), "w") as f:
        for r in range(NB_KERNEL_ROWS):
            for s in range(NUM_SECTIONS):
                # Extract the 32-byte slice for this section of this row
                section = kernel[r, s * BYTES_PER_SECTION : (s + 1) * BYTES_PER_SECTION]
                f.write(section_to_hex(section) + "\n")

    # =========================================================================
    # FILE 2: feature_vector.txt for 1 full feature vector
    # =========================================================================
    with open(os.path.join(dimc_tests_dir, "feature_vector.txt"), "w") as f:
        for s in range(NUM_SECTIONS):
            section = feature[s * BYTES_PER_SECTION : (s + 1) * BYTES_PER_SECTION]
            f.write(section_to_hex(section) + "\n")


    # =========================================================================
    # FILE 3: feature_vector_8times.txt for 8 different feature vectors
    # =========================================================================
    features_8 = feature_sets_8[0]
    with open(os.path.join(cleo_test1_dir, "feature_vector_8times.txt"), "w") as f:
        for p in range(8):
            for s in range(NUM_SECTIONS):
                section = features_8[p, s * BYTES_PER_SECTION : (s + 1) * BYTES_PER_SECTION]
                f.write(section_to_hex(section) + "\n")

    # =========================================================================
    # PRE-COMPUTE MAC VALUES (compute mask=0, all 128 elements active)
    # =========================================================================
    mac_full  = [compute_mac(kernel[r], feature, compute_mask=0) for r in range(NB_KERNEL_ROWS)]

    psum_full = [(mac + BIAS) & 0xFFFFFFFF for mac in mac_full]

    # =========================================================================
    # FILE 4: golden_matvec_4bit.txt
    # =========================================================================
    golden_matvec = [clip_with_bias(mac, BIAS) for mac in mac_full]
    write_golden(os.path.join(dimc_tests_dir, "golden_clipped_8bit.txt"), golden_matvec, width=8)

    # =========================================================================
    # FILE 5: golden_psum_32bit.txt  (Test 3 expected 32-bit partial sums)
    # =========================================================================
    write_golden(os.path.join(dimc_tests_dir, "golden_psum_32bit.txt"), psum_full, width=32)

    # =========================================================================
    # FILE 6: golden_output_cleopatra.txt
    # =========================================================================
    # Flatten the 32x8 matrix of row/feature-vector biased partial sums into a single
    # 256-entry vector in output order:
    #   index = 32*j + i
    # where j is the feature-vector index and i is the kernel-row index.
    cleopatra_golden = [
        # index for golden file = 32*j + i (feature-major output order).
        (compute_dot_product(kernel[i], features_8[j]) + BIAS) & 0xFFFFFFFF
        for j in range(8)
        for i in range(NB_KERNEL_ROWS)
    ]
    write_golden(
        os.path.join(cleo_test1_dir, "golden_output_cleopatra.txt"),
        cleopatra_golden,
        width=32,
    )

    # =========================================================================
    # FILE 7: golden_output_cleopatra_test2.txt
    # =========================================================================
    # Sum the 256 outputs from the selected range of numbered K stimulus sets,
    # matching repeated accumulation without clearing in tb_cleopatra Test 2.

    cleopatra_golden_test2_cleo = [0] * (NB_KERNEL_ROWS * 8)
    for kk in range(TEST2_START, NUM_STIM_SETS):
        # generate the requested kernel and feature files
        kernel_file = os.path.join(cleo_test2_dir, f"kernel_stim_{kk}.txt")
        with open(kernel_file, "w") as f:
            for r in range(NB_KERNEL_ROWS):
                for s in range(NUM_SECTIONS):
                    section = kernel_sets[kk][r, s * BYTES_PER_SECTION : (s + 1) * BYTES_PER_SECTION]
                    f.write(section_to_hex(section) + "\n")

        feature_file = os.path.join(
            cleo_test2_dir,
            f"feature_stim_8times_{kk}.txt",
        )
        with open(feature_file, "w") as f:
            for p in range(8):
                for s in range(NUM_SECTIONS):
                    section = feature_sets_8[kk][p, s * BYTES_PER_SECTION : (s + 1) * BYTES_PER_SECTION]
                    f.write(section_to_hex(section) + "\n")

        # get the dot procuct of kk-th kernel and feature request
        new_val = [
            # index for golden file = 32*j + i (feature-major output order).
            (
                compute_dot_product(
                    kernel_sets[kk][i],
                    feature_sets_8[kk][j],
                )
                + BIAS
            )
            & 0xFFFFFFFF
            for j in range(8)
            for i in range(NB_KERNEL_ROWS)
        ]
        for index in range(NB_KERNEL_ROWS * 8):
            cleopatra_golden_test2_cleo[index] = (cleopatra_golden_test2_cleo[index] + new_val[index]) & 0xFFFFFFFF

    # write the golden output for test2 to a file
    write_golden(
        os.path.join(cleo_test2_dir, "golden_output_cleopatra_test2.txt"),
        cleopatra_golden_test2_cleo,
        width=32,
    )


    # =========================================================================
    # PRE-COMPUTE MAC VALUES FOR compute mask SWEEP (row 0 only, 6 compute mask values)
    # =========================================================================
    mac_compute_mask  = [compute_mac(kernel[0], feature, compute_mask) for compute_mask in COMPUTE_MASK_VALS]

    # Pre-compute 32-bit psums for each compute-mask value (before clipping).
    psum_compute_mask = [(mac + BIAS) & 0xFFFFFFFF for mac in mac_compute_mask]

    # =========================================================================
    # FILE 7: golden_compute_mask_8bit.txt  (Test 4 expected 8-bit outputs)
    # =========================================================================
    golden_compute_mask = [clip_with_bias(mac, BIAS) for mac in mac_compute_mask]
    write_golden(os.path.join(dimc_tests_dir, "golden_compute_mask_8bit.txt"), golden_compute_mask, width=8)

    # =========================================================================
    # FILE 8: golden_psum_compute_mask_32bit.txt  (Test 4 expected 32-bit partial sums)
    # =========================================================================
    write_golden(
        os.path.join(dimc_tests_dir, "golden_psum_compute_mask_32bit.txt"),
        psum_compute_mask,
        width=32,
    )

    # =========================================================================
    # SUMMARY REPORT
    # =========================================================================
    out = args.outdir
    print(f"    BIAS = {BIAS},  COMPUTE_MASK_VALS = {COMPUTE_MASK_VALS}")
    print(f"    Files written to: {os.path.abspath(out)}")

if __name__ == "__main__":
    main()
