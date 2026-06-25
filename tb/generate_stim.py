#!/usr/bin/env python3
"""
gen_stim.py — Stimulus and golden-output generator for tb_DIMC_18_fixed.sv

============================================================
PURPOSE
============================================================
This script generates all input stimuli (kernel weights, feature vector)
and all expected golden outputs that the SystemVerilog testbench
(tb_DIMC_18_fixed.sv) needs to verify the DIMC_18_fixed hardware.

It acts as a SOFTWARE REFERENCE MODEL of the DUT:
  - The DUT (spatz_DIMC.sv) implements matrix-vector multiplication
    in silicon: y[r] = ReLU4( dot(kernel[r], feature) + BIAS )
  - This script computes the exact same result in Python, producing
    "golden" files that the testbench compares against DUT outputs.
  - If the DUT and Python agree, the hardware is correct.

IMPORTANT: The constants in this script (BIAS, MCT_VALS) must EXACTLY
match the localparams of the same name in tb_DIMC_18_fixed.sv.
If you change one, you must change the other.

============================================================
MATRIX-VECTOR MULTIPLICATION OVERVIEW
============================================================
The DIMC_18_fixed hardware stores a 32×128 weight matrix (kernel)
in on-chip SRAM and multiplies it by a 128-element feature vector:

  kernel  : 32 rows × 128 uint8 elements = 32 × 1024 bits
  feature :  1 row  × 128 uint8 elements =  1 ×  1024 bits
  output  : 32 scalar values, each 4-bit after ReLU+quantization

The kernel is stored in four 256-bit "sections" per row
(4 × 256 bits = 1024 bits = 128 bytes = 128 uint8 elements).

============================================================
MCT MASKING
============================================================
MCT (Mask Count Threshold) lets the caller trim the number of
active byte-elements participating in each dot product:

  valid_bits = 1024 - MCT * 4
  element i (8-bit) is active when: i*8 < valid_bits

MCT=0   → valid_bits=1024 → all 128 elements active  (full row)
MCT=128 → valid_bits=512  → 64 elements active  (first two sections)
MCT=192 → valid_bits=256  → 32 elements active  (first section only)
MCT=224 → valid_bits=128  → 16 elements active
MCT=240 → valid_bits= 64  →  8 elements active
MCT=248 → valid_bits= 32  →  4 elements active

============================================================
OUTPUT FILES
============================================================
File 1  kernel_weights.txt          — kernel weights to load into SRAM (Tests 1, 3)
File 2  feature_vector.txt          — feature vector to load into feature buffer (Tests 2-4)
File 3  golden_matvec.txt           — expected 4-bit result per row, MCT=0 + BIAS (Test 3)
File 4  golden_psout.txt            — expected 24-bit psum per row before ReLU+quant, MCT=0 (Test 3)
File 5  golden_dot_product_mct.txt  — expected 4-bit result for row 0 at each MCT value + BIAS (Test 4)
File 6  golden_psout_mct.txt        — expected 24-bit psum for row 0 before ReLU+quant, one per MCT value (Test 4)

============================================================
FILE FORMAT — $readmemh compatible
============================================================
All files use hex notation so the SV testbench can load them with $readmemh.

  Stimulus sections (256-bit / 32 bytes):
    64 hex characters per line, MSB first.
    Byte 31 (bits [255:248]) is written leftmost,
    byte  0 (bits   [7:0])   is written rightmost.

  Golden 4-bit results:
    2 hex characters per line; result stored in the lower nibble.
    Upper nibble is always 0.  E.g., result=9 → "09".

  Golden 24-bit psums:
    6 hex characters per line.
    Two's-complement representation masked to 24 bits.
    E.g., psum = -1 → "ffffff".

============================================================
USAGE
============================================================
    python3 generate_stim.py [--seed SEED] [--outdir DIR]

    --seed   Random seed for reproducibility (default: 42).
             The same seed produces the same kernel and feature every
             time, so golden outputs are stable across runs.
    --outdir Directory to write all six output files (default: current dir).

============================================================
CHANGING BIAS OR MCT_VALS
============================================================
  1. Edit BIAS / MCT_VALS constants below.
  2. Update the matching localparams in tb_DIMC_18_fixed.sv.
  3. Re-run this script to regenerate the golden files.
  No simulator recompile is needed — files are loaded at runtime.
"""

import argparse
import os
import numpy as np

# =========================================================================
# Constants — must match the SV testbench localparams
# =========================================================================

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
    # In hardware: if (psum[23]) → negative → ReLU clamps to 0.
    if psum & 0x800000:
        return 0

    # Check saturation: if any bit above bit 3 is set, the value exceeds 15.
    # In hardware: if (|psum[23:4]) → value > 15 → saturate to 15.
    # Python equivalent: psum >> 4 is non-zero iff any bit above bit 3 is set.
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

    $readmemh assigns leftmost hex digits to the highest-numbered bits of the
    SystemVerilog array.  For a logic [255:0] signal, bits [255:248] are the
    "MSB" and must appear first (leftmost) in the file.

    Within our byte array, byte index 31 holds bits [255:248] and byte index 0
    holds bits [7:0].  So we iterate bytes in REVERSE order (31 down to 0)
    to produce the required MSB-first representation.

    Example:
      section_bytes = [0x01, 0x02, ..., 0xFF]  (byte 0 = 0x01, byte 31 = 0xFF)
      output = "ff...01"   (byte 31 first, byte 0 last)
    """
    return "".join(f"{b:02x}" for b in reversed(section_bytes))


def write_golden(path: str, values) -> None:
    """
    Write a list of 4-bit golden results to a file, one per line as 2 hex chars.
    """
    with open(path, "w") as f:
        for v in values:
            f.write(f"{v:02x}\n")


def write_golden_24(path: str, values) -> None:
    with open(path, "w") as f:
        for v in values:
            f.write(f"{v & 0xFFFFFF:06x}\n")


# =========================================================================
# MAIN
# =========================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Generate DIMC_18_fixed testbench stimulus files"
    )
    parser.add_argument("--seed",   type=int, default=42,
                        help="Random seed for reproducibility (default: 42)")
    parser.add_argument("--outdir", type=str, default=".",
                        help="Output directory for generated files (default: .)")
    args = parser.parse_args()

    np.random.seed(args.seed)
    os.makedirs(args.outdir, exist_ok=True)

    
    # GENERATE RANDOM STIMULUS DATA
    kernel = np.random.randint(0, 256, size=(NB_KERNEL_ROWS, BYTES_PER_ROW), dtype=np.uint8)

    feature = np.random.randint(0, 256, size=BYTES_PER_ROW, dtype=np.uint8)

    # =========================================================================
    # FILE 1: kernel_weights.txt  (used in Tests 1 and 3)
    # =========================================================================
    with open(os.path.join(args.outdir, "kernel_weights.txt"), "w") as f:
        for r in range(NB_KERNEL_ROWS):
            for s in range(NUM_SECTIONS):
                # Extract the 32-byte slice for this section of this row
                section = kernel[r, s * BYTES_PER_SECTION : (s + 1) * BYTES_PER_SECTION]
                f.write(section_to_hex(section) + "\n")

    # =========================================================================
    # FILE 2: feature_vector.txt  (used in Tests 2, 3, and 4)
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
    # FILE 3: golden_matvec.txt  (Test 3 expected 4-bit outputs)
    # =========================================================================
    golden_matvec = [relu_quant_with_bias(mac, BIAS) for mac in mac_full]
    write_golden(os.path.join(args.outdir, "golden_matvec.txt"), golden_matvec)

    # =========================================================================
    # FILE 4: golden_psout.txt  (Test 3 expected 24-bit partial sums)
    # =========================================================================
    write_golden_24(os.path.join(args.outdir, "golden_psout.txt"), psum_full)

    # =========================================================================
    # PRE-COMPUTE MAC VALUES FOR MCT SWEEP (row 0 only, 6 MCT values)
    # =========================================================================
    mac_mct  = [compute_mac(kernel[0], feature, mct) for mct in MCT_VALS]

    # Pre-compute 24-bit psums for each MCT value (before ReLU+quant).
    psum_mct = [(mac + BIAS) & 0xFFFFFF for mac in mac_mct]

    # =========================================================================
    # FILE 5: golden_dot_product_mct.txt  (Test 4 expected 4-bit outputs)
    # =========================================================================
    golden_mct = [relu_quant_with_bias(mac, BIAS) for mac in mac_mct]
    write_golden(os.path.join(args.outdir, "golden_dot_product_mct.txt"), golden_mct)

    # =========================================================================
    # FILE 6: golden_psout_mct.txt  (Test 4 expected 24-bit partial sums)
    # =========================================================================
    write_golden_24(os.path.join(args.outdir, "golden_psout_mct.txt"), psum_mct)

    # =========================================================================
    # SUMMARY REPORT
    # =========================================================================
    out = args.outdir
    print(f"Files written to: {os.path.abspath(out)}")
    print(f"  BIAS = {BIAS},  MCT_VALS = {MCT_VALS}")
    print()
    print(f"  File 1  kernel_weights.txt         — {NB_KERNEL_ROWS * NUM_SECTIONS} entries  (32 rows × 4 sections)")
    print(f"  File 2  feature_vector.txt         — {NUM_SECTIONS} entries  (4 sections of the feature vector)")
    print(f"  File 3  golden_matvec.txt           — {NB_KERNEL_ROWS} entries  (Test 3: 4-bit result per row, MCT=0 + BIAS)")
    print(f"  File 4  golden_psout.txt            — {NB_KERNEL_ROWS} entries  (Test 3: 24-bit psum per row before ReLU+quant)")
    print(f"  File 5  golden_dot_product_mct.txt — {NB_MCT_VALS} entries  (Test 4: 4-bit result for row 0, one per MCT value + BIAS)")
    print(f"  File 6  golden_psout_mct.txt        — {NB_MCT_VALS} entries  (Test 4: 24-bit psum for row 0 before ReLU+quant, one per MCT value)")
    print()
    print(f"  MAC range (MCT=0, no bias): [{min(mac_full)}, {max(mac_full)}]")
    print(f"  After bias={BIAS}: saturated to 15: {sum(v == 15 for v in golden_matvec)} / {NB_KERNEL_ROWS} rows")
    print(f"                     clamped to  0:  {sum(v == 0  for v in golden_matvec)} / {NB_KERNEL_ROWS} rows")
    print()
    print("  Active elements and row-0 result per MCT value:")
    for mct, result in zip(MCT_VALS, golden_mct):
        valid_bits = max(0, 1024 - mct * 4)
        active = sum(1 for i in range(BYTES_PER_ROW) if i * 8 < valid_bits)
        print(f"    MCT={mct:3d} → {active:3d} active elements → golden={result}")


if __name__ == "__main__":
    main()

# =============================================================================
# HOW TO USE THIS SCRIPT
# =============================================================================
# BASIC USAGE
# -----------
#   Run from the directory where the simulator will be launched.  The simulator
#   looks for stimulus files by filename only (no path), so the files must be
#   in the simulator's working directory.
#
#     cd hw/ip/spatz/tb
#     python3 gen_stim.py
#
#   All six files are written to the current directory by default.
#
# OPTIONS
# -------
#   --seed SEED    Integer seed for the NumPy RNG.  Default: 42.
#                  Changing the seed changes ALL random data (kernel and feature).
#                  The testbench and this Python model share the same stimulus
#                  because both read the same files, so golden values always
#                  match the stimulus regardless of the seed.
#
#   --outdir DIR   Directory to write output files into.  Default: . (current dir)
#
# CHANGING BIAS OR MCT_VALS
# -------------------------
#   1. Edit the BIAS and MCT_VALS constants at the top of this file.
#   2. Update the matching localparams in tb_DIMC_18_fixed.sv.
#   3. Re-run this script to regenerate all six files.
#   No simulator recompilation is needed — files are loaded at simulation runtime.
#
# VERIFYING THE REFERENCE MODEL
# ------------------------------
#   The summary printed at the end of each run shows:
#     - BIAS and MCT_VALS used (confirm they match the testbench)
#     - MAC value range before bias (confirm data is non-trivial, spread > 0)
#     - How many rows saturate (15) or ReLU-clamp (0) after bias
#       (ideally a mix of both to exercise the quantizer range)
#     - Active element count and row-0 result for each MCT value
#       (confirm masking is trimming the correct number of elements)
#   Cross-check these numbers against your expectations before simulating.
#
# OUTPUT FILE FORMATS (for reference)
# ------------------------------------
#   kernel_weights.txt         128 lines × 64 hex chars  — File 1
#   feature_vector.txt           4 lines × 64 hex chars  — File 2
#   golden_matvec.txt           32 lines × 2 hex chars   — File 3
#   golden_psout.txt            32 lines × 6 hex chars   — File 4
#   golden_dot_product_mct.txt   6 lines × 2 hex chars   — File 5
#   golden_psout_mct.txt         6 lines × 6 hex chars   — File 6
# =============================================================================
