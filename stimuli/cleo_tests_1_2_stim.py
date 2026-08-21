#!/usr/bin/env python3
"""Generate stimulus and golden files for tb_cleopatra Tests 1 and 2."""

import argparse
import os
import numpy as np

from spatz_dimc_stim import (
    BIAS,
    BYTES_PER_ROW,
    BYTES_PER_SECTION,
    NB_KERNEL_ROWS,
    NUM_SECTIONS,
    section_to_hex,
    write_golden,
)


# Cleopatra test configuration
NUM_FEATURE_VECTORS = 8
NUM_STIM_SETS = 8
TEST2_START = 0


def compute_dot_product(kernel_row: np.ndarray, feature_vec: np.ndarray) -> int:
    """Compute one unsigned kernel/feature dot product."""
    return int(np.dot(kernel_row.astype(np.int64), feature_vec.astype(np.int64)))


def write_kernel(path: str, kernel: np.ndarray) -> None:
    with open(path, "w") as file:
        for row in range(NB_KERNEL_ROWS):
            for section_index in range(NUM_SECTIONS):
                start = section_index * BYTES_PER_SECTION
                section = kernel[row, start : start + BYTES_PER_SECTION]
                file.write(section_to_hex(section) + "\n")


def write_features(path: str, features: np.ndarray) -> None:
    with open(path, "w") as file:
        for feature_index in range(NUM_FEATURE_VECTORS):
            for section_index in range(NUM_SECTIONS):
                start = section_index * BYTES_PER_SECTION
                section = features[feature_index, start : start + BYTES_PER_SECTION]
                file.write(section_to_hex(section) + "\n")


def biased_outputs(kernel: np.ndarray, features: np.ndarray):
    return [
        (compute_dot_product(kernel[row], features[feature_index]) + BIAS)
        & 0xFFFFFFFF
        for feature_index in range(NUM_FEATURE_VECTORS)
        for row in range(NB_KERNEL_ROWS)
    ]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate Cleopatra Test 1 and Test 2 stimulus files"
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--outdir", type=str, default="stimuli")
    args = parser.parse_args()

    np.random.seed(args.seed)
    test1_dir = os.path.join(args.outdir, "cleo_test1")
    test2_dir = os.path.join(args.outdir, "cleo_test2")
    os.makedirs(test1_dir, exist_ok=True)
    os.makedirs(test2_dir, exist_ok=True)

    kernel_sets = [
        np.random.randint(
            0, 256, size=(NB_KERNEL_ROWS, BYTES_PER_ROW), dtype=np.uint8
        )
        for _ in range(NUM_STIM_SETS)
    ]
    feature_sets = [
        np.random.randint(
            0,
            256,
            size=(NUM_FEATURE_VECTORS, BYTES_PER_ROW),
            dtype=np.uint8,
        )
        for _ in range(NUM_STIM_SETS)
    ]

    # Test 1 reuses kernel_weights.txt generated from kernel_sets[0].
    write_features(
        os.path.join(test1_dir, "feature_vector_8times.txt"), feature_sets[0]
    )
    write_golden(
        os.path.join(test1_dir, "golden_output_cleopatra.txt"),
        biased_outputs(kernel_sets[0], feature_sets[0]),
        width=32,
    )

    accumulated = [0] * (NB_KERNEL_ROWS * NUM_FEATURE_VECTORS)
    for set_index in range(TEST2_START, NUM_STIM_SETS):
        write_kernel(
            os.path.join(test2_dir, f"kernel_stim_{set_index}.txt"),
            kernel_sets[set_index],
        )
        write_features(
            os.path.join(test2_dir, f"feature_stim_8times_{set_index}.txt"),
            feature_sets[set_index],
        )
        current = biased_outputs(kernel_sets[set_index], feature_sets[set_index])
        for index, value in enumerate(current):
            accumulated[index] = (accumulated[index] + value) & 0xFFFFFFFF

    write_golden(
        os.path.join(test2_dir, "golden_output_cleopatra_test2.txt"),
        accumulated,
        width=32,
    )

    print(f"    Cleopatra Tests 1–2 files written to: {os.path.abspath(args.outdir)}")


if __name__ == "__main__":
    main()
