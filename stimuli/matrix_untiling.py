#!/usr/bin/env python3
"""Untile Cleopatra Test 3 accumulator output into global row-major order."""

from pathlib import Path

from cleo_test3_stim import K, M, Q, p

OUTPUT_DIR = Path(__file__).resolve().parent / "cleo_test3"
INPUT_FILE = OUTPUT_DIR / "test3_accumulator_output.txt"
OUTPUT_FILE = OUTPUT_DIR / "test3_final_matmul_output.txt"


def reorganize_accumulator_output(
    accumulator_file: str = str(INPUT_FILE),
    output_file: str = str(OUTPUT_FILE),
) -> None:
    """Reassemble row-major M-by-p output tiles into one full matrix."""
    raw_values = [
        int(token, 16)
        for token in Path(accumulator_file).read_text(encoding="utf-8").split()
    ]

    tile_elements = M * p
    expected_values = K * Q * tile_elements
    if len(raw_values) != expected_values:
        raise ValueError(
            f"Expected {expected_values} accumulator values, "
            f"but found {len(raw_values)}"
        )

    # Tiles are stored as output[0][0], ..., output[0][Q-1],
    # output[1][0], ..., output[K-1][Q-1]. Each tile is M-by-p and row-major.
    # Join all tile rows with the same k_idx and local row to form one row of
    # the full (k*M)-by-(q*p) output matrix.
    final_matrix = []
    for k_idx in range(K):
        for row in range(M):
            final_row = []
            for q_idx in range(Q):
                tile_start = (k_idx * Q + q_idx) * tile_elements
                for col in range(p):
                    raw_index = tile_start + row * p + col
                    final_row.append(raw_values[raw_index])
            final_matrix.append(final_row)

    with Path(output_file).open("w", encoding="utf-8") as output:
        for row in final_matrix:
            output.write(" ".join(f"{value:08x}" for value in row) + "\n")


if __name__ == "__main__":
    reorganize_accumulator_output()
