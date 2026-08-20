#!/usr/bin/env python3
"""Generate independent tiled stimuli for the double-buffering testbench."""

import argparse
import random
from pathlib import Path
from typing import Dict, List, Tuple

# Full-matrix tile counts. Keep these aligned with tb_double_buffering.sv.
K = 4
L = 3
Q = 2

# DIMC tile dimensions. N is represented here in unsigned 8-bit elements.
M = 32
N_BITS = 1024
P = 8
ELEMENT_BITS = 8
N_ELEMENTS = N_BITS // ELEMENT_BITS
WEIGHT_MATRIX_ROWS = K * M
WEIGHT_MATRIX_COLS = L * N_ELEMENTS
INPUT_MATRIX_ROWS = L * N_ELEMENTS
INPUT_MATRIX_COLS = Q * P
BIAS = -2_080_000
UINT32_MASK = (1 << 32) - 1

Matrix = List[List[int]]
WeightTiles = Dict[Tuple[int, int], Matrix]
InputTiles = Dict[Tuple[int, int], Matrix]

DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "double_buffering"
ACCUMULATOR_OUTPUT_FILE = DEFAULT_OUTPUT_DIR / "double_buffering_accumulator_output.txt"
FINAL_OUTPUT_FILE = DEFAULT_OUTPUT_DIR / "double_buffering_final_matmul_output.txt"


def generate_random_matrix(rng: random.Random, rows: int, cols: int) -> Matrix:
    """Return a reproducible unsigned 8-bit matrix."""
    return [[rng.getrandbits(ELEMENT_BITS) for _ in range(cols)] for _ in range(rows)]


def matrix_tiling(
    weight_matrix: Matrix,
    input_matrix: Matrix,
) -> Tuple[WeightTiles, InputTiles]:
    """Split the full matrices into DIMC-sized MxN and NxP tiles."""
    weight_tiles = {}
    for k_index in range(K):
        for l_index in range(L):
            row_start = k_index * M
            col_start = l_index * N_ELEMENTS
            weight_tiles[k_index, l_index] = [
                row[col_start:col_start + N_ELEMENTS]
                for row in weight_matrix[row_start:row_start + M]
            ]

    input_tiles = {}
    for l_index in range(L):
        for q_index in range(Q):
            row_start = l_index * N_ELEMENTS
            col_start = q_index * P
            input_tiles[l_index, q_index] = [
                row[col_start:col_start + P]
                for row in input_matrix[row_start:row_start + N_ELEMENTS]
            ]

    return weight_tiles, input_tiles


def write_tiled_weights(path: Path, tiles: WeightTiles) -> None:
    """Write one packed MxN kernel tile per line in [k,l] order."""
    with path.open("w", encoding="utf-8") as output:
        for tile_index in sorted(tiles):
            output.write(
                "".join(f"{value:02x}" for row in tiles[tile_index] for value in row)
                + "\n"
            )


def write_tiled_inputs(path: Path, tiles: InputTiles) -> None:
    """Write one packed NxP input tile per line in [q,l] order."""
    with path.open("w", encoding="utf-8") as output:
        for tile_index in sorted(tiles, key=lambda index: (index[1], index[0])):
            tile = tiles[tile_index]
            output.write(
                "".join(
                    f"{tile[row_index][col_index]:02x}"
                    for col_index in range(P)
                    for row_index in range(N_ELEMENTS)
                )
                + "\n"
            )


def calculate_golden_matmul(weight_matrix: Matrix, input_matrix: Matrix) -> Matrix:
    """Model tiled accumulation, including one 32-bit biased sum per L tile."""
    return [
        [
            sum(
                (
                    sum(
                        weight_matrix[row][index] * input_matrix[index][col]
                        for index in range(
                            l_index * N_ELEMENTS,
                            (l_index + 1) * N_ELEMENTS,
                        )
                    )
                    + BIAS
                )
                & UINT32_MASK
                for l_index in range(L)
            )
            & UINT32_MASK
            for col in range(Q * P)
        ]
        for row in range(K * M)
    ]


def write_matrix(path: Path, matrix: Matrix) -> None:
    with path.open("w", encoding="utf-8") as output:
        for row in matrix:
            output.write(" ".join(f"{value:08x}" for value in row) + "\n")


def write_u8_matrix(path: Path, matrix: Matrix) -> None:
    """Write an unsigned 8-bit matrix as one row per line."""
    with path.open("w", encoding="utf-8") as output:
        for row in matrix:
            output.write(" ".join(f"{value:02x}" for value in row) + "\n")


def reorganize_accumulator_output(
    accumulator_file: Path = ACCUMULATOR_OUTPUT_FILE,
    output_file: Path = FINAL_OUTPUT_FILE,
) -> None:
    """Reassemble saved M-by-P accumulator tiles into the full output matrix."""
    raw_values = [
        int(token, 16)
        for token in accumulator_file.read_text(encoding="utf-8").split()
    ]
    tile_elements = M * P
    expected_values = K * Q * tile_elements
    if len(raw_values) != expected_values:
        raise ValueError(
            f"Expected {expected_values} accumulator values, "
            f"but found {len(raw_values)}"
        )

    with output_file.open("w", encoding="utf-8") as output:
        for k_index in range(K):
            for row in range(M):
                final_row = []
                for q_index in range(Q):
                    tile_start = (k_index * Q + q_index) * tile_elements
                    for col in range(P):
                        final_row.append(raw_values[tile_start + row * P + col])
                output.write(" ".join(f"{value:08x}" for value in final_row) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=43)
    parser.add_argument("--outdir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--untile-only",
        action="store_true",
        help="Only convert the saved accumulator tiles into the final matrix",
    )
    args = parser.parse_args()

    if args.untile_only:
        reorganize_accumulator_output(
            args.outdir / ACCUMULATOR_OUTPUT_FILE.name,
            args.outdir / FINAL_OUTPUT_FILE.name,
        )
        return

    args.outdir.mkdir(parents=True, exist_ok=True)
    rng = random.Random(args.seed)

    # Full matrix shapes:
    #   weights = (K*M) x (L*N_ELEMENTS)
    #   inputs  = (L*N_ELEMENTS) x (Q*P)
    # Full, untiled matrices of unsigned 8-bit elements.
    kernel_stim = generate_random_matrix(
        rng,
        WEIGHT_MATRIX_ROWS,
        WEIGHT_MATRIX_COLS,
    )
    feature_stim = generate_random_matrix(
        rng,
        INPUT_MATRIX_ROWS,
        INPUT_MATRIX_COLS,
    )

    # software tiling done here 
    weight_tiles, input_tiles = matrix_tiling(kernel_stim, feature_stim)

    write_u8_matrix(
        args.outdir / "double_buffering_kernel_stim.txt",
        kernel_stim,
    )
    write_u8_matrix(
        args.outdir / "double_buffering_feature_stim.txt",
        feature_stim,
    )

    write_tiled_weights(
        args.outdir / "double_buffering_tiled_weights.txt",
        weight_tiles,
    )
    write_tiled_inputs(
        args.outdir / "double_buffering_tiled_inputs.txt",
        input_tiles,
    )
    write_matrix(
        args.outdir / "double_buffering_golden_matmul_output.txt",
        calculate_golden_matmul(kernel_stim, feature_stim),
    )

    print(f"Double-buffering stimulus written to {args.outdir.resolve()}")


if __name__ == "__main__":
    main()
