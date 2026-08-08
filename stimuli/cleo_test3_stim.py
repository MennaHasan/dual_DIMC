#!/usr/bin/env python3
"""Generate tiled stimuli and golden output for Cleopatra Test 3."""

import random
from pathlib import Path
from typing import Dict, List, Tuple

############### SET ###############
# ints that define Full matrix dimensions
# should match the values in the tb_cleoatra
K = 3
L = 2
Q = 4


# matrices dimensions in DIMC 
# For weight_matrix * input_matrix:
M = 32
N = 1024          # DIMC row width in bits
p = 8
ELEMENT_BITS = 8
N_ELEMENTS = N // ELEMENT_BITS
UINT8_MAX = (1 << ELEMENT_BITS) - 1
UINT24_MAX = (1 << 24) - 1
UINT32_MAX = (1 << 32) - 1
BIAS = 0xE04300



# Full matrix dimensions in 8-bit elements. Each weight row and input column
# contains L*N bits, represented as L*N_ELEMENTS byte values.
weight_matrix_rows = K * M
weight_matrix_cols = L * N_ELEMENTS

input_matrix_rows = L * N_ELEMENTS
input_matrix_cols = Q * p


Matrix = List[List[int]]
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "cleo_test3"
DEFAULT_OUTPUT_FILE = (
    DEFAULT_OUTPUT_DIR / "test3_golden_matmul_output.txt"
)
DEFAULT_TILED_WEIGHTS_FILE = (
    DEFAULT_OUTPUT_DIR / "test3_tiled_weights.txt"
)
DEFAULT_TILED_INPUTS_FILE = (
    DEFAULT_OUTPUT_DIR / "test3_tiled_inputs.txt"
)


def validate_unsigned_8_matrix(matrix: Matrix, name: str) -> None:
    """Check that every matrix element is an unsigned 8-bit integer."""
    for row_index, row in enumerate(matrix):
        for col_index, value in enumerate(row):
            if not isinstance(value, int) or not 0 <= value <= UINT8_MAX:
                raise ValueError(
                    f"{name}[{row_index}][{col_index}] must be an unsigned "
                    f"8-bit integer; got {value!r}"
                )


def generate_random_matrix(rows: int, cols: int) -> Matrix:
    """Return a rows-by-cols matrix filled with unsigned 8-bit values."""
    return [
        [random.getrandbits(ELEMENT_BITS) for _ in range(cols)]
        for _ in range(rows)
    ]


def matrix_tiling(
    weight_matrix: Matrix,
    input_matrix: Matrix,
) -> Tuple[Dict[Tuple[int, int], Matrix], Dict[Tuple[int, int], Matrix]]:
    """
    Divide weights into M-by-N_ELEMENTS tiles and inputs into
    N_ELEMENTS-by-p tiles.

    Returned dictionaries are indexed as weights_tiled_matrices[i, j] and
    inputs_tiled_matrices[i, j].
    """
    if not weight_matrix or not input_matrix:
        raise ValueError("Input and weight matrices must not be empty")

    full_weight_matrix_rows = len(weight_matrix)
    full_weight_matrix_cols = len(weight_matrix[0])
    full_input_matrix_rows = len(input_matrix)
    full_input_matrix_cols = len(input_matrix[0])

    if any(len(row) != full_weight_matrix_cols for row in weight_matrix):
        raise ValueError("All weight-matrix rows must have the same length")
    if any(len(row) != full_input_matrix_cols for row in input_matrix):
        raise ValueError("All input-matrix rows must have the same length")

    validate_unsigned_8_matrix(weight_matrix, "weight_matrix")
    validate_unsigned_8_matrix(input_matrix, "input_matrix")

    if full_weight_matrix_cols != full_input_matrix_rows:
        raise ValueError(
            "weight_matrix_cols must equal input_matrix_rows; "
            f"got {full_weight_matrix_cols} and {full_input_matrix_rows}"
        )
    if (
        full_weight_matrix_rows % M != 0
        or full_weight_matrix_cols % N_ELEMENTS != 0
        or full_input_matrix_cols % p != 0
    ):
        raise ValueError(
            "input or weight matrix are not an int multiple of DIMC matrices "
            "dimensions"
        )

    k = full_weight_matrix_rows // M
    l = full_weight_matrix_cols // N_ELEMENTS
    q = full_input_matrix_cols // p

    weights_tiled_matrices = {}
    for i in range(k):
        for j in range(l):
            row_start = i * M
            col_start = j * N_ELEMENTS
            weights_tiled_matrices[i, j] = [
                row[col_start:col_start + N_ELEMENTS]
                for row in weight_matrix[row_start:row_start + M]
            ]

    inputs_tiled_matrices = {}
    for i in range(l):
        for j in range(q):
            row_start = i * N_ELEMENTS
            col_start = j * p
            inputs_tiled_matrices[i, j] = [
                row[col_start:col_start + p]
                for row in input_matrix[row_start:row_start + N_ELEMENTS]
            ]

    return weights_tiled_matrices, inputs_tiled_matrices


def save_tiled_matrices(
    weights_tiled_matrices: Dict[Tuple[int, int], Matrix],
    inputs_tiled_matrices: Dict[Tuple[int, int], Matrix],
    weights_file: str = str(DEFAULT_TILED_WEIGHTS_FILE),
    inputs_file: str = str(DEFAULT_TILED_INPUTS_FILE),
) -> None:
    """
    Save weight and input tiles as one hexadecimal token per tile.

    Weight tiles are ordered row-major by tile coordinate:
    weights_tiled_matrices[0,0], weights_tiled_matrices[0,1], ...,
    weights_tiled_matrices[k-1,l-1].

    Input tiles are ordered column-major by tile coordinate:
    inputs_tiled_matrices[0,0], inputs_tiled_matrices[1,0], ...,
    inputs_tiled_matrices[l-1,0], inputs_tiled_matrices[0,1], ...

    Weight tiles are packed row-major as M*N_ELEMENTS 8-bit values.

    Input tiles are packed column-major so each complete N-bit feature
    vector is contiguous. Column 0 is first (most significant), followed by
    columns 1 through p-1. This lets the testbench select one feature vector
    using a single packed range.
    """
    if str(weights_file) == str(inputs_file):
        raise ValueError("Weights and inputs must be saved to different files")

    with open(weights_file, "w", encoding="utf-8") as output:
        for tile_index in sorted(weights_tiled_matrices):
            output.write(
                "".join(
                    f"{value:02x}"
                    for row in weights_tiled_matrices[tile_index]
                    for value in row
                ) + "\n"
            )

    with open(inputs_file, "w", encoding="utf-8") as output:
        for tile_index in sorted(
            inputs_tiled_matrices,
            key=lambda index: (index[1], index[0]),
        ):
            tile = inputs_tiled_matrices[tile_index]
            output.write(
                "".join(
                    f"{tile[row_index][col_index]:02x}"
                    for col_index in range(len(tile[0]))
                    for row_index in range(len(tile))
                ) + "\n"
            )


def calculate_matmul(
    weight_matrix: Matrix,
    input_matrix: Matrix,
    output_file: str = str(DEFAULT_OUTPUT_FILE),
    bias: int = BIAS,
) -> Matrix:
    """Calculate weight_matrix * input_matrix with bias and save the result."""
    if not input_matrix or not weight_matrix:
        raise ValueError("Input and weight matrices must not be empty")

    weight_cols = len(weight_matrix[0])
    input_cols = len(input_matrix[0])

    if any(len(row) != input_cols for row in input_matrix):
        raise ValueError("All input-matrix rows must have the same length")
    if any(len(row) != weight_cols for row in weight_matrix):
        raise ValueError("All weight-matrix rows must have the same length")

    validate_unsigned_8_matrix(weight_matrix, "weight_matrix")
    validate_unsigned_8_matrix(input_matrix, "input_matrix")

    if weight_cols != len(input_matrix):
        raise ValueError(
            "Matrix multiplication requires the number of weight-matrix "
            "columns to equal the number of input-matrix rows"
        )
    if weight_cols % N_ELEMENTS != 0:
        raise ValueError(
            "The shared matrix dimension must be divisible by N_ELEMENTS"
        )

    num_l_tiles = weight_cols // N_ELEMENTS

    output_matrix = [
        [
            sum(
                (
                    sum(
                        weight_matrix[row][index]
                        * input_matrix[index][col]
                        for index in range(
                            l_index * N_ELEMENTS,
                            (l_index + 1) * N_ELEMENTS,
                        )
                    )
                    + bias
                ) & UINT24_MAX
                for l_index in range(num_l_tiles)
            ) & UINT32_MAX
            for col in range(input_cols)
        ]
        for row in range(len(weight_matrix))
    ]

    # Output order is row-major, with one 32-bit hexadecimal token per element.
    with open(output_file, "w", encoding="utf-8") as output:
        for row in output_matrix:
            output.write(" ".join(f"{value:08x}" for value in row) + "\n")

    return output_matrix


def main() -> None:
    DEFAULT_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    if weight_matrix_cols != input_matrix_rows:
        raise ValueError(
            "weight_matrix_cols must equal input_matrix_rows; "
            f"got {weight_matrix_cols} and {input_matrix_rows}"
        )

    full_input_matrix = generate_random_matrix(
        input_matrix_rows,
        input_matrix_cols,
    )
    full_weight_matrix = generate_random_matrix(
        weight_matrix_rows,
        weight_matrix_cols,
    )
    weights_tiled_matrices, inputs_tiled_matrices = matrix_tiling(
        full_weight_matrix,
        full_input_matrix,
    )
    save_tiled_matrices(weights_tiled_matrices, inputs_tiled_matrices)
    calculate_matmul(full_weight_matrix, full_input_matrix)


if __name__ == "__main__":
    main()
