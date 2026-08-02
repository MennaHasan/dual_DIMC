/*
 * cleopatra.sv
 *
 * Wires a spatz_DIMC_dual and an accumulator together: the dual DIMC's
 * output stream (out_data) feeds directly into the accumulator's input.
*/

module cleopatra #(
    parameter int SECTION_WIDTH  = 256,

    parameter int INP_FIFO_DEPTH = 8,
    parameter int WGT_FIFO_DEPTH = 128,
    parameter int OUT_FIFO_DEPTH = 64,

    parameter int DATA_WIDTH = 24,
    parameter int OUT_WIDTH  = 32
)(
    input  logic clk,
    input  logic rst_n,

    // dimc_dual selector
    input  logic sel,

    // dimc_dual control inputs
    input  logic                     COMPE,
    input  logic                     FCSN,
    input  logic [1:0]               MODE,
    input  logic [1:0]               FA,
    input  logic [23:0]              ADDIN,
    input  logic [6:0]               RA,
    input  logic [6:0]               WA,
    input  logic                     RCSN,
    input  logic                     RCSN0, RCSN1, RCSN2, RCSN3,
    input  logic                     WCSN,
    input  logic                     WEN,
    input  logic [SECTION_WIDTH-1:0] M,
    input  logic [7:0]               MCT,

    // dimc_dual input FIFO
    input  logic                     inp_push,
    input  logic [SECTION_WIDTH-1:0] inp_data,

    // dimc_dual weight FIFO
    input  logic                     wgt_push,
    input  logic [SECTION_WIDTH-1:0] wgt_data,

    // accumulator control
    input  logic                        acc_enable_i,
    input  logic                        acc_clear_i,
    output logic signed [OUT_WIDTH-1:0] acc_o
);

    // dimc_dual status/handshake signals -- internal only, not exposed as ports
    logic                     READYN;
    logic [23:0]              PSOUT;
    logic                     inp_full, inp_empty;
    logic                     wgt_full, wgt_empty;

    // dimc_dual output FIFO 
    logic out_pop, out_full, out_empty;
    assign out_pop = ~out_empty;

    // Accumulate exactly once per popped result. acc_enable_i can't be
    // pulsed correctly from outside cleopatra since out_pop/out_empty aren't
    // ports -- it's ANDed in here as a global enable/disable instead.
    logic acc_enable;
    assign acc_enable = out_pop & acc_enable_i;

    // dimc_dual output stream -> accumulator input (internal, not exposed)
    logic signed [23:0] out_data;

    spatz_DIMC_dual #(
        .SECTION_WIDTH  (SECTION_WIDTH ),
        .INP_FIFO_DEPTH (INP_FIFO_DEPTH),
        .WGT_FIFO_DEPTH (WGT_FIFO_DEPTH),
        .OUT_FIFO_DEPTH (OUT_FIFO_DEPTH)
    ) u_dimc_dual (
        .clk       (clk      ),
        .rst_n     (rst_n    ),
        .sel       (sel      ),
        .COMPE     (COMPE    ),
        .FCSN      (FCSN     ),
        .MODE      (MODE     ),
        .FA        (FA       ),
        .ADDIN     (ADDIN    ),
        .RA        (RA       ),
        .WA        (WA       ),
        .RCSN      (RCSN     ),
        .RCSN0     (RCSN0    ),
        .RCSN1     (RCSN1    ),
        .RCSN2     (RCSN2    ),
        .RCSN3     (RCSN3    ),
        .WCSN      (WCSN     ),
        .WEN       (WEN      ),
        .M         (M        ),
        .MCT       (MCT      ),
        .READYN    (READYN   ),
        .PSOUT     (PSOUT    ),
        .inp_push  (inp_push ),
        .inp_data  (inp_data ),
        .inp_full  (inp_full ),
        .inp_empty (inp_empty),
        .wgt_push  (wgt_push ),
        .wgt_data  (wgt_data ),
        .wgt_full  (wgt_full ),
        .wgt_empty (wgt_empty),
        .out_pop   (out_pop  ),
        .out_data  (out_data ),
        .out_full  (out_full ),
        .out_empty (out_empty)
    );

    accumulator #(
        .DATA_WIDTH (DATA_WIDTH),
        .OUT_WIDTH  (OUT_WIDTH )
    ) u_accumulator (
        .clk_i    (clk         ),
        .rst_ni   (rst_n       ),
        .enable_i (acc_enable   ),
        .clear_i  (acc_clear_i ),
        .data_i   (out_data    ),
        .acc_o    (acc_o       )
    );

endmodule // cleopatra
