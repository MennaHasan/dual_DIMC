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

    parameter int DATA_WIDTH = 32,
    parameter int OUT_WIDTH  = 32
)(
    input  logic clk,
    input  logic rst_n,

    // dimc_dual selector
    input  logic sel,

    // independent control inputs for DIMC 0 and DIMC 1
    input  logic                     COMPE_m0, FCSN_m0,
    input  logic [1:0]               MODE_m0, FA_m0,
    input  logic [31:0]              ADDIN_m0,
    input  logic [6:0]               RA_m0, WA_m0,
    input  logic                     RCSN_m0,
    input  logic                     RCSN0_m0, RCSN1_m0, RCSN2_m0, RCSN3_m0,
    input  logic                     WCSN_m0, WEN_m0,
    input  logic [SECTION_WIDTH-1:0] M_m0,
    input  logic [9:0]               compute_mask_m0,
    input  logic [1:0]               sign_8b_m0,

    input  logic                     COMPE_m1, FCSN_m1,
    input  logic [1:0]               MODE_m1, FA_m1,
    input  logic [31:0]              ADDIN_m1,
    input  logic [6:0]               RA_m1, WA_m1,
    input  logic                     RCSN_m1,
    input  logic                     RCSN0_m1, RCSN1_m1, RCSN2_m1, RCSN3_m1,
    input  logic                     WCSN_m1, WEN_m1,
    input  logic [SECTION_WIDTH-1:0] M_m1,
    input  logic [9:0]               compute_mask_m1,
    input  logic [1:0]               sign_8b_m1,

    // dimc_dual input FIFO
    input  logic                     inp_push,
    input  logic [SECTION_WIDTH-1:0] inp_data,

    // dimc_dual weight FIFO
    input  logic                     wgt_push,
    input  logic [SECTION_WIDTH-1:0] wgt_data,

    // accumulator control
    input  logic                        clear,
    output logic [OUT_WIDTH-1:0]        acc_o [0:255]
);

    // dimc_dual status/handshake signals
    logic                     READYN;
    logic [31:0]              PSOUT;
    logic                     inp_full, inp_empty;
    logic                     wgt_full, wgt_empty;

    // dimc_dual output FIFO 
    logic out_pop, out_full, out_empty;
    assign out_pop = ~out_empty;

    // Accumulate exactly once per popped result. Each accumulator uses its
    // own enable bit, gated by the local out_pop handshake.
    logic [255:0] acc_sel;

    cleopatra_ctrl #(
        .WIDTH (256)
    ) u_ctrl (
        .clk      (clk   ),
        .rst_n    (rst_n ),
        .clear    (clear ),
        .out_pop  (out_pop),
        .acc_sel_o(acc_sel)
    );

    // dimc_dual output stream -> accumulator input 
    logic [31:0] out_data;

    spatz_DIMC_dual #(
        .SECTION_WIDTH  (SECTION_WIDTH ),
        .INP_FIFO_DEPTH (INP_FIFO_DEPTH),
        .WGT_FIFO_DEPTH (WGT_FIFO_DEPTH),
        .OUT_FIFO_DEPTH (OUT_FIFO_DEPTH)
    ) u_dimc_dual (
        .clk       (clk      ),
        .rst_n     (rst_n    ),
        .sel       (sel      ),
        .COMPE_m0  (COMPE_m0 ),
        .FCSN_m0   (FCSN_m0  ),
        .MODE_m0   (MODE_m0  ),
        .FA_m0     (FA_m0    ),
        .ADDIN_m0  (ADDIN_m0 ),
        .RA_m0     (RA_m0    ),
        .WA_m0     (WA_m0    ),
        .RCSN_m0   (RCSN_m0  ),
        .RCSN0_m0  (RCSN0_m0 ),
        .RCSN1_m0  (RCSN1_m0 ),
        .RCSN2_m0  (RCSN2_m0 ),
        .RCSN3_m0  (RCSN3_m0 ),
        .WCSN_m0   (WCSN_m0  ),
        .WEN_m0    (WEN_m0   ),
        .M_m0      (M_m0     ),
        .compute_mask_m0(compute_mask_m0),
        .sign_8b_m0(sign_8b_m0),
        .COMPE_m1  (COMPE_m1 ),
        .FCSN_m1   (FCSN_m1  ),
        .MODE_m1   (MODE_m1  ),
        .FA_m1     (FA_m1    ),
        .ADDIN_m1  (ADDIN_m1 ),
        .RA_m1     (RA_m1    ),
        .WA_m1     (WA_m1    ),
        .RCSN_m1   (RCSN_m1  ),
        .RCSN0_m1  (RCSN0_m1 ),
        .RCSN1_m1  (RCSN1_m1 ),
        .RCSN2_m1  (RCSN2_m1 ),
        .RCSN3_m1  (RCSN3_m1 ),
        .WCSN_m1   (WCSN_m1  ),
        .WEN_m1    (WEN_m1   ),
        .M_m1      (M_m1     ),
        .compute_mask_m1(compute_mask_m1),
        .sign_8b_m1(sign_8b_m1),
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

/* for debugging 
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            // reset behavior, no debug output
        end else if (~out_empty) begin
            $display("[CLEO] time=%0t out_empty=%b out_data=%0h", $time, out_empty, out_data);
        end
    end
*/
    genvar i;
    generate
        for (i = 0; i < 256; i++) begin : gen_accumulators
            logic acc_enable;
            assign acc_enable = out_pop & acc_sel[i];

            accumulator #(
                .DATA_WIDTH (DATA_WIDTH),
                .OUT_WIDTH  (OUT_WIDTH )
            ) u_accumulator (
                .clk_i    (clk         ),
                .rst_ni   (rst_n       ),
                .enable_i (acc_enable  ),
                .clear_i  (clear       ),
                .data_i   (out_data    ),
                .acc_o    (acc_o[i]    )
            );
        end
    endgenerate

endmodule // cleopatra
