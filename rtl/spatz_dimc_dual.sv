/* spatz_dimc_dual.sv
*
* ============================================================
* Structure:
* ============================================================
*      2 DIMC Macros (u_mac0, u_mac1)      
*      wgt_fifo (Weight FIFO, depth=128):
*      inp_fifo (Input/Feature FIFO, depth=8):
*      out_fifo (Output FIFO, depth=64):
* ============================================================
* INDEPENDENT CONTROL / OUTPUT SELECT
* ============================================================
* Each macro has an independent set of control inputs. 
* clk and rst_n are the only shared control signals. 
* sel does not gate commands; it only chooses the
* macro whose outputs are forwarded to the output FIFO.
*
* ============================================================
* DATA PATHS FOR INPUTS AND OUTPUTS
* ============================================================
* D  (kernel write data):  wgt_fifo.data_out → both macros; each macro's
*                          independent WCSN/WEN controls whether it writes.
* FD (feature data):       inp_fifo.data_out → both macros; each macro's
*                          independent FCSN controls whether it loads.
* 32-bit psum (PSOUT):     sel-selected macro → out_fifo.
*/

`timescale 1ns/1ps

// =============================================================================
// MODULE DEFINITION
// =============================================================================
module spatz_dimc_dual #(
    // Width of each 256-bit SRAM section (must match spatz_dimc parameter).
    parameter int SECTION_WIDTH  = 256,
    // FIFO depths
    parameter int INP_FIFO_DEPTH = 8,    // input feature FIFO:  2 complete feature vectors (2 × 4 = 8)
    parameter int WGT_FIFO_DEPTH = 128,  // weight FIFO:         1 complete kernel (32 rows × 4 sections)
    parameter int OUT_FIFO_DEPTH = 64    // output result FIFO:  2 complete MatVec outputs (2 × 32 = 64)
)(
    input  logic clk,     // single clock for all sub modules
    input  logic rst_n,   // active-low reset; clears FIFOs, DIMC pipeline regs, and memories

    // sel: DIMC selector
    input  logic sel,

    // Independent DIMC 0 control inputs
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

    // Independent DIMC 1 control inputs
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

    // Outputs — muxed from the currently selected DIMC.
    output logic                     READYN,
    output logic [31:0]              PSOUT,

    // Input FIFO signals
    input  logic                     inp_push,      
    input  logic [SECTION_WIDTH-1:0] inp_data,      
    output logic                     inp_full,      
    output logic                     inp_empty,     

    // Weight FIFO signals
    input  logic                     wgt_push,      
    input  logic [SECTION_WIDTH-1:0] wgt_data,      
    output logic                     wgt_full,      
    output logic                     wgt_empty,    

    // Output FIFO signals
    input  logic                     out_pop,
    output logic [31:0]              out_data,
    output logic                     out_full,
    output logic                     out_empty
);

    // =========================================================================
    // Input FIFO instantiation
    // =========================================================================
    logic                     inp_pop;       
    logic [SECTION_WIDTH-1:0] inp_rdata;     

    fifo_v3 #(
        .FALL_THROUGH (1'b0),
        .DATA_WIDTH   (SECTION_WIDTH),
        .DEPTH        (INP_FIFO_DEPTH)
    ) u_inp_fifo (
        .clk_i      (clk),
        .rst_ni     (rst_n),
        .flush_i    (1'b0),
        .testmode_i (1'b0),
        .full_o     (inp_full),
        .empty_o    (inp_empty),
        .usage_o    (),
        .data_i     (inp_data),
        .push_i     (inp_push),
        .data_o     (inp_rdata),
        .pop_i      (inp_pop)
    );

    // =========================================================================
    // Weight FIFO instantiation
    // =========================================================================
    logic                     wgt_pop;       
    logic [SECTION_WIDTH-1:0] wgt_rdata;     

    fifo_v3 #(
        .FALL_THROUGH (1'b0),
        .DATA_WIDTH   (SECTION_WIDTH),
        .DEPTH        (WGT_FIFO_DEPTH)
    ) u_wgt_fifo (
        .clk_i      (clk),
        .rst_ni     (rst_n),
        .flush_i    (1'b0),
        .testmode_i (1'b0),
        .full_o     (wgt_full),
        .empty_o    (wgt_empty),
        .usage_o    (),
        .data_i     (wgt_data),
        .push_i     (wgt_push),
        .data_o     (wgt_rdata),
        .pop_i      (wgt_pop)
    );

    // =========================================================================
    // Output FIFO instantiation
    // =========================================================================
    logic        out_push;
    logic [31:0] out_wdata;

    fifo_v3 #(
        .FALL_THROUGH (1'b0),
        .DATA_WIDTH   (32),
        .DEPTH        (OUT_FIFO_DEPTH)
    ) u_out_fifo (
        .clk_i      (clk),
        .rst_ni     (rst_n),
        .flush_i    (1'b0),
        .testmode_i (1'b0),
        .full_o     (out_full),
        .empty_o    (out_empty),
        .usage_o    (),
        .data_i     (out_wdata),
        .push_i     (out_push),
        .data_o     (out_data),
        .pop_i      (out_pop)
    );

    // =========================================================================
    // Per-macro outputs
    // =========================================================================
    // Control inputs are independent. FIFO-provided D and FD remain shared.

    logic [1:0]      m_readyn;
    logic [1:0][7:0] m_sout;
    logic [1:0][31:0]              m_psout; // per-macro PSOUT, muxed by sel into the PSOUT port
    logic [1:0][SECTION_WIDTH-1:0] m_q;     // per-macro Q,     muxed by sel into the internal Q signal below

    logic [SECTION_WIDTH-1:0] Q;

    // =========================================================================
    // DIMC macro 0 instantiation
    // =========================================================================
    // DIMC 0 is driven directly by the _m0 control-input set.
    spatz_dimc #(.SECTION_WIDTH(SECTION_WIDTH)) u_mac0 (
        .RCK     (clk),
        .RESETn  (rst_n),
        .COMPE   (COMPE_m0),
        .READYN  (m_readyn[0]),
        .FCSN    (FCSN_m0),
        .MODE    (MODE_m0),
        .FA      (FA_m0),
        .FD      (inp_rdata),
        .ADDIN   (ADDIN_m0),
        .SOUT    (m_sout[0]),
        .PSOUT   (m_psout[0]),
        .Q       (m_q[0]),
        .D       (wgt_rdata),
        .RA      (RA_m0),
        .WA      (WA_m0),
        .RCSN    (RCSN_m0),
        .RCSN0   (RCSN0_m0),
        .RCSN1   (RCSN1_m0),
        .RCSN2   (RCSN2_m0),
        .RCSN3   (RCSN3_m0),
        .WCK     (clk),
        .WCSN    (WCSN_m0),
        .WEN     (WEN_m0),
        .M       (M_m0),
        .compute_mask(compute_mask_m0),
        .sign_8b (sign_8b_m0)
    );

    // =========================================================================
    // DIMC macro 1 instantiation
    // =========================================================================
    // DIMC 1 is driven directly by the _m1 control-input set.
    spatz_dimc #(.SECTION_WIDTH(SECTION_WIDTH)) u_mac1 (
        .RCK     (clk),
        .RESETn  (rst_n),
        .COMPE   (COMPE_m1),
        .READYN  (m_readyn[1]),
        .FCSN    (FCSN_m1),
        .MODE    (MODE_m1),
        .FA      (FA_m1),
        .FD      (inp_rdata),
        .ADDIN   (ADDIN_m1),
        .SOUT    (m_sout[1]),
        .PSOUT   (m_psout[1]),
        .Q       (m_q[1]),
        .D       (wgt_rdata),
        .RA      (RA_m1),
        .WA      (WA_m1),
        .RCSN    (RCSN_m1),
        .RCSN0   (RCSN0_m1),
        .RCSN1   (RCSN1_m1),
        .RCSN2   (RCSN2_m1),
        .RCSN3   (RCSN3_m1),
        .WCK     (clk),
        .WCSN    (WCSN_m1),
        .WEN     (WEN_m1),
        .M       (M_m1),
        .compute_mask(compute_mask_m1),
        .sign_8b (sign_8b_m1)
    );

    /* =========================================================================
    *  OUTPUT MUX + FIFO AUTO-MANAGEMENT (combinational)
    * =========================================================================
    * wgt_pop fires when either macro requests a write
    * NOTE: simultaneous writes consume one shared FIFO word and present it to both macros.
    *
    * inp_pop fires when either macro requests a feature load.
    *
    * out_push: Fires when the selected DIMC's READYN goes low (pipeline done).
    * =========================================================================*/
    always_comb begin
        // --- Output mux ---
        READYN  = m_readyn[sel];
        Q       = m_q[sel];
        PSOUT   = m_psout[sel];

        // --- FIFO auto-management ---
        wgt_pop  = ((~WCSN_m0 & ~WEN_m0) | (~WCSN_m1 & ~WEN_m1)) & ~wgt_empty;
        inp_pop  = ((~FCSN_m0) | (~FCSN_m1)) & ~inp_empty;
        out_push  = ~READYN & ~out_full;
        out_wdata = PSOUT;
    end

endmodule
