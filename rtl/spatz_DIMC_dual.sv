// spatz_DIMC_dual.sv
//
// ============================================================
// PURPOSE
// ============================================================
// Structural RTL wrapper around TWO spatz_DIMC macros.
// Contents: 
//      2 DIMC Macros (u_mac0, u_mac1)      
//      wgt_fifo (Weight FIFO, depth=128):
//      inp_fifo (Input/Feature FIFO, depth=8):
//      out_fifo (Output FIFO, depth=64):
// ============================================================
// SEL MUX
// ============================================================
// sel routes the active-low enables (COMPE, FCSN, RCSN*, WCSN, WEN) to only
// the selected macro; the idle macro's enables are deasserted.
// Data buses (D, FD, RA, WA, MODE, ADDIN, FA, M, MCT) are shared: both macros
// receive the same values simultaneously.  Only the enables determine which
// macro actually performs an operation.
//   sel=0 → enables go to u_mac0; u_mac1 idles (enables deasserted)
//   sel=1 → enables go to u_mac1; u_mac0 idles (enables deasserted)
//
// ============================================================
// DATA PATHS
// ============================================================
// D  (kernel write data):  wgt_fifo.data_out → both macros (only selected one writes)
// FD (feature data):       inp_fifo.data_out → both macros (only selected one loads)
// 24-bit psum (PSOUT):     selected macro → sign-extended to 32 bits →
//                          out_fifo (auto-push on READYN=0)
//

`timescale 1ns/1ps

// =============================================================================
// MODULE: spatz_DIMC_dual — wrapper around two spatz_DIMC instances
// =============================================================================
module spatz_DIMC_dual #(
    // Width of each 256-bit SRAM section (must match spatz_DIMC parameter).
    parameter int SECTION_WIDTH  = 256,
    // Number of kernel rows in each DIMC (32 rows × 128 bytes = 4096 bytes).
    parameter int NB_KERNEL_ROWS = 32,
    // FIFO depths in number of SECTION_WIDTH-bit entries:
    parameter int INP_FIFO_DEPTH = 8,    // input feature FIFO:  2 complete feature vectors (2 × 4 = 8)
    parameter int WGT_FIFO_DEPTH = 128,  // weight FIFO:         1 complete kernel (32 rows × 4 sections)
    parameter int OUT_FIFO_DEPTH = 64    // output result FIFO:  2 complete MatVec outputs (2 × 32 = 64)
)(
    input  logic clk,     // single clock for all FIFOs and both DIMC macros
    input  logic rst_n,   // active-low reset; clears FIFOs, DIMC pipeline regs, and memories

    // sel: DIMC selector
    input  logic sel,

    // Control inputs 
    // The always_comb mux routes them to only the selected DIMC.
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

    // Outputs — muxed from the CURRENTLY SELECTED DIMC.
    output logic                     READYN,
    output logic [23:0]              PSOUT,

    // Input FIFO 
    input  logic                     inp_push,      
    input  logic [SECTION_WIDTH-1:0] inp_data,      
    output logic                     inp_full,      
    output logic                     inp_empty,     

    // Weight FIFO 
    input  logic                     wgt_push,      
    input  logic [SECTION_WIDTH-1:0] wgt_data,      
    output logic                     wgt_full,      
    output logic                     wgt_empty,    

    // Output FIFO
    input  logic                     out_pop,
    output logic [31:0]              out_data, // PSOUT, sign-extended to 32 bits
    output logic                     out_full,
    output logic                     out_empty
);

    // =========================================================================
    // Input FIFO 
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
    // Weight FIFO 
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
    // Output FIFO
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
    // Per-macro enable signals (sel-gated)
    // =========================================================================
    // Only the enables are per-DIMC — sel routes them to one macro at a time.
    // Data buses (D, FD, RA, WA, MODE, ADDIN, FA, M, MCT) are shared and wired
    // directly to both macro instantiations below.

    logic [1:0] m_compe;   
    logic [1:0] m_fcsn;    
    logic [1:0] m_rcsn;    
    logic [1:0] m_rcsn0;   
    logic [1:0] m_rcsn1;   
    logic [1:0] m_rcsn2;   
    logic [1:0] m_rcsn3;   
    logic [1:0] m_wcsn;    
    logic [1:0] m_wen;     

    logic [1:0]      m_readyn;
    logic [1:0]      m_sout;
    logic [1:0][2:0] m_res_out;
    logic [1:0][23:0]              m_psout; // per-macro PSOUT, muxed by sel into the PSOUT port
    logic [1:0][SECTION_WIDTH-1:0] m_q;     // per-macro Q,     muxed by sel into the internal Q signal below

    logic [SECTION_WIDTH-1:0] Q;
    logic                     SOUT;
    logic [2:0]               RES_OUT;

    // =========================================================================
    // DIMC macro 0 instantiation
    // =========================================================================
    // Enables (COMPE, FCSN, RCSN*, WCSN, WEN) are sel-gated via m_*[0].
    // Data buses (D, FD, RA, WA, MODE, ADDIN, FA, M, MCT) are shared with mac1.
    spatz_DIMC #(.SECTION_WIDTH(SECTION_WIDTH)) u_mac0 (
        .RCK     (clk),
        .RESETn  (rst_n),
        .COMPE   (m_compe[0]),
        .READYN  (m_readyn[0]),
        .FCSN    (m_fcsn[0]),
        .MODE    (MODE),
        .FA      (FA),
        .FD      (inp_rdata),
        .ADDIN   (ADDIN),
        .SOUT    (m_sout[0]),
        .RES_OUT (m_res_out[0]),
        .PSOUT   (m_psout[0]),
        .Q       (m_q[0]),
        .D       (wgt_rdata),
        .RA      (RA),
        .WA      (WA),
        .RCSN    (m_rcsn[0]),
        .RCSN0   (m_rcsn0[0]),
        .RCSN1   (m_rcsn1[0]),
        .RCSN2   (m_rcsn2[0]),
        .RCSN3   (m_rcsn3[0]),
        .WCK     (clk),
        .WCSN    (m_wcsn[0]),
        .WEN     (m_wen[0]),
        .M       (M),
        .MCT     (MCT)
    );

    // =========================================================================
    // DIMC macro 1 instantiation
    // =========================================================================
    // Enables (COMPE, FCSN, RCSN*, WCSN, WEN) are sel-gated via m_*[1].
    // Data buses are identical to mac0 — both see the same shared inputs.
    spatz_DIMC #(.SECTION_WIDTH(SECTION_WIDTH)) u_mac1 (
        .RCK     (clk),
        .RESETn  (rst_n),
        .COMPE   (m_compe[1]),
        .READYN  (m_readyn[1]),
        .FCSN    (m_fcsn[1]),
        .MODE    (MODE),
        .FA      (FA),
        .FD      (inp_rdata),
        .ADDIN   (ADDIN),
        .SOUT    (m_sout[1]),
        .RES_OUT (m_res_out[1]),
        .PSOUT   (m_psout[1]),
        .Q       (m_q[1]),
        .D       (wgt_rdata),
        .RA      (RA),
        .WA      (WA),
        .RCSN    (m_rcsn[1]),
        .RCSN0   (m_rcsn0[1]),
        .RCSN1   (m_rcsn1[1]),
        .RCSN2   (m_rcsn2[1]),
        .RCSN3   (m_rcsn3[1]),
        .WCK     (clk),
        .WCSN    (m_wcsn[1]),
        .WEN     (m_wen[1]),
        .M       (M),
        .MCT     (MCT)
    );

    // =========================================================================
    // ENABLE MUX + FIFO AUTO-MANAGEMENT (combinational)
    // =========================================================================
    // TASK 1 — enable mux:
    //   Default: deassert all enables for both macros so neither fires.
    //   Then:   forward the external enables to only the selected macro.
    //
    // TASK 2 — FIFO auto-management:
    //   wgt_pop:  Fires whenever a kernel write is triggered (WCSN=0 & WEN=0).
    //
    //   inp_pop:  Fires each cycle FCSN=0 is asserted (one pop per feature section).
    //
    //   out_push: Fires when the selected DIMC's READYN goes low (pipeline done).
    //
    //   NOTE: The FIFO registers the push at P(N+5).  Testbench must wait 1 extra cycle.
    // =========================================================================
    always_comb begin
        // --- Defaults: deassert all enables for both macros ---
        m_compe = 2'b00;  // COMPE active-HIGH: deassert both
        m_fcsn  = 2'b11;  // FCSN  active-LOW:  1 = idle
        m_rcsn  = 2'b11;  // RCSN  active-LOW:  1 = idle
        m_rcsn0 = 2'b11;
        m_rcsn1 = 2'b11;
        m_rcsn2 = 2'b11;
        m_rcsn3 = 2'b11;
        m_wcsn  = 2'b11;  // WCSN  active-LOW:  1 = idle
        m_wen   = 2'b11;  // WEN   active-LOW:  1 = idle

        // --- Forward enables to the selected macro only ---
        m_compe[sel]  = COMPE;
        m_fcsn[sel]   = FCSN;
        m_rcsn[sel]   = RCSN;
        m_rcsn0[sel]  = RCSN0;
        m_rcsn1[sel]  = RCSN1;
        m_rcsn2[sel]  = RCSN2;
        m_rcsn3[sel]  = RCSN3;
        m_wcsn[sel]   = WCSN;
        m_wen[sel]    = WEN;

        // --- Output mux: expose selected macro's outputs (READYN/PSOUT as
        // ports; Q/SOUT/RES_OUT as internal signals only) ---
        READYN  = m_readyn[sel];
        Q       = m_q[sel];
        SOUT    = m_sout[sel];
        RES_OUT = m_res_out[sel];
        PSOUT   = m_psout[sel];

        // --- FIFO auto-management ---
        wgt_pop  = ~WCSN & ~WEN & ~wgt_empty;
        inp_pop  = ~FCSN & ~inp_empty;
        out_push  = ~READYN & ~out_full;
        out_wdata = { {8{PSOUT[23]}}, PSOUT }; // sign-extend 24-bit PSOUT to 32 bits
    end

endmodule
