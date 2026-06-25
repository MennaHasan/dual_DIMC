// spatz_DIMC_dual.sv
//
// ============================================================
// PURPOSE
// ============================================================
// Structural RTL wrapper around TWO DIMC_18_fixed macros.
// It adds three synchronous FIFOs and a 1-bit selector mux so that
// the testbench can drive both macros through a single shared port set.
//
// WHAT DIMC_18_fixed DOES (single-macro summary):
//   Stores a 32×128 kernel matrix (uint8) in on-chip SRAM.
//   Multiplies each row by a 128-element uint8 feature vector.
//   Produces a 4-bit quantized result per row (ReLU + saturation).
//   Pipeline depth: 4 cycles after compute is triggered.
//
// WHY TWO MACROS?
//   Two independent DIMC macros can serve two independent tasks
//   (e.g., two different layers or two parallel inference streams)
//   while sharing a common control interface.  The `sel` bit routes
//   all control signals to only one macro at a time.
//
// ============================================================
// THREE FIFOs AND WHY THEY EXIST
// ============================================================
//
//  wgt_fifo (Weight FIFO, depth=128):
//    Holds kernel weight sections waiting to be written into the SRAM.
//    The testbench pushes 256-bit sections here before triggering a
//    kernel write cycle.  The FIFO's data_out drives the DUT's D port;
//    a pop fires automatically when WCSN & WEN are asserted.
//    Depth 128 = exactly one complete kernel (32 rows × 4 sections).
//
//  inp_fifo (Input/Feature FIFO, depth=8):
//    Holds feature vector sections waiting to be loaded into the DIMC's
//    feature buffer.  The testbench pushes all 4 sections here first,
//    then asserts FCSN to trigger the load.  The FIFO's data_out drives
//    the DUT's FD port; a pop fires each cycle FCSN is asserted.
//    Depth 8 = two complete feature vectors (4 sections each).
//
//  out_fifo (Output FIFO, depth=64):
//    Captures the 4-bit quantized result every time the selected DIMC's
//    pipeline completes (READYN goes low).  The push is fully automatic —
//    the testbench only needs to pop results when it wants to read them.
//    Depth 64 = two full matrix-vector outputs (32 results each).
//
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
// 4-bit result (SOUT/RES_OUT): selected macro → out_fifo (auto-push on READYN=0)
//
// ============================================================
// DIAGNOSTIC OUTPUTS
// ============================================================
// mac_psout[0..1]: exposes PSOUT of both macros simultaneously.
//   Lets the testbench compare the two macros' raw partial sums side by side.
// mac_q[0..1]:     exposes Q (kernel readback) of both macros simultaneously.

`timescale 1ns/1ps



// =============================================================================
// MODULE: dimc_dual — wrapper around two DIMC_18_fixed instances
// =============================================================================
module dimc_dual #(
    // Width of each 256-bit SRAM section (must match DIMC_18_fixed parameter).
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

    // -------------------------------------------------------------------------
    // sel: DIMC selector
    //   0 = all control signals go to DIMC 0 (u_mac0); DIMC 1 idles
    //   1 = all control signals go to DIMC 1 (u_mac1); DIMC 0 idles
    //
    // Can be changed between operations (write kernel to DIMC 0, then write
    // kernel to DIMC 1, then compute on either one by flipping sel).
    // -------------------------------------------------------------------------
    input  logic sel,

    // -------------------------------------------------------------------------
    // Control inputs — these are the same signals as DIMC_18_fixed's ports.
    // The always_comb mux routes them to only the selected DIMC.
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    // Outputs — muxed from the CURRENTLY SELECTED DIMC.
    // Switching sel changes which macro's outputs appear here.
    // -------------------------------------------------------------------------
    output logic                     READYN,        
    output logic [SECTION_WIDTH-1:0] Q,             
    output logic                     SOUT,          
    output logic [2:0]               RES_OUT,       
    output logic [23:0]              PSOUT,         

    // -------------------------------------------------------------------------
    // Input FIFO (inp_fifo) external interface — write port only.
    // The testbench pushes feature sections here before calling FCSN load cycles.
    // Internal pop is handled automatically when FCSN is asserted.
    // -------------------------------------------------------------------------
    input  logic                     inp_push,      
    input  logic [SECTION_WIDTH-1:0] inp_data,      
    output logic                     inp_full,      
    output logic                     inp_empty,     

    // -------------------------------------------------------------------------
    // Weight FIFO (wgt_fifo) external interface — write port only.
    // The testbench pushes one kernel section here before each write_kernel_dual cycle.
    // Internal pop fires automatically when WCSN & WEN are both asserted.
    // -------------------------------------------------------------------------
    input  logic                     wgt_push,      
    input  logic [SECTION_WIDTH-1:0] wgt_data,      
    output logic                     wgt_full,      
    output logic                     wgt_empty,    

    // -------------------------------------------------------------------------
    // Output FIFO (out_fifo) external interface — read port only.
    // Results are auto-pushed by this module when READYN goes low.
    // The testbench pops entries at any time after a compute completes.
    // NOTE: out_push is COMBINATIONAL off READYN (a registered DUT output).
    //   This means the FIFO write fires at the posedge AFTER the posedge that
    //   brings READYN low.  Wait one cycle after compute_and_capture_dual returns
    //   before checking out_empty.
    // -------------------------------------------------------------------------
    input  logic                     out_pop,       
    output logic [3:0]               out_data,      
    output logic                     out_full,      
    output logic                     out_empty,     

    // -------------------------------------------------------------------------
    // Diagnostic outputs — both macros exposed simultaneously (not muxed).
    // Useful for debugging: compare DIMC 0 and DIMC 1 side by side in waveforms.
    // -------------------------------------------------------------------------
    output logic [1:0][23:0]              mac_psout,  // mac_psout[0]=DIMC0 partial sum, [1]=DIMC1
    output logic [1:0][SECTION_WIDTH-1:0] mac_q       // mac_q[0]=DIMC0 readback,        [1]=DIMC1
);

    // =========================================================================
    // Input FIFO — feature sections wait here until FCSN is asserted
    // =========================================================================
    // The testbench pushes all 4 sections into this FIFO before driving FCSN=0.
    // Each cycle FCSN=0, inp_rdata (= FIFO head) appears on the DUT's FD port
    // AND inp_pop fires to advance the tail.
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
    // Weight FIFO — kernel sections wait here until a write cycle fires
    // =========================================================================
    // The testbench pushes one section per write_kernel_dual call.
    // When WCSN=0 & WEN=0, wgt_rdata (= FIFO head) appears on the DUT's D port
    // AND wgt_pop fires to advance the tail.
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
    // Output FIFO — captures 4-bit results automatically when READYN goes low
    // =========================================================================
    // out_push fires combinationally whenever ~READYN & ~out_full is true.
    // Because READYN is a registered DUT output, out_push goes high in the same
    // combinational window that READYN goes low (at posedge P(N+4) for the pipeline).
    // The FIFO registers the push at posedge P(N+5) — one cycle later.
    // The testbench must wait one extra cycle after compute completes before popping.
    logic       out_push;     
    logic [3:0] out_wdata;    

    fifo_v3 #(
        .FALL_THROUGH (1'b0),
        .DATA_WIDTH   (4),
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

    // =========================================================================
    // DIMC macro 0 instantiation
    // =========================================================================
    // Enables (COMPE, FCSN, RCSN*, WCSN, WEN) are sel-gated via m_*[0].
    // Data buses (D, FD, RA, WA, MODE, ADDIN, FA, M, MCT) are shared with mac1.
    DIMC_18_fixed #(.SECTION_WIDTH(SECTION_WIDTH)) u_mac0 (
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
        .PSOUT   (mac_psout[0]),
        .Q       (mac_q[0]),
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
    DIMC_18_fixed #(.SECTION_WIDTH(SECTION_WIDTH)) u_mac1 (
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
        .PSOUT   (mac_psout[1]),
        .Q       (mac_q[1]),
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
    // JOB 1 — enable mux:
    //   Default: deassert all enables for both macros so neither fires.
    //   Then:   forward the external enables to only the selected macro.
    //   Data buses are not touched here — they are wired directly in the
    //   instantiations above and both macros see them simultaneously.
    //
    // JOB 2 — FIFO auto-management:
    //   wgt_pop:  Fires whenever a kernel write is triggered (WCSN=0 & WEN=0).
    //             The FIFO's data_out (wgt_rdata) is stable before posedge because
    //             the pop itself is registered and fires at the SAME posedge as the
    //             DIMC write — the DIMC sees data_out (the old tail) and the FIFO
    //             advances the tail at that posedge. ✓ No extra cycle needed.
    //
    //   inp_pop:  Fires each cycle FCSN=0 is asserted (one pop per feature section).
    //             Same safe same-posedge pattern: DIMC captures FD=inp_rdata while
    //             the FIFO tail advances. ✓
    //
    //   out_push: Fires when the selected DIMC's READYN goes low (pipeline done).
    //             READYN is a REGISTERED DUT output, so out_push is combinational
    //             but based on a flip-flop's output — it goes high the moment the
    //             flip-flop goes low, i.e., at posedge P(N+4).  The FIFO registers
    //             the push at P(N+5).  The testbench must wait one extra cycle after
    //             compute_and_capture_dual before checking out_empty.
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

        // --- Output mux: expose selected macro's outputs at module ports ---
        READYN  = m_readyn[sel];
        Q       = mac_q[sel];
        SOUT    = m_sout[sel];
        RES_OUT = m_res_out[sel];
        PSOUT   = mac_psout[sel];

        // --- FIFO auto-management ---

        // wgt_pop: pop the weight FIFO whenever a write fires (both WCSN & WEN low).
        // The guard (~wgt_empty) prevents popping an empty FIFO.
        // Timing: wgt_rdata (old tail) is captured by the DIMC at the posedge that
        // also fires the pop, so data arrives at the macro before being discarded. ✓
        wgt_pop  = ~WCSN & ~WEN & ~wgt_empty;

        // inp_pop: pop the input FIFO once per FCSN=0 cycle.
        // Each FCSN cycle loads one 256-bit section into feature_buf[FA].
        // The FIFO tail advances simultaneously, delivering the next section for the
        // following cycle. ✓
        inp_pop  = ~FCSN & ~inp_empty;

        // out_push: push the 4-bit result into the output FIFO whenever READYN is low.
        // READYN is a registered output: it goes low at posedge P(N+4).
        // out_push therefore goes high at P(N+4) (combinationally).
        // The FIFO registers the push at P(N+5).  Testbench must wait 1 extra cycle.
        out_push  = ~READYN & ~out_full;
        out_wdata = {RES_OUT, SOUT};
    end

endmodule
