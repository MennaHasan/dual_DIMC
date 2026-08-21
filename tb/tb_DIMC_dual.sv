/*
 * tb_DIMC_dual.sv
 *
 * ============================================================
 * PURPOSE
 * ============================================================
 * Testbench for spatz_DIMC_dual (spatz_DIMC_dual.sv).
 *
 * spatz_DIMC_dual wraps two independently controlled spatz_dimc macros and
 * three FIFOs (weight, input, output). sel selects only the observed output.
 *
 * PIPELINE LATENCY
 *
 *   Trigger at posedge P(N) + ApplTime → registered at P(N+1)
 *   P(N+1) — Stage 0: input capture
 *   P(N+2) — Stage 1: compute_mask masking
 *   P(N+3) — Stage 2: MAC accumulation
 *   P(N+4) — Stage 3: psum + ReLU + clipped; READYN goes low
 *   P(N+5) — out_fifo registers the push (one cycle after READYN goes low)
 *
 * ============================================================
 * TEST STRUCTURE
 * ============================================================
 * Test  0 — Reset check: both DIMCs via sel mux + hierarchical refs
 * Test  1 — Kernel write: DIMC 0, 32 rows × 4 sections
 * Test  2 — Kernel write: DIMC 1, 32 rows × 4 sections (same kernel)
 * Test  3 — Kernel read-back: DIMC 0 (verifies Test 1)
 * Test  4 — Kernel read-back: DIMC 1 (verifies Test 2)
 * Test  5 — Feature load DIMC 0 + single dot product (row 1) + FIFO check
 * Test  6 — Feature load DIMC 1 + single dot product (row 1) + FIFO check
 * Test  7 — Dot product row 4, DIMC 0 (reuses data from Tests 1 & 5)
 * Test  8 — Dot product row 4, DIMC 1 (reuses data from Tests 2 & 6)
 * Test  9 — Full MatVec: DIMC 1, all 32 rows; bulk drain and verify out_fifo
 * Test 10 — Full MatVec: DIMC 0, all 32 rows; bulk drain and verify out_fifo
 * Test 11 — compute_mask sweep: DIMC 1, row 0, 6 compute_mask values; drain and verify out_fifo
 * Test 12 — compute_mask sweep: DIMC 0, row 0, 6 compute_mask values; drain and verify out_fifo
 * Test 13 — Overlapping computes: DIMC0 row 5 triggered first; DIMC1 row 7
 *            triggered 3 cycles before DIMC0 finishes; both results read from
 *            out_fifo in order after both operations complete
 * Test 14 — Pipelined MatVec: DIMC 0, all 32 rows, triggers and result
 *            collection interleaved in one loop (36 cycles total)
 */



// ── Test enable macros ────────────────────────────────────────────────────
// Comment out a line to skip that test at compile time.


`define TB_DUAL_TEST0    // Reset verification
`define TB_DUAL_TEST1    // Kernel write DIMC 0
`define TB_DUAL_TEST5    // Feature load DIMC 0 + dot product row 1 (requires Tests 1 and 3)

`define TB_DUAL_TEST2    // Kernel write DIMC 1
`define TB_DUAL_TEST3    // Kernel read-back DIMC 0 (requires Test 1)
`define TB_DUAL_TEST4    // Kernel read-back DIMC 1 (requires Test 2)
`define TB_DUAL_TEST6    // Feature load DIMC 1 + dot product row 1 (requires Tests 2 and 4)
`define TB_DUAL_TEST7    // Dot product row 4, DIMC 0 (requires Tests 1 and 5)
`define TB_DUAL_TEST8    // Dot product row 4, DIMC 1 (requires Tests 2 and 6)
`define TB_DUAL_TEST9    // Full MatVec DIMC 1 (requires Tests 2 and 6)
`define TB_DUAL_TEST10   // Full MatVec DIMC 0 (requires Tests 1 and 5)
`define TB_DUAL_TEST11   // compute_mask sweep DIMC 1 (requires Tests 2 and 6)
`define TB_DUAL_TEST12   // compute_mask sweep DIMC 0 (requires Tests 1 and 5)
`define TB_DUAL_TEST13   // Overlapping computes DIMC0 row 5 / DIMC1 row 7
`define TB_DUAL_TEST14   // Pipelined MatVec DIMC 0, single-phase

`define TB_DUAL_TEST15   // Pipelined MatVec DIMC 0, single-phase

// ─────────────────────────────────────────────────────────────────────────

`timescale 1ns/1ps

module tb_DIMC_dual;
  import dimc_package::*;

  // -------------------------------------------------------------------------
  // Parameters
  // -------------------------------------------------------------------------
  // SECTION_WIDTH: each DIMC memory section is 256 bits = 32 bytes.
  parameter SECTION_WIDTH  = 256;
  parameter KERNEL_WEIGHTS_FILE         = "stimuli/spatz_dimc_stims/kernel_weights.txt";
  parameter FEATURE_VECTOR_FILE         = "stimuli/spatz_dimc_stims/feature_vector.txt";
  parameter GOLDEN_MATVEC_FILE          = "stimuli/spatz_dimc_stims/golden_clipped_8bit.txt";
  parameter GOLDEN_PSOUT_FILE           = "stimuli/spatz_dimc_stims/golden_psum_32bit.txt";
  parameter GOLDEN_COMPUTE_MASK_FILE    = "stimuli/spatz_dimc_stims/golden_with_masking_8bit.txt";
  parameter GOLDEN_PSOUT_MASK_FILE      = "stimuli/spatz_dimc_stims/golden_psum_with_masking_32bit.txt";

  // NB_COMPUTE_MASK_VALS: number of distinct compute_mask values in the sweep test (Test 11-12).
  parameter NB_COMPUTE_MASK_VALS = 6;

  // BIAS: 32-bit unsigned two's-complement bias constant added to every MAC result at Stage 3.
  // Must match BIAS in spatz_dimc_stim.py and tb_spatz_dimc.sv.
  localparam logic [31:0] BIAS = 32'd0;

  // COMPUTE_MASK_VALS: six threshold values, each trimming different numbers of active elements.
  // Must match COMPUTE_MASK_VALS in spatz_dimc_stim.py and tb_spatz_dimc.sv.
  localparam logic [9:0] COMPUTE_MASK_VALS [NB_COMPUTE_MASK_VALS] =
      '{10'd0, 10'd512, 10'd768, 10'd896, 10'd960, 10'd992};

  // Stimulus and golden arrays (filled by $readmemh at simulation start)
  logic [SECTION_WIDTH-1:0] kernel_stim    [0 : NB_KERNEL_ROWS*4-1]; // 128 sections
  logic [SECTION_WIDTH-1:0] feature_stim   [0 : 3];                   // 4 sections
  logic [7:0]  golden_clipped [0 : NB_KERNEL_ROWS-1];
  logic [31:0] golden_psout [0 : NB_KERNEL_ROWS-1];
  logic [7:0] golden_compute_mask [0 : NB_COMPUTE_MASK_VALS-1];
  logic [31:0] golden_psout_compute_mask [0 : NB_COMPUTE_MASK_VALS-1];

  // Timing: same as tb_spatz_dimc.sv (100 MHz, 2 ns apply, 8 ns test)
  localparam time ClkPeriod = 10ns;
  localparam time ApplTime  =  2ns;
  localparam time TestTime  =  8ns;

  // =========================================================================
  // DUT SIGNAL DECLARATIONS
  // =========================================================================

  // Clock and reset
  logic clk;     // single clock for all FIFOs and both DIMC macros
  logic rst_n;   // active-low reset

  // sel: 0 = u_mac0  1 = u_mac1
  logic sel = 1'b0;

  // All initial values are the safe idle state (no operation firing at time 0).
  logic        COMPE  = 1'b0;
  logic        FCSN   = 1'b1;
  logic [1:0]  MODE   = 2'b11;
  logic [1:0]  FA     = '0;
  logic [31:0] ADDIN  = '0;
  logic [6:0]  RA     = '0;
  logic [6:0]  WA     = '0;
  logic        RCSN   = 1'b1;
  logic        RCSN0  = 1'b1;
  logic        RCSN1  = 1'b1;
  logic        RCSN2  = 1'b1;
  logic        RCSN3  = 1'b1;
  logic        WCSN   = 1'b1;
  logic        WEN    = 1'b1;
  logic [SECTION_WIDTH-1:0] M   = '1;   // write mask: all ones = full word write
  logic [9:0] compute_mask = '0;
  logic [1:0] sign_8b = 2'b00;

  // Outputs from the selected DIMC (muxed inside spatz_DIMC_dual by sel)
  // Q and m_sout are read through hierarchical references for per-macro checks.
  logic                     READYN;
  logic [31:0]              PSOUT;

  // Input feature FIFO interface (driven by this testbench)
  logic                     inp_push = 1'b0;  // push inp_data when high and not full
  logic [SECTION_WIDTH-1:0] inp_data = '0;    // 256-bit section to enqueue
  logic                     inp_full;          // DUT output: FIFO is full
  logic                     inp_empty;         // DUT output: FIFO is empty

  // Weight FIFO interface (driven by this testbench)
  logic                     wgt_push = 1'b0;  // push wgt_data when high and not full
  logic [SECTION_WIDTH-1:0] wgt_data = '0;    // 256-bit kernel section to enqueue
  logic                     wgt_full;          // DUT output: FIFO is full
  logic                     wgt_empty;         // DUT output: FIFO is empty

  // Output FIFO interface (read by this testbench after computes)
  logic        out_pop  = 1'b0;   // pop and discard oldest result when high
  logic [31:0] out_data;
  logic        out_full;           // DUT output: FIFO is full (should never happen in these tests)
  logic        out_empty;          // DUT output: FIFO is empty (no results ready)

  // End-of-test flag — asserted when simulation finishes
  logic eot = 1'b0;

  // =========================================================================
  // DUT INSTANTIATION
  // =========================================================================
  spatz_DIMC_dual #(
    .SECTION_WIDTH  (SECTION_WIDTH)
  ) i_dut (
    .clk      (clk),
    .rst_n    (rst_n),
    .sel      (sel),
    .COMPE_m0 (sel ? 1'b0 : COMPE),
    .FCSN_m0  (sel ? 1'b1 : FCSN),
    .MODE_m0  (MODE),
    .FA_m0    (FA),
    .ADDIN_m0 (ADDIN),
    .RA_m0    (RA),
    .WA_m0    (WA),
    .RCSN_m0  (sel ? 1'b1 : RCSN),
    .RCSN0_m0 (sel ? 1'b1 : RCSN0),
    .RCSN1_m0 (sel ? 1'b1 : RCSN1),
    .RCSN2_m0 (sel ? 1'b1 : RCSN2),
    .RCSN3_m0 (sel ? 1'b1 : RCSN3),
    .WCSN_m0  (sel ? 1'b1 : WCSN),
    .WEN_m0   (sel ? 1'b1 : WEN),
    .M_m0     (M),
    .compute_mask_m0(compute_mask),
    .sign_8b_m0(sign_8b),
    .COMPE_m1 (sel ? COMPE : 1'b0),
    .FCSN_m1  (sel ? FCSN : 1'b1),
    .MODE_m1  (MODE),
    .FA_m1    (FA),
    .ADDIN_m1 (ADDIN),
    .RA_m1    (RA),
    .WA_m1    (WA),
    .RCSN_m1  (sel ? RCSN : 1'b1),
    .RCSN0_m1 (sel ? RCSN0 : 1'b1),
    .RCSN1_m1 (sel ? RCSN1 : 1'b1),
    .RCSN2_m1 (sel ? RCSN2 : 1'b1),
    .RCSN3_m1 (sel ? RCSN3 : 1'b1),
    .WCSN_m1  (sel ? WCSN : 1'b1),
    .WEN_m1   (sel ? WEN : 1'b1),
    .M_m1     (M),
    .compute_mask_m1(compute_mask),
    .sign_8b_m1(sign_8b),
    .READYN   (READYN),
    .PSOUT    (PSOUT),
    .inp_push (inp_push),
    .inp_data (inp_data),
    .inp_full (inp_full),
    .inp_empty(inp_empty),
    .wgt_push (wgt_push),
    .wgt_data (wgt_data),
    .wgt_full (wgt_full),
    .wgt_empty(wgt_empty),
    .out_pop  (out_pop),
    .out_data (out_data),
    .out_full (out_full),
    .out_empty(out_empty)
  );

  // CLOCK GENERATION AND RESET
  initial begin
    clk   = 1'b0;
    rst_n = 1'b0;
    repeat (3) begin
      #(ClkPeriod/2) clk = 1'b0;
      #(ClkPeriod/2) clk = 1'b1;
    end
    rst_n = 1'b1;
    forever begin
      #(ClkPeriod/2) clk = 1'b0;
      #(ClkPeriod/2) clk = 1'b1;
    end
  end

  // PROTOCOL TASKS
  task automatic write_kernel_section_dual(
    input [4:0]               row,   // kernel row to write (0-31)
    input [1:0]               sec,   // section within row (0-3, each 256 bits)
    input [SECTION_WIDTH-1:0] data   // 256-bit data to write
  );
    // Cycle 0: push section data into wgt_fifo; FIFO registers it at posedge
    @(posedge clk); #ApplTime;
    wgt_push = 1'b1; wgt_data = data;
    WCSN = 1'b1; WEN = 1'b1; RCSN = 1'b1; FCSN = 1'b1;   // all idle

    // Cycle 1: wgt_rdata is now valid; assert write enables
    @(posedge clk); #ApplTime;
    wgt_push = 1'b0;                   // stop pushing (only one section needed)
    COMPE = 1'b0; WA = {row, sec}; M = '1;
    WCSN = 1'b0; WEN = 1'b0; RCSN = 1'b1; FCSN = 1'b1;   // trigger write

    // Cycle 2: posedge latches kernel write AND wgt_pop; deassert enables
    @(posedge clk); #ApplTime;
    WCSN = 1'b1; WEN = 1'b1;
  endtask


  task automatic write_full_kernel_dual(
    input logic [SECTION_WIDTH-1:0] kernel [0:NB_KERNEL_ROWS*4-1]
  );
    int NB_SECTIONS = NB_KERNEL_ROWS*4;

    // Cycle 1 (alignment): nothing pushed/written yet; set up section 0's push.
    @(posedge clk); #ApplTime;
    COMPE = 1'b0; RCSN = 1'b1; FCSN = 1'b1; M = '1;
    WCSN  = 1'b1; WEN  = 1'b1;
    wgt_push = 1'b1; wgt_data = kernel[0];

    // Each edge: push section i+1 (if any left) while writing section i-1
    // (its push registered 1 edge earlier), addressed by {row,sec} = i-1.
    for (int i = 0; i < NB_SECTIONS; i++) begin
      @(posedge clk); #ApplTime;
      if (i < NB_SECTIONS-1)
        wgt_data = kernel[i+1];
      else
        wgt_push = 1'b0;              // last section already pushed this edge
      WA = 7'(i);                     // {row,sec} of section i -- write it next edge
      WCSN = 1'b0; WEN = 1'b0;
    end

    // Final edge: last section's write completes.
    @(posedge clk); #ApplTime;
    WCSN = 1'b1; WEN = 1'b1;
  endtask

  // read_kernel_dual — reads one 256-bit section from the selected DIMC's SRAM.
  task automatic read_kernel_dual(
    input  [4:0]               row,    // row to read (0-31)
    input  [1:0]               sec,    // section (0-3)
    output [SECTION_WIDTH-1:0] rdata   // output: captured Q value
  );
    @(posedge clk); #ApplTime;
    COMPE = 1'b0; RCSN = 1'b0; RA = {row, sec};
    WCSN = 1'b1; WEN = 1'b1; FCSN = 1'b1;   // write and feature paths idle
    @(posedge clk); #TestTime;
    rdata = i_dut.Q;   // Q is internal to spatz_DIMC_dual, not a port
    RCSN  = 1'b1;
  endtask

  // load_feature_dual — writes all 4 sections of the feature vector into the
  // feature buffer. Push and load are interleaved rather than phase-separated:
  // a section's push registers at edge N, so inp_fifo's empty_o/data_o already
  // reflect it starting the very next edge -- there's no need to wait for all
  // 4 pushes before starting to drain. Loading section i therefore starts
  // exactly 1 cycle after section i's push registers, overlapping with the
  // push of section i+1. Total: 6 edges (1 alignment + 5), vs. 9 for a fully
  // phase-separated push-then-load version.
  task automatic load_feature_dual(
    input [SECTION_WIDTH-1:0] f0,
    input [SECTION_WIDTH-1:0] f1,
    input [SECTION_WIDTH-1:0] f2,
    input [SECTION_WIDTH-1:0] f3
  );
    @(posedge clk); #ApplTime;
    inp_push = 1'b1; inp_data = f0;                // set up for f0 push

    @(posedge clk); #ApplTime;                     // f0 pushed this edge
    inp_data = f1;
    FCSN = 1'b0; FA = 2'd0;                        // set up for f0 load

    @(posedge clk); #ApplTime;                     // f1 pushed; f0 now loaded
    inp_data = f2; FA = 2'd1;

    @(posedge clk); #ApplTime;                     // f2 pushed; f1 now loaded
    inp_data = f3; FA = 2'd2;

    @(posedge clk); #ApplTime;                     // f3 pushed, f2 now loaded
    inp_push = 1'b0; FA = 2'd3;

    @(posedge clk); #ApplTime;                     // f3 now loaded
    FCSN = 1'b1; FA = '0;                          // deassert; buffer retains all four values
  endtask

  // ---------------------------------------------------------------------------
  // compute_and_capture_dual
  // ---------------------------------------------------------------------------
  // triggers one MAC on the selected DIMC and captures the Stage 3 result.
  // NOTES:
  //   - The out_fifo receives a copy of the result AUTOMATICALLY
  //   - out_push goes high at P(N+4)
  //   - out_fifo registers the push at P(N+5)
  task automatic compute_and_capture_dual(
    input  [4:0]  row,
    input  [31:0] bias,
    input  [9:0] compute_mask_val,
    output [31:0] psout,
    output [7:0] clipped
  );
    // --- Cycle N: assert compute trigger for exactly ONE cycle ---
    @(posedge clk); #ApplTime;
    COMPE = 1'b1; MODE = 2'b11; compute_mask = compute_mask_val; sign_8b = 2'b00;
    RA = {row, 2'b00}; ADDIN = bias;   // RA section bits ignored in compute mode
    RCSN = 1'b0; RCSN0 = 1'b0; RCSN1 = 1'b0; RCSN2 = 1'b0; RCSN3 = 1'b0;
    WCSN = 1'b1; WEN = 1'b1; FCSN = 1'b1;   // write/feature paths idle

    // --- Cycle N+1: deassert trigger; Stage 0 captures inputs ---
    @(posedge clk); #ApplTime;
    COMPE = 1'b0;
    RCSN = 1'b1; RCSN0 = 1'b1; RCSN1 = 1'b1; RCSN2 = 1'b1; RCSN3 = 1'b1;

    // --- Cycles N+2, N+3: pipeline advances through Stages 1 and 2 ---
    @(posedge clk);   // Stage 1: computation masking
    @(posedge clk);   // Stage 2: MAC accumulation

    // --- Cycle N+4: Stage 3 completes; READYN goes low ---
    @(posedge clk); #TestTime;
    if (READYN !== 1'b0)
      $error("[TB] READYN did not go low after 4-cycle pipeline (row=%0d, sel=%0d)", row, sel);
    psout = PSOUT;
    clipped = i_dut.m_sout[sel];
    // NOTE:
    // Caller must wait one extra posedge before checking out_empty.
  endtask

  // =========================================================================
  // PASS / FAIL COUNTERS
  // =========================================================================
  int pass_count = 0;
  int fail_count = 0;

  // =========================================================================
  // MAIN TEST SEQUENCE
  // =========================================================================
  initial begin
    logic [SECTION_WIDTH-1:0] rd_data;
    logic [31:0]              psout;
    logic [7:0]               clipped;

    // Wait for reset release, then one idle cycle for DUT outputs to settle
    @(posedge rst_n);
    @(posedge clk);

    // Load all stimulus and golden data from files generated by spatz_dimc_stim.py.
    $readmemh(KERNEL_WEIGHTS_FILE,         kernel_stim);     // 128 sections: 32 rows × 4
    $readmemh(FEATURE_VECTOR_FILE,         feature_stim);    //   4 sections: 1024-bit vector
    $readmemh(GOLDEN_MATVEC_FILE,       golden_clipped);
    $readmemh(GOLDEN_PSOUT_FILE,        golden_psout);
    $readmemh(GOLDEN_COMPUTE_MASK_FILE, golden_compute_mask);
    $readmemh(GOLDEN_PSOUT_MASK_FILE,   golden_psout_compute_mask);

`ifdef TB_DUAL_TEST0
    // =======================================================================
    // TEST 0: RESET VERIFICATION
    // =======================================================================
    $display("[TB] Test 0: Reset verification");
    begin
      automatic logic reset_ok = 1'b1;

      // Check DIMC 0 via sel mux
      sel = 1'b0;
      @(posedge clk); #TestTime;
      if (READYN !== 1'b1)                       reset_ok = 1'b0;   // not-ready on reset
      if (PSOUT !== 32'h0)             reset_ok = 1'b0;
      if (i_dut.m_sout[sel] !== 8'h0)  reset_ok = 1'b0;
      if (i_dut.Q !== '0)                         reset_ok = 1'b0;   // kernel readback cleared

      // Check DIMC 1 via sel mux
      sel = 1'b1;
      @(posedge clk); #TestTime;
      if (READYN !== 1'b1)                       reset_ok = 1'b0;
      if (PSOUT !== 32'h0)             reset_ok = 1'b0;
      if (i_dut.m_sout[sel] !== 8'h0)  reset_ok = 1'b0;
      if (i_dut.Q !== '0)                         reset_ok = 1'b0;
      sel = 1'b0;   // return to default

      // Check internal feature buffers of both macros via hierarchical reference
      for (int s = 0; s < 4; s++) begin
        if (i_dut.u_mac0.feature_buf[s] !== '0) reset_ok = 1'b0;
        if (i_dut.u_mac1.feature_buf[s] !== '0) reset_ok = 1'b0;
      end

      // Check kernel SRAMs of both macros via hierarchical reference (32 rows × 4 sections)
      for (int r = 0; r < NB_KERNEL_ROWS; r++)
        for (int s = 0; s < 4; s++) begin
          if (i_dut.u_mac0.kernel_mem[r][s] !== '0) reset_ok = 1'b0;
          if (i_dut.u_mac1.kernel_mem[r][s] !== '0) reset_ok = 1'b0;
        end

      if (!reset_ok) begin $error("[TB] Test 0 FAIL: signals not zero/one after reset"); fail_count++; end
      else           begin $display("[TB] Test 0: PASS"); pass_count++; end
    end
`endif // TB_DUAL_TEST0

`ifdef TB_DUAL_TEST1
    // =======================================================================
    // TEST 1: KERNEL WRITE — DIMC 0
    // =======================================================================
    $display("[TB] Test 1: Kernel write DIMC 0");
    sel = 1'b0;   // target DIMC 0
    write_full_kernel_dual(kernel_stim);
    $display("[TB] Test 1: DONE (verified in Test 3)");
`endif // TB_DUAL_TEST1

`ifdef TB_DUAL_TEST2
    // =======================================================================
    // TEST 2: KERNEL WRITE — DIMC 1
    // =======================================================================
    $display("[TB] Test 2: Kernel write DIMC 1");
    sel = 1'b1;   // target DIMC 1
    write_full_kernel_dual(kernel_stim);
    $display("[TB] Test 2: DONE (verified in Test 4)");
`endif // TB_DUAL_TEST2

`ifdef TB_DUAL_TEST3
    // =======================================================================
    // TEST 3: KERNEL READ-BACK — DIMC 0
    // =======================================================================
    $display("[TB] Test 3: Kernel read-back DIMC 0");
    begin
      automatic int test_fail = 0;
      sel = 1'b0;
      for (int r = 0; r < NB_KERNEL_ROWS; r++)
        for (int s = 0; s < 4; s++) begin
          read_kernel_dual(5'(r), 2'(s), rd_data);
          if (rd_data !== kernel_stim[r*4 + s]) begin
            $error("[TB] Test3 DIMC0 row%0d sec%0d: got 0x%h, expected 0x%h",
                   r, s, rd_data, kernel_stim[r*4 + s]);
            test_fail++;
          end
        end
      if (test_fail == 0) begin $display("[TB] Test 3: PASS"); pass_count++; end
      else                begin $display("[TB] Test 3: FAIL (%0d mismatches)", test_fail); fail_count++; end
    end
`endif // TB_DUAL_TEST3

`ifdef TB_DUAL_TEST4
    // =======================================================================
    // TEST 4: KERNEL READ-BACK — DIMC 1
    // =======================================================================
    $display("[TB] Test 4: Kernel read-back DIMC 1");
    begin
      automatic int test_fail = 0;
      sel = 1'b1;
      for (int r = 0; r < NB_KERNEL_ROWS; r++)
        for (int s = 0; s < 4; s++) begin
          read_kernel_dual(5'(r), 2'(s), rd_data);
          if (rd_data !== kernel_stim[r*4 + s]) begin
            $error("[TB] Test4 DIMC1 row%0d sec%0d: got 0x%h, expected 0x%h",
                   r, s, rd_data, kernel_stim[r*4 + s]);
            test_fail++;
          end
        end
      if (test_fail == 0) begin $display("[TB] Test 4: PASS"); pass_count++; end
      else                begin $display("[TB] Test 4: FAIL (%0d mismatches)", test_fail); fail_count++; end
    end
`endif // TB_DUAL_TEST4

`ifdef TB_DUAL_TEST5
    // =======================================================================
    // TEST 5: FEATURE LOAD DIMC 0 + DOT PRODUCT ROW 1
    // =======================================================================
    $display("[TB] Test 5: Feature load DIMC 0 + dot product row 1");
    begin
      automatic int test_fail = 0;
      sel = 1'b0;
      load_feature_dual(feature_stim[0], feature_stim[1], feature_stim[2], feature_stim[3]);
      compute_and_capture_dual(5'd1, BIAS, 10'd0, psout, clipped);
      // Verify direct DIMC output ports
      if (psout !== golden_psout[1] || clipped !== golden_clipped[1]) begin
        if (psout !== golden_psout[1])
          $error("[TB] Test5 DIMC0 row1: psout got 0x%08h, expected 0x%08h", psout, golden_psout[1]);
        if (clipped !== golden_clipped[1])
          $error("[TB] Test5 DIMC0 row1: clipped got %0d, expected %0d", clipped, golden_clipped[1]);
        test_fail++;
      end
      // Wait one cycle: out_push fires at P(N+4); out_fifo registers at P(N+5).
      // Without this wait, out_empty may still be high immediately after the task returns.
      @(posedge clk); #ApplTime;
      if (out_empty) begin
        $error("[TB] Test5: out_fifo empty AND push did not fire"); test_fail++;
      end else if (out_data !== golden_psout[1]) begin
        $error("[TB] Test5: out_fifo got 0x%08h, expected 0x%08h", out_data, golden_psout[1]); test_fail++;
      end else begin
        // Pop to clear the result from the FIFO
        out_pop = 1'b1; @(posedge clk); #ApplTime; out_pop = 1'b0;
      end
      if (test_fail == 0) begin $display("[TB] Test 5: PASS"); pass_count++; end
      else                begin $display("[TB] Test 5: FAIL"); fail_count++; end
    end
`endif // TB_DUAL_TEST5

`ifdef TB_DUAL_TEST6
    // =======================================================================
    // TEST 6: FEATURE LOAD DIMC 1 + DOT PRODUCT ROW 1
    // =======================================================================
    $display("[TB] Test 6: Feature load DIMC 1 + dot product row 1");
    begin
      automatic int test_fail = 0;
      sel = 1'b1;
      load_feature_dual(feature_stim[0], feature_stim[1], feature_stim[2], feature_stim[3]);
      compute_and_capture_dual(5'd1, BIAS, 10'd0, psout, clipped);
      if (psout !== golden_psout[1] || clipped !== golden_clipped[1]) begin
        if (psout !== golden_psout[1])
          $error("[TB] Test6 DIMC1 row1: psout got 0x%08h, expected 0x%08h", psout, golden_psout[1]);
        if (clipped !== golden_clipped[1])
          $error("[TB] Test6 DIMC1 row1: clipped got %0d, expected %0d", clipped, golden_clipped[1]);
        test_fail++;
      end
      @(posedge clk); #ApplTime;   // wait for out_fifo push to register
      if (out_empty) begin
        $error("[TB] Test6: out_fifo empty — push did not fire"); test_fail++;
      end else if (out_data !== golden_psout[1]) begin
        $error("[TB] Test6: out_fifo got 0x%08h, expected 0x%08h", out_data, golden_psout[1]); test_fail++;
      end
      out_pop = 1'b1; @(posedge clk); #ApplTime; out_pop = 1'b0;
      if (test_fail == 0) begin $display("[TB] Test 6: PASS"); pass_count++; end
      else                begin $display("[TB] Test 6: FAIL"); fail_count++; end
    end
`endif // TB_DUAL_TEST6

`ifdef TB_DUAL_TEST7
    // =======================================================================
    // TEST 7: DOT PRODUCT ROW 4 — DIMC 0
    // =======================================================================
    $display("[TB] Test 7: Dot product row 4, DIMC 0");
    begin
      automatic int test_fail = 0;
      sel = 1'b0;
      compute_and_capture_dual(5'd4, BIAS, 10'd0, psout, clipped);
      if (psout !== golden_psout[4] || clipped !== golden_clipped[4]) begin
        if (psout !== golden_psout[4])
          $error("[TB] Test7 DIMC0 row4: psout got 0x%08h, expected 0x%08h", psout, golden_psout[4]);
        if (clipped !== golden_clipped[4])
          $error("[TB] Test7 DIMC0 row4: clipped got %0d, expected %0d", clipped, golden_clipped[4]);
        test_fail++;
      end
      @(posedge clk); #ApplTime;   // wait for out_fifo push to register
      if (out_empty) begin
        $error("[TB] Test7: out_fifo empty — push did not fire"); test_fail++;
      end else if (out_data !== golden_psout[4]) begin
        $error("[TB] Test7: out_fifo got 0x%08h, expected 0x%08h", out_data, golden_psout[4]); test_fail++;
      end
      out_pop = 1'b1; @(posedge clk); #ApplTime; out_pop = 1'b0;
      if (test_fail == 0) begin $display("[TB] Test 7: PASS"); pass_count++; end
      else                begin $display("[TB] Test 7: FAIL"); fail_count++; end
    end
`endif // TB_DUAL_TEST7

`ifdef TB_DUAL_TEST8
    // =======================================================================
    // TEST 8: DOT PRODUCT ROW 4 — DIMC 1
    // =======================================================================
    $display("[TB] Test 8: Dot product row 4, DIMC 1");
    begin
      automatic int test_fail = 0;
      sel = 1'b1;
      compute_and_capture_dual(5'd4, BIAS, 10'd0, psout, clipped);
      if (psout !== golden_psout[4] || clipped !== golden_clipped[4]) begin
        if (psout !== golden_psout[4])
          $error("[TB] Test8 DIMC1 row4: psout got 0x%08h, expected 0x%08h", psout, golden_psout[4]);
        if (clipped !== golden_clipped[4])
          $error("[TB] Test8 DIMC1 row4: clipped got %0d, expected %0d", clipped, golden_clipped[4]);
        test_fail++;
      end
      @(posedge clk); #ApplTime;
      if (out_empty) begin
        $error("[TB] Test8: out_fifo empty — push did not fire"); test_fail++;
      end else if (out_data !== golden_psout[4]) begin
        $error("[TB] Test8: out_fifo got 0x%08h, expected 0x%08h", out_data, golden_psout[4]); test_fail++;
      end
      out_pop = 1'b1; @(posedge clk); #ApplTime; out_pop = 1'b0;
      if (test_fail == 0) begin $display("[TB] Test 8: PASS"); pass_count++; end
      else                begin $display("[TB] Test 8: FAIL"); fail_count++; end
    end
`endif // TB_DUAL_TEST8

`ifdef TB_DUAL_TEST9
    // =======================================================================
    // TEST 9: FULL MATRIX-VECTOR MULTIPLICATION — DIMC 1 (all 32 rows)
    // =======================================================================
    // Fires 32 consecutive compute operations on DIMC 1 (rows 0-31) without
    // draining out_fifo in between.
    $display("[TB] Test 9: Full matrix-vector multiplication, DIMC 1 (32 rows)");
    begin
      automatic int test_fail = 0;
      sel = 1'b1;
      for (int r = 0; r < NB_KERNEL_ROWS; r++) begin
        compute_and_capture_dual(5'(r), BIAS, 10'd0, psout, clipped);
        // Check direct outputs from each compute immediately
        if (psout !== golden_psout[r] || clipped !== golden_clipped[r]) begin
          if (psout !== golden_psout[r])
            $error("[TB] Test9 DIMC1 row%0d: psout got 0x%08h, expected 0x%08h",
                   r, psout, golden_psout[r]);
          if (clipped !== golden_clipped[r])
            $error("[TB] Test9 DIMC1 row%0d: clipped got %0d, expected %0d",
                   r, clipped, golden_clipped[r]);
          test_fail++;
        end
        // Note: out_fifo pushes are batching up in the background.
        // We do NOT pop here; the FIFO holds all 32 results until the drain below.
      end

      // Wait for the LAST result to register into out_fifo.
      // (The last compute_and_capture_dual returned after P(N+4)+TestTime;
      // out_fifo push fires at P(N+4) but registers at P(N+5).)
      @(posedge clk); #ApplTime;

      // Drain and verify all 32 results in issue order
      for (int r = 0; r < NB_KERNEL_ROWS; r++) begin
        if (out_empty) begin
          $error("[TB] Test9 out_fifo empty at row%0d — push did not fire", r); test_fail++;
        end else if (out_data !== golden_psout[r]) begin
          $error("[TB] Test9 out_fifo row%0d: got 0x%08h, expected 0x%08h",
                 r, out_data, golden_psout[r]); test_fail++;
        end
        out_pop = 1'b1; @(posedge clk); #ApplTime; out_pop = 1'b0;
      end
      if (test_fail == 0) begin $display("[TB] Test 9: PASS"); pass_count++; end
      else                begin $display("[TB] Test 9: FAIL (%0d mismatches)", test_fail); fail_count++; end
    end
`endif // TB_DUAL_TEST9

`ifdef TB_DUAL_TEST10
    // =======================================================================
    // TEST 10: FULL MATRIX-VECTOR MULTIPLICATION — DIMC 0 (all 32 rows)
    // =======================================================================
    // Same as Test 9 but on DIMC 0.
    $display("[TB] Test 10: Full matrix-vector multiplication, DIMC 0 (32 rows)");
    begin
      automatic int test_fail = 0;
      sel = 1'b0;
      for (int r = 0; r < NB_KERNEL_ROWS; r++) begin
        compute_and_capture_dual(5'(r), BIAS, 10'd0, psout, clipped);
        if (psout !== golden_psout[r] || clipped !== golden_clipped[r]) begin
          if (psout !== golden_psout[r])
            $error("[TB] Test10 DIMC0 row%0d: psout got 0x%08h, expected 0x%08h",
                   r, psout, golden_psout[r]);
          if (clipped !== golden_clipped[r])
            $error("[TB] Test10 DIMC0 row%0d: clipped got %0d, expected %0d",
                   r, clipped, golden_clipped[r]);
          test_fail++;
        end
      end
      @(posedge clk); #ApplTime;   // wait for last push to register
      for (int r = 0; r < NB_KERNEL_ROWS; r++) begin
        if (out_empty) begin
          $error("[TB] Test10 out_fifo empty at row%0d — push did not fire", r); test_fail++;
        end else if (out_data !== golden_psout[r]) begin
          $error("[TB] Test10 out_fifo row%0d: got 0x%08h, expected 0x%08h",
                 r, out_data, golden_psout[r]); test_fail++;
        end
        out_pop = 1'b1; @(posedge clk); #ApplTime; out_pop = 1'b0;
      end
      if (test_fail == 0) begin $display("[TB] Test 10: PASS"); pass_count++; end
      else                begin $display("[TB] Test 10: FAIL (%0d mismatches)", test_fail); fail_count++; end
    end
`endif // TB_DUAL_TEST10

`ifdef TB_DUAL_TEST11
    // =======================================================================
    // TEST 11: compute_mask MASKING SWEEP — DIMC 1 (row 0, 6 compute_mask values)
    // =======================================================================
    // Sweeps 6 compute_mask values on DIMC 1's row 0.
    // The 6 results accumulate
    // in out_fifo and are drained in a bulk drain after the loop.

    $display("[TB] Test 11: compute_mask masking sweep, DIMC 1 (%0d values, row 0)", NB_COMPUTE_MASK_VALS);
    begin
      automatic int test_fail = 0;
      sel = 1'b1;
      for (int m = 0; m < NB_COMPUTE_MASK_VALS; m++) begin
        compute_and_capture_dual(5'd0, BIAS, COMPUTE_MASK_VALS[m], psout, clipped);
        if (psout !== golden_psout_compute_mask[m] || clipped !== golden_compute_mask[m]) begin
          if (psout !== golden_psout_compute_mask[m])
            $error("[TB] Test11 DIMC1 compute_mask=0x%02h: psout got 0x%08h, expected 0x%08h",
                   COMPUTE_MASK_VALS[m], psout, golden_psout_compute_mask[m]);
          if (clipped !== golden_compute_mask[m])
            $error("[TB] Test11 DIMC1 compute_mask=0x%02h: clipped got %0d, expected %0d",
                   COMPUTE_MASK_VALS[m], clipped, golden_compute_mask[m]);
          test_fail++;
        end
      end
      @(posedge clk); #ApplTime;   // wait for last push to register
      for (int m = 0; m < NB_COMPUTE_MASK_VALS; m++) begin
        if (out_empty) begin
          $error("[TB] Test11 out_fifo empty at compute_mask index%0d — push did not fire", m); test_fail++;
        end else if (out_data !== golden_psout_compute_mask[m]) begin
          $error("[TB] Test11 out_fifo compute_mask=0x%02h: got 0x%08h, expected 0x%08h",
                 COMPUTE_MASK_VALS[m], out_data, golden_psout_compute_mask[m]); test_fail++;
        end
        out_pop = 1'b1; @(posedge clk); #ApplTime; out_pop = 1'b0;
      end
      if (test_fail == 0) begin $display("[TB] Test 11: PASS"); pass_count++; end
      else                begin $display("[TB] Test 11: FAIL (%0d mismatches)", test_fail); fail_count++; end
    end
`endif // TB_DUAL_TEST11

`ifdef TB_DUAL_TEST12
    // =======================================================================
    // TEST 12: compute_mask MASKING SWEEP — DIMC 0 (row 0, 6 compute_mask values)
    // =======================================================================
    $display("[TB] Test 12: compute_mask masking sweep, DIMC 0 (%0d values, row 0)", NB_COMPUTE_MASK_VALS);
    begin
      automatic int test_fail = 0;
      sel = 1'b0;
      for (int m = 0; m < NB_COMPUTE_MASK_VALS; m++) begin
        compute_and_capture_dual(5'd0, BIAS, COMPUTE_MASK_VALS[m], psout, clipped);
        if (psout !== golden_psout_compute_mask[m] || clipped !== golden_compute_mask[m]) begin
          if (psout !== golden_psout_compute_mask[m])
            $error("[TB] Test12 DIMC0 compute_mask=0x%02h: psout got 0x%08h, expected 0x%08h",
                   COMPUTE_MASK_VALS[m], psout, golden_psout_compute_mask[m]);
          if (clipped !== golden_compute_mask[m])
            $error("[TB] Test12 DIMC0 compute_mask=0x%02h: clipped got %0d, expected %0d",
                   COMPUTE_MASK_VALS[m], clipped, golden_compute_mask[m]);
          test_fail++;
        end
      end
      @(posedge clk); #ApplTime;
      for (int m = 0; m < NB_COMPUTE_MASK_VALS; m++) begin
        if (out_empty) begin
          $error("[TB] Test12 out_fifo empty at compute_mask index%0d — push did not fire", m); test_fail++;
        end else if (out_data !== golden_psout_compute_mask[m]) begin
          $error("[TB] Test12 out_fifo compute_mask=0x%02h: got 0x%08h, expected 0x%08h",
                 COMPUTE_MASK_VALS[m], out_data, golden_psout_compute_mask[m]); test_fail++;
        end
        out_pop = 1'b1; @(posedge clk); #ApplTime; out_pop = 1'b0;
      end
      if (test_fail == 0) begin $display("[TB] Test 12: PASS"); pass_count++; end
      else                begin $display("[TB] Test 12: FAIL (%0d mismatches)", test_fail); fail_count++; end
    end
`endif // TB_DUAL_TEST12

`ifdef TB_DUAL_TEST13
    // =======================================================================
    // TEST 13: OVERLAPPING OPERATIONS — DIMC0 row 5 then DIMC1 row 7
    // =======================================================================
    // DIMC0 row 5 is triggered first.  DIMC1 row 7 is triggered 3 cycles before
    // DIMC0 finishes (i.e., one cycle after DIMC0 enters its pipeline).
    // The out_fifo is NOT read until both operations have completed.
    // Both results are then read and verified in order (DIMC0 first, DIMC1 second).
    $display("[TB] Test 13: Overlapping computes — DIMC0 row 5 / DIMC1 row 7");
    begin
      automatic int test_fail = 0;

      // P(N): trigger DIMC0 row 5
      sel = 1'b0;
      @(posedge clk); #ApplTime;
      COMPE = 1'b1; MODE = 2'b11; compute_mask = 10'd0;
      RA    = {5'd5, 2'b00}; ADDIN = BIAS;
      RCSN  = 1'b0; RCSN0 = 1'b0; RCSN1 = 1'b0; RCSN2 = 1'b0; RCSN3 = 1'b0;
      WCSN  = 1'b1; WEN   = 1'b1; FCSN  = 1'b1;

      // P(N+1): DIMC0 Stage0 latches; switch sel=1 and trigger DIMC1 row 7.
      // This is 3 cycles before DIMC0 Stage3 — the precise overlap requested.
      @(posedge clk); #ApplTime;
      sel   = 1'b1;
      COMPE = 1'b1;
      RA    = {5'd7, 2'b00};   // row 7 for DIMC1; ADDIN/compute_mask/MODE unchanged

      // P(N+2): DIMC1 Stage0 latches; deassert trigger; switch sel=0 so READYN
      // tracks DIMC0 from here through its Stage3 completion at P(N+4).
      @(posedge clk); #ApplTime;
      COMPE = 1'b0;
      RCSN  = 1'b1; RCSN0 = 1'b1; RCSN1 = 1'b1; RCSN2 = 1'b1; RCSN3 = 1'b1;
      sel   = 1'b0;

      // P(N+3): DIMC0 Stage2; DIMC1 Stage1
      @(posedge clk);

      // P(N+4): DIMC0 Stage3 → READYN[0] falls → out_push=1 with sel=0
      @(posedge clk);

      // P(N+5): out_fifo registers DIMC0 result; DIMC1 Stage3 → READYN[1] falls.
      // Switch sel=1 after ApplTime so out_push reflects DIMC1's READYN.
      @(posedge clk); #ApplTime;
      sel = 1'b1;

      // P(N+6): out_fifo registers DIMC1 result — both entries now in FIFO
      @(posedge clk); #ApplTime;

      // Read DIMC0 row 5 result (oldest FIFO entry)
      if (out_empty) begin
        $error("[TB] Test13: out_fifo empty — DIMC0 row5 result missing"); test_fail++;
      end else if (out_data !== golden_psout[5]) begin
        $error("[TB] Test13 DIMC0 row5: out_fifo got 0x%08h, expected 0x%08h",
               out_data, golden_psout[5]); test_fail++;
      end
      out_pop = 1'b1; @(posedge clk); #ApplTime; out_pop = 1'b0;

      // Read DIMC1 row 7 result (next FIFO entry)
      if (out_empty) begin
        $error("[TB] Test13: out_fifo empty — DIMC1 row7 result missing"); test_fail++;
      end else if (out_data !== golden_psout[7]) begin
        $error("[TB] Test13 DIMC1 row7: out_fifo got 0x%08h, expected 0x%08h",
               out_data, golden_psout[7]); test_fail++;
      end
      out_pop = 1'b1; @(posedge clk); #ApplTime; out_pop = 1'b0;

      if (test_fail == 0) begin $display("[TB] Test 13: PASS"); pass_count++; end
      else                begin $display("[TB] Test 13: FAIL"); fail_count++; end
    end
`endif // TB_DUAL_TEST13


`ifdef TB_DUAL_TEST14
    // ===============================================================================
    // TEST 14: PIPELINED MATRIX-VECTOR MULTIPLICATION — DIMC 0 - POP all at the end
    // ===============================================================================
    $display("[TB] Test 14: Pipelined MatVec DIMC 0, single phase");
    begin
      automatic int test_fail = 0;
      sel = 1'b0;

      // Drain any stale entries from previous tests
      while (!out_empty) begin
        out_pop = 1'b1; @(posedge clk); #ApplTime;
      end
      out_pop = 1'b0;

      // Reload kernel and feature into DIMC 0
      write_full_kernel_dual(kernel_stim);
      load_feature_dual(feature_stim[0], feature_stim[1], feature_stim[2], feature_stim[3]);

      begin
        automatic int fd;

        for (int i = 0; i <= 32; i++) begin
          if (i < NB_KERNEL_ROWS) begin
            COMPE = 1'b1; MODE = 2'b11; compute_mask = 10'd0;
            RA    = {5'(i), 2'b00}; ADDIN = BIAS;
            RCSN  = 1'b0; RCSN0 = 1'b0; RCSN1 = 1'b0; RCSN2 = 1'b0; RCSN3 = 1'b0;
            WCSN  = 1'b1; WEN   = 1'b1; FCSN  = 1'b1;

          end else begin
            // COMPE/RCSN* deasserted after last row
            COMPE = 1'b0;
            RCSN  = 1'b1; RCSN0 = 1'b1; RCSN1 = 1'b1; RCSN2 = 1'b1; RCSN3 = 1'b1;
          end
          @(posedge clk); #ApplTime;
        end

        // Test -------------------------
        if (out_data != golden_psout[0]) begin
            test_fail = 1;
        end
        out_pop = 1'b1;
          for (int i = 1; i < 32; i++) begin
            @(posedge clk); #TestTime;
            if (out_data != golden_psout[i]) begin
              test_fail = 1;
            end
          end
        out_pop = 1'b0;



        /* for debugging -------------------------
        fd = $fopen("debug/test14_fifo_dump.txt", "w");
        if (fd == 0)
          $fatal(1, "[TB] Could not open debug/test14_fifo_dump.txt");

        $fdisplay(fd, "i= %d out_data=%08h", 0, out_data);
        out_pop = 1'b1;
          for (int i = 1; i < 32; i++) begin
            @(posedge clk); #TestTime;
            $fdisplay(fd, "i= %d out_data=%08h", i, out_data);
          end
        out_pop = 1'b0;

        $fclose(fd);
        $display("[TB] Test 14: FIFO dump written to debug/test14_fifo_dump.txt");
        */
      end

      if (test_fail == 0) begin $display("[TB] Test 14: PASS"); pass_count++; end
      else                begin $display("[TB] Test 14: FAIL (%0d mismatches)", test_fail); fail_count++; end
    end
`endif // TB_DUAL_TEST14



`ifdef TB_DUAL_TEST15
    // ===============================================================================
    // TEST 15: PIPELINED MATRIX-VECTOR MULTIPLICATION — DIMC 0 - POP once ready
    // ===============================================================================
    $display("[TB] Test 15: Pipelined MatVec DIMC 0, single phase, pop once ready");
    begin
      automatic int test_fail = 0;
      automatic int count_correct = 0;
      sel = 1'b0;

      // Drain any stale entries from previous tests
      while (!out_empty) begin
        out_pop = 1'b1; @(posedge clk); #ApplTime;
      end
      out_pop = 1'b0;

      // Reload kernel and feature into DIMC 0
      write_full_kernel_dual(kernel_stim);
      load_feature_dual(feature_stim[0], feature_stim[1], feature_stim[2], feature_stim[3]);

      begin
        automatic int fd;

        for (int i = 0; i <= NB_KERNEL_ROWS + 5; i++) begin
          if (i < NB_KERNEL_ROWS) begin
            // requesting row i
            COMPE = 1'b1; MODE = 2'b11; compute_mask = 10'd0;
            RA    = {5'(i), 2'b00}; ADDIN = BIAS;
            RCSN  = 1'b0; RCSN0 = 1'b0; RCSN1 = 1'b0; RCSN2 = 1'b0; RCSN3 = 1'b0;
            WCSN  = 1'b1; WEN   = 1'b1; FCSN  = 1'b1;
          end else if (i == NB_KERNEL_ROWS) begin
            // COMPE/RCSN* deasserted after last row
            COMPE = 1'b0;
            RCSN  = 1'b1; RCSN0 = 1'b1; RCSN1 = 1'b1; RCSN2 = 1'b1; RCSN3 = 1'b1;
          end

          if (i >= 5 && i <= NB_KERNEL_ROWS + 4) begin
            if (out_empty) begin
              $error("[TB] Test 15: attempted pop when FIFO empty at cycle %0d", i);
              test_fail = 1;
              out_pop = 1'b0;
            end else begin
              out_pop = 1'b1;
              if (out_data != golden_psout[i-5]) begin
                test_fail = 1;
              end else begin
                count_correct ++;
              end
            end
          end else begin
            out_pop = 1'b0;
          end

          @(posedge clk); #ApplTime;

        end





        /* for debugging -------------------------
        fd = $fopen("debug/test14_fifo_dump.txt", "w");
        if (fd == 0)
          $fatal(1, "[TB] Could not open debug/test14_fifo_dump.txt");

        $fdisplay(fd, "i= %d out_data=%08h", 0, out_data);
        out_pop = 1'b1;
          for (int i = 1; i < 32; i++) begin
            @(posedge clk); #TestTime;
            $fdisplay(fd, "i= %d out_data=%08h", i, out_data);
          end
        out_pop = 1'b0;

        $fclose(fd);
        $display("[TB] Test 15: FIFO dump written to debug/test14_fifo_dump.txt");
        */
      end

      if (test_fail == 0) begin $display("[TB] Test 15: PASS"); pass_count++; end
      else                begin $display("[TB] Test 15: FAIL (%0d mismatches)", test_fail); fail_count++; end
    end
`endif // TB_DUAL_TEST15





    // =========================================================================
    // FINAL SUMMARY
    // =========================================================================
    $display("[TB] RESULTS: %0d PASSED, %0d FAILED", pass_count, fail_count);
    if (fail_count == 0) $display("[TB] ALL TESTS PASSED");
    else                  $display("[TB] FAILURES DETECTED");
    $display("Testbench: Test finished.");
    eot = 1'b1;
    $finish;
  end

  // =========================================================================
  // WAVEFORM DUMP
  // =========================================================================
  initial begin
    $dumpfile("sim/tb_DIMC_dual.vcd");
    $dumpvars(0, tb_DIMC_dual);
  end

  // =========================================================================
  // WATCHDOG TIMER
  // =========================================================================
  // 100 µs ceiling
  // A watchdog trip always indicates a bug (DUT stalls or task deadlock).
  initial begin
    #(50000 * ClkPeriod);
    $error("[TB] WATCHDOG: simulation exceeded 100 us");
    $finish;
  end
endmodule
