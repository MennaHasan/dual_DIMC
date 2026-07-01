/*
 * tb_DIMC_dual.sv
 *
 * ============================================================
 * PURPOSE
 * ============================================================
 * Testbench for dimc_dual (spatz_DIMC_dual.sv).
 *
 * dimc_dual wraps two DIMC_18_fixed macros behind a shared port set
 * and three FIFOs (weight, input, output).  
 * 
 * PIPELINE LATENCY
 * 
 *   Trigger at posedge P(N) + ApplTime → registered at P(N+1)
 *   P(N+1) — Stage 0: input capture
 *   P(N+2) — Stage 1: MCT masking
 *   P(N+3) — Stage 2: MAC accumulation
 *   P(N+4) — Stage 3: psum + ReLU + quant; READYN goes low
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
 * Test 11 — MCT sweep: DIMC 1, row 0, 6 MCT values; drain and verify out_fifo
 * Test 12 — MCT sweep: DIMC 0, row 0, 6 MCT values; drain and verify out_fifo
 * Test 13 — Overlapping computes: DIMC0 row 5 triggered first; DIMC1 row 7
 *            triggered 3 cycles before DIMC0 finishes; both results read from
 *            out_fifo in order after both operations complete
 * Test 14 — Pipelined MatVec: DIMC 0, all 32 rows, triggers and result
 *            collection interleaved in one loop (36 cycles total)
 */
 


// ── Test enable macros ────────────────────────────────────────────────────
// Comment out a line to skip that test at compile time.

/*
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
`define TB_DUAL_TEST11   // MCT sweep DIMC 1 (requires Tests 2 and 6)
`define TB_DUAL_TEST12   // MCT sweep DIMC 0 (requires Tests 1 and 5)
`define TB_DUAL_TEST13   // Overlapping computes DIMC0 row 5 / DIMC1 row 7

*/



`define TB_DUAL_TEST14   // Pipelined MatVec DIMC 0, single-phase
// ─────────────────────────────────────────────────────────────────────────

`timescale 1ns/1ps

module tb_DIMC_dual;

  // -------------------------------------------------------------------------
  // Parameters
  // -------------------------------------------------------------------------
  // SECTION_WIDTH: each DIMC memory section is 256 bits = 32 bytes.
  parameter SECTION_WIDTH  = 256;
  // NB_KERNEL_ROWS: each DIMC has 32 kernel rows (32 × 128 uint8 elements).
  parameter NB_KERNEL_ROWS = 32;

  parameter KERNEL_WEIGHTS_FILE         = "stimuli/kernel_weights.txt";
  parameter FEATURE_VECTOR_FILE         = "stimuli/feature_vector.txt";
  parameter GOLDEN_MATVEC_FILE          = "stimuli/golden_4bit.txt";
  parameter GOLDEN_PSOUT_FILE           = "stimuli/golden_psum_24bit.txt";
  parameter GOLDEN_DOT_PRODUCT_MCT_FILE = "stimuli/golden_mct_4bit.txt";
  parameter GOLDEN_PSOUT_MCT_FILE       = "stimuli/golden_psum_mct_24bit.txt";

  // NB_MCT_VALS: number of distinct MCT values in the sweep test (Test 11-12).
  parameter NB_MCT_VALS = 6;

  // BIAS: 24-bit signed constant bias added to every MAC result at Stage 3.
  // Must match BIAS in gen_stim.py and tb_DIMC.sv.
  localparam logic signed [23:0] BIAS = -2_080_000;

  // MCT_VALS: six threshold values, each trimming different numbers of active elements.
  // Must match MCT_VALS in gen_stim.py and tb_DIMC.sv.
  localparam logic [7:0] MCT_VALS [NB_MCT_VALS] = '{8'd0, 8'd128, 8'd192, 8'd224, 8'd240, 8'd248};

  // Stimulus and golden arrays (filled by $readmemh at simulation start)
  logic [SECTION_WIDTH-1:0] kernel_stim    [0 : NB_KERNEL_ROWS*4-1]; // 128 sections
  logic [SECTION_WIDTH-1:0] feature_stim   [0 : 3];                   // 4 sections
  logic [7:0]  golden_4bit    [0 : NB_KERNEL_ROWS-1];   // 4-bit results per row (lower nibble)
  logic [23:0] golden_psout     [0 : NB_KERNEL_ROWS-1];   // 24-bit psums per row
  logic [7:0]  golden_mct       [0 : NB_MCT_VALS-1];      // 4-bit results for row 0, one per MCT
  logic [23:0] golden_psout_mct [0 : NB_MCT_VALS-1];      // 24-bit psums for row 0, one per MCT

  // Timing: same as tb_DIMC.sv (100 MHz, 2 ns apply, 8 ns test)
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
  logic [23:0] ADDIN  = '0;     
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
  logic [7:0]               MCT = '0;   // MCT=0: all 128 elements active (no masking)

  // Outputs from the selected DIMC (muxed inside dimc_dual by sel)
  logic                     READYN;    
  logic [SECTION_WIDTH-1:0] Q;         
  logic                     SOUT;      
  logic [2:0]               RES_OUT;   
  logic [23:0]              PSOUT;     

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
  logic [23:0] out_data;           // 24-bit psum at current FIFO head
  logic        out_full;           // DUT output: FIFO is full (should never happen in these tests)
  logic        out_empty;          // DUT output: FIFO is empty (no results ready)

  // Diagnostic: both macros' PSOUT and Q simultaneously (not muxed by sel)
  logic [1:0][23:0]              mac_psout;   
  logic [1:0][SECTION_WIDTH-1:0] mac_q;       

  // End-of-test flag — asserted when simulation finishes
  logic eot = 1'b0;

  // =========================================================================
  // DUT INSTANTIATION
  // =========================================================================
  dimc_dual #(
    .SECTION_WIDTH  (SECTION_WIDTH),
    .NB_KERNEL_ROWS (NB_KERNEL_ROWS)
  ) i_dut (
    .clk      (clk),
    .rst_n    (rst_n),
    .sel      (sel),
    .COMPE    (COMPE),
    .FCSN     (FCSN),
    .MODE     (MODE),
    .FA       (FA),
    .ADDIN    (ADDIN),
    .RA       (RA),
    .WA       (WA),
    .RCSN     (RCSN),
    .RCSN0    (RCSN0),
    .RCSN1    (RCSN1),
    .RCSN2    (RCSN2),
    .RCSN3    (RCSN3),
    .WCSN     (WCSN),
    .WEN      (WEN),
    .M        (M),
    .MCT      (MCT),
    .READYN   (READYN),
    .Q        (Q),
    .SOUT     (SOUT),
    .RES_OUT  (RES_OUT),
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
    .out_empty(out_empty),
    .mac_psout(mac_psout),
    .mac_q    (mac_q)
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
  task automatic write_kernel_dual(
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
    rdata = Q;
    RCSN  = 1'b1;
  endtask

  // load_feature_dual — writes all 4 sections of the feature vector into the
    task automatic load_feature_dual(
    input [SECTION_WIDTH-1:0] f0,   
    input [SECTION_WIDTH-1:0] f1,   
    input [SECTION_WIDTH-1:0] f2,   
    input [SECTION_WIDTH-1:0] f3    
  );
    // --- Push phase: load all four sections into inp_fifo ---
    @(posedge clk); #ApplTime; inp_push = 1'b1; inp_data = f0;   
    @(posedge clk); #ApplTime; inp_data = f1;                     
    @(posedge clk); #ApplTime; inp_data = f2;                     
    @(posedge clk); #ApplTime; inp_data = f3;                     
    @(posedge clk); #ApplTime; inp_push = 1'b0;                   

    // --- Load phase: FD = inp_fifo head; one section loaded + popped per cycle ---
    FCSN = 1'b0; FA = 2'd0;                       // cycle A: feature_buf[0] ← f0 at posedge
    @(posedge clk); #ApplTime; FA = 2'd1;          // cycle B: feature_buf[1] ← f1
    @(posedge clk); #ApplTime; FA = 2'd2;          // cycle C: feature_buf[2] ← f2
    @(posedge clk); #ApplTime; FA = 2'd3;          // cycle D: feature_buf[3] ← f3
    @(posedge clk); #ApplTime; FCSN = 1'b1; FA = '0;   // deassert; buffer retains all values
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
    input  [23:0] bias,     
    input  [7:0]  mct_val,  
    output [23:0] psout,    
    output [3:0]  quant     
  );
    // --- Cycle N: assert compute trigger for exactly ONE cycle ---
    @(posedge clk); #ApplTime;
    COMPE = 1'b1; MODE = 2'b11; MCT = mct_val;
    RA = {row, 2'b00}; ADDIN = bias;   // RA section bits ignored in compute mode
    RCSN = 1'b0; RCSN0 = 1'b0; RCSN1 = 1'b0; RCSN2 = 1'b0; RCSN3 = 1'b0;
    WCSN = 1'b1; WEN = 1'b1; FCSN = 1'b1;   // write/feature paths idle

    // --- Cycle N+1: deassert trigger; Stage 0 captures inputs ---
    @(posedge clk); #ApplTime;
    COMPE = 1'b0;
    RCSN = 1'b1; RCSN0 = 1'b1; RCSN1 = 1'b1; RCSN2 = 1'b1; RCSN3 = 1'b1;

    // --- Cycles N+2, N+3: pipeline advances through Stages 1 and 2 ---
    @(posedge clk);   // Stage 1: MCT masking
    @(posedge clk);   // Stage 2: MAC accumulation

    // --- Cycle N+4: Stage 3 completes; READYN goes low ---
    @(posedge clk); #TestTime;
    if (READYN !== 1'b0)
      $error("[TB] READYN did not go low after 4-cycle pipeline (row=%0d, sel=%0d)", row, sel);
    psout = PSOUT;
    quant = {RES_OUT, SOUT};   // pack 4-bit result
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
    logic [23:0]              psout;
    logic [3:0]               quant;

    // Wait for reset release, then one idle cycle for DUT outputs to settle
    @(posedge rst_n);
    @(posedge clk);

    // Load all stimulus and golden data from files generated by gen_stim.py.
    $readmemh(KERNEL_WEIGHTS_FILE,         kernel_stim);     // 128 sections: 32 rows × 4
    $readmemh(FEATURE_VECTOR_FILE,         feature_stim);    //   4 sections: 1024-bit vector
    $readmemh(GOLDEN_MATVEC_FILE,          golden_4bit);   //  32 entries: 4-bit results per row
    $readmemh(GOLDEN_PSOUT_FILE,           golden_psout);    //  32 entries: 24-bit psums per row
    $readmemh(GOLDEN_DOT_PRODUCT_MCT_FILE, golden_mct);      //   6 entries: 4-bit result for row 0 per MCT
    $readmemh(GOLDEN_PSOUT_MCT_FILE,       golden_psout_mct);//   6 entries: 24-bit psum for row 0 per MCT

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
      if (READYN !== 1'b1)         reset_ok = 1'b0;   // not-ready on reset
      if (PSOUT  !== 24'h0)        reset_ok = 1'b0;   // psum register cleared
      if ({RES_OUT,SOUT} !== 4'h0) reset_ok = 1'b0;   // quant output cleared
      if (Q !== '0)                reset_ok = 1'b0;   // kernel readback cleared

      // Check DIMC 1 via sel mux
      sel = 1'b1;
      @(posedge clk); #TestTime;
      if (READYN !== 1'b1)         reset_ok = 1'b0;
      if (PSOUT  !== 24'h0)        reset_ok = 1'b0;
      if ({RES_OUT,SOUT} !== 4'h0) reset_ok = 1'b0;
      if (Q !== '0)                reset_ok = 1'b0;
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
    for (int r = 0; r < NB_KERNEL_ROWS; r++)
      for (int s = 0; s < 4; s++)
        write_kernel_dual(5'(r), 2'(s), kernel_stim[r*4 + s]);
    $display("[TB] Test 1: DONE (verified in Test 3)");
`endif // TB_DUAL_TEST1

`ifdef TB_DUAL_TEST2
    // =======================================================================
    // TEST 2: KERNEL WRITE — DIMC 1
    // =======================================================================
    $display("[TB] Test 2: Kernel write DIMC 1");
    sel = 1'b1;   // target DIMC 1
    for (int r = 0; r < NB_KERNEL_ROWS; r++)
      for (int s = 0; s < 4; s++)
        write_kernel_dual(5'(r), 2'(s), kernel_stim[r*4 + s]);
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
      compute_and_capture_dual(5'd1, BIAS, 8'd0, psout, quant);
      // Verify direct DIMC output ports
      if (psout !== golden_psout[1] || quant !== golden_4bit[1][3:0]) begin
        if (psout !== golden_psout[1])
          $error("[TB] Test5 DIMC0 row1: psout got 0x%06h, expected 0x%06h", psout, golden_psout[1]);
        if (quant !== golden_4bit[1][3:0])
          $error("[TB] Test5 DIMC0 row1: quant got %0d, expected %0d", quant, golden_4bit[1][3:0]);
        test_fail++;
      end
      // Wait one cycle: out_push fires at P(N+4); out_fifo registers at P(N+5).
      // Without this wait, out_empty may still be high immediately after the task returns.
      @(posedge clk); #ApplTime;
      if (out_empty) begin
        $error("[TB] Test5: out_fifo empty — push did not fire"); test_fail++;
      end else if (out_data !== golden_psout[1]) begin
        $error("[TB] Test5: out_fifo got 0x%06h, expected 0x%06h", out_data, golden_psout[1]); test_fail++;
      end
      // Pop to clear the result from the FIFO
      out_pop = 1'b1; @(posedge clk); #ApplTime; out_pop = 1'b0;
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
      compute_and_capture_dual(5'd1, BIAS, 8'd0, psout, quant);
      if (psout !== golden_psout[1] || quant !== golden_4bit[1][3:0]) begin
        if (psout !== golden_psout[1])
          $error("[TB] Test6 DIMC1 row1: psout got 0x%06h, expected 0x%06h", psout, golden_psout[1]);
        if (quant !== golden_4bit[1][3:0])
          $error("[TB] Test6 DIMC1 row1: quant got %0d, expected %0d", quant, golden_4bit[1][3:0]);
        test_fail++;
      end
      @(posedge clk); #ApplTime;   // wait for out_fifo push to register
      if (out_empty) begin
        $error("[TB] Test6: out_fifo empty — push did not fire"); test_fail++;
      end else if (out_data !== golden_psout[1]) begin
        $error("[TB] Test6: out_fifo got 0x%06h, expected 0x%06h", out_data, golden_psout[1]); test_fail++;
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
      compute_and_capture_dual(5'd4, BIAS, 8'd0, psout, quant);
      if (psout !== golden_psout[4] || quant !== golden_4bit[4][3:0]) begin
        if (psout !== golden_psout[4])
          $error("[TB] Test7 DIMC0 row4: psout got 0x%06h, expected 0x%06h", psout, golden_psout[4]);
        if (quant !== golden_4bit[4][3:0])
          $error("[TB] Test7 DIMC0 row4: quant got %0d, expected %0d", quant, golden_4bit[4][3:0]);
        test_fail++;
      end
      @(posedge clk); #ApplTime;   // wait for out_fifo push to register
      if (out_empty) begin
        $error("[TB] Test7: out_fifo empty — push did not fire"); test_fail++;
      end else if (out_data !== golden_psout[4]) begin
        $error("[TB] Test7: out_fifo got 0x%06h, expected 0x%06h", out_data, golden_psout[4]); test_fail++;
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
      compute_and_capture_dual(5'd4, BIAS, 8'd0, psout, quant);
      if (psout !== golden_psout[4] || quant !== golden_4bit[4][3:0]) begin
        if (psout !== golden_psout[4])
          $error("[TB] Test8 DIMC1 row4: psout got 0x%06h, expected 0x%06h", psout, golden_psout[4]);
        if (quant !== golden_4bit[4][3:0])
          $error("[TB] Test8 DIMC1 row4: quant got %0d, expected %0d", quant, golden_4bit[4][3:0]);
        test_fail++;
      end
      @(posedge clk); #ApplTime;
      if (out_empty) begin
        $error("[TB] Test8: out_fifo empty — push did not fire"); test_fail++;
      end else if (out_data !== golden_psout[4]) begin
        $error("[TB] Test8: out_fifo got 0x%06h, expected 0x%06h", out_data, golden_psout[4]); test_fail++;
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
        compute_and_capture_dual(5'(r), BIAS, 8'd0, psout, quant);
        // Check direct outputs from each compute immediately
        if (psout !== golden_psout[r] || quant !== golden_4bit[r][3:0]) begin
          if (psout !== golden_psout[r])
            $error("[TB] Test9 DIMC1 row%0d: psout got 0x%06h, expected 0x%06h",
                   r, psout, golden_psout[r]);
          if (quant !== golden_4bit[r][3:0])
            $error("[TB] Test9 DIMC1 row%0d: quant got %0d, expected %0d",
                   r, quant, golden_4bit[r][3:0]);
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
          $error("[TB] Test9 out_fifo row%0d: got 0x%06h, expected 0x%06h",
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
        compute_and_capture_dual(5'(r), BIAS, 8'd0, psout, quant);
        if (psout !== golden_psout[r] || quant !== golden_4bit[r][3:0]) begin
          if (psout !== golden_psout[r])
            $error("[TB] Test10 DIMC0 row%0d: psout got 0x%06h, expected 0x%06h",
                   r, psout, golden_psout[r]);
          if (quant !== golden_4bit[r][3:0])
            $error("[TB] Test10 DIMC0 row%0d: quant got %0d, expected %0d",
                   r, quant, golden_4bit[r][3:0]);
          test_fail++;
        end
      end
      @(posedge clk); #ApplTime;   // wait for last push to register
      for (int r = 0; r < NB_KERNEL_ROWS; r++) begin
        if (out_empty) begin
          $error("[TB] Test10 out_fifo empty at row%0d — push did not fire", r); test_fail++;
        end else if (out_data !== golden_psout[r]) begin
          $error("[TB] Test10 out_fifo row%0d: got 0x%06h, expected 0x%06h",
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
    // TEST 11: MCT MASKING SWEEP — DIMC 1 (row 0, 6 MCT values)
    // =======================================================================
    // Sweeps 6 MCT values on DIMC 1's row 0.
    // The 6 results accumulate
    // in out_fifo and are drained in a bulk drain after the loop.

    $display("[TB] Test 11: MCT masking sweep, DIMC 1 (%0d values, row 0)", NB_MCT_VALS);
    begin
      automatic int test_fail = 0;
      sel = 1'b1;
      for (int m = 0; m < NB_MCT_VALS; m++) begin
        compute_and_capture_dual(5'd0, BIAS, MCT_VALS[m], psout, quant);
        if (psout !== golden_psout_mct[m] || quant !== golden_mct[m][3:0]) begin
          if (psout !== golden_psout_mct[m])
            $error("[TB] Test11 DIMC1 MCT=0x%02h: psout got 0x%06h, expected 0x%06h",
                   MCT_VALS[m], psout, golden_psout_mct[m]);
          if (quant !== golden_mct[m][3:0])
            $error("[TB] Test11 DIMC1 MCT=0x%02h: quant got %0d, expected %0d",
                   MCT_VALS[m], quant, golden_mct[m][3:0]);
          test_fail++;
        end
      end
      @(posedge clk); #ApplTime;   // wait for last push to register
      for (int m = 0; m < NB_MCT_VALS; m++) begin
        if (out_empty) begin
          $error("[TB] Test11 out_fifo empty at MCT index%0d — push did not fire", m); test_fail++;
        end else if (out_data !== golden_psout_mct[m]) begin
          $error("[TB] Test11 out_fifo MCT=0x%02h: got 0x%06h, expected 0x%06h",
                 MCT_VALS[m], out_data, golden_psout_mct[m]); test_fail++;
        end
        out_pop = 1'b1; @(posedge clk); #ApplTime; out_pop = 1'b0;
      end
      if (test_fail == 0) begin $display("[TB] Test 11: PASS"); pass_count++; end
      else                begin $display("[TB] Test 11: FAIL (%0d mismatches)", test_fail); fail_count++; end
    end
`endif // TB_DUAL_TEST11

`ifdef TB_DUAL_TEST12
    // =======================================================================
    // TEST 12: MCT MASKING SWEEP — DIMC 0 (row 0, 6 MCT values)
    // =======================================================================
    $display("[TB] Test 12: MCT masking sweep, DIMC 0 (%0d values, row 0)", NB_MCT_VALS);
    begin
      automatic int test_fail = 0;
      sel = 1'b0;
      for (int m = 0; m < NB_MCT_VALS; m++) begin
        compute_and_capture_dual(5'd0, BIAS, MCT_VALS[m], psout, quant);
        if (psout !== golden_psout_mct[m] || quant !== golden_mct[m][3:0]) begin
          if (psout !== golden_psout_mct[m])
            $error("[TB] Test12 DIMC0 MCT=0x%02h: psout got 0x%06h, expected 0x%06h",
                   MCT_VALS[m], psout, golden_psout_mct[m]);
          if (quant !== golden_mct[m][3:0])
            $error("[TB] Test12 DIMC0 MCT=0x%02h: quant got %0d, expected %0d",
                   MCT_VALS[m], quant, golden_mct[m][3:0]);
          test_fail++;
        end
      end
      @(posedge clk); #ApplTime;
      for (int m = 0; m < NB_MCT_VALS; m++) begin
        if (out_empty) begin
          $error("[TB] Test12 out_fifo empty at MCT index%0d — push did not fire", m); test_fail++;
        end else if (out_data !== golden_psout_mct[m]) begin
          $error("[TB] Test12 out_fifo MCT=0x%02h: got 0x%06h, expected 0x%06h",
                 MCT_VALS[m], out_data, golden_psout_mct[m]); test_fail++;
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
      COMPE = 1'b1; MODE = 2'b11; MCT = 8'd0;
      RA    = {5'd5, 2'b00}; ADDIN = BIAS;
      RCSN  = 1'b0; RCSN0 = 1'b0; RCSN1 = 1'b0; RCSN2 = 1'b0; RCSN3 = 1'b0;
      WCSN  = 1'b1; WEN   = 1'b1; FCSN  = 1'b1;

      // P(N+1): DIMC0 Stage0 latches; switch sel=1 and trigger DIMC1 row 7.
      // This is 3 cycles before DIMC0 Stage3 — the precise overlap requested.
      @(posedge clk); #ApplTime;
      sel   = 1'b1;
      COMPE = 1'b1;
      RA    = {5'd7, 2'b00};   // row 7 for DIMC1; ADDIN/MCT/MODE unchanged

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
        $error("[TB] Test13 DIMC0 row5: out_fifo got 0x%06h, expected 0x%06h",
               out_data, golden_psout[5]); test_fail++;
      end
      out_pop = 1'b1; @(posedge clk); #ApplTime; out_pop = 1'b0;

      // Read DIMC1 row 7 result (next FIFO entry)
      if (out_empty) begin
        $error("[TB] Test13: out_fifo empty — DIMC1 row7 result missing"); test_fail++;
      end else if (out_data !== golden_psout[7]) begin
        $error("[TB] Test13 DIMC1 row7: out_fifo got 0x%06h, expected 0x%06h",
               out_data, golden_psout[7]); test_fail++;
      end
      out_pop = 1'b1; @(posedge clk); #ApplTime; out_pop = 1'b0;

      if (test_fail == 0) begin $display("[TB] Test 13: PASS"); pass_count++; end
      else                begin $display("[TB] Test 13: FAIL"); fail_count++; end
    end
`endif // TB_DUAL_TEST13


`ifdef TB_DUAL_TEST14
    // =======================================================================
    // TEST 14: PIPELINED MATRIX-VECTOR MULTIPLICATION — DIMC 0, SINGLE PHASE
    // =======================================================================
    // Phase 1: fire all 32 row triggers back-to-back (fully pipelined).
    // Phase 2: wait for the pipeline to drain, then pop and verify all 32
    //          results from out_fifo against golden_psout.
    // =======================================================================
    $display("[TB] Test 14: Pipelined MatVec DIMC 0, single phase");
    begin
      automatic int test_fail = 0;
      sel = 1'b1;

      // Drain any stale entries from previous tests
      while (!out_empty) begin
        out_pop = 1'b1; @(posedge clk); #ApplTime;
      end
      out_pop = 1'b0;

      // Reload kernel and feature into DIMC 0
      for (int r = 0; r < NB_KERNEL_ROWS; r++)
        for (int s = 0; s < 4; s++)
          write_kernel_dual(5'(r), 2'(s), kernel_stim[r*4 + s]);
      load_feature_dual(feature_stim[0], feature_stim[1], feature_stim[2], feature_stim[3]);

      begin
        automatic int fd = $fopen("debug/test14_fifo_dump.txt", "w");


        @(posedge clk); #ApplTime;      // N
      // request for dot product of row i happen here @ N
          COMPE = 1'b1; MODE = 2'b11; MCT = 8'd0;
          RA    = {5'(0), 2'b00}; ADDIN = BIAS;
          RCSN  = 1'b0; RCSN0 = 1'b0; RCSN1 = 1'b0; RCSN2 = 1'b0; RCSN3 = 1'b0;
          WCSN  = 1'b1; WEN   = 1'b1; FCSN  = 1'b1;
        @(posedge clk);                 // N+1
        @(posedge clk); #ApplTime;      // N+2
        
        $fdisplay(fd, "clk cycle N+%0d out_data=%06h  out_empty=%0b  READYN=%0b  PSOUT=%06h", 2, out_data, out_empty, READYN, PSOUT);
        for (int i = 1; i < NB_KERNEL_ROWS+2; i++) begin
          // request for dot product of row i happen here @ N+1+i
          // request for dot product of row 1 happen here @ N+2

          if (i < NB_KERNEL_ROWS) begin
          COMPE = 1'b1; MODE = 2'b11; MCT = 8'd0;
          RA    = {5'(i), 2'b00}; ADDIN = BIAS;
          RCSN  = 1'b0; RCSN0 = 1'b0; RCSN1 = 1'b0; RCSN2 = 1'b0; RCSN3 = 1'b0;
          WCSN  = 1'b1; WEN   = 1'b1; FCSN  = 1'b1;
          
          end
          @(posedge clk); #TestTime;// @ i=1 ---> N+i+2

          // for debugging
          $fdisplay(fd, "clk cycle N+%0d out_data=%06h  out_empty=%0b  READYN=%0b  PSOUT=%06h", i+2, out_data, out_empty, READYN, PSOUT);
          if (i >= 3) begin   // @ N+5 till N+36
            if (out_data !== golden_psout[i-3]) begin
              $error("[TB] Test14 DIMC0 row%0d: out_fifo got 0x%06h, expected 0x%06h",
                     i-3, out_data, golden_psout[i-3]);
              test_fail++;
            end
          end
          if (i == 3) begin     // @ N+5
            out_pop = 1'b1;
            $fdisplay(fd, "out pop now = 1");   // for debugging
          end
          
        end
        out_pop = 1'b0;
        $fclose(fd);
        // for debugging
        $display("[TB] Test 14: FIFO dump written to debug/test14_fifo_dump.txt");
      end

      if (test_fail == 0) begin $display("[TB] Test 14: PASS"); pass_count++; end
      else                begin $display("[TB] Test 14: FAIL (%0d mismatches)", test_fail); fail_count++; end
    end
`endif // TB_DUAL_TEST14





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
