/*
 * tb_cleopatra.sv
 *
 */

// Comment out a line to skip that test at compile time.
`define TB_CLEOPATRA_TEST1 
`define TB_CLEOPATRA_TEST2   // Repeated accumulation without clearing -- K passes
// ----------------------------------------------------------------------------

`timescale 1ns/1ps

module tb_cleopatra;
  import dimc_package::*;

  // -------------------------------------------------------------------------
  // Parameters
  // -------------------------------------------------------------------------
  // SECTION_WIDTH: each DIMC memory section is 256 bits = 32 bytes.
  
  // Number of numbered kernel and feature stimulus sets to generate.
  localparam int NUM_STIM_SETS = 8;  // has to be the same as value in stimuli/generate_stim.py

  parameter SECTION_WIDTH  = 256;
  parameter KERNEL_WEIGHTS_FILE       = "stimuli/kernel_weights.txt";
  parameter FEATURE_VECTOR_8X_FILE    = "stimuli/feature_vector_8times.txt";
  parameter GOLDEN_OUTPUT_CLEOPATRA_FILE = "stimuli/golden_output_cleopatra.txt";
  parameter GOLDEN_OUTPUT_CLEOPATRA_TEST2_FILE = "stimuli/golden_output_cleopatra_test2.txt";
  localparam int TEST2_K_START        = 0;  // start index for numbered stimulus sets
  localparam int TEST2_K_END          = NUM_STIM_SETS;  // end index for numbered stimulus sets

  // BIAS: 24-bit unsigned two's-complement bias constant added to every MAC result.
  localparam logic [23:0] BIAS = 24'hE04300;

  // Stimulus arrays (filled by $readmemh at simulation start)
  logic [SECTION_WIDTH-1:0] kernel_stim         [0 : NB_KERNEL_ROWS*4-1]; // 128 sections
  logic [SECTION_WIDTH-1:0] feature_stim_8times [0 : 8*4-1];              // 8 feature vectors x 4 sections
  logic [31:0]              golden_acc_o        [0 : 255];               // 256 accumulator golden values
  logic [31:0]              golden_acc_o_test2_cleo [0 : 255];      // 256 accumulator golden values for Test 2 sum

  // Timing: same as tb_DIMC_dual.sv (100 MHz, 2 ns apply, 8 ns test)
  localparam time ClkPeriod = 10ns;
  localparam time ApplTime  =  2ns;
  localparam time TestTime  =  8ns;

  // =========================================================================
  // DUT SIGNAL DECLARATIONS
  // =========================================================================

  // Clock and reset
  logic clk;
  logic rst_n;

  // sel: 0 = u_mac0  1 = u_mac1 (inside spatz_DIMC_dual)
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

  // Input feature FIFO interface (driven by this testbench). 
  logic                     inp_push = 1'b0;  // push inp_data when high and not full
  logic [SECTION_WIDTH-1:0] inp_data = '0;    // 256-bit section to enqueue

  // Weight FIFO interface (driven by this testbench)
  logic                     wgt_push = 1'b0;  // push wgt_data when high and not full
  logic [SECTION_WIDTH-1:0] wgt_data = '0;    // 256-bit kernel section to enqueue

  // Accumulator control: clear all accumulators before the test sequence.
  logic               acc_clear_i  = 1'b0;
  logic [31:0]        acc_o [0:255];

  // End-of-test flag - asserted when simulation finishes
  logic eot = 1'b0;

  // =========================================================================
  // DUT INSTANTIATION
  // =========================================================================
  cleopatra #(
    .SECTION_WIDTH (SECTION_WIDTH)
  ) i_dut (
    .clk          (clk),
    .rst_n        (rst_n),
    .sel          (sel),
    .COMPE        (COMPE),
    .FCSN         (FCSN),
    .MODE         (MODE),
    .FA           (FA),
    .ADDIN        (ADDIN),
    .RA           (RA),
    .WA           (WA),
    .RCSN         (RCSN),
    .RCSN0        (RCSN0),
    .RCSN1        (RCSN1),
    .RCSN2        (RCSN2),
    .RCSN3        (RCSN3),
    .WCSN         (WCSN),
    .WEN          (WEN),
    .M            (M),
    .MCT          (MCT),
    .inp_push     (inp_push),
    .inp_data     (inp_data),
    .wgt_push     (wgt_push),
    .wgt_data     (wgt_data),
    .acc_clear_i  (acc_clear_i),
    .acc_o        (acc_o)
  );

  // CLOCK GENERATION
  initial begin
    clk = 1'b0;
    forever #(ClkPeriod/2) clk = ~clk;
  end

  // RESET GENERATION
  initial begin
    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    #ApplTime;
    rst_n = 1'b1;
  end

  task automatic clear_accumulators();
    @(posedge clk);
    #ApplTime;
    acc_clear_i = 1'b1;

    @(posedge clk);
    #ApplTime;
    acc_clear_i = 1'b0;
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

  // load_feature_dual - writes all 4 sections of the feature vector into the feature buffer. 
  
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

  // =========================================================================
  // PASS / FAIL COUNTERS
  // =========================================================================
  int pass_count = 0;
  int fail_count = 0;

  task automatic extra_cycles_stall();
    for (int i = 0; i < 5; i++) begin
      @(posedge clk); #TestTime;
    end
  endtask

  task automatic request_full_matvec_dimc0(
  );

    for (int i = 0; i < NB_KERNEL_ROWS ; i++) begin
        COMPE = 1'b1; MODE = 2'b11; MCT = 8'd0;
        RA    = {5'(i), 2'b00}; ADDIN = BIAS;
        RCSN  = 1'b0; RCSN0 = 1'b0; RCSN1 = 1'b0; RCSN2 = 1'b0; RCSN3 = 1'b0;
        WCSN  = 1'b1; WEN   = 1'b1; FCSN  = 1'b1;
        @(posedge clk); #TestTime; 

      end


      // COMPE/RCSN* deasserted after last row
      COMPE = 1'b0;
      RCSN  = 1'b1; RCSN0 = 1'b1; RCSN1 = 1'b1; RCSN2 = 1'b1; RCSN3 = 1'b1;

      @(posedge clk); #TestTime; 
  endtask

  // =========================================================================
  // MAIN TEST SEQUENCE
  // =========================================================================
  initial begin
    // Wait for reset release, then clear the accumulators once before starting.
    @(posedge rst_n);
    @(posedge clk);
    acc_clear_i = 1'b1;
    @(posedge clk);
    acc_clear_i = 1'b0;
    @(posedge clk);

    // Load stimulus generated by stimuli/generate_stim.py.
    $readmemh(KERNEL_WEIGHTS_FILE,           kernel_stim);          // 128 sections: 32 rows x 4
    $readmemh(FEATURE_VECTOR_8X_FILE,        feature_stim_8times);  //  32 sections: 8 vectors x 4
    $readmemh(GOLDEN_OUTPUT_CLEOPATRA_FILE,  golden_acc_o);         // 256 accumulator golden values
    $readmemh(GOLDEN_OUTPUT_CLEOPATRA_TEST2_FILE, golden_acc_o_test2_cleo); // 256 accumulator golden values for Test 2 sum


`ifdef TB_CLEOPATRA_TEST1
    // =======================================================================
    // TEST 1: Testing Cleopatra outpput with p=8
    // =======================================================================
    $display("[TB] Test 1: Testing Cleopatra outpput with p=8");
    begin
      automatic int test_fail = 0;
      acc_clear_i = 1'b0;


      write_full_kernel_dual(kernel_stim);
      for (int p = 0; p < 8; p++) begin
        // Use p different feature vectors  
        load_feature_dual(feature_stim_8times[4*p],
                          feature_stim_8times[4*p+1],
                          feature_stim_8times[4*p+2],
                          feature_stim_8times[4*p+3]);
        request_full_matvec_dimc0();
      end
      extra_cycles_stall();

      for (int i = 0; i < 256; i++) begin
        if (acc_o[i] !== golden_acc_o[i]) begin
          /*$display("[TB] MISMATCH at accumulator %0d: got 0x%08h expected 0x%08h",
                   i, acc_o[i], golden_acc_o[i]);*/
          test_fail = 1;
        end else begin
          /*$display("[TB] MATCH at accumulator %0d: got 0x%08h expected 0x%08h",
                   i, acc_o[i], golden_acc_o[i]);*/
        end
      end

      $display("[TB] Test 1: completed");

      if (test_fail == 0) begin $display("[TB] Test 1: PASS"); pass_count++; end
      else                begin $display("[TB] Test 1: FAIL"); fail_count++; end
    end
`endif // TB_CLEOPATRA_TEST1


`ifdef TB_CLEOPATRA_TEST2
    // =======================================================================
    // TEST 2: REPEATED ACCUMULATION WITHOUT CLEARING -- K passes
    // =======================================================================
    $display("[TB] Test 2: REPEATED ACCUMULATION WITHOUT CLEARING -- K passes");
    begin
      automatic int test_fail = 0;
      automatic int k;
      automatic string feature_file;
      automatic string kernel_file;

      // clear accumulators
      clear_accumulators();

      for (k = TEST2_K_START; k < TEST2_K_END; k++) begin
        // reading txt file names into the string variables for this iteration 
        $sformat(kernel_file, "stimuli/kernel_stim_%0d.txt", k);
        $sformat(feature_file, "stimuli/feature_stim_8times_%0d.txt", k);

        // reading txt files into the arrays 
        $readmemh(kernel_file, kernel_stim);
        $readmemh(feature_file, feature_stim_8times);

        write_full_kernel_dual(kernel_stim);
        for (int p = 0; p < 8; p++) begin
          load_feature_dual(feature_stim_8times[4*p],
                            feature_stim_8times[4*p+1],
                            feature_stim_8times[4*p+2],
                            feature_stim_8times[4*p+3]);
          request_full_matvec_dimc0();
        end
        extra_cycles_stall();

      end

      for (int i = 0; i < 256; i++) begin
        if (acc_o[i] !== golden_acc_o_test2_cleo[i]) begin
          /*$display("[TB] MISMATCH at accumulator %0d: got 0x%08h expected 0x%08h",
                   i, acc_o[i], golden_acc_o_test2_cleo[i]);*/
          test_fail = 1;
        end else begin
          /*$display("[TB] MATCH at accumulator %0d: got 0x%08h expected 0x%08h",
                   i, acc_o[i], golden_acc_o_test2_cleo[i]);*/
        end
      end

      $display("[TB] Test 2: completed");

      if (test_fail == 0) begin $display("[TB] Test 2: PASS"); pass_count++; end
      else                begin $display("[TB] Test 2: FAIL"); fail_count++; end
    end

`endif // TB_CLEOPATRA_TEST2


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
    $dumpfile("sim/tb_cleopatra.vcd");
    $dumpvars(0, tb_cleopatra);
  end



endmodule // tb_cleopatra
