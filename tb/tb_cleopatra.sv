/*
 * tb_cleopatra.sv
 *
 */

// Comment out a line to skip that test at compile time.
`define TB_CLEOPATRA_TEST1 
`define TB_CLEOPATRA_TEST2   // Repeated accumulation without clearing -- K passes
`define TB_CLEOPATRA_TEST3   // Full matrix multiplication - tiled to fit DIMC matrices size
// ----------------------------------------------------------------------------

`timescale 1ns/1ps

module tb_cleopatra;
  import dimc_package::*;

  // -------------------------------------------------------------------------
  // Parameters
  // -------------------------------------------------------------------------
  // SECTION_WIDTH: each DIMC memory section is 256 bits = 32 bytes.
  

/* ############### SET ############### */
//Full matrix dimensions, made multiples of DIMC dimensions for now
//should match the values in the cleo_test3_stim.py file

  localparam int TEST3_K              = 4;
  localparam int TEST3_L              = 3;
  localparam int TEST3_Q              = 2;




  // Number of numbered kernel and feature stimulus sets to generate.
  localparam int NUM_STIM_SETS = 8;  // Must match stimuli/cleo_tests_1_2_stim.py.

  parameter SECTION_WIDTH  = 256;
  parameter KERNEL_WEIGHTS_FILE       = "stimuli/dimc_tests/kernel_weights.txt";
  parameter FEATURE_VECTOR_8X_FILE    = "stimuli/cleo_test1/feature_vector_8times.txt";
  parameter GOLDEN_OUTPUT_CLEOPATRA_FILE = "stimuli/cleo_test1/golden_output_cleopatra.txt";
  parameter GOLDEN_OUTPUT_CLEOPATRA_TEST2_FILE = "stimuli/cleo_test2/golden_output_cleopatra_test2.txt";
  parameter TEST3_TILED_WEIGHTS_FILE = "stimuli/cleo_test3/test3_tiled_weights.txt";
  parameter TEST3_TILED_INPUTS_FILE = "stimuli/cleo_test3/test3_tiled_inputs.txt";
  parameter TEST3_GOLDEN_OUTPUT_FILE = "stimuli/cleo_test3/test3_golden_matmul_output.txt";
  parameter TEST3_FINAL_OUTPUT_FILE = "stimuli/cleo_test3/test3_final_matmul_output.txt";
  localparam int TEST2_K_START        = 0;  // start index for numbered stimulus sets
  localparam int TEST2_K_END          = NUM_STIM_SETS;  // end index for numbered stimulus sets
  localparam int TEST3_M              = 32;
  localparam int TEST3_N              = 32;
  localparam int TEST3_P              = 8;

  
  localparam int TEST3_NUM_WEIGHT_TILES = TEST3_K * TEST3_L;
  localparam int TEST3_WEIGHT_TILE_BITS = 32 * TEST3_M * TEST3_N;

  localparam int TEST3_NUM_INPUT_TILES = TEST3_L * TEST3_Q;
  localparam int TEST3_INPUT_TILE_BITS = 32 * TEST3_N * TEST3_P;
  
  localparam int TEST3_OUTPUT_ELEMENTS = TEST3_K * TEST3_M * TEST3_Q * TEST3_P;
  localparam int TEST3_FEATURE_BITS = 32 * TEST3_N;
  localparam int TEST3_FEATURE_SECTIONS = TEST3_FEATURE_BITS / SECTION_WIDTH;


  // One packed value represents one complete matrix tile. Kernel tiles are
  // row-major; input tiles are column-major so each feature is contiguous.
  typedef logic [TEST3_WEIGHT_TILE_BITS-1:0] test3_kernel_tile_t;
  // [column][256-bit section][bit within section]. Ascending outer ranges
  // place column 0/section 0 at the most-significant end, matching the file.
  typedef logic
      [0:TEST3_P-1]
      [0:TEST3_FEATURE_SECTIONS-1]
      [SECTION_WIDTH-1:0] test3_input_tile_t;

  // BIAS: 32-bit two's-complement bias constant added to every MAC result.
  localparam logic [31:0] BIAS = 32'hFFE04300;

  // Stimulus arrays (filled by $readmemh at simulation start)
  logic [SECTION_WIDTH-1:0] kernel_stim         [0 : NB_KERNEL_ROWS*4-1]; // 128 sections
  logic [SECTION_WIDTH-1:0] feature_stim_8times [0 : 8*4-1];              // 8 feature vectors x 4 sections
  logic [31:0]              golden_acc_o        [0 : 255];               // 256 accumulator golden values
  logic [31:0]              golden_acc_o_test2_cleo [0 : 255];      // 256 accumulator golden values for Test 2 sum
  test3_kernel_tile_t test3_tiled_weights_flat
      [0 : TEST3_NUM_WEIGHT_TILES-1];
  test3_kernel_tile_t test3_kernel_stim
      [0 : TEST3_K-1][0 : TEST3_L-1];
  test3_input_tile_t test3_tiled_inputs_flat
      [0 : TEST3_NUM_INPUT_TILES-1];
  test3_input_tile_t test3_input_stim
      [0 : TEST3_L-1][0 : TEST3_Q-1];
  logic [31:0] test3_golden_matmul_output
      [0 : TEST3_OUTPUT_ELEMENTS-1];
  logic [31:0] test3_final_matmul_output
      [0 : TEST3_OUTPUT_ELEMENTS-1];

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
  logic count_test3_cycles = 1'b0;
  longint unsigned test3_cycle_count = 0;

  // sel: 0 = u_mac0  1 = u_mac1 (inside spatz_DIMC_dual)
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
  logic [9:0]               compute_mask = '0;
  logic [1:0]               sign_8b = 2'b00;

  // Input feature FIFO interface (driven by this testbench). 
  logic                     inp_push = 1'b0;  // push inp_data when high and not full
  logic [SECTION_WIDTH-1:0] inp_data = '0;    // 256-bit section to enqueue

  // Weight FIFO interface (driven by this testbench)
  logic                     wgt_push = 1'b0;  // push wgt_data when high and not full
  logic [SECTION_WIDTH-1:0] wgt_data = '0;    // 256-bit kernel section to enqueue

  // Accumulator control: clear all accumulators before the test sequence.
  logic               clear = 1'b0;
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
    .COMPE_m0     (sel ? 1'b0 : COMPE),
    .FCSN_m0      (sel ? 1'b1 : FCSN),
    .MODE_m0      (MODE),
    .FA_m0        (FA),
    .ADDIN_m0     (ADDIN),
    .RA_m0        (RA),
    .WA_m0        (WA),
    .RCSN_m0      (sel ? 1'b1 : RCSN),
    .RCSN0_m0     (sel ? 1'b1 : RCSN0),
    .RCSN1_m0     (sel ? 1'b1 : RCSN1),
    .RCSN2_m0     (sel ? 1'b1 : RCSN2),
    .RCSN3_m0     (sel ? 1'b1 : RCSN3),
    .WCSN_m0      (sel ? 1'b1 : WCSN),
    .WEN_m0       (sel ? 1'b1 : WEN),
    .M_m0         (M),
    .compute_mask_m0(compute_mask),
    .sign_8b_m0   (sign_8b),
    .COMPE_m1     (sel ? COMPE : 1'b0),
    .FCSN_m1      (sel ? FCSN : 1'b1),
    .MODE_m1      (MODE),
    .FA_m1        (FA),
    .ADDIN_m1     (ADDIN),
    .RA_m1        (RA),
    .WA_m1        (WA),
    .RCSN_m1      (sel ? RCSN : 1'b1),
    .RCSN0_m1     (sel ? RCSN0 : 1'b1),
    .RCSN1_m1     (sel ? RCSN1 : 1'b1),
    .RCSN2_m1     (sel ? RCSN2 : 1'b1),
    .RCSN3_m1     (sel ? RCSN3 : 1'b1),
    .WCSN_m1      (sel ? WCSN : 1'b1),
    .WEN_m1       (sel ? WEN : 1'b1),
    .M_m1         (M),
    .compute_mask_m1(compute_mask),
    .sign_8b_m1   (sign_8b),
    .inp_push     (inp_push),
    .inp_data     (inp_data),
    .wgt_push     (wgt_push),
    .wgt_data     (wgt_data),
    .clear        (clear),
    .acc_o        (acc_o)
  );

  // CLOCK GENERATION
  initial begin
    clk = 1'b0;
    forever #(ClkPeriod/2) clk = ~clk;
  end

  always @(posedge clk) begin
    if (count_test3_cycles)
      test3_cycle_count <= test3_cycle_count + 1;
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
    clear = 1'b1;

    @(posedge clk);
    #ApplTime;
    clear = 1'b0;
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

  // Convert a packed Test 3 tile into the section-array interface used by
  // write_full_kernel_dual. Section 0 comes from the tile's most-significant
  // bits, matching the Python generator's row-major kernel packing.
  task automatic write_test3_kernel_dual(
    input test3_kernel_tile_t kernel
  );
    logic [SECTION_WIDTH-1:0] kernel_sections
        [0:NB_KERNEL_ROWS*4-1];

    for (int section = 0; section < NB_KERNEL_ROWS*4; section++) begin
      kernel_sections[section] =
          kernel[
              TEST3_WEIGHT_TILE_BITS - section*SECTION_WIDTH - 1
              -: SECTION_WIDTH
          ];
    end

    write_full_kernel_dual(kernel_sections);
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
        COMPE = 1'b1; MODE = 2'b11; compute_mask = 10'd0;
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
    clear = 1'b1;
    @(posedge clk);
    clear = 1'b0;
    @(posedge clk);

    // Load stimulus generated by stimuli/cleo_tests_1_2_stim.py.
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
      clear = 1'b0;


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
        $sformat(kernel_file, "stimuli/cleo_test2/kernel_stim_%0d.txt", k);
        $sformat(feature_file, "stimuli/cleo_test2/feature_stim_8times_%0d.txt", k);

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

      end
      extra_cycles_stall();

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

`ifdef TB_CLEOPATRA_TEST3
    // =======================================================================
    // TEST 3: Full matrix multiplication - tiled to fit DIMC matrices size
    // =======================================================================
    $display("[TB] Test 3: Load tiled weight matrices");
    begin
      automatic int test_fail = 0;
      automatic int accumulator_file;
      automatic int python_status;

      test3_cycle_count = 0;
      count_test3_cycles = 1'b1;
      clear_accumulators();

      // Each file token is one complete M-by-N kernel tile.
      $readmemh(TEST3_TILED_WEIGHTS_FILE, test3_tiled_weights_flat);

      // Tile order is [0,0], [0,1], ... [k-1,l-1].
      for (int k_idx = 0; k_idx < TEST3_K; k_idx++) begin
        for (int l_idx = 0; l_idx < TEST3_L; l_idx++) begin
          test3_kernel_stim[k_idx][l_idx] =
              test3_tiled_weights_flat[k_idx*TEST3_L + l_idx];
        end
      end

      // Each file token is one complete N-by-P input tile.
      $readmemh(TEST3_TILED_INPUTS_FILE, test3_tiled_inputs_flat);

      // Column-major tile order:
      // [0,0], [1,0], ... [l-1,0], [0,1], ... [l-1,q-1].
      for (int l_idx = 0; l_idx < TEST3_L; l_idx++) begin
        for (int q_idx = 0; q_idx < TEST3_Q; q_idx++) begin
          test3_input_stim[l_idx][q_idx] =
              test3_tiled_inputs_flat[q_idx*TEST3_L + l_idx];
        end
      end

      accumulator_file = $fopen(
          "stimuli/cleo_test3/test3_accumulator_output.txt",
          "w"
      );
      if (accumulator_file == 0) begin
        $fatal(1, "[TB] Test 3: failed to open accumulator output file");
      end

      for (int k = 0; k < TEST3_K; k++) begin
        for (int q = 0; q < TEST3_Q; q++) begin

          // This computes one M-by-P output tile.
          for (int l = 0; l < TEST3_L; l++) begin
            write_test3_kernel_dual(test3_kernel_stim[k][l]);
            // this loop is for the p=8 feature vectors in the input matrix
            for (int p = 0; p < TEST3_P; p++) begin
              load_feature_dual(test3_input_stim[l][q][p][0],
                                test3_input_stim[l][q][p][1],
                                test3_input_stim[l][q][p][2],
                                test3_input_stim[l][q][p][3]);
              
              request_full_matvec_dimc0();
            end
          end
          extra_cycles_stall();

          // Append one complete M-by-P output tile in row-major order.
          for (int row = 0; row < TEST3_M; row++) begin
            for (int col = 0; col < TEST3_P; col++) begin
              $fdisplay(
                  accumulator_file,
                  "%08h",
                  acc_o[col*TEST3_M + row]
              );
            end
          end

          clear_accumulators();

        end
      end
      $fclose(accumulator_file);
      count_test3_cycles = 1'b0;
      $display("[TB] Test 3 completed in %0d clock cycles", test3_cycle_count);

      python_status = $system(
          "python3 stimuli/matrix_untiling.py"
      );

      if (python_status != 0) begin
        $error("[TB] Test 3: matrix_untiling.py failed");
        test_fail++;
      end else begin
        $readmemh(TEST3_GOLDEN_OUTPUT_FILE, test3_golden_matmul_output);
        $readmemh(TEST3_FINAL_OUTPUT_FILE, test3_final_matmul_output);

        for (int output_index = 0;
             output_index < TEST3_OUTPUT_ELEMENTS;
             output_index++) begin
          if (test3_final_matmul_output[output_index] !==
              test3_golden_matmul_output[output_index]) begin
            $error(
                "[TB] Test 3 mismatch at %0d: got 0x%08h expected 0x%08h",
                output_index,
                test3_final_matmul_output[output_index],
                test3_golden_matmul_output[output_index]
            );
            test_fail++;
          end
        end
      end

      if (test_fail == 0) begin $display("[TB] Test 3: PASS"); pass_count++; end
      else                begin $display("[TB] Test 3: FAIL"); fail_count++; end
    end

`endif // TB_CLEOPATRA_TEST3

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
