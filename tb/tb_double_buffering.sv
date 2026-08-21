/*
 * tb_double_buffering.sv
 *
 * Test 1 exercises the independent DIMC controls as a double buffer:
 *   1. Load a complete kernel and feature vector into macro 0.
 *   2. Alternate compute/load operations between macros until the configured
 *      number of MatVec requests has been reached.
 *   3. Finish by computing the last loaded macro without another buffer load.
 *   4. Check every result stream and newly loaded buffer.
 */
// Comment out a line to skip that test at compile time.
`define TB_DOUBLE_BUFFERING_TEST1 

`timescale 1ns/1ps

module tb_double_buffering;
  import dimc_package::*;

  parameter int SECTION_WIDTH = 256;
  parameter KERNEL_STIM_FILE =
      "stimuli/double_buffering/double_buffering_kernel_stim.txt";
  parameter FEATURE_STIM_FILE =
      "stimuli/double_buffering/double_buffering_feature_stim.txt";
  parameter TILED_WEIGHTS_FILE =
      "stimuli/double_buffering/double_buffering_tiled_weights.txt";
  parameter TILED_INPUTS_FILE =
      "stimuli/double_buffering/double_buffering_tiled_inputs.txt";
  parameter GOLDEN_MATMUL_FILE =
      "stimuli/double_buffering/double_buffering_golden_matmul_output.txt";
  parameter ACCUMULATOR_OUTPUT_FILE =
      "stimuli/double_buffering/double_buffering_accumulator_output.txt";
  parameter FINAL_MATMUL_FILE =
      "stimuli/double_buffering/double_buffering_final_matmul_output.txt";

  // Full matrix dimensions match the tiled Cleopatra Test 3 convention:
  //   weights = (K*M) x (L*N_ELEMENTS)
  //   inputs  = (L*N_ELEMENTS) x (Q*P)
  localparam int DB_K = 4;
  localparam int DB_L = 3;
  localparam int DB_Q = 2;

// these shouldn't change
  localparam int DB_M = NB_KERNEL_ROWS;
  localparam int DB_N_BITS = 1024;
  localparam int DB_P = 8;
  localparam int DB_ELEMENT_BITS = 8;
  localparam int DB_N_ELEMENTS = DB_N_BITS / DB_ELEMENT_BITS;


  localparam int DB_NUM_WEIGHT_TILES = DB_K * DB_L;
  localparam int DB_WEIGHT_TILE_BITS = DB_M * DB_N_BITS;

  localparam int DB_NUM_INPUT_TILES = DB_L * DB_Q;
  localparam int DB_INPUT_TILE_BITS = DB_N_BITS * DB_P;

  localparam int DB_OUTPUT_ELEMENTS = DB_K * DB_M * DB_Q * DB_P;
  localparam int DB_KERNEL_ELEMENTS = DB_K * DB_M * DB_L * DB_N_ELEMENTS;
  localparam int DB_FEATURE_ELEMENTS = DB_L * DB_N_ELEMENTS * DB_Q * DB_P;

  localparam int NUM_SECTIONS = 1024 / SECTION_WIDTH;
  localparam logic [31:0] BIAS = 32'hFFE04300;



  typedef logic [DB_WEIGHT_TILE_BITS-1:0] db_kernel_tile_t;
  typedef logic
      [0:DB_P-1]
      [0:NUM_SECTIONS-1]
      [SECTION_WIDTH-1:0] db_input_tile_t;

  // $readmemh targets are flat because the files contain one packed tile per
  // token.  Coordinate-indexed views are populated after the files are read.
  db_kernel_tile_t tiled_weights_flat [0:DB_NUM_WEIGHT_TILES-1];
  db_input_tile_t  tiled_inputs_flat  [0:DB_NUM_INPUT_TILES-1];
  logic [DB_ELEMENT_BITS-1:0] kernel_stim_flat [0:DB_KERNEL_ELEMENTS-1];
  logic [DB_ELEMENT_BITS-1:0] feature_stim_flat[0:DB_FEATURE_ELEMENTS-1];
  db_kernel_tile_t kernel_stim [0:DB_K-1][0:DB_L-1];
  db_input_tile_t  feature_stim[0:DB_L-1][0:DB_Q-1];
  logic [31:0] golden_matmul [0:DB_OUTPUT_ELEMENTS-1];
  logic [31:0] final_matmul [0:DB_OUTPUT_ELEMENTS-1];

  // Timing: same as tb_DIMC_dual.sv (100 MHz, 2 ns apply, 8 ns test)
  localparam time ClkPeriod = 10ns;
  localparam time ApplTime  = 2ns;
  localparam time TestTime  =  8ns;
 
 
  // =========================================================================
  // DUT SIGNAL DECLARATIONS
  // =========================================================================
  
  // Clock and reset
  logic clk = 1'b0;
  logic rst_n = 1'b0;
  logic count_test_cycles = 1'b0;
  longint unsigned test_cycle_count = 0;
  longint unsigned result_pop_count = 0;

  logic sel = 1'b0;

  // All initial values are the safe idle state (no operation firing at time 0).
  // for m0
  logic COMPE_m0 = 1'b0, FCSN_m0 = 1'b1;
  logic [1:0] MODE_m0 = 2'b11, FA_m0 = '0;
  logic [31:0] ADDIN_m0 = '0;
  logic [6:0] RA_m0 = '0, WA_m0 = '0;
  logic RCSN_m0 = 1'b1;
  logic RCSN0_m0 = 1'b1, RCSN1_m0 = 1'b1;
  logic RCSN2_m0 = 1'b1, RCSN3_m0 = 1'b1;
  logic WCSN_m0 = 1'b1, WEN_m0 = 1'b1;
  logic [SECTION_WIDTH-1:0] M_m0 = '1;
  logic [9:0] compute_mask_m0 = '0;
  logic [1:0] sign_8b_m0 = 2'b00;
  
  // for m1
  logic COMPE_m1 = 1'b0, FCSN_m1 = 1'b1;
  logic [1:0] MODE_m1 = 2'b11, FA_m1 = '0;
  logic [31:0] ADDIN_m1 = '0;
  logic [6:0] RA_m1 = '0, WA_m1 = '0;
  logic RCSN_m1 = 1'b1;
  logic RCSN0_m1 = 1'b1, RCSN1_m1 = 1'b1;
  logic RCSN2_m1 = 1'b1, RCSN3_m1 = 1'b1;
  logic WCSN_m1 = 1'b1, WEN_m1 = 1'b1;
  logic [SECTION_WIDTH-1:0] M_m1 = '1;
  logic [9:0] compute_mask_m1 = '0;
  logic [1:0] sign_8b_m1 = 2'b00;

  // for fifos
  logic inp_push = 1'b0;
  logic [SECTION_WIDTH-1:0] inp_data = '0;
  logic inp_full, inp_empty;
  logic wgt_push = 1'b0;
  logic [SECTION_WIDTH-1:0] wgt_data = '0;
  logic wgt_full, wgt_empty;
  logic out_pop = 1'b0;
  logic [31:0] out_data;
  logic out_full, out_empty;
  logic READYN;
  logic [31:0] PSOUT;
  logic clear = 1'b0;
  logic [31:0] acc_o [0:255];

  typedef enum logic [3:0] {
    STATE_RESET,
    STATE_1_LOAD_M0,
    STATE_2_COMPUTE_M0_LOAD_M1,
    STATE_3_COMPUTE_M1_LOAD_M0,
    STATE_4_COMPUTE_M0,
    STATE_5_COMPUTE_M1,
    STATE_6_FINISH
  } double_buffer_state_t;

  double_buffer_state_t state;
  int unsigned MATMUL_Requested = 0;
  int unsigned MATMUL_to_request = DB_Q * DB_K * DB_L;
  int MATMUL_Computed = 0;
  longint unsigned MATMUL_Computed_Total = 0;
  int unsigned k, l, q, p;

  cleopatra #(
    .SECTION_WIDTH (SECTION_WIDTH),
    .INP_FIFO_DEPTH(DB_P * NUM_SECTIONS)
  ) i_dut (
    .clk, .rst_n, .sel,
    .COMPE_m0, .FCSN_m0, .MODE_m0, .FA_m0, .ADDIN_m0,
    .RA_m0, .WA_m0, .RCSN_m0, .RCSN0_m0, .RCSN1_m0,
    .RCSN2_m0, .RCSN3_m0, .WCSN_m0, .WEN_m0, .M_m0,
    .compute_mask_m0, .sign_8b_m0,
    .COMPE_m1, .FCSN_m1, .MODE_m1, .FA_m1, .ADDIN_m1,
    .RA_m1, .WA_m1, .RCSN_m1, .RCSN0_m1, .RCSN1_m1,
    .RCSN2_m1, .RCSN3_m1, .WCSN_m1, .WEN_m1, .M_m1,
    .compute_mask_m1, .sign_8b_m1,
    .inp_push, .inp_data,
    .wgt_push, .wgt_data,
    .clear, .acc_o
  );

  always #(ClkPeriod/2) clk = ~clk;

  always @(posedge clk) begin
    if (count_test_cycles)
      test_cycle_count <= test_cycle_count + 1;
    if (!rst_n)
      result_pop_count <= 0;
    else if (i_dut.out_pop)
      result_pop_count <= result_pop_count + 1;
  end



  task automatic state_1_load_m0_idle_m1(
    input db_input_tile_t features,
    input db_kernel_tile_t kernel
  );

    int NB_SECTIONS = NB_KERNEL_ROWS*4;
    // putting kernel into kernel_sections
    logic [SECTION_WIDTH-1:0] kernel_sections
        [0:NB_KERNEL_ROWS*4-1];
    // Macro 1 must not consume either shared FIFO while macro 0 is loaded.
    COMPE_m1 = 1'b0;
    FCSN_m1  = 1'b1;
    RCSN_m1  = 1'b1;
    RCSN0_m1 = 1'b1;
    RCSN1_m1 = 1'b1;
    RCSN2_m1 = 1'b1;
    RCSN3_m1 = 1'b1;
    WCSN_m1  = 1'b1;
    WEN_m1   = 1'b1;

    // unpacking kernel into sections for writing to macro 0
    for (int section = 0; section < NB_KERNEL_ROWS*4; section++) begin
      kernel_sections[section] =
          kernel[
              DB_WEIGHT_TILE_BITS - section*SECTION_WIDTH - 1
              -: SECTION_WIDTH
          ];
    end


    // Queue all P=8 feature vectors while both macros leave the input FIFO idle.
    FCSN_m0 = 1'b1;
    inp_push = 1'b1;
    inp_data = features[0][0];

    for (int input_section = 0; input_section < DB_P*NUM_SECTIONS; input_section++) begin
      @(posedge clk); #ApplTime;
      if (input_section < DB_P*NUM_SECTIONS-1)
        // features[p][section]: p selects one of 8 vectors; section selects one of its 4 256-bit chunks.
        inp_data = features[(input_section+1)/NUM_SECTIONS]
                             [(input_section+1)%NUM_SECTIONS];
      else
        inp_push = 1'b0;
    end

    // Pop the first vector (p=0) into macro 0.  The remaining P-1 vectors
    // stay queued for subsequent compute operations.
    FCSN_m0 = 1'b0;
    FA_m0   = 2'd0;
    for (int section = 0; section < NUM_SECTIONS; section++) begin
      @(posedge clk); #ApplTime;
      if (section < NUM_SECTIONS-1)
        FA_m0 = 2'(section+1);
      else begin
        FCSN_m0 = 1'b1;
        FA_m0   = '0;
      end
    end


    // loading kernel for macro 0
    // Set up section 0 before the next edge; no alignment cycle is needed.
    COMPE_m0 = 1'b0; RCSN_m0 = 1'b1; FCSN_m0 = 1'b1; M_m0 = '1;
    WCSN_m0  = 1'b1; WEN_m0  = 1'b1;
    wgt_push = 1'b1; wgt_data = kernel_sections[0];

    // Each edge: push section i+1 (if any left) while writing section i-1
    // (its push registered 1 edge earlier), addressed by {row,sec} = i-1.
    for (int i = 0; i < NB_SECTIONS; i++) begin
      @(posedge clk); #ApplTime;
      if (i < NB_SECTIONS-1)
        wgt_data = kernel_sections[i+1];
      else
        wgt_push = 1'b0;              // last section already pushed this edge
      WA_m0 = 7'(i);                     // {row,sec} of section i -- write it next edge
      WCSN_m0 = 1'b0; WEN_m0 = 1'b0;
    end

    // Final edge: last section's write completes.
    @(posedge clk); #ApplTime;
    WCSN_m0 = 1'b1; WEN_m0 = 1'b1;


  endtask

  task automatic state_2_compute_m0_while_loading_m1(
    input db_input_tile_t features,
    input db_kernel_tile_t kernel
  );

    int NB_SECTIONS = NB_KERNEL_ROWS*4;
    // putting kernel into kernel_sections
    logic [SECTION_WIDTH-1:0] kernel_sections
        [0:NB_KERNEL_ROWS*4-1];
    logic last_vector_loaded_m0;
    longint unsigned expected_result_count;

    // unpacking kernel into sections for writing to macro 0
    for (int section = 0; section < NB_KERNEL_ROWS*4; section++) begin
      kernel_sections[section] =
          kernel[
              DB_WEIGHT_TILE_BITS - section*SECTION_WIDTH - 1
              -: SECTION_WIDTH
          ];
    end

    sel = 1'b0;
    last_vector_loaded_m0 = (DB_P == 1);
    expected_result_count = result_pop_count + DB_M*DB_P;

    fork
      begin : compute_current_tile_on_m0
        // Vector 0 is already in macro 0; vectors 1..7 are at the FIFO head.
        for (int vector_index = 0; vector_index < DB_P; vector_index++) begin
          for (int row = 0; row < NB_KERNEL_ROWS; row++) begin
            COMPE_m0 = 1'b1; MODE_m0 = 2'b11; compute_mask_m0 = 10'd0;
            RA_m0 = {5'(row), 2'b00}; ADDIN_m0 = BIAS;
            RCSN_m0 = 1'b0; RCSN0_m0 = 1'b0; RCSN1_m0 = 1'b0;
            RCSN2_m0 = 1'b0; RCSN3_m0 = 1'b0;
            WCSN_m0 = 1'b1; WEN_m0 = 1'b1; FCSN_m0 = 1'b1;
            @(posedge clk); #TestTime;
          end

          COMPE_m0 = 1'b0;
          RCSN_m0 = 1'b1; RCSN0_m0 = 1'b1; RCSN1_m0 = 1'b1;
          RCSN2_m0 = 1'b1; RCSN3_m0 = 1'b1;

          if (vector_index < DB_P-1) begin
            FCSN_m0 = 1'b0;
            FA_m0 = 2'd0;
            for (int section = 0; section < NUM_SECTIONS; section++) begin
              @(posedge clk); #ApplTime;
              if (section < NUM_SECTIONS-1)
                FA_m0 = 2'(section+1);
              else begin
                FCSN_m0 = 1'b1;
                FA_m0 = '0;
              end
            end
            if (vector_index == DB_P-2)
              last_vector_loaded_m0 = 1'b1;
          end
        end
      end

      begin : enqueue_next_features_for_m1
        // New sections are appended behind macro 0's seven queued vectors.
        FCSN_m1 = 1'b1;
        inp_push = 1'b0;
        for (int input_section = 0;
             input_section < DB_P*NUM_SECTIONS;
             input_section++) begin
          // Wait for space created as macro 0 pops an old section.
          while (i_dut.inp_full)
            @(negedge clk);
          inp_data = features[input_section/NUM_SECTIONS]
                             [input_section%NUM_SECTIONS];
          inp_push = 1'b1;
          @(posedge clk); #ApplTime;
          inp_push = 1'b0;
        end

        // Once macro 0 has loaded its last vector, the FIFO head belongs to
        // macro 1. Load macro 1's first vector during macro 0's final compute.
        wait (last_vector_loaded_m0);
        FCSN_m1 = 1'b0;
        FA_m1 = 2'd0;
        for (int section = 0; section < NUM_SECTIONS; section++) begin
          @(posedge clk); #ApplTime;
          if (section < NUM_SECTIONS-1)
            FA_m1 = 2'(section+1);
          else begin
            FCSN_m1 = 1'b1;
            FA_m1 = '0;
          end
        end
      end

      begin : load_next_kernel_into_m1
        COMPE_m1 = 1'b0; RCSN_m1 = 1'b1; FCSN_m1 = 1'b1; M_m1 = '1;
        WCSN_m1 = 1'b1; WEN_m1 = 1'b1;
        wgt_push = 1'b1; wgt_data = kernel_sections[0];

        for (int i = 0; i < NB_SECTIONS; i++) begin
          @(posedge clk); #ApplTime;
          if (i < NB_SECTIONS-1)
            wgt_data = kernel_sections[i+1];
          else
            wgt_push = 1'b0;
          WA_m1 = 7'(i);
          WCSN_m1 = 1'b0; WEN_m1 = 1'b0;
        end

        @(posedge clk); #ApplTime;
        WCSN_m1 = 1'b1; WEN_m1 = 1'b1;
      end
    join

    // Keep sel on macro 0 until every issued result has been captured.
    wait (result_pop_count >= expected_result_count);
    #TestTime;

  endtask

  task automatic state_3_compute_m1_while_loading_m0(
    input db_input_tile_t features,
    input db_kernel_tile_t kernel
  );
    int NB_SECTIONS = NB_KERNEL_ROWS*4;
    logic [SECTION_WIDTH-1:0] kernel_sections
        [0:NB_KERNEL_ROWS*4-1];
    logic last_vector_loaded_m1;
    longint unsigned expected_result_count;

    for (int section = 0; section < NB_KERNEL_ROWS*4; section++) begin
      kernel_sections[section] =
          kernel[
              DB_WEIGHT_TILE_BITS - section*SECTION_WIDTH - 1
              -: SECTION_WIDTH
          ];
    end

    sel = 1'b1;
    last_vector_loaded_m1 = (DB_P == 1);
    expected_result_count = result_pop_count + DB_M*DB_P;

    fork
      begin : compute_current_tile_on_m1
        for (int vector_index = 0; vector_index < DB_P; vector_index++) begin
          for (int row = 0; row < NB_KERNEL_ROWS; row++) begin
            COMPE_m1 = 1'b1; MODE_m1 = 2'b11; compute_mask_m1 = 10'd0;
            RA_m1 = {5'(row), 2'b00}; ADDIN_m1 = BIAS;
            RCSN_m1 = 1'b0; RCSN0_m1 = 1'b0; RCSN1_m1 = 1'b0;
            RCSN2_m1 = 1'b0; RCSN3_m1 = 1'b0;
            WCSN_m1 = 1'b1; WEN_m1 = 1'b1; FCSN_m1 = 1'b1;
            @(posedge clk); #TestTime;
          end

          COMPE_m1 = 1'b0;
          RCSN_m1 = 1'b1; RCSN0_m1 = 1'b1; RCSN1_m1 = 1'b1;
          RCSN2_m1 = 1'b1; RCSN3_m1 = 1'b1;

          if (vector_index < DB_P-1) begin
            FCSN_m1 = 1'b0;
            FA_m1 = 2'd0;
            for (int section = 0; section < NUM_SECTIONS; section++) begin
              @(posedge clk); #ApplTime;
              if (section < NUM_SECTIONS-1)
                FA_m1 = 2'(section+1);
              else begin
                FCSN_m1 = 1'b1;
                FA_m1 = '0;
              end
            end
            if (vector_index == DB_P-2)
              last_vector_loaded_m1 = 1'b1;
          end
        end
      end

      begin : enqueue_next_features_for_m0
        FCSN_m0 = 1'b1;
        inp_push = 1'b0;
        for (int input_section = 0;
             input_section < DB_P*NUM_SECTIONS;
             input_section++) begin
          while (i_dut.inp_full)
            @(negedge clk);
          inp_data = features[input_section/NUM_SECTIONS]
                             [input_section%NUM_SECTIONS];
          inp_push = 1'b1;
          @(posedge clk); #ApplTime;
          inp_push = 1'b0;
        end

        wait (last_vector_loaded_m1);
        FCSN_m0 = 1'b0;
        FA_m0 = 2'd0;
        for (int section = 0; section < NUM_SECTIONS; section++) begin
          @(posedge clk); #ApplTime;
          if (section < NUM_SECTIONS-1)
            FA_m0 = 2'(section+1);
          else begin
            FCSN_m0 = 1'b1;
            FA_m0 = '0;
          end
        end
      end

      begin : load_next_kernel_into_m0
        COMPE_m0 = 1'b0; RCSN_m0 = 1'b1; FCSN_m0 = 1'b1; M_m0 = '1;
        WCSN_m0 = 1'b1; WEN_m0 = 1'b1;
        wgt_push = 1'b1; wgt_data = kernel_sections[0];

        for (int i = 0; i < NB_SECTIONS; i++) begin
          @(posedge clk); #ApplTime;
          if (i < NB_SECTIONS-1)
            wgt_data = kernel_sections[i+1];
          else
            wgt_push = 1'b0;
          WA_m0 = 7'(i);
          WCSN_m0 = 1'b0; WEN_m0 = 1'b0;
        end

        @(posedge clk); #ApplTime;
        WCSN_m0 = 1'b1; WEN_m0 = 1'b1;
      end
    join

    // Keep sel on macro 1 until every issued result has been captured.
    wait (result_pop_count >= expected_result_count);
    #TestTime;

  endtask


  // Compute all P columns of the final M-by-P output tile on macro 0.
  task automatic state_4_compute_m0_idle_m1;
    // Select macro 0's results and keep macro 1 completely idle.
    sel = 1'b0;
    COMPE_m1 = 1'b0;
    FCSN_m1  = 1'b1;
    RCSN_m1  = 1'b1;
    RCSN0_m1 = 1'b1;
    RCSN1_m1 = 1'b1;
    RCSN2_m1 = 1'b1;
    RCSN3_m1 = 1'b1;
    WCSN_m1  = 1'b1;
    WEN_m1   = 1'b1;

    // Vector 0 was loaded by State 1.  After each MatVec, pop the next
    // vector's four sections from the FIFO before starting the next MatVec.
    for (int vector_index = 0; vector_index < DB_P; vector_index++) begin
      for (int row = 0; row < NB_KERNEL_ROWS; row++) begin
        COMPE_m0 = 1'b1; MODE_m0 = 2'b11; compute_mask_m0 = 10'd0;
        RA_m0    = {5'(row), 2'b00}; ADDIN_m0 = BIAS;
        RCSN_m0  = 1'b0; RCSN0_m0 = 1'b0; RCSN1_m0 = 1'b0;
        RCSN2_m0 = 1'b0; RCSN3_m0 = 1'b0;
        WCSN_m0  = 1'b1; WEN_m0 = 1'b1; FCSN_m0 = 1'b1;
        @(posedge clk); #TestTime;
      end

      // Deassert compute controls after the last row of this vector.
      COMPE_m0 = 1'b0;
      RCSN_m0  = 1'b1; RCSN0_m0 = 1'b1; RCSN1_m0 = 1'b1;
      RCSN2_m0 = 1'b1; RCSN3_m0 = 1'b1;

      if (vector_index < DB_P-1) begin
        // FCSN_m0 makes the DUT pop one FIFO section per clock edge.
        FCSN_m0 = 1'b0;
        FA_m0   = 2'd0;
        for (int section = 0; section < NUM_SECTIONS; section++) begin
          @(posedge clk); #ApplTime;
          if (section < NUM_SECTIONS-1)
            FA_m0 = 2'(section+1);
          else begin
            FCSN_m0 = 1'b1;
            FA_m0   = '0;
          end
        end
      end
    end

  endtask

  // Compute all P columns of the final M-by-P output tile on macro 1.
  task automatic state_5_compute_m1_idle_m0;
    // Select macro 1's results and keep macro 0 completely idle.
    sel = 1'b1;

    COMPE_m0 = 1'b0;
    FCSN_m0  = 1'b1;
    RCSN_m0  = 1'b1;
    RCSN0_m0 = 1'b1;
    RCSN1_m0 = 1'b1;
    RCSN2_m0 = 1'b1;
    RCSN3_m0 = 1'b1;
    WCSN_m0  = 1'b1;
    WEN_m0   = 1'b1;

    // Vector 0 was loaded by State 1.  After each MatVec, pop the next
    // vector's four sections from the FIFO before starting the next MatVec.
    for (int vector_index = 0; vector_index < DB_P; vector_index++) begin
      for (int row = 0; row < NB_KERNEL_ROWS; row++) begin
        COMPE_m1 = 1'b1; MODE_m1 = 2'b11; compute_mask_m1 = 10'd0;
        RA_m1    = {5'(row), 2'b00}; ADDIN_m1 = BIAS;
        RCSN_m1  = 1'b0; RCSN0_m1 = 1'b0; RCSN1_m1 = 1'b0;
        RCSN2_m1 = 1'b0; RCSN3_m1 = 1'b0;
        WCSN_m1  = 1'b1; WEN_m1 = 1'b1; FCSN_m1 = 1'b1;
        @(posedge clk); #TestTime;
      end

      // Deassert compute controls after the last row of this vector.
      COMPE_m1 = 1'b0;
      RCSN_m1  = 1'b1; RCSN0_m1 = 1'b1; RCSN1_m1 = 1'b1;
      RCSN2_m1 = 1'b1; RCSN3_m1 = 1'b1;

      if (vector_index < DB_P-1) begin
        // FCSN_m1 makes the DUT pop one FIFO section per clock edge.
        FCSN_m1 = 1'b0;
        FA_m1   = 2'd0;
        for (int section = 0; section < NUM_SECTIONS; section++) begin
          @(posedge clk); #ApplTime;
          if (section < NUM_SECTIONS-1)
            FA_m1 = 2'(section+1);
          else begin
            FCSN_m1 = 1'b1;
            FA_m1   = '0;
          end
        end
      end
    end

  endtask
  
  task automatic clear_accumulators;
    clear = 1'b1;
    @(posedge clk); #ApplTime;
    clear = 1'b0;
  endtask

  task automatic save_accumulator_tile(input int accumulator_file);
    // File order is row-major; acc_o is arranged as [column*DB_M + row].
    for (int row = 0; row < DB_M; row++) begin
      for (int col = 0; col < DB_P; col++) begin
        $fdisplay(accumulator_file, "%08h", acc_o[col*DB_M + row]);
      end
    end
  endtask

  task automatic finish_l_matmuls
  (
    input int accumulator_file,
    inout int MATMUL_Computed
  );
    MATMUL_Computed++;
    MATMUL_Computed_Total++;
    if (MATMUL_Computed == DB_L) begin
      // Wait for exactly the results issued so far; avoid a fixed drain stall.
      wait (result_pop_count >= MATMUL_Computed_Total*DB_M*DB_P);
      #TestTime;
      save_accumulator_tile(accumulator_file);
      clear_accumulators();
      MATMUL_Computed = 0;
    end
  endtask

  task automatic advance_tile_indices;
    // Advance in l-fastest order: [k][q][l].
    if (l == DB_L-1) begin
      l = 0;
      if (q == DB_Q-1) begin
        q = 0;
        if (k != DB_K-1)
          k++;
      end else begin
        q++;
      end
    end else begin
      l++;
    end
  endtask

  // =========================================================================
  // PASS / FAIL COUNTERS
  // =========================================================================
  int pass_count = 0;
  int fail_count = 0;


  initial begin
    int errors;
    int accumulator_file;
    int python_status;

    errors = 0;

    $readmemh(KERNEL_STIM_FILE, kernel_stim_flat);
    $readmemh(FEATURE_STIM_FILE, feature_stim_flat);
    $readmemh(TILED_WEIGHTS_FILE, tiled_weights_flat);
    $readmemh(TILED_INPUTS_FILE, tiled_inputs_flat);
    $readmemh(GOLDEN_MATMUL_FILE, golden_matmul);

`ifdef TB_DOUBLE_BUFFERING_TEST1
    // =======================================================================
    // TEST 1: Testing Cleopatra bigger matrix handling with double buffering 
    // =======================================================================

    // Pack the full (K*M)-by-(L*N) kernel matrix into [k,l] tiles.
    for (int k = 0; k < DB_K; k++) begin
      for (int l = 0; l < DB_L; l++) begin
        for (int row = 0; row < DB_M; row++) begin
          for (int n = 0; n < DB_N_ELEMENTS; n++) begin
            kernel_stim[k][l][
                DB_WEIGHT_TILE_BITS - (row*DB_N_ELEMENTS+n)*DB_ELEMENT_BITS - 1
                -: DB_ELEMENT_BITS
            ] = kernel_stim_flat[
                (k*DB_M+row)*(DB_L*DB_N_ELEMENTS) + l*DB_N_ELEMENTS+n
            ];
          end
        end
        if (kernel_stim[k][l] !== tiled_weights_flat[k*DB_L+l])
          $fatal(1, "[TB_DOUBLE] Kernel tile [%0d,%0d] packing mismatch", k, l);
      end
    end

    // Pack the full (L*N)-by-(Q*P) feature matrix into [l,q] tiles;
    // columns are contiguous inside each packed tile.
    for (int l = 0; l < DB_L; l++) begin
      for (int q = 0; q < DB_Q; q++) begin
        for (int p = 0; p < DB_P; p++) begin
          for (int n = 0; n < DB_N_ELEMENTS; n++) begin
            feature_stim[l][q]
                        [p]
                        [n/(SECTION_WIDTH/DB_ELEMENT_BITS)]
                        [SECTION_WIDTH
                         - (n%(SECTION_WIDTH/DB_ELEMENT_BITS))*DB_ELEMENT_BITS
                         - 1 -: DB_ELEMENT_BITS] = feature_stim_flat[
                (l*DB_N_ELEMENTS+n)*(DB_Q*DB_P) + q*DB_P+p
            ];
          end
        end
        if (feature_stim[l][q] !== tiled_inputs_flat[q*DB_L+l])
          $fatal(1, "[TB_DOUBLE] Input tile [%0d,%0d] packing mismatch", l, q);
      end
    end

    accumulator_file = $fopen(ACCUMULATOR_OUTPUT_FILE, "w");
    if (accumulator_file == 0)
      $fatal(1, "[TB_DOUBLE] Failed to open accumulator output file");

    test_cycle_count = 0;
    count_test_cycles = 1'b1;
    state = STATE_RESET;

    // Testbench FSM: each state is implemented by one state task. Checks are
    // completed before advancing, so the next state always starts cleanly.
    while (state != STATE_6_FINISH) begin
      case (state)
        STATE_RESET: begin
          $display("[TB_DOUBLE] Reset");
          rst_n = 1'b0;
          MATMUL_Requested = 0;
          MATMUL_Computed = 0;
          MATMUL_Computed_Total = 0;
          k = 0;
          l = 0; // note: for now l and matmul_requested do the exact same function 
          q = 0;
          repeat (3) @(posedge clk);
          #ApplTime;
          rst_n = 1'b1;
          clear_accumulators();
          state = STATE_1_LOAD_M0;
        end

        STATE_1_LOAD_M0: begin
          //$display("[TB_DOUBLE] State 1: load macro 0; macro 1 idle");
          state_1_load_m0_idle_m1(feature_stim[l][q],
                                  kernel_stim[k][l]);
          MATMUL_Requested++;
          advance_tile_indices();

          // State 1 loaded macro 0, so its final tile must run on macro 0.
          if (MATMUL_Requested == MATMUL_to_request)
            state = STATE_4_COMPUTE_M0;
          else
            state = STATE_2_COMPUTE_M0_LOAD_M1;
        end

        STATE_2_COMPUTE_M0_LOAD_M1: begin
          //$display("[TB_DOUBLE] State 2: compute macro 0; load macro 1");
          state_2_compute_m0_while_loading_m1(feature_stim[l][q],
                                              kernel_stim[k][l]);
          advance_tile_indices();
          finish_l_matmuls(accumulator_file, MATMUL_Computed);
          MATMUL_Requested++;
          if (MATMUL_Requested < MATMUL_to_request)
            state = STATE_3_COMPUTE_M1_LOAD_M0;
          else
            state = STATE_5_COMPUTE_M1;
        end

        STATE_3_COMPUTE_M1_LOAD_M0: begin
          //$display("[TB_DOUBLE] State 3: compute macro 1; load macro 0");
          state_3_compute_m1_while_loading_m0(feature_stim[l][q],
                                              kernel_stim[k][l]);
          advance_tile_indices();
          finish_l_matmuls(accumulator_file, MATMUL_Computed);
          MATMUL_Requested++;
          if (MATMUL_Requested < MATMUL_to_request)
            state = STATE_2_COMPUTE_M0_LOAD_M1;
          else
            state = STATE_4_COMPUTE_M0;
        end

        STATE_4_COMPUTE_M0: begin
          //$display("[TB_DOUBLE] State 4: compute macro 0 and verify");
          state_4_compute_m0_idle_m1();
          finish_l_matmuls(accumulator_file, MATMUL_Computed);
          state = STATE_6_FINISH;
        end

        STATE_5_COMPUTE_M1: begin
          //$display("[TB_DOUBLE] State 5: compute macro 1 and verify");
          state_5_compute_m1_idle_m0();
          finish_l_matmuls(accumulator_file, MATMUL_Computed);
          state = STATE_6_FINISH;
        end

        default: begin
          $error("[TB_DOUBLE] Illegal FSM state");
          errors++;
          state = STATE_6_FINISH;
        end
      endcase
    end

    count_test_cycles = 1'b0;
    $display("[TB_DOUBLE] FSM completed in %0d clock cycles", test_cycle_count);

    $fclose(accumulator_file);

    if (MATMUL_Computed != 0) begin
      $error("[TB_DOUBLE] Finished with an incomplete L accumulation group");
      errors++;
    end

    python_status = $system(
        "python3 stimuli/double_buffering_stim.py --untile-only"
    );
    if (python_status != 0) begin
      $error("[TB_DOUBLE] Failed to untile accumulator output");
      errors++;
    end else begin
      $readmemh(FINAL_MATMUL_FILE, final_matmul);
      for (int output_index = 0;
           output_index < DB_OUTPUT_ELEMENTS;
           output_index++) begin
        if (final_matmul[output_index] !== golden_matmul[output_index]) begin
         /* $error(
              "[TB_DOUBLE] Final output mismatch at %0d: got %08h expected %08h",
              output_index,
              final_matmul[output_index],
              golden_matmul[output_index]
          );*/
          errors++;
        end
      end
    end

    if (errors == 0) begin
      $display("[TB_DOUBLE] Test 1: PASS");
      pass_count++;
    end else begin
      $error("[TB_DOUBLE] Test 1: FAIL (%0d errors)", errors);
      fail_count++;
    end
`endif // TB_DOUBLE_BUFFERING_TEST1

    // =========================================================================
    // FINAL SUMMARY
    // =========================================================================
    $display("[TB] RESULTS: %0d PASSED, %0d FAILED", pass_count, fail_count);
    if (fail_count == 0) $display("[TB] ALL TESTS PASSED");
    else                  $display("[TB] FAILURES DETECTED");
    $display("Testbench: Test finished.");
    if (fail_count != 0)
      $fatal(1, "[TB_DOUBLE] Testbench failed");
    else
      $finish;
  end

  // =========================================================================
  // WAVEFORM DUMP
  // =========================================================================
  initial begin
    $dumpfile("sim/tb_double_buffering.vcd");
    $dumpvars(0, tb_double_buffering);
  end

endmodule
