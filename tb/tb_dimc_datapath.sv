/*
 * tb_dimc_datapath.sv
 *
 * ============================================================
 * PURPOSE
 * ============================================================
 * Standalone unit test for dimc_datapath (rtl/dimc_datapath.sv) --
 * the module that bridges HWPE-Stream (valid/ready) handshakes to
 * spatz_DIMC_dual's raw push/full/pop/empty FIFO protocol and sequences
 * its control pins (WA/FA/RA, WCSN/WEN/FCSN/COMPE/RCSN*).
 *
 * No dimc_streamer/TCDM/register-file is involved here -- this directly
 * drives dimc_datapath's input_i/kernel_i HWPE-Stream sink ports as a
 * plain valid/ready producer and drains output_o as a plain valid/ready
 * consumer, then checks the results against the SAME golden vectors used
 * by tb_DIMC_dual.sv (stimuli/dimc_tests/golden_psum_32bit.txt), reusing the
 * same kernel/feature stimuli. This proves the bridge + sequencer inside
 * dimc_datapath reproduce spatz_DIMC_dual's known-good behavior (verified
 * directly, in tb_DIMC_dual.sv) when driven purely through HWPE-Stream.
 */

`timescale 1ns/1ps

module tb_dimc_datapath;
  import dimc_package::*;

  // -------------------------------------------------------------------------
  // Parameters (mirrors tb_DIMC_dual.sv)
  // -------------------------------------------------------------------------
  parameter int unsigned SECTION_WIDTH  = 256;
  localparam int unsigned NUM_SECTIONS       = 1024 / SECTION_WIDTH; // = 4
  localparam int unsigned NB_KERNEL_SECTIONS = NB_KERNEL_ROWS * NUM_SECTIONS; // = 128

  parameter KERNEL_WEIGHTS_FILE = "stimuli/dimc_tests/kernel_weights.txt";
  parameter FEATURE_VECTOR_FILE = "stimuli/dimc_tests/feature_vector.txt";
  parameter GOLDEN_PSOUT_FILE   = "stimuli/dimc_tests/golden_psum_32bit.txt";

  localparam time ClkPeriod = 10ns;
  localparam time ApplTime  =  2ns;

  // -------------------------------------------------------------------------
  // Stimulus / golden arrays (filled by $readmemh)
  // -------------------------------------------------------------------------
  logic [SECTION_WIDTH-1:0] kernel_stim  [0 : NB_KERNEL_SECTIONS-1]; // 128 sections
  logic [SECTION_WIDTH-1:0] feature_stim [0 : NUM_SECTIONS-1];       //   4 sections
  logic [31:0]              golden_psout [0 : NB_KERNEL_ROWS-1];

  // -------------------------------------------------------------------------
  // Clock / reset
  // -------------------------------------------------------------------------
  logic clk;
  logic rst_n;

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

  // -------------------------------------------------------------------------
  // DUT
  // -------------------------------------------------------------------------
  logic clear;

  hwpe_stream_intf_stream #( .DATA_WIDTH ( SECTION_WIDTH ) ) input_stream  ( .clk ( clk ) );
  hwpe_stream_intf_stream #( .DATA_WIDTH ( SECTION_WIDTH ) ) kernel_stream ( .clk ( clk ) );
  hwpe_stream_intf_stream #( .DATA_WIDTH ( SECTION_WIDTH ) ) output_stream ( .clk ( clk ) );

  dimc_datapath #(
    .SECTION_WIDTH  ( SECTION_WIDTH  )
  ) i_dut (
    .clk_i    ( clk           ),
    .rst_ni   ( rst_n         ),
    .clear_i  ( clear         ),
    .input_i  ( input_stream  ),
    .kernel_i ( kernel_stream ),
    .output_o ( output_stream )
  );

  // -------------------------------------------------------------------------
  // Driver: push all 128 kernel sections + all 4 feature sections as
  // independent, concurrent HWPE-Stream valid/ready producers -- the
  // wgt_fifo/inp_fifo inside dimc_datapath decouple timing, so both can be
  // pushed without waiting for the sequencer to consume them.
  // -------------------------------------------------------------------------
  initial begin
    kernel_stream.valid = 1'b0;
    kernel_stream.data  = '0;
    kernel_stream.strb  = '1;
    @(posedge rst_n);
    @(posedge clk);
    for (int i = 0; i < NB_KERNEL_SECTIONS; i++) begin
      @(posedge clk); #ApplTime;
      kernel_stream.valid = 1'b1;
      kernel_stream.data  = kernel_stim[i];
      wait (kernel_stream.ready);
    end
    @(posedge clk); #ApplTime;
    kernel_stream.valid = 1'b0;
  end

  initial begin
    input_stream.valid = 1'b0;
    input_stream.data  = '0;
    input_stream.strb  = '1;
    @(posedge rst_n);
    @(posedge clk);
    for (int i = 0; i < NUM_SECTIONS; i++) begin
      @(posedge clk); #ApplTime;
      input_stream.valid = 1'b1;
      input_stream.data  = feature_stim[i];
      wait (input_stream.ready);
    end
    @(posedge clk); #ApplTime;
    input_stream.valid = 1'b0;
  end

  // -------------------------------------------------------------------------
  // Receiver: drain output_o as a plain valid/ready consumer, collecting
  // NB_KERNEL_ROWS results.
  // -------------------------------------------------------------------------
  logic [SECTION_WIDTH-1:0] results [0 : NB_KERNEL_ROWS-1];
  int unsigned results_cnt;

  initial begin
    output_stream.ready = 1'b0;
    results_cnt = 0;
    @(posedge rst_n);
    output_stream.ready = 1'b1;
    forever begin
      @(posedge clk); #ApplTime;
      if (output_stream.valid && output_stream.ready) begin
        results[results_cnt] = output_stream.data;
        results_cnt++;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Main sequence: load stimuli, run reset, wait for all results, check.
  // -------------------------------------------------------------------------
  int pass_count = 0;
  int fail_count = 0;

  initial begin
    clear = 1'b1;

    $readmemh(KERNEL_WEIGHTS_FILE, kernel_stim);
    $readmemh(FEATURE_VECTOR_FILE, feature_stim);
    $readmemh(GOLDEN_PSOUT_FILE,   golden_psout);

    @(posedge rst_n);
    @(posedge clk); #ApplTime;
    clear = 1'b0;

    $display("[TB] tb_dimc_datapath: waiting for %0d results...", NB_KERNEL_ROWS);
    wait (results_cnt == NB_KERNEL_ROWS);
    @(posedge clk); #ApplTime;

    for (int r = 0; r < NB_KERNEL_ROWS; r++) begin
      automatic logic [31:0] expected32 = golden_psout[r];
      automatic logic [31:0] got32      = results[r][31:0];
      automatic logic [SECTION_WIDTH-33:0] got_upper = results[r][SECTION_WIDTH-1:32];

      if (got32 !== expected32) begin
        $error("[TB] row%0d: got 0x%08h, expected 0x%08h", r, got32, expected32);
        fail_count++;
      end
      else if (got_upper !== '0) begin
        $error("[TB] row%0d: upper bits of output word not zero (0x%0h)", r, got_upper);
        fail_count++;
      end
      else begin
        pass_count++;
      end
    end

    $display("[TB] RESULTS: %0d PASSED, %0d FAILED", pass_count, fail_count);
    if (fail_count == 0) $display("[TB] ALL TESTS PASSED");
    else                  $display("[TB] FAILURES DETECTED");
    $display("Testbench: Test finished.");
    $finish;
  end

  // -------------------------------------------------------------------------
  // Waveform dump
  // -------------------------------------------------------------------------
  initial begin
    $dumpfile("sim/tb_dimc_datapath.vcd");
    $dumpvars(0, tb_dimc_datapath);
  end



endmodule // tb_dimc_datapath
