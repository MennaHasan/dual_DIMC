/* 
 * dimc_top.sv
 */
module dimc_top
  import dimc_package::*;
  import hwpe_ctrl_package::*;
  import hci_package::*;
#(
  parameter int unsigned N_CORES = 2,     // number of CPU cores that can start/control this HWPE (see periph slave port)
  parameter int unsigned MP  = 3,         // number of TCDM master ports (one per data stream: input, kernel, output)
  parameter int unsigned ID  = 10,        
  
  parameter int unsigned DATA_WIDTH = 256      // bit-width of a single input/kernel/output section (must match spatz_DIMC_dual's SECTION_WIDTH)
)
(
  // global signals
  input  logic                                  clk_i,
  input  logic                                  rst_ni,
  input  logic                                  test_mode_i,
 
  output logic [N_CORES-1:0][REGFILE_N_EVT-1:0] evt_o,
 
  hci_core_intf.initiator                          tcdm[MP-1:0],
 
  hwpe_ctrl_intf_periph.slave                   periph
);

  logic enable, clear;
 
  dimc_streamer_ctrl_t    streamer_ctrl;
  dimc_streamer_flags_t   streamer_flags;

  hwpe_stream_intf_stream #( .DATA_WIDTH( DATA_WIDTH ) )         input_stream        ( .clk ( clk_i ) );
  hwpe_stream_intf_stream #( .DATA_WIDTH( DATA_WIDTH ) )         kernel_stream        ( .clk ( clk_i ) );
  hwpe_stream_intf_stream #( .DATA_WIDTH( DATA_WIDTH ) )         output_stream        ( .clk ( clk_i ) );

  dimc_datapath #(
    .SECTION_WIDTH  ( DATA_WIDTH      )
  ) i_datapath (
    .clk_i    ( clk_i         ),
    .rst_ni   ( rst_ni        ),
    .clear_i  ( clear         ),
    .input_i  ( input_stream  ),
    .kernel_i ( kernel_stream ),
    .output_o ( output_stream )
  );

  dimc_streamer #(
    .MEM_WIDTH  ( DATA_WIDTH ), // uniform width: split/serialize/deserialize/merge all degenerate to NB=1 passthrough
    .DATA_WIDTH ( DATA_WIDTH ),
    .MP         ( MP         )
  ) i_streamer (
    .clk_i            ( clk_i          ),
    .rst_ni           ( rst_ni         ),
    .test_mode_i      ( test_mode_i    ),
    .enable_i         ( enable         ),
    .clear_i          ( clear          ),
    .input_o          ( input_stream   ),
    .kernel_o         ( kernel_stream  ),
    .output_i         ( output_stream  ),
    .tcdm             ( tcdm           ),
    .ctrl_i           ( streamer_ctrl  ),
    .flags_o          ( streamer_flags )
  );

  dimc_ctrl #(
    .N_CORES        ( 2               ),
    .N_CONTEXT      ( 2               ),
    .ID             ( ID              )
  ) i_ctrl (
    .clk_i              ( clk_i            ),
    .rst_ni             ( rst_ni           ),
    .test_mode_i        ( test_mode_i      ),
    .evt_o              ( evt_o            ),
    .clear_o            ( clear            ),
    .streamer_ctrl_o    ( streamer_ctrl    ),
    .streamer_flags_i   ( streamer_flags   ),
    .periph             ( periph           )
  );

  assign enable = 1'b1;

endmodule // dimc_top
