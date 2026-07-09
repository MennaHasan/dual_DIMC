/*
 * dimc_streamer.sv
*/

import dimc_package::*;
import hwpe_stream_package::*;


module dimc_streamer
#(
  parameter int unsigned MEM_WIDTH  = 32, // data width of the TCDM interface (32 bits)
  parameter int unsigned DATA_WIDTH = 16, // data width of the streams (16 bits)
  parameter int unsigned MP = 3           // number of master ports
)
(
  // global signals
  input  logic                   clk_i,
  input  logic                   rst_ni,
  input  logic                   test_mode_i,
  // local enable & clear
  input  logic                   enable_i,
  input  logic                   clear_i,


  // input stream towards dimc_datapath (loaded from memory)
  hwpe_stream_intf_stream.source input_o,
  // kernel stream towards dimc_datapath (loaded from memory)
  hwpe_stream_intf_stream.source kernel_o,
  // input port receiving the output stream from dimc_datapath (to be stored to memory)
  hwpe_stream_intf_stream.sink   output_i,

    hci_core_intf.initiator        tcdm [MP-1:0],

  input  dimc_streamer_ctrl_t     ctrl_i,
  output dimc_streamer_flags_t    flags_o
);

  hwpe_stream_intf_stream #( .DATA_WIDTH ( MEM_WIDTH ) ) input_mem ( .clk ( clk_i ) );
  hwpe_stream_intf_stream #( .DATA_WIDTH ( MEM_WIDTH ) ) kernel_mem ( .clk ( clk_i ) );
  hwpe_stream_intf_stream #( .DATA_WIDTH ( MEM_WIDTH ) ) output_mem ( .clk ( clk_i ) );

  hwpe_stream_intf_stream #( .DATA_WIDTH ( DATA_WIDTH ) ) input_split [MEM_WIDTH/DATA_WIDTH-1:0] ( .clk ( clk_i ) );
  hwpe_stream_intf_stream #( .DATA_WIDTH ( DATA_WIDTH ) ) kernel_split [MEM_WIDTH/DATA_WIDTH-1:0] ( .clk ( clk_i ) );
  hwpe_stream_intf_stream #( .DATA_WIDTH ( DATA_WIDTH ) ) output_split [MEM_WIDTH/DATA_WIDTH-1:0] ( .clk ( clk_i ) );

  hwpe_stream_intf_stream #( .DATA_WIDTH ( DATA_WIDTH ) ) input_split_postfifo [MEM_WIDTH/DATA_WIDTH-1:0] ( .clk ( clk_i ) );
  hwpe_stream_intf_stream #( .DATA_WIDTH ( DATA_WIDTH ) ) kernel_split_postfifo [MEM_WIDTH/DATA_WIDTH-1:0] ( .clk ( clk_i ) );
  hwpe_stream_intf_stream #( .DATA_WIDTH ( DATA_WIDTH ) ) output_split_prefence [MEM_WIDTH/DATA_WIDTH-1:0] ( .clk ( clk_i ) );

  hwpe_stream_intf_stream #( .DATA_WIDTH ( DATA_WIDTH ) ) input_prefifo  ( .clk ( clk_i ) );
  hwpe_stream_intf_stream #( .DATA_WIDTH ( DATA_WIDTH ) ) kernel_prefifo  ( .clk ( clk_i ) );
  hwpe_stream_intf_stream #( .DATA_WIDTH ( DATA_WIDTH ) ) output_postfifo ( .clk ( clk_i ) );

  hci_core_intf #( .DW ( MEM_WIDTH) ) tcdm_fifo [MP-1:0] ( .clk ( clk_i ) );


  hci_core_source #(
    .MISALIGNED_ACCESSES( 0                      )
  ) i_input_source          (
    .clk_i              ( clk_i                  ),
    .rst_ni             ( rst_ni                 ),
    .test_mode_i        ( test_mode_i            ),
    .clear_i            ( clear_i                ),
    .enable_i           ( enable_i               ),
    .tcdm               ( tcdm_fifo[INPUT_STREAM_IDX]),
    .stream             ( input_mem                  ),
    .ctrl_i             ( ctrl_i.input_source_ctrl   ),
    .flags_o            ( flags_o.input_source_flags )
  );


  hci_core_source #(
    .MISALIGNED_ACCESSES( 0                      )
  ) i_kernel_source          (
    .clk_i              ( clk_i                  ),
    .rst_ni             ( rst_ni                 ),
    .test_mode_i        ( test_mode_i            ),
    .clear_i            ( clear_i                ),
    .enable_i           ( enable_i               ),
    .tcdm               ( tcdm_fifo[KERNEL_STREAM_IDX]),
    .stream             ( kernel_mem                  ),
    .ctrl_i             ( ctrl_i.kernel_source_ctrl   ),
    .flags_o            ( flags_o.kernel_source_flags )
  );


  hci_core_sink #(
    .MISALIGNED_ACCESSES( 0                      )
  ) i_output_sink            (
    .clk_i              ( clk_i                  ),
    .rst_ni             ( rst_ni                 ),
    .test_mode_i        ( test_mode_i            ),
    .clear_i            ( clear_i                ),
    .enable_i           ( enable_i               ),
    .tcdm               ( tcdm_fifo[OUTPUT_STREAM_IDX]),
    .stream             ( output_mem                  ),
    .ctrl_i             ( ctrl_i.output_sink_ctrl     ),
    .flags_o            ( flags_o.output_sink_flags   )
  );


  hci_core_fifo #(
    .FIFO_DEPTH     ( 2                      )
  ) i_input_tcdm_fifo   (
    .clk_i          ( clk_i                  ),
    .rst_ni         ( rst_ni                 ),
    .clear_i        ( clear_i                ),
    .flags_o        (                        ),
    .tcdm_target    ( tcdm_fifo[INPUT_STREAM_IDX]),
    .tcdm_initiator ( tcdm[INPUT_STREAM_IDX]     )
  );
  // kernel fifo
  hci_core_fifo #(
    .FIFO_DEPTH     ( 2                      )
  ) i_kernel_tcdm_fifo   (
    .clk_i          ( clk_i                  ),
    .rst_ni         ( rst_ni                 ),
    .clear_i        ( clear_i                ),
    .flags_o        (                        ),
    .tcdm_target    ( tcdm_fifo[KERNEL_STREAM_IDX]),
    .tcdm_initiator ( tcdm[KERNEL_STREAM_IDX]     )
  );
  // output fifo
  hci_core_fifo #(
    .FIFO_DEPTH     ( 2                      )
  ) i_output_tcdm_fifo   (
    .clk_i          ( clk_i                  ),
    .rst_ni         ( rst_ni                 ),
    .clear_i        ( clear_i                ),
    .flags_o        (                        ),
    .tcdm_target    ( tcdm_fifo[OUTPUT_STREAM_IDX]),
    .tcdm_initiator ( tcdm[OUTPUT_STREAM_IDX]     )
  ); 

  hwpe_stream_split #(
    .NB_OUT_STREAMS ( MEM_WIDTH/DATA_WIDTH ),
    .DATA_WIDTH_IN  ( MEM_WIDTH            )
  ) i_input_split       (
    .clk_i          ( clk_i                ),
    .rst_ni         ( rst_ni               ),
    .clear_i        ( clear_i              ),
    .push_i         ( input_mem                ),
    .pop_o          ( input_split              )
  );
  // input split FIFO -- necessary to avoid deadlocks with serializer in SYNC_READY mode
  for(genvar ii=0; ii<MEM_WIDTH/DATA_WIDTH; ii++) begin : input_split_fifo_gen
    hwpe_stream_fifo #(
      .FIFO_DEPTH   ( 2                      ),
      .DATA_WIDTH   ( DATA_WIDTH             )
    ) i_input_split_fifo(
      .clk_i        ( clk_i                  ),
      .rst_ni       ( rst_ni                 ),
      .clear_i      ( clear_i                ),
      .flags_o      (                        ),
      .push_i       ( input_split[ii]            ),
      .pop_o        ( input_split_postfifo[ii]   )
    );
  end

  hwpe_stream_serialize #(
    .NB_IN_STREAMS ( MEM_WIDTH/DATA_WIDTH    ),
    .DATA_WIDTH    ( DATA_WIDTH              ),
    .SYNC_READY    ( 1'b1                    )
  ) i_input_serialize  (
    .clk_i         ( clk_i                   ),
    .rst_ni        ( rst_ni                  ),
    .clear_i       ( clear_i                 ),
    .ctrl_i        ( ctrl_i.input_serialize_ctrl ),
    .push_i        ( input_split_postfifo        ),
    .pop_o         ( input_prefifo               )
  );

  // kernel split: same role as input split above, but for the kernel load stream.
  hwpe_stream_split #(
    .NB_OUT_STREAMS ( MEM_WIDTH/DATA_WIDTH ),
    .DATA_WIDTH_IN  ( MEM_WIDTH            )
  ) i_kernel_split       (
    .clk_i          ( clk_i                ),
    .rst_ni         ( rst_ni               ),
    .clear_i        ( clear_i              ),
    .push_i         ( kernel_mem                ),
    .pop_o          ( kernel_split              )
  );
  // kernel split FIFO -- necessary to avoid deadlocks with serializer in SYNC_READY mode
  for(genvar ii=0; ii<MEM_WIDTH/DATA_WIDTH; ii++) begin : kernel_split_fifo_gen
    hwpe_stream_fifo #(
      .FIFO_DEPTH    ( 2                   ),
      .DATA_WIDTH    ( DATA_WIDTH          )
    ) i_kernel_split_fifo (
      .clk_i         ( clk_i               ),
      .rst_ni        ( rst_ni              ),
      .clear_i       ( clear_i             ),
      .flags_o       (                     ),
      .push_i        ( kernel_split[ii]         ),
      .pop_o         ( kernel_split_postfifo[ii])
    );
  end
  // kernel serialize -- the serializer requires SYNC_READY mode to deal with streams
  // generated by the same producer (here: the kernel source)
  hwpe_stream_serialize #(
    .NB_IN_STREAMS ( MEM_WIDTH/DATA_WIDTH    ),
    .DATA_WIDTH    ( DATA_WIDTH              ),
    .SYNC_READY    ( 1'b1                    )
  ) i_kernel_serialize  (
    .clk_i         ( clk_i                   ),
    .rst_ni        ( rst_ni                  ),
    .clear_i       ( clear_i                 ),
    .ctrl_i        ( ctrl_i.kernel_serialize_ctrl ),
    .push_i        ( kernel_split_postfifo        ),
    .pop_o         ( kernel_prefifo               )
  );


  hwpe_stream_deserialize #(
    .NB_OUT_STREAMS ( MEM_WIDTH/DATA_WIDTH      ),
    .DATA_WIDTH     ( DATA_WIDTH                )
  ) i_output_deserialize (
    .clk_i          ( clk_i                     ),
    .rst_ni         ( rst_ni                    ),
    .clear_i        ( clear_i                   ),
    .ctrl_i         ( ctrl_i.output_deserialize_ctrl ),
    .push_i         ( output_postfifo                ),
    .pop_o          ( output_split_prefence          )
  );


  hwpe_stream_fence #(
    .NB_STREAMS  ( MEM_WIDTH/DATA_WIDTH   ),
    .DATA_WIDTH  ( DATA_WIDTH             )
  ) i_output_fence    (
    .clk_i       ( clk_i                  ),
    .rst_ni      ( rst_ni                 ),
    .clear_i     ( clear_i                ),
    .test_mode_i ( 1'b0                   ),
    .push_i      ( output_split_prefence       ),
    .pop_o       ( output_split                )
  ); 


  hwpe_stream_merge #(
    .NB_IN_STREAMS ( MEM_WIDTH/DATA_WIDTH ),
    .DATA_WIDTH_IN ( DATA_WIDTH           )
  ) i_output_merge      (
    .clk_i         ( clk_i                ),
    .rst_ni        ( rst_ni               ),
    .clear_i       ( clear_i              ),
    .push_i        ( output_split              ),
    .pop_o         ( output_mem                )
  );


  hwpe_stream_fifo #(
    .DATA_WIDTH( DATA_WIDTH ),
    .FIFO_DEPTH( 2          ),
    .LATCH_FIFO( 0          )
  ) i_input_fifo   (
    .clk_i     ( clk_i      ),
    .rst_ni    ( rst_ni     ),
    .clear_i   ( clear_i    ),
    .push_i    ( input_prefifo  ),
    .pop_o     ( input_o        ), 
    .flags_o   (            )
  );

  hwpe_stream_fifo #(
    .DATA_WIDTH( DATA_WIDTH ),
    .FIFO_DEPTH( 2          ),
    .LATCH_FIFO( 0          )
  ) i_kernel_fifo   (
    .clk_i     ( clk_i      ),
    .rst_ni    ( rst_ni     ),
    .clear_i   ( clear_i    ),
    .push_i    ( kernel_prefifo  ),
    .pop_o     ( kernel_o        ), // -> dimc_datapath.kernel
    .flags_o   (            )
  );


  hwpe_stream_fifo #(
    .DATA_WIDTH( DATA_WIDTH ),
    .FIFO_DEPTH( 2          ),
    .LATCH_FIFO( 0          )
  ) i_output_fifo   (
    .clk_i     ( clk_i      ),
    .rst_ni    ( rst_ni     ),
    .clear_i   ( clear_i    ),
    .push_i    ( output_i        ),
    .pop_o     ( output_postfifo ),
    .flags_o   (            )
  );

endmodule // dimc_streamer
