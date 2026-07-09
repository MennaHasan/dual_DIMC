/* 
 * dimc_ctrl.sv
 */

module dimc_ctrl
  import dimc_package::*;
  import hwpe_ctrl_package::*;
#(
  parameter int unsigned N_CORES        = 2,  // number of CPU cores allowed to program/start this HWPE
  parameter int unsigned N_CONTEXT      = 2,  // number of job "contexts" the register file can hold (double-buffering: one running + one queued)
  parameter int unsigned NB_KERNEL_ROWS = 32, // kernel rows per job (must match dimc_top/dimc_datapath's NB_KERNEL_ROWS)
  parameter int unsigned ID             = 10  // ID width for outstanding memory transactions
)
(
  // global signals
  input  logic                                  clk_i,
  input  logic                                  rst_ni,
  input  logic                                  test_mode_i,
  output logic                                  clear_o,

  output logic [N_CORES-1:0][REGFILE_N_EVT-1:0] evt_o,

  output dimc_streamer_ctrl_t                    streamer_ctrl_o,
  input  dimc_streamer_flags_t                   streamer_flags_i,

  hwpe_ctrl_intf_periph.slave                   periph
);


  hwpe_ctrl_package::ctrl_slave_t   slave_ctrl;
  hwpe_ctrl_package::flags_slave_t  slave_flags;
  hwpe_ctrl_package::ctrl_regfile_t reg_file;

  // NUM_SECTIONS: 256-bit sections per kernel row / feature vector (=4,
  // matches dimc_datapath's own localparam of the same name -- both are
  // derived from the same fixed 1024-bit row/vector width).
  localparam int unsigned NUM_SECTIONS       = 4;
  localparam int unsigned NB_KERNEL_SECTIONS = NB_KERNEL_ROWS * NUM_SECTIONS;

  dimc_config_t config_;
  
  dimc_fsm_state_t state_d, state_q;

  hwpe_ctrl_slave #(
    .N_CORES        ( N_CORES     ),
    .N_CONTEXT      ( N_CONTEXT   ),
    .N_IO_REGS      ( DIMC_NB_REGS ),
    .N_GENERIC_REGS ( 0           ),
    .ID_WIDTH       ( ID          )
  ) i_slave (
    .clk_i    ( clk_i       ),
    .rst_ni   ( rst_ni      ),
    .clear_o  ( clear_o     ),
    .cfg      ( periph      ),
    .ctrl_i   ( slave_ctrl  ),
    .flags_o  ( slave_flags ),
    .reg_file ( reg_file    )
  );
  
   assign slave_ctrl.done = (state_d == FSM_IDLE && state_q == FSM_COMPUTE) ? 1'b1 : 1'b0;
  assign slave_ctrl.evt  = (state_d == FSM_IDLE && state_q == FSM_COMPUTE) ? 1'b1 : 1'b0;
  assign slave_ctrl.ext_flags = '0;
  assign evt_o = slave_flags.evt;

  assign config_.input_addr = reg_file.hwpe_params[DIMC_REG_INPUT_ADDR];
  assign config_.kernel_addr = reg_file.hwpe_params[DIMC_REG_KERNEL_ADDR];
  assign config_.output_addr = reg_file.hwpe_params[DIMC_REG_OUTPUT_ADDR];
  
  
  assign config_.signal_length = reg_file.hwpe_params[DIMC_REG_LENGTH][31:16];

  always_ff @(posedge clk_i or negedge rst_ni)
  begin : main_fsm_seq
    if(~rst_ni) begin
      state_q <= FSM_IDLE;
    end
    else if(clear_o) begin
      state_q <= FSM_IDLE;
    end
    else begin
      state_q <= state_d;
    end
  end

  always_comb
  begin : main_fsm_comb
    state_d = state_q;
    case(state_q)
      FSM_IDLE: begin
        if(slave_flags.start) begin
          state_d = FSM_COMPUTE;
        end
      end
      FSM_COMPUTE: begin
        if(streamer_flags_i.output_sink_flags.done) begin
          state_d = FSM_IDLE;
        end
      end
    endcase
  end


  // Input stream
  assign streamer_ctrl_o.input_source_ctrl.req_start = (state_d == FSM_COMPUTE && state_q == FSM_IDLE) ? 1'b1 : 1'b0;
  assign streamer_ctrl_o.input_source_ctrl.addressgen_ctrl.base_addr = config_.input_addr; // directly from configuration
  assign streamer_ctrl_o.input_source_ctrl.addressgen_ctrl.tot_len   = NUM_SECTIONS; // one feature vector = 4 sections (MVP: fixed, see signal_length note above)
  assign streamer_ctrl_o.input_source_ctrl.addressgen_ctrl.d0_len    = NUM_SECTIONS;
  assign streamer_ctrl_o.input_source_ctrl.addressgen_ctrl.d0_stride = 32; // 32 bytes per 256-bit section
  assign streamer_ctrl_o.input_source_ctrl.addressgen_ctrl.d1_len    = 1;
  assign streamer_ctrl_o.input_source_ctrl.addressgen_ctrl.d1_stride = 0;
  assign streamer_ctrl_o.input_source_ctrl.addressgen_ctrl.d2_stride = 0;
  assign streamer_ctrl_o.input_source_ctrl.addressgen_ctrl.dim_enable_1h = hwpe_stream_package::HWPE_STREAM_ADDRESSGEN_1D; // single-dimension counting

  assign streamer_ctrl_o.input_serialize_ctrl.first_stream       = 0;
  assign streamer_ctrl_o.input_serialize_ctrl.clear_serdes_state = FSM_IDLE;
  assign streamer_ctrl_o.input_serialize_ctrl.nb_contig_m1       = 0;

  // Kernel stream
  assign streamer_ctrl_o.kernel_source_ctrl.req_start = (state_d == FSM_COMPUTE && state_q == FSM_IDLE) ? 1'b1 : 1'b0;
  assign streamer_ctrl_o.kernel_source_ctrl.addressgen_ctrl.base_addr = config_.kernel_addr;
  assign streamer_ctrl_o.kernel_source_ctrl.addressgen_ctrl.tot_len   = NB_KERNEL_SECTIONS; // full kernel = NB_KERNEL_ROWS*4 sections
  assign streamer_ctrl_o.kernel_source_ctrl.addressgen_ctrl.d0_len    = NB_KERNEL_SECTIONS;
  assign streamer_ctrl_o.kernel_source_ctrl.addressgen_ctrl.d0_stride = 32; // 32 bytes per 256-bit section
  assign streamer_ctrl_o.kernel_source_ctrl.addressgen_ctrl.d1_len    = 1;
  assign streamer_ctrl_o.kernel_source_ctrl.addressgen_ctrl.d1_stride = 0;
  assign streamer_ctrl_o.kernel_source_ctrl.addressgen_ctrl.d2_stride = 0;
  assign streamer_ctrl_o.kernel_source_ctrl.addressgen_ctrl.dim_enable_1h = hwpe_stream_package::HWPE_STREAM_ADDRESSGEN_1D; // single-dimension counting

  assign streamer_ctrl_o.kernel_serialize_ctrl.first_stream       = 0;
  assign streamer_ctrl_o.kernel_serialize_ctrl.clear_serdes_state = FSM_IDLE;
  assign streamer_ctrl_o.kernel_serialize_ctrl.nb_contig_m1       = 0;

  // Output stream
  assign streamer_ctrl_o.output_sink_ctrl.req_start = (state_d == FSM_COMPUTE && state_q == FSM_IDLE) ? 1'b1 : 1'b0;
  assign streamer_ctrl_o.output_sink_ctrl.addressgen_ctrl.base_addr = config_.output_addr;
  assign streamer_ctrl_o.output_sink_ctrl.addressgen_ctrl.tot_len   = NB_KERNEL_ROWS; // one result word per kernel row
  assign streamer_ctrl_o.output_sink_ctrl.addressgen_ctrl.d0_len    = NB_KERNEL_ROWS;
  assign streamer_ctrl_o.output_sink_ctrl.addressgen_ctrl.d0_stride = 32; // 32 bytes per 256-bit section
  assign streamer_ctrl_o.output_sink_ctrl.addressgen_ctrl.d1_len    = 1;
  assign streamer_ctrl_o.output_sink_ctrl.addressgen_ctrl.d1_stride = 0;
  assign streamer_ctrl_o.output_sink_ctrl.addressgen_ctrl.d2_stride = 0;
  assign streamer_ctrl_o.output_sink_ctrl.addressgen_ctrl.dim_enable_1h = hwpe_stream_package::HWPE_STREAM_ADDRESSGEN_1D; // single-dimension counting

  assign streamer_ctrl_o.output_deserialize_ctrl.first_stream       = 0;
  assign streamer_ctrl_o.output_deserialize_ctrl.clear_serdes_state = FSM_IDLE;
  assign streamer_ctrl_o.output_deserialize_ctrl.nb_contig_m1       = 0;

endmodule // dimc_ctrl
