/*
 * dimc_package.sv
 */


package dimc_package;


  typedef enum {
    FSM_IDLE,
    FSM_COMPUTE
  } dimc_fsm_state_t;


  parameter int unsigned DIMC_REG_INPUT_ADDR  = 0;
  parameter int unsigned DIMC_REG_KERNEL_ADDR = 1;
  parameter int unsigned DIMC_REG_OUTPUT_ADDR = 2;
  parameter int unsigned DIMC_REG_LENGTH      = 3;
  parameter int unsigned DIMC_NB_REGS         = 4;


  parameter int unsigned INPUT_STREAM_IDX = 0;
  parameter int unsigned KERNEL_STREAM_IDX = 1;
  parameter int unsigned OUTPUT_STREAM_IDX = 2;

  parameter int unsigned NB_KERNEL_ROWS = 32; // kernel rows per DIMC macro (fixed: RA/WA are hardcoded 7 bits = 5-bit row + 2-bit section)

  typedef struct packed {
    logic unsigned [31:0] input_addr;     // memory address of input feature data
    logic unsigned [31:0] kernel_addr;    // memory address of kernel (weight) data
    logic unsigned [31:0] output_addr;    // memory address of output data
    logic unsigned [15:0] signal_length;  // number of 16-bit samples in the input (and produced in the output)
  } dimc_config_t;


  typedef struct packed {
    hci_package::hci_streamer_ctrl_t   input_source_ctrl;
    hci_package::hci_streamer_ctrl_t   kernel_source_ctrl;
    hci_package::hci_streamer_ctrl_t   output_sink_ctrl;
    hwpe_stream_package::ctrl_serdes_t input_serialize_ctrl;
    hwpe_stream_package::ctrl_serdes_t kernel_serialize_ctrl;
    hwpe_stream_package::ctrl_serdes_t output_deserialize_ctrl;
  } dimc_streamer_ctrl_t;


  typedef struct packed {
    hci_package::hci_streamer_flags_t input_source_flags;
    hci_package::hci_streamer_flags_t kernel_source_flags;
    hci_package::hci_streamer_flags_t output_sink_flags;
  } dimc_streamer_flags_t;

endpackage // dimc_package
