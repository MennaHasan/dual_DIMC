/*
 * accumulator.sv
*/

module accumulator
#(
  parameter int unsigned DATA_WIDTH = 24,
  parameter int unsigned OUT_WIDTH  = 24
)
(
  // global signals
  input  logic                        clk_i,
  input  logic                        rst_ni,
  // local enable & clear
  input  logic                        enable_i,
  input  logic                        clear_i,

  input  logic signed [DATA_WIDTH-1:0] data_i,
  output logic signed [OUT_WIDTH-1:0]  acc_o
);

  logic signed [OUT_WIDTH-1:0] acc_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if(~rst_ni) begin
      acc_q <= '0;
    end
    else if(clear_i) begin
      acc_q <= '0;
    end
    else if(enable_i) begin
      acc_q <= acc_q + data_i;
    end
  end

  assign acc_o = acc_q;

endmodule // accumulator
