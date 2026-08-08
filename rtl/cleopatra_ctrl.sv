/*
 * cleopatra_ctrl.sv
 *
 * Generates a one-hot accumulator selector vector that advances one bit per
 * clock edge. At reset, bit 0 is asserted and all others are deasserted.
 */

module cleopatra_ctrl #(
    parameter int unsigned WIDTH = 256
)(
    input  logic clk,
    input  logic rst_n,
    input  logic clear,
    input  logic out_pop,
    output logic [WIDTH-1:0] acc_sel_o
);

    logic [7:0] idx_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            idx_q <= 8'd0;
        end else if (clear) begin
            idx_q <= 8'd0;
        end else begin
            if (out_pop)
                idx_q <= idx_q + 1'b1;
        end
    end

    // Drive the one-hot selector combinationally from the current idx_q
    always_comb begin
        acc_sel_o = '0;
        acc_sel_o[idx_q] = 1'b1;
    end

endmodule
