// DIMC.sv created

`timescale 1ns/1ps

    module DIMC_18_fixed #(
    
 	    //Parameter for Section Width
    parameter SECTION_WIDTH = 256   // can be 256, 512, or 1024
)(

    // System Interface
    input  logic        RCK,        // Main clock
    input  logic        RESETn,     // Active-low reset
    
    // Control Signals
    output logic        READYN,     // Active-low ready (output valid)
    input  logic        COMPE,      // Operation mode (1=compute, -
    input  logic        FCSN,       // Feature buffer chip select (active-low)
    input  logic [1:0]  MODE,       // Bit resolution (0=1b, 1=2b, 2=4b, 3=8b)
    
    // Address/Data Interface
    input  logic [1:0]                  FA,        // Feature buffer address
    input  logic [SECTION_WIDTH-1:0]    FD,        // Feature buffer data
    input  logic [23:0]                 ADDIN,     // Bias/partial sum input
    output logic                        SOUT,      // Sum output (LSB of result)
    output logic [2:0]                  RES_OUT,   // Result output (MSBs of 4-bit result)
    output logic [23:0]                 PSOUT,     // Pre-ReLU output
    output logic [SECTION_WIDTH-1:0]    Q,         // Memory output
    input  logic [SECTION_WIDTH-1:0]    D,         // Memory input
    input  logic [6:0]                  RA,        // Memory address
    input  logic [6:0]                  WA,        // Write address
    
    // Memory Control
    input  logic        RCSN,       // Read chip select (active-low)
    input  logic        RCSN0,      // Computation control
    input  logic        RCSN1,      // Computation control
    input  logic        RCSN2,      // Computation control
    input  logic        RCSN3,      // Computation control
    input  logic        WCK,        // Write clock
    input  logic        WCSN,       // Write chip select (active-low)
    input  logic        WEN,        // Write enable (active-low)
    
    // Masking Signals
    input  logic [SECTION_WIDTH-1:0]    M,         // Bitwise write mask
    input  logic [7:0]                  MCT        // Masking coding thermometric
    
   
);


//------------------------------------------------------------------------------
// Derived Parameters
//------------------------------------------------------------------------------

localparam NUM_SECTIONS = 1024/SECTION_WIDTH;
localparam ROW_WIDTH = NUM_SECTIONS * SECTION_WIDTH;

//------------------------------------------------------------------------------
// Memory Architecture
//------------------------------------------------------------------------------
logic [NUM_SECTIONS-1:0][SECTION_WIDTH-1:0] kernel_mem [31:0];  // 32 rows, 4 sections, 256 bits each
logic [NUM_SECTIONS-1:0][SECTION_WIDTH-1:0] feature_buf;       // 4 sections, 256 bits each

//------------------------------------------------------------------------------
// Control Signals and Registers
//------------------------------------------------------------------------------
logic compute_trigger;
logic mem_read_en;
logic mem_write_en;
logic [10:0] valid_bits;

// Pipeline registers
logic [3:0]  pipeline_valid;  // Valid flag for each stage
logic [4:0]  pipeline_row [0:3];
logic [23:0] pipeline_bias [0:3];
logic [ROW_WIDTH-1:0] pipeline_kernel [0:3];
logic [ROW_WIDTH-1:0] pipeline_feature [0:3];
logic [23:0]  pipeline_result [0:3];
logic [1:0]  pipeline_mode [0:3];

// Computation intermediates
logic [ROW_WIDTH-1:0] masked_kernel;
logic [ROW_WIDTH-1:0] masked_feature;
logic [23:0] comp_result;

// Output logic
logic [23:0] psum;
logic [3:0] result_4bit;


//------------------------------------------------------------------------------
// Combinational Logic Blocks
//------------------------------------------------------------------------------

// Control signal assignments
assign compute_trigger = COMPE & ~RCSN & ~RCSN0 & ~RCSN1 & ~RCSN2 & ~RCSN3;
assign mem_read_en     = ~COMPE & ~RCSN;
assign mem_write_en    = ~COMPE & ~WCSN & ~WEN;
//assign WCK = RCK;  // Write clock tied to main clock

// Valid bits calculation
always_comb begin
    valid_bits = 1024 - (MCT * 4);
    if (valid_bits > 1024) valid_bits = 0;
end

// Stage 1: Masking logic
always_comb begin
    masked_kernel = pipeline_kernel[0];
    masked_feature = pipeline_feature[0];
    
    for (int i = 0; i < 1024; i++) begin
        if (i >= valid_bits) begin
            masked_kernel[i] = 0;
            masked_feature[i] = 0;
        end
    end
end

// Stage 2: Computation logic
always_comb begin
    comp_result = 0;
    
    case (pipeline_mode[1])
        // 1-bit Mode: XNOR + Popcount
        2'b00: begin
            logic [ROW_WIDTH-1:0] xnor_result;
            logic [10:0] popcount;
            
            xnor_result = ~(masked_kernel ^ masked_feature);
            popcount = $countones(xnor_result);
            comp_result = popcount;
        end
        
        // 2-bit Mode: Vector multiplication
        2'b01: begin
            for (int i = 0; i < 512; i++) begin
                automatic logic [1:0] k_val = masked_kernel[i*2 +: 2];
                automatic logic [1:0] f_val = masked_feature[i*2 +: 2];
                comp_result += k_val * f_val;
            end
        end
        
        // 4-bit Mode: Vector multiplication
        2'b10: begin
            for (int i = 0; i < 256; i++) begin
                automatic logic [3:0] k_val = masked_kernel[i*4 +: 4];
                automatic logic [3:0] f_val = masked_feature[i*4 +: 4];
                comp_result += k_val * f_val;
            end
        end
        
        // Default: 8-bit Mode (vector multiplication)
        default: begin
             for (int i = 0; i < ROW_WIDTH/8; i++) begin
                 automatic logic [7:0] k_val = masked_kernel[i*8 +: 8];
                 automatic logic [7:0] f_val = masked_feature[i*8 +: 8];
                 comp_result += k_val * f_val;
             end
         end
   endcase
end

// Stage 3: Output processing
always_comb begin
    psum = pipeline_result[2] + pipeline_bias[2];
    
    // ReLU + 4-bit quantization
    if (psum[23]) begin          // Negative value
        result_4bit = 4'b0;
    end
    else if (|psum[23:4]) begin  // Value > 15
        result_4bit = 4'b1111;
    end
    else begin                   // Value 0-15
        result_4bit = psum[3:0];
    end
end

//------------------------------------------------------------------------------
// Sequential Logic Blocks
//------------------------------------------------------------------------------


// Memory Mode Operations
always_ff @(posedge RCK or negedge RESETn) begin
    if (!RESETn) begin
        feature_buf <= '{default:'0};
        Q <= '0;
        kernel_mem <= '{default:'{default:'0}};
    end
    else begin
        // Feature buffer loading
        if (~FCSN) feature_buf[FA] <= FD;
        
        // Memory read
        if (mem_read_en) Q <= kernel_mem[RA[6:2]][RA[1:0]];
        
        // Memory write
        if (mem_write_en) begin
            for (int i = 0; i < 256; i++) begin
                if (M[i]) kernel_mem[WA[6:2]][WA[1:0]][i] <= D[i];
            end
        end
    end
end

// Pipeline Stage 0: Input
always_ff @(posedge RCK or negedge RESETn) begin
    if (!RESETn) begin
        pipeline_valid[0] <= 0;
        pipeline_row[0] <= 0;
        pipeline_bias[0] <= 0;
        pipeline_mode[0] <= 0;
        pipeline_kernel[0] <= 0;
        pipeline_feature[0] <= 0;
    end
    else begin
        pipeline_valid[0] <= compute_trigger;

        if (compute_trigger) begin
            pipeline_row[0]   <= RA[6:2];  // 5-bit row index
            pipeline_bias[0]  <= ADDIN;
            pipeline_mode[0]  <= MODE;

            if (SECTION_WIDTH == 256) begin
                pipeline_kernel[0] <= {
                    kernel_mem[RA[6:2]][3],
                    kernel_mem[RA[6:2]][2],
                    kernel_mem[RA[6:2]][1],
                    kernel_mem[RA[6:2]][0]
                };
                pipeline_feature[0] <= {
                    feature_buf[3],
                    feature_buf[2],
                    feature_buf[1],
                    feature_buf[0]
                };
            end
            else if (SECTION_WIDTH == 512) begin
                pipeline_kernel[0] <= {
                    kernel_mem[RA[6:2]][1],
                    kernel_mem[RA[6:2]][0]
                };
                pipeline_feature[0] <= {
                    feature_buf[1],
                    feature_buf[0]
                };
            end
            else if (SECTION_WIDTH == 1024) begin
                pipeline_kernel[0] <= kernel_mem[RA[6:2]][0];
                pipeline_feature[0] <= feature_buf[0];
            end
            else begin
                pipeline_kernel[0] <= '0;  // default/fallback
                pipeline_feature[0] <= '0;
            end
        end
    end
end

// Pipeline Stage 1: Masking
always_ff @(posedge RCK or negedge RESETn) begin
    if (!RESETn) begin
        pipeline_valid[1] <= 0;
        pipeline_row[1] <= 0;
        pipeline_bias[1] <= 0;
        pipeline_mode[1] <= 0;
        pipeline_kernel[1] <= 0;
        pipeline_feature[1] <= 0;
    end
    else begin
        pipeline_valid[1] <= pipeline_valid[0];
        pipeline_row[1]   <= pipeline_row[0];
        pipeline_bias[1]  <= pipeline_bias[0];
        pipeline_mode[1]  <= pipeline_mode[0];
        pipeline_kernel[1] <= masked_kernel;
        pipeline_feature[1] <= masked_feature;
    end
end

// Pipeline Stage 2: Computation
always_ff @(posedge RCK or negedge RESETn) begin
    if (!RESETn) begin
        pipeline_valid[2] <= 0;
        pipeline_row[2] <= 0;
        pipeline_bias[2] <= 0;
        pipeline_mode[2] <= 0;
        pipeline_result[2] <= 0;
    end
    else begin
        pipeline_valid[2] <= pipeline_valid[1];
        pipeline_row[2]   <= pipeline_row[1];
        pipeline_bias[2]  <= pipeline_bias[1];
        pipeline_mode[2]  <= pipeline_mode[1];
        pipeline_result[2] <= comp_result;
    end
end

// Pipeline Stage 3: Output
always_ff @(posedge RCK or negedge RESETn) begin
    if (!RESETn) begin
        pipeline_valid[3] <= 0;
        pipeline_row[3] <= 0;
        PSOUT <= 0;
        {RES_OUT, SOUT} <= 0;
        READYN <= 1;
    end
    else begin
        pipeline_valid[3] <= pipeline_valid[2];
        pipeline_row[3]   <= pipeline_row[2];
        
        if (pipeline_valid[2]) begin 
            PSOUT <= psum;
            SOUT <= result_4bit[0];
            RES_OUT <= result_4bit[3:1];
            READYN <= 0;
        end
        else begin
            READYN <= 1;
        end
    end
end

endmodule
