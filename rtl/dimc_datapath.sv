/*
 * dimc_datapath.sv
 */
module dimc_datapath
  import dimc_package::*;
#(
  parameter int unsigned SECTION_WIDTH  = 256, // width of one DIMC memory/compute section (must stay 256: spatz_DIMC hardcodes FA/RA/WA bit widths assuming 1024/SECTION_WIDTH == 4 sections per row)
  parameter int unsigned INP_FIFO_DEPTH = 8,
  parameter int unsigned WGT_FIFO_DEPTH = 128,
  parameter int unsigned OUT_FIFO_DEPTH = 64
)
(
  input  logic                 clk_i,
  input  logic                 rst_ni,
  input  logic                 clear_i,

  // input feature stream from dimc_streamer (one SECTION_WIDTH-wide section per beat)
  hwpe_stream_intf_stream.sink   input_i,
  // kernel (weight) stream from dimc_streamer (one SECTION_WIDTH-wide section per beat)
  hwpe_stream_intf_stream.sink   kernel_i,
  // output stream towards dimc_streamer (one SECTION_WIDTH-wide word per beat, holding a sign-extended 32-bit result)
  hwpe_stream_intf_stream.source output_o
);

  localparam int unsigned NUM_SECTIONS       = 1024 / SECTION_WIDTH;         // sections per kernel row / feature vector (=4)
  localparam int unsigned NB_KERNEL_SECTIONS = NB_KERNEL_ROWS * NUM_SECTIONS; // total kernel sections per job (=128)

  // =========================================================================
  // spatz_DIMC_dual instance and its raw FIFO/control ports
  // =========================================================================
  logic                     sel;
  logic                     compe, fcsn, rcsn, rcsn0, rcsn1, rcsn2, rcsn3, wcsn, wen;
  logic [1:0]                mode;
  logic [1:0]                fa;
  logic [23:0]               addin;
  logic [6:0]                ra, wa;
  logic [SECTION_WIDTH-1:0]  m_mask;
  logic [7:0]                mct;

  logic                      readyn;
  logic [23:0]               psout;

  logic                      inp_push, inp_full, inp_empty;
  logic [SECTION_WIDTH-1:0]  inp_data;

  logic                      wgt_push, wgt_full, wgt_empty;
  logic [SECTION_WIDTH-1:0]  wgt_data;

  logic                      out_pop, out_full, out_empty;
  logic [23:0]               out_data;

  spatz_DIMC_dual #(
    .SECTION_WIDTH  ( SECTION_WIDTH  ),
    .INP_FIFO_DEPTH ( INP_FIFO_DEPTH ),
    .WGT_FIFO_DEPTH ( WGT_FIFO_DEPTH ),
    .OUT_FIFO_DEPTH ( OUT_FIFO_DEPTH )
  ) i_dimc_dual (
    .clk        ( clk_i    ),
    .rst_n      ( rst_ni   ),
    .sel        ( sel      ),
    .COMPE_m0   ( compe    ),
    .FCSN_m0    ( fcsn     ),
    .MODE_m0    ( mode     ),
    .FA_m0      ( fa       ),
    .ADDIN_m0   ( addin    ),
    .RA_m0      ( ra       ),
    .WA_m0      ( wa       ),
    .RCSN_m0    ( rcsn     ),
    .RCSN0_m0   ( rcsn0    ),
    .RCSN1_m0   ( rcsn1    ),
    .RCSN2_m0   ( rcsn2    ),
    .RCSN3_m0   ( rcsn3    ),
    .WCSN_m0    ( wcsn     ),
    .WEN_m0     ( wen      ),
    .M_m0       ( m_mask   ),
    .MCT_m0     ( mct      ),
    .COMPE_m1   ( 1'b0     ),
    .FCSN_m1    ( 1'b1     ),
    .MODE_m1    ( '0       ),
    .FA_m1      ( '0       ),
    .ADDIN_m1   ( '0       ),
    .RA_m1      ( '0       ),
    .WA_m1      ( '0       ),
    .RCSN_m1    ( 1'b1     ),
    .RCSN0_m1   ( 1'b1     ),
    .RCSN1_m1   ( 1'b1     ),
    .RCSN2_m1   ( 1'b1     ),
    .RCSN3_m1   ( 1'b1     ),
    .WCSN_m1    ( 1'b1     ),
    .WEN_m1     ( 1'b1     ),
    .M_m1       ( '1       ),
    .MCT_m1     ( '0       ),
    .READYN     ( readyn   ),
    .PSOUT      ( psout    ),
    .inp_push   ( inp_push ),
    .inp_data   ( inp_data ),
    .inp_full   ( inp_full ),
    .inp_empty  ( inp_empty),
    .wgt_push   ( wgt_push ),
    .wgt_data   ( wgt_data ),
    .wgt_full   ( wgt_full ),
    .wgt_empty  ( wgt_empty),
    .out_pop    ( out_pop  ),
    .out_data   ( out_data ),
    .out_full   ( out_full ),
    .out_empty  ( out_empty)
  );

  // =========================================================================
  // Data-plane bridge: HWPE-Stream (valid/ready) <-> push/full/pop/empty
  // =========================================================================
  // input/kernel loads: push whenever the streamer has data and the target
  // FIFO has room; back-pressure the streamer via `ready` otherwise.
  assign inp_push      = input_i.valid & ~inp_full;
  assign inp_data       = input_i.data[SECTION_WIDTH-1:0];
  assign input_i.ready = ~inp_full;

  assign wgt_push       = kernel_i.valid & ~wgt_full;
  assign wgt_data       = kernel_i.data[SECTION_WIDTH-1:0];
  assign kernel_i.ready = ~wgt_full;

  // output store: present a result whenever out_fifo is non-empty; pop it
  // only once the streamer's Y/output sink actually accepts it.
  assign output_o.valid = ~out_empty;
  assign out_pop         = output_o.valid & output_o.ready;
  assign output_o.data   = { {(SECTION_WIDTH-24){out_data[23]}}, out_data };
  assign output_o.strb   = '1;

  // =========================================================================
  // Control-plane sequencer
  // =========================================================================
  typedef enum logic [1:0] {
    LOAD_KERNEL,
    LOAD_FEATURE,
    COMPUTE,
    DONE
  } dimc_datapath_state_t;

  dimc_datapath_state_t state_d, state_q;
  logic [6:0] kernel_cnt_d, kernel_cnt_q; // used as WA value too (0..NB_KERNEL_SECTIONS-1)
  logic [1:0] feat_cnt_d,   feat_cnt_q;   // used as FA too (0..NUM_SECTIONS-1)
  logic [4:0] compute_cnt_d, compute_cnt_q; // used as RA too (0..NB_KERNEL_ROWS-1)
  logic row0_repeated_d, row0_repeated_q; // set once row 0 has been issued twice this compute pass

  always_ff @(posedge clk_i or negedge rst_ni)
  begin : datapath_state_seq
    if (~rst_ni) begin
      state_q <= LOAD_KERNEL;
    end
    else if (clear_i) begin
      state_q <= LOAD_KERNEL;
    end
    else begin
      state_q <= state_d;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni)
  begin : datapath_counters_seq
    if (~rst_ni) begin
      kernel_cnt_q    <= '0;
      feat_cnt_q      <= '0;
      compute_cnt_q   <= '0;
      row0_repeated_q <= 1'b0;
    end
    else if (clear_i) begin
      kernel_cnt_q    <= '0;
      feat_cnt_q      <= '0;
      compute_cnt_q   <= '0;
      row0_repeated_q <= 1'b0;
    end
    else begin
      kernel_cnt_q    <= kernel_cnt_d;
      feat_cnt_q      <= feat_cnt_d;
      compute_cnt_q   <= compute_cnt_d;
      row0_repeated_q <= row0_repeated_d;
    end
  end

  always_comb
  begin : datapath_sequencer_comb
    // Defaults: every control pin idle, no state/counter advance.
    state_d         = state_q;
    kernel_cnt_d    = kernel_cnt_q;
    feat_cnt_d      = feat_cnt_q;
    compute_cnt_d   = compute_cnt_q;
    row0_repeated_d = row0_repeated_q;

    sel   = 1'b0; // single-macro MVP: always target u_mac0
    compe = 1'b0;
    fcsn  = 1'b1;
    mode  = 2'b11; // fixed 8-bit precision (MVP default)
    fa    = '0;
    addin = '0;    // no bias (MVP default)
    ra    = '0;
    wa    = kernel_cnt_q;
    rcsn  = 1'b1;
    rcsn0 = 1'b1;
    rcsn1 = 1'b1;
    rcsn2 = 1'b1;
    rcsn3 = 1'b1;
    wcsn  = 1'b1;
    wen   = 1'b1;
    m_mask = '1;   // full write mask (MVP default)
    mct    = 8'd0; // no masking (MVP default)

    case (state_q)
      // Drives one kernel-section write per cycle.
      LOAD_KERNEL: begin
        if (~wgt_empty) begin
          wcsn = 1'b0;
          wen  = 1'b0;
          kernel_cnt_d = kernel_cnt_q + 1'b1;     // wa is set to equal it - updated here 
          if (kernel_cnt_q == NB_KERNEL_SECTIONS-1) begin
            state_d = LOAD_FEATURE;
          end
        end
      end

      LOAD_FEATURE: begin
        fa = feat_cnt_q;
        if (~inp_empty) begin
          fcsn = 1'b0;
          feat_cnt_d = feat_cnt_q + 1'b1;
          if (feat_cnt_q == NUM_SECTIONS-1) begin
            state_d = COMPUTE;
          end
        end
      end

      COMPUTE: begin
        ra    = {compute_cnt_q, 2'b00};
        compe = 1'b1;
        rcsn  = 1'b0;
        rcsn0 = 1'b0;
        rcsn1 = 1'b0;
        rcsn2 = 1'b0;
        rcsn3 = 1'b0;
        // this if condition is added to add an extra cycle after row 0 requested
        // to avoid race between row 0 and row 1 outputs
        if (compute_cnt_q == 0 && ~row0_repeated_q) begin
          compute_cnt_d   = compute_cnt_q; // repeat row 0 once more
          row0_repeated_d = 1'b1;
        end
        else begin
          compute_cnt_d = compute_cnt_q + 1'b1;
        end
        if (compute_cnt_q == NB_KERNEL_ROWS-1) begin
          state_d = DONE;
        end
      end

      // Idle until clear_i is triggered 
      // The output bridge keeps draining out_fifo in the
      DONE: begin
      end
    endcase
  end

endmodule // dimc_datapath
