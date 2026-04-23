`timescale 1ns/1ps

//==============================================================================
// uart_top (board-level top)  -- STREAM readback version
//------------------------------------------------------------------------------
// Clock plan:
//   i_clk (100 MHz) ──► clk_wiz_0 ──► clk_ddr3_100 (100 MHz, MIG sys_clk_i)
//                                  └► clk_ref_200  (200 MHz, MIG clk_ref_i)
//                                  └► pll_locked
//                                            │
//                                            ▼
//                                      ddr3_top ──► ui_clk (100 MHz)
//                                                └► ui_clk_sync_rst
//
//
// Register map (NUM_REGS=64):
//   reg[0..3]    AD9833 (cmd / freq / phase_ctrl / control)
//   reg[4..57]   ADC core (reg[57]=ctrl)
//   reg[58]      STREAM trigger — a READ op to this address kicks a stream
//                of the whole written DDR3 region out to UART TX.
//   reg[59]      bytes_written status (STATUS overlay, 32-bit read)
//                Layout: {3'b0, bytes_written[28:0]}
//
// Stream protocol (host-side):
//   1) send op byte {1'b1, 7'd59}    → receive 4 bytes == bytes_written MSB-first
//   2) send op byte {1'b1, 7'd58}    → receive EXACTLY `bytes_written` bytes
//                                       (MSB-first, 16 bytes per DDR3 word,
//                                        N = bytes_written / 16 words total)
//==============================================================================
module uart_top #(
    parameter int DATA_WIDTH         = 8,
    parameter int RX_FIFO_DEPTH      = 8,
    parameter int RX_FIFO_AF_DEPTH   = 6,
    parameter int RX_FIFO_AE_DEPTH   = 2,
    parameter int TX_FIFO_DEPTH      = 8,
    parameter int TX_FIFO_AF_DEPTH   = 6,
    parameter int TX_FIFO_AE_DEPTH   = 2,
    parameter int NUM_REGS           = 64,

    // AD9833 register map
    parameter int AD9833_BASE_ADDR   = 0,
    parameter int SPI_FRAME_W        = 16,
    parameter int SPI_CLK_DIV        = 65535,

    // ADC core / top register map
    parameter int ADC_CORE_BASE_ADDR = 4,
    parameter int ADC_CORE_NUM_REGS  = 54,
    parameter int ADC_CTRL_REG_IDX   = 53,

    // DDR3 readback register map
    parameter int STREAM_REG_IDX     = 58,
    parameter int STATUS_REG_IDX     = 59,

    // ADC front-end (sine model)
    parameter int ADC_DATA_W         = 16,
    parameter int ADC_LUT_DEPTH      = 256,
    parameter int ADC_SAMPLE_GAP     = 8,
    parameter int ADC_PHASE_STEP     = 4,

    // DDR3 user interface
    parameter int                     DDR_DATA_W     = 128,
    parameter int                     DDR_ADDR_W     = 28,
    parameter int                     P_WR_BURST_LEN = 8,
    parameter int                     P_WR_BURST_NUM = 1,

    parameter int                     P_RD_BURST_LEN = 8,
    parameter int                     P_RD_BURST_NUM = 1,
    parameter int                     WR_FIFO_DEPTH  = 64,
    parameter int                     RD_FIFO_DEPTH  = 64,
    parameter logic [DDR_ADDR_W-1:0]  DDR_BASE_ADDR  = '0
)(
    //==========================================================================
    // Board IO
    //==========================================================================
    input  logic                         i_clk,         // 100 MHz board clock
    input  logic                         i_rst_n,       // active-low reset

    // UART
    input  logic                         i_rx,
    output logic                         o_tx,

    // AD9833 SPI
    output logic                         o_spi_sclk,
    output logic                         o_spi_fsync,
    output logic                         o_spi_mosi,

    //==========================================================================
    // DDR3 PHY pins (to the chip)
    //==========================================================================
    output logic [13:0]                  ddr3_addr,
    output logic [2:0]                   ddr3_ba,
    output logic                         ddr3_cas_n,
    output logic [0:0]                   ddr3_ck_n,
    output logic [0:0]                   ddr3_ck_p,
    output logic [0:0]                   ddr3_cke,
    output logic                         ddr3_ras_n,
    output logic                         ddr3_reset_n,
    output logic                         ddr3_we_n,
    inout  wire  [15:0]                  ddr3_dq,
    inout  wire  [1:0]                   ddr3_dqs_n,
    inout  wire  [1:0]                   ddr3_dqs_p,
    output logic [0:0]                   ddr3_cs_n,
    output logic [1:0]                   ddr3_dm,
    output logic [0:0]                   ddr3_odt,


    //==========================================================================
    output logic                         o_init_calib_complete
);

    //==========================================================================
    // Clock generation 
    //==========================================================================
    logic clk_ddr3_100;    // 100 MHz -> MIG sys_clk_i
    logic clk_ref_200;     // 200 MHz -> MIG clk_ref_i
    logic pll_locked;

    clk_wiz_0 sys_clk_gen (
        .clk_out1 (clk_ddr3_100),   // 100 MHz
        .clk_out2 (clk_ref_200),    // 200 MHz
        .resetn   (i_rst_n),
        .locked   (pll_locked),
        .clk_in1  (i_clk)
    );

    //==========================================================================
    // ui_clk / ui_clk_sync_rst come out of ddr3_top (MIG)
    //==========================================================================
    logic w_ui_clk;
    logic w_ui_clk_sync_rst;        // active-high, synchronous to w_ui_clk
    logic w_init_calib_complete;
    logic w_mmcm_locked;

    assign o_init_calib_complete = w_init_calib_complete;

    //==========================================================================
    // User-domain reset 
    //==========================================================================
    logic w_user_rst;
    logic w_user_rst_n;

    assign w_user_rst   =  w_ui_clk_sync_rst;
    assign w_user_rst_n = ~w_ui_clk_sync_rst;

    //==========================================================================
    // DDR3 user-side signals
    //==========================================================================
    // WRITE port (driven by adc_top)
    logic                    w_user_wr_valid;
    logic [DDR_ADDR_W-1:0]   w_user_wr_addr_base;
    logic                    w_user_wr_data_valid;
    logic [DDR_DATA_W-1:0]   w_user_wr_data;
    logic                    w_user_wr_finish;
    logic                    w_user_wr_fifo_ready;

    // READ port (driven by ddr3_reader)
    logic                    w_user_rd_valid;
    logic [DDR_ADDR_W-1:0]   w_user_rd_addr_base;
    logic                    w_user_rd_finish;
    logic                    w_user_rd_data_valid;
    logic [DDR_DATA_W-1:0]   w_user_rd_data;

    //==========================================================================
    // DDR3 controller (ddr3_top)
    //==========================================================================
    ddr3_top #(
        .DDR3_WITH      (DDR_DATA_W),
        .WR_FIFO_DEPTH  (WR_FIFO_DEPTH),
        .RD_FIFO_DEPTH  (RD_FIFO_DEPTH),
        .P_WR_BURST_LEN (P_WR_BURST_LEN),
        .P_WR_BURST_NUM (P_WR_BURST_NUM),
        .P_RD_BURST_LEN (P_RD_BURST_LEN),
        .P_RD_BURST_NUM (P_RD_BURST_NUM)
    ) u_ddr3 (
        .ddr3_addr            (ddr3_addr),
        .ddr3_ba              (ddr3_ba),
        .ddr3_cas_n           (ddr3_cas_n),
        .ddr3_ck_n            (ddr3_ck_n),
        .ddr3_ck_p            (ddr3_ck_p),
        .ddr3_cke             (ddr3_cke),
        .ddr3_ras_n           (ddr3_ras_n),
        .ddr3_reset_n         (ddr3_reset_n),
        .ddr3_we_n            (ddr3_we_n),
        .ddr3_dq              (ddr3_dq),
        .ddr3_dqs_n           (ddr3_dqs_n),
        .ddr3_dqs_p           (ddr3_dqs_p),
        .ddr3_cs_n            (ddr3_cs_n),
        .ddr3_dm              (ddr3_dm),
        .ddr3_odt             (ddr3_odt),

        .i_clk_ddr3           (clk_ddr3_100),
        .i_clk_ref            (clk_ref_200),
        .i_clk_locked         (pll_locked),
        .sys_rst_n            (i_rst_n),

        .ui_clk               (w_ui_clk),
        .ui_clk_sync_rst      (w_ui_clk_sync_rst),

        .i_user_wr_valid      (w_user_wr_valid),
        .i_user_wr_addr_base  (w_user_wr_addr_base),
        .i_user_wr_data_valid (w_user_wr_data_valid),
        .i_user_wr_data       (w_user_wr_data),
        .o_user_wr_finish     (w_user_wr_finish),
        .o_user_wr_fifo_ready (w_user_wr_fifo_ready),

        .i_user_rd_valid      (w_user_rd_valid),
        .i_user_rd_addr_base  (w_user_rd_addr_base),
        .o_user_rd_finish     (w_user_rd_finish),
        .o_user_rd_data_valid (w_user_rd_data_valid),
        .o_user_rd_data       (w_user_rd_data),

        .init_calib_complete  (w_init_calib_complete),
        .mmcm_locked          (w_mmcm_locked)
    );

    //==========================================================================
    // Sticky WR FIFO backpressure indicator
    //==========================================================================
    logic r_wr_fifo_stuck;

    always_ff @(posedge w_ui_clk) begin
        if (w_user_rst)
            r_wr_fifo_stuck <= 1'b0;
        else if (w_user_wr_data_valid && !w_user_wr_fifo_ready)
            r_wr_fifo_stuck <= 1'b1;
    end

    //==========================================================================
    // Elaboration-time sanity check on the register map
    //==========================================================================
    localparam int AD9833_END_ADDR   = AD9833_BASE_ADDR + 4 - 1;
    localparam int ADC_CORE_END_ADDR = ADC_CORE_BASE_ADDR + ADC_CORE_NUM_REGS - 1;

    // if (STREAM_REG_IDX >= NUM_REGS) begin : gen_bad_stream_idx
    //     $error("uart_top: STREAM_REG_IDX=%0d must be < NUM_REGS=%0d",
    //            STREAM_REG_IDX, NUM_REGS);
    // end
    // if (STATUS_REG_IDX >= NUM_REGS) begin : gen_bad_status_idx
    //     $error("uart_top: STATUS_REG_IDX=%0d must be < NUM_REGS=%0d",
    //            STATUS_REG_IDX, NUM_REGS);
    // end
    // if (ADC_CORE_END_ADDR >= STREAM_REG_IDX) begin : gen_adc_stream_overlap
    //     $error("uart_top: ADC window [%0d..%0d] overlaps STREAM reg at %0d",
    //            ADC_CORE_BASE_ADDR, ADC_CORE_END_ADDR, STREAM_REG_IDX);
    // end

    //==========================================================================
    // Stream handshake nets (uart_regs <-> ddr3_reader)
    //==========================================================================
    logic        w_stream_start;
    logic        w_stream_ready;
    logic [7:0]  w_stream_byte;
    logic        w_stream_valid;
    logic        w_stream_done;

    //==========================================================================
    // Register file (on ui_clk) with 1-entry status overlay for reg[STATUS_REG_IDX]
    //==========================================================================
    logic [0:NUM_REGS-1][31:0] w_regs;
    logic [0:0][31:0]          w_status_regs;     // STATUS_NUM = 1

    uart_regs #(
        .DATA_WIDTH       (DATA_WIDTH),
        .RX_FIFO_DEPTH    (RX_FIFO_DEPTH),
        .RX_FIFO_AF_DEPTH (RX_FIFO_AF_DEPTH),
        .RX_FIFO_AE_DEPTH (RX_FIFO_AE_DEPTH),
        .TX_FIFO_DEPTH    (TX_FIFO_DEPTH),
        .TX_FIFO_AF_DEPTH (TX_FIFO_AF_DEPTH),
        .TX_FIFO_AE_DEPTH (TX_FIFO_AE_DEPTH),
        .NUM_REGS         (NUM_REGS),
        .STATUS_BASE      (STATUS_REG_IDX),
        .STATUS_NUM       (1),
        .STREAM_ADDR      (STREAM_REG_IDX)
    ) u_uart_regs (
        .i_clk          (w_ui_clk),
        .i_rst          (w_user_rst),
        .i_rx           (i_rx),
        .o_tx           (o_tx),
        .i_status_regs  (w_status_regs),

        .o_stream_start (w_stream_start),
        .o_stream_ready (w_stream_ready),
        .i_stream_byte  (w_stream_byte),
        .i_stream_valid (w_stream_valid),
        .i_stream_done  (w_stream_done),

        .o_regs         (w_regs)
    );

    //==========================================================================
    // AD9833 sub-block (regs 0..3)
    //==========================================================================
    logic [31:0] w_reg_cmd;
    logic [31:0] w_reg_freq;
    logic [31:0] w_reg_phase_ctrl;
    logic [31:0] w_reg_control;

    assign w_reg_cmd        = w_regs[AD9833_BASE_ADDR + 0];
    assign w_reg_freq       = w_regs[AD9833_BASE_ADDR + 1];
    assign w_reg_phase_ctrl = w_regs[AD9833_BASE_ADDR + 2];
    assign w_reg_control    = w_regs[AD9833_BASE_ADDR + 3];

    ad9833_top #(
        .FRAME_W     (SPI_FRAME_W),
        .SPI_CLK_DIV (SPI_CLK_DIV)
    ) u_ad9833_top (
        .i_clk            (w_ui_clk),
        .i_rst_n          (w_user_rst_n),
        .i_reg_cmd        (w_reg_cmd),
        .i_reg_freq       (w_reg_freq),
        .i_reg_phase_ctrl (w_reg_phase_ctrl),
        .i_reg_control    (w_reg_control),
        .o_spi_sclk       (o_spi_sclk),
        .o_spi_fsync      (o_spi_fsync),
        .o_spi_mosi       (o_spi_mosi)
    );

    //==========================================================================
    // ADC subsystem
    //==========================================================================
    logic [DDR_ADDR_W-1:0]        w_bytes_written;

    logic                         w_adc_sampling;
    logic                         w_adc_active;
    logic signed [ADC_DATA_W-1:0] w_adc_data;
    logic                         w_adc_data_valid;
    logic                         w_adc_spi_finish;

    adc_top #(
        .NUM_REGS       (ADC_CORE_NUM_REGS),
        .CTRL_REG_IDX   (ADC_CTRL_REG_IDX),
        .ADC_DATA_W     (ADC_DATA_W),
        .ADC_LUT_DEPTH  (ADC_LUT_DEPTH),
        .ADC_SAMPLE_GAP (ADC_SAMPLE_GAP),
        .ADC_PHASE_STEP (ADC_PHASE_STEP),
        .DDR_DATA_W     (DDR_DATA_W),
        .DDR_ADDR_W     (DDR_ADDR_W),
        .P_WR_BURST_LEN (P_WR_BURST_LEN),
        .P_WR_BURST_NUM (P_WR_BURST_NUM),
        .DDR_BASE_ADDR  (DDR_BASE_ADDR)
    ) u_adc_top (
        .i_clk                (w_ui_clk),
        .i_rst                (w_user_rst),
        .i_all_regs           (w_regs[ADC_CORE_BASE_ADDR +: ADC_CORE_NUM_REGS]),

        .o_adc_sampling       (w_adc_sampling),
        .o_active             (w_adc_active),

        .o_adc_data           (w_adc_data),
        .o_adc_data_valid     (w_adc_data_valid),
        .o_adc_spi_finish     (w_adc_spi_finish),

        .o_user_wr_valid      (w_user_wr_valid),
        .o_user_wr_addr_base  (w_user_wr_addr_base),
        .o_user_wr_data       (w_user_wr_data),
        .o_user_wr_data_valid (w_user_wr_data_valid),

        .o_bytes_written      (w_bytes_written)
    );

    //==========================================================================
    // DDR3 stream reader
    //==========================================================================
    ddr3_reader #(
        .DDR_DATA_W     (DDR_DATA_W),
        .DDR_ADDR_W     (DDR_ADDR_W),
        .P_RD_BURST_LEN (P_RD_BURST_LEN),
        .P_RD_BURST_NUM (P_RD_BURST_NUM)
    ) u_ddr3_reader (
        .i_clk                (w_ui_clk),
        .i_rst                (w_user_rst),

        .i_stream_start       (w_stream_start),
        .i_start_addr         (DDR_BASE_ADDR),
        .i_total_bytes        (w_bytes_written),
        .o_stream_done        (w_stream_done),

        .o_stream_byte        (w_stream_byte),
        .o_stream_valid       (w_stream_valid),
        .i_stream_ready       (w_stream_ready),

        .o_user_rd_valid      (w_user_rd_valid),
        .o_user_rd_addr_base  (w_user_rd_addr_base),
        .i_user_rd_data_valid (w_user_rd_data_valid),
        .i_user_rd_data       (w_user_rd_data),
        .i_user_rd_finish     (w_user_rd_finish)
    );


    //==========================================================================
    assign w_status_regs[0] =
        {{(32-DDR_ADDR_W){1'b0}}, w_bytes_written};

endmodule