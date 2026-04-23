`timescale 1ps/100fs

//==============================================================================
// ddr_combine_tb
//------------------------------------------------------------------------------
// Test program is intentionally SMALL (no JMP loop, SAM count = 64) so the
// whole simulation finishes in seconds, not minutes.
//==============================================================================
module ddr_combine_tb;

    //============================================================
    // DDR3 model environment (matches ddr3_top_tb)
    //============================================================
    parameter int CS_WIDTH       = 1;
    parameter int DM_WIDTH       = 2;
    parameter int DQ_WIDTH       = 16;
    parameter int DQS_WIDTH      = 2;
    parameter int ROW_WIDTH      = 14;
    parameter int ODT_WIDTH      = 1;

    parameter int  CLKIN_PERIOD  = 10000;     // ps   (100 MHz board clock)
    parameter int  RESET_PERIOD  = 200000;    // ps   (200 ns reset hold)

    localparam int MEMORY_WIDTH  = 16;
    localparam int NUM_COMP      = DQ_WIDTH / MEMORY_WIDTH;

    //============================================================
    // uart_top register / interface params 
    //============================================================
    localparam int N_REGS             = 64;
    localparam int AD9833_BASE_ADDR   = 0;
    localparam int ADC_CORE_BASE_ADDR = 4;
    localparam int ADC_CORE_NUM_REGS  = 54;
    localparam int ADC_CTRL_REG_IDX   = 53;
    localparam int ADC_CTRL_REG_ADDR  = ADC_CORE_BASE_ADDR + ADC_CTRL_REG_IDX;

    localparam int ADC_DATA_W      = 16;
    localparam int ADC_SAMPLE_GAP  = 8;

    localparam int                     DDR_DATA_W      = 128;
    localparam int                     DDR_ADDR_W      = 28;
    localparam int                     P_WR_BURST_LEN  = 8;
    localparam int                     P_WR_BURST_NUM  = 1;
    localparam int                     BYTES_PER_WORD  = DDR_DATA_W / 8;
    localparam int                     WORDS_PER_TX    = P_WR_BURST_LEN * P_WR_BURST_NUM;
    localparam int                     BYTES_PER_TX    = WORDS_PER_TX * BYTES_PER_WORD;
    localparam logic [DDR_ADDR_W-1:0]  DDR_BASE_ADDR_TB = 28'h010_0000;

    //============================================================
    // UART timing  (timescale = 1 ps; bit_duration in ps)
    //============================================================
    localparam int      BAUDRATE     = 921600;
    localparam realtime bit_duration = (1.0e12 / BAUDRATE);   // ~1 085 069 ps

    //============================================================
    // Board clock / reset
    //============================================================
    logic i_clk;
    logic i_rst_n;

    initial i_clk = 1'b0;
    always #(CLKIN_PERIOD/2.0) i_clk = ~i_clk;

    initial begin
        i_rst_n = 1'b0;
        #RESET_PERIOD;
        i_rst_n = 1'b1;
    end

    //============================================================
    // UART line + AD9833 SPI
    //============================================================
    logic w_rx = 1'b1;
    logic w_tx;

    logic w_spi_sclk;
    logic w_spi_fsync;
    logic w_spi_mosi;

    //============================================================
    // ADC debug / status signals
    //============================================================
    logic                          w_init_calib_complete;

    wire                          w_pll_locked             = DUT.pll_locked;
    wire                          w_mmcm_locked            = DUT.w_mmcm_locked;
    wire                          w_user_wr_fifo_ready_top = DUT.w_user_wr_fifo_ready;
    wire                          w_wr_fifo_stuck          = DUT.r_wr_fifo_stuck;

    wire                          w_adc_sampling           = DUT.w_adc_sampling;
    wire                          w_adc_active             = DUT.w_adc_active;

    wire signed [ADC_DATA_W-1:0]  w_adc_data               = DUT.w_adc_data;
    wire                          w_adc_data_valid         = DUT.w_adc_data_valid;
    wire                          w_adc_spi_finish         = DUT.w_adc_spi_finish;

    //============================================================
    // DDR3 PHY wires - FPGA side
    //============================================================
    wire [DQ_WIDTH-1:0]       ddr3_dq_fpga;
    wire [DQS_WIDTH-1:0]      ddr3_dqs_p_fpga;
    wire [DQS_WIDTH-1:0]      ddr3_dqs_n_fpga;
    wire [ROW_WIDTH-1:0]      ddr3_addr_fpga;
    wire [2:0]                ddr3_ba_fpga;
    wire                      ddr3_ras_n_fpga;
    wire                      ddr3_cas_n_fpga;
    wire                      ddr3_we_n_fpga;
    wire                      ddr3_reset_n;
    wire [0:0]                ddr3_ck_p_fpga;
    wire [0:0]                ddr3_ck_n_fpga;
    wire [0:0]                ddr3_cke_fpga;
    wire [CS_WIDTH-1:0]       ddr3_cs_n_fpga;
    wire [DM_WIDTH-1:0]       ddr3_dm_fpga;
    wire [ODT_WIDTH-1:0]      ddr3_odt_fpga;

    //============================================================
    // DDR3 PHY wires - SDRAM side
    //============================================================
    wire [DQ_WIDTH-1:0]       ddr3_dq_sdram;
    wire [DQS_WIDTH-1:0]      ddr3_dqs_p_sdram;
    wire [DQS_WIDTH-1:0]      ddr3_dqs_n_sdram;
    logic [ROW_WIDTH-1:0]     ddr3_addr_sdram [0:1];
    logic [2:0]               ddr3_ba_sdram   [0:1];
    logic                     ddr3_ras_n_sdram;
    logic                     ddr3_cas_n_sdram;
    logic                     ddr3_we_n_sdram;
    logic [0:0]               ddr3_cke_sdram;
    logic [0:0]               ddr3_ck_p_sdram;
    logic [0:0]               ddr3_ck_n_sdram;
    logic [CS_WIDTH-1:0]      ddr3_cs_n_sdram_tmp;
    logic [DM_WIDTH-1:0]      ddr3_dm_sdram_tmp;
    logic [ODT_WIDTH-1:0]     ddr3_odt_sdram_tmp;
    wire  [CS_WIDTH-1:0]      ddr3_cs_n_sdram;
    wire  [DM_WIDTH-1:0]      ddr3_dm_sdram;
    wire  [ODT_WIDTH-1:0]     ddr3_odt_sdram;

    //============================================================
    // PCB delay modeling (verbatim from ddr3_top_tb)
    //============================================================
    always @(*) begin
        ddr3_ck_p_sdram    <= ddr3_ck_p_fpga;
        ddr3_ck_n_sdram    <= ddr3_ck_n_fpga;
        ddr3_addr_sdram[0] <= ddr3_addr_fpga;
        ddr3_addr_sdram[1] <= ddr3_addr_fpga;
        ddr3_ba_sdram[0]   <= ddr3_ba_fpga;
        ddr3_ba_sdram[1]   <= ddr3_ba_fpga;
        ddr3_ras_n_sdram   <= ddr3_ras_n_fpga;
        ddr3_cas_n_sdram   <= ddr3_cas_n_fpga;
        ddr3_we_n_sdram    <= ddr3_we_n_fpga;
        ddr3_cke_sdram     <= ddr3_cke_fpga;
    end

    always @(*) ddr3_cs_n_sdram_tmp <= ddr3_cs_n_fpga;
    assign ddr3_cs_n_sdram = ddr3_cs_n_sdram_tmp;

    always @(*) ddr3_dm_sdram_tmp <= ddr3_dm_fpga;
    assign ddr3_dm_sdram = ddr3_dm_sdram_tmp;

    always @(*) ddr3_odt_sdram_tmp <= ddr3_odt_fpga;
    assign ddr3_odt_sdram = ddr3_odt_sdram_tmp;

    genvar dqwd;
    generate
        for (dqwd = 0; dqwd < DQ_WIDTH; dqwd++) begin : dq_delay
            WireDelay #(
                .Delay_g   (0.0),
                .Delay_rd  (0.0),
                .ERR_INSERT("OFF")
            ) u_delay_dq (
                .A             (ddr3_dq_fpga[dqwd]),
                .B             (ddr3_dq_sdram[dqwd]),
                .reset         (i_rst_n),
                .phy_init_done (w_init_calib_complete)
            );
        end
    endgenerate

    genvar dqswd;
    generate
        for (dqswd = 0; dqswd < DQS_WIDTH; dqswd++) begin : dqs_delay
            WireDelay #(
                .Delay_g   (0.0),
                .Delay_rd  (0.0),
                .ERR_INSERT("OFF")
            ) u_delay_dqs_p (
                .A             (ddr3_dqs_p_fpga[dqswd]),
                .B             (ddr3_dqs_p_sdram[dqswd]),
                .reset         (i_rst_n),
                .phy_init_done (w_init_calib_complete)
            );

            WireDelay #(
                .Delay_g   (0.0),
                .Delay_rd  (0.0),
                .ERR_INSERT("OFF")
            ) u_delay_dqs_n (
                .A             (ddr3_dqs_n_fpga[dqswd]),
                .B             (ddr3_dqs_n_sdram[dqswd]),
                .reset         (i_rst_n),
                .phy_init_done (w_init_calib_complete)
            );
        end
    endgenerate

    //============================================================
    // DUT = uart_top  
    //============================================================
    uart_top #(
        .DATA_WIDTH         (8),
        .RX_FIFO_DEPTH      (8),
        .RX_FIFO_AF_DEPTH   (6),
        .RX_FIFO_AE_DEPTH   (2),
        .TX_FIFO_DEPTH      (8),
        .TX_FIFO_AF_DEPTH   (6),
        .TX_FIFO_AE_DEPTH   (2),
        .NUM_REGS           (N_REGS),
        .AD9833_BASE_ADDR   (AD9833_BASE_ADDR),
        .SPI_FRAME_W        (16),
        .SPI_CLK_DIV        (10),
        .ADC_CORE_BASE_ADDR (ADC_CORE_BASE_ADDR),
        .ADC_CORE_NUM_REGS  (ADC_CORE_NUM_REGS),
        .ADC_CTRL_REG_IDX   (ADC_CTRL_REG_IDX),
        .ADC_DATA_W         (ADC_DATA_W),
        .ADC_SAMPLE_GAP     (ADC_SAMPLE_GAP),
        .DDR_DATA_W         (DDR_DATA_W),
        .DDR_ADDR_W         (DDR_ADDR_W),
        .P_WR_BURST_LEN     (P_WR_BURST_LEN),
        .P_WR_BURST_NUM     (P_WR_BURST_NUM),
        .DDR_BASE_ADDR      (DDR_BASE_ADDR_TB)
    ) DUT (
        .i_clk                  (i_clk),
        .i_rst_n                (i_rst_n),
        .i_rx                   (w_rx),
        .o_tx                   (w_tx),

        .o_spi_sclk             (w_spi_sclk),
        .o_spi_fsync            (w_spi_fsync),
        .o_spi_mosi             (w_spi_mosi),

        // DDR3 PHY straight to the WireDelay net set
        .ddr3_addr              (ddr3_addr_fpga),
        .ddr3_ba                (ddr3_ba_fpga),
        .ddr3_cas_n             (ddr3_cas_n_fpga),
        .ddr3_ck_n              (ddr3_ck_n_fpga),
        .ddr3_ck_p              (ddr3_ck_p_fpga),
        .ddr3_cke               (ddr3_cke_fpga),
        .ddr3_ras_n             (ddr3_ras_n_fpga),
        .ddr3_reset_n           (ddr3_reset_n),
        .ddr3_we_n              (ddr3_we_n_fpga),
        .ddr3_dq                (ddr3_dq_fpga),
        .ddr3_dqs_n             (ddr3_dqs_n_fpga),
        .ddr3_dqs_p             (ddr3_dqs_p_fpga),
        .ddr3_cs_n              (ddr3_cs_n_fpga),
        .ddr3_dm                (ddr3_dm_fpga),
        .ddr3_odt               (ddr3_odt_fpga),

        .o_init_calib_complete  (w_init_calib_complete)
    );

    //============================================================
    // DDR3 model array  1 component 
    //============================================================
    genvar r, i;
    generate
        for (r = 0; r < CS_WIDTH; r = r + 1) begin : mem_rnk
            for (i = 0; i < NUM_COMP; i = i + 1) begin : gen_mem
                ddr3_model u_comp_ddr3 (
                    .rst_n   (ddr3_reset_n),
                    .ck      (ddr3_ck_p_sdram),
                    .ck_n    (ddr3_ck_n_sdram),
                    .cke     (ddr3_cke_sdram[r]),
                    .cs_n    (ddr3_cs_n_sdram[r]),
                    .ras_n   (ddr3_ras_n_sdram),
                    .cas_n   (ddr3_cas_n_sdram),
                    .we_n    (ddr3_we_n_sdram),
                    .dm_tdqs (ddr3_dm_sdram[(2*(i+1)-1):(2*i)]),
                    .ba      (ddr3_ba_sdram[r]),
                    .addr    (ddr3_addr_sdram[r]),
                    .dq      (ddr3_dq_sdram[16*(i+1)-1:16*i]),
                    .dqs     (ddr3_dqs_p_sdram[(2*(i+1)-1):(2*i)]),
                    .dqs_n   (ddr3_dqs_n_sdram[(2*(i+1)-1):(2*i)]),
                    .tdqs_n  (),
                    .odt     (ddr3_odt_sdram[r])
                );
            end
        end
    endgenerate

    //============================================================
    // Scoreboard 
    //============================================================
    int                    n_samples       = 0;
    int                    n_words         = 0;
    int                    n_txs           = 0;
    logic [DDR_ADDR_W-1:0] running_addr    = '0;
    int                    words_in_cur_tx = 0;
    logic [DDR_DATA_W-1:0] ddr_mem [logic [DDR_ADDR_W-1:0]];

    wire        sb_clk           = DUT.w_ui_clk;
    wire        sb_rst           = DUT.w_user_rst;
    wire        sb_wr_valid      = DUT.w_user_wr_valid;
    wire [27:0] sb_wr_addr_base  = DUT.w_user_wr_addr_base;
    wire        sb_wr_data_valid = DUT.w_user_wr_data_valid;
    wire [127:0]sb_wr_data       = DUT.w_user_wr_data;

    wire [DDR_ADDR_W-1:0] effective_wr_addr =
        sb_wr_valid ? sb_wr_addr_base : running_addr;

    always_ff @(posedge sb_clk) begin
        if (!sb_rst) begin
            if (w_adc_data_valid) begin
                $display("[%7t ps] ADC sample[%0d] = %0d (0x%0h)",
                         $time, n_samples, $signed(w_adc_data), w_adc_data);
                n_samples = n_samples + 1;
            end

            if (sb_wr_valid) begin
                $display("[%7t ps] >>> TX #%0d launch : addr_base = 0x%07h",
                         $time, n_txs + 1, sb_wr_addr_base);
                n_txs           = n_txs + 1;
                words_in_cur_tx = 0;
            end

            if (sb_wr_data_valid) begin
                ddr_mem[effective_wr_addr] = sb_wr_data;
                $display("[%7t ps]     word %0d @ 0x%07h = 0x%032h",
                         $time, words_in_cur_tx,
                         effective_wr_addr, sb_wr_data);
                n_words         = n_words + 1;
                running_addr    = effective_wr_addr + DDR_ADDR_W'(BYTES_PER_WORD);
                words_in_cur_tx = words_in_cur_tx + 1;
            end
        end
    end

    //============================================================
    // PC -> FPGA UART byte transmit  
    //============================================================
    task automatic pc_tsmt(input logic [7:0] data);
    begin
        $display("[%7t ps] pc sends 0x%02h", $time, data);
        @(negedge i_clk);
        w_rx = 1'b0;
        #bit_duration;
        for (int b = 0; b < 8; b++) begin
            @(negedge i_clk);
            w_rx = data[b];
            #bit_duration;
        end
        @(negedge i_clk);
        w_rx = 1'b1;
        #bit_duration;
    end
    endtask

    task automatic pc_tsmt_gap(input int max_bits_idle = 2);
        int idle_bits;
        begin
            idle_bits = $urandom_range(0, max_bits_idle);
            if (idle_bits > 0)
                #(bit_duration * idle_bits);
        end
    endtask

    task automatic send_write(input logic [6:0] idx, input logic [31:0] data);
    begin
        $display("[%7t ps] WRITE regs[%0d] = 0x%08x", $time, idx, data);
        pc_tsmt({1'b0, idx});
        pc_tsmt_gap();
        pc_tsmt(data[31:24]);
        pc_tsmt_gap();
        pc_tsmt(data[23:16]);
        pc_tsmt_gap();
        pc_tsmt(data[15:8]);
        pc_tsmt_gap();
        pc_tsmt(data[7:0]);
        pc_tsmt_gap();
    end
    endtask

    //============================================================
    // ADC instruction encoder 
    //============================================================
    function automatic [31:0] make_adc_insn(
        input logic [3:0]  opcode,
        input logic [11:0] count_field,
        input logic [15:0] delay_or_target
    );
        make_adc_insn = {opcode, count_field, delay_or_target};
    endfunction

    //============================================================
    // PC <- FPGA UART byte receive  
    //============================================================
    task automatic pc_rcv(output logic [7:0] data);
    begin
        // wait for start bit (w_tx falls to 0)
        wait (w_tx === 1'b0);
        // align to middle of the start bit
        #(bit_duration/2.0);
        // sample 8 data bits at each bit center
        for (int b = 0; b < 8; b++) begin
            #(bit_duration);
            data[b] = w_tx;
        end
        // skip the stop bit
        #(bit_duration);
        $display("[%7t ps] pc received 0x%02h", $time, data);
    end
    endtask

    //============================================================
    // PC-style register read via UART
    //============================================================
    task automatic send_read(input  logic [6:0]  idx,
                             output logic [31:0] data);
        logic [7:0] b0, b1, b2, b3;
    begin
        $display("[%7t ps] READ regs[%0d]", $time, idx);
        pc_tsmt({1'b1, idx});

        fork
            begin : recv_bytes
                pc_rcv(b0);
                pc_rcv(b1);
                pc_rcv(b2);
                pc_rcv(b3);
                data = {b0, b1, b2, b3};
            end
            begin : recv_timeout
                #(bit_duration * 200.0);
                $fatal(1, "UART read of reg[%0d] timed out", idx);
            end
        join_any
        disable fork;
        $display("[%7t ps]   reg[%0d] = 0x%08x", $time, idx, data);
    end
    endtask

    //============================================================
    // DDR3 readback through the STREAM path.
    //
    // New register layout (STREAM):
    //   reg[58] = STREAM_REG  : READ op on this reg kicks a byte-stream of
    //                           the full written DDR3 region (bytes_written
    //                           bytes, MSB-first, 16 bytes per 128b word).
    //   reg[59] = STATUS_REG  : READ op returns {3'b0, bytes_written[28:0]}
    //                           (MSB-first 4 bytes, same as any normal reg).
    //
    // Flow:
    //   1) READ reg[59]                -> pick up bytes_written
    //   2) READ reg[58] (op byte only) -> FPGA streams bytes_written bytes
    //   3) PC calls pc_rcv() bytes_written times to collect the stream
    //============================================================
    localparam int STREAM_REG = 58;
    localparam int STATUS_REG = 59;

    task automatic ddr_stream_read_all(
        input  int          expected_bytes,
        output logic [7:0]  byte_q [$]
    );
        logic [7:0] b;
    begin
        byte_q.delete();

        $display("[%7t ps] STREAM READ reg[%0d], expecting %0d bytes",
                 $time, STREAM_REG, expected_bytes);

        // One op byte "read STREAM_REG" kicks the stream FSM.
        pc_tsmt({1'b1, 7'(STREAM_REG)});

        // Then PC just receives `expected_bytes` bytes back-to-back.
        // pc_rcv waits for each start bit, so it will naturally pace
        // itself against the FPGA's TX FIFO / DDR3 latency.
        for (int i = 0; i < expected_bytes; i++) begin
            pc_rcv(b);
            byte_q.push_back(b);
        end

        $display("[%7t ps] STREAM READ complete, %0d bytes received",
                 $time, byte_q.size());
    end
    endtask

    //============================================================
    // ADC TEST
    //============================================================
    task automatic run_adc_test(input int sam_count);
        logic [31:0] adc_insn0, adc_insn1, adc_insn2;
        int          exp_samples, exp_words, exp_txs;
        bit          pass;
        int          samples_at_start, words_at_start, txs_at_start;
        logic [31:0]           status_reg;
        logic [DDR_ADDR_W-1:0] hw_bytes_written;
        int                    sb_bytes_written;
        int                    SAMPLES_PER_TX;
    begin
        SAMPLES_PER_TX = (DDR_DATA_W / ADC_DATA_W) * WORDS_PER_TX;   // 64

        $display("\n==================================================");
        $display("START COMBINED ADC + DDR3 MODEL TEST  (SAM count=%0d)", sam_count);
        $display("==================================================");

        samples_at_start = n_samples;
        words_at_start   = n_words;
        txs_at_start     = n_txs;

        adc_insn0 = make_adc_insn(4'b0000, 12'd0,              16'd3);          // NOP delay = 3
        adc_insn1 = make_adc_insn(4'b0001, 12'(sam_count),     16'd0);          // SAM
        adc_insn2 = make_adc_insn(4'b1111, 12'd0,              16'd0);          // END

        exp_samples = sam_count;
        exp_txs     = (exp_samples + SAMPLES_PER_TX - 1) / SAMPLES_PER_TX;      // ceil
        exp_words   = exp_txs * WORDS_PER_TX;

        send_write(ADC_CTRL_REG_ADDR, 32'h0000_0000);

        send_write(ADC_CORE_BASE_ADDR + 0, adc_insn0);
        send_write(ADC_CORE_BASE_ADDR + 1, adc_insn1);
        send_write(ADC_CORE_BASE_ADDR + 2, adc_insn2);

        $display("ADC program loaded (combined TB):");
        $display("  reg[%0d] = 0x%08x  (NOP delay=3)",       ADC_CORE_BASE_ADDR + 0, adc_insn0);
        $display("  reg[%0d] = 0x%08x  (SAM count=%0d)",     ADC_CORE_BASE_ADDR + 1, adc_insn1, sam_count);
        $display("  reg[%0d] = 0x%08x  (END)",               ADC_CORE_BASE_ADDR + 2, adc_insn2);
        $display("  expected: txs=%0d, words=%0d, samples=%0d",
                 exp_txs, exp_words, exp_samples);

        repeat (50) @(posedge sb_clk);

        // start the ADC
        send_write(ADC_CTRL_REG_ADDR, 32'h8000_0000);

        // wait for ADC core to finish its program
        fork
            begin : wait_active_low
                wait (w_adc_active === 1'b1);
                $display("[%7t ps] ADC active asserted", $time);
                wait (w_adc_active === 1'b0);
                $display("[%7t ps] ADC active deasserted", $time);
            end
            begin : timeout_active_low
                #(bit_duration * 50000.0);
                $fatal(1, "Timeout waiting for ADC active to drop");
            end
        join_any
        disable fork;

        // let the WR FIFO drain and the DDR3 actually accept the burst
        repeat (4000) @(posedge sb_clk);

        send_write(ADC_CTRL_REG_ADDR, 32'h0000_0000);
        repeat (200) @(posedge sb_clk);

        //------------------------------------------------------------
        // Scoreboard summary
        //------------------------------------------------------------
        sb_bytes_written = (n_words - words_at_start) * BYTES_PER_WORD;

        $display("\n============== ADC SUMMARY ==============");
        $display("ADC samples captured       : %0d  (expected %0d)",
                 n_samples - samples_at_start, exp_samples);
        $display("128-bit DDR words written  : %0d  (expected %0d)",
                 n_words - words_at_start,     exp_words);
        $display("DDR3 transactions launched : %0d  (expected %0d)",
                 n_txs - txs_at_start,         exp_txs);
        $display("SB bytes_written           : %0d",  sb_bytes_written);
        $display("o_wr_fifo_stuck            : %b  (expected 0)", w_wr_fifo_stuck);
        $display("=========================================");

        pass = 1'b1;
        if ((n_samples - samples_at_start) != exp_samples) begin
            $display("FAIL: sample count");  pass = 0;
        end
        if ((n_words - words_at_start) != exp_words) begin
            $display("FAIL: word count");    pass = 0;
        end
        if ((n_txs - txs_at_start) != exp_txs) begin
            $display("FAIL: tx count");      pass = 0;
        end
        if (w_wr_fifo_stuck) begin
            $display("FAIL: WR FIFO went not-ready during writes (back-pressure)");
            pass = 0;
        end

        //------------------------------------------------------------
        // HW-side bytes_written via the status register overlay
        //------------------------------------------------------------
        $display("\n=== BYTES_WRITTEN STATUS CHECK (reg[%0d]) ===", STATUS_REG);
        send_read(STATUS_REG, status_reg);
        hw_bytes_written = status_reg[DDR_ADDR_W-1:0];
        $display("  HW reg[%0d] = 0x%08x  ->  bytes_written=%0d",
                 STATUS_REG, status_reg, hw_bytes_written);
        if (hw_bytes_written != DDR_ADDR_W'(sb_bytes_written)) begin
            $display("  FAIL: HW bytes_written (%0d) != SB (%0d)",
                     hw_bytes_written, sb_bytes_written);
            pass = 0;
        end
        else begin
            $display("  [OK] HW bytes_written matches scoreboard");
        end

        //------------------------------------------------------------

        $display("\n=== STREAM READBACK VERIFICATION ===");
        begin
            logic [7:0]             rx_bytes [$];
            logic [DDR_DATA_W-1:0]  got_word;
            logic [DDR_DATA_W-1:0]  expected_word;
            logic [DDR_ADDR_W-1:0]  a;
            int                     n_stream_words;

            ddr_stream_read_all(int'(hw_bytes_written), rx_bytes);

            n_stream_words = rx_bytes.size() / BYTES_PER_WORD;
            if ((rx_bytes.size() % BYTES_PER_WORD) != 0) begin
                $display("  FAIL: stream byte count %0d not a multiple of %0d",
                         rx_bytes.size(), BYTES_PER_WORD);
                pass = 0;
            end

            for (int k = 0; k < n_stream_words; k++) begin
                got_word = '0;
                // MSB-first: byte[0] -> bits[127:120], byte[15] -> bits[7:0]
                for (int bi = 0; bi < BYTES_PER_WORD; bi++) begin
                    got_word[((BYTES_PER_WORD-1-bi)*8) +: 8] =
                        rx_bytes[k*BYTES_PER_WORD + bi];
                end

                a = DDR_BASE_ADDR_TB + DDR_ADDR_W'(k * BYTES_PER_WORD);
                if (ddr_mem.exists(a)) begin
                    expected_word = ddr_mem[a];
                end
                else begin
                    // Never written by the scoreboard -> zero-pad region
                    expected_word = '0;
                end

                if (got_word === expected_word) begin
                    $display("  [OK]   word[%0d] @0x%07h  exp=0x%032h  got=0x%032h",
                             k, a, expected_word, got_word);
                end
                else begin
                    $display("  [FAIL] word[%0d] @0x%07h  exp=0x%032h  got=0x%032h",
                             k, a, expected_word, got_word);
                    pass = 0;
                end
            end
        end

        if (!pass) $fatal(1, "COMBINED TEST FAILED (SAM count=%0d)", sam_count);
        else       $display("\nCOMBINED TEST PASS (SAM count=%0d)", sam_count);
    end
    endtask

    //============================================================
    // Main
    //============================================================
    initial begin
        $display("[%7t ps] Waiting for board reset release ...", $time);
        @(posedge i_rst_n);

        $display("[%7t ps] Waiting for clk_wiz lock ...", $time);
        wait (w_pll_locked === 1'b1);
        $display("[%7t ps] PLL locked.", $time);

        $display("[%7t ps] Waiting for MIG calibration ...", $time);
        wait (w_init_calib_complete === 1'b1);
        $display("[%7t ps] Calibration done.", $time);

        // give the user-domain reset synchronizer a few sys cycles to drop
        repeat (50) @(posedge sb_clk);

        
        run_adc_test(64);    // clean case, multiple of 64
        run_adc_test(50);    // <64 : exercises zero-pad flush
        run_adc_test(65);    // >64 : 2 transactions with trailing zero pad

        $display("\n==================================================");
        $display("ddr_combine_tb: ALL TESTS PASS");
        $display("==================================================\n");
        #(100_000);  // 100 ns
        $finish;
    end

    //============================================================
    // Safety net (sim of MIG + ddr3_model can be slow; pick a high cap)
    //============================================================
    initial begin
        #(1_000_000_000);   // 1 ms in ps  -> bump up if needed
        $display("[%7t ps] WALL TIMEOUT - $finish", $time);
        $finish;
    end

endmodule