//`default_nettype none
`timescale 1ns / 1ps

//==============================================================================
// uart_regs  (with status-register overlay + stream-read path)
//------------------------------------------------------------------------------
//   reg[0..3]    AD9833
//   reg[4..57]   ADC core (reg[57]=ctrl)
//   reg[58]      STREAM trigger   (PC "read reg[58]" -> stream DDR3 out)
//   reg[59]      {3'b0, bytes_written[28:0]}   (STATUS overlay, 32-bit read)
//==============================================================================
module uart_regs
   #(parameter DATA_WIDTH=8,
     parameter RX_FIFO_DEPTH=8,
     parameter RX_FIFO_AF_DEPTH=6,
     parameter RX_FIFO_AE_DEPTH=2,
     parameter TX_FIFO_DEPTH=8,
     parameter TX_FIFO_AF_DEPTH=6,
     parameter TX_FIFO_AE_DEPTH=2,
     parameter NUM_REGS=54,
     parameter STATUS_BASE=0,
     parameter STATUS_NUM =0,
     
     parameter STREAM_ADDR=127)
    (input  logic i_clk,
     input  logic i_rst,

     input  logic i_rx,
     output logic o_tx,


     input  logic [0:(STATUS_NUM==0 ? 1 : STATUS_NUM)-1][31:0] i_status_regs,

     // Stream-read interface (to ddr3_reader)
     output logic       o_stream_start,   // 1-cycle pulse on entry to STREAM
     output logic       o_stream_ready,   // = !tx_fifo_full while in STREAM
     input  logic [7:0] i_stream_byte,
     input  logic       i_stream_valid,
     input  logic       i_stream_done,    // 1-cycle pulse: leave STREAM

     output logic [0:NUM_REGS-1][31:0] o_regs);

    logic [6:0] w_addr;
    logic [31:0] r_regs [NUM_REGS];

    for (genvar i = 0; i < NUM_REGS; i++) begin : O_REGS_GEN
        assign o_regs[i] = r_regs[i];
    end

    /******
    * uart
    *******/

    logic w_deq_rxq;
    logic [DATA_WIDTH-1:0] w_rxq_data;
    logic w_rxq_empty;

    logic w_enq_txq;
    logic [DATA_WIDTH-1:0] w_txq_data;
    logic w_txq_full;

    uart #(
        .DATA_WIDTH(DATA_WIDTH),
        .RX_FIFO_DEPTH (RX_FIFO_DEPTH),
        .RX_FIFO_AF_DEPTH(RX_FIFO_AF_DEPTH),
        .RX_FIFO_AE_DEPTH(RX_FIFO_AE_DEPTH),
        .TX_FIFO_DEPTH(TX_FIFO_DEPTH),
        .TX_FIFO_AF_DEPTH(TX_FIFO_AF_DEPTH),
        .TX_FIFO_AE_DEPTH(TX_FIFO_AE_DEPTH)
    ) UART (
        .i_clk(i_clk),
        .i_rst(i_rst),

        .i_rx(i_rx),
        .o_tx(o_tx),

        .i_dvsr(11'd6),

        .i_deq_rxq  (w_deq_rxq),
        .o_rxq_data (w_rxq_data),
        .o_rxq_empty(w_rxq_empty),
        .o_rxq_ae   (),
        .o_rxq_full (),
        .o_rxq_af   (),

        .i_enq_txq  (w_enq_txq),
        .i_txq_data (w_txq_data),
        .o_txq_empty(),
        .o_txq_ae   (),
        .o_txq_full (w_txq_full),
        .o_txq_af   ()
    );

    /*********
    * op byte
    *********/

    logic [7:0] r_op;
    logic w_latch_op;

    always_ff @(posedge i_clk) begin
        if (i_rst)
            r_op <= 'h0;
        else if (w_latch_op)
            r_op <= w_rxq_data;
    end

    /*******
    * write
    *******/

    logic w_wr;

    logic [31:0] r_wr_data;
    logic w_shift_in;

    always_ff @(posedge i_clk) begin
        if (i_rst)
            r_wr_data <= 'h0;
        else if (w_shift_in)
            r_wr_data <= {r_wr_data[23:0], w_rxq_data};
    end

    for (genvar i = 0; i < NUM_REGS; i++) begin : WR_GEN
        always_ff @(posedge i_clk) begin
            if (i_rst)
                r_regs[i] <= 32'hDEADC0DE;
            else if (w_wr && w_addr == i)
                r_regs[i] <= r_wr_data;
        end
    end

    /******
    * read  (with STATUS overlay)
    ******/

    logic w_rd;

    logic [31:0] r_rd_data;
    logic w_shift_out;

    
    logic [31:0] w_read_mux;
    always_comb begin
        w_read_mux = r_regs[w_addr];
        if (STATUS_NUM != 0) begin
            if ((w_addr >= STATUS_BASE) &&
                (w_addr <  STATUS_BASE + STATUS_NUM))
                w_read_mux = i_status_regs[w_addr - STATUS_BASE];
        end
    end

    always_ff @(posedge i_clk) begin
        if (i_rst)
            r_rd_data <= 32'hDEADC0DE;
        else if (w_rd)
            r_rd_data <= w_read_mux;
        else if (w_shift_out)
            r_rd_data <= {r_rd_data[23:0], r_rd_data[31:24]};
    end

    /**************
    * byte counter
    **************/

    logic [2:0] r_bcnt;
    logic w_bcnt_en, w_bcnt_clr;

    always_ff @(posedge i_clk) begin
        if (i_rst || w_bcnt_clr)
            r_bcnt <= 'd0;
        else if (w_bcnt_en)
            r_bcnt <= r_bcnt + 'd1;
    end

    /*************
    * tx arbitration
    *************/
 
    logic w_stream_active;

    assign o_stream_ready = w_stream_active && !w_txq_full;

    assign w_enq_txq  = w_stream_active ? (i_stream_valid && !w_txq_full)
                                        : w_shift_out;
    assign w_txq_data = w_stream_active ? i_stream_byte
                                        : r_rd_data[31:24];

    /*****
    * fsm
    *****/

    enum {IDLE, OP, WR, RD, STREAM} r_state, w_next_state;

    always_ff @(posedge i_clk) begin
        r_state <= i_rst ? IDLE : w_next_state;
    end

    assign w_stream_active = (r_state == STREAM);

    always_comb begin

        w_next_state = r_state;

        // addr ctrl
        w_addr = 'h0;

        // op ctrl
        w_latch_op = 1'b0;

        // wr ctrl
        w_shift_in = 1'b0;
        w_wr = 1'b0;

        // rd_ctrl
        w_rd = 1'b0;
        w_shift_out = 1'b0;

        // bcnt ctrl
        w_bcnt_en = 1'b0;
        w_bcnt_clr = 1'b0;

        // stream ctrl
        o_stream_start = 1'b0;

        case (r_state)

            IDLE: begin
                if (!w_rxq_empty) begin
                    w_next_state = OP;
                    w_latch_op = 1'b1;
                end
            end

            OP: begin
                // Dispatch based on op byte.  MSB=1 -> read, MSB=0 -> write.
                // A read targeting STREAM_ADDR enters STREAM instead of RD.
                w_addr     = r_op[6:0];
                w_bcnt_clr = 1'b1;

                if (r_op[7]) begin
                    if (r_op[6:0] == STREAM_ADDR[6:0]) begin
                        w_next_state   = STREAM;
                        o_stream_start = 1'b1;    // 1-cycle pulse on entry
                    end
                    else begin
                        w_next_state = RD;
                        w_rd         = 1'b1;      // latch read data
                    end
                end
                else begin
                    w_next_state = WR;
                end
            end

            RD: begin
                if (r_bcnt == 'd4) begin
                    w_next_state = IDLE;
                end
                else begin
                    w_shift_out = !w_txq_full;
                    w_bcnt_en   = !w_txq_full;
                end
            end

            WR: begin
                if (r_bcnt == 'd4) begin
                    w_next_state = IDLE;
                    w_addr       = r_op[6:0];
                    w_wr         = 1'b1;
                end
                else begin
                    w_shift_in = !w_rxq_empty;
                    w_bcnt_en  = !w_rxq_empty;
                end
            end

            STREAM: begin

                if (i_stream_done)
                    w_next_state = IDLE;
            end

        endcase

    end

    assign w_deq_rxq = w_latch_op || w_shift_in;

endmodule