import common::*;

module uart(
    input rst_n,
    input clk28,
    input en,

    cpu_bus bus,
    output [7:0] d_out,
    output d_out_active,

    input uart_rx,
    output reg uart_tx,
    output reg uart_rts
);

localparam CLK_HZ = 28000000;
localparam BAUD_RATE = 115200;
localparam PERIOD_1_4 = (CLK_HZ / (BAUD_RATE * 8));

localparam
    IDLE  = 3'd0,
    START = 3'd1,
    DATA  = 3'd2,
    STOP  = 3'd3,
    WAIT  = 3'd4;

reg [5:0] clk_cnt = 6'b0;
/* bit_clk[0] is 115200 * 4, bit_clk[2] is 115200 */
reg [2:0] bit_clk = 3'b0;

reg tx_start = 1'b0;
reg tx_busy = 1'b0;
reg rx_ready = 1'b0;
reg rx_data_read = 1'b0;
reg [1:0] tx_state = IDLE;
reg [2:0] rx_state = IDLE;

reg [7:0] tx_data = 8'hFF;
reg [7:0] rx_data = 8'h00;

wire uart_tx_rd = en && bus.iorq && bus.rd && bus.a == 16'h133B;
wire uart_tx_wr = en && bus.iorq && bus.wr && bus.a == 16'h133B;
wire uart_rx_rd = en && bus.iorq && bus.rd && bus.a == 16'h143B;

always @(posedge clk28 or negedge rst_n) begin
    if (!rst_n) begin
        rx_data_read <= 0;
    end
    else begin
        if (rx_data_read && !rx_ready) begin
            rx_data_read <= 0;
        end
        if (uart_tx_rd) begin
            d_out <= {6'h00, tx_busy | tx_start, rx_ready & !rx_data_read};
        end
        else if (uart_rx_rd) begin
            d_out <= rx_data;
            if (rx_ready) begin
                rx_data_read <= 1'b1;
            end;
        end
        d_out_active <= uart_tx_rd | uart_rx_rd;
    end
end

always @(posedge clk28 or negedge rst_n) begin
    if (!rst_n) begin
        clk_cnt <= 6'h0;
        bit_clk <= 3'h0;
    end
    else begin
        clk_cnt <= clk_cnt + 1'b1;
        if (clk_cnt == PERIOD_1_4) begin
            clk_cnt <= 6'b0;
            bit_clk <= bit_clk + 1'b1;
        end
    end
end

always @(posedge clk28 or negedge rst_n) begin
    if (!rst_n) begin
        tx_start <= 0;
    end
    else begin
        if (uart_tx_wr && !tx_start && !tx_busy) begin
            tx_data <= bus.d;
            tx_start <= 1'b1;
        end
        else if (tx_start && tx_busy) begin
            tx_start <= 0;
        end
    end
end

reg [2:0] tx_bit_cnt = 3'b0;
reg [7:0] tx_work_data = 8'hFF;


/* TX state machine. Works on bit_clk (baud rate) */
always @(negedge bit_clk[2] or negedge rst_n) begin
    if (!rst_n) begin
        tx_busy <= 0;
        tx_state <= IDLE;
        tx_bit_cnt <= 0;
        uart_tx <= 1'b1;
    end
    else begin
        case (tx_state)
            IDLE:
                begin
                    if (tx_start) begin
                        tx_busy <= 1'b1;
                        tx_state <= START;
                    end
                end
            START:
                begin
                    tx_work_data <= tx_data;
                    uart_tx <= 1'b0;
                    tx_bit_cnt <= 3'd7;
                    tx_state <= DATA;
                end
            DATA:
                begin
                    uart_tx <= tx_work_data[0];
                    tx_work_data <= {1'b0, tx_work_data[7:1]};
                    tx_bit_cnt <= tx_bit_cnt - 3'd1;
                    if (tx_bit_cnt == 3'd0) begin
                        tx_state <= STOP;
                    end
                end
            STOP:
                begin
                    uart_tx <= 1'b1;
                    tx_busy <= 1'b0;
                    tx_state <= IDLE;
                end
            default:
                begin
                    tx_state <= IDLE;
                end
        endcase
    end
end

reg [1:0] bit_clk4x_samples = 2'b00;
wire sample_tick = { bit_clk4x_samples == 2'b10 };

reg [1:0] rx_clk_cnt;
wire rx_bit_middle = { rx_clk_cnt == 2'd2 };

reg [2:0] rx_bit_cnt;

/* RX state machine. Works on bit_clk * 4 (baud_rate * 4) */
always @(posedge clk28 or negedge rst_n) begin
    if (!rst_n) begin
        // Raise RTS to indicate that we are not ready to receive.
        uart_rts <= 1'b1;
        rx_state <= IDLE;
        rx_ready <= 0;
        rx_data <= 8'hff;
    end
    else begin
        bit_clk4x_samples <= { bit_clk4x_samples[0], bit_clk[0] };
        case (rx_state)
            IDLE:
                begin
                    rx_ready <= 0;
                    if (sample_tick) begin
                        uart_rts <= 0; // Lower RTS to signal that we are ready to receive data
                        if (!uart_rx) begin
                            rx_state <= START;
                            rx_clk_cnt <= 2'd1; // We may have lost up to 1 cycle waiting for negedge
                        end
                    end
                end
            START:
                begin
                    if (sample_tick) begin
                        rx_clk_cnt <= rx_clk_cnt + 2'b1;
                        if (rx_bit_middle) begin // Middle of the bit
                            if (uart_rx) begin
                                // Start must be 0, go back to IDLE
                                rx_state <= IDLE;
                            end
                        end
                        // Start bit confirmed, wait for 2 more cycles and
                        // switch to DATA state
                        else if (rx_clk_cnt == 2'd0) begin
                            rx_data <= 8'h00;
                            rx_state <= DATA;
                            rx_bit_cnt <= 3'd7;
                        end
                    end
                end
            DATA:
                begin
                    if (sample_tick) begin
                        rx_clk_cnt <= rx_clk_cnt + 2'b1;
                        if (rx_bit_middle) begin // Middle of the bit
                            // Sample data bit
                            rx_data <= { uart_rx, rx_data[7:1] };
                        end
                        else if (rx_clk_cnt == 2'd0) begin
                            rx_bit_cnt <= rx_bit_cnt - 3'd1;
                            if (rx_bit_cnt == 3'd0) begin
                                rx_state <= STOP;
                            end
                        end
                    end
                end
            STOP:
                begin
                    if (sample_tick) begin
                        rx_clk_cnt <= rx_clk_cnt + 2'b1;
                        if (rx_bit_middle) begin // Middle of the bit
                            if (!uart_rx) begin
                                // Stop bit must be 1. Go back to IDLE
                                rx_state <= IDLE;
                            end
                            else begin
                                // Data received successfully
                                rx_ready <= 1'b1;
                                uart_rts <= 1'b1; // Raise RTS to signal that we are not ready to receive more data
                                rx_state <= WAIT;
                            end
                        end
                    end
                end
            WAIT:
                begin
                    if (rx_data_read) begin
                        rx_state <= IDLE;
                    end
                end
            default:
                begin
                    rx_state <= IDLE;
                end
        endcase
    end
end

endmodule
