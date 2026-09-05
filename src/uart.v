// uart.v — identico a Parte 3 do tutorial original.
// Sem mudancas: este modulo so ve prog_rx/prog_tx, que continuam sendo
// 2 sinais dedicados independente de quantos GPIOs de uso geral existem.
module uart #(
    parameter CLK_FREQ_HZ = 16_000_000,
    parameter BAUD        = 115200
)(
    input  wire clk,
    input  wire rst,
    input  wire rx_pin,
    output reg  tx_pin,
    output reg  [7:0] rx_data,
    output reg  rx_ready,
    input  wire rx_ack,
    input  wire tx_start,
    input  wire [7:0] tx_data,
    output reg  tx_busy
);
    localparam integer DIV = CLK_FREQ_HZ / BAUD;

    reg [3:0]  rx_state; reg [15:0] rx_cnt; reg [2:0] rx_bitidx; reg [7:0] rx_shift;
    localparam RX_IDLE=0, RX_START=1, RX_DATA=2, RX_STOP=3;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rx_state <= RX_IDLE; rx_ready <= 0; rx_cnt <= 0; rx_bitidx <= 0;
        end else begin
            if (rx_ack) rx_ready <= 0;
            case (rx_state)
                RX_IDLE:  if (!rx_pin) begin rx_state<=RX_START; rx_cnt<=DIV/2; end
                RX_START: if (rx_cnt==0) begin rx_state<=RX_DATA; rx_cnt<=DIV; rx_bitidx<=0; end
                          else rx_cnt <= rx_cnt - 1;
                RX_DATA:  if (rx_cnt==0) begin
                              rx_shift <= {rx_pin, rx_shift[7:1]};
                              rx_cnt <= DIV;
                              if (rx_bitidx==7) rx_state <= RX_STOP;
                              else rx_bitidx <= rx_bitidx + 1;
                          end else rx_cnt <= rx_cnt - 1;
                RX_STOP:  if (rx_cnt==0) begin
                              rx_data <= rx_shift; rx_ready <= 1; rx_state <= RX_IDLE;
                          end else rx_cnt <= rx_cnt - 1;
            endcase
        end
    end

    reg [3:0] tx_state; reg [15:0] tx_cnt; reg [2:0] tx_bitidx; reg [7:0] tx_shift;
    localparam TX_IDLE=0, TX_START=1, TX_DATA=2, TX_STOP=3;

    always @(posedge clk or posedge rst) begin
        if (rst) begin tx_state<=TX_IDLE; tx_pin<=1; tx_busy<=0; end
        else begin
            case (tx_state)
                TX_IDLE: if (tx_start) begin
                             tx_shift<=tx_data; tx_busy<=1; tx_pin<=0;
                             tx_cnt<=DIV; tx_state<=TX_DATA; tx_bitidx<=0;
                         end
                TX_DATA: if (tx_cnt==0) begin
                             tx_pin <= tx_shift[0]; tx_shift <= tx_shift >> 1;
                             tx_cnt <= DIV;
                             if (tx_bitidx==7) tx_state<=TX_STOP; else tx_bitidx<=tx_bitidx+1;
                         end else tx_cnt <= tx_cnt - 1;
                TX_STOP: if (tx_cnt==0) begin tx_pin<=1; tx_busy<=0; tx_state<=TX_IDLE; end
                         else tx_cnt <= tx_cnt - 1;
            endcase
        end
    end
endmodule