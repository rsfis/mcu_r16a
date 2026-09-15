// spi_byte_engine.v — NOVO
//
// Motor SPI generico de baixo nivel: transfere 1 byte por vez, modo 0
// (CPOL=0, CPHA=0), MSB primeiro. Identico, bit a bit, ao motor que
// antes vivia embutido dentro de boot_ctrl.v -- so foi extraido pra um
// modulo separado pra poder ser reaproveitado tanto pelo boot (leitura
// da flash) quanto pelo controlador da SRAM externa (ext_ram_ctrl.v),
// sem duplicar a logica do "bit-banging" em dois lugares.
//
// Protocolo de uso: com o motor ocioso (busy=0), coloque o byte a
// enviar em tx_byte e pulse "start" por 1 ciclo. O motor entao:
//   - dirige sck/mosi ele mesmo (o chamador so precisa ligar esses
//     sinais direto nos pinos fisicos, com o CS controlado
//     separadamente pelo sequenciador que instancia este motor);
//   - amostra miso a cada borda de subida de sck;
//   - ao terminar o 8o bit, pulsa "done" por 1 ciclo e disponibiliza
//     o byte recebido em rx_byte.
// Tempo por byte: 16 ciclos de clk (2 por bit: metade sck=0, metade
// sck=1), igual ao motor original.
module spi_byte_engine (
    input  wire clk,
    input  wire rst,
    input  wire        start,
    input  wire [7:0]  tx_byte,
    output reg  [7:0]  rx_byte,
    output reg         busy,
    output reg         done,
    output reg         sck,
    output reg         mosi,
    input  wire        miso
);
    reg [3:0] bitcnt;
    reg       half;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            busy <= 0; sck <= 0; bitcnt <= 0; half <= 0; done <= 0; mosi <= 0;
        end else begin
            done <= 0;
            if (start && !busy) begin
                busy <= 1; bitcnt <= 0; half <= 0;
                mosi <= tx_byte[7];
            end else if (busy) begin
                half <= ~half;
                if (!half) begin
                    sck <= 1;
                    rx_byte <= {rx_byte[6:0], miso};
                end else begin
                    sck <= 0;
                    if (bitcnt == 7) begin
                        busy <= 0; done <= 1;
                    end else begin
                        bitcnt   <= bitcnt + 1;
                        mosi     <= tx_byte[6 - bitcnt];
                    end
                end
            end
        end
    end
endmodule
