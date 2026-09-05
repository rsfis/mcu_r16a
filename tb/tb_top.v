// tt_um_modo_macaco.v
//
// Wrapper no formato exigido pelo Tiny Tapeout. O nome do modulo TEM
// que comecar com "tt_um_" e ser unico (inclua seu usuario do GitHub
// antes de submeter -- ex: tt_um_seunome_modo_macaco).
//
// Mapa de pinos escolhido (18 sinais usados de 24 disponiveis):
//
//   ui_in[0]  = prog_rx   (entrada fixa -- UART, recebe do computador)
//   ui_in[1]  = spi_miso  (entrada fixa -- dado vindo da Flash SPI)
//   ui_in[7:2]            (nao usados)
//
//   uo_out[0] = prog_tx   (saida fixa -- UART, manda pro computador)
//   uo_out[1] = spi_cs    (saida fixa -- chip-select da Flash, ativo baixo)
//   uo_out[2] = spi_sck   (saida fixa -- clock do SPI)
//   uo_out[3] = spi_mosi  (saida fixa -- dado indo pra Flash)
//   uo_out[7:4]           (nao usados)
//
//   uio[7:0]  = GPIO0..GPIO7  (bidirecionais de verdade, com PWM e
//                              direcao configuravel em tempo real via
//                              GPIO_DIR -- ver top.v/gpio.v)
module tt_um_modo_macaco (
    input  wire [7:0] ui_in,    // entradas dedicadas
    output wire [7:0] uo_out,   // saidas dedicadas
    input  wire [7:0] uio_in,   // pino bidirecional: valor lido de fora
    output wire [7:0] uio_out,  // pino bidirecional: valor a dirigir
    output wire [7:0] uio_oe,   // pino bidirecional: 1 = chip dirige o pino
    input  wire       ena,      // 1 quando o projeto esta selecionado (mux do TT)
    input  wire       clk,
    input  wire       rst_n     // reset ativo em NIVEL BAIXO (padrao do TT)
);
    wire rst = ~rst_n;   // top.v usa reset ativo em alto

    wire prog_rx  = ui_in[0];
    wire spi_miso = ui_in[1];

    wire prog_tx, spi_cs, spi_sck, spi_mosi;
    wire [7:0] gpio_out, gpio_oe;

    top u_top (
        .clk(clk),
        .rst(rst),
        .gpio_pad_in(uio_in),
        .gpio_pad_out(gpio_out),
        .gpio_pad_oe(gpio_oe),
        .prog_rx(prog_rx),
        .prog_tx(prog_tx),
        .spi_cs(spi_cs),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso)
    );

    // Enquanto ena=0 (projeto nao selecionado no mux do TT), mantem as
    // saidas em zero -- pratica recomendada pelo TT pra nao "vazar"
    // atividade eletrica quando o projeto nao esta em uso.
    assign uo_out  = ena ? {4'b0000, spi_mosi, spi_sck, spi_cs, prog_tx} : 8'b0;
    assign uio_out = ena ? gpio_out : 8'b0;
    assign uio_oe  = ena ? gpio_oe  : 8'b0;

    // Evita warning de "sinal nao usado" no lint do TT (ui_in[7:2] sao
    // reservados de proposito pra expansao futura).
    wire _unused = &{ui_in[7:2], 1'b0};

endmodule
