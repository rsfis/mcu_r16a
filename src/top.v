// top.v — v6 (XIP: programa direto da flash, SRAM 100% livre p/ dados)
//
// MUDANCA PRINCIPAL em relacao a v5: nao existe mais boot_ctrl.v nem
// copia do programa (flash -> SRAM) no boot. A CPU agora busca cada
// instrucao (opcode+operando) DIRETO da flash externa, via o novo
// modulo flash_fetch (barramento fetch_addr/fetch_req/fetch_rdata/
// fetch_ready, fisicamente ligado aos mesmos pinos spi_cs/sck/mosi/
// miso que antes eram do boot_ctrl). A CPU sai do reset e ja comeca
// buscando pc=0 na flash -- nao ha' mais fase de boot nem cpu_rst
// extra. A SRAM externa (ext_ram_ctrl, mem_addr/mem_req/...) agora e'
// usada SOMENTE para dados do usuario (variaveis, pilha de CALL/RET/
// PUSH/POP) -- antes ela guardava o programa inteiro, agora fica
// inteira livre.
//
// Trade-off aceito: acesso RANDOMICO na flash (desvio/JMP/CALL/RET)
// custa uma transacao SPI completa (~96 ciclos, igual a SRAM), mas
// fetch SEQUENCIAL (o caso comum, pc incrementando) e' otimizado por
// streaming dentro de flash_fetch (CS fica baixo entre fetches
// consecutivos) -- ver comentario em flash_fetch.v.
//
// Pinos NOVOS (dedicados a SRAM externa, barramento SPI proprio,
// SEM compartilhamento com o barramento da flash):
//   sram_cs   (saida, ativo em 0)
//   sram_sck  (saida)
//   sram_mosi (saida)
//   sram_miso (entrada)
//
// Mapa de enderecos de I/O no topo do espaco de 1024 palavras
// ("deslocamento a partir do topo", mesmo padrao de antes):
//
//   0x3EE  UART_STATUS   (bit0 = byte novo chegou)
//   0x3EF  UART_DATA     (leitura=byte recebido / escrita=byte a transmitir)
//   0x3F0  GPIO_DIR      (bit=1 -> pino em modo saida)
//   0x3F1  GPIO_DATA     (leitura reflete o pino de verdade se for entrada)
//   0x3F2  (livre -- sem pull-up interno, use resistor externo)
//   0x3F3  GPIO_PWM_EN
//   0x3F4..0x3FB  GPIO_PWM_DUTY[0..7]  (8 GPIOs)
//   0x3FC..0x3FF  (reservado / nao usado)
//
// Programa e dados do usuario: enderecos 0x000..0x3ED (1006 palavras
// uteis = 503 instrucoes no formato compacto de 2 palavras/instrucao).
module top #(
    parameter CLK_FREQ_HZ = 16_000_000
)(
    input  wire clk,
    input  wire rst,
    input  wire [7:0] gpio_pad_in,
    output wire [7:0] gpio_pad_out,
    output wire [7:0] gpio_pad_oe,
    input  wire prog_rx,
    output wire prog_tx,
    output wire spi_cs,
    output wire spi_sck,
    output wire spi_mosi,
    input  wire spi_miso,
    output wire sram_cs,
    output wire sram_sck,
    output wire sram_mosi,
    input  wire sram_miso
);
    wire [9:0]  mem_addr;
    wire [15:0] mem_wdata, mem_rdata;
    wire        mem_we, mem_req;

    // ---- busca de instrucao (XIP): direto da flash, via flash_fetch ----
    // Nao ha' mais boot_ctrl nem copia flash->SRAM: a CPU sai do reset
    // e ja comeca a buscar a instrucao em pc=0 direto da flash. A SRAM
    // externa fica inteira disponivel pra dados do usuario.
    wire [9:0]  fetch_addr;
    wire [15:0] fetch_rdata;
    wire        fetch_req, fetch_ready;

    flash_fetch u_fetch (
        .clk(clk), .rst(rst),
        .spi_cs(spi_cs), .spi_sck(spi_sck), .spi_mosi(spi_mosi), .spi_miso(spi_miso),
        .addr(fetch_addr), .req(fetch_req),
        .rdata(fetch_rdata), .ready(fetch_ready)
    );

    cpu #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) u_cpu(.clk(clk), .rst(rst), .mem_addr(mem_addr),
              .mem_wdata(mem_wdata), .mem_rdata(mem_rdata),
              .mem_we(mem_we), .mem_req(mem_req), .mem_ready(mem_ready),
              .fetch_addr(fetch_addr), .fetch_req(fetch_req),
              .fetch_rdata(fetch_rdata), .fetch_ready(fetch_ready),
              .halted(), .flag_z());

    wire sel_uart_status = (mem_addr == 10'h3EE);
    wire sel_uart_data   = (mem_addr == 10'h3EF);
    wire sel_gpio        = (mem_addr >= 10'h3F0);
    wire sel_ram         = !(sel_uart_status || sel_uart_data || sel_gpio);

    // ---- controlador da SRAM serial externa (substitui o antigo array ram[]) ----
    wire [15:0] ext_ram_rdata;
    wire        ext_ram_ready;
    ext_ram_ctrl u_ext_ram (
        .clk(clk), .rst(rst),
        .addr(mem_addr), .wdata(mem_wdata), .we(mem_we),
        .req(mem_req && sel_ram),
        .rdata(ext_ram_rdata), .ready(ext_ram_ready),
        .sram_cs(sram_cs), .sram_sck(sram_sck), .sram_mosi(sram_mosi), .sram_miso(sram_miso)
    );

    // Enderecos que nao sao RAM (UART/GPIO, ja internos) respondem 1 ciclo
    // depois do pedido -- mesma latencia que o projeto original ja tinha
    // pra esses casos. Enderecos de RAM esperam o tempo real da transacao
    // SPI (ext_ram_ready).
    reg immediate_ready;
    always @(posedge clk) immediate_ready <= mem_req && !sel_ram;
    wire mem_ready = sel_ram ? ext_ram_ready : immediate_ready;

    wire uart_rx_ready;
    wire [7:0] uart_rx_data;
    reg  uart_rx_ack;
    always @(posedge clk) uart_rx_ack <= mem_req && sel_uart_data && !mem_we;

    uart u_uart(
        .clk(clk), .rst(rst),
        .rx_pin(prog_rx), .tx_pin(prog_tx),
        .rx_data(uart_rx_data), .rx_ready(uart_rx_ready), .rx_ack(uart_rx_ack),
        .tx_start(mem_req && sel_uart_data && mem_we), .tx_data(mem_wdata[7:0]), .tx_busy()
    );

    wire [15:0] gpio_rdata;
    gpio u_gpio(.clk(clk), .rst(rst),
                .addr(mem_addr), .wdata(mem_wdata), .we(mem_req && mem_we && sel_gpio),
                .rdata(gpio_rdata),
                .pad_in(gpio_pad_in), .pad_out(gpio_pad_out), .pad_oe(gpio_pad_oe));

    assign mem_rdata = sel_uart_status ? {15'b0, uart_rx_ready} :
                        sel_uart_data   ? {8'b0, uart_rx_data}   :
                        sel_gpio        ? gpio_rdata             :
                        ext_ram_rdata;
endmodule