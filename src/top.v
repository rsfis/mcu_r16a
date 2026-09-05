// top.v — v4
//
// Mudancas em relacao a Parte 11 do tutorial original:
//   - RAM e' 1024 palavras -- reg [15:0] ram [0:1023] (2KB).
//   - mem_addr e' 10 bits em toda a hierarquia.
//   - gpio_pad deixou de ser "inout"; agora sao 3 sinais separados
//     (gpio_pad_in/out/oe) que casam direto com o modelo
//     ui_in/uo_out/uio_* do wrapper Tiny Tapeout (ver tt_um_*.v).
//   - Mapa de enderecos de I/O no topo do espaco de 1024 palavras
//     ("deslocamento a partir do topo", mesmo padrao de antes):
//
//       0x3EE  UART_STATUS   (bit0 = byte novo chegou)
//       0x3EF  UART_DATA     (leitura=byte recebido / escrita=byte a transmitir)
//       0x3F0  GPIO_DIR      (bit=1 -> pino em modo saida)
//       0x3F1  GPIO_DATA     (leitura reflete o pino de verdade se for entrada)
//       0x3F2  (livre -- sem pull-up interno, use resistor externo)
//       0x3F3  GPIO_PWM_EN
//       0x3F4..0x3FB  GPIO_PWM_DUTY[0..7]  (8 GPIOs)
//       0x3FC..0x3FF  (reservado / nao usado)
//
//     Programa e dados do usuario: enderecos 0x000..0x3ED (1006 palavras
//     uteis = 503 instrucoes no formato compacto de 2 palavras/instrucao).
module top (
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
    input  wire spi_miso
);
    wire [9:0]  cpu_mem_addr;
    wire [15:0] cpu_mem_wdata, mem_rdata;
    wire cpu_mem_we;

    wire boot_cpu_rst;
    wire [9:0]  boot_ram_addr;
    wire [15:0] boot_ram_wdata;
    wire boot_ram_we;

    reg [15:0] ram [0:1023];   // 2KB = 1024 palavras de 16 bits

    boot_ctrl u_boot(
        .clk(clk), .rst(rst),
        .spi_cs(spi_cs), .spi_sck(spi_sck), .spi_mosi(spi_mosi), .spi_miso(spi_miso),
        .ram_addr(boot_ram_addr), .ram_wdata(boot_ram_wdata), .ram_we(boot_ram_we),
        .cpu_rst(boot_cpu_rst)
    );

    wire cpu_rst_final = rst | boot_cpu_rst;   // CPU so sai do reset quando o boot terminar

    // Enquanto o boot esta rodando (boot_cpu_rst=1), quem escreve na RAM e' a boot_ctrl.
    // Depois que termina, quem escreve e' a CPU normalmente.
    wire [9:0]  mem_addr  = boot_cpu_rst ? boot_ram_addr  : cpu_mem_addr;
    wire [15:0] mem_wdata = boot_cpu_rst ? boot_ram_wdata : cpu_mem_wdata;
    wire        mem_we    = boot_cpu_rst ? boot_ram_we    : cpu_mem_we;

    cpu u_cpu(.clk(clk), .rst(cpu_rst_final), .mem_addr(cpu_mem_addr),
              .mem_wdata(cpu_mem_wdata), .mem_rdata(mem_rdata),
              .mem_we(cpu_mem_we), .halted(), .flag_z());

    wire sel_uart_status = (mem_addr == 10'h3EE);
    wire sel_uart_data   = (mem_addr == 10'h3EF);
    wire sel_gpio        = (mem_addr >= 10'h3F0);

    always @(posedge clk) begin
        if (mem_we && !sel_uart_data && !sel_gpio)
            ram[mem_addr] <= mem_wdata;
    end

    wire uart_rx_ready;
    wire [7:0] uart_rx_data;
    reg  uart_rx_ack;
    always @(posedge clk) uart_rx_ack <= sel_uart_data && !mem_we;

    uart u_uart(
        .clk(clk), .rst(rst),
        .rx_pin(prog_rx), .tx_pin(prog_tx),
        .rx_data(uart_rx_data), .rx_ready(uart_rx_ready), .rx_ack(uart_rx_ack),
        .tx_start(sel_uart_data && mem_we), .tx_data(mem_wdata[7:0]), .tx_busy()
    );

    wire [15:0] gpio_rdata;
    gpio u_gpio(.clk(clk), .rst(rst),
                .addr(mem_addr), .wdata(mem_wdata), .we(mem_we && sel_gpio),
                .rdata(gpio_rdata),
                .pad_in(gpio_pad_in), .pad_out(gpio_pad_out), .pad_oe(gpio_pad_oe));

    assign mem_rdata = sel_uart_status ? {15'b0, uart_rx_ready} :
                        sel_uart_data   ? {8'b0, uart_rx_data}   :
                        sel_gpio        ? gpio_rdata             :
                        ram[mem_addr];
endmodule