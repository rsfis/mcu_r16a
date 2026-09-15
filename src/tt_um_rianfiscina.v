// tt_um_rianfiscina.v — wrapper universal de simulacao/GDS (v2, SRAM+flash externas)
//
// Mapa de pinos:
//   ui_in[0]  = prog_rx    (entrada da UART de "boot/depuracao")
//   ui_in[1]  = spi_miso   (linha MISO vinda da flash SPI, boot)
//   ui_in[2]  = sram_miso  (linha MISO vinda da SRAM SPI externa) -- NOVO
//   ui_in[7:3]= nao usados por este projeto
//
//   uo_out[0] = prog_tx    (saida da UART)
//   uo_out[1] = spi_cs     (chip-select da flash, ativo em 0)
//   uo_out[2] = spi_sck    (clock da flash)
//   uo_out[3] = spi_mosi   (linha MOSI para a flash)
//   uo_out[4] = sram_cs    (chip-select da SRAM externa, ativo em 0)  -- NOVO
//   uo_out[5] = sram_sck   (clock da SRAM externa)                    -- NOVO
//   uo_out[6] = sram_mosi  (linha MOSI para a SRAM externa)           -- NOVO
//   uo_out[7] = 0 (nao usado)
//
//   uio_in/uio_out/uio_oe = os 8 GPIOs de uso geral do projeto,
//   passados direto para/de gpio.v via top.v (uio[4] e' o LED usado
//   nos programas de exemplo, mas isso e' decisao do software, nao
//   do wrapper).
//
// O barramento da SRAM (cs/sck/mosi/miso) e' FISICAMENTE SEPARADO do
// barramento da flash -- nao ha' compartilhamento de pinos entre os
// dois chips externos, entao nao ha' risco de contencao no barramento
// mesmo com os dois dispositivos conectados ao mesmo tempo na placa.
module tt_um_rianfiscina #(
    parameter CLK_FREQ_HZ = 16_000_000   // frequencia REAL do "clk" (ajuste aqui pra bater com o testbench/silicio)
)(
    input  wire [7:0] ui_in,    // entradas dedicadas
    output wire [7:0] uo_out,   // saidas dedicadas
    input  wire [7:0] uio_in,   // IO bidirecional (entrada)
    output wire [7:0] uio_out,  // IO bidirecional (saida)
    output wire [7:0] uio_oe,   // IO bidirecional (habilita saida, 1=saida)
    input  wire       ena,      // 1 quando o design esta habilitado (sempre 1 na simulacao)
    input  wire       clk,      // clock do sistema
    input  wire        rst_n     // reset ativo em 0 (padrao Tiny Tapeout)
);

    // ---- pinos dedicados: UART de boot/depuracao + SPI da flash + SPI da SRAM ----
    wire prog_rx   = ui_in[0];
    wire spi_miso  = ui_in[1];
    wire sram_miso = ui_in[2];
    // ui_in[7:3] nao usados por este projeto

    wire prog_tx, spi_cs, spi_sck, spi_mosi;
    wire sram_cs, sram_sck, sram_mosi;
    assign uo_out = {1'b0, sram_mosi, sram_sck, sram_cs, spi_mosi, spi_sck, spi_cs, prog_tx};

    // ---- instancia unica do sistema (CPU + SRAM externa + boot + GPIO + UART) ----
    top #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) u_top (
        .clk          (clk),
        .rst          (~rst_n),
        .gpio_pad_in  (uio_in),
        .gpio_pad_out (uio_out),
        .gpio_pad_oe  (uio_oe),
        .prog_rx      (prog_rx),
        .prog_tx      (prog_tx),
        .spi_cs       (spi_cs),
        .spi_sck      (spi_sck),
        .spi_mosi     (spi_mosi),
        .spi_miso     (spi_miso),
        .sram_cs      (sram_cs),
        .sram_sck     (sram_sck),
        .sram_mosi    (sram_mosi),
        .sram_miso    (sram_miso)
    );

    // `ena` nao e' usado pela logica interna do projeto (o tutorial
    // original tambem nao usa); mantido apenas para casar com a
    // interface padrao tt_um_* exigida pelo Tiny Tapeout.
    wire _unused_ena = ena;

    // =====================================================================
    // ---- Sinais de depuracao, expostos "no plano" pra facilitar o wave ----
    // Nada aqui influencia a simulacao: sao apenas fios (assign) lendo
    // sinais que ja existem dentro da hierarquia.
    // =====================================================================

    // XIP: nao ha' mais fase de boot/copia -- estes sinais acompanham a
    // busca de instrucao direto da flash (ver flash_fetch.v)
    wire [9:0]  dbg_fetch_addr  = u_top.fetch_addr;
    wire        dbg_fetch_req   = u_top.fetch_req;
    wire [15:0] dbg_fetch_rdata = u_top.fetch_rdata;
    wire        dbg_fetch_ready = u_top.fetch_ready;

    // Estado da CPU
    wire [9:0]  dbg_cpu_pc      = u_top.u_cpu.pc;
    wire [3:0]  dbg_cpu_state   = u_top.u_cpu.state;
    wire [7:0]  dbg_cpu_opcode  = u_top.u_cpu.opcode;
    wire [3:0]  dbg_cpu_regdst  = u_top.u_cpu.reg_dst;
    wire [3:0]  dbg_cpu_regsrc  = u_top.u_cpu.reg_src;
    wire        dbg_cpu_halted  = u_top.u_cpu.halted;
    wire        dbg_cpu_flagz   = u_top.u_cpu.flag_z;
    wire [9:0]  dbg_mem_addr    = u_top.mem_addr;    // agora so' DADOS (SRAM), programa nao passa mais por aqui
    wire [15:0] dbg_mem_wdata   = u_top.mem_wdata;
    wire [15:0] dbg_mem_rdata   = u_top.mem_rdata;
    wire        dbg_mem_we      = u_top.mem_we;
    wire        dbg_mem_req     = u_top.mem_req;
    wire        dbg_mem_ready   = u_top.mem_ready;

    // Banco de registradores (R0..R14) -- util pra acompanhar valores
    // sem precisar abrir o array regfile manualmente no GTKWave.
    wire [15:0] dbg_r0  = u_top.u_cpu.regfile[0];
    wire [15:0] dbg_r1  = u_top.u_cpu.regfile[1];
    wire [15:0] dbg_r2  = u_top.u_cpu.regfile[2];
    wire [15:0] dbg_r3  = u_top.u_cpu.regfile[3];
    wire [15:0] dbg_r4  = u_top.u_cpu.regfile[4];
    wire [15:0] dbg_r5  = u_top.u_cpu.regfile[5];
    wire [15:0] dbg_r6  = u_top.u_cpu.regfile[6];
    wire [15:0] dbg_r7  = u_top.u_cpu.regfile[7];
    wire [15:0] dbg_r8  = u_top.u_cpu.regfile[8];
    wire [15:0] dbg_r9  = u_top.u_cpu.regfile[9];
    wire [15:0] dbg_r10 = u_top.u_cpu.regfile[10];
    wire [15:0] dbg_r11 = u_top.u_cpu.regfile[11];
    wire [15:0] dbg_r12 = u_top.u_cpu.regfile[12];
    wire [15:0] dbg_r13 = u_top.u_cpu.regfile[13];
    wire [15:0] dbg_r14 = u_top.u_cpu.regfile[14];

    // GPIO (direcao / dado / PWM)
    wire [7:0]  dbg_gpio_dir    = u_top.u_gpio.dir_reg;
    wire [7:0]  dbg_gpio_data   = u_top.u_gpio.data_reg;
    wire [7:0]  dbg_gpio_pwm_en = u_top.u_gpio.pwm_en_reg;

    // UART
    wire        dbg_uart_rx_ready = u_top.uart_rx_ready;
    wire [7:0]  dbg_uart_rx_data  = u_top.uart_rx_data;

    // SPI da flash (boot)
    wire        dbg_spi_cs   = spi_cs;
    wire        dbg_spi_sck  = spi_sck;
    wire        dbg_spi_mosi = spi_mosi;
    wire        dbg_spi_miso = spi_miso;

    // SPI da SRAM externa
    wire        dbg_sram_cs   = sram_cs;
    wire        dbg_sram_sck  = sram_sck;
    wire        dbg_sram_mosi = sram_mosi;
    wire        dbg_sram_miso = sram_miso;

endmodule