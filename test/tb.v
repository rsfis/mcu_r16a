`default_nettype none
`timescale 1ns / 1ps

module tb_full ();

    // ---- dump pro GTKWave ----
    initial begin
        $dumpfile("tb_full.vcd");
        $dumpvars(0, tb_full);
    end

    // ---- clock e reset ----
    reg clk = 0;
    always #5 clk = ~clk;   // 16 MHz "nominal" (so importa se voce nao mexer no CLK_FREQ_HZ)

    reg rst_n = 0;
    reg ena   = 1;
    reg [7:0] ui_in  = 8'h01;  // bit0 = prog_rx ocioso em 1 (idle da UART)
    reg [7:0] uio_in = 8'h00;

    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;

    // ---- sinais SPI extraidos de uo_out (ver tt_um_rianfiscina.v) ----
    // uo_out = {4'b0000, spi_mosi, spi_sck, spi_cs, prog_tx}
    wire spi_mosi = uo_out[3];
    wire spi_sck  = uo_out[2];
    wire spi_cs   = uo_out[1];
    wire prog_tx  = uo_out[0];

    reg spi_miso_r = 1'b1;
    // liga o miso simulado no ui_in[1], que o wrapper repassa pra top.spi_miso
    wire [7:0] ui_in_drv = {ui_in[7:2], spi_miso_r, ui_in[0]};

    tt_um_rianfiscina user_project (
        .ui_in  (ui_in_drv),
        .uo_out (uo_out),
        .uio_in (uio_in),
        .uio_out(uio_out),
        .uio_oe (uio_oe),
        .ena    (ena),
        .clk    (clk),
        .rst_n  (rst_n)
    );

    // acelera a simulacao: reduz CLK_FREQ_HZ visto pela CPU pra DELAY
    // nao levar 100k ciclos por "1 ms". Isso so afeta os calculos de
    // tempo (DELAY / baud da UART) dentro do design, nao muda logica.
    defparam tb_full.user_project.u_top.u_cpu.CLK_FREQ_HZ  = 1000;
    defparam tb_full.user_project.u_top.u_uart.CLK_FREQ_HZ = 1000;
    defparam tb_full.user_project.u_top.u_uart.BAUD        = 100;

    // =====================================================================
    // Modelo comportamental de flash SPI (modo 0), so implementa comando
    // READ (0x03): 1 byte de comando + 3 bytes de endereco (ignorados,
    // boot_ctrl sempre manda 0x000000) + depois transmite os bytes de
    // program.hex a partir do offset 0, em sequencia, MSB primeiro.
    // =====================================================================
    reg [7:0] flash_mem [0:255];   // so precisa caber program.hex (146 bytes)
    initial $readmemh("program.hex", flash_mem);

    integer byte_idx;   // 0=cmd, 1..3=addr, 4+ = dados
    integer bit_idx;    // 0..7 dentro do byte atual
    integer data_ptr;   // indice em flash_mem durante a fase de dados
    reg [7:0] out_byte;

    // reinicia o "transaction state" toda vez que CS cai
    always @(negedge spi_cs) begin
        byte_idx  = 0;
        bit_idx   = 0;
        data_ptr  = 0;
        out_byte  = flash_mem[0];
        spi_miso_r = out_byte[7];
    end

    // amostra MOSI e desloca a saida MISO nas bordas de subida do SCK
    always @(posedge spi_sck) begin
        if (!spi_cs) begin
            bit_idx = bit_idx + 1;
            if (bit_idx == 8) begin
                bit_idx = 0;
                // se o byte que acabou de terminar ja era de payload,
                // avanca o ponteiro para o PROXIMO byte de payload
                if (byte_idx >= 4)
                    data_ptr = data_ptr + 1;
                byte_idx = byte_idx + 1;
                if (byte_idx >= 4)
                    out_byte = flash_mem[data_ptr];
            end
        end
    end

    // atualiza a linha MISO na borda de descida (setup antes da proxima subida)
    always @(negedge spi_sck) begin
        if (!spi_cs)
            spi_miso_r = out_byte[7 - bit_idx];
    end

    // =====================================================================
    // Sequencia de reset
    // =====================================================================
    initial begin
        $display("t=%0t  iniciando simulacao (rst_n=0)", $time);
        #50;
        rst_n = 1;
        $display("t=%0t  liberando reset (rst_n=1) -- comeca o boot via SPI", $time);
    end

    // avisa quando o boot termina e a CPU comeca a executar o programa
    wire boot_cpu_rst = user_project.u_top.boot_cpu_rst;
    wire [15:0] boot_word_count = user_project.u_top.u_boot.word_count;
    always @(negedge boot_cpu_rst) begin
        $display("t=%0t  boot concluido (word_count=%0d palavras lidas da flash) -- CPU comeca a rodar",
                  $time, boot_word_count);
    end

    // ---- tempo maximo de simulacao ----
    // Generico: nao depende de nenhum pino, GPIO ou comportamento
    // especifico de programa. A simulacao roda ate aqui e entao encerra
    // sozinha -- sirva para qualquer program.hex carregado pela flash.
    // 1_000_000_000 ns = 1 segundos (timescale 1ns/1ps deste arquivo).
    initial begin
        #1_000_000_000;
        $display("t=%0t  fim do tempo de simulacao -- encerrando", $time);
        $finish;
    end

endmodule