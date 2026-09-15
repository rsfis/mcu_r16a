`timescale 1ns/1ps
// tb.v — testbench de topo, clock real de 16 MHz (periodo = 62.5 ns).
//
// Liga o tt_um_rianfiscina aos dois modelos de SPI (flash de boot +
// SRAM externa) exatamente como a placa de verdade faria: cada
// dispositivo tem seu proprio barramento fisico, sem compartilhamento
// (ver comentario em tt_um_rianfiscina.v).
//
//   uo_out[1]=spi_cs, uo_out[2]=spi_sck, uo_out[3]=spi_mosi  -> flash
//   ui_in[1]=spi_miso                                        <- flash
//
//   uo_out[4]=sram_cs, uo_out[5]=sram_sck, uo_out[6]=sram_mosi -> sram
//   ui_in[2]=sram_miso                                          <- sram
//
// Rode com:  make   (ver Makefile)
module tb;
    reg clk = 0;
    reg rst_n = 0;

    reg  [7:0] ui_in  = 8'h00;
    wire [7:0] uo_out;
    reg  [7:0] uio_in = 8'h00;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;

    // ---- clock de 16 MHz de verdade: periodo = 1/16e6 s = 62.5 ns ----
    always #31.25 clk = ~clk;   // meio periodo = 31.25 ns -> periodo total 62.5 ns

    // ---- reset: mantem em reset por alguns ciclos e solta ----
    initial begin
        rst_n = 0;
        repeat (10) @(posedge clk);
        rst_n = 1;
    end

    // ---- barramento da flash de boot (fisicamente separado do da SRAM) ----
    wire flash_cs   = uo_out[1];
    wire flash_sck  = uo_out[2];
    wire flash_mosi = uo_out[3];
    wire flash_miso;

    spi_flash_model #(.HEXFILE("program.hex")) u_flash (
        .cs(flash_cs), .sck(flash_sck), .mosi(flash_mosi), .miso(flash_miso)
    );

    // ---- barramento da SRAM externa (fisicamente separado do da flash) ----
    wire sram_cs   = uo_out[4];
    wire sram_sck  = uo_out[5];
    wire sram_mosi = uo_out[6];
    wire sram_miso;

    spi_sram_model u_sram (
        .cs(sram_cs), .sck(sram_sck), .mosi(sram_mosi), .miso(sram_miso)
    );

    always @(*) begin
        ui_in[0] = 1'b1;       // prog_rx ocioso (UART em idle = linha alta)
        ui_in[1] = flash_miso;
        ui_in[2] = sram_miso;
        ui_in[7:3] = 5'b0;
    end

    // ---- DUT: CLK_FREQ_HZ = 16_000_000, batendo com o clock de verdade acima ----
    tt_um_rianfiscina #(.CLK_FREQ_HZ(16_000_000)) dut (
        .ui_in   (ui_in),
        .uo_out  (uo_out),
        .uio_in  (uio_in),
        .uio_out (uio_out),
        .uio_oe  (uio_oe),
        .ena     (1'b1),
        .clk     (clk),
        .rst_n   (rst_n)
    );

    // ---- dump pro GTKWave ----
    initial begin
        $dumpfile("wave.vcd");
        $dumpvars(0, tb);
        $timeformat(-9, 3, " ns", 12);  // deixa %t sempre em ns, 3 casas decimais
    end

    // ---- roda tempo suficiente pra ver pelo menos 1 ciclo completo do
    //      pisca-pisca (2 x 50 ms de delay + boot + overhead de instrucoes) ----
    initial begin
        #150_000_000;   // 150 ms de tempo simulado
        $display("fim da simulacao (150 ms simulados)");
        $finish;
    end

    // ---- print de log toda vez que o GPIO (uio_out) muda, pra
    //      conferir o timing sem precisar abrir o GTKWave ----
    real t_last_change = 0;
    always @(uio_out) begin
        // $time bruto e' contado na unidade local do timescale (1 ns
        // aqui), entao 1 ms = 1e6 ns.
        $display("t=%t (%.4f ms) uio_out=0x%02x  (delta desde a ultima mudanca: %.4f ms)",
                  $time, $time/1.0e6, uio_out, ($time - t_last_change)/1.0e6);
        t_last_change = $time;
    end
endmodule