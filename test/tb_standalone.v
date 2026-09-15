`default_nettype none
`timescale 1ns / 1ps

// tb_standalone.v — testbench AUTOCONTIDO de proposito geral (nao depende
// de cocotb nem dos modelos em test/spi_flash_model.v / test/spi_sram_model.v
// -- os modelos de flash e SRAM externas vao inline aqui embaixo, entao
// basta compilar este arquivo + os fontes de src/ pra simular qualquer
// program.hex).
//
// Serve pra rodar rapido qualquer programa novo (ex: testar CALL/RET/AND)
// sem precisar subir o ambiente cocotb inteiro. Ideal pra debug: alem do
// .vcd pro GTKWave, imprime no console:
//   - cada instrucao no instante em que ela entra em EXEC, ja decodificada
//     (mnemonico, Rd/Rs, operando) MAIS uma foto de TODO o regfile (R0-R14),
//     SP, flag_z e halted naquele instante (que e' exatamente o estado
//     produzido pela instrucao ANTERIOR, entao da' pra acompanhar o efeito
//     de cada instrucao sem precisar abrir o .vcd);
//   - toda escrita/leitura de UART (via UART_DATA);
//   - toda mudanca nos pinos uio (GPIO);
//   - inicio/fim do boot (flash -> SRAM externa) e HALT.
//
// Uso:
//   make -C test SIM=icarus TESTBENCH=tb_standalone   (ver Makefile)
// ou direto:
//   iverilog -g2012 -o sim.out -s tb_standalone \
//       test/tb_standalone.v src/*.v
//   vvp sim.out [+hex=program.hex] [+notrace] [+maxtime_ns=500000000]
//
// Plusargs aceitos (todos opcionais):
//   +hex=arquivo.hex     -- qual .hex carregar na flash simulada
//                           (default: "program.hex", igual o boot real espera)
//   +notrace             -- desliga o trace de instrucao por linha
//                           (o dump de regfile no boot/HALT continua ligado)
//   +maxtime_ns=N        -- tempo maximo de simulacao em ns
//                           (default: 500_000_000 = 500ms)
module tb_standalone ();

    // ---- dump pro GTKWave ----
    initial begin
        $dumpfile("wave.vcd");
        $dumpvars(0, tb_standalone);
    end

    // ---- clock e reset ----
    // clock de 16 MHz de verdade: periodo = 1/16e6 s = 62.5 ns
    // (meio periodo = 31.25 ns -> periodo total 62.5 ns), batendo com o
    // CLK_FREQ_HZ = 16_000_000 default de cpu.v/uart.v (ver defparams
    // removidos abaixo).
    reg clk = 0;
    always #31.25 clk = ~clk;

    reg rst_n = 0;
    reg ena   = 1;
    reg [7:0] ui_in  = 8'h01;  // bit0 = prog_rx ocioso em 1 (idle da UART)
    reg [7:0] uio_in = 8'h00;

    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;

    // ---- sinais SPI extraidos de uo_out (ver tt_um_rianfiscina.v) ----
    // uo_out = {1'b0, sram_mosi, sram_sck, sram_cs, spi_mosi, spi_sck, spi_cs, prog_tx}
    wire spi_mosi  = uo_out[3];
    wire spi_sck   = uo_out[2];
    wire spi_cs    = uo_out[1];
    wire prog_tx   = uo_out[0];
    wire sram_mosi = uo_out[6];
    wire sram_sck  = uo_out[5];
    wire sram_cs   = uo_out[4];

    reg spi_miso_r  = 1'b1;
    reg sram_miso_r = 1'b0;
    // liga os miso simulados em ui_in[1] (flash) e ui_in[2] (SRAM), que o
    // wrapper repassa pra top.spi_miso / top.sram_miso
    wire [7:0] ui_in_drv = {ui_in[7:3], sram_miso_r, spi_miso_r, ui_in[0]};

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

    // CLK_FREQ_HZ da CPU e do UART ficam no default de 16_000_000
    // (bate com o clock de 16 MHz de verdade gerado acima), entao
    // DELAY <ms> no .asm corresponde a <ms> milissegundos de tempo
    // simulado de verdade, e o BAUD do UART fica no default de 115200
    // (igual ao hardware real). Antes havia aqui um par de "defparam"
    // reduzindo CLK_FREQ_HZ pra 1000 Hz so' pra acelerar a simulacao
    // -- isso fazia CYCLES_PER_MS virar 1 (em vez de 16000), entao
    // DELAY 50 esperava so' 50 ciclos de clock (~500 ns reais) em vez
    // de 50 ms. Removido pra que o .vcd reflita o tempo real do DELAY.
    // Se precisar simular rapido de novo, reative os defparams abaixo
    // (e lembre que o tempo no wave NAO vai bater com ms reais):
    //   defparam tb_standalone.user_project.u_top.u_cpu.CLK_FREQ_HZ  = 1000;
    //   defparam tb_standalone.user_project.u_top.u_uart.CLK_FREQ_HZ = 1000;
    //   defparam tb_standalone.user_project.u_top.u_uart.BAUD        = 100;

    // =====================================================================
    // Modelo comportamental de flash SPI (modo 0), comando READ (0x03).
    // v2 (XIP): agora HONRA de verdade os 3 bytes de endereco enviados
    // (antes assumia sempre 0x000000, porque so' o boot_ctrl antigo lia
    // a flash, sempre a partir do inicio). Agora flash_fetch.v faz
    // acesso RANDOMICO na flash a cada desvio/JMP/CALL/RET, entao o
    // endereco tem que ser respeitado de verdade -- mesmo esquema ja
    // usado no modelo de SRAM logo abaixo (amostra addr nos bytes 1..3,
    // depois streaming sequencial enquanto CS ficar em 0).
    // =====================================================================
    reg [7:0] flash_mem [0:4095];   // folga generosa (o espaco de enderecos so' tem 1024 palavras = 2048 bytes de programa + cabecalho)
    reg [255:0] hexfile_path;       // buffer pro nome do arquivo vindo do plusarg (ate 32 chars)

    initial begin : load_flash
        integer i;
        for (i = 0; i < 4096; i = i + 1) flash_mem[i] = 8'h00;
        hexfile_path = "program.hex";
        if (!$value$plusargs("hex=%s", hexfile_path))
            hexfile_path = "program.hex";
        $display("t=%0t  carregando imagem da flash de: %0s", $time, hexfile_path);
        $readmemh(hexfile_path, flash_mem);
    end

    integer flash_byte_idx;   // 0=cmd, 1..3=endereco, 4+ = dados
    integer flash_bit_idx;    // 0..7 dentro do byte atual
    reg [7:0]  flash_in_byte;
    reg [23:0] flash_addr;
    reg [7:0]  flash_out_byte;

    // reinicia o "transaction state" toda vez que CS cai
    always @(negedge spi_cs) begin
        flash_byte_idx = 0;
        flash_bit_idx  = 0;
    end

    // amostra MOSI, decodifica comando/endereco e prepara a saida MISO
    // (mesmo esquema do modelo de SRAM logo abaixo)
    always @(posedge spi_sck) begin
        if (!spi_cs) begin
            flash_in_byte = {flash_in_byte[6:0], spi_mosi};
            flash_bit_idx = flash_bit_idx + 1;
            if (flash_bit_idx == 8) begin
                flash_bit_idx = 0;
                case (flash_byte_idx)
                    0: ; // comando (0x03) -- nao precisa guardar, so' READ existe
                    1: flash_addr[23:16] <= flash_in_byte;   // nao usado (so' 16 bits uteis)
                    2: flash_addr[15:8]  <= flash_in_byte;
                    3: begin
                        flash_addr[7:0] <= flash_in_byte;
                        flash_out_byte = flash_mem[{flash_addr[15:8], flash_in_byte}];
                    end
                    default: flash_out_byte = flash_mem[(flash_addr[15:0] + (flash_byte_idx - 3)) & 16'hFFFF];
                endcase
                flash_byte_idx = flash_byte_idx + 1;
            end
        end
    end

    // atualiza a linha MISO na borda de descida (setup antes da proxima subida)
    always @(negedge spi_sck) begin
        if (!spi_cs)
            spi_miso_r = flash_out_byte[7 - flash_bit_idx];
    end

    // =====================================================================
    // Modelo comportamental de SRAM SPI externa (Microchip 23LCV04M-like),
    // modo 0, MSB primeiro. Implementa READ (0x03) e WRITE (0x02) de
    // verdade contra uma memoria de 2048 bytes (cobre as 1024 palavras de
    // 16 bits do projeto -- so os 11 bits baixos do endereco de 3 bytes
    // sao usados). SEM esse modelo, sram_miso fica parado no valor inicial
    // (0) e toda leitura da CPU volta 0x0000 -- o programa nunca roda de
    // verdade (fica so incrementando o PC em cima de opcodes 0x00 = NOP).
    // Isso agora importa mais ainda: CALL/RET usam essa mesma SRAM pra
    // empilhar/desempilhar o endereco de retorno (R14=SP), entao um bug
    // aqui quebraria sub-rotinas de um jeito confuso (retorno pra endereco
    // errado) em vez de travar visivelmente.
    // =====================================================================
    reg [7:0]  sram_mem [0:2047];
    integer    sram_byte_idx;   // 0=cmd, 1..3=endereco, 4+=dados
    integer    sram_bit_idx;    // 0..7 dentro do byte atual
    reg [7:0]  sram_cmd;
    reg [23:0] sram_addr;
    reg [7:0]  sram_in_byte;
    reg [7:0]  sram_out_byte;

    initial begin : init_sram
        integer i;
        for (i = 0; i < 2048; i = i + 1) sram_mem[i] = 8'h00;
    end

    // reinicia o "transaction state" toda vez que CS cai
    always @(negedge sram_cs) begin
        sram_byte_idx = 0;
        sram_bit_idx  = 0;
        sram_out_byte = 8'h00;
    end

    // amostra MOSI na borda de subida do SCK (mesmo instante em que o
    // motor SPI mestre -- spi_byte_engine.v -- amostra o MISO que
    // devolvemos), decodifica comando/endereco e le/escreve sram_mem
    always @(posedge sram_sck) begin
        if (!sram_cs) begin
            sram_in_byte = {sram_in_byte[6:0], sram_mosi};
            sram_bit_idx = sram_bit_idx + 1;
            if (sram_bit_idx == 8) begin
                sram_bit_idx = 0;
                case (sram_byte_idx)
                    0: sram_cmd <= sram_in_byte;
                    1: sram_addr[23:16] <= sram_in_byte;
                    2: sram_addr[15:8]  <= sram_in_byte;
                    3: begin
                        sram_addr[7:0] <= sram_in_byte;
                        // usa sram_in_byte (ja atualizado por blocking assign acima)
                        // em vez de sram_addr[7:0] (so' fica valido no proximo ciclo)
                        if (sram_cmd == 8'h03)
                            sram_out_byte = sram_mem[{sram_addr[10:8], sram_in_byte}];
                    end
                    default: begin
                        if (sram_cmd == 8'h02)
                            sram_mem[(sram_addr[10:0] + (sram_byte_idx - 4)) & 11'h7FF] <= sram_in_byte;
                        else if (sram_cmd == 8'h03)
                            sram_out_byte = sram_mem[(sram_addr[10:0] + (sram_byte_idx - 3)) & 11'h7FF];
                    end
                endcase
                sram_byte_idx = sram_byte_idx + 1;
            end
        end
    end

    // atualiza a linha MISO na borda de descida (setup antes da proxima subida)
    always @(negedge sram_sck) begin
        if (!sram_cs)
            sram_miso_r = sram_out_byte[7 - sram_bit_idx];
    end

    // =====================================================================
    // Sequencia de reset
    // =====================================================================
    initial begin
        $display("t=%0t  iniciando simulacao (rst_n=0)", $time);
        #50;
        rst_n = 1;
        $display("t=%0t  liberando reset (rst_n=1) -- XIP: CPU ja comeca buscando pc=0 direto da flash (sem fase de boot/copia)", $time);
    end

    // =====================================================================
    // ---- DEBUG "DE TUDAO" -----------------------------------------------
    // =====================================================================

    // liga/desliga o trace de instrucao (dump de regfile continua sempre ligado)
    reg trace_on;
    initial trace_on = !$test$plusargs("notrace");

    // atalhos hierarquicos pro estado interno da CPU (so' pra deixar o
    // resto do arquivo legivel)
    wire [3:0]  cpu_state   = user_project.u_top.u_cpu.state;
    wire [9:0]  cpu_pc      = user_project.u_top.u_cpu.pc;
    wire [7:0]  cpu_opcode  = user_project.u_top.u_cpu.opcode;
    wire [3:0]  cpu_rd      = user_project.u_top.u_cpu.reg_dst;
    wire [3:0]  cpu_rs      = user_project.u_top.u_cpu.reg_src;
    wire [15:0] cpu_operand = user_project.u_top.u_cpu.operand;
    wire        cpu_halted  = user_project.u_top.u_cpu.halted;
    wire        cpu_flagz   = user_project.u_top.u_cpu.flag_z;

    localparam ST_EXEC = 4'd4;   // tem que bater com cpu.v (EXEC = 4)

    // mnemonico legivel pro opcode (so' pra impressao -- nao influencia a simulacao)
    task print_mnem;
        input [7:0] op;
        begin
            case (op)
                8'h00: $write("NOP   ");
                8'h01: $write("LOAD  ");
                8'h02: $write("STORE ");
                8'h03: $write("ADD   ");
                8'h04: $write("SUB   ");
                8'h05: $write("JMP   ");
                8'h06: $write("JZ    ");
                8'h07: $write("OUT   ");
                8'h08: $write("IN    ");
                8'h09: $write("MUL   ");
                8'h0A: $write("DIV   ");
                8'h0B: $write("MOD   ");
                8'h0C: $write("OR    ");
                8'h0D: $write("CMP   ");
                8'h0E: $write("NOT   ");
                8'h0F: $write("HALT  ");
                8'h10: $write("DELAY ");
                8'h11: $write("MOV   ");
                8'h12: $write("LOADI ");
                8'h13: $write("LOADR ");
                8'h14: $write("STORER");
                8'h15: $write("AND   ");
                8'h16: $write("CALL  ");
                8'h17: $write("RET   ");
                8'h18: $write("JNZ   ");
                8'h19: $write("ADDI  ");
                8'h1A: $write("SUBI  ");
                8'h1B: $write("INC   ");
                8'h1C: $write("DEC   ");
                8'h1D: $write("PUSH  ");
                8'h1E: $write("POP   ");
                8'h1F: $write("XOR   ");
                8'h20: $write("CMPI  ");
                8'h21: $write("JE    ");
                8'h22: $write("JNE   ");
                8'h23: $write("JL    ");
                8'h24: $write("JNL   ");
                8'h25: $write("JG    ");
                8'h26: $write("JNG   ");
                default: $write("???(%02h)", op);
            endcase
        end
    endtask

    // dump completo do regfile (R0-R14) + flags -- chamado no boot, no
    // HALT e (opcionalmente) a cada instrucao pelo trace abaixo
    task dump_regfile;
        input [399:0] motivo;   // buffer generoso (bytes = ASCII) pra nao truncar a frase
        integer i;
        begin
            $write("        regfile:");
            for (i = 0; i <= 14; i = i + 1)
                $write(" R%0d=%0h", i, user_project.u_top.u_cpu.regfile[i]);
            $write("  flag_z=%0b halted=%0b  (%0s)\n", cpu_flagz, cpu_halted, motivo);
        end
    endtask

    // trace de instrucao: dispara 1x por instrucao, exatamente quando ela
    // entra em EXEC (opcode/rd/rs/operando ja decodificados pelo fetch,
    // regfile ainda com os valores DE ANTES desta instrucao rodar -- ou
    // seja, o resultado da instrucao anterior).
    // OBS: depois do HALT, 'state' fica travado em EXEC pra sempre (cpu.v
    // so' seta halted<=1 sem trocar de estado, e o resto do always fica
    // congelado pelo "if (!halted)"), entao sem o "&& !cpu_halted" abaixo
    // esta linha ia repetir pra sempre a cada ciclo depois do HALT.
    always @(posedge clk) begin
        if (rst_n && trace_on && cpu_state == ST_EXEC && !cpu_halted) begin
            $write("t=%0t  PC=0x%03h  ", $time, cpu_pc);
            print_mnem(cpu_opcode);
            $write("  Rd=R%0d Rs=R%0d operand=0x%04h  |", cpu_rd, cpu_rs, cpu_operand);
            dump_regfile("antes de executar esta instrucao");
        end
    end

    // avisa a transicao para HALT (uma vez so') e tira uma foto final do regfile
    reg halted_seen = 0;
    always @(posedge clk) begin
        if (cpu_halted && !halted_seen) begin
            halted_seen = 1;
            $display("t=%0t  CPU executou HALT em PC=0x%03h", $time, cpu_pc);
            dump_regfile("no HALT");
        end
    end

    // ---- monitor de GPIO (uio) -- imprime sempre que uio_out/uio_oe mudam ----
    always @(uio_out or uio_oe) begin
        if (rst_n)
            $display("t=%0t  GPIO uio_out=0x%02h uio_oe=0x%02h", $time, uio_out, uio_oe);
    end

    // ---- monitor de UART: escrita (TX, CPU->fio) e leitura (RX, byte que chegou) ----
    wire uart_tx_fire = user_project.u_top.mem_req && user_project.u_top.mem_we &&
                         user_project.u_top.sel_uart_data;
    wire [7:0] uart_tx_byte = user_project.u_top.mem_wdata[7:0];
    always @(posedge clk) if (uart_tx_fire)
        $display("t=%0t  UART TX byte=0x%02h ('%0c')", $time, uart_tx_byte,
                  (uart_tx_byte >= 8'h20 && uart_tx_byte < 8'h7F) ? uart_tx_byte : "?");

    wire uart_rx_ready = user_project.u_top.uart_rx_ready;
    wire [7:0] uart_rx_byte = user_project.u_top.uart_rx_data;
    always @(posedge clk) if (uart_rx_ready)
        $display("t=%0t  UART RX byte pronto=0x%02h ('%0c')", $time, uart_rx_byte,
                  (uart_rx_byte >= 8'h20 && uart_rx_byte < 8'h7F) ? uart_rx_byte : "?");

    // ---- tempo maximo de simulacao (configuravel por plusarg) ----
    // Generico: nao depende de nenhum pino, GPIO ou comportamento
    // especifico de programa. Roda ate aqui OU ate um tempo curto depois
    // do HALT (o que vier primeiro) -- assim testes com HALT terminam
    // rapido em vez de esperar os 500ms inteiros.
    integer maxtime_ns;
    initial begin
        if (!$value$plusargs("maxtime_ns=%d", maxtime_ns))
            maxtime_ns = 500_000_000;
        # (maxtime_ns);
        $display("t=%0t  fim do tempo de simulacao -- encerrando", $time);
        $finish;
    end

    // se o programa der HALT, ainda espera uma folga (pra deixar ondas de
    // GPIO/UART em andamento terminarem de aparecer no .vcd) e encerra
    // sozinho, sem precisar dos 500ms cheios.
    always @(posedge clk) begin
        if (halted_seen) begin
            repeat (2000) @(posedge clk);
            $display("t=%0t  HALT detectado ha' um tempo -- encerrando simulacao", $time);
            $finish;
        end
    end

endmodule