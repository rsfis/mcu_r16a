// spi_flash_model.v — modelo comportamental de testbench (NAO sintetizavel).
// v2 (XIP): agora HONRA de verdade os 3 bytes de endereco enviados no
// comando READ (0x03), em vez de assumir sempre 0x000000 -- necessario
// porque flash_fetch.v faz acesso RANDOMICO (nao so' sequencial a
// partir de 0) toda vez que a CPU da' um desvio/JMP/CALL/RET. Depois
// dos 3 bytes de endereco, o streaming continua sequencial (endereco
// auto-incrementando) enquanto cs ficar em 0 -- igual ao modo
// "sequential read" padrao de qualquer flash SPI real e ao jeito que
// flash_fetch.v mantem CS baixo entre fetches consecutivos.
//
// Conteudo da flash = o mesmo program.hex de sempre (2 bytes de
// cabecalho com a contagem de palavras, little-endian, seguidos do
// programa palavra a palavra, tambem little-endian) -- nenhuma
// conversao de formato e necessaria, e' o arquivo tal e qual.
module spi_flash_model #(
    parameter HEXFILE = "program.hex",
    parameter DEPTH   = 65536   // bytes disponiveis na flash simulada
)(
    input  wire cs,     // ativo em 0
    input  wire sck,
    input  wire mosi,
    output reg  miso
);
    reg [7:0] mem [0:DEPTH-1];
    integer i;
    initial begin
        for (i = 0; i < DEPTH; i = i + 1) mem[i] = 8'h00;
        $readmemh(HEXFILE, mem);
    end

    reg [2:0]  bitpos;      // proximo bit a amostrar/gerar, contando de 7 a 0
    reg [2:0]  byte_index;  // 0=comando, 1..3=endereco (24 bits, so os 16 baixos usados), 4+=dados
    reg [7:0]  shreg;       // acumula os bits de mosi recebidos no byte de cmd/endereco atual
    reg [7:0]  addr_mid;    // guarda o byte de endereco[15:8] (byte_index==2) ate formar os 16 bits
    reg [23:0] data_addr;   // endereco corrente dentro de 'mem' (usado so' na fase de dados)
    reg [7:0]  cur_out;     // byte de dado sendo deslocado pra fora agora

    always @(negedge cs) begin
        bitpos     <= 7;
        byte_index <= 0;
        shreg      <= 0;
    end

    always @(posedge sck) begin
        if (byte_index < 4) begin
            // ---- fase de comando (byte 0, ignorado) / endereco (bytes 1..3) ----
            if (bitpos == 0) begin
                // ultimo bit do byte atual: {shreg[6:0], mosi} = byte completo agora
                if (byte_index == 2) addr_mid <= {shreg[6:0], mosi};        // addr[15:8]
                if (byte_index == 3) begin                                  // addr[7:0] -> endereco pronto
                    data_addr <= {8'b0, addr_mid, {shreg[6:0], mosi}};
                    cur_out   <= mem[{addr_mid, {shreg[6:0], mosi}}];
                    miso      <= mem[{addr_mid, {shreg[6:0], mosi}}][7];
                end
                byte_index <= byte_index + 1;
                bitpos     <= 7;
                shreg      <= 0;
            end else begin
                shreg  <= {shreg[6:0], mosi};
                bitpos <= bitpos - 1;
            end
        end else begin
            // ---- fase de dados: desloca cur_out pra fora, 1 bit por posedge ----
            if (bitpos == 0) begin
                data_addr <= data_addr + 1;
                cur_out   <= mem[data_addr + 1];
                miso      <= mem[data_addr + 1][7];
                bitpos    <= 7;
            end else begin
                miso   <= cur_out[bitpos - 1];
                bitpos <= bitpos - 1;
            end
        end
    end
endmodule
