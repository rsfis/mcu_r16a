// boot_ctrl.v — v3 (adaptado para RAM externa via SPI)
//
// MUDANCA em relacao a versao anterior: antes, cada palavra lida da
// flash era gravada DIRETO num array de RAM interno (1 ciclo,
// imediato). Agora a RAM e' a SRAM serial externa 23LCV04M, acessada
// atraves de ext_ram_ctrl.v (mesmo controlador que a CPU usa depois
// do boot) -- entao cada palavra lida da flash agora exige uma
// transacao SPI completa de ESCRITA na SRAM externa (dezenas de
// ciclos), usando o mesmo handshake ram_addr/ram_wdata/ram_we/ram_req
// -> aguarda ram_ready que cpu.v tambem usa (ver top.v, onde os dois
// "clientes" -- boot_ctrl durante o boot, cpu depois -- sao
// multiplexados para o mesmo ext_ram_ctrl).
//
// O lado da FLASH continua EXATAMENTE como antes: um unico comando
// READ (0x03) e' mandado uma vez, com spi_cs mantido em 0 (leitura
// sequencial em streaming, sem reenviar comando/endereco a cada
// palavra) -- essa parte nao muda porque a flash e' so lida (nunca
// escrita) e continua com barramento SPI proprio, totalmente
// independente do barramento da SRAM (pinos fisicos diferentes, sem
// nenhum compartilhamento/contencao entre os dois).
//
// Formato de dados na flash: 2 bytes de cabecalho (tamanho em
// PALAVRAS de 16 bits, little-endian) seguidos do programa, palavra
// por palavra, little-endian -- inalterado.
module boot_ctrl (
    input  wire clk,
    input  wire rst,
    output reg  spi_cs,
    output reg  spi_sck,
    output reg  spi_mosi,
    input  wire spi_miso,
    output reg  [9:0]  ram_addr,
    output reg  [15:0] ram_wdata,
    output reg  ram_we,
    output reg  ram_req,      // NOVO: pulso de 1 ciclo pedindo escrita na SRAM externa
    input  wire ram_ready,    // NOVO: pulso de 1 ciclo = escrita na SRAM externa terminou
    output reg  cpu_rst       // fica em 1 (CPU parada) ate a copia terminar
);
    // ---- motor SPI da flash (mesmo motor generico usado por ext_ram_ctrl) ----
    reg        eng_start;
    reg [7:0]  eng_tx;
    wire [7:0] eng_rx;
    wire       eng_done;

    spi_byte_engine u_eng (
        .clk(clk), .rst(rst),
        .start(eng_start), .tx_byte(eng_tx), .rx_byte(eng_rx),
        .busy(), .done(eng_done),
        .sck(spi_sck), .mosi(spi_mosi), .miso(spi_miso)
    );

    // ---- sequenciador: manda comando READ pra flash, endereco 0x000000, le o
    //      tamanho (2 bytes, em NUMERO DE PALAVRAS de 16 bits) e depois le esse
    //      tanto de PALAVRAS, escrevendo cada uma (16 bits) na SRAM externa via
    //      ext_ram_ctrl (uma transacao SPI de escrita completa por palavra). ----
    localparam S_CMD=0, S_CMD_W=1, S_ADDR=2, S_ADDR_W=3,
               S_LEN_LO=4, S_LEN_LO_W=5, S_LEN_HI=6, S_LEN_HI_W=7,
               S_DATA_LO=8, S_DATA_LO_W=9, S_DATA_HI=10, S_DATA_HI_W=11,
               S_WRITE_WAIT=12, S_DONE=13;

    reg [3:0]  state;
    reg [1:0]  addr_i;
    reg [15:0] word_count;   // quantas PALAVRAS de 16 bits o programa tem (nao bytes)
    reg [15:0] count;
    reg [7:0]  low_byte;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_CMD; spi_cs <= 1; eng_start <= 0;
            ram_we <= 0; ram_req <= 0; cpu_rst <= 1; count <= 0; addr_i <= 0;
        end else begin
            eng_start <= 0;
            ram_we    <= 0;
            ram_req   <= 0;
            case (state)
                S_CMD: begin
                    spi_cs    <= 0;
                    eng_tx    <= 8'h03;      // comando READ
                    eng_start <= 1;
                    state     <= S_CMD_W;
                end
                S_CMD_W: if (eng_done) begin addr_i <= 0; state <= S_ADDR; end
                S_ADDR: begin
                    eng_tx    <= 8'h00;      // 3 bytes de endereco: sempre 0x000000
                    eng_start <= 1;
                    state     <= S_ADDR_W;
                end
                S_ADDR_W: if (eng_done) begin
                    if (addr_i == 2) state <= S_LEN_LO;
                    else begin addr_i <= addr_i + 1; state <= S_ADDR; end
                end
                // ---- cabecalho: 2 bytes, little-endian, em NUMERO DE PALAVRAS ----
                S_LEN_LO: begin eng_tx <= 8'h00; eng_start <= 1; state <= S_LEN_LO_W; end
                S_LEN_LO_W: if (eng_done) begin low_byte <= eng_rx; state <= S_LEN_HI; end
                S_LEN_HI: begin eng_tx <= 8'h00; eng_start <= 1; state <= S_LEN_HI_W; end
                S_LEN_HI_W: if (eng_done) begin
                    word_count <= {eng_rx, low_byte};
                    state <= (({eng_rx, low_byte}) == 16'h0000) ? S_DONE : S_DATA_LO;
                end
                // ---- payload: 1 palavra (16 bits) por vez = 2 bytes lidos da flash ----
                S_DATA_LO: begin eng_tx <= 8'h00; eng_start <= 1; state <= S_DATA_LO_W; end
                S_DATA_LO_W: if (eng_done) begin low_byte <= eng_rx; state <= S_DATA_HI; end
                S_DATA_HI: begin eng_tx <= 8'h00; eng_start <= 1; state <= S_DATA_HI_W; end
                S_DATA_HI_W: if (eng_done) begin
                    // Palavra completa (16 bits) montada -- pede a ext_ram_ctrl
                    // pra gravar na SRAM externa, endereco = indice da palavra.
                    ram_addr  <= count[9:0];
                    ram_wdata <= {eng_rx, low_byte};
                    ram_we    <= 1;
                    ram_req   <= 1;
                    state     <= S_WRITE_WAIT;
                end
                S_WRITE_WAIT: if (ram_ready) begin
                    if (count + 1 >= word_count) begin
                        spi_cs <= 1;      // libera o barramento da flash (streaming acabou)
                        state  <= S_DONE;
                    end else begin
                        count <= count + 1;
                        state <= S_DATA_LO;
                    end
                end
                S_DONE: cpu_rst <= 0;      // libera a CPU: comeca a rodar a partir da SRAM[0]
                default: state <= S_DONE;
            endcase
        end
    end
endmodule
