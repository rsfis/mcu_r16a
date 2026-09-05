// boot_ctrl.v — identico a Parte 11 do tutorial original na logica do
// motor SPI e do sequenciador; a unica mudanca real e' a largura de
// ram_addr (10 bits, porque a RAM alvo e' de 1024 palavras). O formato
// de dados na Flash continua o mesmo: 2 bytes de
// cabecalho (tamanho em PALAVRAS de 16 bits, little-endian) seguidos
// do programa, palavra por palavra, little-endian.
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
    output reg  cpu_rst    // fica em 1 (CPU parada) ate a copia terminar
);
    // ---- motor generico de SPI: transfere 1 byte por vez (modo 0, MSB primeiro) ----
    reg [7:0] tx_byte, rx_byte;
    reg [3:0] bitcnt;
    reg       busy, half, done, start;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            busy <= 0; spi_sck <= 0; bitcnt <= 0; half <= 0; done <= 0;
        end else begin
            done <= 0;
            if (start && !busy) begin
                busy <= 1; bitcnt <= 0; half <= 0;
                spi_mosi <= tx_byte[7];
            end else if (busy) begin
                half <= ~half;
                if (!half) begin
                    spi_sck <= 1;
                    rx_byte <= {rx_byte[6:0], spi_miso};
                end else begin
                    spi_sck <= 0;
                    if (bitcnt == 7) begin
                        busy <= 0; done <= 1;
                    end else begin
                        bitcnt   <= bitcnt + 1;
                        spi_mosi <= tx_byte[6 - bitcnt];
                    end
                end
            end
        end
    end

    // ---- sequenciador: manda comando READ, endereco 0x000000, le o tamanho (2 bytes,
    //      em NUMERO DE PALAVRAS de 16 bits) e depois le esse tanto de PALAVRAS,
    //      escrevendo cada uma inteira (16 bits) numa posicao da RAM. ----
    localparam S_CMD=0, S_CMD_W=1, S_ADDR=2, S_ADDR_W=3,
               S_LEN_LO=4, S_LEN_LO_W=5, S_LEN_HI=6, S_LEN_HI_W=7,
               S_DATA_LO=8, S_DATA_LO_W=9, S_DATA_HI=10, S_DATA_HI_W=11, S_DONE=12;

    reg [3:0]  state;
    reg [1:0]  addr_i;
    reg [15:0] word_count;   // quantas PALAVRAS de 16 bits o programa tem (nao bytes)
    reg [15:0] count;
    reg [7:0]  low_byte;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_CMD; spi_cs <= 1; start <= 0;
            ram_we <= 0; cpu_rst <= 1; count <= 0; addr_i <= 0;
        end else begin
            start  <= 0;
            ram_we <= 0;
            case (state)
                S_CMD: begin
                    spi_cs  <= 0;
                    tx_byte <= 8'h03;      // comando READ
                    start   <= 1;
                    state   <= S_CMD_W;
                end
                S_CMD_W: if (done) begin addr_i <= 0; state <= S_ADDR; end
                S_ADDR: begin
                    tx_byte <= 8'h00;      // 3 bytes de endereco: sempre 0x000000
                    start   <= 1;
                    state   <= S_ADDR_W;
                end
                S_ADDR_W: if (done) begin
                    if (addr_i == 2) state <= S_LEN_LO;
                    else begin addr_i <= addr_i + 1; state <= S_ADDR; end
                end
                // ---- cabecalho: 2 bytes, little-endian, em NUMERO DE PALAVRAS ----
                S_LEN_LO: begin tx_byte <= 8'h00; start <= 1; state <= S_LEN_LO_W; end
                S_LEN_LO_W: if (done) begin low_byte <= rx_byte; state <= S_LEN_HI; end
                S_LEN_HI: begin tx_byte <= 8'h00; start <= 1; state <= S_LEN_HI_W; end
                S_LEN_HI_W: if (done) begin
                    word_count <= {rx_byte, low_byte};
                    state <= S_DATA_LO;
                end
                // ---- payload: 1 palavra (16 bits) por vez = 2 bytes lidos da flash ----
                S_DATA_LO: begin tx_byte <= 8'h00; start <= 1; state <= S_DATA_LO_W; end
                S_DATA_LO_W: if (done) begin low_byte <= rx_byte; state <= S_DATA_HI; end
                S_DATA_HI: begin tx_byte <= 8'h00; start <= 1; state <= S_DATA_HI_W; end
                S_DATA_HI_W: if (done) begin
                    ram_addr  <= count[9:0];
                    ram_wdata <= {rx_byte, low_byte};  // palavra completa de 16 bits
                    ram_we    <= 1;
                    if (count + 1 >= word_count) begin
                        spi_cs <= 1;
                        state  <= S_DONE;
                    end else begin
                        count <= count + 1;
                        state <= S_DATA_LO;
                    end
                end
                S_DONE: cpu_rst <= 0;      // libera a CPU: comeca a rodar a partir da RAM[0]
            endcase
        end
    end
endmodule