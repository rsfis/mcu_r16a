// flash_fetch.v — NOVO (substitui boot_ctrl.v)
//
// Busca de instrucao DIRETO da flash externa (execute-in-place / XIP).
// Nao existe mais copia flash->SRAM no boot: cada FETCH_OP/FETCH_OPERAND
// da CPU vira uma leitura na flash, e a SRAM externa fica 100% livre
// para dados do usuario (antes ela guardava o programa inteiro).
//
// Trade-off: acesso RANDOMICO na flash custa uma transacao inteira
// (CMD 0x03 + 3 bytes de endereco + 2 bytes de dado = 6 bytes = 96
// ciclos, igual ao custo de um acesso na SRAM). MAS, como a imensa
// maioria dos fetches e' SEQUENCIAL (pc, pc+1, pc+2, pc+3, ... — so'
// quebra em desvio/JMP/CALL/RET), este modulo mantem spi_cs baixo
// entre fetches e, quando o proximo endereco pedido e' exatamente o
// seguinte do ultimo lido, pula direto pra leitura dos 2 bytes de
// dado (sem reenviar comando/endereco) — igual ao streaming que a
// flash ja fazia no boot antigo. Isso deixa o caminho comum (codigo
// linear) tao rapido quanto o boot original; so' desvios pagam o
// preco cheio de uma nova transacao.
//
// Formato na flash: mantido igual a antes -- 2 bytes de cabecalho
// (tamanho em palavras, little-endian, nao usado por este modulo,
// so' pulado) seguido do programa, palavra por palavra, little-endian.
// endereco_byte_na_flash = 2 + addr*2  (addr = endereco em PALAVRAS,
// o mesmo 'pc' que a CPU ja usava).
module flash_fetch (
    input  wire clk,
    input  wire rst,
    output reg  spi_cs,
    output reg  spi_sck,
    output reg  spi_mosi,
    input  wire spi_miso,

    input  wire [9:0]  addr,     // endereco em palavras (pc ou pc+1)
    input  wire        req,      // pulso de 1 ciclo pedindo a leitura de 'addr'
    output reg  [15:0] rdata,
    output reg          ready    // pulso de 1 ciclo quando rdata fica valido
);
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

    localparam S_IDLE      = 0,
               S_CS_HIGH   = 1,   // deassert de CS por 1 ciclo antes de recomecar (streaming quebrado)
               S_CMD       = 2,
               S_CMD_W     = 3,
               S_ADDR0     = 4,
               S_ADDR0_W   = 5,
               S_ADDR1     = 6,
               S_ADDR1_W   = 7,
               S_ADDR2     = 8,
               S_ADDR2_W   = 9,
               S_DATA_LO   = 10,
               S_DATA_LO_W = 11,
               S_DATA_HI   = 12,
               S_DATA_HI_W = 13;

    reg [3:0]  state;
    reg [9:0]  cur_addr;
    reg [15:0] byte_addr;    // = 2 + cur_addr*2, zero-extendido
    reg [7:0]  low_byte;

    reg        streaming;       // 1 = CS esta baixo e o proximo byte esperado e' sequencial
    reg [9:0]  expected_addr;   // proximo 'addr' que continuaria o streaming sem custo extra

    wire sequential_hit = streaming && (addr == expected_addr);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE; spi_cs <= 1; eng_start <= 0; ready <= 0;
            streaming <= 0; expected_addr <= 0;
        end else begin
            eng_start <= 0;
            ready     <= 0;
            case (state)
                S_IDLE: if (req) begin
                    cur_addr  <= addr;
                    byte_addr <= 16'd2 + {addr, 1'b0};
                    if (sequential_hit) begin
                        // continua o streaming: CS ja esta baixo, so le os 2 bytes
                        state <= S_DATA_LO;
                    end else if (streaming) begin
                        // quebra de sequencia (desvio) com CS ja baixo: precisa
                        // soltar CS por >=1 ciclo antes de mandar novo comando
                        state <= S_CS_HIGH;
                    end else begin
                        // primeiro fetch (pos-reset): CS ja esta em repouso (alto)
                        state <= S_CMD;
                    end
                end

                S_CS_HIGH: begin
                    spi_cs <= 1;
                    state  <= S_CMD;
                end

                S_CMD: begin
                    spi_cs    <= 0;
                    eng_tx    <= 8'h03;      // comando READ
                    eng_start <= 1;
                    state     <= S_CMD_W;
                end
                S_CMD_W: if (eng_done) state <= S_ADDR0;

                S_ADDR0:   begin eng_tx <= 8'h00;             eng_start <= 1; state <= S_ADDR0_W; end
                S_ADDR0_W: if (eng_done) state <= S_ADDR1;
                S_ADDR1:   begin eng_tx <= byte_addr[15:8];   eng_start <= 1; state <= S_ADDR1_W; end
                S_ADDR1_W: if (eng_done) state <= S_ADDR2;
                S_ADDR2:   begin eng_tx <= byte_addr[7:0];    eng_start <= 1; state <= S_ADDR2_W; end
                S_ADDR2_W: if (eng_done) state <= S_DATA_LO;

                S_DATA_LO:   begin eng_tx <= 8'h00; eng_start <= 1; state <= S_DATA_LO_W; end
                S_DATA_LO_W: if (eng_done) begin low_byte <= eng_rx; state <= S_DATA_HI; end
                S_DATA_HI:   begin eng_tx <= 8'h00; eng_start <= 1; state <= S_DATA_HI_W; end
                S_DATA_HI_W: if (eng_done) begin
                    rdata         <= {eng_rx, low_byte};
                    ready         <= 1;
                    streaming     <= 1;
                    expected_addr <= cur_addr + 10'd1;   // CS continua baixo: proximo fetch pode ser rapido
                    state         <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
