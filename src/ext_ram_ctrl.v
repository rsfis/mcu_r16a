// ext_ram_ctrl.v — NOVO
//
// Substitui o antigo "reg [15:0] ram [0:1023]" (RAM interna) por um
// controlador SPI que fala com a SRAM serial EXTERNA especificada no
// BOM.txt: Microchip 23LCV04M-I/P (SRAM serial de 4 Mbit = 512K x 8,
// interface SPI, modo Byte por padrao no power-up).
//
// Comandos usados (familia 23LC/23A/23LCV da Microchip, modo Byte,
// SPI modo 0): READ = 0x03, WRITE = 0x02, seguidos de 3 bytes de
// endereco (24 bits, MSB primeiro -- os bits mais altos sao "nao
// usados" pois o chip so tem 19 bits de endereco de verdade, mas os
// 3 bytes tem que ser enviados do mesmo jeito, igual a flash).
// ATENCAO: confira esses codigos de comando contra o datasheet real
// do 23LCV04M antes de gravar a placa -- foram usados de memoria
// (sem acesso a internet neste ambiente) por serem o padrao de toda
// a familia 23LC/23A/23LCV da Microchip, mas vale a dupla checagem.
//
// Mapa de enderecos: o espaco continua sendo 1024 palavras de 16 bits
// (mesmos 10 bits de endereco que o resto do projeto ja usa), so que
// agora cada palavra vira 2 bytes CONSECUTIVOS dentro da SRAM externa:
//   endereco_byte_na_sram = {13'b0, addr, 1'b0}   (= addr * 2)
// Byte menos significativo da palavra e' escrito/lido PRIMEIRO (nesse
// endereco_byte), depois o mais significativo (endereco_byte + 1) --
// mesma convencao little-endian ja usada pela flash de boot.
//
// Interface (substitui o acesso direto ao array ram[]):
//   addr/wdata/we    -- iguais ao antigo mem_addr/mem_wdata/mem_we,
//                        devem ficar ESTAVEIS durante toda a
//                        transacao (do pulso de req ate o pulso de
//                        ready).
//   req              -- pulso de 1 ciclo pedindo uma nova transacao
//                        (leitura se we=0, escrita se we=1). So deve
//                        ser pulsado com o motor ocioso (ready anterior
//                        ja recebido, ou logo apos reset).
//   ready            -- pulso de 1 ciclo quando a transacao termina.
//                        Em leituras, rdata ja esta valido nesse ciclo
//                        (e continua valido ate a proxima transacao).
//
// Custo em ciclos de clk por transacao: 1 (cmd) + 3 (endereco) + 2
// (dado) = 6 bytes * 16 ciclos/byte = 96 ciclos, mais alguns ciclos de
// sequenciamento -- ou seja, MUITO mais lento que a RAM interna
// original (que era 1 ciclo). Isso e' esperado ao trocar RAM interna
// por SRAM serial externa; cpu.v foi ajustado para esperar por "ready"
// em vez de assumir 1 ciclo fixo.
module ext_ram_ctrl (
    input  wire clk,
    input  wire rst,

    input  wire [9:0]  addr,
    input  wire [15:0] wdata,
    output reg  [15:0] rdata,
    input  wire        we,
    input  wire        req,
    output reg         ready,

    output reg         sram_cs,    // ativo em 0
    output wire        sram_sck,
    output wire        sram_mosi,
    input  wire        sram_miso
);
    localparam CMD_READ  = 8'h03;
    localparam CMD_WRITE = 8'h02;

    localparam S_IDLE      = 0,
               S_CMD_W     = 2,
               S_ADDR0     = 3,
               S_ADDR0_W   = 4,
               S_ADDR1     = 5,
               S_ADDR1_W   = 6,
               S_ADDR2     = 7,
               S_ADDR2_W   = 8,
               S_WDATA_LO  = 9,
               S_WDATA_LO_W= 10,
               S_WDATA_HI  = 11,
               S_WDATA_HI_W= 12,
               S_RDATA_LO  = 13,
               S_RDATA_LO_W= 14,
               S_RDATA_HI  = 15,
               S_RDATA_HI_W= 16;

    reg [4:0]  state;
    reg        we_latched;
    reg [23:0] byte_addr;      // = addr*2, zero-extendido pra 24 bits
    reg [15:0] wdata_latched;
    reg [7:0]  rlow;

    reg        eng_start;
    reg [7:0]  eng_tx;
    wire [7:0] eng_rx;
    wire       eng_done;

    spi_byte_engine u_eng (
        .clk(clk), .rst(rst),
        .start(eng_start), .tx_byte(eng_tx), .rx_byte(eng_rx),
        .busy(), .done(eng_done),
        .sck(sram_sck), .mosi(sram_mosi), .miso(sram_miso)
    );

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE; sram_cs <= 1; eng_start <= 0; ready <= 0;
        end else begin
            eng_start <= 0;
            ready     <= 0;
            case (state)
                S_IDLE: if (req) begin
                    we_latched    <= we;
                    byte_addr     <= {13'b0, addr, 1'b0};
                    wdata_latched <= wdata;
                    sram_cs       <= 0;
                    eng_tx        <= we ? CMD_WRITE : CMD_READ;
                    eng_start     <= 1;
                    state         <= S_CMD_W;
                end
                S_CMD_W: if (eng_done) state <= S_ADDR0;

                S_ADDR0:   begin eng_tx <= byte_addr[23:16]; eng_start <= 1; state <= S_ADDR0_W; end
                S_ADDR0_W: if (eng_done) state <= S_ADDR1;
                S_ADDR1:   begin eng_tx <= byte_addr[15:8];  eng_start <= 1; state <= S_ADDR1_W; end
                S_ADDR1_W: if (eng_done) state <= S_ADDR2;
                S_ADDR2:   begin eng_tx <= byte_addr[7:0];   eng_start <= 1; state <= S_ADDR2_W; end
                S_ADDR2_W: if (eng_done) state <= (we_latched ? S_WDATA_LO : S_RDATA_LO);

                // ---- escrita: byte baixo primeiro, depois o alto (little-endian) ----
                S_WDATA_LO:   begin eng_tx <= wdata_latched[7:0];  eng_start <= 1; state <= S_WDATA_LO_W; end
                S_WDATA_LO_W: if (eng_done) state <= S_WDATA_HI;
                S_WDATA_HI:   begin eng_tx <= wdata_latched[15:8]; eng_start <= 1; state <= S_WDATA_HI_W; end
                S_WDATA_HI_W: if (eng_done) begin
                    sram_cs <= 1; ready <= 1; state <= S_IDLE;
                end

                // ---- leitura: byte baixo primeiro, depois o alto ----
                S_RDATA_LO:   begin eng_tx <= 8'h00; eng_start <= 1; state <= S_RDATA_LO_W; end
                S_RDATA_LO_W: if (eng_done) begin rlow <= eng_rx; state <= S_RDATA_HI; end
                S_RDATA_HI:   begin eng_tx <= 8'h00; eng_start <= 1; state <= S_RDATA_HI_W; end
                S_RDATA_HI_W: if (eng_done) begin
                    rdata   <= {eng_rx, rlow};
                    sram_cs <= 1; ready <= 1; state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
