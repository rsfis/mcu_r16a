// gpio.v — v2
//
// Mudancas em relacao a Parte 4 do tutorial original:
//   - NPINS default 12 -> 8: o Tiny Tapeout so oferece 8 pinos
//     *de verdade* bidirecionais (uio[7:0] com uio_oe individual).
//     Os outros 4 GPIOs do projeto original ficariam com direcao fixa
//     em hardware (nao configuravel via GPIO_DIR em tempo real), o que
//     e' uma pegadinha silenciosa -- por isso a decisao foi reduzir pra
//     8, todos genuinamente bidirecionais.
//   - Removido o "inout" / alta impedancia (1'bz) interno. Nao e'
//     recomendado inferir tri-state dentro da logica de um ASIC no
//     fluxo do Tiny Tapeout -- em vez disso, exportamos 3 sinais downstream
//     (pad_in, pad_out, pad_oe) que casam direto com o padrao
//     ui_in/uo_out/uio_in/uio_out/uio_oe do wrapper tt_um_*.
//   - Corrigido um bug real: a leitura de GPIO_DATA (endereco 0x1F1)
//     antes sempre devolvia o ultimo valor ESCRITO (data_reg), nunca o
//     valor de verdade lido do pino quando ele estava em modo entrada.
//     Ou seja, um software rodando "LOAD Rd, GPIO_DATA" pra ler um botao
//     externo nunca teria funcionado. Agora, pra cada bit em modo
//     entrada (dir=0), a leitura reflete pad_in; pra bits em modo saida
//     (dir=1), reflete o que foi escrito (data_reg), como antes.
module gpio #(
    parameter NPINS = 8
)(
    input  wire clk,
    input  wire rst,
    input  wire [8:0]  addr,      // 9 bits (RAM de 512 palavras)
    input  wire [15:0] wdata,
    input  wire we,
    output reg  [15:0] rdata,
    input  wire [NPINS-1:0] pad_in,   // valor lido do pino fisico (quando em modo entrada)
    output wire [NPINS-1:0] pad_out,  // valor a dirigir no pino (quando em modo saida)
    output wire [NPINS-1:0] pad_oe    // 1 = pino em modo saida (direction/output-enable)
);
    localparam ADDR_DIR    = 9'h1F0;
    localparam ADDR_DATA   = 9'h1F1;
    // 0x1F2 livre (pull-up removido -- use resistor externo na placa)
    localparam ADDR_PWM_EN = 9'h1F3;
    localparam ADDR_DUTY0  = 9'h1F4; // 0x1F4..0x1FB = 8 enderecos, 1 por pino

    reg [NPINS-1:0] dir_reg, data_reg, pwm_en_reg;
    reg [7:0] duty [0:NPINS-1];
    reg [7:0] pwm_counter;

    wire is_duty_addr   = (addr >= ADDR_DUTY0) && (addr < ADDR_DUTY0 + NPINS);
    wire [3:0] duty_idx = addr - ADDR_DUTY0;

    integer i;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            dir_reg <= 0; data_reg <= 0; pwm_en_reg <= 0;
            pwm_counter <= 0;
            for (i = 0; i < NPINS; i = i + 1) duty[i] <= 0;
        end else begin
            pwm_counter <= pwm_counter + 1; // conta 0..255 e repete -> "periodo" do PWM

            if (we) begin
                case (addr)
                    ADDR_DIR:    dir_reg    <= wdata[NPINS-1:0];
                    ADDR_DATA:   data_reg   <= wdata[NPINS-1:0];
                    ADDR_PWM_EN: pwm_en_reg <= wdata[NPINS-1:0];
                    default: if (is_duty_addr) duty[duty_idx] <= wdata[7:0];
                endcase
            end
        end
    end

    // Leitura de volta: GPIO_DATA agora mistura, por bit, o valor
    // escrito (se saida) com o valor externo de verdade (se entrada).
    wire [NPINS-1:0] data_readback;
    genvar r;
    generate
        for (r = 0; r < NPINS; r = r + 1) begin : READBACK
            assign data_readback[r] = dir_reg[r] ? data_reg[r] : pad_in[r];
        end
    endgenerate

    always @(*) begin
        case (addr)
            ADDR_DIR:    rdata = {7'b0, dir_reg};
            ADDR_DATA:   rdata = {7'b0, data_readback};
            ADDR_PWM_EN: rdata = {7'b0, pwm_en_reg};
            default:     rdata = is_duty_addr ? {8'b0, duty[duty_idx]} : 16'h0000;
        endcase
    end

    genvar g;
    generate
        for (g = 0; g < NPINS; g = g + 1) begin : PADS
            assign pad_out[g] = pwm_en_reg[g] ? (pwm_counter < duty[g]) : data_reg[g];
            assign pad_oe[g]  = dir_reg[g];
        end
    endgenerate
endmodule
