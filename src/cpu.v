// cpu.v — "Modo Macaco" v4
//
// Mudancas em relacao ao tutorial original (v3.1):
//   - Formato de instrucao comprimido: 2 palavras (opcode+regs, operando)
//     em vez de 8. As 6 palavras reservadas do formato antigo nunca eram
//     usadas, entao isso so corta desperdicio -- nao muda nenhuma
//     instrucao existente de lugar nem de significado.
//   - RAM alvo agora e' 512 palavras (9 bits de PC/endereco) em vez de
//     1024 -- decisao de area para caber num tile do Tiny Tapeout
//     (RAM aqui e' feita de flip-flops, nao de uma macro de SRAM densa).
//   - Quatro opcodes que na v3.1 so mexiam no barramento de endereco mas
//     nunca capturavam o resultado de volta no registrador foram
//     corrigidos: LOAD (0x01), OUT (0x07), IN (0x08), LOADR (0x13).
//     Todos os quatro agora usam o mesmo estado MEM_WAIT (1 ciclo extra
//     de latencia, leitura sincrona padrao).
//   - STORE (0x02) e STORER (0x14) ja estavam corretos na v3.1 e nao
//     mudaram de logica, so de largura de barramento (10->9 bits).
module cpu #(
    parameter CLK_FREQ_HZ = 100_000_000  // chute inicial -- ajuste depois da STA real (OpenLane/LibreLane)
)(
    input  wire clk,
    input  wire rst,
    output reg  [8:0]  mem_addr,   // 9 bits = 512 palavras
    output reg  [15:0] mem_wdata,
    input  wire [15:0] mem_rdata,
    output reg  mem_we,
    output reg  halted,
    output reg  flag_z
);
    localparam CYCLES_PER_MS = CLK_FREQ_HZ / 1000;

    // Endereco de GPIO_DATA no novo mapa de memoria (ver top.v) --
    // usado internamente pelos opcodes OUT/IN pra nao precisar de um
    // barramento separado so pra GPIO.
    localparam ADDR_GPIO_DATA = 9'h1F1;

    reg [15:0] regfile [0:14];   // R0=A ... R14=RS (15 registradores)
    reg [8:0]  pc;                 // avanca de 2 em 2 (1 instrucao = 2 palavras)
    reg [3:0]  state;

    localparam FETCH_OP=0, FETCH_OPERAND=1, EXEC=2, DELAY_WAIT=3, MEM_WAIT=4;

    reg [7:0]  opcode;
    reg [3:0]  reg_dst, reg_src;
    reg [31:0] delay_cnt;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pc <= 0; state <= FETCH_OP; halted <= 0;
            mem_we <= 0; flag_z <= 0; delay_cnt <= 0;
        end else if (!halted) begin
            case (state)
                FETCH_OP: begin
                    mem_addr <= pc;
                    mem_we   <= 0;
                    state    <= FETCH_OPERAND;
                end
                FETCH_OPERAND: begin
                    opcode   <= mem_rdata[15:8];
                    reg_dst  <= mem_rdata[7:4];
                    reg_src  <= mem_rdata[3:0];
                    mem_addr <= pc + 1;
                    state    <= EXEC;
                end
                EXEC: begin
                    // mem_rdata aqui = palavra do operando (end/imm), ja
                    // que mem_addr foi ajustado pra pc+1 no estado anterior.
                    case (opcode)
                        8'h00: pc <= pc + 2; // NOP
                        8'h01: begin mem_addr <= mem_rdata; state <= MEM_WAIT; end // LOAD (corrigido: deref de verdade)
                        8'h02: begin mem_we<=1; mem_wdata<=regfile[reg_src];
                                     mem_addr<=mem_rdata; pc<=pc+2; end // STORE
                        8'h03: begin regfile[reg_dst] <= regfile[reg_dst]+regfile[reg_src]; pc<=pc+2; end // ADD
                        8'h04: begin regfile[reg_dst] <= regfile[reg_dst]-regfile[reg_src]; pc<=pc+2; end // SUB
                        8'h05: pc <= mem_rdata; // JMP
                        8'h06: pc <= (regfile[reg_src]==0) ? mem_rdata : pc+2; // JZ
                        8'h07: begin mem_we<=1; mem_wdata<=regfile[reg_src];
                                     mem_addr<=ADDR_GPIO_DATA; pc<=pc+2; end // OUT (corrigido: agora escreve de verdade)
                        8'h08: begin mem_addr<=ADDR_GPIO_DATA; state<=MEM_WAIT; end // IN (corrigido: agora le de verdade)
                        8'h09: begin regfile[reg_dst] <= regfile[reg_dst]*regfile[reg_src]; pc<=pc+2; end // MUL
                        8'h0A: begin regfile[reg_dst] <= (regfile[reg_src]==0)?16'hFFFF:regfile[reg_dst]/regfile[reg_src]; pc<=pc+2; end // DIV
                        8'h0B: begin regfile[reg_dst] <= (regfile[reg_src]==0)?16'h0000:regfile[reg_dst]%regfile[reg_src]; pc<=pc+2; end // MOD
                        8'h0C: begin regfile[reg_dst] <= regfile[reg_dst]|regfile[reg_src]; pc<=pc+2; end // OR
                        8'h0D: begin flag_z <= (regfile[reg_dst]==regfile[reg_src]); pc<=pc+2; end // CMP
                        8'h0E: begin regfile[reg_dst] <= ~regfile[reg_dst]; pc<=pc+2; end // NOT
                        8'h0F: halted <= 1; // HALT
                        8'h10: begin delay_cnt <= mem_rdata * CYCLES_PER_MS; state<=DELAY_WAIT; end // DELAY
                        8'h11: begin regfile[reg_dst] <= regfile[reg_src]; pc<=pc+2; end // MOV
                        8'h12: begin regfile[reg_dst] <= mem_rdata; pc<=pc+2; end // LOADI
                        8'h13: begin mem_addr <= regfile[reg_src]; state<=MEM_WAIT; end // LOADR (corrigido: agora captura o resultado)
                        8'h14: begin mem_we<=1; mem_wdata<=regfile[reg_dst];
                                     mem_addr<=regfile[reg_src]; pc<=pc+2; end // STORER
                        default: pc <= pc + 2;
                    endcase
                    // Opcodes que precisam de mais um ciclo antes de voltar
                    // a buscar a proxima instrucao: LOAD/IN/LOADR (vao pro
                    // MEM_WAIT) e DELAY (vai pro DELAY_WAIT). Os demais
                    // completam nesse mesmo ciclo e voltam direto pro fetch.
                    if (!(opcode==8'h10 || opcode==8'h01 || opcode==8'h08 || opcode==8'h13))
                        state <= FETCH_OP;
                end
                MEM_WAIT: begin
                    // mem_rdata agora reflete a leitura sincrona do
                    // endereco que setamos no EXEC (LOAD: memoria[end];
                    // IN: GPIO_DATA; LOADR: memoria[regfile[reg_src]]).
                    regfile[reg_dst] <= mem_rdata;
                    pc    <= pc + 2;
                    state <= FETCH_OP;
                end
                DELAY_WAIT: begin
                    if (delay_cnt==0) begin pc<=pc+2; state<=FETCH_OP; end
                    else delay_cnt <= delay_cnt - 1;
                end
            endcase
        end
    end
endmodule
