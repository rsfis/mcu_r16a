// cpu.v — v6 (XIP: busca de instrucao direto da flash, dados na SRAM)
//
// NOVO nesta revisao: a busca de instrucao (FETCH_OP/FETCH_OPERAND, ou
// seja, os fetches indexados por 'pc') deixou de usar o barramento
// mem_*/SRAM e passou a usar um barramento PROPRIO -- fetch_addr/
// fetch_req/fetch_rdata/fetch_ready -- que em top.v vai direto pro
// modulo flash_fetch (le a flash de boot via SPI, sem passar pela
// SRAM). O programa nao e' mais copiado pra SRAM no boot (boot_ctrl.v
// nao existe mais); ele roda direto da flash (execute-in-place). Isso
// libera A SRAM INTEIRA (antes ocupada pelo programa) so' para dados
// do usuario (variaveis, pilha de CALL/RET/PUSH/POP). LOAD/STORE/
// LOADR/STORER/IN/OUT/CALL/RET/PUSH/POP continuam usando mem_*
// (endereco de DADO) exatamente como antes, sem nenhuma mudanca --
// so' o que mudou foi de onde vem o OPCODE/OPERANDO da instrucao.
//
// cpu.v — v5 (adaptado para RAM externa via SPI)
//
// MUDANCA IMPORTANTE em relacao a versao anterior: a RAM deixou de
// ser interna (1 ciclo de latencia fixo, leitura combinacional) e
// passou a ser a SRAM serial externa 23LCV04M (ver ext_ram_ctrl.v),
// que leva dezenas de ciclos por acesso via SPI. Por isso, TODO
// acesso a memoria (busca de opcode, busca de operando, LOAD, STORE,
// IN, OUT, LOADR, STORER) agora usa um handshake explicito:
//
//   - a CPU coloca mem_addr/mem_we/mem_wdata estaveis e pulsa
//     mem_req por 1 ciclo;
//   - a CPU fica parada ate ver mem_ready pulsar (1 ciclo) -- so
//     entao mem_rdata (em leituras) e' considerado valido e a CPU
//     segue em frente.
//
// mem_addr/mem_wdata/mem_we tem que permanecer estaveis do pulso de
// mem_req ate o pulso de mem_ready (quem atende o pedido -- top.v /
// ext_ram_ctrl / gpio / uart -- depende disso).
//
// A logica de cada opcode (o que ele calcula, quais registradores
// usa) NAO mudou; so a mecanica de buscar/gravar memoria foi
// generalizada para esperar o handshake em vez de assumir 1 ciclo.
//
// NOVO nesta revisao: AND (0x15, bitwise, mesmo layout de OR) e o par
// CALL/RET (0x16/0x17) pra sub-rotinas. CALL empilha o endereco de
// retorno em mem[R14] (R14=SP) e RET desempilha; como cada push/pop
// tambem e' um acesso a SRAM externa via SPI, ambos passam pelo mesmo
// handshake mem_req/mem_ready generico -- so' que, ao contrario de
// LOAD/STORE, o que fazer QUANDO mem_ready chegar depende do opcode
// (mexer no PC e no SP), entao MEM_REQ_WAIT agora usa 'pending_op'
// pra saber qual dos tres comportamentos aplicar.
//
// NOVO nesta revisao (2): JNZ (0x18, complemento do JZ), ADDI/SUBI
// (0x19/0x1A, imediato direto sem precisar de LOADI+ADD/SUB antes),
// INC/DEC (0x1B/0x1C, atalho de ADDI/SUBI com passo 1), PUSH/POP
// (0x1D/0x1E, empilhar/desempilhar um registrador avulso usando o
// mesmo SP=R14 do CALL/RET, sem mexer no pc), XOR (0x1F, fecha o trio
// AND/OR/XOR) e CMPI (0x20, compara direto com imediato). PUSH/POP
// reusam a mesma mecanica de acesso a memoria de CALL/RET (por isso
// 'pending_op' ganhou PEND_PUSH/PEND_POP).
//
// NOVO nesta revisao (3): esquema de comparacao com flags "de verdade",
// no estilo classico (CMP calcula uma vez, os jumps condicionais
// testam o resultado depois) em vez de um opcode por relacao. CMP e
// CMPI (0x0D/0x20, ja existiam pra "==") agora, alem de 'flag_z'
// (Rd==Rs / Rd==imm), tambem gravam 'flag_lt' (Rd<Rs / Rd<imm,
// comparacao SEM SINAL, ja que regfile e' 'reg [15:0]'). A partir
// dessas 2 flags da' pra testar as 6 relacoes (==, !=, <, >=, >, <=)
// com 6 jumps condicionais novos, sem precisar de opcode CMP separado
// pra cada relacao:
//   0x21 JE   addr   pc <= flag_z              ? addr : pc+2   (igual)
//   0x22 JNE  addr   pc <= !flag_z             ? addr : pc+2   (diferente)
//   0x23 JL   addr   pc <= flag_lt             ? addr : pc+2   (menor)
//   0x24 JNL  addr   pc <= !flag_lt            ? addr : pc+2   (nao-menor = >=)
//   0x25 JG   addr   pc <= (!flag_z && !flag_lt) ? addr : pc+2 (maior)
//   0x26 JNG  addr   pc <= (flag_z || flag_lt)  ? addr : pc+2  (nao-maior = <=)
// Uso tipico (comparacao + desvio condicional, igual JZ/JNZ mas
// usando o resultado de CMP em vez de testar um registrador direto):
//   CMP R0, R1
//   JNL pular        ; pula se R0 NAO for menor que R1 (R0>=R1)
//   CALL rotina_menor
//   pular:
// 'flag_z'/'flag_lt' nao sao zerados a cada instrucao -- so' sao
// atualizados por CMP/CMPI; qualquer JE/JNE/JL/JNL/JG/JNG executado
// sem um CMP/CMPI antes testa o resultado da ULTIMA comparacao feita.
module cpu #(
    parameter CLK_FREQ_HZ = 16_000_000  // chute inicial -- ajuste depois da STA real (OpenLane/LibreLane)
)(
    input  wire clk,
    input  wire rst,
    output reg  [9:0]  mem_addr,   // 10 bits = 1024 palavras (agora so' DADOS na SRAM)
    output reg  [15:0] mem_wdata,
    input  wire [15:0] mem_rdata,
    output reg  mem_we,
    output reg  mem_req,           // pulso de 1 ciclo = "endereco valido, comece a transacao"
    input  wire mem_ready,         // pulso de 1 ciclo = "transacao terminou"
    output reg  [9:0]  fetch_addr,  // NOVO (XIP): endereco de INSTRUCAO, direto na flash
    output reg  fetch_req,          // NOVO: pulso de 1 ciclo pedindo a palavra em fetch_addr
    input  wire [15:0] fetch_rdata, // NOVO
    input  wire fetch_ready,        // NOVO: pulso de 1 ciclo quando fetch_rdata fica valido
    output reg  halted,
    output reg  flag_z
);
    localparam CYCLES_PER_MS = CLK_FREQ_HZ / 1000;

    reg flag_lt;   // flag_lt <= (Rd<Rs)/(Rd<imm), gravada por CMP/CMPI junto com flag_z --
                    // nao e' porta do modulo (igual regfile), mas fica acessivel via
                    // caminho hierarquico (u_top.u_cpu.flag_lt) pra debug, como as demais.

    // Endereco de GPIO_DATA no mapa de memoria (ver top.v) -- usado
    // internamente pelos opcodes OUT/IN pra nao precisar de um
    // barramento separado so pra GPIO.
    localparam ADDR_GPIO_DATA = 10'h3F1;

    reg [15:0] regfile [0:14];   // R0=A ... R14=RS (15 registradores)
    // R14 (RS) agora tem um uso reservado: e' o SP (stack pointer) usado
    // por CALL/RET (opcodes 0x16/0x17) pra empilhar/desempilhar o
    // endereco de retorno. Igual aos outros registradores, o regfile NAO
    // e' zerado no reset -- o programa PRECISA inicializar R14 (ex:
    // "LOADI R14, 0x3EC", perto do topo do espaco de dados livre, antes
    // de qualquer CALL, senao a pilha comeca em lixo/0x0000 e o primeiro
    // push vai escrever fora de onde voce espera.
    reg [9:0]  pc;                 // avanca de 2 em 2 (1 instrucao = 2 palavras)
    reg [3:0]  state;

    localparam FETCH_OP        = 0,
               FETCH_OP_WAIT   = 1,
               FETCH_OPERAND   = 2,
               FETCH_OPERAND_WAIT = 3,
               EXEC            = 4,
               MEM_REQ_WAIT    = 5,   // esperando mem_ready de um LOAD/STORE/IN/OUT/LOADR/STORER
               DELAY_WAIT      = 6;

    reg [7:0]  opcode;
    reg [3:0]  reg_dst, reg_src;
    reg [15:0] operand;      // palavra de operando (endereco/imediato), capturada apos o fetch
    reg [31:0] delay_cnt;
    reg        pending_is_read;   // 1 = quando mem_ready chegar, capturar regfile[reg_dst] <= mem_rdata

    // O que fazer quando mem_ready chegar em MEM_REQ_WAIT -- generalizado
    // pra tambem cobrir CALL (push do endereco de retorno) e RET (pop do
    // endereco de retorno), que nao sao um simples LOAD/STORE/IN/OUT/
    // LOADR/STORER (mexem no PC e no SP=R14 depois do acesso).
    localparam PEND_NORMAL = 3'd0,  // LOAD/STORE/IN/OUT/LOADR/STORER (comportamento antigo)
               PEND_CALL   = 3'd1,  // push concluido -> SP--, pc <= operand (endereco de destino)
               PEND_RET    = 3'd2,  // pop concluido  -> SP++, pc <= mem_rdata (endereco de retorno)
               PEND_PUSH   = 3'd3,  // push concluido -> SP-- (sem mexer no pc)
               PEND_POP    = 3'd4;  // pop concluido  -> SP++, regfile[reg_dst] <= mem_rdata
    reg [2:0]  pending_op;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pc <= 0; state <= FETCH_OP; halted <= 0;
            mem_we <= 0; mem_req <= 0; fetch_req <= 0; flag_z <= 0; flag_lt <= 0; delay_cnt <= 0;
            pending_op <= PEND_NORMAL;
        end else if (!halted) begin
            mem_req   <= 0;   // pulso de 1 ciclo por padrao (states abaixo re-setam quando precisam)
            mem_we    <= 0;
            fetch_req <= 0;   // idem, para a busca de instrucao (agora na flash)
            case (state)
                // ---- busca da 1a palavra da instrucao (opcode + regs) -- na FLASH agora ----
                FETCH_OP: begin
                    fetch_addr <= pc;
                    fetch_req  <= 1;
                    state      <= FETCH_OP_WAIT;
                end
                FETCH_OP_WAIT: if (fetch_ready) begin
                    opcode  <= fetch_rdata[15:8];
                    reg_dst <= fetch_rdata[7:4];
                    reg_src <= fetch_rdata[3:0];
                    state   <= FETCH_OPERAND;
                end

                // ---- busca da 2a palavra da instrucao (operando: endereco/imediato) -- na FLASH ----
                FETCH_OPERAND: begin
                    fetch_addr <= pc + 1;
                    fetch_req  <= 1;
                    state      <= FETCH_OPERAND_WAIT;
                end
                FETCH_OPERAND_WAIT: if (fetch_ready) begin
                    operand <= fetch_rdata;
                    state   <= EXEC;
                end

                EXEC: begin
                    // `operand` aqui = palavra de operando (endereco/imediato) ja
                    // capturada em FETCH_OPERAND_WAIT.
                    pending_is_read <= 1'b0;   // default; opcodes de leitura sobrescrevem abaixo
                    pending_op      <= PEND_NORMAL; // default; CALL/RET sobrescrevem abaixo
                    case (opcode)
                        8'h00: begin pc <= pc + 2; state <= FETCH_OP; end // NOP
                        8'h01: begin // LOAD (deref de verdade)
                            mem_addr <= operand; mem_req <= 1;
                            pending_is_read <= 1'b1; state <= MEM_REQ_WAIT;
                        end
                        8'h02: begin // STORE
                            mem_we <= 1; mem_wdata <= regfile[reg_src];
                            mem_addr <= operand; mem_req <= 1;
                            state <= MEM_REQ_WAIT;
                        end
                        8'h03: begin regfile[reg_dst] <= regfile[reg_dst]+regfile[reg_src]; pc<=pc+2; state<=FETCH_OP; end // ADD
                        8'h04: begin regfile[reg_dst] <= regfile[reg_dst]-regfile[reg_src]; pc<=pc+2; state<=FETCH_OP; end // SUB
                        8'h05: begin pc <= operand; state <= FETCH_OP; end // JMP
                        8'h06: begin pc <= (regfile[reg_src]==0) ? operand : pc+2; state <= FETCH_OP; end // JZ
                        8'h07: begin // OUT (escreve de verdade em GPIO_DATA)
                            mem_we <= 1; mem_wdata <= regfile[reg_src];
                            mem_addr <= ADDR_GPIO_DATA; mem_req <= 1;
                            state <= MEM_REQ_WAIT;
                        end
                        8'h08: begin // IN (le de verdade GPIO_DATA)
                            mem_addr <= ADDR_GPIO_DATA; mem_req <= 1;
                            pending_is_read <= 1'b1; state <= MEM_REQ_WAIT;
                        end
                        8'h09: begin regfile[reg_dst] <= regfile[reg_dst]*regfile[reg_src]; pc<=pc+2; state<=FETCH_OP; end // MUL
                        8'h0A: begin regfile[reg_dst] <= (regfile[reg_src]==0)?16'hFFFF:regfile[reg_dst]/regfile[reg_src]; pc<=pc+2; state<=FETCH_OP; end // DIV
                        8'h0B: begin regfile[reg_dst] <= (regfile[reg_src]==0)?16'h0000:regfile[reg_dst]%regfile[reg_src]; pc<=pc+2; state<=FETCH_OP; end // MOD
                        8'h0C: begin regfile[reg_dst] <= regfile[reg_dst]|regfile[reg_src]; pc<=pc+2; state<=FETCH_OP; end // OR
                        8'h0D: begin flag_z <= (regfile[reg_dst]==regfile[reg_src]); flag_lt <= (regfile[reg_dst]<regfile[reg_src]); pc<=pc+2; state<=FETCH_OP; end // CMP
                        8'h0E: begin regfile[reg_dst] <= ~regfile[reg_dst]; pc<=pc+2; state<=FETCH_OP; end // NOT
                        8'h0F: halted <= 1; // HALT
                        8'h10: begin delay_cnt <= operand * CYCLES_PER_MS; state<=DELAY_WAIT; end // DELAY
                        8'h11: begin regfile[reg_dst] <= regfile[reg_src]; pc<=pc+2; state<=FETCH_OP; end // MOV
                        8'h12: begin regfile[reg_dst] <= operand; pc<=pc+2; state<=FETCH_OP; end // LOADI
                        8'h13: begin // LOADR
                            mem_addr <= regfile[reg_src]; mem_req <= 1;
                            pending_is_read <= 1'b1; state <= MEM_REQ_WAIT;
                        end
                        8'h14: begin // STORER
                            mem_we <= 1; mem_wdata <= regfile[reg_dst];
                            mem_addr <= regfile[reg_src]; mem_req <= 1;
                            state <= MEM_REQ_WAIT;
                        end
                        8'h15: begin regfile[reg_dst] <= regfile[reg_dst]&regfile[reg_src]; pc<=pc+2; state<=FETCH_OP; end // AND
                        8'h16: begin // CALL addr -- empilha o endereco de retorno (pc+2) em
                            // mem[SP] (SP=R14), depois desvia pra 'operand'. SP e' decrementado
                            // SO' depois que o mem_ready confirmar que a escrita terminou (a
                            // SRAM externa e' serial, entao isso leva varios ciclos) -- ver
                            // MEM_REQ_WAIT / PEND_CALL. Pilha "descendente com topo vazio":
                            // SP sempre aponta pro proximo slot livre (ver comentario no
                            // handshake de RET, mais abaixo).
                            mem_we <= 1; mem_wdata <= pc + 2;
                            mem_addr <= regfile[14]; mem_req <= 1;
                            pending_op <= PEND_CALL; state <= MEM_REQ_WAIT;
                        end
                        8'h17: begin // RET -- desempilha o endereco de retorno de mem[SP+1] e
                            // pula pra ele. Le de SP+1 (nao de SP) porque, na convencao usada
                            // por CALL (push escreve em mem[SP] e SO' DEPOIS faz SP--), o
                            // ultimo valor empilhado esta' em SP+1, nao em SP. SP e' incrementado
                            // (desfazendo o decremento do CALL) quando mem_ready confirmar a
                            // leitura -- ver MEM_REQ_WAIT / PEND_RET.
                            mem_addr <= regfile[14] + 1; mem_req <= 1;
                            pending_op <= PEND_RET; state <= MEM_REQ_WAIT;
                        end
                        8'h18: begin pc <= (regfile[reg_src]!=0) ? operand : pc+2; state <= FETCH_OP; end // JNZ
                        8'h19: begin regfile[reg_dst] <= regfile[reg_dst]+operand; pc<=pc+2; state<=FETCH_OP; end // ADDI
                        8'h1A: begin regfile[reg_dst] <= regfile[reg_dst]-operand; pc<=pc+2; state<=FETCH_OP; end // SUBI
                        8'h1B: begin regfile[reg_dst] <= regfile[reg_dst]+16'd1; pc<=pc+2; state<=FETCH_OP; end // INC
                        8'h1C: begin regfile[reg_dst] <= regfile[reg_dst]-16'd1; pc<=pc+2; state<=FETCH_OP; end // DEC
                        8'h1D: begin // PUSH Rs -- mesma mecanica de push do CALL, mas sem
                            // mexer no pc (so' empilha regfile[reg_src] e decrementa SP).
                            mem_we <= 1; mem_wdata <= regfile[reg_src];
                            mem_addr <= regfile[14]; mem_req <= 1;
                            pending_op <= PEND_PUSH; state <= MEM_REQ_WAIT;
                        end
                        8'h1E: begin // POP Rd -- mesma mecanica de pop do RET, mas guarda o
                            // valor lido em regfile[reg_dst] em vez de desviar o pc.
                            mem_addr <= regfile[14] + 1; mem_req <= 1;
                            pending_op <= PEND_POP; state <= MEM_REQ_WAIT;
                        end
                        8'h1F: begin regfile[reg_dst] <= regfile[reg_dst]^regfile[reg_src]; pc<=pc+2; state<=FETCH_OP; end // XOR
                        8'h20: begin flag_z <= (regfile[reg_dst]==operand); flag_lt <= (regfile[reg_dst]<operand); pc<=pc+2; state<=FETCH_OP; end // CMPI
                        8'h21: begin pc <= flag_z ? operand : pc+2; state <= FETCH_OP; end // JE  addr (pula se igual)
                        8'h22: begin pc <= !flag_z ? operand : pc+2; state <= FETCH_OP; end // JNE addr (pula se diferente)
                        8'h23: begin pc <= flag_lt ? operand : pc+2; state <= FETCH_OP; end // JL  addr (pula se menor)
                        8'h24: begin pc <= !flag_lt ? operand : pc+2; state <= FETCH_OP; end // JNL addr (pula se NAO menor, ou seja, >=)
                        8'h25: begin pc <= (!flag_z && !flag_lt) ? operand : pc+2; state <= FETCH_OP; end // JG  addr (pula se maior)
                        8'h26: begin pc <= (flag_z || flag_lt) ? operand : pc+2; state <= FETCH_OP; end // JNG addr (pula se NAO maior, ou seja, <=)
                        default: begin pc <= pc + 2; state <= FETCH_OP; end
                    endcase
                end

                // ---- espera generica pelo fim de um acesso a memoria (dado, nao instrucao) ----
                MEM_REQ_WAIT: if (mem_ready) begin
                    case (pending_op)
                        PEND_CALL: begin // push do CALL confirmado: agora sim desvia e decrementa SP
                            regfile[14] <= regfile[14] - 16'd1;
                            pc          <= operand;
                            state       <= FETCH_OP;
                        end
                        PEND_RET: begin // pop do RET confirmado: agora sim desvia e incrementa SP
                            regfile[14] <= regfile[14] + 16'd1;
                            pc          <= mem_rdata;
                            state       <= FETCH_OP;
                        end
                        PEND_PUSH: begin // push do PUSH confirmado: so' decrementa SP e segue pra proxima instrucao
                            regfile[14] <= regfile[14] - 16'd1;
                            pc          <= pc + 2;
                            state       <= FETCH_OP;
                        end
                        PEND_POP: begin // pop do POP confirmado: incrementa SP e grava o registrador destino
                            regfile[14]     <= regfile[14] + 16'd1;
                            regfile[reg_dst] <= mem_rdata;
                            pc              <= pc + 2;
                            state           <= FETCH_OP;
                        end
                        default: begin // LOAD/STORE/IN/OUT/LOADR/STORER (comportamento antigo)
                            if (pending_is_read) regfile[reg_dst] <= mem_rdata;
                            pc    <= pc + 2;
                            state <= FETCH_OP;
                        end
                    endcase
                end

                DELAY_WAIT: begin
                    if (delay_cnt==0) begin pc<=pc+2; state<=FETCH_OP; end
                    else delay_cnt <= delay_cnt - 1;
                end

                default: state <= FETCH_OP;
            endcase
        end
    end
endmodule
