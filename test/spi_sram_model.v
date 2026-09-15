// spi_sram_model.v — modelo comportamental de testbench (NAO sintetizavel).
// Emula a SRAM serial 23LCV04M do jeito que ext_ram_ctrl.v fala com ela:
// comando (1 byte: 0x03=READ, 0x02=WRITE), 3 bytes de endereco (MSB
// primeiro), depois os bytes de dado (lidos ou escritos, MSB primeiro
// tambem dentro de cada byte, auto-incrementando o endereco a cada
// byte). No projeto real cada transacao so tem 2 bytes de dado (uma
// palavra de 16 bits), mas o modelo aceita qualquer quantidade
// enquanto o CS ficar em 0.
module spi_sram_model #(
    parameter DEPTH = 4096   // bytes disponiveis na SRAM simulada
)(
    input  wire cs,     // ativo em 0
    input  wire sck,
    input  wire mosi,
    output reg  miso
);
    reg [7:0] mem [0:DEPTH-1];
    integer i;
    initial for (i = 0; i < DEPTH; i = i + 1) mem[i] = 8'h00;

    localparam CMD_READ  = 8'h03;
    localparam CMD_WRITE = 8'h02;

    reg [2:0]  bitpos;       // proximo bit a amostrar/gerar dentro do byte atual (7..0)
    reg [23:0] byte_index;   // 0=cmd, 1..3=endereco, 4+ = dados
    reg [7:0]  shift_in;     // acumula os bits que estao chegando por mosi
    reg [7:0]  cmd_reg;
    reg [23:0] addr_hdr;     // vai montando o endereco de 24 bits enquanto chegam os 3 bytes
    reg [23:0] cur_addr;     // endereco corrente na fase de dados (incrementa a cada byte)
    reg        is_write;
    reg [7:0]  cur_out;

    always @(negedge cs) begin
        bitpos     <= 7;
        byte_index <= 0;
        shift_in   <= 8'h00;
        miso       <= 1'b0;
    end

    always @(posedge sck) begin : sample
        reg [7:0] completed_byte;
        shift_in <= {shift_in[6:0], mosi};
        completed_byte = {shift_in[6:0], mosi}; // byte cheio quando bitpos chega a 0

        if (bitpos == 0) begin
            case (byte_index)
                24'd0: cmd_reg <= completed_byte;
                24'd1: addr_hdr[23:16] <= completed_byte;
                24'd2: addr_hdr[15:8]  <= completed_byte;
                24'd3: begin
                    cur_addr <= {addr_hdr[23:8], completed_byte};
                    is_write <= (cmd_reg == CMD_WRITE);
                    if (cmd_reg == CMD_READ) begin
                        cur_out <= mem[{addr_hdr[23:8], completed_byte}];
                        miso    <= mem[{addr_hdr[23:8], completed_byte}][7];
                    end
                end
                default: begin // fase de dados (byte_index >= 4)
                    if (is_write) mem[cur_addr] <= completed_byte;
                    cur_addr <= cur_addr + 1;
                    if (!is_write) begin
                        cur_out <= mem[cur_addr + 1];
                        miso    <= mem[cur_addr + 1][7];
                    end
                end
            endcase
            byte_index <= byte_index + 1;
            bitpos     <= 7;
        end else begin
            bitpos <= bitpos - 1;
            if (byte_index >= 4 && !is_write) miso <= cur_out[bitpos - 1];
        end
    end
endmodule
