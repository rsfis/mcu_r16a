"""
Testbench cocotb para o sistema "Modo Macaco" (cpu + boot_ctrl + uart + gpio).

Peca central: SEM um modelo de flash SPI respondendo a boot_ctrl, o sinal
cpu_rst nunca vai a 0 e a CPU fica presa em reset para sempre. Por isso o
primeiro teste sobe um "flash" fake em background antes de liberar o rst_n.

NOVO (versao com SRAM externa): a RAM tambem deixou de ser interna -- agora
e' a SRAM SPI externa (Microchip 23LCV04M), acessada por ext_ram_ctrl.v em
TODA busca de instrucao e TODO acesso a dado. Sem um modelo dela respondendo
em ui_in[UI_SRAM_MISO], a CPU le lixo (sempre 0, ja que ui_in nao-dirigido
fica em 0 no cocotb) em vez do programa de verdade -- e' por isso que,
antes desta mudanca, o boot ate' terminava mas a CPU nao executava nada de
util (tudo virava opcode 0x00 = NOP). Por isso "iniciar_e_bootar" agora
tambem sobe um "spi_sram_model" em background, igual ja fazia com a flash.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, ClockCycles, Timer

UO_PROG_TX = 0
UO_SPI_CS = 1
UO_SPI_SCK = 2
UO_SPI_MOSI = 3
UO_SRAM_CS = 4
UO_SRAM_SCK = 5
UO_SRAM_MOSI = 6
UI_PROG_RX = 0
UI_SPI_MISO = 1
UI_SRAM_MISO = 2


def valor_seguro(sig):
    """Converte um vetor pra inteiro tratando X/Z como 0 (evita excecao
    de conversao nos primeiros ciclos, antes do reset propagar por tudo)."""
    s = str(sig.value).replace("x", "0").replace("z", "0")
    return int(s, 2)


def montar_imagem_flash(palavras):
    """Recebe uma lista de palavras de 16 bits e monta o payload da flash:
    2 bytes de tamanho (little-endian, em PALAVRAS) + palavras little-endian.
    Isso é só o que vem DEPOIS do comando 0x03 + 3 bytes de endereço,
    que o mestre (boot_ctrl) manda e o modelo de flash ignora.
    """
    n = len(palavras)
    imagem = [n & 0xFF, (n >> 8) & 0xFF]
    for w in palavras:
        imagem.append(w & 0xFF)
        imagem.append((w >> 8) & 0xFF)
    return imagem


def montar_programa():
    """Programa mínimo pra 'Modo Macaco':
        LOADI R0, 0x00FF      ; mascara: todos os 8 GPIOs como saida
        STORE [0x3F0], R0     ; GPIO_DIR = R0
        LOADI R1, 0x00AB      ; valor de teste
        OUT R1                ; GPIO_DATA = R1  (opcode 0x07 grava direto no endereco)
        HALT
    Formato de instrucao: 2 palavras -> {opcode[7:0],reg_dst[3:0],reg_src[3:0]}, operando
    """
    def instr(opcode, reg_dst, reg_src, operando):
        word0 = (opcode << 8) | (reg_dst << 4) | reg_src
        return [word0, operando & 0xFFFF]

    prog = []
    prog += instr(0x12, 0, 0, 0x00FF)   # LOADI R0, 0x00FF
    prog += instr(0x02, 0, 0, 0x03F0)   # STORE [0x3F0], R0
    prog += instr(0x12, 1, 0, 0x00AB)   # LOADI R1, 0x00AB
    prog += instr(0x07, 0, 1, 0x0000)   # OUT R1
    prog += instr(0x0F, 0, 0, 0x0000)   # HALT
    return prog


async def spi_flash_model(dut, imagem_bytes):
    """Modelo de flash SPI modo 0 (CPOL=0, CPHA=0), MSB primeiro.
    Ignora o que o mestre manda (comando/endereco) e so entrega os bytes
    de 'imagem_bytes' em sequencia, um por transacao com CS ativo (baixo).

    Importante: em vez de fazer polling nos pinos externos (uo_out) a cada
    RisingEdge(dut.clk), usamos RisingEdge/FallingEdge diretamente no sinal
    interno spi_sck do boot_ctrl. Isso elimina qualquer ambiguidade de
    "em que meio-ciclo estou" e evita erros de off-by-one no bit amostrado
    (o polling externo, mesmo com cuidado, pode escorregar 1 bit dependendo
    de como o RTL organiza o registrador de half-cycle internamente).
    Essa e' uma tecnica legitima de whitebox testbench: acoplamos o modelo
    ao nome do sinal interno, o que e' aceitavel aqui porque o objetivo e'
    testar a CPU/GPIO/UART, nao redescobrir o protocolo SPI por fora.
    """
    CABECALHO_IGNORADO = 4  # 1 byte de comando (0x03) + 3 bytes de endereco
    boot = dut.user_project.u_top.u_boot

    dut.ui_in[UI_SPI_MISO].value = 0
    bit_idx = 0
    byte_idx = 0

    while True:
        await FallingEdge(boot.spi_sck)

        if boot.spi_cs.value == 1:
            # CS inativo: fim (ou ainda nao inicio) de transacao
            bit_idx = 0
            byte_idx = 0
            continue

        # o bit "bit_idx" (0=MSB) acabou de ser amostrado pelo mestre;
        # avancamos o contador e deixamos pronto o PROXIMO bit
        bit_idx += 1
        if bit_idx == 8:
            bit_idx = 0
            byte_idx += 1

        idx_resposta = byte_idx - CABECALHO_IGNORADO
        if 0 <= idx_resposta < len(imagem_bytes):
            byte_atual = imagem_bytes[idx_resposta]
        else:
            byte_atual = 0x00  # durante cmd/addr, ou depois do fim da imagem

        proximo_bit = (byte_atual >> (7 - bit_idx)) & 1
        dut.ui_in[UI_SPI_MISO].value = proximo_bit


async def spi_sram_model(dut):
    """Modelo de SRAM SPI (Microchip 23LCV04M-like), modo 0 (CPOL=0, CPHA=0),
    MSB primeiro. Implementa READ (0x03) e WRITE (0x02) de verdade, com uma
    memoria interna de 2048 bytes (cobre as 1024 palavras de 16 bits do
    projeto -- so os 11 bits baixos do endereco de 3 bytes sao usados).

    Full-duplex: amostra MOSI na borda de SUBIDA do sck (mesmo instante em
    que o mestre -- spi_byte_engine.v -- amostra o MISO que devolvemos),
    e atualiza o proximo bit de MISO na borda de DESCIDA (setup antes da
    proxima subida), espelhando exatamente a temporizacao do motor SPI
    real usado por ext_ram_ctrl.v/boot_ctrl.v.
    """
    ram_bus = dut.user_project.u_top  # sram_cs/sram_sck/sram_mosi sao portas de top.v
    mem = bytearray(2048)
    st = {"bit": 0, "byte": 0, "cmd": 0, "addr": 0, "shift": 0, "out": 0}

    async def watch_cs():
        while True:
            await FallingEdge(ram_bus.sram_cs)
            st["bit"] = 0
            st["byte"] = 0
            st["out"] = 0

    cocotb.start_soon(watch_cs())
    dut.ui_in[UI_SRAM_MISO].value = 0

    while True:
        await RisingEdge(ram_bus.sram_sck)
        if ram_bus.sram_cs.value == 1:
            continue
        bit = int(ram_bus.sram_mosi.value)
        st["shift"] = ((st["shift"] << 1) | bit) & 0xFF
        st["bit"] += 1
        if st["bit"] == 8:
            st["bit"] = 0
            b, v = st["byte"], st["shift"]
            if b == 0:
                st["cmd"] = v
            elif b == 1:
                st["addr"] = (st["addr"] & 0x00FFFF) | (v << 16)
            elif b == 2:
                st["addr"] = (st["addr"] & 0xFF00FF) | (v << 8)
            elif b == 3:
                st["addr"] = (st["addr"] & 0xFFFF00) | v
                if st["cmd"] == 0x03:
                    st["out"] = mem[st["addr"] & 0x7FF]
            else:
                i = b - 4
                if st["cmd"] == 0x02:
                    mem[(st["addr"] + i) & 0x7FF] = v
                elif st["cmd"] == 0x03:
                    st["out"] = mem[(st["addr"] + i) & 0x7FF]
            st["byte"] += 1

        await FallingEdge(ram_bus.sram_sck)
        if ram_bus.sram_cs.value == 1:
            continue
        next_bit = (st["out"] >> (7 - st["bit"])) & 1
        dut.ui_in[UI_SRAM_MISO].value = next_bit


async def iniciar_e_bootar(dut, imagem_bytes, ciclos_pos_boot=8000):
    """Sobe clock, reset, os modelos de flash e SRAM, e espera o boot terminar.

    ciclos_pos_boot subiu bastante em relacao a versao com RAM interna
    (era 700): agora TODO acesso a memoria e' uma transacao SPI completa
    (~96+ ciclos: 1 cmd + 3 endereco + 2 dados, 16 ciclos/byte), entao boot
    + poucas instrucoes ja soma varios milhares de ciclos. 8000 e' uma
    estimativa com folga, nao um numero medido em simulacao -- ajuste se
    o seu programa for maior ou se o teste ainda cortar cedo demais.
    """
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.ui_in[UI_PROG_RX].value = 1  # UART ociosa em nivel alto
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await Timer(1, unit="ns")  # deixa os regs iniciais assentarem antes dos modelos lerem

    cocotb.start_soon(spi_flash_model(dut, imagem_bytes))
    cocotb.start_soon(spi_sram_model(dut))

    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    # tempo suficiente pro boot (agora envolvendo escritas na SRAM externa
    # tambem) + a CPU rodar o programa
    await ClockCycles(dut.clk, ciclos_pos_boot)


@cocotb.test()
async def test_boot_e_gpio_out(dut):
    """Verifica que a flash carrega o programa e a CPU escreve 0xAB no GPIO."""
    imagem = montar_imagem_flash(montar_programa())
    await iniciar_e_bootar(dut, imagem)

    valor = valor_seguro(dut.uio_out)
    direcao = valor_seguro(dut.uio_oe)

    dut._log.info(f"uio_out = 0x{valor:02X}, uio_oe = 0x{direcao:02X}")

    assert direcao == 0xFF, f"GPIO_DIR deveria ser 0xFF (todos saida), veio 0x{direcao:02X}"
    assert valor == 0xAB, f"GPIO_DATA deveria ser 0xAB, veio 0x{valor:02X}"


@cocotb.test()
async def test_reset_mantem_cpu_parada(dut):
    """Enquanto rst_n=0, a CPU nao deve produzir nenhuma saida em uo_out/uio_out."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0

    await ClockCycles(dut.clk, 20)

    assert valor_seguro(dut.uio_oe) == 0x00, "Em reset, nenhum pino deveria estar configurado como saida"