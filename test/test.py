"""
Testbench cocotb para o sistema "Modo Macaco" (cpu + boot_ctrl + uart + gpio).

Peca central: SEM um modelo de flash SPI respondendo a boot_ctrl, o sinal
cpu_rst nunca vai a 0 e a CPU fica presa em reset para sempre. Por isso o
primeiro teste sobe um "flash" fake em background antes de liberar o rst_n.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, ClockCycles, Timer

UO_PROG_TX = 0
UO_SPI_CS = 1
UO_SPI_SCK = 2
UO_SPI_MOSI = 3
UI_PROG_RX = 0
UI_SPI_MISO = 1


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
        STORE [0x1F0], R0     ; GPIO_DIR = R0
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
    prog += instr(0x02, 0, 0, 0x01F0)   # STORE [0x1F0], R0
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


async def iniciar_e_bootar(dut, imagem_bytes, ciclos_pos_boot=700):
    """Sobe clock, reset, o modelo de flash, e espera o boot terminar."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.ui_in[UI_PROG_RX].value = 1  # UART ociosa em nivel alto
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await Timer(1, unit="ns")  # deixa os regs iniciais assentarem antes do modelo de flash ler

    cocotb.start_soon(spi_flash_model(dut, imagem_bytes))

    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    # tempo suficiente pro boot (26 bytes * 16 ciclos/byte) + a CPU rodar o programa
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