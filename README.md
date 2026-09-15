# mcu\_r16a

#### 16-bit custom microcontroller — Tiny Tapeout submission

CPU de 16 bits com 1024 palavras de RAM em uma **SRAM SPI externa**
(Microchip 23LCV04M-I/P), boot automatico via **flash SPI externa**
(Winbond W25Q80), GPIO com PWM e UART para programacao/depuracao. Veja
`docs/info.md` para o datasheet completo (pinout, como funciona, como
testar).

## Hardware externo necessario

* **SRAM**: Microchip 23LCV04M-I/P (SPI, 4 Mbit) -- guarda o programa
e os dados do usuario (1024 palavras de 16 bits = 2KB usados de um
chip de 512KB).
* **Flash**: Winbond W25Q80 (SPI, modo 0) -- guarda a imagem do
programa que e' copiada pra SRAM no boot.

Os dois chips ficam em barramentos SPI fisicamente **separados** (cada
um com seu proprio CS/SCK/MOSI/MISO) -- ver `docs/info.md` para o
pinout exato.

## Rodando os testes (cocotb — o que o CI do Tiny Tapeout roda)

```
cd test
make
```

**IMPORTANTE:** apos a troca da RAM interna pela SRAM externa, o
testbench cocotb (`test/tb.v` / `test/test.py`) e o standalone
(`test/tb\\\_standalone.v`) ainda simulam so a flash -- eles precisam de
um modelo de SRAM SPI (23LCV04M, comandos READ=0x03/WRITE=0x02) para
voltarem a passar. Isso NAO foi feito aqui a pedido explicito de quem
encomendou a mudanca (para nao gastar tempo/tokens rodando simulacao);
fica como proximo passo antes de sintetizar/gravar a placa de verdade.

## Simulação standalone com waveform (gtkwave)

Um testbench autocontido (clock, reset e um modelo de flash SPI, tudo em
Verilog puro) fica em `test/tb\\\\\\\_standalone.v`, útil para depurar com gtkwave
sem depender do cocotb (precisa do modelo de SRAM mencionado acima
para refletir o comportamento atual):

```
iverilog -g2012 -o sim test/tb_standalone.v src/top.v src/cpu.v src/gpio.v src/uart.v src/boot_ctrl.v src/tt_um_rianfiscina.v src/ext_ram_ctrl.v src/spi_byte_engine.v test/spi_flash_model.v test/spi_sram_model.v src/flash_fetch.vvvp sim
gtkwave wave.vcd -g
yosys -p "read\_verilog src/cpu.v src/uart.v src/gpio.v src/top.v src/boot\_ctrl.v src/tt\_um\_rianfiscina.v src/ext\_ram\_ctrl.v src/spi\_byte\_engine.v; synth; write\_verilog synth\_out\_v3.v"
```

## Estrutura

* `src/` — RTL (CPU, controlador da SRAM externa, boot via SPI, GPIO, UART, wrapper `tt\\\\\\\_um\\\\\\\_\\\\\\\*`)

  * `ext\\\_ram\\\_ctrl.v` — mestre SPI que fala com a SRAM externa (23LCV04M)
  * `spi\\\_byte\\\_engine.v` — motor SPI generico (1 byte por vez), usado por `boot\\\_ctrl.v` e `ext\\\_ram\\\_ctrl.v`
* `test/tb.v` — testbench mínimo, usado pelo cocotb (`test/test.py`)
* `test/tb\\\\\\\_standalone.v` — testbench autocontido, para debug local com gtkwave
* `docs/info.md` — datasheet do projeto (gerado pelo Tiny Tapeout)
* `info.yaml` — metadados do projeto (pinout, top module, etc.)
* `BOM.txt` — lista dos chips externos (SRAM e flash) usados neste projeto

