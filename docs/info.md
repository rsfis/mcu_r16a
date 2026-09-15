<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

Fiscion R16A is a small custom 16-bit microcontroller: a 16-bit CPU with a
register file, 1024 words of RAM held in an **external SPI SRAM**
(Microchip 23LCV04M-I/P), GPIO with PWM, and a UART for
programming/debugging.

On reset, `boot_ctrl` reads a program image from an external SPI flash
(Winbond W25Q80, mode 0, standard 0x03 READ command) and, word by word,
writes it into the external SRAM (command 0x02 WRITE) before releasing the
CPU from reset, so the chip boots straight into your program with no
external MCU or bootloader interaction required. Once booted, the CPU reads
and writes that same external SRAM directly (command 0x03 READ / 0x02
WRITE) for every instruction fetch and every data access -- there is no RAM
inside the chip anymore.

Because every memory access is now a full SPI transaction (command + 3
address bytes + data bytes) instead of a single clock cycle, the CPU is
much slower per instruction than the previous internal-RAM version. This is
the expected trade-off of moving RAM off-chip.

- `ui_in[0]`  = `prog_rx`   — UART RX for boot/debug
- `ui_in[1]`  = `spi_miso`  — SPI flash MISO
- `ui_in[2]`  = `sram_miso` — SPI SRAM MISO
- `uo_out[0]` = `prog_tx`   — UART TX
- `uo_out[1]` = `spi_cs`    — SPI flash chip-select (active low)
- `uo_out[2]` = `spi_sck`   — SPI flash clock
- `uo_out[3]` = `spi_mosi`  — SPI flash MOSI
- `uo_out[4]` = `sram_cs`   — SPI SRAM chip-select (active low)
- `uo_out[5]` = `sram_sck`  — SPI SRAM clock
- `uo_out[6]` = `sram_mosi` — SPI SRAM MOSI
- `uio[7:0]`  = 8 general-purpose GPIOs (with PWM support), driven directly by the
  running program

The flash bus (`spi_*`) and the SRAM bus (`sram_*`) are electrically
independent -- separate CS/SCK/MOSI/MISO lines, no sharing -- so both
chips can be wired up at the same time with no contention.

## How to test

Connect:
- an SPI flash (mode 0, e.g. Winbond W25Q80) holding your program image to
  `spi_cs`/`spi_sck`/`spi_mosi`/`spi_miso`;
- an SPI SRAM (e.g. Microchip 23LCV04M-I/P) to
  `sram_cs`/`sram_sck`/`sram_mosi`/`sram_miso`;

and release `rst_n`. The CPU will copy the program from flash into the SRAM
and then boot from it automatically, driving the GPIOs. `prog_rx`/`prog_tx`
expose a UART that can be used for programming/debugging.

A self-contained Icarus Verilog testbench (`test/tb.v`) simulates the SPI
flash and boot sequence, driven from `test/test.py` via cocotb (`make`
inside `test/`). **Note:** this testbench currently only models the flash;
it needs a matching SPI SRAM model (responding to 0x03 READ / 0x02 WRITE)
to exercise the external-SRAM path added in this version.

## External hardware

- SPI SRAM (Microchip 23LCV04M-I/P or pin/command-compatible part), used to
  store the running program and all data (1024 words = 2KB used out of the
  chip's 512KB)
- SPI flash (mode 0, Winbond W25Q80 or compatible), used to store the boot
  program image
- Optional UART adapter on `prog_rx`/`prog_tx` for programming/debugging
