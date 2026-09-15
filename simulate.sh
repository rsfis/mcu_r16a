iverilog -g2012 -o sim test/tb_standalone.v src/top.v src/cpu.v src/gpio.v src/uart.v src/boot_ctrl.v src/tt_um_rianfiscina.v src/ext_ram_ctrl.v src/spi_byte_engine.v test/spi_flash_model.v test/spi_sram_model.v src/flash_fetch.v

vvp sim

gtkwave wave.vcd -g