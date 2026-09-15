SRCS = tb.v tt_um_rianfiscina.v top.v cpu.v boot_ctrl.v ext_ram_ctrl.v \
       spi_byte_engine.v gpio.v uart.v spi_flash_model.v spi_sram_model.v

.PHONY: sim wave clean

sim: wave.vcd

wave.vcd: $(SRCS) program.hex
	iverilog -g2012 -o sim.out $(SRCS)
	vvp sim.out

wave: wave.vcd
	gtkwave wave.vcd &

# ---- tb_standalone: testbench autocontido de proposito geral (nao usa
# cocotb nem spi_flash_model.v/spi_sram_model.v -- tem os modelos de flash
# e SRAM externas embutidos). Bom pra iterar rapido num programa novo.
#
#   make standalone                          -> usa program.hex, trace ligado
#   make standalone HEX=meu_programa.hex      -> usa outro .hex
#   make standalone ARGS=+notrace             -> desliga o trace por instrucao
#
STANDALONE_SRCS = ../src/tt_um_rianfiscina.v ../src/top.v ../src/cpu.v \
                   ../src/boot_ctrl.v ../src/ext_ram_ctrl.v \
                   ../src/spi_byte_engine.v ../src/gpio.v ../src/uart.v
HEX ?= program.hex

standalone: tb_standalone.v $(STANDALONE_SRCS) $(HEX)
	iverilog -g2012 -o standalone.out -s tb_standalone tb_standalone.v $(STANDALONE_SRCS)
	vvp standalone.out +hex=$(HEX) $(ARGS)

standalone-wave: standalone
	gtkwave tb_standalone.vcd &

clean:
	rm -f sim.out wave.vcd standalone.out tb_standalone.vcd