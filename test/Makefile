SIM ?= icarus
TOPLEVEL_LANG ?= verilog

SRC_DIR = $(PWD)/../src

# ordem nao importa pro icarus, mas mantenha todos os modulos usados pela hierarquia
PROJECT_SOURCES = \
	uart.v \
	boot_ctrl.v \
	cpu.v \
	gpio.v \
	top.v \
	tt_um_rianfiscina.v

ifeq ($(SIM), icarus)
	VERILOG_SOURCES += $(PWD)/tb.v
	VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))
	COMPILE_ARGS += -DSIM
endif

TOPLEVEL := tb
MODULE   := test

include $(shell cocotb-config --makefiles)/Makefile.sim