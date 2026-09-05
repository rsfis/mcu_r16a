# mcu_r16a
#### 16bit custom microcontroller

## How to run
iverilog -g2012 -o sim test/tb.v src/top.v src/cpu.v src/gpio.v src/uart.v src/boot_ctrl.v src/tt_um_rianfiscina.v
vvp sim
gtkwave tb_full.vcd
