; blink.asm — pisca o GPIO ligando/desligando, com atraso entre as
; transicoes. Reconstruido a partir do program.hex original (mesma
; sequencia de instrucoes), so que com o DELAY alterado para 50 ms
; (era 2000 ms no arquivo original).
;
; Monte com:  python3 asm2hex.py blink.asm program.hex

        LOADI R1, 16        ; R1 = 16 (valor de setup, igual ao original)
        STORE R1, 0x3F0     ; guarda R1 em 0x3F0 (igual ao original)

LOOP:
        LOADI R2, 16        ; R2 = 16 -> liga a saida
        OUT   R2            ; GPIO_DATA <= R2
        DELAY 50            ; espera 50 ms

        LOADI R2, 0         ; R2 = 0 -> desliga a saida
        OUT   R2            ; GPIO_DATA <= R2
        DELAY 50            ; espera 50 ms

        JMP   LOOP          ; repete pra sempre
