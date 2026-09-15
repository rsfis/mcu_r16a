; compare_demo.asm — demonstra as 6 relacoes de comparacao (==, !=, <,
; >=, >, <=) usando o par CMP (calcula) + jump condicional (testa).
;
; So' existe UM par de opcodes que calcula a comparacao (CMP Rd,Rs e
; CMPI Rd,imm): eles gravam duas flags, flag_z (Rd==operando) e
; flag_lt (Rd<operando, SEM SINAL). Os seis jumps condicionais
; (JE/JNE/JL/JNL/JG/JNG) sao o que decide o desvio a partir dessas
; flags -- ver comentario no topo de cpu.v e em asm2hex.py.
;
; R0 = 5, R1 = 10 (R0 < R1). Cada bloco abaixo faz CMP R0,R1 de novo
; (flags nao "grudam" sozinhas de forma util entre blocos diferentes,
; entao refazemos a comparacao antes de cada teste) e usa OUT pra
; jogar 1 no GPIO se a relacao testada bateu, 0 se nao bateu.

LOADI R0, 5
LOADI R1, 10
LOADI R9, 1        ; R9 = "verdadeiro" (usado so' pra ter algo pra OUT)
LOADI R8, 0        ; R8 = "falso"

; ---- == ----
CMP R0, R1
JE  eq_true
OUT R8
JMP eq_done
eq_true: OUT R9
eq_done:

; ---- != ----
CMP R0, R1
JNE ne_true
OUT R8
JMP ne_done
ne_true: OUT R9
ne_done:

; ---- < ----
CMP R0, R1
JL  lt_true
OUT R8
JMP lt_done
lt_true: OUT R9
lt_done:

; ---- >= ----
CMP R0, R1
JNL ge_true
OUT R8
JMP ge_done
ge_true: OUT R9
ge_done:

; ---- > ----
CMP R0, R1
JG  gt_true
OUT R8
JMP gt_done
gt_true: OUT R9
gt_done:

; ---- <= ----
CMP R0, R1
JNG le_true
OUT R8
JMP le_done
le_true: OUT R9
le_done:

HALT
