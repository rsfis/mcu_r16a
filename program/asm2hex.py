#!/usr/bin/env python3
"""
asm2hex.py — monta um .asm da CPU custom (ver cpu.v) para o .hex que o
boot_ctrl.v espera na flash de boot.

Formato do .hex de saida (confirmado lendo boot_ctrl.v):
  - 2 bytes de cabecalho = quantidade de PALAVRAS de 16 bits do
    programa (nao bytes), little-endian (byte baixo primeiro).
  - depois, o programa: uma palavra de 16 bits por instrucao (opcode +
    regs) seguida de uma palavra de operando (endereco/imediato/ms),
    cada palavra em little-endian (byte baixo primeiro, byte alto
    depois) -- mesma convencao usada pela SRAM externa via
    ext_ram_ctrl.v.
  - o arquivo de saida tem 1 byte em hexadecimal (2 digitos, minusculo,
    sem "0x") por linha -- exatamente como o program.hex original.

Todo opcode usa SEMPRE 2 palavras (opcode+regs, depois operando), pois
e assim que cpu.v busca cada instrucao (FETCH_OP sempre seguido de
FETCH_OPERAND, mesmo quando o opcode nao usa operando). Por isso, cada
linha de instrucao no .asm ocupa 2 enderecos de palavra (pc anda de 2
em 2), e labels/JMP/JZ apontam para esse endereco de PALAVRA.

Conjunto de instrucoes (ver cpu.v EXEC):
  NOP                 0x00  (nao usa operandos)
  LOAD   Rd, addr     0x01  Rd <= mem[addr]
  STORE  Rs, addr     0x02  mem[addr] <= Rs
  ADD    Rd, Rs       0x03  Rd <= Rd + Rs
  SUB    Rd, Rs       0x04  Rd <= Rd - Rs
  JMP    addr         0x05  pc <= addr
  JZ     Rs, addr     0x06  if (Rs==0) pc <= addr
  OUT    Rs           0x07  GPIO_DATA <= Rs
  IN     Rd           0x08  Rd <= GPIO_DATA
  MUL    Rd, Rs       0x09  Rd <= Rd * Rs
  DIV    Rd, Rs       0x0A  Rd <= Rd / Rs
  MOD    Rd, Rs       0x0B  Rd <= Rd % Rs
  OR     Rd, Rs       0x0C  Rd <= Rd | Rs
  CMP    Rd, Rs       0x0D  flag_z<=(Rd==Rs), flag_lt<=(Rd<Rs)  (unsigned)
  NOT    Rd           0x0E  Rd <= ~Rd
  HALT                0x0F
  DELAY  ms           0x10  espera "ms" milissegundos (operando = valor
                             direto em ms, NAO passa por registrador;
                             cpu.v faz delay_cnt = operando * CYCLES_PER_MS)
  MOV    Rd, Rs       0x11  Rd <= Rs
  LOADI  Rd, imm      0x12  Rd <= imm
  LOADR  Rd, Rs       0x13  Rd <= mem[Rs]      (endereco vem do registrador Rs)
  STORER Rd, Rs       0x14  mem[Rs] <= Rd      (endereco vem do registrador Rs)
  AND    Rd, Rs       0x15  Rd <= Rd & Rs
  CALL   addr         0x16  empilha (pc+2) em mem[R14], R14--, pc <= addr
  RET                 0x17  R14++, pc <= mem[R14]   (endereco de retorno empilhado pelo CALL)
  JNZ    Rs, addr     0x18  if (Rs!=0) pc <= addr   (complemento do JZ)
  ADDI   Rd, imm      0x19  Rd <= Rd + imm
  SUBI   Rd, imm      0x1A  Rd <= Rd - imm
  INC    Rd           0x1B  Rd <= Rd + 1
  DEC    Rd           0x1C  Rd <= Rd - 1
  PUSH   Rs           0x1D  mem[R14] <= Rs, R14--   (mesmo SP do CALL/RET)
  POP    Rd           0x1E  R14++, Rd <= mem[R14]   (mesmo SP do CALL/RET)
  XOR    Rd, Rs       0x1F  Rd <= Rd ^ Rs
  CMPI   Rd, imm      0x20  flag_z<=(Rd==imm), flag_lt<=(Rd<imm) (unsigned)
  JE     addr         0x21  if (flag_z)          pc <= addr   (igual)
  JNE    addr         0x22  if (!flag_z)         pc <= addr   (diferente)
  JL     addr         0x23  if (flag_lt)         pc <= addr   (menor)
  JNL    addr         0x24  if (!flag_lt)        pc <= addr   (nao-menor, >=)
  JG     addr         0x25  if (!flag_z&&!flag_lt) pc <= addr (maior)
  JNG    addr         0x26  if (flag_z||flag_lt) pc <= addr   (nao-maior, <=)

CMP/CMPI calculam a comparacao UMA vez (igual e menor, ambos SEM SINAL
pois regfile e' 'reg [15:0]') e gravam em flag_z/flag_lt; depois, os
seis jumps condicionais (JE/JNE/JL/JNL/JG/JNG) testam essas flags pra
cobrir as 6 relacoes (==, !=, <, >=, >, <=). Uso tipico:
    CMP   R0, R1
    JNL   pular        ; pula se R0 NAO for menor que R1 (R0>=R1)
    CALL  rotina_menor
    pular:
As flags so' mudam quando CMP/CMPI roda -- um JE/JNE/JL/JNL/JG/JNG sem
CMP/CMPI antes testa o resultado da ULTIMA comparacao feita.

CALL/RET/PUSH/POP usam R14 como stack pointer (SP) -- ver comentario em cpu.v.
O regfile NAO e' zerado no reset, entao todo programa que usa CALL ou
PUSH/POP precisa dar um LOADI R14, <endereco> antes do primeiro uso
(sugestao: perto do topo do espaco de dados livre, ex. 0x3EC, pra pilha
crescer "pra baixo" sem invadir a area de I/O em 0x3EE+).

Registradores: R0..R14 (15 no total, regfile[0:14]).
Numeros: decimal ("50") ou hexadecimal ("0x32"). Labels: "nome:" numa
linha, referenciadas sem os dois-pontos em JMP/JZ/enderecos.
Comentarios: ";" ate o fim da linha.

Uso:
    python3 asm2hex.py entrada.asm saida.hex
"""
import re
import sys

OPCODES = {
    "NOP":    0x00,
    "LOAD":   0x01,
    "STORE":  0x02,
    "ADD":    0x03,
    "SUB":    0x04,
    "JMP":    0x05,
    "JZ":     0x06,
    "OUT":    0x07,
    "IN":     0x08,
    "MUL":    0x09,
    "DIV":    0x0A,
    "MOD":    0x0B,
    "OR":     0x0C,
    "CMP":    0x0D,
    "NOT":    0x0E,
    "HALT":   0x0F,
    "DELAY":  0x10,
    "MOV":    0x11,
    "LOADI":  0x12,
    "LOADR":  0x13,
    "STORER": 0x14,
    "AND":    0x15,
    "CALL":   0x16,
    "RET":    0x17,
    "JNZ":    0x18,
    "ADDI":   0x19,
    "SUBI":   0x1A,
    "INC":    0x1B,
    "DEC":    0x1C,
    "PUSH":   0x1D,
    "POP":    0x1E,
    "XOR":    0x1F,
    "CMPI":   0x20,
    "JE":     0x21,
    "JNE":    0x22,
    "JL":     0x23,
    "JNL":    0x24,
    "JG":     0x25,
    "JNG":    0x26,
}

# Para cada mnemonico: quais campos ele usa.
#   "rd"    -> primeiro operando e' um registrador, vira reg_dst
#   "rs"    -> primeiro (ou segundo) operando e' um registrador, vira reg_src
#   "imm"   -> operando numerico/label, vira a palavra de operando (word 2)
# Formato de cada entrada: (usa_rd, usa_rs, usa_imm)
LAYOUT = {
    "NOP":    (False, False, False),
    "LOAD":   (True,  False, True),   # LOAD Rd, addr
    "STORE":  (False, True,  True),   # STORE Rs, addr
    "ADD":    (True,  True,  False),  # ADD Rd, Rs
    "SUB":    (True,  True,  False),
    "JMP":    (False, False, True),   # JMP addr
    "JZ":     (False, True,  True),   # JZ Rs, addr
    "OUT":    (False, True,  False),  # OUT Rs
    "IN":     (True,  False, False),  # IN Rd
    "MUL":    (True,  True,  False),
    "DIV":    (True,  True,  False),
    "MOD":    (True,  True,  False),
    "OR":     (True,  True,  False),
    "CMP":    (True,  True,  False),
    "NOT":    (True,  False, False),  # NOT Rd
    "HALT":   (False, False, False),
    "DELAY":  (False, False, True),   # DELAY ms  (imediato puro, nao e' registrador)
    "MOV":    (True,  True,  False),  # MOV Rd, Rs
    "LOADI":  (True,  False, True),   # LOADI Rd, imm
    "LOADR":  (True,  True,  False),  # LOADR Rd, Rs   (endereco = Rs)
    "STORER": (True,  True,  False),  # STORER Rd, Rs  (endereco = Rs)
    "AND":    (True,  True,  False),  # AND Rd, Rs
    "CALL":   (False, False, True),   # CALL addr
    "RET":    (False, False, False),  # RET (sem operandos)
    "JNZ":    (False, True,  True),   # JNZ Rs, addr
    "ADDI":   (True,  False, True),   # ADDI Rd, imm
    "SUBI":   (True,  False, True),   # SUBI Rd, imm
    "INC":    (True,  False, False),  # INC Rd
    "DEC":    (True,  False, False),  # DEC Rd
    "PUSH":   (False, True,  False),  # PUSH Rs
    "POP":    (True,  False, False),  # POP Rd
    "XOR":    (True,  True,  False),  # XOR Rd, Rs
    "CMPI":   (True,  False, True),   # CMPI Rd, imm
    "JE":     (False, False, True),   # JE addr
    "JNE":    (False, False, True),   # JNE addr
    "JL":     (False, False, True),   # JL addr
    "JNL":    (False, False, True),   # JNL addr
    "JG":     (False, False, True),   # JG addr
    "JNG":    (False, False, True),   # JNG addr
}


class AsmError(Exception):
    pass


def parse_reg(tok):
    tok = tok.strip().upper()
    m = re.fullmatch(r"R(\d+)", tok)
    if not m:
        raise AsmError(f"registrador invalido: {tok!r}")
    n = int(m.group(1))
    if not (0 <= n <= 14):
        raise AsmError(f"registrador fora do intervalo (R0-R14): {tok!r}")
    return n


def parse_number(tok, labels):
    tok = tok.strip()
    if tok in labels:
        return labels[tok]
    tok_low = tok.lower()
    if tok_low.startswith("0x"):
        return int(tok, 16)
    return int(tok, 10)


def strip_comment(line):
    out = []
    for ch in line:
        if ch == ";":
            break
        out.append(ch)
    return "".join(out)


def tokenize_line(line):
    """Retorna (label_ou_None, mnemonico_ou_None, [operandos])."""
    line = strip_comment(line).strip()
    if not line:
        return None, None, []

    label = None
    m = re.match(r"^([A-Za-z_][A-Za-z0-9_]*)\s*:\s*(.*)$", line)
    if m:
        label = m.group(1)
        line = m.group(2).strip()
        if not line:
            return label, None, []

    parts = line.split(None, 1)
    mnem = parts[0].upper()
    operands = []
    if len(parts) > 1:
        operands = [p.strip() for p in parts[1].split(",")]
    return label, mnem, operands


def assemble(src_text):
    lines = src_text.splitlines()

    # ---- passo 1: descobre o endereco (em PALAVRAS) de cada label ----
    labels = {}
    addr = 0
    parsed = []  # (mnem, operandos, linha_original) na ordem, sem os labels
    for raw in lines:
        label, mnem, operands = tokenize_line(raw)
        if label is not None:
            if label in labels:
                raise AsmError(f"label duplicada: {label!r}")
            labels[label] = addr
        if mnem is not None:
            if mnem not in OPCODES:
                raise AsmError(f"mnemonico desconhecido: {mnem!r} (linha: {raw!r})")
            parsed.append((mnem, operands, raw))
            addr += 2  # toda instrucao ocupa 2 palavras (opcode+regs, operando)

    # ---- passo 2: gera as palavras de verdade, agora que labels sao conhecidas ----
    words = []
    for mnem, operands, raw in parsed:
        opcode = OPCODES[mnem]
        usa_rd, usa_rs, usa_imm = LAYOUT[mnem]

        expected = sum([usa_rd, usa_rs, usa_imm])
        # DELAY/JMP usam 1 operando mas ele e' "imm", nao registrador;
        # os demais layouts com 2 campos usam 2 operandos separados por virgula.
        if mnem in ("LOAD", "LOADI"):
            n_expected = 2
        elif mnem in ("STORE",):
            n_expected = 2
        elif mnem in ("JZ", "JNZ"):
            n_expected = 2
        elif mnem in ("JMP", "DELAY", "OUT", "IN", "NOT", "CALL", "PUSH", "INC", "DEC", "POP",
                      "JE", "JNE", "JL", "JNL", "JG", "JNG"):
            n_expected = 1
        elif mnem in ("ADDI", "SUBI", "CMPI"):
            n_expected = 2
        elif mnem in ("NOP", "HALT", "RET"):
            n_expected = 0
        else:
            n_expected = 2  # ADD/SUB/MUL/DIV/MOD/OR/CMP/MOV/LOADR/STORER/AND/XOR

        if len(operands) != n_expected:
            raise AsmError(
                f"{mnem} espera {n_expected} operando(s), recebeu "
                f"{len(operands)} (linha: {raw!r})"
            )

        reg_dst = 0
        reg_src = 0
        imm = 0

        if mnem in ("LOAD", "LOADI"):
            reg_dst = parse_reg(operands[0])
            imm = parse_number(operands[1], labels)
        elif mnem == "STORE":
            reg_src = parse_reg(operands[0])
            imm = parse_number(operands[1], labels)
        elif mnem == "JZ":
            reg_src = parse_reg(operands[0])
            imm = parse_number(operands[1], labels)
        elif mnem == "JNZ":
            reg_src = parse_reg(operands[0])
            imm = parse_number(operands[1], labels)
        elif mnem in ("JMP", "CALL", "JE", "JNE", "JL", "JNL", "JG", "JNG"):
            imm = parse_number(operands[0], labels)
        elif mnem == "DELAY":
            imm = parse_number(operands[0], labels)
        elif mnem in ("OUT", "PUSH"):
            reg_src = parse_reg(operands[0])
        elif mnem in ("IN", "NOT", "INC", "DEC", "POP"):
            reg_dst = parse_reg(operands[0])
        elif mnem in ("ADD", "SUB", "MUL", "DIV", "MOD", "OR", "AND", "XOR", "CMP", "MOV"):
            reg_dst = parse_reg(operands[0])
            reg_src = parse_reg(operands[1])
        elif mnem in ("ADDI", "SUBI", "CMPI"):
            reg_dst = parse_reg(operands[0])
            imm = parse_number(operands[1], labels)
        elif mnem == "LOADR":
            reg_dst = parse_reg(operands[0])
            reg_src = parse_reg(operands[1])
        elif mnem == "STORER":
            reg_dst = parse_reg(operands[0])
            reg_src = parse_reg(operands[1])
        elif mnem in ("NOP", "HALT", "RET"):
            pass
        else:
            raise AsmError(f"mnemonico nao tratado: {mnem}")

        if not (0 <= imm <= 0xFFFF):
            raise AsmError(f"operando fora de 16 bits em {mnem}: {imm} (linha: {raw!r})")

        op_word = ((opcode & 0xFF) << 8) | ((reg_dst & 0xF) << 4) | (reg_src & 0xF)
        words.append(op_word)
        words.append(imm & 0xFFFF)

    return words


def words_to_hex_bytes(words):
    """Cabecalho (contagem de palavras, little-endian) + palavras (little-endian)."""
    word_count = len(words) // 2  # numero de INSTRUCOES = numero de palavras / 2? -- ver nota abaixo
    # NOTA: boot_ctrl.v guarda em word_count a quantidade TOTAL de
    # palavras de 16 bits do programa (opcode+operando contam como 2
    # palavras cada), nao o numero de instrucoes. Corrigido abaixo.
    word_count = len(words)

    out = bytearray()
    out.append(word_count & 0xFF)
    out.append((word_count >> 8) & 0xFF)
    for w in words:
        out.append(w & 0xFF)
        out.append((w >> 8) & 0xFF)
    return out


def main():
    if len(sys.argv) != 3:
        print("uso: python3 asm2hex.py entrada.asm saida.hex", file=sys.stderr)
        sys.exit(1)

    src_path, dst_path = sys.argv[1], sys.argv[2]
    with open(src_path, "r", encoding="utf-8") as f:
        src = f.read()

    try:
        words = assemble(src)
    except AsmError as e:
        print(f"erro de montagem: {e}", file=sys.stderr)
        sys.exit(1)

    data = words_to_hex_bytes(words)
    with open(dst_path, "w", encoding="utf-8") as f:
        for b in data:
            f.write(f"{b:02x}\n")

    print(f"OK: {len(words)} palavras ({len(words)//2} instrucoes), "
          f"{len(data)} bytes -> {dst_path}")


if __name__ == "__main__":
    main()
