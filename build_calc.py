#!/usr/bin/env python3
"""
ARM64 기계어 수준 사칙연산 계산기 빌드 스크립트

모든 ARM64 명령어를 기계어 바이트로 직접 인코딩한 후,
.s 파일의 .byte 디렉티브로 출력합니다.
as/ld로 유효한 Mach-O를 만들지만, 실행 코드는 100% 수동 기계어입니다.

사용법:
  python3 build_calc.py
  => calc 실행파일 생성
"""
import struct
import subprocess
import os
import sys

OUTPUT = "calc"

# ============================================================
# ARM64 명령어 인코딩 (모든 명령어를 32비트 정수로 직접 생성)
# ============================================================

def movz(rd, imm16, hw=0):
    """MOVZ Xrd, #imm16, LSL #(hw*16)"""
    return (0b110100101 << 23) | (hw << 21) | ((imm16 & 0xFFFF) << 5) | rd

def movk(rd, imm16, hw=0):
    """MOVK Xrd, #imm16, LSL #(hw*16)"""
    return (0b111100101 << 23) | (hw << 21) | ((imm16 & 0xFFFF) << 5) | rd

def mov_reg(rd, rm):
    """MOV Xrd, Xrm (ORR Xrd, XZR, Xrm)"""
    return (0b10101010000 << 21) | (rm << 16) | (31 << 5) | rd

def add_reg(rd, rn, rm):
    """ADD Xrd, Xrn, Xrm"""
    return (0b10001011000 << 21) | (rm << 16) | (rn << 5) | rd

def sub_reg(rd, rn, rm):
    """SUB Xrd, Xrn, Xrm"""
    return (0b11001011000 << 21) | (rm << 16) | (rn << 5) | rd

def neg_reg(rd, rm):
    """NEG Xrd, Xrm (SUB Xrd, XZR, Xrm)"""
    return sub_reg(rd, 31, rm)

def add_imm(rd, rn, imm12):
    """ADD Xrd, Xrn, #imm12"""
    return (0b1001000100 << 22) | ((imm12 & 0xFFF) << 10) | (rn << 5) | rd

def sub_imm(rd, rn, imm12):
    """SUB Xrd, Xrn, #imm12"""
    return (0b1101000100 << 22) | ((imm12 & 0xFFF) << 10) | (rn << 5) | rd

def mul_reg(rd, rn, rm):
    """MUL Xrd, Xrn, Xrm (MADD Xrd, Xrn, Xrm, XZR)"""
    return (0b10011011000 << 21) | (rm << 16) | (31 << 10) | (rn << 5) | rd

def sdiv_reg(rd, rn, rm):
    """SDIV Xrd, Xrn, Xrm"""
    return (0b10011010110 << 21) | (rm << 16) | (0b000011 << 10) | (rn << 5) | rd

def msub_reg(rd, rn, rm, ra):
    """MSUB Xrd, Xrn, Xrm, Xra (Ra - Rn*Rm)"""
    return (0b10011011000 << 21) | (rm << 16) | (1 << 15) | (ra << 10) | (rn << 5) | rd

def stp_pre(rt, rt2, rn, simm7):
    """STP Xt, Xt2, [Xn, #simm7*8]! (pre-index, signed offset)"""
    return (0b1010100110 << 22) | ((simm7 & 0x7F) << 15) | (rt2 << 10) | (rn << 5) | rt

def ldp_post(rt, rt2, rn, simm7):
    """LDP Xt, Xt2, [Xn], #simm7*8 (post-index)"""
    return (0b1010100011 << 22) | ((simm7 & 0x7F) << 15) | (rt2 << 10) | (rn << 5) | rt

def ldrb_uoff(rt, rn, uoff):
    """LDRB Wt, [Xn, #uoff]"""
    return (0b0011100101 << 22) | ((uoff & 0xFFF) << 10) | (rn << 5) | rt

def strb_uoff(rt, rn, uoff):
    """STRB Wt, [Xn, #uoff]"""
    return (0b0011100100 << 22) | ((uoff & 0xFFF) << 10) | (rn << 5) | rt

def svc(imm16):
    """SVC #imm16"""
    return (0b11010100000 << 21) | ((imm16 & 0xFFFF) << 5) | 0b00001

def ret_lr():
    """RET (x30)"""
    return 0xd65f03c0

def b_off(off):
    """B <offset_in_instructions>"""
    return (0b000101 << 26) | (off & 0x3FFFFFF)

def b_cond(cond, off):
    """B.cond <offset_in_instructions>"""
    return (0b01010100 << 24) | ((off & 0x7FFFF) << 5) | cond

def cbz_x(rt, off):
    """CBZ Xt, <offset_in_instructions>"""
    return (0b10110100 << 24) | ((off & 0x7FFFF) << 5) | rt

def cbnz_x(rt, off):
    """CBNZ Xt, <offset_in_instructions>"""
    return (0b10110101 << 24) | ((off & 0x7FFFF) << 5) | rt

def cmp_imm12(rn, imm12):
    """CMP Xrn, #imm12 (SUBS XZR, Xrn, #imm12)"""
    return (0b1111000100 << 22) | ((imm12 & 0xFFF) << 10) | (rn << 5) | 31

def adr_x(rd, off):
    """ADR Xrd, <byte_offset_from_PC>"""
    immlo = off & 0x3
    immhi = (off >> 2) & 0x7FFFF
    return (immlo << 29) | (0b10000 << 24) | (immhi << 5) | rd

# Condition codes
NE = 0b0001
GE = 0b1010
HI = 0b1000  # unsigned higher

SP = 31  # stack pointer (when used as Rn in add/sub)
XZR = 31 # zero register (when used as Rt in strb)


# ============================================================
# 코드 빌더: 라벨 기반 forward/backward 참조 해결
# ============================================================

class CodeBuilder:
    """명령어 리스트를 관리하고 라벨/패치를 처리"""
    def __init__(self):
        self.instrs = []       # list of 32-bit integers
        self.patches = []      # (index, label_name, patch_func)
        self.labels = {}       # label_name -> instruction index

    def emit(self, instr):
        self.instrs.append(instr)
        return len(self.instrs) - 1

    def label(self, name):
        self.labels[name] = len(self.instrs)

    def emit_patch(self, patch_func, label_name):
        """placeholder를 emit하고 나중에 patch_func(offset)로 채움"""
        idx = len(self.instrs)
        self.instrs.append(0)  # placeholder
        self.patches.append((idx, label_name, patch_func))
        return idx

    def cur(self):
        return len(self.instrs)

    def resolve(self):
        for idx, label_name, patch_func in self.patches:
            target = self.labels[label_name]
            offset = target - idx  # offset in instructions
            self.instrs[idx] = patch_func(offset)

    def to_bytes(self):
        self.resolve()
        return b''.join(struct.pack('<I', i & 0xFFFFFFFF) for i in self.instrs)


# ============================================================
# 사칙연산 계산기 기계어 코드 생성
# ============================================================

def build_calculator():
    """
    10자리 10진수 사칙연산 계산기를 ARM64 기계어로 생성합니다.

    입력: "num1 op num2" (stdin에서 읽음)
    출력: 결과 (stdout에 출력)

    스택 레이아웃 (sp 기준):
      sp+0   ~ sp+63:   input_buf (64 bytes)
      sp+64  ~ sp+95:   output_buf (32 bytes)
      sp+96  ~ sp+127:  여분

    레지스터 사용:
      x19: rodata 포인터 (callee-saved)
      x20: input 파서 포인터 (callee-saved)
      x21: num1 부호 플래그
      x22: num1 값
      x23: 연산자 문자
      x24: num2 부호 플래그
      x25: num2 값
      x26: 연산 결과
      x27: output 포인터
      x28: output 길이
      x15: 결과 부호 플래그
    """
    c = CodeBuilder()

    # 스택 상수
    STK_INPUT  = 0
    STK_OUTPUT = 64

    # --- 프롤로그 ---
    c.emit(stp_pre(29, 30, SP, -16 & 0x7F))  # STP x29, x30, [sp, #-128]!
    c.emit(add_imm(29, SP, 0))                  # MOV x29, sp (ADD x29, sp, #0)
    c.emit(sub_imm(SP, SP, 160))               # SUB sp, sp, #160 (로컬 변수)

    # --- x19 = rodata 주소 (ADR 사용, PC-relative) ---
    # 코드 끝에 rodata를 배치하고 ADR로 참조
    # ADR의 오프셋은 나중에 패치
    adr_idx = c.emit_patch(lambda off: adr_x(19, off * 4), 'rodata')

    # === 프롬프트 출력: write(1, prompt, 6) ===
    c.emit(movz(0, 1, 0))               # x0 = 1 (stdout)
    c.emit(add_imm(1, 19, 0))           # x1 = rodata + 0 (prompt)
    c.emit(movz(2, 6, 0))               # x2 = 6 (len "Calc> ")
    c.emit(movz(16, 4, 0))              # x16 = 4 (SYS_write)
    c.emit(svc(0x80))

    # === stdin 읽기: read(0, sp, 63) ===
    c.emit(movz(0, 0, 0))               # x0 = 0 (stdin)
    c.emit(add_imm(1, SP, STK_INPUT))   # x1 = sp (input_buf)
    c.emit(movz(2, 63, 0))              # x2 = 63
    c.emit(movz(16, 3, 0))              # x16 = 3 (SYS_read)
    c.emit(svc(0x80))                   # x0 = bytes read

    # null terminate: 먼저 sp를 x10에 복사 후 bytes_read(x0) 더하기
    c.emit(add_imm(10, SP, 0))          # x10 = sp (ADD immediate에서 31=SP)
    c.emit(add_reg(10, 10, 0))          # x10 = sp + bytes_read (x0)
    c.emit(strb_uoff(XZR, 10, 0))      # *(sp + bytes_read) = 0

    # x20 = input parser pointer
    c.emit(add_imm(20, SP, STK_INPUT))

    # === 공백 스킵 (num1 앞) ===
    c.label('skip_sp1')
    c.emit(ldrb_uoff(9, 20, 0))
    c.emit(cmp_imm12(9, 0x20))          # ' '
    c.emit_patch(lambda off: b_cond(NE, off), 'skip_sp1_end')
    c.emit(add_imm(20, 20, 1))
    c.emit_patch(lambda off: b_off(off), 'skip_sp1')
    c.label('skip_sp1_end')

    # === num1 부호 체크 ===
    c.emit(movz(21, 0, 0))              # x21 = 0 (양수)
    c.emit(ldrb_uoff(9, 20, 0))
    c.emit(cmp_imm12(9, 0x2D))          # '-'
    c.emit_patch(lambda off: b_cond(NE, off), 'no_neg1')
    c.emit(movz(21, 1, 0))              # 음수 플래그
    c.emit(add_imm(20, 20, 1))
    c.label('no_neg1')

    # === num1 파싱 (x22) ===
    c.emit(movz(22, 0, 0))              # x22 = 0
    c.label('parse1')
    c.emit(ldrb_uoff(9, 20, 0))         # x9 = 현재 문자
    c.emit(sub_imm(10, 9, 0x30))        # x10 = ch - '0'
    c.emit(cmp_imm12(10, 9))            # 숫자가 아니면 (>9)
    c.emit_patch(lambda off: b_cond(HI, off), 'parse1_end')
    c.emit(movz(11, 10, 0))             # x11 = 10
    c.emit(mul_reg(22, 22, 11))          # x22 *= 10
    c.emit(add_reg(22, 22, 10))          # x22 += digit
    c.emit(add_imm(20, 20, 1))
    c.emit_patch(lambda off: b_off(off), 'parse1')
    c.label('parse1_end')

    # 부호 적용
    c.emit_patch(lambda off: cbz_x(21, off), 'no_apply_neg1')
    c.emit(neg_reg(22, 22))
    c.label('no_apply_neg1')

    # === 공백 스킵 (연산자 앞) ===
    c.label('skip_sp2')
    c.emit(ldrb_uoff(9, 20, 0))
    c.emit(cmp_imm12(9, 0x20))
    c.emit_patch(lambda off: b_cond(NE, off), 'skip_sp2_end')
    c.emit(add_imm(20, 20, 1))
    c.emit_patch(lambda off: b_off(off), 'skip_sp2')
    c.label('skip_sp2_end')

    # === 연산자 (x23) ===
    c.emit(ldrb_uoff(23, 20, 0))
    c.emit(add_imm(20, 20, 1))

    # === 공백 스킵 (num2 앞) ===
    c.label('skip_sp3')
    c.emit(ldrb_uoff(9, 20, 0))
    c.emit(cmp_imm12(9, 0x20))
    c.emit_patch(lambda off: b_cond(NE, off), 'skip_sp3_end')
    c.emit(add_imm(20, 20, 1))
    c.emit_patch(lambda off: b_off(off), 'skip_sp3')
    c.label('skip_sp3_end')

    # === num2 부호 체크 ===
    c.emit(movz(24, 0, 0))
    c.emit(ldrb_uoff(9, 20, 0))
    c.emit(cmp_imm12(9, 0x2D))
    c.emit_patch(lambda off: b_cond(NE, off), 'no_neg2')
    c.emit(movz(24, 1, 0))
    c.emit(add_imm(20, 20, 1))
    c.label('no_neg2')

    # === num2 파싱 (x25) ===
    c.emit(movz(25, 0, 0))
    c.label('parse2')
    c.emit(ldrb_uoff(9, 20, 0))
    c.emit(sub_imm(10, 9, 0x30))
    c.emit(cmp_imm12(10, 9))
    c.emit_patch(lambda off: b_cond(HI, off), 'parse2_end')
    c.emit(movz(11, 10, 0))
    c.emit(mul_reg(25, 25, 11))
    c.emit(add_reg(25, 25, 10))
    c.emit(add_imm(20, 20, 1))
    c.emit_patch(lambda off: b_off(off), 'parse2')
    c.label('parse2_end')

    c.emit_patch(lambda off: cbz_x(24, off), 'no_apply_neg2')
    c.emit(neg_reg(25, 25))
    c.label('no_apply_neg2')

    # ============================================================
    # 사칙연산: x22 op x25 -> x26
    # ============================================================

    # '+' (0x2B)
    c.emit(cmp_imm12(23, 0x2B))
    c.emit_patch(lambda off: b_cond(NE, off), 'not_add')
    c.emit(add_reg(26, 22, 25))
    c.emit_patch(lambda off: b_off(off), 'print_result')
    c.label('not_add')

    # '-' (0x2D)
    c.emit(cmp_imm12(23, 0x2D))
    c.emit_patch(lambda off: b_cond(NE, off), 'not_sub')
    c.emit(sub_reg(26, 22, 25))
    c.emit_patch(lambda off: b_off(off), 'print_result')
    c.label('not_sub')

    # '*' (0x2A)
    c.emit(cmp_imm12(23, 0x2A))
    c.emit_patch(lambda off: b_cond(NE, off), 'not_mul')
    c.emit(mul_reg(26, 22, 25))
    c.emit_patch(lambda off: b_off(off), 'print_result')
    c.label('not_mul')

    # '/' (0x2F)
    c.emit(cmp_imm12(23, 0x2F))
    c.emit_patch(lambda off: b_cond(NE, off), 'err_op')
    c.emit_patch(lambda off: cbz_x(25, off), 'err_div')
    c.emit(sdiv_reg(26, 22, 25))
    c.emit_patch(lambda off: b_off(off), 'print_result')

    # --- 에러: 0으로 나누기 ---
    c.label('err_div')
    c.emit(movz(0, 2, 0))               # stderr
    c.emit(add_imm(1, 19, 7))           # "Error: division by zero\n"
    c.emit(movz(2, 24, 0))
    c.emit(movz(16, 4, 0))
    c.emit(svc(0x80))
    c.emit_patch(lambda off: b_off(off), 'do_exit')

    # --- 에러: 알 수 없는 연산자 ---
    c.label('err_op')
    c.emit(movz(0, 2, 0))               # stderr
    c.emit(add_imm(1, 19, 31))          # "Error: unknown operator\n"
    c.emit(movz(2, 24, 0))
    c.emit(movz(16, 4, 0))
    c.emit(svc(0x80))
    c.emit_patch(lambda off: b_off(off), 'do_exit')

    # ============================================================
    # 결과 출력: x26 -> 10진 문자열
    # ============================================================
    c.label('print_result')

    # x27 = output_buf 끝 (sp + 94)
    c.emit(add_imm(27, SP, STK_OUTPUT + 30))
    # '\n' 먼저
    c.emit(movz(9, 0x0A, 0))
    c.emit(strb_uoff(9, 27, 0))
    c.emit(sub_imm(27, 27, 1))
    c.emit(movz(28, 1, 0))              # 길이 카운터 (newline 포함)

    # 음수 체크
    c.emit(movz(15, 0, 0))              # x15 = 부호 플래그
    c.emit(cmp_imm12(26, 0))
    c.emit_patch(lambda off: b_cond(GE, off), 'not_negative')
    c.emit(movz(15, 1, 0))
    c.emit(neg_reg(26, 26))
    c.label('not_negative')

    # 0인 경우
    c.emit_patch(lambda off: cbnz_x(26, off), 'digit_loop')
    c.emit(movz(9, 0x30, 0))            # '0'
    c.emit(strb_uoff(9, 27, 0))
    c.emit(sub_imm(27, 27, 1))
    c.emit(add_imm(28, 28, 1))
    c.emit_patch(lambda off: b_off(off), 'after_digits')

    # 숫자 추출 루프
    c.label('digit_loop')
    c.emit(movz(11, 10, 0))             # x11 = 10
    c.emit(sdiv_reg(9, 26, 11))         # x9 = x26 / 10
    c.emit(msub_reg(10, 9, 11, 26))     # x10 = x26 % 10
    c.emit(add_imm(10, 10, 0x30))       # x10 += '0'
    c.emit(strb_uoff(10, 27, 0))
    c.emit(sub_imm(27, 27, 1))
    c.emit(add_imm(28, 28, 1))
    c.emit(mov_reg(26, 9))              # x26 = quotient
    c.emit_patch(lambda off: cbnz_x(26, off), 'digit_loop')

    c.label('after_digits')

    # 음수 부호 추가
    c.emit_patch(lambda off: cbz_x(15, off), 'no_minus')
    c.emit(movz(9, 0x2D, 0))            # '-'
    c.emit(strb_uoff(9, 27, 0))
    c.emit(sub_imm(27, 27, 1))
    c.emit(add_imm(28, 28, 1))
    c.label('no_minus')

    # write(1, ptr, len)
    c.emit(movz(0, 1, 0))               # stdout
    c.emit(add_imm(1, 27, 1))           # 문자열 시작
    c.emit(mov_reg(2, 28))              # 길이
    c.emit(movz(16, 4, 0))              # SYS_write
    c.emit(svc(0x80))

    # === exit(0) ===
    c.label('do_exit')
    c.emit(movz(0, 0, 0))               # exit code 0
    c.emit(movz(16, 1, 0))              # SYS_exit
    c.emit(svc(0x80))

    # === rodata (코드 뒤에 배치, ADR로 참조) ===
    c.label('rodata')
    # 데이터를 32비트 명령어 슬롯에 저장 (바이트 데이터)
    rodata = bytearray()
    rodata.extend(b"Calc> ")                     # +0  (6 bytes)
    rodata.extend(b"\n")                         # +6  (1 byte)
    rodata.extend(b"Error: division by zero\n")  # +7  (24 bytes)
    rodata.extend(b"Error: unknown operator\n")  # +31 (24 bytes)
    # 4바이트 정렬 패딩
    while len(rodata) % 4:
        rodata.append(0)

    # rodata를 32비트 워드로 emit
    for i in range(0, len(rodata), 4):
        word = struct.unpack('<I', rodata[i:i+4])[0]
        c.emit(word)

    return c.to_bytes()


def main():
    print("Generating ARM64 machine code...")
    code_bytes = build_calculator()
    print(f"  Code size: {len(code_bytes)} bytes ({len(code_bytes)//4} instructions)")

    # .byte 디렉티브로 .s 파일 생성
    asm_lines = [
        '.global _main',
        '.align 2',
        '_main:',
    ]
    for i in range(0, len(code_bytes), 4):
        word = code_bytes[i:i+4]
        hex_bytes = ', '.join(f'0x{b:02x}' for b in word)
        # 디스어셈블리 주석 추가
        instr_val = struct.unpack('<I', word)[0]
        asm_lines.append(f'    .byte {hex_bytes}    // 0x{instr_val:08x}')

    asm_content = '\n'.join(asm_lines) + '\n'

    asm_path = '/tmp/calc_raw.s'
    obj_path = '/tmp/calc_raw.o'

    with open(asm_path, 'w') as f:
        f.write(asm_content)
    print(f"  Assembly written to {asm_path}")

    # 어셈블 + 링크
    sdk = subprocess.check_output(['xcrun', '--show-sdk-path']).decode().strip()

    print("  Assembling...")
    r = subprocess.run(['as', '-arch', 'arm64', '-o', obj_path, asm_path],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(f"  as failed: {r.stderr}")
        sys.exit(1)

    print("  Linking...")
    r = subprocess.run(['ld', '-arch', 'arm64', '-o', OUTPUT, '-e', '_main',
                        f'-L{sdk}/usr/lib', '-lSystem', obj_path],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(f"  ld failed: {r.stderr}")
        sys.exit(1)

    print("  Code signing...")
    r = subprocess.run(['codesign', '-s', '-', OUTPUT], capture_output=True, text=True)
    if r.returncode != 0:
        print(f"  codesign failed: {r.stderr}")
        sys.exit(1)

    size = os.path.getsize(OUTPUT)
    print(f"\nSuccess! Generated: {OUTPUT} ({size} bytes)")
    print(f"Run: ./{OUTPUT}")
    print(f"Usage: enter 'num1 op num2' (e.g. '123 + 456')")

    # 기계어 바이트 덤프 (확인용)
    print(f"\n--- Machine code ({len(code_bytes)} bytes) ---")
    for i in range(0, min(len(code_bytes), 64), 4):
        word = struct.unpack('<I', code_bytes[i:i+4])[0]
        print(f"  +{i:04x}: {code_bytes[i:i+4].hex()}  (0x{word:08x})")
    if len(code_bytes) > 64:
        print(f"  ... ({len(code_bytes) - 64} more bytes)")


if __name__ == '__main__':
    main()
