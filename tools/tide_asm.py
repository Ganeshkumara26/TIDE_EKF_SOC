#!/usr/bin/env python3
"""tide_asm.py -- TIDE-EKF microcode assembler (gate G1, Agent 1).

Implements doc 20 with the clarifications recorded in ASSUMPTIONS.md.
Python 3.8+, standard library only.

    tide_asm.py a.tasm b.tasm ... -o tide_ucode.bin [--lst f.lst] [--map f.map]
    tide_asm.py --test

Source format: one instruction per line, five '|' separated slots
    MUL | ADD | DS | LSU/ALU | CTRL
(missing trailing slots are NOP).  '//' starts a comment.  'label:' defines a
label (may share a line with an instruction).  Directives:
    .equ NAME, value        .entry NAME  (export a start address)
    .livein r1-r27, r118    (registers defined on entry to any .entry/addr 0)

Slot syntax
    MUL:  MUL rd, ra, rb
    ADD:  ADD|SUB|MIN|MAX rd, ra, rb   CMP [rd,] ra, rb   ABS|NEG|MOV rd, ra
    DS:   DIV rd, ra, rb   SQRT rd, ra      (rd goes to the LSU-slot reg field, D1)
    LSU:  LOAD rd,[addr|addr+idx0|idx1]  LOAD rd,[CONST:n]  STORE rs,[addr..]
          IADD rd,imm  ISUB rd,rs2  ICMP r,rs2  ITOF r  LDLOOP r  LDIDX0 r
          LDIDX1 r  IMOV rd,imm
    CTRL: NOP | END_UNIT | BOUNDARY | BRA l | BLT l | BGE l | LOOP_DEC |
          LOOP_BRA l | IDX0_INC | IDX1_INC   (combine with '+', e.g.
          LOOP_DEC + LOOP_BRA l)
"""
import sys, os, re, struct, zlib, argparse, glob

# ----------------------------------------------------------------------------
# ISA tables (contract: shared/contract/isa.yaml, Step-0 decisions)
# ----------------------------------------------------------------------------
NREG = 128
R_IDX1, R_IDX0, R_ZERO = 125, 126, 127
MUL_OPS = {'NOP': 0, 'FMUL': 1}
ADD_OPS = {'FADD': 0, 'FSUB': 1, 'FCMP': 2, 'FMIN': 3, 'FMAX': 4, 'FABS': 5, 'FNEG': 6, 'FMOV': 7}
DS_OPS = {'NOP': 0, 'FDIV': 1, 'FSQRT': 2}
LSU_OPS = {'NOP': 0, 'LOAD': 1, 'STORE': 2, 'LOAD_ROM': 3, 'IADD': 4, 'ISUB': 5, 'ICMP': 6,
           'ITOF': 7, 'LDLOOP': 8, 'LDIDX0': 9, 'LDIDX1': 10, 'IMOV': 11}
LSU_NAMES = {v: k for k, v in LSU_OPS.items()}
ADD_NAMES = {v: k for k, v in ADD_OPS.items()}
ADD_BYPASS = {'FCMP', 'FMIN', 'FMAX', 'FABS', 'FNEG', 'FMOV'}
ADD_UNARY = {'FABS', 'FNEG', 'FMOV'}
# write cycle offset W (write at end of cycle issue+W; readable at issue+W+1)
LAT = {'FMUL': 2, 'FADD': 3, 'FSUB': 3, 'BYPASS': 1, 'FDIV': 28, 'FSQRT': 27, 'LSU': 1}
DS_BUSY = {'FDIV': 29, 'FSQRT': 28}          # earliest next DS start = issue + lat + 1
CMP_TO_BRANCH = 2                             # A-CMP: branch >= 2 cycles after FCMP/ICMP
BRANCH_BUBBLE = 1                             # Step 0: 1-cycle squash on taken branch
UCODE_WORDS = 512
MAGIC = 0x54494445
LOOP_MAX = 0xFFFF

SLOT_OF = (
    {'MUL', 'FMUL'},
    {'ADD', 'SUB', 'CMP', 'MIN', 'MAX', 'ABS', 'NEG', 'MOV', 'FADD', 'FSUB', 'FCMP', 'FMIN', 'FMAX', 'FABS', 'FNEG', 'FMOV'},
    {'DIV', 'SQRT', 'FDIV', 'FSQRT'},
    {'LOAD', 'STORE', 'IADD', 'ISUB', 'ICMP', 'ITOF', 'LDLOOP', 'LDIDX0', 'LDIDX1', 'IMOV'},
    {'END_UNIT', 'BOUNDARY', 'BRA', 'BLT', 'BGE', 'BEQ', 'BNE', 'LOOP_DEC', 'LOOP_BRA', 'IDX0_INC', 'IDX1_INC'},
)

class AsmError(Exception):
    pass

class Diag:
    def __init__(self, sev, where, msg):
        self.sev, self.where, self.msg = sev, where, msg
    def __str__(self):
        return '%s %s: %s' % (self.sev, self.where, self.msg)

# ----------------------------------------------------------------------------
# Instruction representation
# ----------------------------------------------------------------------------
class Ins:
    __slots__ = ('addr', 'file', 'line', 'text', 'mul', 'add', 'ds', 'lsu', 'ctrl', 'word')
    def __init__(self):
        self.mul = self.add = self.ds = self.lsu = None
        self.ctrl = dict(end=0, boundary=0, branch=0, bcond=0, bsense=0, loopdec=0, loopbr=0,
                         idx0=0, idx1=0, target=None)
        self.word = 0
    def where(self):
        return '%s:%d' % (os.path.basename(self.file), self.line)

def parse_int(tok, syms):
    tok = tok.strip()
    if tok in syms:
        return syms[tok]
    try:
        return int(tok, 0)
    except ValueError:
        raise AsmError("bad number or unknown symbol '%s'" % tok)

def parse_reg(tok, allow_special=True):
    t = tok.strip().lower()
    if t == 'zero': return R_ZERO
    if t == 'idx0': return R_IDX0
    if t == 'idx1': return R_IDX1
    m = re.fullmatch(r'r(\d+)', t)
    if not m or int(m.group(1)) >= NREG:
        raise AsmError("bad register '%s'" % tok)
    return int(m.group(1))

def split_args(s):
    """split on commas not inside [...]"""
    out, depth, cur = [], 0, ''
    for ch in s:
        if ch == '[': depth += 1
        if ch == ']': depth -= 1
        if ch == ',' and depth == 0:
            out.append(cur.strip()); cur = ''
        else:
            cur += ch
    if cur.strip() or out:
        out.append(cur.strip())
    return out

def check_dest(r, what):
    if r in (R_IDX0, R_IDX1):
        raise AsmError('%s destination r%d is an index register (read-only via RF)' % (what, r))

# ----------------------------------------------------------------------------
# Slot parsers
# ----------------------------------------------------------------------------
def parse_mul(txt, syms):
    t = txt.split(None, 1)
    m = t[0].upper()
    if m == 'NOP': return None
    if m not in ('MUL', 'FMUL'): raise AsmError("bad MUL-slot mnemonic '%s'" % t[0])
    a = split_args(t[1] if len(t) > 1 else '')
    if len(a) != 3: raise AsmError('MUL needs rd, ra, rb')
    rd, ra, rb = [parse_reg(x) for x in a]
    check_dest(rd, 'MUL')
    return ('FMUL', rd, ra, rb)

def parse_add(txt, syms):
    t = txt.split(None, 1)
    m = t[0].upper()
    if m == 'NOP': return None
    names = {'ADD': 'FADD', 'SUB': 'FSUB', 'CMP': 'FCMP', 'MIN': 'FMIN', 'MAX': 'FMAX',
             'ABS': 'FABS', 'NEG': 'FNEG', 'MOV': 'FMOV'}
    m = names.get(m, m)
    if m not in ADD_OPS: raise AsmError("bad ADD-slot mnemonic '%s'" % t[0])
    a = split_args(t[1] if len(t) > 1 else '')
    regs = [parse_reg(x) for x in a]
    if m == 'FCMP':
        if len(regs) == 2: rd, ra, rb = R_ZERO, regs[0], regs[1]
        elif len(regs) == 3: rd, ra, rb = regs
        else: raise AsmError('CMP needs [rd,] ra, rb')
    elif m in ADD_UNARY:
        if len(regs) != 2: raise AsmError('%s needs rd, ra' % m)
        rd, ra, rb = regs[0], regs[1], None
    else:
        if len(regs) != 3: raise AsmError('%s needs rd, ra, rb' % m)
        rd, ra, rb = regs
    check_dest(rd, m)
    if m == 'FADD' and rd == 0 and ra == 0 and rb == 0:
        raise AsmError('ADD r0, r0, r0 has the all-zero encoding reserved for the ADD-slot NOP (A-ADDNOP)')
    return (m, rd, ra, rb)

def parse_ds(txt, syms):
    t = txt.split(None, 1)
    m = t[0].upper()
    if m == 'NOP': return None
    m = {'DIV': 'FDIV', 'SQRT': 'FSQRT'}.get(m, m)
    if m not in ('FDIV', 'FSQRT'): raise AsmError("bad DS-slot mnemonic '%s'" % t[0])
    a = split_args(t[1] if len(t) > 1 else '')
    regs = [parse_reg(x) for x in a]
    if m == 'FDIV':
        if len(regs) != 3: raise AsmError('DIV needs rd, ra, rb')
        rd, ra, rb = regs
    else:
        if len(regs) != 2: raise AsmError('SQRT needs rd, ra')
        rd, ra, rb = regs[0], regs[1], None
    check_dest(rd, m)
    return (m, rd, ra, rb)

def parse_addr(s, syms):
    """'[...]' memory operand -> (kind, value, idx0, idx1)"""
    s = s.strip()
    if not (s.startswith('[') and s.endswith(']')): raise AsmError("memory operand must be [..], got '%s'" % s)
    s = s[1:-1].strip()
    m = re.fullmatch(r'CONST\s*:\s*(.+)', s, re.I)
    if m: return ('rom', parse_int(m.group(1), syms), 0, 0)
    idx0 = idx1 = 0
    base = 0
    for part in [p.strip() for p in s.split('+')]:
        pl = part.lower()
        if pl == 'idx0': idx0 += 1
        elif pl == 'idx1': idx1 += 1
        else: base += parse_int(part, syms)
    if idx0 + idx1 > 1: raise AsmError('address may use at most one index register (A-IDX)')
    return ('ram', base, idx0, idx1)

def parse_lsu(txt, syms):
    t = txt.split(None, 1)
    m = t[0].upper()
    if m == 'NOP': return None
    a = split_args(t[1] if len(t) > 1 else '')
    if m in ('LOAD', 'STORE'):
        if len(a) != 2: raise AsmError('%s needs reg, [addr]' % m)
        r = parse_reg(a[0])
        kind, v, i0, i1 = parse_addr(a[1], syms)
        if kind == 'rom':
            if m == 'STORE': raise AsmError('cannot STORE to the constant ROM')
            if not 0 <= v < 64: raise AsmError('ROM index out of range')
            check_dest(r, m)
            return ('LOAD_ROM', r, v, 0, 0)
        if not 0 <= v < 4096: raise AsmError('TIDE-RAM word address out of range (0..4095)')
        if m == 'LOAD': check_dest(r, m)
        return (m, r, v, i0, i1)
    if m == 'IMOV' or m == 'IADD':
        if len(a) != 2: raise AsmError('%s needs rd, imm' % m)
        r = parse_reg(a[0]); v = parse_int(a[1], syms)
        if not -4096 <= v <= 4095: raise AsmError('13-bit signed immediate out of range')
        check_dest(r, m)
        return (m, r, v, 0, 0)
    if m in ('ISUB', 'ICMP'):
        if len(a) != 2: raise AsmError('%s needs reg, rs2' % m)
        r = parse_reg(a[0]); r2 = parse_reg(a[1])
        if m == 'ISUB': check_dest(r, m)
        return (m, r, r2, 0, 0)
    if m == 'ITOF':
        if len(a) == 2:
            if parse_reg(a[0]) != parse_reg(a[1]): raise AsmError('ITOF is in place (reg <- itof(reg)); rd must equal ra')
        elif len(a) != 1: raise AsmError('ITOF needs reg')
        r = parse_reg(a[0]); check_dest(r, m)
        return ('ITOF', r, 0, 0, 0)
    if m in ('LDLOOP', 'LDIDX0', 'LDIDX1'):
        if len(a) != 1: raise AsmError('%s needs reg' % m)
        return (m, parse_reg(a[0]), 0, 0, 0)
    raise AsmError("bad LSU/ALU mnemonic '%s'" % t[0])

def parse_ctrl(txt, ctrl):
    for part in re.split(r'[+,]', txt):
        p = part.strip()
        if not p: continue
        w = p.split()
        m = w[0].upper()
        if m == 'NOP': continue
        elif m == 'END_UNIT': ctrl['end'] = 1
        elif m == 'BOUNDARY': ctrl['boundary'] = 1
        elif m in ('BRA', 'BLT', 'BGE', 'LOOP_BRA'):
            if len(w) != 2: raise AsmError('%s needs a label' % m)
            if ctrl['target'] is not None and ctrl['target'] != w[1]:
                raise AsmError('only one branch target per instruction')
            ctrl['target'] = w[1]
            if m == 'LOOP_BRA': ctrl['loopbr'] = 1
            else:
                if ctrl['branch']: raise AsmError('multiple branches in one instruction')
                ctrl['branch'] = 1
                ctrl['bcond'] = 0 if m == 'BRA' else 1
                ctrl['bsense'] = 1 if m == 'BLT' else 0
        elif m in ('BEQ', 'BNE'):
            raise AsmError('%s is not encodable: the ISA has only the cmp_lt latch (BRA/BLT/BGE)' % m)
        elif m == 'LOOP_DEC': ctrl['loopdec'] = 1
        elif m == 'IDX0_INC': ctrl['idx0'] = 1
        elif m == 'IDX1_INC': ctrl['idx1'] = 1
        else: raise AsmError("bad CTRL mnemonic '%s'" % p)

# ----------------------------------------------------------------------------
# Static (per-instruction) checks + operand summaries
# ----------------------------------------------------------------------------
def reads_of(ins):
    """(p01_regs, p23_regs) semantic register reads split by read-port pair"""
    p01, p23 = [], []
    if ins.mul: p01 += [ins.mul[2], ins.mul[3]]
    if ins.ds:
        p01.append(ins.ds[2])
        if ins.ds[3] is not None: p01.append(ins.ds[3])
    if ins.add:
        p23.append(ins.add[2])
        if ins.add[3] is not None: p23.append(ins.add[3])
    if ins.lsu:
        op, r = ins.lsu[0], ins.lsu[1]
        if op in ('STORE', 'IADD', 'ITOF', 'LDLOOP', 'LDIDX0', 'LDIDX1'): p23.append(r)
        elif op in ('ISUB', 'ICMP'): p23 += [r, ins.lsu[2]]
    return p01, p23

def writes_of(ins):
    """list of (reg, W, port, srcname)"""
    w = []
    if ins.mul: w.append((ins.mul[1], LAT['FMUL'], 0, 'FMUL'))
    if ins.add:
        m = ins.add[0]
        lat = LAT['BYPASS'] if m in ADD_BYPASS else LAT[m]
        w.append((ins.add[1], lat, 1, m))
    if ins.ds: w.append((ins.ds[1], LAT[ins.ds[0]], 1, ins.ds[0]))
    if ins.lsu and ins.lsu[0] in ('LOAD', 'LOAD_ROM', 'IADD', 'ISUB', 'ITOF', 'IMOV'):
        w.append((ins.lsu[1], LAT['LSU'], 2, ins.lsu[0]))
    return w

def static_checks(ins, diags):
    def err(m): diags.append(Diag('ERROR', ins.where(), m))
    p01, p23 = reads_of(ins)
    if len(set(p01)) > 2: err('R1: MUL and DS need >2 distinct read-port-0/1 registers')
    if len(set(p23)) > 2: err('R2: ADD and LSU/ALU need >2 distinct read-port-2/3 registers (binary ADD + register-reading LSU op)')
    c = ins.ctrl
    if ins.ds and ins.lsu: err('D1: DS active but LSU/ALU slot is not NOP (reg field is the DS destination)')
    if (c['branch'] or c['loopbr']) and ins.lsu: err('B1: branch/loop_branch with an active LSU/ALU op (imm is the branch target)')
    if c['end'] and c['boundary']: err('END_UNIT and BOUNDARY are mutually exclusive')
    if c['end'] and (c['branch'] or c['loopbr']): err('END_UNIT with a branch')
    if c['branch'] and c['loopbr']: err('branch and loop_branch in one instruction')
    if c['loopbr'] and not c['loopdec']: pass       # allowed: branch on counter without decrement
    if ins.lsu and ins.lsu[0] in ('LOAD', 'STORE'):
        if ins.lsu[3] and not c['idx0']: c['idx0'] = 1     # [..+idx0] implies post-increment of idx0
        if ins.lsu[4] and not c['idx1']: c['idx1'] = 1
        if c['idx0'] and c['idx1'] and (ins.lsu[3] or ins.lsu[4]):
            err('idx_mode 11 with an indexed LOAD/STORE is ambiguous (A-IDX)')
    if ins.add and ins.add[0] in ADD_BYPASS and ins.add[0] != 'FCMP' and ins.add[1] == R_ZERO:
        err('bypass op with rd=r127 is a discarded result (only CMP may do this)')

# ----------------------------------------------------------------------------
# Parsing a whole program
# ----------------------------------------------------------------------------
class Program:
    def __init__(self):
        self.ins, self.labels, self.entries, self.livein, self.diags, self.syms = [], {}, {}, set(), [], {}
        self.files = []

def parse_program(sources):
    """sources: list of (filename, text)"""
    P = Program()
    pending_labels = []
    for fname, text in sources:
        P.files.append(fname)
        for ln, raw in enumerate(text.splitlines(), 1):
            line = raw.split('//')[0].strip()
            if not line: continue
            where = '%s:%d' % (os.path.basename(fname), ln)
            try:
                if line.startswith('.'):
                    w = line.split(None, 1)
                    d = w[0].lower()
                    if d == '.equ':
                        a = split_args(w[1]); P.syms[a[0]] = parse_int(a[1], P.syms)
                    elif d == '.entry':
                        P.entries[w[1].strip()] = None
                    elif d == '.livein':
                        for part in split_args(w[1]):
                            if '-' in part:
                                lo, hi = [parse_reg(x) for x in part.split('-')]
                                P.livein.update(range(lo, hi + 1))
                            else:
                                P.livein.add(parse_reg(part))
                    else:
                        raise AsmError('unknown directive ' + d)
                    continue
                while True:                                    # leading labels
                    m = re.match(r'([A-Za-z_][\w]*)\s*:(.*)$', line)
                    if not m: break
                    pending_labels.append((m.group(1), where))
                    line = m.group(2).strip()
                if not line: continue
                ins = Ins(); ins.file, ins.line, ins.text = fname, ln, line
                if '|' in line:
                    slots = [s.strip() for s in line.split('|')]
                else:                                      # single mnemonic: infer its slot
                    slots = ['NOP'] * 5
                    mn = line.split(None, 1)[0].upper()
                    if mn != 'NOP':
                        for k, names in enumerate(SLOT_OF):
                            if mn in names: slots[k] = line; break
                        else: raise AsmError("unknown mnemonic '%s'" % mn)
                if len(slots) > 5: raise AsmError('more than 5 slots')
                slots += ['NOP'] * (5 - len(slots))
                ins.mul = parse_mul(slots[0] or 'NOP', P.syms)
                ins.add = parse_add(slots[1] or 'NOP', P.syms)
                ins.ds = parse_ds(slots[2] or 'NOP', P.syms)
                ins.lsu = parse_lsu(slots[3] or 'NOP', P.syms)
                parse_ctrl(slots[4] or 'NOP', ins.ctrl)
                ins.addr = len(P.ins)
                for lab, wh in pending_labels:
                    if lab in P.labels: P.diags.append(Diag('ERROR', wh, "duplicate label '%s'" % lab))
                    P.labels[lab] = ins.addr
                pending_labels = []
                P.ins.append(ins)
            except AsmError as e:
                P.diags.append(Diag('ERROR', where, str(e)))
    for lab, wh in pending_labels:
        P.labels[lab] = len(P.ins)
    for e in list(P.entries):
        if e in P.labels: P.entries[e] = P.labels[e]
        else: P.diags.append(Diag('ERROR', 'directive', ".entry '%s' has no label" % e)); del P.entries[e]
    if len(P.ins) > UCODE_WORDS:
        P.diags.append(Diag('ERROR', 'program', 'program has %d words, microcode RAM holds %d' % (len(P.ins), UCODE_WORDS)))
    # encode + static checks
    for ins in P.ins:
        static_checks(ins, P.diags)
        try:
            ins.word = encode(ins, P.labels)
        except AsmError as e:
            P.diags.append(Diag('ERROR', ins.where(), str(e)))
    return P

# ----------------------------------------------------------------------------
# Encoding / decoding
# ----------------------------------------------------------------------------
def encode(ins, labels):
    mul = add = ds = 0
    if ins.mul:
        _, rd, ra, rb = ins.mul
        mul = (1 << 21) | (ra << 14) | (rb << 7) | rd
    if ins.add:
        m, rd, ra, rb = ins.add
        add = (ADD_OPS[m] << 21) | (ra << 14) | ((R_ZERO if rb is None else rb) << 7) | rd
    lsu_op, lsu_reg, lsu_imm = 0, 0, 0
    if ins.ds:
        m, rd, ra, rb = ins.ds
        ds = (DS_OPS[m] << 14) | (ra << 7) | (R_ZERO if rb is None else rb)
        lsu_reg = rd
    if ins.lsu:
        op, r, v = ins.lsu[0], ins.lsu[1], ins.lsu[2]
        lsu_op, lsu_reg = LSU_OPS[op], r
        if op in ('LOAD', 'STORE'): lsu_imm = v & 0xFFF
        elif op == 'LOAD_ROM': lsu_imm = v & 0x3F
        elif op in ('IADD', 'IMOV'): lsu_imm = v & 0x1FFF
        elif op in ('ISUB', 'ICMP'): lsu_imm = v & 0x7F
    c = ins.ctrl
    if c['target'] is not None:
        if c['target'] not in labels: raise AsmError("undefined label '%s'" % c['target'])
        lsu_imm = labels[c['target']] & 0x1FF
        if labels[c['target']] > 511: raise AsmError('branch target beyond 511')
    lsu = (lsu_op << 20) | (lsu_reg << 13) | lsu_imm
    ctrl = (c['end'] << 8) | (c['boundary'] << 7) | (c['branch'] << 6) | (c['bcond'] << 5) | \
           (c['bsense'] << 4) | (c['loopdec'] << 3) | (c['loopbr'] << 2) | (c['idx1'] << 1) | c['idx0']
    return (mul << 73) | (add << 49) | (ds << 33) | (lsu << 9) | ctrl

def decode_fields(w):
    """word -> dict of raw fields"""
    return dict(
        mul_op=(w >> 94) & 3, mul_ra=(w >> 87) & 127, mul_rb=(w >> 80) & 127, mul_rd=(w >> 73) & 127,
        add_op=(w >> 70) & 7, add_ra=(w >> 63) & 127, add_rb=(w >> 56) & 127, add_rd=(w >> 49) & 127,
        ds_op=(w >> 47) & 3, ds_ra=(w >> 40) & 127, ds_rb=(w >> 33) & 127,
        lsu_op=(w >> 29) & 15, lsu_reg=(w >> 22) & 127, lsu_imm=(w >> 9) & 0x1FFF,
        end=(w >> 8) & 1, boundary=(w >> 7) & 1, branch=(w >> 6) & 1, bcond=(w >> 5) & 1,
        bsense=(w >> 4) & 1, loopdec=(w >> 3) & 1, loopbr=(w >> 2) & 1, idx_mode=w & 3)

def sx(v, bits):
    return v - (1 << bits) if v & (1 << (bits - 1)) else v

def disasm(w):
    """96-bit word -> canonical source text (re-assemblable)"""
    f = decode_fields(w)
    r = lambda n: 'r%d' % n
    s_mul = 'NOP'
    if f['mul_op'] == 1: s_mul = 'MUL %s, %s, %s' % (r(f['mul_rd']), r(f['mul_ra']), r(f['mul_rb']))
    elif f['mul_op']: s_mul = '.reserved_mul%d' % f['mul_op']
    s_add = 'NOP'
    if w >> 49 & 0xFFFFFF:
        n = ADD_NAMES[f['add_op']]
        short = {'FADD': 'ADD', 'FSUB': 'SUB', 'FMIN': 'MIN', 'FMAX': 'MAX', 'FABS': 'ABS', 'FNEG': 'NEG', 'FMOV': 'MOV'}
        if n in ADD_UNARY: s_add = '%s %s, %s' % (short[n], r(f['add_rd']), r(f['add_ra']))
        elif n == 'FCMP': s_add = 'CMP %s, %s, %s' % (r(f['add_rd']), r(f['add_ra']), r(f['add_rb']))
        else: s_add = '%s %s, %s, %s' % (short[n], r(f['add_rd']), r(f['add_ra']), r(f['add_rb']))
    s_ds = 'NOP'
    if f['ds_op'] == 1: s_ds = 'DIV %s, %s, %s' % (r(f['lsu_reg']), r(f['ds_ra']), r(f['ds_rb']))
    elif f['ds_op'] == 2: s_ds = 'SQRT %s, %s' % (r(f['lsu_reg']), r(f['ds_ra']))
    elif f['ds_op']: s_ds = '.reserved_ds3'
    s_lsu = 'NOP'
    op = f['lsu_op']; rg = f['lsu_reg']; im = f['lsu_imm']
    branching = f['branch'] or f['loopbr']
    idxs = ('+idx0' if f['idx_mode'] == 1 else '+idx1' if f['idx_mode'] == 2 else '') 
    if f['ds_op'] or branching: s_lsu = 'NOP'
    elif op == 1: s_lsu = 'LOAD %s, [%d%s]' % (r(rg), im & 0xFFF, idxs)
    elif op == 2: s_lsu = 'STORE %s, [%d%s]' % (r(rg), im & 0xFFF, idxs)
    elif op == 3: s_lsu = 'LOAD %s, [CONST:%d]' % (r(rg), im & 0x3F)
    elif op == 4: s_lsu = 'IADD %s, %d' % (r(rg), sx(im, 13))
    elif op == 5: s_lsu = 'ISUB %s, %s' % (r(rg), r(im & 127))
    elif op == 6: s_lsu = 'ICMP %s, %s' % (r(rg), r(im & 127))
    elif op == 7: s_lsu = 'ITOF %s' % r(rg)
    elif op == 8: s_lsu = 'LDLOOP %s' % r(rg)
    elif op == 9: s_lsu = 'LDIDX0 %s' % r(rg)
    elif op == 10: s_lsu = 'LDIDX1 %s' % r(rg)
    elif op == 11: s_lsu = 'IMOV %s, %d' % (r(rg), sx(im, 13))
    elif op: s_lsu = '.reserved_lsu%d' % op
    parts = []
    if f['end']: parts.append('END_UNIT')
    if f['boundary']: parts.append('BOUNDARY')
    tgt = '@%d' % (im & 0x1FF)
    if f['branch']: parts.append(('BRA' if not f['bcond'] else 'BLT' if f['bsense'] else 'BGE') + ' ' + tgt)
    if f['loopdec']: parts.append('LOOP_DEC')
    if f['loopbr']: parts.append('LOOP_BRA ' + tgt)
    if (f['idx_mode'] & 1) and not (op in (1, 2) and idxs == '+idx0'): parts.append('IDX0_INC')
    if (f['idx_mode'] & 2) and not (op in (1, 2) and idxs == '+idx1'): parts.append('IDX1_INC')
    s_ctrl = ' + '.join(parts) if parts else 'NOP'
    return ' | '.join([s_mul, s_add, s_ds, s_lsu, s_ctrl])

# ----------------------------------------------------------------------------
# CFG dataflow: hazards, ports, DS exclusivity, definedness
# ----------------------------------------------------------------------------
class St:
    """abstract state at instruction entry (offsets relative to 'now')"""
    __slots__ = ('pend', 'p1', 'ds', 'cmp', 'loop', 'idx0', 'idx1', 'defd')
    def __init__(self):
        self.pend = {}    # reg -> {off: (line, srcname)}   pending write lands at end of cycle now+off
        self.p1 = {}      # off -> (line, srcname)          port-1 write lands at end of cycle now+off
        self.ds = 0       # cycles until DS may start
        self.cmp = 0; self.loop = 0; self.idx0 = 0; self.idx1 = 0
        self.defd = None  # set of defined regs (None = unknown/top)
    def copy(self):
        s = St()
        s.pend = {r: dict(d) for r, d in self.pend.items()}
        s.p1 = dict(self.p1); s.ds = self.ds; s.cmp = self.cmp; s.loop = self.loop
        s.idx0 = self.idx0; s.idx1 = self.idx1
        s.defd = None if self.defd is None else set(self.defd)
        return s
    def tick(self, n):
        s = St()
        for r, d in self.pend.items():
            nd = {o - n: v for o, v in d.items() if o - n >= 0}
            if nd: s.pend[r] = nd
        s.p1 = {o - n: v for o, v in self.p1.items() if o - n >= 0}
        s.ds = max(0, self.ds - n); s.cmp = max(0, self.cmp - n); s.loop = max(0, self.loop - n)
        s.idx0 = max(0, self.idx0 - n); s.idx1 = max(0, self.idx1 - n)
        s.defd = None if self.defd is None else set(self.defd)
        return s
    def merge(self, o):
        """merge o into self; return True if self changed"""
        ch = False
        for r, d in o.pend.items():
            sd = self.pend.setdefault(r, {})
            for off, v in d.items():
                if off not in sd: sd[off] = v; ch = True
        for off, v in o.p1.items():
            if off not in self.p1: self.p1[off] = v; ch = True
        for a in ('ds', 'cmp', 'loop', 'idx0', 'idx1'):
            if getattr(o, a) > getattr(self, a): setattr(self, a, getattr(o, a)); ch = True
        if self.defd is None:
            if o.defd is not None: self.defd = set(o.defd); ch = True
        elif o.defd is not None:
            nd = self.defd & o.defd
            if nd != self.defd: self.defd = nd; ch = True
        return ch

def analyze(P):
    """CFG fixpoint; fills P.diags with hazard/port/definedness errors, returns reg stats"""
    n = len(P.ins)
    if n == 0: return
    roots = {0} | {a for a in P.entries.values() if a is not None}
    IN = [None] * n
    livein = set(P.livein) | {R_IDX1, R_IDX0, R_ZERO}
    work = []
    for a in roots:
        s = St(); s.defd = set(livein); IN[a] = s; work.append(a)

    def step(i, st, report):
        ins = P.ins[i]
        wh = ins.where()
        def err(m):
            if report: P.diags.append(Diag('ERROR', wh, m))
        def warn(m):
            if report: P.diags.append(Diag('WARNING', wh, m))
        p01, p23 = reads_of(ins)
        # RAW / undefined
        for r in set(p01 + p23):
            if r in (R_ZERO, R_IDX0, R_IDX1): continue
            if r in st.pend:
                off = min(st.pend[r]); mx = max(st.pend[r])
                ln, src = st.pend[r][mx]
                err('RAW: r%d read at cycle C but %s (line %d) result not readable for %d more cycle(s)' % (r, src, ln, mx + 1))
            if st.defd is not None and r not in st.defd:
                err('r%d read but never written on some path (missing .livein?)' % r)
        c = ins.ctrl
        if ins.ds and st.ds > 0: err('DS exclusivity: DIV/SQRT issued while the divider is busy (%d more cycle(s))' % st.ds)
        if (c['branch'] and c['bcond']) and st.cmp > 0: err('conditional branch %d cycle(s) too soon after CMP (A-CMP: >= %d)' % (st.cmp, CMP_TO_BRANCH))
        if (c['loopdec'] or c['loopbr']) and st.loop > 0: err('loop_dec/loop_branch too soon after LDLOOP')
        if ins.lsu and ins.lsu[0] in ('LOAD', 'STORE'):
            if ins.lsu[3] and st.idx0 > 0: err('indexed access too soon after LDIDX0')
            if ins.lsu[4] and st.idx1 > 0: err('indexed access too soon after LDIDX1')
        ns = st.tick(1)      # state after this cycle (used for successors), before adding own writes
        # writes are relative to 'now'; after tick they are offset-1
        tmp = st.copy()
        for reg, W, port, src in writes_of(ins):
            if port == 1:
                if W in tmp.p1:
                    ln, s2 = tmp.p1[W]
                    err('port-1 conflict: %s and %s (line %d) both write port 1 at the same cycle (W2)' % (src, s2, ln))
                tmp.p1[W] = (ins.line, src)
            if reg == R_ZERO: continue
            d = tmp.pend.setdefault(reg, {})
            if W in d: err('WAW: r%d written twice on the same cycle (%s and %s)' % (reg, src, d[W][1]))
            elif d and W < max(d): warn('WAW order: %s to r%d lands before an earlier-issued write' % (src, reg))
            d[W] = (ins.line, src)
            if tmp.defd is not None: tmp.defd.add(reg)
        if ins.ds: tmp.ds = DS_BUSY[ins.ds[0]]
        if ins.add and ins.add[0] == 'FCMP': tmp.cmp = CMP_TO_BRANCH
        if ins.lsu:
            op = ins.lsu[0]
            if op == 'ICMP': tmp.cmp = CMP_TO_BRANCH
            if op == 'LDLOOP': tmp.loop = 1
            if op == 'LDIDX0': tmp.idx0 = 1
            if op == 'LDIDX1': tmp.idx1 = 1
        return tmp

    def successors(i, ins):
        c = ins.ctrl
        if c['end']: return []
        tgt = P.labels.get(c['target']) if c['target'] else None
        res = []
        if c['branch'] and not c['bcond']:
            if tgt is not None: res.append((tgt, 1 + BRANCH_BUBBLE))
            return res
        if i + 1 < n: res.append((i + 1, 1))
        if (c['branch'] or c['loopbr']) and tgt is not None: res.append((tgt, 1 + BRANCH_BUBBLE))
        return res

    it = 0
    while work and it < 200000:
        it += 1
        i = work.pop()
        out = step(i, IN[i], False)
        for (j, dt) in successors(i, P.ins[i]):
            s = out.tick(dt)
            if IN[j] is None: IN[j] = s; work.append(j)
            elif IN[j].merge(s): work.append(j)
    # final reporting pass
    reach = 0
    for i in range(n):
        if IN[i] is None:
            P.diags.append(Diag('WARNING', P.ins[i].where(), 'unreachable instruction'))
            continue
        reach += 1
        step(i, IN[i], True)

def reg_report(P):
    """unique destinations and peak live (CFG liveness, def at issue)"""
    n = len(P.ins)
    uses, defs = [], []
    for ins in P.ins:
        p01, p23 = reads_of(ins)
        u = {r for r in p01 + p23 if r < R_IDX1}
        d = {w[0] for w in writes_of(ins) if w[0] < R_IDX1}
        uses.append(u); defs.append(d)
    succ = []
    for i, ins in enumerate(P.ins):
        c = ins.ctrl; s = []
        tgt = P.labels.get(c['target']) if c['target'] else None
        if c['end']: pass
        elif c['branch'] and not c['bcond']:
            if tgt is not None: s.append(tgt)
        else:
            if i + 1 < n: s.append(i + 1)
            if (c['branch'] or c['loopbr']) and tgt is not None: s.append(tgt)
        succ.append(s)
    live_in = [set() for _ in range(n)]
    ch = True
    while ch:
        ch = False
        for i in range(n - 1, -1, -1):
            lo = set()
            for j in succ[i]: lo |= live_in[j]
            li = uses[i] | (lo - defs[i])
            if li != live_in[i]: live_in[i] = li; ch = True
    peak = 0
    for i in range(n):
        lo = set()
        for j in succ[i]: lo |= live_in[j]
        peak = max(peak, len(live_in[i] | (lo & defs[i]) | defs[i]))
    dests = set().union(*defs) if defs else set()
    return len(dests), peak

def check_pressure(peak, limit=125):
    return ['register pressure: peak live %d exceeds %d' % (peak, limit)] if peak > limit else []

# ----------------------------------------------------------------------------
# Public assemble API
# ----------------------------------------------------------------------------
def assemble_sources(sources):
    P = parse_program(sources)
    if not any(d.sev == 'ERROR' for d in P.diags if d.where != 'program'):
        analyze(P)
    P.uniq_dests, P.peak_live = reg_report(P)
    for m in check_pressure(P.peak_live): P.diags.append(Diag('ERROR', 'program', m))
    return P

def assemble_files(paths):
    srcs = []
    for p in paths:
        with open(p) as f: srcs.append((p, f.read()))
    return assemble_sources(srcs)

def assemble_text(text, name='<test>'):
    return assemble_sources([(name, text)])

def words_of(P):
    return [i.word for i in P.ins]

def pack_image(words, vmajor=1, vminor=0):
    payload = bytearray()
    for i in range(UCODE_WORDS):
        w = words[i] if i < len(words) else 0
        payload += struct.pack('<III', w & 0xFFFFFFFF, (w >> 32) & 0xFFFFFFFF, (w >> 64) & 0xFFFFFFFF)
    crc = zlib.crc32(bytes(payload)) & 0xFFFFFFFF
    return struct.pack('<IHHII', MAGIC, vmajor, vminor, crc, len(payload)) + bytes(payload)

def unpack_image(img):
    magic, vmaj, vmin, crc, ln = struct.unpack('<IHHII', img[:16])
    if magic != MAGIC: raise ValueError('bad magic')
    payload = img[16:]
    if ln != 6144 or len(payload) != 6144: raise ValueError('bad length')
    if zlib.crc32(payload) & 0xFFFFFFFF != crc: raise ValueError('bad CRC')
    words = []
    for i in range(UCODE_WORDS):
        a, b, c = struct.unpack('<III', payload[12 * i:12 * i + 12])
        words.append(a | (b << 32) | (c << 64))
    return words

def write_listing(P, path):
    with open(path, 'w') as f:
        inv = {}
        for l, a in P.labels.items(): inv.setdefault(a, []).append(l)
        for ins in P.ins:
            for l in inv.get(ins.addr, []): f.write('%s:\n' % l)
            f.write('%03d  %024x  %s\n' % (ins.addr, ins.word, ins.text))
        for d in P.diags: f.write('// %s\n' % d)

def write_map(P, path):
    with open(path, 'w') as f:
        f.write('# label -> word address ( * = .entry )\n')
        for l, a in sorted(P.labels.items(), key=lambda kv: kv[1]):
            f.write('%-24s %3d %s\n' % (l, a, '*' if l in P.entries else ''))

def summary(P, src, out):
    nerr = sum(1 for d in P.diags if d.sev == 'ERROR')
    nwarn = sum(1 for d in P.diags if d.sev == 'WARNING')
    hz = sum(1 for d in P.diags if d.sev == 'ERROR' and ('RAW' in d.msg or 'WAW' in d.msg or 'DS excl' in d.msg or 'too soon' in d.msg))
    pc = sum(1 for d in P.diags if d.sev == 'ERROR' and ('port' in d.msg.lower() or d.msg[:2] in ('R1', 'R2', 'D1', 'B1')))
    n = len(P.ins)
    return ('=== TIDE Microcode Assembly Report ===\n'
            'Source:    %s\nOutput:    %s\nWords:     %d / %d used (%.1f%%)\n'
            'Registers: %d unique destinations, peak %d live\nHazards:   %d errors\nPorts:     %d conflicts\n'
            'Errors:    %d   Warnings: %d' % (src, out, n, UCODE_WORDS, 100.0 * n / UCODE_WORDS,
                                            P.uniq_dests, P.peak_live, hz, pc, nerr, nwarn))

# ----------------------------------------------------------------------------
# Self test
# ----------------------------------------------------------------------------
def _expect(name, text, want_err=None, want_words=None):
    P = assemble_text(text)
    errs = [d for d in P.diags if d.sev == 'ERROR']
    ok = True
    if want_err is None:
        ok = not errs
        if not ok: print('    unexpected:', '; '.join(map(str, errs)))
    else:
        ok = any(want_err.lower() in d.msg.lower() for d in errs)
        if not ok: print('    wanted error containing %r, got: %s' % (want_err, '; '.join(map(str, errs)) or 'none'))
    if ok and want_words is not None:
        got = words_of(P)
        if got != want_words:
            ok = False; print('    encoding mismatch: got %s want %s' % (['%024x' % g for g in got], ['%024x' % w for w in want_words]))
    print('  [%s] %s' % ('PASS' if ok else 'FAIL', name))
    return ok

def selftest():
    ok = True
    print('tide_asm self-test')
    # ---- encodings (doc 16 examples, with the isa.yaml opcode map) ----
    E = lambda mul=0, add=0, ds=0, lsu=0, ctrl=0: (mul << 73) | (add << 49) | (ds << 33) | (lsu << 9) | ctrl
    ok &= _expect('enc: FMUL r4,r7->r12 (doc16 ex.1)', '.livein r0-r124\nMUL r12, r4, r7', None,
                  [E(mul=(1 << 21) | (4 << 14) | (7 << 7) | 12)])
    ok &= _expect('enc: FMUL || FADD parallel (doc16 ex.2, FADD=000)', '.livein r0-r124\nMUL r12, r4, r7 | ADD r20, r10, r11', None,
                  [E(mul=(1 << 21) | (4 << 14) | (7 << 7) | 12, add=(0 << 21) | (10 << 14) | (11 << 7) | 20)])
    ok &= _expect('enc: FDIV r15,r3,r5 (dest in LSU reg field, doc16 ex.3)', '.livein r0-r124\nDIV r15, r3, r5', None,
                  [E(ds=(1 << 14) | (3 << 7) | 5, lsu=(0 << 20) | (15 << 13))])
    ok &= _expect('enc: hw loop (doc16 ex.4)', 'top: NOP\nNOP | NOP | NOP | NOP | LOOP_DEC + LOOP_BRA top', None,
                  [0, E(lsu=0, ctrl=0b000001100)])
    ok &= _expect('enc: all-zero word == NOP in every slot', 'NOP', None, [0])
    ok &= _expect('enc: SQRT / STORE idx0 / LOAD_ROM / IMOV', '.livein r0-r124\nSQRT r9, r2\nNOP | NOP | NOP | STORE r5, [16+idx0]\nNOP | NOP | NOP | LOAD r6, [CONST:7]\nNOP | NOP | NOP | IMOV r117, -3',
                  None, [E(ds=(2 << 14) | (2 << 7) | 127, lsu=(9 << 13)),
                         E(lsu=(2 << 20) | (5 << 13) | 16, ctrl=1),
                         E(lsu=(3 << 20) | (6 << 13) | 7),
                         E(lsu=(11 << 20) | (117 << 13) | (-3 & 0x1FFF))])
    # ---- hazards ----
    ok &= _expect('hazard: MUL result read one cycle early', 'MUL r5, r1, r2\nNOP | ADD r6, r5, r3', 'RAW')
    ok &= _expect('hazard: MUL result read at C+3 is legal (defined inputs via livein)', '.livein r1-r3\nMUL r5, r1, r2\nNOP\nNOP\nNOP | ADD r6, r5, r3', None)
    ok &= _expect('hazard: ADD result read at C+3 is illegal', '.livein r1-r3\nNOP | ADD r5, r1, r2\nNOP\nNOP\nNOP | ADD r6, r5, r3', 'RAW')
    ok &= _expect('hazard: ADD result read at C+4 is legal', '.livein r1-r3\nNOP | ADD r5, r1, r2\nNOP\nNOP\nNOP\nNOP | ADD r6, r5, r3', None)
    ok &= _expect('hazard: bypass result read at C+2 legal, C+1 illegal', '.livein r1-r3\nNOP | ABS r5, r1\nNOP | ADD r6, r5, r3', 'RAW')
    ok &= _expect('hazard: undefined register read', 'NOP | ADD r6, r5, r3', 'never written')
    ok &= _expect('hazard: DIV result at C+28 illegal / C+29 legal',
                  '.livein r1-r2\nDIV r5, r1, r2\n' + 'NOP\n' * 27 + 'NOP | ADD r6, r5, r2', 'RAW')
    ok &= _expect('hazard: DIV result at C+29 legal',
                  '.livein r1-r2\nDIV r5, r1, r2\n' + 'NOP\n' * 28 + 'NOP | ADD r6, r5, r2', None)
    ok &= _expect('hazard: second DIV while busy', '.livein r1-r2\nDIV r5, r1, r2\nNOP\nDIV r6, r1, r2', 'DS exclusivity')
    ok &= _expect('hazard: DIV then DIV at +29 legal', '.livein r1-r2\nDIV r5, r1, r2\n' + 'NOP\n' * 28 + 'DIV r6, r1, r2', None)
    ok &= _expect('hazard: SQRT then DIV at +28 legal', '.livein r1-r2\nSQRT r5, r1\n' + 'NOP\n' * 27 + 'DIV r6, r1, r2', None)
    ok &= _expect('hazard: SQRT then DIV at +27 illegal', '.livein r1-r2\nSQRT r5, r1\n' + 'NOP\n' * 26 + 'DIV r6, r1, r2', 'DS exclusivity')
    # ---- ports ----
    ok &= _expect('port: ADD pipeline (C+3) vs bypass (C+1) collide', '.livein r1-r3\nNOP | ADD r5, r1, r2\nNOP\nNOP | MOV r6, r3', 'port-1 conflict')
    ok &= _expect('port: DS done vs ADD pipeline collide', '.livein r1-r3\nDIV r5, r1, r2\n' + 'NOP\n' * 24 + 'NOP | ADD r6, r1, r2', 'port-1 conflict')
    ok &= _expect('port: same-cycle FADD + FMOV distinct cycles ok', '.livein r1-r3\nNOP | ADD r5, r1, r2\nNOP | MOV r6, r3', None)
    ok &= _expect('port: R2 binary ADD + STORE', '.livein r1-r4\nNOP | ADD r5, r1, r2 | NOP | STORE r3, [5]', 'R2')
    ok &= _expect('port: unary ABS + STORE ok', '.livein r1-r4\nNOP | ABS r5, r1 | NOP | STORE r3, [5]', None)
    ok &= _expect('port: LOAD alongside binary ADD ok', '.livein r1-r4\nNOP | ADD r5, r1, r2 | NOP | LOAD r3, [5]', None)
    ok &= _expect('port: R1 MUL + DS distinct sources', '.livein r1-r4\nMUL r5, r1, r2 | NOP | DIV r6, r3, r4', 'R1')
    ok &= _expect('rule: D1 DS + active LSU op', '.livein r1-r4\nNOP | NOP | DIV r6, r3, r4 | IMOV r7, 1', 'D1')
    ok &= _expect('rule: B1 branch + LSU op', 'x: NOP | NOP | NOP | IMOV r7, 1 | BRA x', 'B1')
    ok &= _expect('rule: END_UNIT + BOUNDARY', 'NOP | NOP | NOP | NOP | END_UNIT + BOUNDARY', 'mutually exclusive')
    ok &= _expect('rule: BEQ rejected (no eq latch)', 'x: NOP | NOP | NOP | NOP | BEQ x', 'BEQ')
    ok &= _expect('rule: ADD r0,r0,r0 aliases NOP', 'ADD r0, r0, r0', 'reserved')
    ok &= _expect('rule: undefined label', 'NOP | NOP | NOP | NOP | BRA nowhere', 'undefined label')
    ok &= _expect('rule: imm out of range', 'IMOV r1, 5000', 'out of range')
    ok &= _expect('rule: write to idx0 as destination', '.livein r1-r2\nMUL r126, r1, r2', 'index register')
    ok &= _expect('rule: conditional branch too soon after CMP', '.livein r1-r2\nx: NOP | CMP r1, r2\nNOP | NOP | NOP | NOP | BLT x', 'too soon')
    ok &= _expect('rule: conditional branch 2 cycles after CMP ok', '.livein r1-r2\nx: NOP | CMP r1, r2\nNOP\nNOP | NOP | NOP | NOP | BLT x', None)
    ok &= _expect('rule: loop_dec right after LDLOOP ok / same cycle n/a', '.livein r1\nx: NOP | NOP | NOP | LDLOOP r1\nNOP | NOP | NOP | NOP | LOOP_DEC + LOOP_BRA x', None)
    ok &= _expect('cfg: hazard across back-edge is caught',
                  '.livein r1-r3\nx: NOP | ADD r5, r1, r2\nNOP | NOP | NOP | NOP | LOOP_DEC + LOOP_BRA x\nNOP | ADD r6, r5, r3\n', 'RAW')
    ok &= _expect('cfg: taken-branch bubble counted (result lands during bubble)',
                  '.livein r1-r3\nMUL r5, r1, r2 | NOP | NOP | NOP | BRA t\nNOP\nt: NOP | ADD r6, r5, r3', 'RAW')
    ok &= _expect('cfg: MUL result across taken branch legal when >=3 cycles', '.livein r1-r3\nMUL r5, r1, r2 | NOP | NOP | NOP | BRA t\nNOP\nt: NOP\nNOP | ADD r6, r5, r3', None)
    ok &= _expect('cfg: memory word budget', '\n'.join(['NOP'] * 513), '513 words')
    # ---- pressure ----
    pr = check_pressure(126)
    print('  [%s] pressure: peak 126 flagged as error' % ('PASS' if pr else 'FAIL')); ok &= bool(pr)
    # ---- round trip ----
    src = ('.livein r1-r9, r15, r16\nMUL r10, r1, r2 | ADD r11, r3, r4 | NOP | LOAD r12, [40+idx1] | NOP\n'
           'NOP | CMP r13, r3, r4 | NOP | STORE r3, [7+idx0]\nNOP\nNOP | ABS r14, r3 | NOP | ITOF r15\n'
           'SQRT r20, r1\n' + 'NOP\n' * 28 + 'NOP | NOP | NOP | IADD r16, -7 | IDX0_INC\nNOP\nNOP | NOP | NOP | LDIDX1 r16\n'
           'NOP | NOP | NOP | NOP | END_UNIT\n')
    P = assemble_text(src)
    words = words_of(P)
    P2 = assemble_text('.livein r1-r9, r15, r16\n' + '\n'.join(disasm(w) for w in words))  # no branches here
    rt = [i.word for i in P2.ins] == words and not [d for d in P.diags if d.sev == 'ERROR']
    print('  [%s] round trip: assemble -> disassemble -> assemble (%d words)' % ('PASS' if rt else 'FAIL', len(words))); ok &= rt
    # ---- image ----
    img = pack_image(words)
    ok_img = len(img) == 6160 and unpack_image(img)[:len(words)] == words and img[:4] == struct.pack('<I', MAGIC)
    print('  [%s] image: 6160 bytes, magic/CRC/length, round trip' % ('PASS' if ok_img else 'FAIL')); ok &= ok_img
    # ---- contract cross-check (isa.yaml opcode tables) ----
    here = os.path.dirname(os.path.abspath(__file__))
    yml = os.path.join(here, '..', 'shared', 'contract', 'isa.yaml')
    if os.path.exists(yml):
        t = open(yml).read()
        good = True
        for sect, tab in (('add_ops', ADD_OPS), ('ds_ops', DS_OPS), ('lsu_alu_ops', LSU_OPS)):
            body = t.split(sect + ':')[1].split('\n\n')[0]
            for m in re.finditer(r'op: 0b([01]+), mnemonic: (\w+)', body):
                v, name = int(m.group(1), 2), m.group(2)
                name = {'NOP': 'NOP'}.get(name, name)
                if name.startswith('RSVD'): continue
                if tab.get(name) != v: good = False; print('    contract mismatch:', sect, name, v, tab.get(name))
        print('  [%s] contract: opcode tables match isa.yaml' % ('PASS' if good else 'FAIL')); ok &= good
    else:
        print('  [SKIP] contract cross-check (isa.yaml not found)')
    # ---- real microcode, if present ----
    mc = sorted(glob.glob(os.path.join(here, '..', 'microcode', '*.tasm')))
    if mc:
        P = assemble_files(mc)
        errs = [d for d in P.diags if d.sev == 'ERROR']
        print(summary(P, ' '.join(os.path.basename(m) for m in mc), '(self-test)'))
        for d in errs[:10]: print('   ', d)
        good = not errs and len(P.ins) <= UCODE_WORDS
        print('  [%s] microcode/*.tasm assembles clean (%d words, <= 512)' % ('PASS' if good else 'FAIL', len(P.ins))); ok &= good
    else:
        print('  [SKIP] no microcode/*.tasm yet')
    print('tide_asm self-test: %s' % ('PASS' if ok else 'FAIL'))
    return ok

def main(argv=None):
    ap = argparse.ArgumentParser(description='TIDE microcode assembler')
    ap.add_argument('files', nargs='*')
    ap.add_argument('-o', '--output')
    ap.add_argument('--lst'); ap.add_argument('--map')
    ap.add_argument('--test', action='store_true')
    ap.add_argument('-q', '--quiet', action='store_true')
    a = ap.parse_args(argv)
    if a.test:
        return 0 if selftest() else 1
    if not a.files:
        ap.error('no input files')
    files = []
    for f in a.files: files += sorted(glob.glob(f)) or [f]
    P = assemble_files(files)
    out = a.output or 'tide_ucode.bin'
    for d in P.diags:
        if not (a.quiet and d.sev == 'WARNING'): print(d, file=sys.stderr)
    if any(d.sev == 'ERROR' for d in P.diags):
        print(summary(P, ' '.join(files), '(none)')); return 1
    with open(out, 'wb') as f: f.write(pack_image(words_of(P)))
    if a.lst: write_listing(P, a.lst)
    if a.map: write_map(P, a.map)
    print(summary(P, ' '.join(files), out))
    return 0

if __name__ == '__main__':
    sys.exit(main())
