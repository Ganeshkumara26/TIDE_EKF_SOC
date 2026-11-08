#!/usr/bin/env python3
"""gen_ucode.py -- generate TIDE microcode (.tasm) from dataflow descriptions.

The routines below mirror golden/tide_core.c operation-for-operation (same
association order, same constants).  A resource-aware list scheduler places
the operations into 5-slot VLIW instructions honouring every rule of
doc 15/16 (latencies, read ports R1/R2, write-port-1 W2, DS exclusivity D1,
branch rule B1); a register allocator maps values to r0..r124 (in-place
precolored x/P); idle DIV/SQRT waits are compressed into delay loops.
The generated files are then verified INDEPENDENTLY by tools/tide_asm.py
(CFG hazard analysis) and tools/tide_sim.py (strict cycle-accurate model
+ bit-exact comparison with the C golden model).

    python3 microcode/gen_ucode.py            # writes microcode/*.tasm
"""
import os, sys, collections

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'tools'))
import tide_abi as ABI

# ROM indices (golden/tide_core.h)
ROM_PI, ROM_PI_2, ROM_TWO_PI, ROM_NEG_PI, ROM_ONE, ROM_ZERO = 0, 1, 2, 3, 4, 5
ROM_US_TO_S, ROM_TAN_PI_8, ROM_PI_4 = 7, 8, 10
ROM_H0, ROM_H1, ROM_H2, ROM_H3 = 11, 12, 13, 14

# registers with global roles
XP_BASE = 90                     # x_prior copy r90..r95 (live across the scalar loop)
R_LOOPCNT = 116                  # holds the scalar-loop count 4
R_DELAY = 117                    # delay-loop counter staging
RESERVED = {ABI.R_EVENT_T, ABI.R_CTX, R_LOOPCNT, R_DELAY} | set(range(XP_BASE, XP_BASE + 6)) | {125, 126, 127}
POOL_ORDER = list(range(37, 90)) + list(range(96, 116)) + [123, 124] + list(range(29, 37)) + \
             [118, 120, 121, 122] + list(range(1, 28))
# r0 is deliberately NEVER used as a pool register: memmap.yaml's register_file
# table suggests r0 may carry t_last if a hardware context-burst preload (see
# tide_lsu.sv burst_* ports) lands words 0..27 into r0..r27 before start_i.
# Our microcode never needs t_last as a value (dt_us is precomputed, A-ABI) and
# never relies on r0, so reserving it costs nothing and removes any risk of
# clobbering a TC-provided value there (A-BURST, ASSUMPTIONS.md).

# ---- op kinds -----------------------------------------------------------------
# kind: (slot, W = write offset, port)
KINDS = {
    'fmul': ('mul', 2, 0), 'fadd': ('add', 3, 1), 'fsub': ('add', 3, 1),
    'fneg': ('add', 1, 1), 'fabs': ('add', 1, 1), 'fmov': ('add', 1, 1),
    'fmax': ('add', 1, 1), 'fmin': ('add', 1, 1), 'fcmp': ('add', 1, 1),
    'fdiv': ('ds', 28, 1), 'fsqrt': ('ds', 27, 1),
    'rom': ('lsu', 1, 2), 'load': ('lsu', 1, 2), 'imov': ('lsu', 1, 2), 'itof': ('lsu', 1, 2),
    'store': ('lsu', 0, None), 'ldidx0': ('lsu', 0, None), 'ldidx1': ('lsu', 0, None),
}
DS_BUSY = {'fdiv': 29, 'fsqrt': 28}
UNARY = {'fneg', 'fabs', 'fmov', 'fsqrt', 'itof', 'store', 'ldidx0', 'ldidx1'}

class Val:
    __slots__ = ('name', 'fixed', 'prod', 'cons', 'liveout', 'web', 'reg', 'start', 'end')
    def __init__(self, name, fixed=None):
        self.name, self.fixed, self.prod, self.cons = name, fixed, None, []
        self.liveout = False; self.web = self; self.reg = None; self.start = self.end = None

class Op:
    __slots__ = ('id', 'kind', 'srcs', 'dst', 'imm', 'extra', 'preds', 'succs', 'prio', 'issue', 'inplace', 'release', 'tag')
    def __init__(self, id_, kind, srcs, dst, imm=None):
        self.id, self.kind, self.srcs, self.dst, self.imm = id_, kind, srcs, dst, imm
        self.extra = []; self.preds = []; self.succs = []; self.prio = 0
        self.issue = None; self.inplace = False; self.release = 0; self.tag = 0

class Prog:
    def __init__(self, name):
        self.name, self.ops, self.vals = name, [], []
        self.by_reg = collections.defaultdict(list)     # fixed reg -> [Val] in definition order
        self.zero = self._val('zero', fixed=ABI.R_ZERO)
    def _val(self, name, fixed=None):
        v = Val(name, fixed); self.vals.append(v)
        if fixed is not None: self.by_reg[fixed].append(v)
        return v
    def livein(self, reg, name=None):
        return self._val(name or 'in_r%d' % reg, fixed=reg)
    def op(self, kind, srcs, name=None, fixed=None, imm=None, inplace=False, dst=True):
        o = Op(len(self.ops), kind, list(srcs), None, imm)
        if dst:
            o.dst = self._val(name or '%s%d' % (kind, o.id), fixed)
            o.dst.prod = o
        for s in o.srcs: s.cons.append(o)
        o.inplace = inplace
        if inplace:
            assert len(o.srcs) == 1 and len(o.srcs[0].cons) >= 1
        self.ops.append(o)
        return o.dst if dst else o
    # arithmetic DSL
    def mul(s, a, b, **k): return s.op('fmul', [a, b], **k)
    def add(s, a, b, **k): return s.op('fadd', [a, b], **k)
    def sub(s, a, b, **k): return s.op('fsub', [a, b], **k)
    def neg(s, a, **k): return s.op('fneg', [a], **k)
    def mov(s, a, **k): return s.op('fmov', [a], **k)
    def fmax(s, a, b, **k): return s.op('fmax', [a, b], **k)
    def fmin(s, a, b, **k): return s.op('fmin', [a, b], **k)
    def abs(s, a, **k): return s.op('fabs', [a], **k)
    def div(s, a, b, **k): return s.op('fdiv', [a, b], **k)
    def sqrt(s, a, **k): return s.op('fsqrt', [a], **k)
    def rom(s, idx, **k): return s.op('rom', [], imm=idx, **k)
    def lt(s, a, b):
        """1.0f if a<b else 0.0f : FCMP (int flag) then in-place ITOF"""
        flag = s.op('fcmp', [a, b], name='flag')
        return s.op('itof', [flag], name='sel', inplace=True)
    def itof_inplace(s, a, **k): return s.op('itof', [a], inplace=True, **k)
    def load(s, addr, idx=None, **k): return s.op('load', [], imm=(addr, idx), **k)
    def store(s, v, addr, idx=None): return s.op('store', [v], imm=(addr, idx), dst=False)

# ---------------------------------------------------------------------------
# Scheduler
# ---------------------------------------------------------------------------
class Cycle:
    def __init__(self): self.mul = self.add = self.ds = self.lsu = None; self.p01 = set(); self.p23 = set()

def ready_delta(k):
    slot, W, port = KINDS[k]
    return W + 1 if port is not None else 0

def schedule(P, max_cycles=2000):
    ops = P.ops
    # --- edges
    def edge(a, b, d):
        b.preds.append((a, d)); a.succs.append((b, d))
    for o in ops:
        for s in o.srcs:
            if s.prod is not None: edge(s.prod, o, ready_delta(s.prod.kind))
        for (a, d) in o.extra: edge(a, o, d)
    # WAR / WAW edges for fixed-register redefinitions
    for reg, vs in P.by_reg.items():
        if reg in (ABI.R_ZERO,): continue
        for old, new in zip(vs, vs[1:]):
            if new.prod is None: continue
            for c in old.cons:
                if c is not new.prod: edge(c, new.prod, 0)
            if old.prod is not None and not old.cons:
                edge(old.prod, new.prod, 2)
            if old.prod is not None:      # new write must land after the old one
                edge(old.prod, new.prod, max(1, KINDS[old.prod.kind][1] - KINDS[new.prod.kind][1] + 1))
    # --- priorities (longest latency-weighted path to a sink)
    for o in reversed(ops):
        o.prio = max([d + s.prio for (s, d) in o.succs], default=0)
        if not o.succs: o.prio = max(1, ready_delta(o.kind))
    # --- ASAP for release times of source-like ops (rom/imov): just-in-time constants
    asap = {}
    for o in ops:
        asap[o.id] = max([asap[p.id] + d for (p, d) in o.preds], default=0)
    for o in ops:
        if o.kind in ('rom', 'imov') and not o.preds:
            best = None
            for (c, d) in o.succs:
                others = [asap[p.id] + dd for (p, dd) in c.preds if p is not o]
                t = max(others, default=0) - d - 3
                best = t if best is None else min(best, t)
            o.release = max(0, best or 0)
    # --- list scheduling
    remaining = list(ops)
    p1_busy = set()
    ds_free = 0
    t = 0
    instrs = []
    while remaining:
        if t > max_cycles: raise RuntimeError('%s: scheduler runaway (deadlock?) with %d ops left, e.g. %s' % (P.name, len(remaining), remaining[0].kind))
        cyc = Cycle()
        cand = [o for o in remaining if o.release <= t and all(p.issue is not None and p.issue + d <= t for (p, d) in o.preds)]
        cand.sort(key=lambda o: (-o.prio, o.id))
        for o in cand:
            slot, W, port = KINDS[o.kind]
            if getattr(cyc, slot) is not None: continue
            reads = [s for s in o.srcs if s.fixed != ABI.R_ZERO or True]
            regset = set(id(s) if s.fixed is None else ('r', s.fixed) for s in reads)
            if slot == 'mul': grp = 'p01'
            elif slot == 'ds': grp = 'p01'
            elif slot == 'add': grp = 'p23'
            else: grp = 'p23' if reads else None
            if grp:
                cur = getattr(cyc, grp)
                if len(cur | regset) > 2: continue
            if slot == 'ds':
                if t < ds_free or cyc.lsu is not None: continue
            if slot == 'lsu' and cyc.ds is not None: continue
            if port == 1 and (t + W) in p1_busy: continue
            # accept
            setattr(cyc, slot, o)
            if grp: getattr(cyc, grp).update(regset)
            if port == 1: p1_busy.add(t + W)
            if slot == 'ds':
                ds_free = t + DS_BUSY[o.kind]
                cyc.lsu = o                      # D1: LSU slot consumed by the DS destination field
            o.issue = t
            remaining.remove(o)
        instrs.append(cyc)
        t += 1
    return instrs

# ---------------------------------------------------------------------------
# Register allocation
# ---------------------------------------------------------------------------
def find(v):
    while v.web is not v:
        v.web = v.web.web; v = v.web
    return v

def allocate(P, instrs, t_end, extra_reserved=(), pool=None):
    ops = P.ops
    # merge in-place webs
    for o in ops:
        if o.inplace:
            src, dst = o.srcs[0], o.dst
            if len(src.cons) != 1: raise RuntimeError('in-place op on a value with multiple consumers: %s' % src.name)
            rs, rd_ = find(src), find(dst)
            if rd_.fixed is not None and rs.fixed is not None and rd_.fixed != rs.fixed:
                raise RuntimeError('in-place fixed-register conflict')
            if rs is not rd_:
                rd_.web = rs
                if rs.fixed is None and rd_.fixed is not None: rs.fixed = rd_.fixed
    # intervals per web
    webs = {}
    for v in P.vals:
        w = find(v)
        webs.setdefault(id(w), [w, None, None])
    for v in P.vals:
        w = find(v)
        rec = webs[id(w)]
        s = -1 if v.prod is None else v.prod.issue
        # a web's value is "written" from its producer issue; consumers extend the end
        e = s
        for c in v.cons: e = max(e, c.issue)
        if v.prod is not None: e = max(e, v.prod.issue + KINDS[v.prod.kind][1])
        if v.liveout: e = t_end + 1
        rec[1] = s if rec[1] is None else min(rec[1], s)
        rec[2] = e if rec[2] is None else max(rec[2], e)
    reserved = set(RESERVED) | set(extra_reserved)
    occ = collections.defaultdict(list)          # reg -> [(s, e)]
    for (w, s, e) in webs.values():
        if w.fixed is not None:
            occ[w.fixed].append((s, e, w))
    def conflict(reg, s, e):
        # Closed-interval (inclusive) overlap: a register may not be reused in
        # the SAME cycle another value is still being read (A-REUSE).  Although
        # a same-cycle old-read/new-write pattern is safe for a single physical
        # register file (read-before-write-lands), tools/tide_sim.py's strict
        # hazard model flags ANY read while ANY write to that register is still
        # in flight, so the allocator must not create that pattern either.
        for (s2, e2, _) in occ[reg]:
            if s <= e2 and s2 <= e: return True
        return False
    # fixed-vs-fixed sanity
    for reg, lst in occ.items():
        if reg == ABI.R_ZERO: continue
        lst.sort(key=lambda x: (x[0], x[1]))
        for (a, b) in zip(lst, lst[1:]):
            if a[1] > b[0] and not (a[2] is b[2]):
                raise RuntimeError('%s: fixed register r%d overlap: %s [%d,%d] vs %s [%d,%d]' % (P.name, reg, a[2].name, a[0], a[1], b[2].name, b[0], b[1]))
    pool = pool or POOL_ORDER
    order = sorted([x for x in webs.values() if x[0].fixed is None], key=lambda x: (x[1], x[2]))
    peak = 0
    for (w, s, e) in order:
        chosen = None
        best_end = -10**9
        for reg in pool:
            if reg in reserved: continue
            if conflict(reg, s, e): continue
            # best fit: prefer the register whose latest previous end is closest to s
            prev = max([e2 for (s2, e2, _) in occ[reg] if e2 <= s] or [-10**9])
            if chosen is None or prev > best_end: best_end, chosen = prev, reg
        if chosen is None:
            raise RuntimeError('%s: out of registers at cycle %d (web %s)' % (P.name, s, w.name))
        w.fixed = chosen
        occ[chosen].append((s, e, w))
    for v in P.vals:
        v.reg = find(v).fixed
    # peak concurrently-live GP registers
    events = collections.Counter()
    for (w, s, e) in webs.values():
        if w.fixed != ABI.R_ZERO:
            for c in range(max(s, 0), min(e, t_end) + 1): events[c] += 1
    return max(events.values()) if events else 0

# ---------------------------------------------------------------------------
# Emission
# ---------------------------------------------------------------------------
def R(v): return 'r%d' % v.reg

def slot_text(o):
    k = o.kind
    if k == 'fmul': return ('mul', 'MUL %s, %s, %s' % (R(o.dst), R(o.srcs[0]), R(o.srcs[1])))
    if k in ('fadd', 'fsub', 'fmax', 'fmin'):
        nm = {'fadd': 'ADD', 'fsub': 'SUB', 'fmax': 'MAX', 'fmin': 'MIN'}[k]
        return ('add', '%s %s, %s, %s' % (nm, R(o.dst), R(o.srcs[0]), R(o.srcs[1])))
    if k == 'fcmp': return ('add', 'CMP %s, %s, %s' % (R(o.dst), R(o.srcs[0]), R(o.srcs[1])))
    if k in ('fneg', 'fabs', 'fmov'):
        return ('add', '%s %s, %s' % ({'fneg': 'NEG', 'fabs': 'ABS', 'fmov': 'MOV'}[k], R(o.dst), R(o.srcs[0])))
    if k == 'fdiv': return ('ds', 'DIV %s, %s, %s' % (R(o.dst), R(o.srcs[0]), R(o.srcs[1])))
    if k == 'fsqrt': return ('ds', 'SQRT %s, %s' % (R(o.dst), R(o.srcs[0])))
    if k == 'rom': return ('lsu', 'LOAD %s, [CONST:%d]' % (R(o.dst), o.imm))
    if k == 'imov': return ('lsu', 'IMOV %s, %d' % (R(o.dst), o.imm))
    if k == 'itof': return ('lsu', 'ITOF %s' % R(o.dst))
    if k in ('load', 'store'):
        addr, idx = o.imm
        a = '%d' % addr + ('+%s' % idx if idx else '')
        if k == 'load': return ('lsu', 'LOAD %s, [%s]' % (R(o.dst), a))
        return ('lsu', 'STORE %s, [%s]' % (R(o.srcs[0]), a))
    if k in ('ldidx0', 'ldidx1'): return ('lsu', '%s %s' % (k.upper(), R(o.srcs[0])))
    raise RuntimeError(k)

def to_lines(instrs):
    """cycle list -> list of dicts {mul,add,ds,lsu,ctrl,note}"""
    out = []
    for i, cyc in enumerate(instrs):
        d = dict(mul=None, add=None, ds=None, lsu=None, ctrl=None, note='c%d' % i)
        for slot in ('mul', 'add', 'ds', 'lsu'):
            o = getattr(cyc, slot)
            if o is None: continue
            if slot == 'lsu' and o.kind in ('fdiv', 'fsqrt'): continue    # DS dest carried in the DS text
            s, txt = slot_text(o)
            d[s] = txt
        out.append(d)
    return out

def pad_to(lines, n):
    while len(lines) < n: lines.append(dict(mul=None, add=None, ds=None, lsu=None, ctrl=None, note='drain'))

def last_landing(instrs):
    L = 0
    for cyc in instrs:
        for slot in ('mul', 'add', 'ds', 'lsu'):
            o = getattr(cyc, slot)
            if o is not None and o.issue is not None and o.dst is not None:
                L = max(L, o.issue + KINDS[o.kind][1])
    return L

def is_empty(d):
    return not any(d[k] for k in ('mul', 'add', 'ds', 'lsu', 'ctrl'))

def compress_delays(lines, min_run=8):
    """replace runs of empty instructions with hardware delay loops (exact cycle count preserved)"""
    out = []; i = 0; nloops = 0
    while i < len(lines):
        if not is_empty(lines[i]): out.append(lines[i]); i += 1; continue
        j = i
        while j < len(lines) and is_empty(lines[j]): j += 1
        n = j - i
        if n < min_run:
            out.extend(lines[i:j]); i = j; continue
        # IMOV, NOP, LDLOOP, then loop of c executions (2c-1 cycles): total 2c+2 (+1 NOP if odd)
        c = (n - 2) // 2 if n % 2 == 0 else (n - 3) // 2
        nloops += 1
        lab = 'DLY%d' % nloops
        E = lambda **k: dict(mul=None, add=None, ds=None, lsu=None, ctrl=None, note='delay', **k)
        out.append(dict(mul=None, add=None, ds=None, lsu='IMOV r%d, %d' % (R_DELAY, c), ctrl=None, note='delay %d cycles' % n))
        out.append(E())
        out.append(dict(mul=None, add=None, ds=None, lsu='LDLOOP r%d' % R_DELAY, ctrl=None, note='delay'))
        out.append(dict(mul=None, add=None, ds=None, lsu=None, ctrl='LOOP_DEC + LOOP_BRA %s' % lab, note='delay', label=lab))
        if n % 2 == 1: out.append(E())
        i = j
    return out, nloops

def emit(lines, path, header, entry_labels=None, labels=None, livein=None):
    labels = labels or {}
    with open(path, 'w') as f:
        for h in header: f.write('// %s\n' % h)
        if livein: f.write('.livein %s\n' % livein)
        for name in (entry_labels or []): f.write('.entry %s\n' % name)
        for idx, d in enumerate(lines):
            if idx in labels: f.write('%s:\n' % labels[idx])
            if d.get('label'): f.write('%s:\n' % d['label'])
            slots = [d['mul'] or 'NOP', d['add'] or 'NOP', d['ds'] or 'NOP', d['lsu'] or 'NOP', d['ctrl'] or 'NOP']
            txt = ' | '.join(slots)
            f.write('    %-96s // %s\n' % (txt, d.get('note', '')))

# ---------------------------------------------------------------------------
# Routine builders (mirror golden/tide_core.c)
# ---------------------------------------------------------------------------
def pidx(i, j): return i * (i + 1) // 2 + j

def load_ctx_regs(P, first_word, last_word, base_reg_val):
    """chain of indexed LOADs of ctx words first..last into their contract registers r_word"""
    ld = P.op('ldidx0', [base_reg_val], dst=False)
    prev, vals = ld, {}
    for w in range(first_word, last_word + 1):
        v = P.load(first_word, 'idx0', fixed=w, name='ctx%d' % w)
        v.prod.extra.append((prev, 1))
        prev = v.prod
        vals[w] = v
    return vals

def atan_poly(P, t2, a0):
    z = P.mul(t2, t2)
    p = P.rom(ROM_H0)
    p = P.add(P.mul(p, z), P.rom(ROM_H1))
    p = P.add(P.mul(p, z), P.rom(ROM_H2))
    p = P.add(P.mul(p, z), P.rom(ROM_H3))
    p = P.mul(P.mul(p, z), t2)
    return P.add(P.add(p, t2), a0)

def atan_reduce_eval(P, t):
    s = P.lt(P.rom(ROM_TAN_PI_8), t)
    num = P.sub(t, s)
    den = P.add(P.rom(ROM_ONE), P.mul(s, t))
    t2 = P.div(num, den)
    a0 = P.mul(s, P.rom(ROM_PI_4))
    return atan_poly(P, t2, a0)

def oct_stage(P, s, K, a):
    c = P.mul(s, K)
    m = P.sub(P.rom(ROM_ONE), P.add(s, s))
    return P.add(c, P.mul(m, a))

def atan2_full(P, y, x):
    ax, ay = P.abs(x), P.abs(y)
    mx, mn = P.fmax(ax, ay), P.fmin(ax, ay)
    nz = P.lt(P.zero, mx)
    e = P.sub(P.rom(ROM_ONE), nz)
    mxg = P.add(mx, e)
    t = P.div(mn, mxg)
    a = atan_reduce_eval(P, t)
    s1 = P.lt(ax, ay)
    a = oct_stage(P, s1, P.rom(ROM_PI_2), a)
    s2 = P.lt(x, P.zero)
    a = oct_stage(P, s2, P.rom(ROM_PI), a)
    s3 = P.lt(y, P.zero)
    m3 = P.sub(P.rom(ROM_ONE), P.add(s3, s3))
    return P.mul(m3, a)

def atan2_xpos(P, y, x):
    ax, ay = P.abs(x), P.abs(y)
    mx, mn = P.fmax(ax, ay), P.fmin(ax, ay)
    t = P.div(mn, mx)
    a = atan_reduce_eval(P, t)
    s1 = P.lt(ax, ay)
    a = oct_stage(P, s1, P.rom(ROM_PI_2), a)
    s3 = P.lt(y, P.zero)
    m3 = P.sub(P.rom(ROM_ONE), P.add(s3, s3))
    return P.mul(m3, a)

def build_unit_pre():
    """CTX_LOAD + PREDICT + GEOM + row stores + x_prior copy + loop pointer setup"""
    P = Prog('unit_pre')
    r_ctx = P.livein(ABI.R_CTX)
    ctx = load_ctx_regs(P, 1, 27, r_ctx)
    X = [ctx[1 + i] for i in range(6)]
    Pold = [ctx[7 + k] for k in range(21)]
    Po = lambda i, j: Pold[pidx(i, j)] if i >= j else Pold[pidx(j, i)]

    # ---- PREDICT
    dt_int = P.livein(ABI.R_DT_US)
    dtf = P.itof_inplace(dt_int, fixed=ABI.R_DT_US)
    dt = P.mul(dtf, P.rom(ROM_US_TO_S))
    sc = P.mul(dt, P.livein(ABI.R_DT0INV))
    qp = P.mul(P.livein(ABI.R_QP), sc)
    qv = P.mul(P.livein(ABI.R_QV), sc)
    dt2 = P.mul(dt, dt)
    xn = list(X)
    for i in range(3):
        xn[i] = P.add(X[i], P.mul(X[i + 3], dt), fixed=1 + i, name='xn%d' % i)
    dPvv = {}
    for a in range(3):
        for b in range(a + 1):
            dPvv[(a, b)] = P.mul(dt, Po(3 + a, 3 + b))
    Pn = list(Pold)
    for a in range(3):
        for b in range(a + 1):
            s = P.add(Po(3 + a, b), Po(3 + b, a))
            t1 = P.mul(dt, s)
            t2 = P.mul(dt2, Po(3 + a, 3 + b))
            n = P.add(P.add(Po(a, b), t1), t2)
            if a == b:
                n = P.add(n, qp, fixed=ABI.R_PBASE + pidx(a, b), name='Pn')
            else:
                n.prod.dst = n     # keep
                n = _retarget(P, n, ABI.R_PBASE + pidx(a, b))
            Pn[pidx(a, b)] = n
    for a in range(3):
        for b in range(3):
            d = dPvv[(a, b)] if a >= b else dPvv[(b, a)]
            Pn[pidx(3 + a, b)] = P.add(Po(3 + a, b), d, fixed=ABI.R_PBASE + pidx(3 + a, b), name='Pcross')
    for a in range(3):
        Pn[pidx(3 + a, 3 + a)] = P.add(Po(3 + a, 3 + a), qv, fixed=ABI.R_PBASE + pidx(3 + a, 3 + a), name='Pvv')
    for v in xn: v.liveout = True
    for v in Pn: v.liveout = True

    # ---- GEOM
    pn, pe, pd, vn, ve, vd = xn
    pn2, pe2, pd2 = P.mul(pn, pn), P.mul(pe, pe), P.mul(pd, pd)
    rg2r = P.add(pn2, pe2)
    r2 = P.add(rg2r, pd2)
    r2c = P.fmax(r2, P.rom(ROM_US_TO_S))
    rg2c = P.fmax(rg2r, P.rom(ROM_US_TO_S))
    r = P.sqrt(r2c)
    rg = P.sqrt(rg2c)
    rrg = P.mul(r, rg)
    inv_rrg = P.div(P.rom(ROM_ONE), rrg)
    inv_r = P.mul(rg, inv_rrg)
    inv_rg = P.mul(r, inv_rrg)
    dv = P.add(P.add(P.mul(vn, pn), P.mul(ve, pe)), P.mul(vd, pd))
    dop = P.mul(dv, inv_r)
    az = atan2_full(P, pe, pn)
    el = atan2_xpos(P, P.neg(pd), rg)
    z = [P.livein(ABI.R_Z[k]) for k in range(4)]
    Rm = [P.livein(ABI.R_R[k]) for k in range(4)]
    y0 = P.sub(z[0], r)
    y1 = P.sub(z[1], az)
    shi = P.lt(P.rom(ROM_PI), y1)
    slo = P.lt(y1, P.rom(ROM_NEG_PI))
    y1 = P.add(P.sub(y1, P.mul(shi, P.rom(ROM_TWO_PI))), P.mul(slo, P.rom(ROM_TWO_PI)))
    y2 = P.sub(z[2], el)
    y3 = P.sub(z[3], dop)
    h00, h01, h02 = P.mul(pn, inv_r), P.mul(pe, inv_r), P.mul(pd, inv_r)
    inv_rg2 = P.mul(inv_rg, inv_rg)
    h10 = P.neg(P.mul(pe, inv_rg2))
    h11 = P.mul(pn, inv_rg2)
    inv_r2 = P.mul(inv_r, inv_r)
    inv_r2rg = P.mul(inv_r2, inv_rg)
    pdpn, pdpe = P.mul(pd, pn), P.mul(pd, pe)
    h20 = P.mul(pdpn, inv_r2rg)
    h21 = P.mul(pdpe, inv_r2rg)
    h22 = P.neg(P.mul(rg, inv_r2))
    hh = [h00, h01, h02]
    h3 = [P.mul(P.sub(v, P.mul(dop, hh[i])), inv_r) for i, v in enumerate((vn, ve, vd))]
    Z = P.zero
    rows = [[h00, h01, h02, Z, Z, Z, Rm[0], y0],
            [h10, h11, Z, Z, Z, Z, Rm[1], y1],
            [h20, h21, h22, Z, Z, Z, Rm[2], y2],
            [h3[0], h3[1], h3[2], h00, h01, h02, Rm[3], y3]]
    for k in range(4):
        for j in range(8):
            P.store(rows[k][j], ABI.RAM_ROWS + 8 * k + j)
    # ---- x_prior copies (live across the scalar loop)
    for i in range(6):
        v = P.mov(xn[i], fixed=XP_BASE + i, name='xp%d' % i); v.liveout = True
    # ---- scalar-loop pointers: idx1 = ROWS (post-incremented by the row loads)
    ri = P.op('imov', [], imm=ABI.RAM_ROWS, name='rowptr')
    li = P.op('ldidx1', [ri], dst=False)
    ri.prod.extra.append((li, 0)) if False else None
    ri.prod.tag = 1; li.tag = 1
    return P

def _retarget(P, v, reg):
    """force an existing value into a fixed register (used for non-diagonal Ppp entries)"""
    v.fixed = reg
    P.by_reg[reg].append(v)
    return v

def build_update_body():
    """one sequential scalar update (loop body).  x=r1..r6, P=r7..r27 updated in place."""
    P = Prog('update')
    X = [P.livein(1 + i, 'x%d' % i) for i in range(6)]
    Pold = [P.livein(ABI.R_PBASE + k, 'P%d' % k) for k in range(21)]
    XP = [P.livein(XP_BASE + i, 'xp%d' % i) for i in range(6)]
    Po = lambda i, j: Pold[pidx(i, j)] if i >= j else Pold[pidx(j, i)]
    # row loads (idx1 post-increment chain)
    ld = []
    prev = None
    for j in range(8):
        v = P.load(0, 'idx1', name='row%d' % j)
        if prev is not None: v.prod.extra.append((prev, 1))
        prev = v.prod
        ld.append(v)
    h, Rk, y0 = ld[:6], ld[6], ld[7]
    d = [P.sub(X[j], XP[j]) for j in range(6)]
    dot = P.add(P.mul(h[0], d[0]), P.mul(h[1], d[1]))
    for j in range(2, 6): dot = P.add(dot, P.mul(h[j], d[j]))
    y = P.sub(y0, dot)
    p = []
    for i in range(6):
        acc = P.add(P.mul(Po(i, 0), h[0]), P.mul(Po(i, 1), h[1]))
        for j in range(2, 6): acc = P.add(acc, P.mul(Po(i, j), h[j]))
        p.append(acc)
    hp = P.add(P.mul(h[0], p[0]), P.mul(h[1], p[1]))
    for i in range(2, 6): hp = P.add(hp, P.mul(h[i], p[i]))
    s = P.add(hp, Rk)
    inv = P.div(P.rom(ROM_ONE), s)
    K = [P.mul(p[i], inv) for i in range(6)]
    xnew = [P.add(X[i], P.mul(K[i], y), fixed=1 + i, name='xnew%d' % i) for i in range(6)]
    cc = [P.sub(p[i], P.mul(K[i], hp)) for i in range(6)]
    rk = [P.mul(Rk, K[i]) for i in range(6)]
    Pnew = []
    for i in range(6):
        for j in range(i + 1):
            u = P.sub(Po(i, j), P.mul(K[i], p[j]))
            w = P.sub(u, P.mul(K[j], cc[i]))
            Pnew.append(P.add(w, P.mul(rk[i], K[j]), fixed=ABI.R_PBASE + pidx(i, j), name='Pnew'))
    for v in xnew + Pnew: v.liveout = True
    for v in XP: v.liveout = True
    return P

def build_store():
    """CTX_STORE: word 0 <- r28 (event t), words 1..27 <- r1..r27, in place at the context base"""
    P = Prog('store')
    base = P.livein(ABI.R_CTX)
    t = P.livein(ABI.R_EVENT_T)
    ld = P.op('ldidx0', [base], dst=False)
    prev = ld
    srcs = [t] + [P.livein(w, 'ctxr%d' % w) for w in range(1, 28)]
    for w, v in enumerate(srcs):
        s = P.store(v, 0, 'idx0')
        s.extra.append((prev, 1)); prev = s
    return P

def build_query():
    P = Prog('query')
    base = P.livein(ABI.R_CTX)
    ctx = load_ctx_regs(P, 1, 6, base)
    X = [ctx[1 + i] for i in range(6)]
    dtf = P.itof_inplace(P.livein(ABI.R_DT_US), fixed=ABI.R_DT_US)
    dt = P.mul(dtf, P.rom(ROM_US_TO_S))
    xo = [P.add(X[i], P.mul(X[i + 3], dt)) for i in range(3)] + X[3:]
    P.store(P.livein(ABI.R_EVENT_T), ABI.RAM_QOUT)
    for i in range(6): P.store(xo[i], ABI.RAM_QOUT + 1 + i)
    return P

# ---------------------------------------------------------------------------
def finish_region(P, name, extra_pad=0):
    instrs = schedule(P)
    L = max(last_landing(instrs), len(instrs) - 1)
    t_end = L + extra_pad
    peak = allocate(P, instrs, t_end)
    return instrs, t_end, peak

def main():
    out = HERE
    info = {}
    # ---------------- region 1
    P1 = build_unit_pre()
    ins1, t_end1, peak1 = finish_region(P1, 'unit_pre')
    lines1 = to_lines(ins1)
    # place IMOV r116,4 into the first instruction with a free LSU slot; LDLOOP after the last op
    for d in lines1[1:]:
        if d['lsu'] is None and d['ds'] is None:
            d['lsu'] = 'IMOV r%d, 4' % R_LOOPCNT; break
    n_ops_end = len(lines1)
    pad_to(lines1, t_end1 + 1)
    # LDLOOP: needs a free LSU slot at/after the last delay loop; use the last drain-pad instruction
    tail = lines1[-1]
    if tail['lsu'] is not None or tail['ds'] is not None:
        pad_to(lines1, len(lines1) + 1); tail = lines1[-1]
    tail['lsu'] = 'LDLOOP r%d' % R_LOOPCNT
    cyc1 = len(lines1)
    lines1c, nd = compress_delays(lines1)
    emit(lines1c, os.path.join(out, '10_unit_pre.tasm'),
         ['TIDE microcode: UNIT prologue = CTX_LOAD + PREDICT + GEOM + row stores (generated by gen_ucode.py)',
          'Mirrors golden/tide_core.c tide_unit()/tide_predict()/tide_geom().  DO NOT EDIT BY HAND.',
          'Region cycles: %d, words after delay-loop compression: %d (%d delay loops)' % (cyc1, len(lines1c), nd)],
         entry_labels=['UNIT_ENTRY'], labels={0: 'UNIT_ENTRY'},
         livein='r28-r36, r118-r122, r119')
    info['pre'] = (cyc1, len(lines1c), peak1, len(P1.ops))
    # ---------------- region 2 (loop body)
    P2 = build_update_body()
    ins2, t_end2, peak2 = finish_region(P2, 'update')
    lines2 = to_lines(ins2)
    pad_to(lines2, t_end2 + 1)
    last = lines2[-1]
    if last['lsu'] is not None or last['ds'] is not None:
        pad_to(lines2, len(lines2) + 1); last = lines2[-1]
    last['ctrl'] = 'LOOP_DEC + LOOP_BRA SCALAR_LOOP'
    cyc2 = len(lines2)
    info['loop'] = (cyc2, len(lines2), peak2, len(P2.ops))
    # ---------------- region 3
    P3 = build_store()
    ins3, t_end3, peak3 = finish_region(P3, 'store')
    lines3 = to_lines(ins3)
    pad_to(lines3, t_end3 + 1)
    lines3[-1]['ctrl'] = 'END_UNIT'
    info['store'] = (len(lines3), len(lines3), peak3, len(P3.ops))
    lines23 = lines2 + lines3
    emit(lines23, os.path.join(out, '20_unit_update.tasm'),
         ['TIDE microcode: UNIT scalar-update loop (x4) + CTX_STORE (generated by gen_ucode.py)',
          'Loop body %d cycles x 4 iterations; CTX_STORE %d cycles.  DO NOT EDIT BY HAND.' % (cyc2, len(lines3))],
         labels={0: 'SCALAR_LOOP', len(lines2): 'CTX_STORE'})
    # ---------------- query
    PQ = build_query()
    insq, t_endq, peakq = finish_region(PQ, 'query')
    linesq = to_lines(insq)
    pad_to(linesq, t_endq + 1)
    if linesq[-1]['lsu'] is not None: pad_to(linesq, len(linesq) + 1)
    linesq[-1]['ctrl'] = 'END_UNIT'
    emit(linesq, os.path.join(out, '90_query.tasm'),
         ['TIDE microcode: QUERY (predict-only, non-destructive) (generated by gen_ucode.py)',
          'Writes t_q,x0..x5 to TIDE-RAM QUERY_OUT.  DO NOT EDIT BY HAND.'],
         entry_labels=['QUERY_ENTRY'], labels={0: 'QUERY_ENTRY'}, livein='r28-r36, r118-r122, r119')
    info['query'] = (len(linesq), len(linesq), peakq, len(PQ.ops))
    total_words = len(lines1c) + len(lines23) + len(linesq)
    est_cycles = cyc1 + 4 * cyc2 + len(lines3)
    print('region          cycles  words  peak-live  ops')
    for k, (c, w, pk, n) in info.items():
        print('%-14s %7d %6d %10d %5d' % (k, c, w, pk, n))
    print('total words %d / 512;  estimated UNIT = %d + 4x%d + %d = %d cycles (+3 seq)' % (total_words, cyc1, cyc2, len(lines3), est_cycles))

if __name__ == '__main__':
    main()
