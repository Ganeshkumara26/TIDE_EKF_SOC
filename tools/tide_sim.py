#!/usr/bin/env python3
"""tide_sim.py -- cycle-accurate TIDE microcode simulator (gate G1, Agent 1).

    tide_sim.py --test                      full G1 acceptance run
    tide_sim.py --asm a.tasm b.tasm --vectors golden/vectors/unit_vectors.txt
    tide_sim.py --bin tide_ucode.bin --vectors ...

Timing model (Step 0 / doc 16): an op issued in cycle C reads the register
file in cycle C and lands at the END of cycle C+W (MUL 2, ADD/SUB 3, bypass 1,
LOAD/ROM/IntALU 1, DIV 28, SQRT 27).  A same-cycle read returns the old value.
The simulator is STRICT: reading a register with a pending write (stale
read), two writes to one port or register in one cycle, DS start while busy,
and consuming a poison (0xFFFFFFFF, never-written) value are fatal errors.
A taken branch costs one bubble cycle.  Python 3.8+, standard library only.
"""
import sys, os, re, math, struct, argparse, glob, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tide_asm as A
import tide_abi as ABI

QNAN = 0x7FC00000
POISON = ABI.POISON

# ----------------------------------------------------------------------------
# IEEE-754 binary32 lane models (doc 16 / doc 19 semantics)
# ----------------------------------------------------------------------------
_pk, _upk = struct.Struct('<f').pack, struct.Struct('<I').unpack
_pkI, _upkf = struct.Struct('<I').pack, struct.Struct('<f').unpack

def u2f(u):
    return _upkf(_pkI(u & 0xFFFFFFFF))[0]

def f2u(x):
    """python float (double) -> binary32 bits, RNE, overflow -> +-inf, NaN -> canonical"""
    if x != x:
        return QNAN
    try:
        return _upk(_pk(x))[0]
    except OverflowError:
        return 0xFF800000 if x < 0 else 0x7F800000

def isnan(u):
    return (u & 0x7F800000) == 0x7F800000 and (u & 0x007FFFFF) != 0

def fadd(a, b): return f2u(u2f(a) + u2f(b))
def fsub(a, b): return f2u(u2f(a) - u2f(b))
def fmul(a, b): return f2u(u2f(a) * u2f(b))

def fdiv(a, b):
    x, y = u2f(a), u2f(b)
    if x != x or y != y: return QNAN
    if y == 0.0:
        if x == 0.0: return QNAN
        return 0xFF800000 if ((a ^ b) >> 31) else 0x7F800000
    return f2u(x / y)

def fsqrt(a):
    x = u2f(a)
    if x != x or x < 0.0: return QNAN
    return f2u(math.sqrt(x))

def fabs_(a): return QNAN if isnan(a) else a & 0x7FFFFFFF
def fneg_(a): return 0xFFC00000 if isnan(a) else a ^ 0x80000000
def flt(a, b): return 1 if u2f(a) < u2f(b) else 0        # ordered less-than

def fmax_(a, b):
    na, nb = isnan(a), isnan(b)
    if na and nb: return QNAN
    if na: return b
    if nb: return a
    return b if u2f(a) < u2f(b) else a

def fmin_(a, b):
    na, nb = isnan(a), isnan(b)
    if na and nb: return QNAN
    if na: return b
    if nb: return a
    return b if u2f(b) < u2f(a) else a

def s32(u):
    u &= 0xFFFFFFFF
    return u - (1 << 32) if u & 0x80000000 else u

def itof(a): return f2u(float(s32(a)))

LANE_OPS = {'add': fadd, 'sub': fsub, 'mul': fmul, 'div': fdiv, 'sqrt': lambda a, b: fsqrt(a),
            'fmin': fmin_, 'fmax': fmax_, 'fabs': lambda a, b: fabs_(a), 'fneg': lambda a, b: fneg_(a),
            'lt': flt, 'itof': lambda a, b: itof(a)}

# ----------------------------------------------------------------------------
class SimError(Exception):
    pass

class RunResult:
    def __init__(self):
        self.core_cycles = 0      # issue cycles + drain
        self.cycles = 0           # + sequencer start/done latency
        self.instrs = 0
        self.mul = self.add = self.ds = self.lsu = 0
        self.snap = None

class TideSim:
    def __init__(self, words, rom, labels=None, listing=None):
        self.words = words
        self.dec = [A.decode_fields(w) for w in words]
        self.rom = list(rom) + [0] * (64 - len(rom))
        self.labels = labels or {}
        self.listing = listing or {}
        self.reset()

    def reset(self):
        self.rf = [POISON] * 128
        self.ram = [POISON] * 1024
        self.idx0 = self.idx1 = 0
        self.loop = 0
        self.cmp_lt = 0

    # -- helpers -------------------------------------------------------------
    def _err(self, msg):
        pc = self.pc
        txt = self.listing.get(pc, '')
        raise SimError('cycle %d pc %d: %s\n      %s' % (self.t, pc, msg, txt))

    def _rd(self, r):
        if r == 127: return 0
        if r == 126: return self.idx0
        if r == 125: return self.idx1
        if self.pend.get(r):
            self._err('stale read of r%d (write still in flight)' % r)
        v = self.rf[r]
        if v == POISON:
            self._err('poison (uninitialised) r%d consumed' % r)
        return v

    def _sched(self, cyc, port, reg, val, tag):
        self.wq.setdefault(cyc, []).append((port, reg, val, tag))
        if reg != 127:
            self.pend[reg] = self.pend.get(reg, 0) + 1
        self.inflight += 1

    def _land(self, cyc):
        lst = self.wq.pop(cyc, None)
        if not lst: return
        ports, regs = set(), set()
        for port, reg, val, tag in lst:
            if port in ports: self._err('two port-%d writes land in cycle %d' % (port, cyc))
            ports.add(port)
            if reg != 127:
                if reg in regs: self._err('two writes to r%d land in cycle %d' % (reg, cyc))
                regs.add(reg)
                self.rf[reg] = val
                self.pend[reg] -= 1
            self.inflight -= 1

    # -- run -------------------------------------------------------------------
    def run(self, entry, max_cycles=20000, stage_pc=None):
        res = RunResult()
        self.pc = entry
        self.t = 0
        self.wq, self.pend, self.inflight = {}, {}, 0
        self.ds_busy_until = -1
        end_t = None
        last_land = -1
        ds_pending_regs = 0
        while True:
            if self.t > max_cycles: self._err('cycle limit (runaway program?)')
            if self.pc >= len(self.dec): self._err('pc ran off the end of the program')
            if stage_pc is not None and self.pc == stage_pc and res.snap is None:
                res.snap = (list(self.rf), list(self.ram), self.t)
            f = self.dec[self.pc]
            t = self.t
            res.instrs += 1
            rd = self._rd
            ports_written = []
            # ---- MUL
            if f['mul_op'] == 1:
                a, b = rd(f['mul_ra']), rd(f['mul_rb'])
                self._sched(t + 2, 0, f['mul_rd'], fmul(a, b), 'FMUL'); res.mul += 1
            elif f['mul_op']: self._err('reserved MUL opcode')
            # ---- ADD (all-zero slot = NOP, A-ADDNOP)
            if (self.words[self.pc] >> 49) & 0xFFFFFF:
                op = f['add_op']; ra = f['add_ra']; rb = f['add_rb']; rdst = f['add_rd']
                a = rd(ra)
                b = rd(rb) if op in (0, 1, 2, 3, 4) else 0
                res.add += 1
                if op == 0: self._sched(t + 3, 1, rdst, fadd(a, b), 'FADD')
                elif op == 1: self._sched(t + 3, 1, rdst, fsub(a, b), 'FSUB')
                elif op == 2:
                    c = flt(a, b); self.pending_cmp = (t + 1, c)
                    self._sched(t + 1, 1, rdst, c, 'FCMP')
                elif op == 3: self._sched(t + 1, 1, rdst, fmin_(a, b), 'FMIN')
                elif op == 4: self._sched(t + 1, 1, rdst, fmax_(a, b), 'FMAX')
                elif op == 5: self._sched(t + 1, 1, rdst, fabs_(a), 'FABS')
                elif op == 6: self._sched(t + 1, 1, rdst, fneg_(a), 'FNEG')
                else: self._sched(t + 1, 1, rdst, a, 'FMOV')
            # ---- DS
            if f['ds_op']:
                if t <= self.ds_busy_until: self._err('DIV/SQRT started while the divider is busy')
                a = rd(f['ds_ra'])
                if f['ds_op'] == 1:
                    b = rd(f['ds_rb']); lat = 28; val = fdiv(a, b)
                elif f['ds_op'] == 2:
                    lat = 27; val = fsqrt(a)
                else: self._err('reserved DS opcode')
                if f['lsu_op']: self._err('D1 violated: LSU op with DS')
                self._sched(t + lat, 1, f['lsu_reg'], val, 'DS'); res.ds += 1
                self.ds_busy_until = t + lat
            # ---- LSU / ALU
            store = None
            new_idx0 = new_idx1 = None
            new_loop = None
            new_cmp = None
            op = f['lsu_op']
            branching = f['branch'] or f['loopbr']
            if op and not f['ds_op'] and not branching:
                reg, imm = f['lsu_reg'], f['lsu_imm']
                res.lsu += 1
                if op in (1, 2):
                    m = f['idx_mode']
                    if m == 3: self._err('indexed LOAD/STORE with idx_mode 11 is undefined')
                    addr = (imm & 0xFFF) + (self.idx0 if m == 1 else self.idx1 if m == 2 else 0)
                    if not 0 <= addr < 1024: self._err('TIDE-RAM address %d out of range' % addr)
                    if op == 1: self._sched(t + 1, 2, reg, self.ram[addr], 'LOAD')
                    else:
                        v = rd(reg)
                        store = (addr, v)
                elif op == 3: self._sched(t + 1, 2, reg, self.rom[imm & 63], 'ROM')
                elif op == 4: self._sched(t + 1, 2, reg, (rd(reg) + s32(A.sx(imm, 13))) & 0xFFFFFFFF, 'IADD')
                elif op == 5: self._sched(t + 1, 2, reg, (rd(reg) - rd(imm & 127)) & 0xFFFFFFFF, 'ISUB')
                elif op == 6: new_cmp = 1 if s32(rd(reg)) < s32(rd(imm & 127)) else 0
                elif op == 7: self._sched(t + 1, 2, reg, itof(rd(reg)), 'ITOF')
                elif op == 8: new_loop = rd(reg) & 0xFFFF
                elif op == 9: new_idx0 = rd(reg) & 0xFFF
                elif op == 10: new_idx1 = rd(reg) & 0xFFF
                elif op == 11: self._sched(t + 1, 2, reg, A.sx(imm, 13) & 0xFFFFFFFF, 'IMOV')
                else: self._err('reserved LSU opcode %d' % op)
            # ---- CTRL
            taken = False
            target = f['lsu_imm'] & 0x1FF
            if f['branch']:
                if not f['bcond']: taken = True
                else: taken = (self.cmp_lt == f['bsense'])
            loop_ctr = self.loop
            if f['loopdec']: loop_ctr = (loop_ctr - 1) & 0xFFFF
            if f['loopbr'] and loop_ctr > 0: taken = True
            if f['boundary']: self._err('BOUNDARY reached (not used by the unit microcode)')
            # ---- end of cycle: index post-inc, latches, RAM store, landings
            m = f['idx_mode']
            if m & 1: self.idx0 = (self.idx0 + 1) & 0xFFFF
            if m & 2: self.idx1 = (self.idx1 + 1) & 0xFFFF
            if new_idx0 is not None: self.idx0 = new_idx0
            if new_idx1 is not None: self.idx1 = new_idx1
            if new_loop is not None: self.loop = new_loop
            if f['loopdec']: self.loop = loop_ctr
            if store is not None: self.ram[store[0]] = store[1]
            self._land(t)
            # cmp latch: FCMP/ICMP result visible from t+1
            if new_cmp is not None: self.cmp_lt = new_cmp
            pc_cmp = getattr(self, 'pending_cmp', None)
            if pc_cmp and pc_cmp[0] <= t + 1 and f['add_op'] == 2 and (self.words[self.pc] >> 49) & 0xFFFFFF:
                self.cmp_lt = pc_cmp[1]; self.pending_cmp = None
            if f['end']:
                end_t = t
                # drain
                tt = t
                while self.inflight:
                    tt += 1
                    self._land(tt)
                    last_land = tt
                    if tt > t + 100: self._err('drain did not finish')
                res.core_cycles = max(end_t, last_land) + 1
                res.cycles = res.core_cycles + ABI.SEQ_START_LATENCY + ABI.SEQ_DONE_LATENCY
                return res
            if taken:
                self._land(t + 1)          # bubble cycle: pipelines keep running
                self.t = t + 2
                self.pc = target
            else:
                self.t = t + 1
                self.pc += 1

# ----------------------------------------------------------------------------
# Test-vector plumbing
# ----------------------------------------------------------------------------
def load_rom(path):
    with open(path) as f:
        return [int(l.strip(), 16) for l in f if l.strip()]

def rom_vs_contract(rom, memmap):
    """list of (addr, contract, final) differences vs memmap.yaml"""
    diffs = []
    if not os.path.exists(memmap): return diffs
    t = open(memmap).read()
    contract = {}
    for m in re.finditer(r'\{ addr: (\d+),\s+hex: (0x[0-9A-Fa-f]+)', t):
        contract[int(m.group(1))] = int(m.group(2), 16)
    for a in range(64):
        c = contract.get(a, 0)
        if c != rom[a]: diffs.append((a, c, rom[a]))
    return diffs

def parse_vectors(path):
    """-> list of scenarios {name, cfg, init, events[(ev, exp, stg)]} and list of queries"""
    scen, qry = [], []
    cfg = None; cur = None
    with open(path) as f:
        for line in f:
            w = line.split()
            if not w or w[0].startswith('#'): continue
            k = w[0]
            if k == 'BEGIN':
                cur = dict(name=w[1], cfg=cfg, init=None, events=[]); scen.append(cur); continue
            v = [int(x, 16) for x in w[1:]]
            if k == 'CFG': cfg = v
            elif k == 'INIT': cur['init'] = v
            elif k == 'EV': cur['events'].append([v, None, None])
            elif k == 'EXP': cur['events'][-1][1] = v
            elif k == 'STG': cur['events'][-1][2] = v
            elif k == 'QRY': qry.append(dict(cfg=cfg, ctx=v[:28], tq=v[28], exp=v[29:36]))
    return scen, qry

CTX_NAMES = ['t_last'] + ['x%d' % i for i in range(6)] + ['P%d' % i for i in range(21)]

def setup_tc(sim, ctx, ev, cfg, base=ABI.RAM_COMMITTED):
    """emulate the temporal controller: RAM context + event registers + ABI regs"""
    t_event, z, R, dt = ev[0], ev[1:5], ev[5:9], ev[9]
    sim.rf = [POISON] * 128
    for i in range(28): sim.ram[base + i] = ctx[i]
    sim.rf[ABI.R_EVENT_T] = t_event
    for i in range(4):
        sim.rf[ABI.R_Z[i]] = z[i]; sim.rf[ABI.R_R[i]] = R[i]
    sim.rf[ABI.R_DT_US] = dt
    sim.rf[ABI.R_CTX] = base
    sim.rf[ABI.R_QP], sim.rf[ABI.R_QV], sim.rf[ABI.R_DT0INV] = cfg
    sim.idx0 = sim.idx1 = 0; sim.loop = 0

def run_unit_vectors(sim, entry, stage_pc, scen, verbose=True):
    cyc = []
    nev = nerr = 0
    for sc in scen:
        ctx = list(sc['init'])
        for k, (ev, exp, stg) in enumerate(sc['events']):
            setup_tc(sim, ctx, ev, sc['cfg'])
            try:
                r = sim.run(entry, stage_pc=stage_pc)
            except SimError as e:
                print('  SIM ERROR in %s event %d: %s' % (sc['name'], k, e)); return None, 1, nev
            got = sim.ram[ABI.RAM_COMMITTED:ABI.RAM_COMMITTED + 28]
            nev += 1
            if got != exp:
                nerr += 1
                bad = [i for i in range(28) if got[i] != exp[i]]
                print('  MISMATCH %s event %d: %d of 28 ctx words differ; first: %s got %08x want %08x'
                      % (sc['name'], k, len(bad), CTX_NAMES[bad[0]], got[bad[0]], exp[bad[0]]))
                if r.snap is not None and stg is not None:
                    rf, ram, t0 = r.snap
                    xs = rf[1:7] + rf[7:28]
                    badx = [i for i in range(27) if xs[i] != stg[i]]
                    rows = ram[ABI.RAM_ROWS:ABI.RAM_ROWS + 32]
                    badr = [i for i in range(32) if rows[i] != stg[27 + i]]
                    print('    stage (loop entry): %d/27 x,P words differ, %d/32 row words differ' % (len(badx), len(badr)))
                    if badx: print('      first x/P word idx %d got %08x want %08x' % (badx[0], xs[badx[0]], stg[badx[0]]))
                    if badr: print('      first row word %d (row %d col %d) got %08x want %08x' % (badr[0], badr[0] // 8, badr[0] % 8, rows[badr[0]], stg[27 + badr[0]]))
                return cyc, nerr, nev
            if stg is not None and r.snap is not None:
                rf, ram, t0 = r.snap
                xs = rf[1:7] + rf[7:28]
                rows = ram[ABI.RAM_ROWS:ABI.RAM_ROWS + 32]
                if xs != stg[:27] or rows != stg[27:59]:
                    nerr += 1
                    print('  STAGE MISMATCH %s event %d at loop entry' % (sc['name'], k)); return cyc, nerr, nev
            cyc.append((r.cycles, r.core_cycles, r.instrs, r.mul, r.add, r.ds))
            ctx = got                       # cumulative: feed the simulator's own output forward
    return cyc, nerr, nev

def run_query_vectors(sim, entry, qry):
    nerr = 0
    for q in qry:
        ctx = q['ctx']; tq = q['tq']
        ev = [tq, 0, 0, 0, 0, 0, 0, 0, 0, (tq - ctx[0]) & 0xFFFFFFFF]
        sim.ram = [POISON] * 1024
        setup_tc(sim, ctx, ev, q['cfg'])
        for i in (29, 30, 31, 32, 33, 34, 35, 36): sim.rf[i] = POISON
        try: r = sim.run(entry)
        except SimError as e:
            print('  SIM ERROR in query: %s' % e); return 1
        got = sim.ram[ABI.RAM_QOUT:ABI.RAM_QOUT + 7]
        if got != q['exp']:
            nerr += 1; print('  QUERY MISMATCH got %s want %s' % (['%08x' % g for g in got], ['%08x' % g for g in q['exp']]))
    return nerr

# ----------------------------------------------------------------------------
def lane_selftest(opvec):
    print('[S1] FP lane models vs C golden primitives (%s)' % os.path.basename(opvec))
    n = bad = 0
    per = {}
    with open(opvec) as f:
        for line in f:
            nm, a, b, r = line.split()
            a, b, r = int(a, 16), int(b, 16), int(r, 16)
            g = LANE_OPS[nm](a, b)
            n += 1
            if g != r:
                bad += 1
                if bad <= 8: print('     MISMATCH %s %08x %08x: sim %08x C %08x' % (nm, a, b, g, r))
            per[nm] = per.get(nm, 0) + 1
    print('     %d vectors over %d ops, %d mismatches' % (n, len(per), bad))
    return bad == 0

def micro_selftest(rom):
    """directed cycle-timing tests from doc 21 sec 10"""
    print('[S2] directed timing tests (doc 21 sec 10)')
    ok = True
    def prog(text, setup=None):
        P = A.assemble_text('.livein r0-r124\n' + text)
        errs = [d for d in P.diags if d.sev == 'ERROR']
        if errs: raise SimError('test program does not assemble: %s' % errs[0])
        s = TideSim(A.words_of(P), rom)
        if setup: setup(s)
        return s, P
    def chk(name, cond, extra=''):
        nonlocal ok
        print('     [%s] %s %s' % ('PASS' if cond else 'FAIL', name, extra)); ok &= bool(cond)
    fl = lambda x: f2u(x)
    s, _ = prog('NOP | NOP | NOP | NOP | END_UNIT')
    r = s.run(0); chk('NOP program halts at END_UNIT', r.core_cycles == 1, '(core cycles %d)' % r.core_cycles)
    # MUL 3.0 x 4.0 -> 12.0, readable at C+3 (write end of C+2)
    s, _ = prog('MUL r10, r1, r2\nNOP\nNOP\nNOP | NOP | NOP | STORE r10, [3] | END_UNIT')
    s.rf[1], s.rf[2] = fl(3.0), fl(4.0); s.rf[127] = 0
    r = s.run(0); chk('MUL 3.0*4.0 = 12.0 (0x41400000) readable at C+3', s.ram[3] == 0x41400000, '(cycles %d)' % r.core_cycles)
    s, _ = prog('NOP | ADD r10, r1, r2\nNOP\nNOP\nNOP\nNOP | NOP | NOP | STORE r10, [3] | END_UNIT')
    s.rf[1], s.rf[2] = fl(1.0), fl(2.0)
    r = s.run(0); chk('ADD 1.0+2.0 = 3.0 (0x40400000) readable at C+4', s.ram[3] == 0x40400000)
    s, _ = prog('DIV r10, r1, r2\n' + 'NOP\n' * 28 + 'NOP | NOP | NOP | STORE r10, [3] | END_UNIT')
    s.rf[1], s.rf[2] = fl(1.0), fl(3.0)
    r = s.run(0); chk('DIV 1/3 = 0x3EAAAAAB readable at C+29', s.ram[3] == 0x3EAAAAAB, '(0x%08x)' % s.ram[3])
    s, _ = prog('SQRT r10, r1\n' + 'NOP\n' * 27 + 'NOP | NOP | NOP | STORE r10, [3] | END_UNIT')
    s.rf[1] = fl(2.0)
    r = s.run(0); chk('SQRT 2 = 0x3FB504F3 readable at C+28', s.ram[3] == 0x3FB504F3, '(0x%08x)' % s.ram[3])
    # strictness: stale read must be fatal
    s, _ = None, None
    P = A.assemble_text('.livein r0-r124\nMUL r10, r1, r2\nNOP | NOP | NOP | STORE r10, [3] | END_UNIT')   # assembler flags it; bypass assembler to test sim
    s = TideSim(A.words_of(P), rom); s.rf[1], s.rf[2] = fl(1.0), fl(2.0)
    try: s.run(0); chk('sim rejects stale read', False)
    except SimError: chk('sim rejects stale read (independent of assembler)', True)
    P = A.assemble_text('MUL r10, r1, r2\nNOP\nNOP\nNOP | NOP | NOP | STORE r10, [3] | END_UNIT')
    s = TideSim(A.words_of(P), rom)
    try: s.run(0); chk('sim rejects poison consumption', False)
    except SimError: chk('sim rejects poison consumption', True)
    # loop + bubble timing: 3 iterations of a self-loop = 2*3-1 = 5 cycles
    s, _ = prog('IMOV r5, 3\nNOP\nNOP | NOP | NOP | LDLOOP r5\nx: NOP | NOP | NOP | NOP | LOOP_DEC + LOOP_BRA x\nNOP | NOP | NOP | NOP | END_UNIT')
    r = s.run(0); chk('delay loop of count c takes 2c-1 cycles (c=3 -> 5)', r.core_cycles == 3 + 5 + 1, '(core %d)' % r.core_cycles)
    # FCMP / branch
    s, _ = prog('NOP | CMP r1, r2\nNOP\nNOP | NOP | NOP | NOP | BLT y\nNOP | NOP | NOP | IMOV r9, 7\ny: NOP | NOP | NOP | IMOV r8, 5\nNOP\nNOP | NOP | NOP | STORE r8, [1] | END_UNIT')
    s.rf[1], s.rf[2] = fl(1.0), fl(2.0)
    r = s.run(0); chk('BLT taken when a<b (skips IMOV r9)', s.ram[1] == 5 and s.rf[9] == POISON)
    return ok

def g1_test(verbose=True):
    root = os.path.abspath(os.path.join(HERE, '..'))
    vec_dir = os.path.join(root, 'golden', 'vectors')
    ok = True
    t0 = time.time()
    print('=== TIDE cycle-accurate simulator: G1 acceptance ===')
    opvec = os.path.join(vec_dir, 'op_vectors.txt')
    if not os.path.exists(opvec):
        print('missing %s -- run golden/build.sh first' % opvec); return False
    ok &= lane_selftest(opvec)
    rom = load_rom(os.path.join(vec_dir, 'rom_final.hex'))
    ok &= micro_selftest(rom)
    # ROM vs contract
    diffs = rom_vs_contract(rom, os.path.join(root, 'shared', 'contract', 'memmap.yaml'))
    print('[S3] ROM (golden-final) vs memmap.yaml contract: %d entr%s differ%s' % (
        len(diffs), 'y' if len(diffs) == 1 else 'ies', ''.join('\n     ROM[%d]: contract %08x -> final %08x' % d for d in diffs)))
    # assemble the microcode
    files = sorted(glob.glob(os.path.join(root, 'microcode', '*.tasm')))
    if not files:
        print('[S4] no microcode/*.tasm found'); return False
    P = A.assemble_files(files)
    errs = [d for d in P.diags if d.sev == 'ERROR']
    print('[S4] assemble microcode: %d words (%d/512), %d errors, %d warnings; regs: %d dests, peak live %d'
          % (len(P.ins), len(P.ins), len(errs), sum(1 for d in P.diags if d.sev == 'WARNING'), P.uniq_dests, P.peak_live))
    for d in errs[:10]: print('     ', d)
    if errs: return False
    # image round trip: run the *unpacked* image
    img = A.pack_image(A.words_of(P))
    words = A.unpack_image(img)
    print('[S5] image: %d bytes, CRC ok, entries: %s' % (len(img), ', '.join('%s=%d' % kv for kv in sorted(P.entries.items()))))
    listing = {i.addr: '%03d  %s' % (i.addr, i.text) for i in P.ins}
    sim = TideSim(words, rom, P.labels, listing)
    scen, qry = parse_vectors(os.path.join(vec_dir, 'unit_vectors.txt'))
    entry = P.entries['UNIT_ENTRY']
    stage = P.labels.get('SCALAR_LOOP')
    cyc, nerr, nev = run_unit_vectors(sim, entry, stage, scen)
    good = cyc is not None and nerr == 0
    print('[S6] unit vectors: %d events in %d scenarios, %s' % (nev, len(scen), 'all BIT-EXACT vs golden (28 ctx words each, cumulative)' if good else 'FAILED'))
    ok &= good
    qe = run_query_vectors(sim, P.entries['QUERY_ENTRY'], qry) if 'QUERY_ENTRY' in P.entries else 1
    print('[S7] query vectors: %d, %s' % (len(qry), 'BIT-EXACT' if qe == 0 else 'FAILED'))
    ok &= (qe == 0)
    if good:
        tot = [c[0] for c in cyc]; core = [c[1] for c in cyc]
        mx, mn = max(tot), min(tot)
        print('\n=== TIDE Cycle-Accurate Simulation Report ===')
        print('Microcode:  %s (%d words)' % (' '.join(os.path.basename(f) for f in files), len(P.ins)))
        print('Events:     %d (%d scenarios), data-independent timing: min %d / max %d cycles' % (nev, len(scen), mn, mx))
        print('UNIT core (issue+drain):      %d cycles' % max(core))
        print('UNIT total (+%d start/done):   %d cycles   <- MUST BE <= 1400' % (ABI.SEQ_START_LATENCY + ABI.SEQ_DONE_LATENCY, mx))
        print('Utilisation (per unit): %d MUL, %d ADD, %d DS issues over %d instructions executed' % (cyc[0][3], cyc[0][4], cyc[0][5], cyc[0][2]))
        limit_ok = mx <= 1400
        ok &= limit_ok
        print('Golden model comparison: ctx (t_last, x, P) after every unit: max |err| = 0.000000e+00 (BIT-EXACT)')
        print('\nG1 RESULT: %s -- UNIT <= 1400 cycles (%d), BIT-EXACT' % ('PASS' if ok else 'FAIL', mx) if limit_ok and ok
              else '\nG1 RESULT: FAIL (see above)')
    else:
        print('\nG1 RESULT: FAIL')
    print('(elapsed %.1f s)' % (time.time() - t0))
    return ok

def main(argv=None):
    ap = argparse.ArgumentParser(description='TIDE cycle-accurate simulator')
    ap.add_argument('--test', action='store_true')
    ap.add_argument('--asm', nargs='*'); ap.add_argument('--bin')
    ap.add_argument('--vectors')
    a = ap.parse_args(argv)
    if a.test:
        return 0 if g1_test() else 1
    root = os.path.abspath(os.path.join(HERE, '..'))
    vec = a.vectors or os.path.join(root, 'golden', 'vectors', 'unit_vectors.txt')
    rom = load_rom(os.path.join(os.path.dirname(vec), 'rom_final.hex'))
    if a.asm:
        P = A.assemble_files(a.asm)
        words, labels, entries = A.words_of(P), P.labels, P.entries
        listing = {i.addr: '%03d  %s' % (i.addr, i.text) for i in P.ins}
    elif a.bin:
        words = A.unpack_image(open(a.bin, 'rb').read()); labels, entries, listing = {}, {'UNIT_ENTRY': 0}, {}
    else:
        ap.error('need --asm or --bin (or --test)')
    sim = TideSim(words, rom, labels, listing)
    scen, qry = parse_vectors(vec)
    cyc, nerr, nev = run_unit_vectors(sim, entries['UNIT_ENTRY'], labels.get('SCALAR_LOOP'), scen)
    if cyc:
        print('%d events, cycles/unit %d..%d, mismatches %d' % (nev, min(c[0] for c in cyc), max(c[0] for c in cyc), nerr))
    return 0 if cyc and nerr == 0 else 1

if __name__ == '__main__':
    sys.exit(main())
