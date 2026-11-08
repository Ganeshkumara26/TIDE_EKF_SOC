# 16 — TIDE Micro-ISA / Encoding Specification

**Purpose:** The single authoritative definition of the 96-bit VLIW microinstruction encoding, every operation's semantics, latency, result-availability, bypass rules, write-conflict rules, and exception behavior. The assembler, simulator, sequencer RTL, and verification all derive from this document.

**If this document conflicts with docs 08 or 20 on encoding or semantics, this document wins. Doc 15 is supreme for interface timing.**

---

## 1. Instruction Word Format

```
┌──────────────────────────────────────────────────────────────────────────────────────────────────┐
│ 95  94 93  87 86  80 79  73 │ 72 70 69  63 62  56 55  49 │ 48 47 46  40 39  33 │ 32 29 28 22 21  9 │ 8  7  6  5  4  3  2  1  0 │
│        MUL SLOT (23)        │       ADD SLOT (24)        │   DS SLOT (16)      │  LSU/ALU SLOT (24) │    CTRL SLOT (9)          │
│ op  ra      rb      rd     │ op  ra      rb      rd     │ op  ra      rb      │ op   reg     imm   │ EU BC BR BS BI LD LB IDX  │
└──────────────────────────────────────────────────────────────────────────────────────────────────┘
```

### 1.1 Field Widths (frozen)

| Slot | Bits | Width | Fields |
|---|---|---|---|
| MUL | [95:73] | 23 | op(2) + ra(7) + rb(7) + rd(7) |
| ADD | [72:49] | 24 | op(3) + ra(7) + rb(7) + rd(7) |
| DS | [48:33] | 16 | op(2) + ra(7) + rb(7) |
| LSU/ALU | [32:9] | 24 | op(4) + reg(7) + imm(13) |
| CTRL | [8:0] | 9 | end(1) + boundary(1) + branch(1) + bcond(1) + bsense(1) + loopdec(1) + loopbr(1) + idx(2) |
| **Total** | | **96** | |

---

## 2. MUL Slot Operations

| op[1:0] | Mnemonic | Semantics | Latency | Result Available | Write Port |
|---|---|---|---|---|---|
| 2'b00 | NOP | No operation | — | — | — |
| 2'b01 | FMUL | rd ← ra × rb (FP32, RNE) | 2 | C+2 | Port 0, cycle C+2 |
| 2'b10 | reserved | — | — | — | — |
| 2'b11 | reserved | — | — | — | — |

### MUL Operand Rules
- `ra`, `rb`: Read from RF on cycle C (combinational read). Values must be stable from previous writes; the assembler ensures no RAW hazard.
- `rd`: Written on cycle C+2 via write port 0. **No other MUL instruction may target port 0 on the same cycle**.
- `ra = 7'h7F` (r127) → read as `+0.0` (hardwired to zero).
- `ra = 7'h7E` (r126) → substituted with index register 0 value.
- `ra = 7'h7D` (r125) → substituted with index register 1 value.
- Same substitution rules apply to `rb` and `rd`.

### MUL Exceptions
- NaN input → canonical qNaN output (`0x7FC00000`). No exception flag raised (quiet NaN propagation).
- Inf × 0 → qNaN + `invalid` flag in lane-internal flags register.
- Overflow → ±Inf, `overflow` + `inexact` flags.
- Underflow → ±0 or subnormal, `underflow` + `inexact` flags.
- Flags are **not architecturally visible** to microcode. They are captured by the sequencer for post-unit diagnostics only.

---

## 3. ADD Slot Operations

| op[2:0] | Mnemonic | Semantics | Latency | Result Available | Write Port |
|---|---|---|---|---|---|
| 3'b000 | FADD | rd ← ra + rb (FP32, RNE) | 3 | C+3 | Port 1, cycle C+3 |
| 3'b001 | FSUB | rd ← ra - rb (FP32, RNE) | 3 | C+3 | Port 1, cycle C+3 |
| 3'b010 | FCMP | Compare ra vs rb; set `cmp_lt` latch | 1 (bypass) | C+1 | Port 1, cycle C+1 (rd ← int comparison result) |
| 3'b011 | FMIN | rd ← min(ra, rb) | 1 (bypass) | C+1 | Port 1, cycle C+1 |
| 3'b100 | FMAX | rd ← max(ra, rb) | 1 (bypass) | C+1 | Port 1, cycle C+1 |
| 3'b101 | FABS | rd ← \|ra\| (clear sign bit) | 1 (bypass) | C+1 | Port 1, cycle C+1 |
| 3'b110 | FNEG | rd ← -ra (flip sign bit) | 1 (bypass) | C+1 | Port 1, cycle C+1 |
| 3'b111 | FMOV | rd ← ra | 1 (bypass) | C+1 | Port 1, cycle C+1 |

### ADD Pipeline vs Bypass

The ADD lane has two distinct paths:
1. **Pipeline path** (FADD, FSUB): 3-stage pipeline. Result at C+3.
2. **Bypass path** (FCMP, FMIN, FMAX, FABS, FNEG): Combinational logic + 1 register stage. Result at C+1.

**Critical rule:** A bypass operation and a pipeline operation may be issued on consecutive cycles. They use different write-back timing and do not conflict **unless their writes target port 1 on the same cycle**. The assembler must check:
- No ADD pipeline write (from C-3) conflicts with an ADD bypass write (from C-1) on the same cycle.
- No ADD write (pipeline or bypass) conflicts with a DIV/SQRT done (which also uses Port 1).
*(Note: LSU, ROM, and IntALU now use a dedicated Port 2, so they do NOT conflict with ADD).*

### ADD Exceptions
- +Inf + (-Inf) → qNaN + `invalid`.
- +0 + (-0) → +0 (RNE rule).
- -0 + (-0) → -0.
- FCMP(NaN, x) → unordered: `cmp_lt` = 0, comparison result = 0 (NaN is not less than anything).
- FABS(NaN) → qNaN with sign cleared.
- FNEG(NaN) → qNaN with sign flipped.
- FMIN/FMAX with NaN → the non-NaN operand is returned. If both NaN → qNaN.

---

## 4. DIV/SQRT Slot Operations

| op[1:0] | Mnemonic | Semantics | Latency | Write Port |
|---|---|---|---|---|
| 2'b00 | NOP | No operation | — | — |
| 2'b01 | FDIV | (stored_rd) ← ra / rb | 28 | Port 1, cycle C+28 |
| 2'b10 | FSQRT | (stored_rd) ← √ra | 27 | Port 1, cycle C+27 |
| 2'b11 | reserved | — | — | — |

### DS Destination Register

The DS slot has **no `rd` field** in its 16 bits. Instead:

**Design decision (frozen):** The destination register for DIV/SQRT is encoded in the `reg` field (bits [28:22]) of the **LSU/ALU slot of the same instruction**. The LSU/ALU op must be NOP (4'b0000) when DS is active. The assembler enforces this coupling.

```
Instruction with FDIV:
  DS slot:     op=01, ra=dividend_reg, rb=divisor_reg
  LSU/ALU slot: op=0000, reg=destination_reg, imm=don't_care
```

When `ds_done_i` fires (28 or 27 cycles later), the sequencer writes `ds_result_i` to `stored_rd` via port 1.

### DS Exclusivity
- Only one DIV/SQRT may be in flight at any time.
- If `start_i` is asserted while `busy_o` is high, it is **silently ignored** (no error, no result).
- The assembler guarantees at least 28 instructions (for DIV) or 27 (for SQRT) between the start and the next DS operation.

### DS Exceptions
- x / 0 → ±Inf + `divzero` flag.
- 0 / 0 → qNaN + `invalid`.
- Inf / Inf → qNaN + `invalid`.
- √(-x) → qNaN + `invalid` (for x > 0).
- √(-0) → -0 (IEEE-754 rule).
- √(+Inf) → +Inf.

---

## 5. LSU/ALU Slot Operations

| op[3:0] | Mnemonic | Semantics | Latency | Write Port |
|---|---|---|---|---|
| 4'b0000 | NOP | No operation (or DS destination carrier) | — | — |
| 4'b0001 | LOAD | reg ← TIDE_RAM[imm[11:0]] | 1 | Port 2, cycle C+1 |
| 4'b0010 | STORE | TIDE_RAM[imm[11:0]] ← reg | 1 (write) | — |
| 4'b0011 | LOAD_ROM | reg ← CONST_ROM[imm[5:0]] | 0 (comb) | Port 2, cycle C+1 |
| 4'b0100 | IADD | reg ← reg + sign_ext(imm) | 0 (comb) | Port 2, cycle C+1 |
| 4'b0101 | ISUB | reg ← reg - RF[imm[6:0]] (reg-reg subtract) | 0 (comb) | Port 2, cycle C+1 |
| 4'b0110 | ICMP | Compare reg vs RF[imm[6:0]], set `cmp_lt` latch | 0 (comb) | — |
| 4'b0111 | ITOF | reg ← int32_to_fp32(reg) (RNE) | 0 (comb) | Port 2, cycle C+1 |
| 4'b1000 | LDLOOP | Hardware loop counter ← reg | 0 (comb) | — |
| 4'b1001 | LDIDX0 | Index register 0 ← reg | 0 (comb) | — |
| 4'b1010 | LDIDX1 | Index register 1 ← reg | 0 (comb) | — |
| 4'b1011 | IMOV | reg ← sign_ext(imm) | 0 (comb) | Port 2, cycle C+1 |
| 4'b1100..1111 | reserved | — | — | — |

### LOAD/STORE Address Computation

The effective address is `imm[11:0]` (word address within TIDE-RAM, 0–1023).

When index registers are active (CTRL.idx_mode ≠ 0), the address is:
```
effective_addr = imm[11:0] + idx_reg_value
```
Post-increment of the index register happens **after** the address is computed.

### Stall on LSU Busy

If the LSU is busy with a burst transfer (`lsu_ready_i == 0`), the sequencer pipeline stalls. The LOAD/STORE does not execute until `lsu_ready_i` goes high.

---

## 6. CTRL Slot

| Bit | Name | Semantics |
|---|---|---|
| [8] | `end_of_unit` | Sequencer waits for all in-flight ops to retire (drain), then halts. Asserts `done_o`. |
| [7] | `boundary_check` | Sequencer pauses. Asserts `boundary_o`. Temporal controller decides to resume or switch. |
| [6] | `branch` | If set, next PC = branch_target instead of PC+1. |
| [5] | `branch_cond` | If set, branch is conditional (taken only if condition met). If clear, branch is unconditional. |
| [4] | `branch_sense` | If set, branch when condition is TRUE. If clear, branch when condition is FALSE. |
| [3] | `loop_dec` | Decrement the loop counter register. |
| [2] | `loop_branch` | Branch to branch_target if loop counter > 0 (checked after decrement). |
| [1:0] | `idx_mode` | 00=none, 01=post-incr idx0, 10=post-incr idx1, 11=post-incr both. |

### Branch Target Encoding

When `branch == 1` or `loop_branch == 1`, the branch target is `lsu_imm[8:0]` (9 bits → addresses 0–511). The LSU/ALU slot's imm field is **repurposed**; no LSU/ALU operation may execute in the same instruction as a branch.

**Forbidden combination:** `branch == 1` AND `lsu_op != NOP`. Assembler error.

### Branch Condition Source

The condition is the `cmp_lt` latch, set by the most recent FCMP or integer compare. The latch persists until the next comparison instruction.

| branch_cond | branch_sense | Branch taken when |
|---|---|---|
| 0 | X | Always (unconditional) |
| 1 | 1 | cmp_lt == 1 (last compare: a < b) |
| 1 | 0 | cmp_lt == 0 (last compare: a >= b) |

### Loop Mechanism

The loop counter is a 16-bit register, set via `IMOV` to a well-known RF register (e.g., r126) and then loaded into the hardware loop counter. The `loop_dec` + `loop_branch` bits together implement:
```
loop_counter--;
if (loop_counter > 0) PC = branch_target;
else PC = PC + 1;  // fall through
```

### Index Register Post-Increment

When `idx_mode != 00`, the addressed index register(s) increment by 1 **after** all other slot operations in this instruction have captured their addresses. This ensures the current instruction uses the pre-increment value.

---

## 7. Simultaneous Slot Rules (frozen)

All 5 slots execute **in parallel** within a single instruction cycle. The following rules govern legality:

### 7.1 Write Port Allocation

| Source | Port | Timing |
|---|---|---|
| MUL result | Port 0 | C+2 |
| ADD/SUB result | Port 1 | C+3 |
| ADD bypass result | Port 1 | C+1 |
| DS done result | Port 1 | C+28 or C+27 |
| LSU LOAD / ROM LOAD | Port 2 | C+1 |
| IntALU / ITOF | Port 2 | C+1 |

**Rule W1:** At most ONE port-0 write per cycle. (MUL is the only port-0 source).

**Rule W2:** At most ONE port-1 write per cycle. The assembler must ensure that across all in-flight instructions, no two port-1 results land on the same cycle.

**Rule W2b:** At most ONE port-2 write per cycle. The assembler must ensure that LSU, IntALU, and ROM loads do not conflict on port 2.

**Rule W3:** MUL, ADD/DS, and LSU can write in the same cycle (different ports).

### 7.2 Read Port Allocation

| Source | Ports Used |
|---|---|
| MUL ra, rb | Port 0, Port 1 |
| ADD ra, rb | Port 2, Port 3 |
| DS ra, rb | Port 0, Port 1 (reuse MUL ports) |
| LSU/ALU reg | Port 2 or Port 3 (reuse ADD port) |

**Rule R1:** MUL and DS share read ports 0 and 1. They cannot both read in the same instruction. The assembler must ensure `mul_op == NOP` when `ds_op != NOP` (or vice versa) **if both need reads on the same cycle**.

**Exception:** If MUL uses the same source registers as DS, port sharing is acceptable (same address, same data).

**Rule R2:** ADD and LSU/ALU share read ports 2 and 3. Similar restriction.

### 7.3 LSU/ALU + Branch Exclusion

**Rule B1:** When `branch == 1` or `loop_branch == 1`, the LSU imm field is the branch target. Therefore, `lsu_op` must be `NOP`.

### 7.4 DS + LSU/ALU Coupling

**Rule D1:** When `ds_op != NOP`, the LSU/ALU `reg` field is the DS destination register. `lsu_op` must be `NOP`.

---

## 8. Hazard Rules (assembler-enforced, no hardware interlocks)

### 8.1 RAW (Read-After-Write)

An instruction at cycle C+N cannot read a register that was written at cycle C+K where K > N (i.e., the write hasn't happened yet).

| Producer | Write cycle | Earliest consumer read cycle |
|---|---|---|
| MUL result (rd) | C+2 | C+3 |
| ADD/SUB result (rd) | C+3 | C+4 |
| ADD bypass result (rd) | C+1 | C+2 |
| DS result (rd) | C+28 or C+27 | C+29 or C+28 |
| LSU LOAD / ROM (reg) | C+1 | C+2 |
| IntALU result (reg) | C+1 | C+2 |

### 8.2 WAW (Write-After-Write)

Two writes to the same register on the same cycle: **illegal**. The assembler must ensure unique write targets per port per cycle across all in-flight instructions.

### 8.3 WAR (Write-After-Read)

Not a hazard in this architecture because reads are combinational (cycle C) and writes are registered (cycle C+K where K ≥ 1). A write at C+1 cannot corrupt a read at C.

---

## 9. Instruction Encoding Examples

### Example 1: FMUL r4, r7 → r12 (standalone)
```
MUL:     op=01, ra=0000100, rb=0000111, rd=0001100
ADD:     op=000 (NOP), ra=0, rb=0, rd=0
DS:      op=00 (NOP), ra=0, rb=0
LSU/ALU: op=0000 (NOP), reg=0, imm=0
CTRL:    000000000

Binary: 01_0000100_0000111_0001100 | 000_0000000_0000000_0000000 | 00_0000000_0000000 | 0000_0000000_0000000000000 | 000000000
```

### Example 2: FMUL r4, r7 → r12 + FADD r10, r11 → r20 (parallel)
```
MUL:     op=01, ra=r4, rb=r7, rd=r12
ADD:     op=001 (FADD), ra=r10, rb=r11, rd=r20
DS:      NOP
LSU/ALU: NOP
CTRL:    000000000

MUL writes r12 via port 0 at C+2
ADD writes r20 via port 1 at C+3
No conflict.
```

### Example 3: FDIV r3, r5 → r15 (DS + destination in LSU reg)
```
MUL:     NOP
ADD:     NOP
DS:      op=01 (FDIV), ra=r3, rb=r5
LSU/ALU: op=0000 (NOP), reg=r15 (destination), imm=0
CTRL:    000000000

DS writes r15 via port 1 at C+28.
LSU/ALU op is NOP so only the reg field is used as DS destination.
```

### Example 4: Loop with branch
```
MUL:     NOP
ADD:     NOP
DS:      NOP
LSU/ALU: op=0000, reg=0, imm=loop_start_addr (9 bits)
CTRL:    000001100 (loop_dec=1, loop_branch=1)

Decrements loop counter. If > 0, branches to loop_start_addr.
```

---

## Agent-1 freeze addendum (G1, own/doc)

Implemented exactly as specified in `tools/tide_asm.py`, cross-checked instruction-by-instruction against `shared/contract/isa.yaml`'s `add_ops`/`ds_ops`/`lsu_alu_ops` tables (0 mismatches, `tools/tide_asm.py --test`). Clarifications and additions beyond the source text, all logged in `ASSUMPTIONS.md`:

- **ABI register additions** (A3): r118=dt_us (int32, TC-precomputed), r119=context base word address, r120/121/122=QP/QV/DT0INV float32 mirrors. Not in memmap.yaml; needed because Step-0 assigns dt_us computation to the TC but the contract never names a register for the result. r0 is reserved/unused by the code generator (A4) as a safety margin against a possible TC burst-preload convention.
- **A-IDX**: an address operand may reference at most one of idx0/idx1; `idx_mode == 2'b11` is treated as undefined/rejected by the assembler (not exercised by the generated microcode).
- **A-ADDNOP**: the assembler treats a literal `ADD r0,r0,r0` as an error rather than silently accepting it as the reserved all-zero NOP encoding, to keep source intent unambiguous. Use `NOP` in the ADD slot.
- **Branch conditions**: only `BRA`/`BLT`/`BGE`/`LOOP_BRA` are encodable (the ISA has a single `cmp_lt` latch, no equality latch); `BEQ`/`BNE` are rejected at assemble time with an explicit message rather than silently miscompiling.
- **Timing values used** (MUL 2 / ADD-SUB 3 / bypass 1 / DIV 28+1 / SQRT 27+1 / LSU-ALU 1, FCMP/ICMP-to-conditional-branch >=2 cycles, 1-cycle bubble on a taken branch) are taken from `15_timing_contracts.md`'s per-interface tables (authoritative per that document's own header), not this document's rougher summary table, where the two differ by rounding.
