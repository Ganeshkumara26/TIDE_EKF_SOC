# 15 — TIDE Internal Interface & Timing Contracts

**Purpose:** Authoritative cycle-by-cycle timing for every internal interface. This document is the single source of truth for interface timing. (Doc 16 is supreme for micro-ISA encodings and semantics).

---

## 1. Clock and Reset

| Signal | Domain | Convention |
|---|---|---|
| `clk_i` | Single clock, 50 MHz target [A4], 20 ns period | All logic is synchronous to posedge |
| `rst_ni` | Active-low, synchronous | All flops reset on `rst_ni == 0`. After deassertion, first valid posedge is cycle 0. |

**Reset guarantee:** After reset deasserts, all outputs are in defined states by cycle 1. No X allowed.

---

## 2. Register File Timing (4R / 2W)

| Operation | Timing | Contract |
|---|---|---|
| Read | Combinational | `rdata` valid in the same cycle `raddr` is presented. No clock edge needed. |
| Write | Posedge-captured | Data written on the rising edge of `clk_i` when `wen == 1`. Available for read on the **next** cycle. |
| Read-during-write (same address) | Read returns **old** value | Write-first semantics would require a bypass; the design uses read-old for simplicity. Assembler must account for this. |
| Port 0 write | Reserved for MUL results | Sequencer write-back scheduler routes MUL lane results here. |
| Port 1 write | Shared: ADD, DIV/SQRT done | Sequencer write-back scheduler routes ADD and DS results here. |
| Port 2 write | Shared: LSU LOAD, IntALU, Const ROM, temporal controller | Priority: temporal controller RF writes (event data loading) take priority over sequencer port-2 writes. This never conflicts because the temporal controller only writes when the sequencer is idle. |

### Timing Diagram: RF Read → MUL → RF Write

```
Cycle C  : Sequencer presents raddr0=ra, raddr1=rb (combinational read)
           rdata0 and rdata1 are valid immediately
           MUL lane receives a_i=rdata0, b_i=rdata1, valid_i=1

Cycle C+1: MUL pipeline stage 1 (Booth encode + partial product)
           RF read ports free for next instruction

Cycle C+2: MUL pipeline stage 2 (normalize + round)
           mul_valid_o asserts, mul_result_o valid
           Write-back scheduler drives rf_wen0=1, rf_waddr0=rd, rf_wdata0=mul_result
           Result available for read starting cycle C+3
```

---

## 3. FP Lane Timing Contracts

### 3.1 MUL Lane (`tide_fp32_mul`)

| Signal | Direction | Timing |
|---|---|---|
| `valid_i` | in | Assert for exactly 1 cycle to start multiplication |
| `a_i`, `b_i` | in | Must be stable when `valid_i` is high |
| `valid_o` | out | Asserts exactly 2 cycles after `valid_i` |
| `result_o` | out | Valid when `valid_o` is high; undefined otherwise |
| `flags_o` | out | Valid when `valid_o` is high |

**Pipeline acceptance:** Every cycle. No stall. Back-to-back operations are legal (new `valid_i` every cycle).

**Latency: 2 cycles. Deterministic. No data-dependent variation.**

### 3.2 ADD Lane (`tide_fp32_add`)

| Signal | Direction | Timing |
|---|---|---|
| `valid_i` | in | Assert for 1 cycle |
| `op_i` | in | Must be stable with `valid_i` |
| `valid_o` | out | ADD/SUB: 3 cycles after `valid_i`. Bypass ops (CMP/ABS/NEG/MIN/MAX/MOV): see bypass. |
| `bypass_valid_o` | out | 1 cycle after `valid_i` for CMP/ABS/NEG/MIN/MAX/MOV |
| `bypass_result_o` | out | Valid when `bypass_valid_o` is high |
| `cmp_lt_o` | out | Valid 1 cycle after a CMP operation. Latched until next CMP. |

**Pipeline acceptance:** Every cycle for ADD/SUB. Bypass ops do not enter the pipeline; they use combinational logic + 1 flop.

**Write-back schedule:**
- ADD/SUB result → port 1, cycle C+3
- CMP/ABS/NEG/FMIN/FMAX/FMOV result → port 1, cycle C+1

### 3.3 DIV/SQRT Lane (`tide_fp32_divsqrt`)

| Signal | Direction | Timing |
|---|---|---|
| `start_i` | in | Assert for 1 cycle to begin. Ignored if `busy_o` is high. |
| `op_i` | in | 0=DIV, 1=SQRT. Stable with `start_i`. |
| `busy_o` | out | High from cycle after `start_i` until cycle of `done_o` (inclusive). |
| `done_o` | out | Single-cycle pulse. DIV: exactly 28 cycles after `start_i`. SQRT: exactly 27 cycles. |
| `result_o` | out | Valid when `done_o` is high. Undefined at all other times. |

**Exclusivity:** Only one DIV/SQRT can be in flight. The assembler guarantees no new `start_i` while `busy_o` is high.

**Write-back:** The sequencer captures `result_o` on the cycle `done_o` fires and writes it to the stored destination register via port 1.

### 3.4 Integer ALU (`tide_fp32_intalu`)

| Signal | Direction | Timing |
|---|---|---|
| `valid_i` | in | Assert for 1 cycle |
| `result_o` | out | Combinational (valid same cycle as `valid_i`) |

**Latency: 0 cycles (combinational).** Result is used immediately by the sequencer for write-back on the **same cycle's** posedge (port 2, if no conflict).

However, since the RF write happens on the posedge, the result is effectively **available for read on cycle C+1** (same as "1-cycle latency" from the perspective of dependent instructions).

### 3.5 Constant ROM (`tide_const_rom`)

| Signal | Direction | Timing |
|---|---|---|
| `addr_i` | in | Combinational |
| `data_o` | out | Combinational (no clock needed; pure LUT) |

**Latency: 0 cycles.** Data is available in the same cycle the address is presented. Write-back to RF is via the LSU/ALU slot, port 2, same posedge.

---

## 4. Microcode RAM Timing

| Signal | Direction | Timing |
|---|---|---|
| `seq_addr_i` | in | Presented by sequencer's PC |
| `seq_data_o` | out | Available **1 cycle later** (synchronous read) |

**Read latency: 1 cycle.** This is why the sequencer has a 2-stage pipeline (Fetch → Decode/Execute).

### CPU Write Protocol (3-phase)

```
Phase 0: CPU writes word[31:0]  to UCODE_BASE + addr*4 + 0  → staging[31:0]
Phase 1: CPU writes word[63:32] to UCODE_BASE + addr*4 + 4  → staging[63:32]
Phase 2: CPU writes word[95:64] to UCODE_BASE + addr*4 + 8  → staging[95:64]
         AND: full 96-bit word committed to ucode_ram[addr]
```

**Commit rule:** The 96-bit word is written to the RAM only on the phase-2 write. Phases 0 and 1 only update staging registers. If phase 2 never happens (interrupt, reset), the RAM is unchanged.

**Endianness:** Little-endian word order. Phase 0 = bits [31:0], phase 1 = bits [63:32], phase 2 = bits [95:64].

---

## 5. LSU Timing

| Operation | Latency | Stall |
|---|---|---|
| Single LOAD (TIDE-RAM → RF) | 1 cycle | None (synchronous read) |
| Single STORE (RF → TIDE-RAM) | 1 cycle | None |
| Burst LOAD (context restore) | `len` cycles | Sequencer stalled via `lsu_ready_i = 0` |
| Burst STORE (context save) | `len` cycles | Sequencer stalled |

**Burst protocol:**
```
Cycle 0: burst_start asserted, rf_base, ram_base, len, we set
Cycle 1: First word transferred (RF[rf_base] ↔ RAM[ram_base])
Cycle 2: Second word (RF[rf_base+1] ↔ RAM[ram_base+1])
...
Cycle len: Last word. burst_done asserts.
```

**Arbitration:** LSU and temporal controller both access TIDE-RAM. The temporal controller has priority for direct RAM access (journal management). The LSU has priority for burst transfers. They never conflict because:
- During sequencer execution: LSU is active, temporal controller is idle.
- During context/checkpoint transfers: temporal controller commands the LSU.
- During journal insert/scan: temporal controller uses direct RAM port, sequencer is idle.

---

## 6. Sequencer ↔ Temporal Controller Interface

| Signal | Direction | Timing | Contract |
|---|---|---|---|
| `start_i` | TC → SEQ | 1-cycle pulse | Sequencer begins executing from `start_addr_i` on the next cycle. Must not be asserted while `running_o` is high. |
| `start_addr_i` | TC → SEQ | Stable when `start_i` is high | Entry points: `UNIT_ENTRY` for predict+update, `PREDICT_ONLY` for query predict. |
| `running_o` | SEQ → TC | Level | High while the sequencer is executing microcode. Low when idle or paused at boundary. |
| `done_o` | SEQ → TC | 1-cycle pulse | Asserts when `end_of_unit` bit is encountered AND all in-flight operations have drained (retired). Sequencer returns to idle. |
| `boundary_o` | SEQ → TC | 1-cycle pulse | Asserts when `boundary_check` bit is encountered. Sequencer pauses. TC decides whether to resume (re-assert `start_i`) or switch to a different event. |

**Critical contract:** `done_o` and `boundary_o` are mutually exclusive. An instruction cannot have both `end_of_unit` and `boundary_check` set.

---

## 7. Temporal Controller ↔ Event FIFO Interface

| Signal | Direction | Timing |
|---|---|---|
| `event_valid_i` | FIFO → TC | Level. High when FIFO is non-empty. |
| `event_ready_o` | TC → FIFO | Level. High when TC can accept an event. TC deasserts during unit execution, replay, and context transfers. |
| Handshake | Both high | Event is consumed on the posedge. FIFO pops. |

**Contract:** The FIFO presents the head-of-queue event continuously while `event_valid_i` is high. The TC captures the event data on the posedge when both `valid` and `ready` are high.

---

## 8. PCPI ↔ Lane Timing

| Operation | PCPI wait_o | Result timing | Total CPU cycles |
|---|---|---|---|
| FMUL (MODE=0) | Asserted for 2 cycles | `ready_o` on cycle 3 | 3 |
| FADD/FSUB (MODE=0) | Asserted for 3 cycles | `ready_o` on cycle 4 | 4 |
| FCMP/FABS/FNEG (MODE=0) | Asserted for 1 cycle | `ready_o` on cycle 2 | 2 |
| FDIV (MODE=0) | Asserted for 28 cycles | `ready_o` on cycle 29 | 29 |
| FSQRT (MODE=0) | Asserted for 27 cycles | `ready_o` on cycle 28 | 28 |
| FITOF (MODE=0) | Asserted for 1 cycle | `ready_o` on cycle 2 | 2 |
| Any FP op (MODE=1) | Not asserted | `ready_o` on cycle 1, result=0, ERR_BUSY set | 1 |

**MODE transition contract:**
- Writing MODE=1 while a PCPI operation is in flight: **undefined behavior** (firmware bug). Hardware must not hang; the in-flight result may be lost.
- Writing MODE=0 while engine is running: **undefined behavior**. ERR_BUSY is set. The sequencer continues using the lanes; the CPU's next PCPI instruction will get a stale result.
- **Safe transition sequence:** (1) Ensure engine is idle (`BUSY == 0`). (2) Write MODE. (3) Wait 1 cycle. (4) Use the new mode.

---

## Agent-1 freeze addendum (own/doc)

Followed as written; every latency figure in `tools/tide_sim.py`'s cycle model (MUL 2, ADD/SUB 3, bypass ops 1, DIV 28 (ready cycle 29), SQRT 27 (ready cycle 28), LSU/int-ALU 1, FCMP/ICMP-to-conditional-branch >=2 cycles) is sourced from this document's per-interface tables and matches the directed timing tests in `21_tide_sim.md` section 10 exactly (`tools/tide_sim.py --test`, `[S2]`). No corrections needed. See `ASSUMPTIONS.md` A4 for a gap this document's burst-transfer description (section on the LSU) exposes together with `tide_lsu.sv` and `memmap.yaml`'s register-file table: the burst ports (`burst_start_i` etc.) have no microcode-level trigger in the frozen `isa.yaml`, so Agent-1's CTX_LOAD/CTX_STORE use ordinary indexed LOAD/STORE instructions instead of the burst path this document describes timing for.
