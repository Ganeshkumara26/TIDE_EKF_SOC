# Design: `tide_lsu.sv` — Load/Store Unit & Burst Context Transfer Engine

**Spec reference:** §6.4, Decision T6 (Burst Context Transfer), §6.5 (TIDE-RAM allocation)
**Gate:** G5 (Engine Integration)
**New file**

---

## 1. Purpose

The Load/Store Unit is the TIDE engine's sole gateway to the 4 KB TIDE-RAM. Every data movement between the 128×32 register file and the external scratchpad — single-word indexed loads and stores during microcode execution, multi-word burst context saves and restores during OOSM replay, and journal slot reads during temporal scanning — flows through this module. It is the bridge that makes the two-level storage hierarchy (fast registers for active computation, dense RAM for historical contexts) transparent to the sequencer and temporal controller.

The design is driven by a hard real-time constraint: when an out-of-sequence measurement arrives, the temporal controller must save the current 27-word EKF context (6 state + 21 covariance), restore a historical context, run the correction, then re-propagate forward. The burst transfer engine must complete a full 27-word save or restore in exactly 27 cycles to keep the total OOSM replay latency under the 400 µs worst-case budget at 50 MHz.

## 2. Interface

```systemverilog
module tide_lsu (
    input  logic        clk_i,
    input  logic        rst_ni,

    // === Single-word interface (from Sequencer, LSU slot) ===
    input  logic        seq_valid_i,       // Sequencer issues a LOAD or STORE
    input  logic        seq_we_i,          // 0 = LOAD (RAM→RF), 1 = STORE (RF→RAM)
    input  logic [6:0]  seq_rf_addr_i,     // Register file address (0..127)
    input  logic [9:0]  seq_ram_addr_i,    // TIDE-RAM word address (0..1023)
    input  logic [31:0] seq_wdata_i,       // Write data for STORE (from RF read port)
    output logic [31:0] seq_rdata_o,       // Read data for LOAD (to RF write port 2)
    output logic        seq_ready_o,       // 1 = LSU can accept a new command

    // === Burst interface (from Temporal Controller) ===
    input  logic        burst_start_i,     // Begin burst transfer
    input  logic        burst_we_i,        // 0 = context LOAD (restore), 1 = context STORE (save)
    input  logic [6:0]  burst_rf_base_i,   // Starting register address
    input  logic [9:0]  burst_ram_base_i,  // Starting TIDE-RAM address
    input  logic [4:0]  burst_len_i,       // Number of words (1..27, typically 27)
    output logic        burst_busy_o,      // 1 = burst in progress
    output logic        burst_done_o,      // 1-cycle pulse when burst completes

    // === Register File write port (shared with sequencer port 2) ===
    output logic        rf_we_o,
    output logic [6:0]  rf_waddr_o,
    output logic [31:0] rf_wdata_o,

    // === Register File read port (for STORE operations) ===
    output logic [6:0]  rf_raddr_o,
    input  logic [31:0] rf_rdata_i,

    // === TIDE-RAM interface (dual-port, port B) ===
    output logic [9:0]  ram_addr_o,
    output logic [31:0] ram_wdata_o,
    output logic        ram_we_o,
    output logic        ram_en_o,          // Chip enable for power gating
    input  logic [31:0] ram_rdata_i
);
```

## 3. TIDE-RAM Address Map

The 4 KB TIDE-RAM is organized as 1024 × 32-bit words. The allocation is managed by firmware and the temporal controller. The LSU itself is address-agnostic — it does not enforce the map below. But the firmware and temporal controller adhere to this convention:

| Address Range | Words | Purpose |
|---|---|---|
| 0x000–0x01A | 27 | Context slot 0 (active context backup) |
| 0x01B–0x034 | 27 | Context slot 1 (OOSM historical #1) |
| 0x035–0x04E | 27 | Context slot 2 (OOSM historical #2) |
| 0x04F–0x068 | 27 | Context slot 3 (OOSM historical #3) |
| 0x069–0x082 | 27 | Context slot 4 (OOSM historical #4) |
| 0x083–0x09C | 27 | Context slot 5 (OOSM historical #5) |
| 0x09D–0x0B6 | 27 | Context slot 6 (OOSM historical #6) |
| 0x0B7–0x0D0 | 27 | Context slot 7 (OOSM historical #7) |
| 0x0D1–0x0FF | 47 | Journal metadata (timestamps, slot validity) |
| 0x100–0x1FF | 256 | IMU delta buffer (ring, 64 entries × 4 words) |
| 0x200–0x3FF | 512 | Scratch / future expansion |

Each context slot holds 6 state elements `x[0..5]` followed by 21 lower-triangle covariance elements `P[0..20]`, stored in row-major order. A context snapshot also includes the estimation timestamp `t_est` as the 27th word.

## 4. Operational Modes

### 4.1 Single-Word LOAD (RAM → RF)

The sequencer's LSU slot decodes a `LOAD` instruction and asserts `seq_valid_i` with `seq_we_i = 0`. The LSU presents the address to TIDE-RAM, and returns the read data one cycle later via `seq_rdata_o`. The sequencer writes this to the register file via port 2.

```
Cycle N  : seq_valid_i=1, seq_we_i=0, seq_ram_addr_i=addr
           LSU drives ram_addr_o=addr, ram_en_o=1, ram_we_o=0
Cycle N+1: ram_rdata_i contains the word
           LSU drives seq_rdata_o = ram_rdata_i
           Sequencer writes to RF port 2: rf_waddr2 = seq_rf_addr_i, rf_wdata2 = seq_rdata_o
```

**Latency: 1 cycle.** `seq_ready_o` deasserts for 0 cycles (pipelined read, next LOAD can be accepted immediately).

### 4.2 Single-Word STORE (RF → RAM)

The sequencer reads the source register via its read port, passes the data as `seq_wdata_i`, and asserts `seq_valid_i` with `seq_we_i = 1`. The LSU writes the data to TIDE-RAM on the same cycle.

```
Cycle N  : seq_valid_i=1, seq_we_i=1, seq_ram_addr_i=addr, seq_wdata_i=data
           LSU drives ram_addr_o=addr, ram_wdata_o=data, ram_we_o=1, ram_en_o=1
```

**Latency: 1 cycle** (fire-and-forget). The sequencer does not stall.

### 4.3 Burst Context SAVE (RF → RAM, 27 words)

Initiated by the temporal controller before OOSM replay. The sequencer must be idle.

```
Cycle 0  : burst_start_i=1, burst_we_i=1, burst_rf_base_i=0, burst_ram_base_i=slot_addr, burst_len_i=27
           LSU captures parameters, asserts burst_busy_o=1, seq_ready_o=0
Cycle 1  : rf_raddr_o = rf_base + 0, RAM write addr = ram_base + 0
           (RF read is combinational → data available same cycle)
           ram_wdata_o = rf_rdata_i, ram_we_o = 1
Cycle 2  : rf_raddr_o = rf_base + 1, RAM write addr = ram_base + 1
           ...
Cycle 27 : rf_raddr_o = rf_base + 26, RAM write addr = ram_base + 26
           Last word written. burst_done_o pulses. burst_busy_o deasserts.
Cycle 28 : seq_ready_o re-asserts. Sequencer may resume.
```

**Latency: 27 cycles for a full EKF context.**

### 4.4 Burst Context RESTORE (RAM → RF, 27 words)

Initiated by the temporal controller to load a historical context for OOSM replay.

```
Cycle 0  : burst_start_i=1, burst_we_i=0, burst_rf_base_i=0, burst_ram_base_i=slot_addr, burst_len_i=27
           LSU captures parameters, asserts burst_busy_o=1
Cycle 1  : ram_addr_o = ram_base + 0, ram_en_o = 1
Cycle 2  : ram_rdata_i returns word 0. rf_we_o=1, rf_waddr_o=rf_base+0, rf_wdata_o=ram_rdata_i
           Simultaneously: ram_addr_o = ram_base + 1 (pipelined)
Cycle 3  : word 1 → RF[rf_base+1], request word 2
           ...
Cycle 28 : word 26 → RF[rf_base+26]. burst_done_o pulses.
```

**Latency: 28 cycles** (1 extra for the read pipeline fill). The temporal controller accounts for this in its cycle budget.

## 5. Arbitration and Priority

The LSU has two command sources: the sequencer (single-word) and the temporal controller (burst). They never legitimately conflict because the temporal controller only issues bursts when the sequencer is idle (between EKF units or during an explicit boundary pause). The hardware enforces this with a simple rule:

- **Burst has absolute priority.** If `burst_start_i` arrives while a single-word operation is in progress, the burst takes over immediately. This should never happen in correct firmware; the hardware handles it defensively.
- **`seq_ready_o` deasserts** for the entire duration of a burst. The sequencer's fetch pipeline sees the stall and inserts bubbles.

There is no fairness arbiter. This is a strict priority scheme: temporal controller > sequencer.

## 6. TIDE-RAM Port Allocation

TIDE-RAM is a true dual-port SRAM (or synthesized dual-port register file for FPGA/ASIC):

| Port | Owner | Purpose |
|---|---|---|
| Port A | PicoRV32 CPU via AHB-Lite | Firmware read/write for diagnostics, initial context loading |
| Port B | TIDE LSU | All engine-side data movement |

The CPU (Port A) must not write to TIDE-RAM while the engine is running. Firmware enforces this by checking the BUSY status register. Simultaneous reads from both ports are always safe.

## 7. Address Generation Logic

For single-word operations, the address comes directly from the sequencer's decoded immediate field (13 bits, but only 10 used for TIDE-RAM, upper 3 reserved).

For burst operations, the LSU contains a pair of auto-incrementing counters:

```systemverilog
logic [9:0] burst_ram_ptr;
logic [6:0] burst_rf_ptr;
logic [4:0] burst_cnt;

always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
        burst_cnt <= '0;
    end else if (burst_start_i) begin
        burst_ram_ptr <= burst_ram_base_i;
        burst_rf_ptr  <= burst_rf_base_i;
        burst_cnt     <= burst_len_i;
    end else if (burst_busy_o && burst_cnt > 0) begin
        burst_ram_ptr <= burst_ram_ptr + 1;
        burst_rf_ptr  <= burst_rf_ptr  + 1;
        burst_cnt     <= burst_cnt - 1;
    end
end
```

## 8. Area Estimate

The LSU is predominantly control logic with minimal datapath. The counters, muxes, and FSM are estimated at:

| Component | Gate Count |
|---|---|
| Burst FSM (3 states) | ~80 gates |
| Address counters (10-bit + 7-bit + 5-bit) | ~200 gates |
| Mux logic (burst vs single) | ~300 gates |
| Handshake / control | ~120 gates |
| **Total** | **~700 gates** |

This is negligible compared to the arithmetic lanes (~20k gates each). On FPGA (Artix-7), the LSU consumes approximately 45 LUTs and 30 flip-flops.

## 9. Verification Plan

| Test | Method | Count | Pass Criterion |
|---|---|---|---|
| Single LOAD: write RAM via CPU, LOAD to RF | Verilator | 100 | Bit-exact match |
| Single STORE: write RF, STORE to RAM, read via CPU | Verilator | 100 | Bit-exact match |
| Burst SAVE: populate RF[0..26], burst save, verify RAM | Verilator | 10 | All 27 words match |
| Burst RESTORE: populate RAM slot, burst restore, verify RF | Verilator | 10 | All 27 words match |
| Round-trip: burst save → burst restore → compare RF | Verilator | 20 | Identical RF contents |
| Burst during sequencer idle | Verilator | 5 | seq_ready_o deasserts correctly |
| Back-to-back single LOADs | Verilator | 50 | Pipeline accepts every cycle |
| Address boundary: RAM addr 0x000 and 0x3FF | Verilator | 4 | No wrap-around errors |
| Burst length edge cases: len=1, len=27, len=0 | Verilator | 6 | Correct behavior, len=0 is NOP |

## 10. Implementation Notes for the Coder

1. **The burst FSM has three states:** IDLE, BURST_ACTIVE, BURST_DONE. Keep it simple. The BURST_DONE state lasts exactly 1 cycle (to pulse `burst_done_o`) then returns to IDLE.

2. **Do NOT use a FIFO** for single-word operations. The sequencer issues at most one LOAD or STORE per cycle, and the result is consumed immediately. A register-based pipeline stage is sufficient.

3. **Register `seq_rdata_o`** for timing closure. The critical path is `ram_rdata_i → seq_rdata_o → RF write port 2 data input`. Registering at the LSU output adds 1 cycle of latency but breaks a long combinational path. The sequencer's write-back scheduler already accounts for this 1-cycle LOAD latency.

4. **The burst length of 27 is not a magic number in hardware.** The burst counter accepts any value 1–31 (5-bit field). The value 27 comes from firmware (6 state + 21 covariance). If the EKF state size changes in a future revision, only firmware changes — the hardware is parametric.

5. **Power gating:** Assert `ram_en_o = 0` when neither a single-word operation nor a burst is in progress. This saves dynamic power on the SRAM read amplifiers. On FPGA this signal can be left tied high.

6. **Reset behavior:** On reset, `burst_busy_o = 0`, `seq_ready_o = 1`, `ram_en_o = 0`, `ram_we_o = 0`, `rf_we_o = 0`. All address and data outputs are don't-care.
