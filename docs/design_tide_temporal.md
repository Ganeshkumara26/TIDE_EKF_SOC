# Design: `tide_temporal.sv` — Temporal Controller & OOSM Replay Coordinator

**Spec reference:** §6.5 (Temporal Arbitration), §6.7 (OOSM Policy), Decision T8 (Temporal Replay Management)
**Gate:** G6 (Temporal & OOSM Closure)
**New file**

---

## 1. Purpose

The temporal controller is the executive brain of the TIDE engine. While the sequencer blindly executes microcode and the arithmetic lanes crunch numbers, the temporal controller decides *what* to compute and *when*. It is responsible for:

1. **Event scheduling:** Accepting timestamped sensor measurement packets from the host CPU via an event FIFO, determining whether each measurement is in-order or out-of-sequence, and dispatching the correct microcode entry point to the sequencer.

2. **Context management:** Coordinating with the LSU to save and restore EKF state snapshots (6 state + 21 covariance + timestamp = 27 words) to and from TIDE-RAM context slots, enabling the engine to rewind to a historical epoch.

3. **OOSM replay orchestration:** When a late measurement arrives, the temporal controller must locate the nearest prior journal entry, restore that historical context, apply the late measurement's correction, then re-propagate the state forward through all subsequent measurements that have already been processed.

4. **Journal management:** Maintaining the `tide_late` out-of-sequence measurement journal — an ordered list of historical state epochs stored in TIDE-RAM. The journal supports insert, scan, and evict operations to manage the 8 available context slots.

Without this module, the TIDE engine would be a dumb FP calculator. The temporal controller is what makes it a real-time Extended Kalman Filter.

## 2. Interface

```systemverilog
module tide_temporal (
    input  logic        clk_i,
    input  logic        rst_ni,

    // === Event FIFO Interface (from host CPU) ===
    input  logic        event_valid_i,     // FIFO non-empty
    output logic        event_ready_o,     // TC can accept event
    input  logic [31:0] event_t_meas_i,    // Measurement timestamp (µs, 32-bit)
    input  logic [1:0]  event_type_i,      // 00=GPS, 01=baro, 10=mag, 11=reserved
    input  logic [31:0] event_z0_i,        // Measurement data word 0
    input  logic [31:0] event_z1_i,        // Measurement data word 1
    input  logic [31:0] event_z2_i,        // Measurement data word 2
    input  logic [31:0] event_z3_i,        // Measurement data word 3

    // === Sequencer Control ===
    output logic        seq_start_o,       // Start sequencer execution
    output logic [8:0]  seq_start_addr_o,  // Microcode entry point
    input  logic        seq_running_i,     // Sequencer is executing
    input  logic        seq_done_i,        // Sequencer completed (end_of_unit)
    input  logic        seq_boundary_i,    // Sequencer at event boundary

    // === LSU Burst Control ===
    output logic        lsu_burst_start_o,
    output logic        lsu_burst_we_o,    // 0=restore, 1=save
    output logic [6:0]  lsu_burst_rf_base_o,
    output logic [9:0]  lsu_burst_ram_base_o,
    output logic [4:0]  lsu_burst_len_o,
    input  logic        lsu_burst_busy_i,
    input  logic        lsu_burst_done_i,

    // === Register File Direct Access (measurement data loading) ===
    output logic        rf_we_o,           // Write enable
    output logic [6:0]  rf_waddr_o,        // Target register (r28–r31 for measurements)
    output logic [31:0] rf_wdata_o,        // Measurement data

    // === Journal RAM Interface (TIDE-RAM port, metadata region) ===
    output logic [9:0]  jrnl_addr_o,
    output logic [31:0] jrnl_wdata_o,
    output logic        jrnl_we_o,
    input  logic [31:0] jrnl_rdata_i,

    // === Status (to host CPU via MMIO) ===
    output logic        busy_o,            // Engine is processing
    output logic        oosm_active_o,     // Currently handling an OOSM replay
    output logic [2:0]  slot_count_o,      // Number of occupied journal slots (0..7)
    output logic [31:0] t_current_o        // Current estimation epoch (µs)
);
```

## 3. Finite State Machine

The temporal controller is a Moore-type FSM with 9 states:

```
                         ┌─────────────┐
                         │    IDLE      │◄──────────────────────┐
                         └──────┬───────┘                       │
                      event_valid_i=1                           │
                                │                               │
                         ┌──────▼───────┐                       │
                         │  CLASSIFY    │                       │
                         └──────┬───────┘                       │
                      t_meas >= t_current?                      │
                       ╱                 ╲                      │
                     YES                  NO                    │
                      │                    │                    │
              ┌───────▼────────┐   ┌───────▼────────┐          │
              │  LOAD_MEAS     │   │  OOSM_SCAN     │          │
              └───────┬────────┘   └───────┬────────┘          │
                      │                    │ found slot τ       │
              ┌───────▼────────┐   ┌───────▼────────┐          │
              │  RUN_PREDICT   │   │  OOSM_SAVE     │          │
              └───────┬────────┘   └───────┬────────┘          │
                seq_done_i                 │ save current ctx   │
                      │            ┌───────▼────────┐          │
              ┌───────▼────────┐   │  OOSM_RESTORE  │          │
              │  CHECKPOINT    │   └───────┬────────┘          │
              └───────┬────────┘           │ restore ctx(τ)    │
                      │            ┌───────▼────────┐          │
                      │            │  OOSM_CORRECT  │          │
                      │            └───────┬────────┘          │
                      │              seq_done_i                │
                      │            ┌───────▼────────┐          │
                      │            │  OOSM_FORWARD  │          │
                      │            └───────┬────────┘          │
                      │              all caught up             │
                      └────────────────────┴───────────────────┘
```

### State Descriptions

| State | Duration | Action |
|---|---|---|
| **IDLE** | Indefinite | Polls `event_valid_i`. Deasserts `busy_o`. |
| **CLASSIFY** | 1 cycle | Compares `event_t_meas_i` against `t_current_o`. Routes to in-order or OOSM path. |
| **LOAD_MEAS** | 4 cycles | Writes measurement data `z[0..3]` to RF registers r28–r31 via direct RF write port. Also writes measurement noise `R[0..3]` to r32–r35 from a lookup table indexed by `event_type_i`. |
| **RUN_PREDICT** | Variable | Asserts `seq_start_o` with `start_addr` pointing to the predict+update microcode entry. Waits for `seq_done_i`. |
| **CHECKPOINT** | 27–28 cycles | Commands LSU to burst-save the updated context to the oldest journal slot (circular overwrite). Updates journal metadata with the new timestamp. |
| **OOSM_SCAN** | 1–8 cycles | Sequentially reads journal metadata timestamps to find the closest epoch $\tau \le t_{meas}$. |
| **OOSM_SAVE** | 27 cycles | Burst-saves the current active context to a temporary slot before rewinding. |
| **OOSM_RESTORE** | 28 cycles | Burst-restores the historical context at epoch $\tau$ from the journal's saved slot. |
| **OOSM_CORRECT** | Variable | Runs the measurement correction microcode at epoch $\tau$ using the late measurement. |
| **OOSM_FORWARD** | Variable | Re-propagates the state forward from $\tau$ to $t_{current}$ by replaying buffered IMU deltas. May involve multiple predict steps. Returns to IDLE when caught up. |

## 4. Timestamp Arithmetic

All timestamps are unsigned 32-bit integers representing microseconds since system boot. The temporal controller performs only two operations on timestamps:

1. **Comparison:** `t_meas >= t_current` determines in-order vs. OOSM. This is a simple unsigned comparison.
2. **Difference:** `t_current - t_meas` gives the OOSM lateness in microseconds. Used only for diagnostic reporting, not for any data-path computation.

**Wrap-around:** At 32 bits, the counter wraps after ~4295 seconds (~71 minutes). For a typical UAV mission of 15–30 minutes, this is not a concern. If wrap-around must be handled, use modular arithmetic: `(a - b)` is correct as unsigned subtraction as long as the true difference is less than $2^{31}$.

## 5. Journal Management

The journal occupies TIDE-RAM addresses 0x0D1–0x0FF (47 words). It stores metadata for up to 8 context slots:

| Word Offset | Content |
|---|---|
| 0 | Slot 0 timestamp (µs) |
| 1 | Slot 0 validity flags (bit 0 = valid, bits [3:1] = measurement type) |
| 2 | Slot 1 timestamp |
| 3 | Slot 1 validity flags |
| ... | ... |
| 14 | Slot 7 timestamp |
| 15 | Slot 7 validity flags |
| 16 | Head pointer (oldest slot index, 3-bit) |
| 17 | Tail pointer (next-write slot index, 3-bit) |
| 18 | Count (number of valid entries, 0–8) |

### Eviction Policy

The journal uses a **circular FIFO** eviction policy. When all 8 slots are full and a new checkpoint is needed, the oldest slot (head pointer) is overwritten. This means the TIDE engine can perform OOSM replay for measurements that are at most 8 estimation epochs old. For typical 50 Hz GPS updates, this gives a ~160 ms OOSM window — sufficient for GPS/INS sensor fusion where GPS typically lags by 50–100 ms.

### OOSM Scan Algorithm

When a late measurement arrives at time $t_{meas}$:

1. Start at the tail pointer (most recent entry).
2. Walk backwards through the circular buffer.
3. For each valid slot, compare its timestamp against $t_{meas}$.
4. Stop when `slot.timestamp <= t_meas`. This slot is the restore target.
5. If no valid slot has `timestamp <= t_meas`, the measurement is too old. Discard it and set an error flag.

Worst case: 8 comparisons = 8 cycles. The scan reads one journal metadata word per cycle via `jrnl_addr_o`.

## 6. Measurement Data Loading

Before launching the sequencer, the temporal controller writes measurement data directly to the register file:

| Cycle | RF Address | Data Source |
|---|---|---|
| 0 | r28 | `event_z0_i` (range or position X) |
| 1 | r29 | `event_z1_i` (azimuth or position Y) |
| 2 | r30 | `event_z2_i` (elevation or position Z) |
| 3 | r31 | `event_z3_i` (Doppler or velocity) |

The measurement noise values `R[0..3]` for registers r32–r35 are loaded from a small 4×4 lookup table (4 measurement types × 4 noise values) hardcoded in the temporal controller. These values are set by firmware via MMIO configuration registers before the engine starts.

## 7. Microcode Entry Points

The temporal controller selects the sequencer's start address based on the operation:

| Operation | Entry Address | Description |
|---|---|---|
| In-order predict + update | `9'h000` | Full EKF cycle: propagate state, compute Jacobian, update covariance, apply correction |
| Predict only (no measurement) | `9'h040` | Time propagation without measurement correction (used during OOSM forward replay) |
| Correction only | `9'h080` | Apply measurement correction at a historical epoch (used during OOSM late update) |

## 8. OOSM Replay Cycle Budget

The worst-case OOSM replay scenario (measurement 8 epochs late, full forward propagation):

| Phase | Cycles | Description |
|---|---|---|
| Classify | 1 | Timestamp comparison |
| OOSM Scan | 8 | Walk all 8 journal slots |
| Save current context | 27 | Burst save to temp slot |
| Restore historical context | 28 | Burst load from journal slot |
| Load late measurement | 4 | Write z[0..3] to RF |
| Run correction microcode | ~400 | EKF correction at epoch τ |
| Forward propagation (×8) | ~8 × 300 | Predict-only for each intermediate epoch |
| Restore saved context | 28 | (If forward replay updates current context in place, this is skipped) |
| **Total worst case** | **~2,896 cycles** | **57.9 µs at 50 MHz** |

This is well within the 400 µs OOSM budget. Even with conservative margins, a single OOSM event takes less than 60 µs.

## 9. Area Estimate

| Component | Gate Count |
|---|---|
| FSM (9 states, 4-bit encoded) | ~200 gates |
| Timestamp comparator (32-bit) | ~250 gates |
| Journal scan pointer + counter | ~150 gates |
| Measurement type LUT (4×4×32) | ~400 gates |
| Control muxes and handshake | ~300 gates |
| Configuration registers (noise values) | ~500 gates |
| **Total** | **~1,800 gates** |

On FPGA: approximately 120 LUTs, 80 flip-flops. Negligible compared to the FP lanes.

## 10. Verification Plan

| Test | Method | Pass Criterion |
|---|---|---|
| In-order measurement: load event, run predict+update | Verilator | Sequencer runs correct entry point, RF updated |
| OOSM measurement: inject late event | Verilator | Correct scan, save, restore, correct, forward sequence |
| Journal full eviction: fill 8 slots, add 9th | Verilator | Oldest slot overwritten, head pointer advances |
| Too-old measurement: event older than all journal entries | Verilator | Error flag set, event discarded |
| Back-to-back in-order events | Verilator | Checkpoint saved between each, timestamps monotonic |
| OOSM during OOSM (nested): second late event during forward replay | Verilator | Handled correctly or queued (spec says: queue in FIFO) |
| Measurement data loading: verify r28–r31 contain correct values | Verilator | Bit-exact match |
| Timestamp wrap-around at 32-bit boundary | Verilator | Correct comparison using unsigned arithmetic |

## 11. Implementation Notes for the Coder

1. **The FSM should be one-hot encoded** for timing closure in 180nm. With 9 states, a binary encoding (4 bits) saves area but the decode logic is slower. One-hot (9 bits) has faster transitions and cleaner synthesis.

2. **The journal scan loop must be fully unrollable.** Do not use a counter that conditionally exits — use a fixed 8-iteration scan with early-exit gating. This makes the scan latency deterministic (always 8 cycles, even if the match is found early). The extra cycles are worth the simplicity.

3. **Measurement noise values** (`R[0..3]` per measurement type) should be stored in MMIO-writable configuration registers, not hardcoded constants. This allows the flight controller firmware to tune noise parameters without re-synthesizing the design.

4. **The forward replay loop** is the most complex part. After correcting at epoch $\tau$, the temporal controller must re-issue predict-only entries for each subsequent journal epoch. Track the replay progress with a dedicated replay pointer that walks from the OOSM slot forward to the current head.

5. **Import `tide_pkg`** for microcode entry point addresses (`ENTRY_PREDICT_UPDATE`, `ENTRY_PREDICT_ONLY`, `ENTRY_CORRECT_ONLY`). These are parameters, not magic numbers.

6. **Reset behavior:** On reset, `t_current_o = 0`, all journal slots invalid, `busy_o = 0`, `event_ready_o = 1`, `oosm_active_o = 0`, FSM in IDLE. The first event after reset initializes the estimation epoch.
