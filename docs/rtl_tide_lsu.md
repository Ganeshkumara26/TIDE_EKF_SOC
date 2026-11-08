# RTL Development & Architectural Implementation Document: `tide_lsu.sv`
**Module Name:** `tide_lsu`  
**Design Reference:** `09_tide_lsu.md`, `18_storage_map.md`, `15_timing_contracts.md`, `STEP_0_DECISIONS.md`  
**Target Architectures:** OS0005 Avionics SoC (SCL 180nm CP8D), OS0035 Micro-Navigation Processor (ASAP7 FinFET)  
**Verification Coverage:** Formal assertions (`tide_lsu_sva.sv`), Directed unit testbench (`tb_tide_lsu.sv`), Post-synthesis timing simulation  

---

## 1. Architectural Role & Storage Hierarchy Gateway

The Load/Store Unit (`tide_lsu`) is the dedicated memory gateway for the TIDE Extended Kalman Filter (EKF) coprocessor. In autonomous aerospace flight controllers such as the OS0005 Avionics SoC, estimation algorithms operate across a strictly partitioned two-level memory hierarchy:
1. **L0 Active Context:** A 128×32-bit multi-ported flip-flop register file (`tide_regfile`) providing single-cycle combinational read and synchronous write access for active vector-matrix floating-point execution.
2. **L1 Historical Scratchpad:** A 4 KB (1024×32-bit) single-cycle synchronous SRAM (`tide_ram`) providing dense, low-power storage for committed state checkpoints, historical measurement journals, and ring buffers.

The `tide_lsu` bridges these two storage levels by servicing two orthogonal operational demands:
- **Fine-Grained Microcode Access:** Servicing single-word indexed loads and stores dispatched by the VLIW sequencer's LSU/ALU slot during nominal Kalman time and measurement updates.
- **Coarse-Grained Autonomous Context Bursts:** Executing multi-word burst streaming transfers commanded by the temporal coordinator (`tide_temporal`) during out-of-sequence measurement (OOSM) rollbacks, historical context restores, and forward state replays.

```
                   ┌─────────────────────────────────────────────────┐
                   │                    tide_lsu                     │
                   │                                                 │
  Sequencer ──────►│  [Single-Word Pipeline]                         │
  (LSU/ALU Slot)   │    - 1-Cycle STORE (Fire-and-Forget)            │
                   │    - 2-Cycle LOAD  (RAM Read -> RF Writeback)   │
                   │                                                 │
  Temporal ───────►│  [Burst Transfer Engine]                        │
  Coordinator      │    - Auto-Increment Address Generators          │
                   │    - Zero-Wait-State Burst SAVE   (N cycles)    │
                   │    - Pipelined Read   Burst RESTORE (N+1 cycles)│
                   └──────────┬───────────────────────────┬──────────┘
                              │                           │
                              ▼                           ▼
                  ┌──────────────────────┐    ┌──────────────────────┐
                  │     tide_regfile     │    │       tide_ram       │
                  │   (128x32 FF Array)  │    │   (1024x32 SRAM)     │
                  └──────────────────────┘    └──────────────────────┘
```

---

## 2. Hard Real-Time OOSM Replay Latency Constraints

The architectural specification of the OS0005 flight computer establishes a rigid upper bound on measurement latency:
- Sensor packets from asynchronous interfaces (e.g., radar altimeter, Doppler radar, GPS receivers) may arrive delayed by up to 8 epochs relative to current navigation time.
- Upon detecting an out-of-sequence measurement, `tide_temporal` halts the current forward propagation, saves the active state into the shadow context buffer, restores the nearest historical checkpoint prior to $t_{\text{meas}}$, inserts the delayed measurement into the temporal sequence, re-evaluates corrections, and fast-forwards through the journal ring.
- **Latency Budget:** The worst-case OOSM replay sequence must complete in $< 400\ \mu\text{s}$ at $f_{\text{clk}} = 50\text{ MHz}$ ($20,000\text{ clock cycles}$).

Because context swapping occurs repeatedly across multi-step replays, the burst transfer engine must eliminate memory stall cycles:
- **Packed EKF Context Size:** 6 state variables $\mathbf{x} = [p_n, p_e, p_d, v_n, v_e, v_d]^T$ plus 21 lower-triangular covariance elements $\mathbf{P} \in \mathbb{R}^{6 \times 6}$, yielding 27 words (or 32 words in 128-byte aligned mode including timestamps and journal pointers).
- **Burst SAVE Target:** Exactly 27 clock cycles for 27 words ($540\text{ ns}$ at 50 MHz).
- **Burst RESTORE Target:** Exactly 28 clock cycles for 27 words ($560\text{ ns}$ at 50 MHz, with 1 cycle initial read pipeline fill).

---

## 3. Detailed Interface & Signal Contracts

The module interface adheres strictly to the frozen port definitions:

```systemverilog
module tide_lsu (
    input  logic        clk_i,
    input  logic        rst_ni,

    // Single-word command interface (from Sequencer LSU_ALU slot)
    input  logic        cmd_valid_i,
    input  logic        cmd_we_i,          // 1 = STORE (RF -> RAM), 0 = LOAD (RAM -> RF)
    input  logic [6:0]  cmd_reg_i,         // RF register index (0..127)
    input  logic [11:0] cmd_addr_i,        // RAM word address (0..1023)
    output logic        cmd_ready_o,

    // Burst command interface (from Sequencer or Temporal Coordinator)
    input  logic        burst_start_i,
    input  logic        burst_we_i,        // 1 = RF -> RAM (SAVE), 0 = RAM -> RF (RESTORE)
    input  logic [6:0]  burst_rf_base_i,   // Starting register address
    input  logic [11:0] burst_ram_base_i,  // Starting TIDE-RAM word address
    input  logic [5:0]  burst_len_i,       // Number of words (1..63; 0 = NOP)
    output logic        burst_busy_o,
    output logic        burst_done_o,

    // Register file interface
    output logic [6:0]  rf_raddr_o,        // Combinational RF read address (STORE)
    input  logic [31:0] rf_rdata_i,        // Combinational RF read data
    output logic        rf_wen_o,          // Synchronous RF write enable (LOAD)
    output logic [6:0]  rf_waddr_o,        // RF write destination address
    output logic [31:0] rf_wdata_o,        // RF writeback data

    // TIDE-RAM interface (Single-port synchronous macro)
    output logic        ram_ce_o,          // Chip enable
    output logic        ram_we_o,          // Write enable (1=write, 0=read)
    output logic [11:0] ram_addr_o,        // Word address
    output logic [31:0] ram_wdata_o,       // Write data bus
    input  logic [31:0] ram_rdata_i        // Read data bus (valid cycle after ce & ~we)
);
```

---

## 4. Single-Word Operational Mechanics

### 4.1 Single-Word STORE ($RF \to RAM$)
A store operation is executed within a single clock cycle with zero wait-states (fire-and-forget).
1. **Cycle $C$:** The sequencer decodes a STORE instruction and asserts `cmd_valid_i = 1`, `cmd_we_i = 1`, `cmd_reg_i = r_src`, and `cmd_addr_i = addr`.
2. The combinational datapath immediately drives `rf_raddr_o = cmd_reg_i`.
3. Because `tide_regfile` provides combinational read ports, `rf_rdata_i` is valid within cycle $C$.
4. The LSU sets `ram_ce_o = 1`, `ram_we_o = 1`, `ram_addr_o = cmd_addr_i`, and `ram_wdata_o = rf_rdata_i`.
5. On the rising clock edge of cycle $C+1$, the SRAM macro latches the address and data into the memory array.
6. The sequencer experiences no pipeline stalls (`cmd_ready_o = 1`).

### 4.2 Single-Word LOAD ($RAM \to RF$)
A load operation is a 2-cycle pipelined sequence with single-cycle throughput under continuous streaming:
1. **Issue Cycle ($C$):** The sequencer asserts `cmd_valid_i = 1`, `cmd_we_i = 0`, `cmd_reg_i = r_dst`, and `cmd_addr_i = addr`. The LSU asserts `ram_ce_o = 1`, `ram_we_o = 0`, and `ram_addr_o = cmd_addr_i`.
2. **Bookkeeping:** The LSU samples the pending request into internal tracking registers:
   ```systemverilog
   if (cmd_fire && !cmd_we_i && !(act && !zero_len)) begin
       pend_q     <= 1'b1;
       pend_reg_q <= cmd_reg_i;
   end
   ```
3. **Writeback Cycle ($C+1$):**
   - The SRAM macro presents `ram_rdata_i` at the clock edge.
   - The LSU observes `pend_q == 1` and drives `rf_wen_o = 1`, `rf_waddr_o = pend_reg_q`, and `rf_wdata_o = ram_rdata_i`.
   - On the rising edge of cycle $C+2$, `tide_regfile` commits the data into register `pend_reg_q`.
   - The sequencer's static writeback scheduler guarantees that write port 2 is reserved for the LSU on cycle $C+1$.

```
Clock         ____/‾‾‾‾\____/‾‾‾‾\____/‾‾‾‾\____/‾‾‾‾\____/‾‾‾‾\____
Cycle                Cycle C                  Cycle C+1
cmd_valid_i   ______/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\__________________________
cmd_we_i      ______________________________________________________ (0 = LOAD)
ram_addr_o    XXXXXX[      0x0240        ]XXXXXXXXXXXXXXXXXXXXXXXXXX
ram_ce_o      ______/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\__________________________
pend_q        ____________________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\____
ram_rdata_i   XXXXXXXXXXXXXXXXXXXXXXXXXXXX[      Data_Word         ]
rf_wen_o      ____________________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\____
rf_waddr_o    XXXXXXXXXXXXXXXXXXXXXXXXXXXX[      cmd_reg_i         ]
rf_wdata_o    XXXXXXXXXXXXXXXXXXXXXXXXXXXX[      Data_Word         ]
```

---

## 5. Burst Context Transfer Engine

The burst transfer engine achieves zero-overhead multi-word transfers by utilizing an instantaneous parameter latching scheme on Cycle 0 combined with dedicated address indexers.

### 5.1 Cycle-0 Combinational Parameter Consumption
Conventional burst DMA controllers require an idle handshake cycle to capture parameters before initiating memory transfers. In `tide_lsu`, parameter consumption is instantaneous. By defining effective parameters combinationally, Cycle 0 of the burst performs the first transfer simultaneously with parameter registration:

```systemverilog
wire        start_e = burst_start_i & ~busy_q;
wire        act     = start_e | busy_q;
wire        we_e    = busy_q ? we_q  : burst_we_i;
wire [6:0]  rfb_e   = busy_q ? rfb_q : burst_rf_base_i;
wire [11:0] rmb_e   = busy_q ? rmb_q : burst_ram_base_i;
wire [5:0]  len_e   = busy_q ? len_q : burst_len_i;
wire [5:0]  k_e     = busy_q ? k_q   : 6'd0;
```

This ensures that address calculation logic and memory drive multiplexers immediately activate on `burst_start_i`, eliminating bubble cycles.

### 5.2 Base + Offset Addressing Datapath
The datapath dynamically generates memory and register addresses via integer offsets:
- **TIDE-RAM Word Address:** $\text{ram\_addr} = \text{rmb\_e} + k_e$
- **Register File Read Address (Burst SAVE):** $\text{rf\_raddr} = \text{rfb\_e} + k_e$
- **Register File Writeback Address (Burst RESTORE):** $\text{rf\_waddr} = \text{rfb\_e} + (k_e - 1)$

```systemverilog
wire [11:0] burst_ram_addr = rmb_e + {6'd0, k_e};
wire [6:0]  burst_rf_st    = rfb_e + {1'b0, k_e};            // Store: RF word read
wire [6:0]  burst_rf_ld    = rfb_e + {1'b0, k_e} - 7'd1;     // Load : RF word written (k>=1)
```

### 5.3 Burst Context SAVE ($RF \to RAM$) Pipeline Flow
During a save operation (`burst_we_i = 1`), data flows from the register file combinational read port directly into the SRAM write bus:
- **Cycle 0 ($k=0$):** `burst_start_i` asserted. LSU reads `rf[rfb_e + 0]` and presents data to SRAM at `rmb_e + 0` with `ram_we_o = 1`.
- **Cycle $k$ ($0 < k < \text{len}-1$):** Index counter increments: $k_q \leftarrow k_q + 1$. Word $k$ is read from RF and written to RAM.
- **Cycle $\text{len}-1$ ($k = \text{len}-1$):** Final word is written to RAM. The LSU asserts `burst_done_o = 1` and resets `busy_q <= 1'b0`.
- **Total Duration:** Exactly $\text{len}$ clock cycles ($27\text{ cycles}$ for a standard EKF context).

### 5.4 Burst Context RESTORE ($RAM \to RF$) Pipeline Flow
During a restore operation (`burst_we_i = 0`), the SRAM read latency introduces a 1-cycle pipeline delay:
- **Cycle 0 ($k=0$):** `burst_start_i` asserted. SRAM read issued for `rmb_e + 0`. RF write enable remains disabled (`k_e == 0`).
- **Cycle $k$ ($1 \le k < \text{len}$):** SRAM read issued for `rmb_e + k`. Simultaneously, `ram_rdata_i` returning from word $k-1$ is written into the register file at `rf_waddr_o = rfb_e + (k - 1)` with `rf_wen_o = 1`.
- **Cycle $\text{len}$ ($k = \text{len}$):** No new SRAM read is issued (`k_e == len_e`). The final data word ($\text{len}-1$) is committed into the register file. The LSU asserts `burst_done_o = 1` and clears `busy_q`.
- **Total Duration:** Exactly $\text{len} + 1$ clock cycles ($28\text{ cycles}$ for a standard 27-word context).

```
Cycle      0        1        2               len-1      len
RAM Read   Word 0   Word 1   Word 2   ...    Word len-1 Idle
RF Write   Idle     Word 0   Word 1   ...    Word len-2 Word len-1
                                                        ▲ burst_done_o
```

---

## 6. Arbitration & Request Validation Rules

To prevent datapath collisions, the LSU enforces strict operational constraints:
1. **Preemptive Priority:** Burst operations have unconditional priority over single-word commands. When `busy_q` is high, `cmd_ready_o` deasserts (`cmd_ready_o = ~busy_q`). Any incoming `cmd_valid_i` during a burst is stalled.
2. **Mutual Exclusion Assertion:** SVA rules (`tide_lsu_sva.sv`) guarantee that the sequencer never issues `cmd_valid_i` concurrently with `burst_start_i`.
3. **Zero-Length Trap Handling:** If `burst_len_i == 6'd0`, the unit treats the command as an immediate NOP:
   ```systemverilog
   wire zero_len = (len_e == 6'd0);
   wire last = zero_len ? 1'b1 : (we_e ? (k_e == len_e - 6'd1) : (k_e == len_e));
   ```
   When `zero_len` is high, memory enables are suppressed, `burst_done_o` pulses high on Cycle 0, and the unit transitions back to IDLE without corrupting memory contents.

---

## 7. Physical Synthesis & Standard Cell Library Considerations

### 7.1 SRAM Read-Data Critical Path
In both SCL 180nm CP8D and ASAP7 FinFET, the critical timing path through `tide_lsu` originates at the output pins of the SRAM macro:
$$\text{Clock} \to \text{RAM } t_{\text{cq}} \to \text{ram\_rdata\_i} \to \text{LSU Mux} \to \text{rf\_wdata\_o} \to \text{RF Setup}$$

To prevent setup time violations at 50 MHz under slow-slow process corners ($1.62\text{ V}, 125^\circ\text{C}$):
- `rf_wdata_o` connects directly to `ram_rdata_i` without intermediate arithmetic manipulation or masking:
  ```systemverilog
  assign rf_wdata_o = ram_rdata_i;
  ```
- No combinational arithmetic or barrel-shifting is permitted between SRAM and RF writeback buses.

### 7.2 Power Minimization via Chip-Enable Gating
Standard-cell SRAM compilers generate sense amplifiers that consume significant dynamic power during read cycles. The LSU ensures `ram_ce_o` is strictly asserted only during valid transfer cycles:
- During idle states, `ram_ce_o = 1'b0`.
- On single-cycle accesses, `ram_ce_o` pulses for exactly 1 cycle.
- On the final cycle of a burst RESTORE ($k = \text{len}$), `ram_ce_o` deasserts to prevent speculative reads beyond the context boundary, avoiding unnecessary dynamic power dissipation.

---

## 8. Verification Defects, Root Cause Analysis, and Engineering Fixes

### 8.1 Defect LSU-01: Burst RESTORE Destination Off-By-One Address Aliasing
- **Symptom:** During gate-level verification of OOSM context replay, state vector element $x[0]$ remained uninitialized in `tide_regfile`, while $x[1]$ through $x[5]$ received values offset by one register position ($x[0]$ value written into $x[1]$).
- **Root Cause:** An early implementation shared the address generation equation for both store and load:
  ```systemverilog
  // DEFECTIVE: Used same offset for read issue and writeback
  assign rf_waddr_o = rfb_e + {1'b0, k_e};
  ```
  Because the SRAM read pipeline has a 1-cycle latency, `ram_rdata_i` arriving at Cycle 1 corresponded to word 0, but was written to `rfb_e + 1`.
- **Engineering Fix:** Created separate combinational address decoders for store read (`burst_rf_st`) and load write (`burst_rf_ld`), explicitly subtracting 1 from the load destination index:
  ```systemverilog
  wire [6:0] burst_rf_ld = rfb_e + {1'b0, k_e} - 7'd1;
  ```
  RF write enable was additionally qualified with `if (k_e != 6'd0)` to suppress bogus writes during Cycle 0.

### 8.2 Defect LSU-02: Counter Underflow Deadlock on Zero-Length Burst Requests
- **Symptom:** When the temporal coordinator issued a dynamic context flush with `burst_len_i = 0`, the LSU locked into a permanent busy state (`burst_busy_o = 1`), hanging the entire coprocessor.
- **Root Cause:** The termination condition evaluated `(k_e == len_e - 1)`. For `len_e = 0`, the right-hand side wrapped around in unsigned 6-bit arithmetic to `6'd63`. The counter iterated indefinitely through unmapped address space.
- **Engineering Fix:** Introduced the explicit `zero_len = (len_e == 6'd0)` comparator, directly asserting `last = 1'b1` during Cycle 0 to abort the transaction cleanly in a single cycle.

### 8.3 Defect LSU-03: Single-Word LOAD Collision with Preempting Burst
- **Symptom:** When a single LOAD was issued immediately before a temporal controller context save, the single load's writeback at cycle $C+1$ corrupted register `cmd_reg_i` with burst store data.
- **Root Cause:** `pend_q` asserted `rf_wen_o` concurrently with the burst store operation.
- **Engineering Fix:** Prioritized burst activity over pending single loads:
  ```systemverilog
  if (act && !zero_len) begin
      // Burst mode has absolute ownership of RF and RAM ports
  end else if (pend_q) begin
      rf_wen_o   = 1'b1;
      rf_waddr_o = pend_reg_q;
  end
  ```
  This guarantees structural mutual exclusion between single and burst writeback pipelines.
