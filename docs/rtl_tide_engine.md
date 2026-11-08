# RTL Development & Architectural Implementation Document: `tide_engine.sv`
**Module Name:** `tide_engine` (Top-Level Autonomous EKF Coprocessor)  
**Design Reference:** `13_tide_top.md`, `18_storage_map.md`, `15_timing_contracts.md`, `STEP_0_DECISIONS.md`  
**Target Architectures:** OS0005 Avionics SoC (SCL 180nm CP8D), OS0035 Micro-Navigation Processor (ASAP7 FinFET)  
**Verification Coverage:** Top-level formal assertions (`tide_engine_sva.sv`), Full EKF mission flight simulations (`tb_tide_engine.sv`), Gate-level post-P&R timing signoff  

---

## 1. Architectural Role & Autonomous Coprocessor Philosophy

The `tide_engine` module is the top-level structural integration binding all computational, sequencing, storage, and temporal control submodules of the TIDE Extended Kalman Filter (EKF) accelerator into a unified, autonomous coprocessor.

In high-reliability aerospace avionics such as the OS0005 SoC, modern inertial navigation systems demand continuous, sub-millisecond multi-sensor fusion across noisy, asynchronous data streams:
- High-rate IMU delta velocities and rotations ($\Delta \mathbf{v}, \Delta \boldsymbol{\theta}$) arriving at 200–1000 Hz.
- Moderate-rate GNSS position and Doppler measurements arriving at 10–50 Hz.
- Asynchronous radar altimeter and optical flow updates subject to variable sensor processing latencies (producing out-of-sequence measurements).

Offloading these matrix-intensive filtering operations to a general-purpose processor (e.g. RISC-V RV32IM) introduces severe CPU utilization bottlenecks and timing jitter. `tide_engine` delivers a fully autonomous hardware solution. Once initialized by the host CPU, the engine operates independently:
1. It ingests sensor packets via a hardware-managed handshake (`event_valid_i` / `event_ready_o`).
2. It evaluates measurement timestamps, determining whether an event is in-order or an out-of-sequence measurement (OOSM).
3. For in-order events, it orchestrates kinematic state propagation and measurement innovation updates directly in hardware.
4. For late-arriving events, it autonomously initiates historical checkpoint rollbacks, restores prior states from dense SRAM, inserts the delayed measurement into the temporal journal, updates covariance estimates, and fast-forwards through subsequent events before resuming real-time tracking.
5. It emits updated navigation states ($\mathbf{x} \in \mathbb{R}^6$) and diagonal covariance uncertainties ($\text{diag}(\mathbf{P})$) to system registers, asserting completion interrupts without intermediate CPU intervention.

```
┌────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                              tide_engine                                               │
│                                                                                                        │
│   ┌────────────────────────┐      ┌─────────────────────────┐      ┌───────────────────────────────┐   │
│   │     tide_temporal      │◄────►│     tide_sequencer      │◄────►│         tide_ucode_ram        │   │
│   │ (Temporal Coordinator) │      │   (5-Slot VLIW Engine)  │      │     (512 x 96 Microcode)      │   │
│   └───────────┬────────────┘      └────────────┬────────────┘      └───────────────────────────────┘   │
│               │                                │                                                       │
│               │  ┌─────────────────────────────┼─────────────────────────────┐                         │
│               │  │                             │                             │                         │
│               ▼  ▼                             ▼                             ▼                         │
│   ┌────────────────────────┐      ┌─────────────────────────┐      ┌───────────────────────────────┐   │
│   │        tide_ram        │      │      tide_regfile       │      │       Execution Lanes         │   │
│   │     (4 KB Scratchpad   │◄────►│    (128 x 32 4R/3W)     │◄────►│  - tide_fp32_mul (2-Cyc)      │   │
│   │      SRAM + Arbiter)   │      │                         │      │  - tide_fp32_add (3-Cyc / Byp)│   │
│   └───────────▲────────────┘      └─────────────────────────┘      │  - tide_fp32_divsqrt (Radix-2)│   │
│               │                                                    │  - tide_const_rom (64 x 32)   │   │
│   ┌───────────┴────────────┐                                       └───────────────────────────────┘   │
│   │        tide_lsu        │                                                                           │
│   │   (Load/Store Unit &   │                                                                           │
│   │     Burst Engine)      │                                                                           │
│   └────────────────────────┘                                                                           │
└────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Temporal Coordinator and VLIW Sequencer Control Handshake

The coordination between `tide_temporal` (coarse-grained algorithmic manager) and `tide_sequencer` (fine-grained instruction dispatcher) is mediated by a 5-wire deterministic handshake protocol:

```mermaid
sequenceDiagram
    participant TC as tide_temporal
    participant SEQ as tide_sequencer
    participant LSU as tide_lsu

    Note over TC,SEQ: Idle state: Engine waiting for measurement event
    TC->>SEQ: seq_start_o = 1, seq_start_addr_o = UNIT_ENTRY (0x000)
    Note over SEQ: Startup bubble: Warm-up cycle (warm_q = 1)
    SEQ-->>TC: seq_running_o = 1
    Note over SEQ: Fetch-Decode-Execute pipeline running VLIW instructions
    opt Preemption Boundary
        SEQ-->>TC: seq_boundary_o = 1 (Microcode safe-pause point)
    end
    Note over SEQ: Execution reaches HALT instruction
    SEQ-->>TC: seq_done_o = 1
    SEQ-->>TC: seq_running_o = 0
    Note over TC,LSU: Temporal controller triggers burst checkpoint snapshot
    TC->>LSU: burst_start_o = 1 (SAVE 27 words to TIDE-RAM)
    LSU-->>TC: burst_done_i = 1 (27 cycles later)
```

1. **`seq_start` / `seq_start_addr`:** Single-cycle pulse initiating execution. For nominal Kalman propagation, `seq_start_addr` points to `UNIT_ENTRY` (`9'd0`). For innovation calculation without propagation, it points to `QUERY_ENTRY` (`9'd471`).
2. **`seq_running`:** High whenever the instruction sequencer is actively processing microcode or draining in-flight arithmetic operations.
3. **`seq_boundary`:** Emitted when microcode passes an explicit preemption point (`BC` bit in CTRL slot), informing `tide_temporal` that register states are mathematically coherent for checkpointing.
4. **`seq_done`:** Single-cycle pulse indicating the microcode has encountered a `HALT` opcode and all arithmetic pipelines are fully drained.

---

## 3. The Six Critical Integration Decisions (I1 through I6)

In modular hardware development, individual leaf modules must make baseline operational assumptions to freeze their pin interfaces. The top-level `tide_engine` synthesizes these decoupled contracts into a fully coherent system by implementing six critical integration mechanisms:

### 3.1 Decision I1: Register File Write Port 2 Multi-Source Arbitration
The register file (`tide_regfile`) provides exactly three physical synchronous write ports:
- **Port 0:** Dedicated to the pipelined floating-point multiplier (`tide_fp32_mul`).
- **Port 1:** Shared between the floating-point adder (`tide_fp32_add`) and iterative divider/square-root (`tide_fp32_divsqrt`).
- **Port 2:** Shared across three functional producers:
  1. `tide_temporal`: Writes incoming event measurements, noise variance parameters, and timestamps when seeding the register file while the sequencer is idle.
  2. `tide_sequencer`: Writes results from inline integer ALU operations and constant ROM lookups during microcode execution.
  3. `tide_lsu`: Writes memory load data returning from single-word SRAM loads.

By design construction, these three producers operate in distinct operational phases: the temporal controller writes while the sequencer is halted; the sequencer executes integer arithmetic on non-memory cycles; the LSU asserts writebacks following explicit load instructions. However, to guard against microcode scheduling faults, `tide_engine` implements a strict hardware priority hierarchy:
$$\text{Temporal Controller } (p2\_tc) > \text{Sequencer ALU } (p2\_seq) > \text{LSU Writeback } (p2\_lsu)$$

```systemverilog
wire p2_tc  = tc_rf_wen;
wire p2_seq = sw2;
wire p2_lsu = l_wen;

wire w2_en  = p2_tc | p2_seq | p2_lsu | seq_start;
wire [6:0]  w2_a = seq_start ? 7'd122 : p2_tc ? tc_rf_waddr : p2_seq ? sa2 : l_waddr;
wire [31:0] w2_d = seq_start ? dt0inv_i : p2_tc ? tc_rf_wdata : p2_seq ? sd2 : l_wdata;

always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) 
        rf_conflict_o <= 1'b0;
    else if (((p2_tc & p2_seq) | (p2_tc & p2_lsu) | (p2_seq & p2_lsu)) |
             (seq_start & (sw0 | sw1 | p2_tc | p2_seq | p2_lsu)))
        rf_conflict_o <= 1'b1;
end
```
Any simultaneous assertion sets the sticky integration error flag `rf_conflict_o`, ensuring that timing hazards during firmware development are immediately visible to flight diagnostics.

### 3.2 Decision I2: Dynamic Read Port 2 Steering for LSU Stores
The register file provides four combinational read ports (`ra0`–`ra3`). Execution lanes consume all four ports: Port 0 and 1 drive the multiplier and divider, while Ports 2 and 3 drive the adder and comparator. The LSU possesses **no dedicated register read port**.

Integration decision I2 implements a time-shared multiplexing architecture governed by sequencer activity (`engine_owns`):
- **During Sequencer Execution (`engine_owns = 1`):** The sequencer's LSU/ALU slot decodes single-word store instructions. It dynamically routes the source register address to Port 2 (or Port 3 if Port 2 is occupied by the adder, per rule R2). The sequencer resolves this combinationally and emits `seq_lsu_rdata`. The LSU samples this bus directly:
  ```systemverilog
  assign rf_rdata_i = engine_owns ? seq_lsu_rdata : rd2;
  ```
- **During Sequencer Quiescence (`engine_owns = 0`):** The temporal controller triggers autonomous multi-word burst stores to save EKF contexts into TIDE-RAM. With the sequencer halted, the LSU assumes direct physical ownership of register read address Port 2:
  ```systemverilog
  wire [6:0] ra2 = engine_owns ? ra2_seq : l_rf_raddr;
  ```

### 3.3 Decision I3: Simultaneous Triple-Port Injection of Microcode ABI Parameters
The microcode EKF kernel requires three floating-point operational parameters: process noise position variance (`QP`), velocity variance (`QV`), and the inverse nominal sample time ($\Delta t_0^{-1}$, `DT0INV`). These live in dedicated architectural registers:
- `r120`: `QP`
- `r121`: `QV`
- `r122`: `DT0INV`

Rather than consuming three cycles of temporal controller state machine execution or requiring dedicated microcode load instructions, `tide_engine` exploits the pipeline bubble present during the engine launch pulse:
1. When `tide_temporal` asserts `seq_start_o`, the sequencer is transitioning from IDLE to RUN.
2. In this cycle, all three register file write ports are idle.
3. `tide_engine` forces all three write enables active simultaneously, routing the CSR input parameters directly into the target registers in a **single clock cycle**:
   ```systemverilog
   wire w0_en = sw0 | seq_start;
   wire [6:0]  w0_a = seq_start ? 7'd120 : sa0;
   wire [31:0] w0_d = seq_start ? qp_i    : sd0;

   wire w1_en = sw1 | seq_start;
   wire [6:0]  w1_a = seq_start ? 7'd121 : sa1;
   wire [31:0] w1_d = seq_start ? qv_i    : sd1;

   wire [6:0]  w2_a = seq_start ? 7'd122 : ...;
   wire [31:0] w2_d = seq_start ? dt0inv_i : ...;
   ```

### 3.4 Decision I4: Direct Streaming Event FIFO Bypass
Earlier architectural sketches proposed an internal 4-deep event FIFO between the bus interface and the temporal coordinator. In the finalized silicon implementation, the FIFO was eliminated:
- Sensor packets are written directly into memory-mapped registers in the system bus wrapper.
- The wrapper drives `event_valid_i`, `event_t_i`, and measurement words `event_z0_i`–`event_z3_i` straight into `tide_temporal`.
- Flow control is regulated via the backpressure handshake signal `event_ready_o`.
- Omitting the FIFO eliminates 720 flip-flops, avoids clock-domain crossing synchronization latencies, and guarantees deterministic cycle counts from bus commit to filter execution.

### 3.5 Decision I5: Dual-Source LSU Burst Dispatch
While burst context transfers are predominantly commanded by the temporal coordinator during OOSM replays (`tc_bst`), the microcode VLIW instruction set provisions burst save/restore opcodes for user-defined matrix dumps. `tide_engine` wires a combinational OR across burst command lines:
```systemverilog
.burst_start_i   (tc_bst | s_bst),
.burst_we_i      (tc_bst ? tc_bwe  : s_bwe),
.burst_rf_base_i (tc_bst ? tc_brf  : s_brf),
.burst_ram_base_i(tc_bst ? tc_bram : s_bram),
.burst_len_i     (tc_bst ? tc_blen : s_blen),
```
Because the sequencer is halted during temporal burst operations, this union ensures zero conflict while retaining complete hardware flexibility.

### 3.6 Decision I6: Dual-Level Memory Protection & Host Gating
To ensure total fault containment during navigation execution:
1. **TIDE-RAM Write Protection:** CPU writes to the 4 KB scratchpad SRAM are qualified by coprocessor status: `cpu_wr_en_i = ~busy_o`. The host CPU can seed initial covariance matrices at system startup, but any write attempt during active filtering is physically blocked and latches `ram_conflict_o`.
2. **UCODE-RAM Write Lock:** Microcode uploads are gated by `enable_i`:
   ```systemverilog
   .cpu_we_i(uc_we_i & ~enable_i)
   ```
   Host attempts to patch microcode while the engine is enabled are discarded, preventing pipeline instruction corruption.

---

## 4. Submodule Interconnect & Datapath Architecture

The structural top-level binds seven discrete functional units:

### 4.1 Arithmetic Execution Lanes
- **Floating-Point Multiplier (`tide_fp32_mul`):** 2-cycle pipelined IEEE-754 multiplier. Operands arrive from RF Ports 0 and 1; writeback commits via RF Write Port 0 (`sa0`, `sd0`).
- **Floating-Point Adder/ALU (`tide_fp32_add`):** 3-cycle pipelined IEEE-754 adder/subtractor with a dedicated 1-cycle bypass path for `FABS`, `FNEG`, `FMOV`, and floating-point comparisons (`FCMP`). Writeback commits via RF Write Port 1.
- **Divider & Square-Root Unit (`tide_fp32_divsqrt`):** Iterative non-blocking radix-2 core executing single-precision division (28 cycles) and square-root (27 cycles). Shares Write Port 1 with the adder, utilizing a deterministic writeback scheduling slot.
- **Constant Mathematical ROM (`tide_const_rom`):** 64×32-bit combinational ROM providing floating-point mathematical constants: $\pi$, $\pi/2$, $2\pi$, $-\pi$, polynomial coefficients for minimax $\text{atan2}(y, x)$ approximation, and microsecond-to-second conversion factors ($10^{-6}$).

### 4.2 Temporal Coordinator (`tide_temporal`)
The autonomous brain of the coprocessor. It manages the 8-slot checkpoint ring and 64-entry measurement journal in `tide_ram`. It evaluates measurement timestamps against current estimator time, initiates historical context rollbacks via `tide_lsu`, and issues sequential microcode kernel execution pulses (`seq_start_o`) targeting entry points `UNIT_ENTRY` (`9'd0`) and `QUERY_ENTRY` (`9'd471`).

---

## 5. Fault Telemetry & Sticky Integration Error Flags

The engine features three hardware error flags designed to trap microcode scheduling bugs and bus protocol violations before flight:

```systemverilog
output logic ram_conflict_o,  // Multi-requester SRAM contention
output logic ram_oor_o,       // Unmapped SRAM address attempt (>= 1024)
output logic rf_conflict_o    // Register file Port 2 multi-driver hazard
```

1. **`ram_conflict_o`:** Latched if the temporal controller, LSU, or host CPU attempt to drive `tide_ram` simultaneously.
2. **`ram_oor_o`:** Latched if any requester addresses words beyond the 1024-word physical SRAM boundary (`addr[11:10] != 2'b00`).
3. **`rf_conflict_o`:** Latched if Port 2 write enables overlap or if `seq_start` coincides with active writebacks.

All three flags are sticky: once asserted, they remain high until the system asserts active-low reset (`rst_ni`), enabling diagnostic software to detect transient bus collisions that occurred during high-g flight maneuvers.

---

## 6. Physical Synthesis, Timing Closure & Standard Cell Constraints

### 6.1 Technology Node Comparison
The structural netlist was validated against two extreme physical implementations:
1. **OS0005 Avionics SoC (SCL 180nm CP8D):** 5-metal bulk planar CMOS, $1.8\text{ V}$ nominal core, targeting $50\text{ MHz}$ ($20.0\text{ ns}$ clock period).
2. **OS0035 Micro-Navigation Processor (ASAP7 FinFET):** 7nm FinFET PDK, $0.7\text{ V}$ nominal core, targeting $400\text{ MHz}$ ($2.5\text{ ns}$ clock period).

### 6.2 Top-Level Gate Count & Area Allocation (SCL 180nm)

| Hierarchical Instance | Leaf Module | Gate Count (GE) | Area ($180\text{nm}$) | Percentage |
|:---|:---|:---:|:---:|:---:|
| `u_rf` | `tide_regfile` | 48,768 | $0.390\text{ mm}^2$ | 52.8% |
| `u_mul` | `tide_fp32_mul` | 19,850 | $0.159\text{ mm}^2$ | 21.5% |
| `u_add` | `tide_fp32_add` | 17,620 | $0.141\text{ mm}^2$ | 19.1% |
| `u_ds` | `tide_fp32_divsqrt` | 11,400 | $0.091\text{ mm}^2$ | 12.3% |
| `u_seq` | `tide_sequencer` | 4,210 | $0.034\text{ mm}^2$ | 4.6% |
| `u_tc` | `tide_temporal` | 1,850 | $0.015\text{ mm}^2$ | 2.0% |
| `u_lsu` | `tide_lsu` | 720 | $0.006\text{ mm}^2$ | 0.8% |
| `u_rom` | `tide_const_rom` | 480 | $0.004\text{ mm}^2$ | 0.5% |
| Interconnect Muxes | Top-level steering | 1,250 | $0.010\text{ mm}^2$ | 1.3% |
| **Total Core Logic** | *(excluding SRAMs)* | **~106,148 GE** | **~0.850 mm²** | **100%** |

Both embedded memory blocks (`tide_ram` 4 KB and `tide_ucode_ram` 6 KB) are instantiated as hard macros placed at the perimeter, occupying an additional $0.62\text{ mm}^2$. The total die footprint conforms comfortably to the $1.5\text{ mm}^2$ coprocessor partition budget.

### 6.3 Critical Timing Paths & Placement Guidelines
Static timing analysis (STA) across worst-case process corners (SS, $1.62\text{ V}, 125^\circ\text{C}$) identified two critical paths:
1. **LSU Read-Data Writeback Path ($14.2\text{ ns}$):**
   $$\text{RAM Clock } \to \text{Macro } t_{\text{cq}} \to \text{LSU Bypass} \to \text{Port 2 Priority Mux} \to \text{RF Write Flop Setup}$$
   *Design Fix:* Eliminated all intermediate arithmetic logic; `w2_d` is a pure multiplexer tree.
2. **Sequencer Start Launch Fanout ($11.8\text{ ns}$):**
   `seq_start` from `u_tc` fans out to the sequencer start logic and simultaneously drives write-enables, address muxes, and data muxes for RF Ports 0, 1, and 2. High-drive buffer cells (`CLKBUFX8`) were manually inserted on `seq_start` to preserve clock transition times ($t_{\text{rise}} < 0.35\text{ ns}$).

---

## 7. Verification Anomalies, Integration Defects, and Engineering Fixes

### 7.1 Defect ENG-01: RF Port 2 Write Collisions During Replay Initialization
- **Symptom:** During formal verification of OOSM replay recovery, covariance diagonal $P_{00}$ was corrupted on the exact cycle the sequencer executed its first microinstruction.
- **Root Cause:** A race condition occurred between `tide_temporal` seeding measurement parameters and the sequencer writeback scheduler. As the sequencer initiated execution, `seq_start` asserted while an LSU writeback was finishing, causing simultaneous writes to `rf[120]` and `rf[l_waddr]`.
- **Engineering Fix:** Hardened the Port 2 write datapath with explicit priority gating:
  ```systemverilog
  wire [6:0] w2_a = seq_start ? 7'd122 : p2_tc ? tc_rf_waddr : p2_seq ? sa2 : l_waddr;
  ```
  Additionally synthesized the sticky assertion tracker `rf_conflict_o` to detect concurrent write assertions and report them to system diagnostics.

### 7.2 Defect ENG-02: Store Data Corruption via ADD Lane Read Conflict
- **Symptom:** In flight trajectory simulation, saving state vector element $x[2]$ ($p_d$, down position) to TIDE-RAM resulted in writing garbage data whenever the ADD slot simultaneously executed an `FSUB` instruction.
- **Root Cause:** The LSU store data bus was originally wired statically to `rf_rdata2_i`. When the sequencer executed an instruction using two ADD operands ($r_a, r_b$), Port 2 was driven by $r_a$, and Port 3 was driven by $r_b$. The store instruction required $r_{\text{src}}$, which the sequencer had mapped to Port 3. Consequently, the LSU sampled the wrong register.
- **Engineering Fix:** Implemented integration fix I2:
  ```systemverilog
  .rf_rdata_i(engine_owns ? seq_lsu_rdata : rd2)
  ```
  The sequencer now performs internal multiplexing between Port 2 and Port 3 (`seq_lsu_rdata`), routing the resolved store operand out to the LSU.

### 7.3 Defect ENG-03: Sequencer Microcode Fetch Corruption via CPU Mirror Contention
- **Symptom:** Microcode execution occasionally branched to illegal addresses when telemetry monitoring software read out microcode contents over AHB.
- **Root Cause:** The microcode RAM read address port was shared without engine-state qualification. Host CPU readback addresses overrode the program counter.
- **Engineering Fix:** Tied the microcode address multiplexer directly to `enable_i`:
  ```systemverilog
  .seq_addr_i(enable_i ? ucode_addr : uc_rd_addr_i)
  ```
  Whenever the engine is active (`enable_i = 1`), the sequencer retains uncompromised ownership of the microprogram memory bus.
