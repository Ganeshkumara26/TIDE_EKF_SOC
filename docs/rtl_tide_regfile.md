# RTL Development & Architectural Implementation Document: `tide_regfile.sv`
**Module Name:** `tide_regfile`  
**Design Reference:** `06_tide_regfile.md`, `18_storage_map.md`, `15_timing_contracts.md`, `STEP_0_DECISIONS.md`  
**Target Architectures:** OS0005 Avionics SoC (SCL 180nm CP8D), OS0035 Micro-Navigation Processor (ASAP7 FinFET)  
**Verification Coverage:** Formal assertions (`tide_regfile_sva.sv`), Directed unit testbench (`tb_tide_regfile.sv`), Gate-level post-P&R timing checks  

---

## 1. Architectural Role & Context Storage Requirements

The `tide_regfile` module serves as the primary high-throughput register file for the TIDE Extended Kalman Filter (EKF) coprocessor. In real-time aerospace state estimation architectures such as the OS0005 Avionics SoC, the EKF algorithm demands continuous matrix-vector arithmetic across a tightly bounded state space:
- 6-element kinematics state vector $\mathbf{x} = [p_n, p_e, p_d, v_n, v_e, v_d]^T$
- 21-element packed lower-triangular error covariance matrix $\mathbf{P} \in \mathbb{R}^{6 \times 6}$
- Intermediate coordinate transform tensors (direction cosine matrices, line-of-sight range and elevation vectors)
- Innovation vectors $\mathbf{y}$, Kalman gain columns $\mathbf{K}_i$, and temporary accumulators

Unlike general-purpose processor register files that provide 16 or 32 entries, the TIDE engine provisions **128 registers of 32-bit width** ($128 \times 32$ bits = 4096 storage elements). The register file stores the *active computational context* only. Historical snapshots, out-of-sequence measurement (OOSM) checkpoints, and sensor journals are held in dense scratchpad SRAM (`tide_ram`).

To support full-rate parallel execution across independent floating-point execution lanes without pipeline stalls, the register file provides **four combinational read ports** and **three synchronous write ports** (4R3W).

```
                      ┌───────────────────────────────────────┐
                      │             tide_regfile              │
                      │               128 x 32                │
                      │          (Flip-Flop Array)            │
                      └──────────────────┬────────────────────┘
                          ▲   ▲   ▲      │    ▲    ▲    ▲
   Write Port 0 (MUL) ────┘   │   │      │    │    │    │
   Write Port 1 (ADD/DS) ─────┘   │      │    │    │    │
   Write Port 2 (LSU/TC/ALU) ─────┘      │    │    │    │
                                         ▼    ▼    ▼    ▼
                                       Read Read Read Read
                                       Port Port Port Port
                                        0    1    2    3
                                       (MUL (MUL (ADD (ADD
                                        OpA) OpB) OpA) OpB/LSU)
```

---

## 2. Storage Cell Topology: Flip-Flop Array vs. SRAM Hard Macro Trade-Offs

A fundamental architectural decision during front-end design was the rejection of multi-ported SRAM macros in favor of a synthesized flip-flop array.

### 2.1 Failure of Standard SRAM Macros in 4R3W Topologies
Foundry memory compilers for space-grade nodes such as SCL 180nm CP8D or commercial FinFET nodes (ASAP7) do not provide native 7-port (4R3W) SRAM bitcells. Conventional standard-cell memory offerings are restricted to:
1. **Single-Port (1RW) Macros:** Highly dense, but allow only one access per clock cycle. Achieving 4R3W via time-division multiplexing (multi-pumping) would require an internal memory clock running at $7 \times f_{\text{sys}} = 350\text{ MHz}$ at 50 MHz core frequency. In 180nm bulk CMOS, generating and distributing a jitter-free 350 MHz clock across memory periphery logic introduces severe dynamic power penalties and clock-tree skew risks.
2. **True Dual-Port (2RW) Macros:** Multiplexing two read/write ports still requires multi-pumping by $3.5\times$ or multi-bank replication.
3. **Multi-Bank Replicated SRAM Arrays:** Replicating a 1W/4R memory across 3 write banks with coherency crossbars creates complex conflict arbitration and write-broadcast overhead that exceeds the area of a flip-flop array for depths below 256 words.

### 2.2 Synthesized Flip-Flop Storage Array Mechanics
For a 128-word depth, standard-cell D-flip-flops deliver optimal routability and deterministic single-cycle combinational read latency:
- **Bitcell Count:** $128 \text{ words} \times 32 \text{ bits} = 4,096 \text{ storage bits}$.
- **Flop Area:** Utilizing unreset standard-cell flip-flops (`DFCL11` in SCL 180nm, $\sim 8\text{ GE}$ per bit), the raw storage consumes:
  $$\text{Storage Area} = 4096 \times 8\text{ GE} \approx 32,768\text{ GE}$$
- **Read Multiplexer Overhead:** Four independent 128-to-1 32-bit combinational multiplexer trees. Implemented as 4-level balanced mux trees using inverting AOI/OAI standard cells (`MUX4` / `AOI22`), each 128:1 32-bit mux consumes $\sim 4,000\text{ GE}$:
  $$\text{Mux Tree Area} = 4 \times 4000\text{ GE} \approx 16,000\text{ GE}$$
- **Total Silicon Footprint:** $\sim 48,768\text{ GE}$ (approximately $0.39\text{ mm}^2$ in SCL 180nm CP8D, or $<0.003\text{ mm}^2$ in ASAP7 FinFET), which fits squarely within the sub-0.5 mm² allocation for active context storage.

---

## 3. Register Addressing & Architectural Conventions

The microcode assembler and hardware coprocessor partition the 128 registers according to the following strict allocation:

| Address (`raddr`) | Identifier | Primary Purpose | Pipeline / Allocation Rules |
|:---|:---|:---|:---|
| `7'd0` – `7'd5` | `x[0..5]` | EKF State Vector $[p_n, p_e, p_d, v_n, v_e, v_d]^T$ | Burst context transfer window (LSU) |
| `7'd6` – `7'd26` | `P[0..20]` | Covariance Lower Triangle (row-major) | Burst context transfer window (LSU) |
| `7'd27` | `t_now` | Estimator epoch timestamp (microseconds) | Updated by temporal coordinator |
| `7'd28` – `7'd31` | `z[0..3]` | Measurement vector (Range, Azimuth, Elev, Doppler) | Populated by temporal coordinator |
| `7'd32` – `7'd35` | `R[0..3]` | Measurement noise diagonal matrix | Loaded from CSR parameter shadow |
| `7'd36` – `7'd39` | `y[0..3]` | Innovation residuals $\mathbf{y} = \mathbf{z} - h(\mathbf{x})$ | Temporary scratchpad |
| `7'd40` – `7'd63` | `TMP[0..23]` | Observation Jacobian $\mathbf{H}$ rows, Kalman gains | Intermediate floating-point scratch |
| `7'd64` – `7'd69` | `x_prior[0..5]` | Saved prior state for Joseph-form update | Preserved across correction steps |
| `7'd70` – `7'd79` | `GEOM[0..9]` | Slant range $r$, ground range $r_g$, $1/r$, angles | Coordinate transform pipeline |
| `7'd80` – `7'd119` | `SCRATCH` | Matrix inversion and covariance update buffers | Microcode general purpose |
| `7'd120` – `7'd122` | `ABI_PAR` | Microcode ABI parameters: `QP`, `QV`, `DT0INV` | Seeded on `seq_start` pulse |
| `7'd123` – `7'd124` | `RESERVED` | Internal sequencer temporary slots | General purpose |
| `7'd125` | `idx1` | Index Register 1 (Indirection pointer) | Intercepted by sequencer (ASSUMPTION A1) |
| `7'd126` | `idx0` | Index Register 0 (Indirection pointer) | Intercepted by sequencer (ASSUMPTION A1) |
| `7'd127` | `ZERO` (`r127`)| Hardwired Constant Floating-Point Zero | Reads return `32'h00000000`; writes discarded |

### 3.1 Hardwired Zero Register (`r127`)
Register 127 (`7'h7F`) is physically decoupled from storage updates and hardwired to zero:
```systemverilog
assign rdata0_o = (raddr0_i == 7'd127) ? 32'h0 : rf[raddr0_i];
assign rdata1_o = (raddr1_i == 7'd127) ? 32'h0 : rf[raddr1_i];
assign rdata2_o = (raddr2_i == 7'd127) ? 32'h0 : rf[raddr2_i];
assign rdata3_o = (raddr3_i == 7'd127) ? 32'h0 : rf[raddr3_i];
```
In floating-point algorithms, zero serves multiple essential roles:
1. **IEEE-754 Additive Identity:** $x + 0.0 = x$, enabling operand passthrough across the adder pipeline (`FMOV` operations synthesized as `FADD r_dst, r_src, r127`).
2. **Harmless Result Sink:** When arithmetic pipelines flush or when instructions execute for condition code side-effects (e.g. `FCMP`), `r127` acts as a destination bit-bucket without requiring custom control-flow cancellation logic.
3. **Pipeline Bubble Neutralization:** When inserting startup bubbles (`SAFE_BUBBLE_WORD`), the ADD destination is explicitly aimed at `r127` to avoid corrupting active EKF states.

Writes targeting `r127` are suppressed at the write-port multiplexing stage:
```systemverilog
if (wen0_i && waddr0_i != 7'd127) rf[waddr0_i] <= wdata0_i;
if (wen1_i && waddr1_i != 7'd127) rf[waddr1_i] <= wdata1_i;
if (wen2_i && waddr2_i != 7'd127) rf[waddr2_i] <= wdata2_i;
```

### 3.2 Dynamic Index Registers (`idx0`, `idx1`) Isolation
Under ISA revision ISA-05, indirect addressing uses hardware loop registers `idx0` (`r126`) and `idx1` (`r125`). Because the physical interface `tide_regfile_if` was frozen without dedicated index register ports, the sequencer intercepts addresses `7'h7E` and `7'h7D` in its fetch/decode logic. Values for `idx0`/`idx1` are substituted before reaching the register file read ports, and writes targeting them update loop counter flops internal to `tide_sequencer`. The physical entries `rf[125]` and `rf[126]` in the storage array remain unaddressed during execution.

---

## 4. Port Mechanics and Timing Semantics

### 4.1 Combinational Read Architecture (Read-Old / Read-Before-Write)
The register file read ports are purely combinational:
- An address asserted on `raddrN_i` during clock cycle $C$ propagates through the multiplexer tree to `rdataN_o` within the same clock cycle ($C$).
- **Timing Path:** The combinational read delay is bounded by the propagation time of the 128:1 mux tree ($t_{\text{mux}} \approx 1.8\text{ ns}$ in SCL 180nm at worst-case slow-slow corner, $1.62\text{ V}, 125^\circ\text{C}$). This enables the execution lanes to sample operands on the very cycle an instruction enters the Decode/Execute stage.
- **Read-Write Race Resolution:** If a read and a write target the same register address within clock cycle $C$, the read returns the **old value** (read-before-write semantics). The storage flip-flops sample new data on the rising clock edge marking the end of cycle $C$; therefore, written data is only visible to read ports starting in cycle $C+1$.

```
Clock         ____/‾‾‾‾\____/‾‾‾‾\____/‾‾‾‾\____/‾‾‾‾\____/‾‾‾‾\____
Cycle                Cycle C                  Cycle C+1
raddr0_i      XXXXXX[      Addr 0x10      ]XXXXXXXXXXXXXXXXXXXXXXXX
waddr0_i      XXXXXX[      Addr 0x10      ]XXXXXXXXXXXXXXXXXXXXXXXX
wen0_i        ______/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\__________________________
wdata0_i      XXXXXX[      New_Data       ]XXXXXXXXXXXXXXXXXXXXXXXX
rf[0x10]      [      Old_Data             ][      New_Data         ]
rdata0_o      XXXXXX[      Old_Data       ][      New_Data         ]
                    (Read-Old sampled)       (Updated value)
```

### 4.2 Write-Port Conflict Resolution: Deterministic WAW Priority
The sequencer instruction set forbids concurrent writes to the same physical register from multiple functional units (Write-After-Write hazard). However, to prevent indeterminate silicon states or latch synthesis in the event of microcode assertion violations, the hardware implements a deterministic priority hierarchy:
$$\text{Port 2 (LSU / Context Restore)} > \text{Port 1 (ADD / DIVSQRT)} > \text{Port 0 (MUL)}$$

This priority is synthesized cleanly in SystemVerilog using non-blocking procedural assignments within a single clocked process:
```systemverilog
always_ff @(posedge clk_i) begin
    if (wen0_i && waddr0_i != 7'd127)
        rf[waddr0_i] <= wdata0_i;
    if (wen1_i && waddr1_i != 7'd127)
        rf[waddr1_i] <= wdata1_i;  // Overwrites Port 0 if waddr1_i == waddr0_i
    if (wen2_i && waddr2_i != 7'd127)
        rf[waddr2_i] <= wdata2_i;  // Overwrites Port 1 and Port 0 on collision
end
```
Because non-blocking assignments (`<=`) within an `always_ff` evaluate in procedural sequence, a later assignment to the same array index overrides prior assignments occurring in the same clock evaluation step. Synthesis compilers map this directly to prioritized clock-enable gating logic on each register bitcell.

---

## 5. Physical Synthesis & Standard Cell Library Constraints

### 5.1 Standard Cell Selection: `DFCL11` vs. `DFFL11` and Reset Stripping
A critical physical design challenge in space-grade ASIC synthesis (SCL 180nm CP8D) is routing congestion and clock-tree network capacitance across dense register arrays.
- Initial RTL drafts applied an asynchronous active-low reset (`rst_ni`) across the entire $128 \times 32$ flip-flop array.
- In standard-cell implementations, an asynchronous reset flop (`DFRCL11`) carries a silicon area penalty of $\approx 2.5\text{ GE}$ compared to a non-reset flop (`DFCL11`), totaling an extra $\sim 10,240\text{ GE}$.
- More critically, distributing `rst_ni` across 4,096 bitcells created severe routing track blockages in Metal 1 (M1) and Metal 2 (M2), leading to design rule check (DRC) violations near the cell pin connections.
- **Architectural Solution (ASSUMPTION A2):** The register array storage flops were explicitly stripped of hardware reset. The reset pin `rst_ni` is retained on the module interface to satisfy contract conformance (`tide_regfile_if`), but drives zero internal loads within this block. The TIDE coprocessor guarantees correctness by requiring the temporal controller to execute a context restore or zero-initialization loop before reading state registers.

### 5.2 Read Mux Routing Congestion & Cell Placement
In 180nm planar CMOS with a 5-metal layer stack:
- M1 is predominantly consumed by standard-cell internal routing and power rail connections (VDD/VSS).
- M2 and M3 are used for intra-cell routing and horizontal operand busing.
- Each of the 4,096 storage bits must fan out to four separate 128:1 multiplexer inputs. A naive flat multiplexer structure induces extreme wire congestion on M3/M4.
- Synthesis scripts must enforce a tree-decomposition strategy (`set_flatten false -design tide_regfile`), forcing the synthesis engine to construct a symmetrical 4-stage hierarchy of 4:1 multiplexers:
  $$128:1 \implies 32 \times (4:1) \to 8 \times (4:1) \to 2 \times (4:1) \to 1 \times (2:1)$$
  This balances capacitive loading and ensures timing closure at 50 MHz across commercial temperature extremes ($-40^\circ\text{C}$ to $+125^\circ\text{C}$).

---

## 6. Verification Anomalies, Silicon Defects, and Engineering Fixes

### 6.1 Defect RF-01: Multi-Driver Contention in Primitive Verilog Emulation
- **Symptom:** During early system-level co-simulation with third-party Verilog-AMS models, multi-driven net warnings and indeterminate `X` values appeared on register read buses during simultaneous multi-port write cycles.
- **Root Cause:** A behavioral model attempted to describe each write port in an independent `always_ff` process:
  ```systemverilog
  // INCORRECT: Separate processes creating multiple driver conflicts
  always_ff @(posedge clk_i) if (wen0_i) rf[waddr0_i] <= wdata0_i;
  always_ff @(posedge clk_i) if (wen1_i) rf[waddr1_i] <= wdata1_i;
  ```
  IEEE 1800-2017 prohibits multiple concurrent procedural drivers on the same variable memory element without resolution functions.
- **Engineering Fix:** Consolidated all write ports into a single `always_ff @(posedge clk_i)` procedural block with sequential `if` precedence, establishing deterministic priority synthesis without multi-driver artifacts.

### 6.2 Defect RF-02: Zero-Register Leakage via Unchecked Write Ports
- **Symptom:** Microcode execution of `FCMP` instructions intended to discard results into `r127` resulted in nonzero values appearing on subsequent reads from register 127.
- **Root Cause:** The combinational read muxes correctly forced `r127` reads to zero (`raddr == 7'd127 ? 32'h0 : rf[...]`), but the write path omitted address qualification for Port 0 and Port 1. Data was successfully written into `rf[127]`. When index registers subsequently wrapped around or accessed invalid pointer offsets, unmasked internal reads leaked non-zero values.
- **Engineering Fix:** Inserted mandatory `waddrX_i != 7'd127` write-suppression guards across all three write paths in RTL, permanently isolating `rf[127]` from state retention.

---

## 7. Formal Verification & Assertion Matrix

Module correctness is formally proven via `tide_regfile_sva.sv`:
1. **Zero Integrity:** `assert property (@(posedge clk_i) (raddr0_i == 7'd127) |-> (rdata0_o == 32'h0));`
2. **Read-Before-Write Isolation:**
   ```systemverilog
   property p_rf_read_old(rport, wport, addr);
       logic [31:0] cur_val;
       @(posedge clk_i)
       (wenX_i && (waddrX_i == addr) && (raddrY_i == addr) && (addr != 7'd127), cur_val = rf[addr])
       |-> (rdataY_o == cur_val);
   endproperty
   ```
3. **Write Priority Determinism:**
   ```systemverilog
   property p_rf_waw_priority;
       @(posedge clk_i)
       (wen0_i && wen2_i && (waddr0_i == waddr2_i) && (waddr2_i != 7'd127))
       |=> (rf[$past(waddr2_i)] == $past(wdata2_i));
   endproperty
   ```
This verification structure guarantees mathematical integrity across all EKF state updates.
