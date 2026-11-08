# RTL Development & Architectural Implementation Document: `tide_ram.sv`
**Module Name:** `tide_ram` (encompassing `tide_sram_sp_32x1024`)  
**Design Reference:** `32_memory_macro_wrapper.md`, `18_storage_map.md`, `15_timing_contracts.md`, `STEP_0_DECISIONS.md`  
**Target Architectures:** OS0005 Avionics SoC (SCL 180nm CP8D), OS0035 Micro-Navigation Processor (ASAP7 FinFET)  
**Verification Coverage:** Formal assertions (`tide_ram_sva.sv`), Directed unit testbench (`tb_tide_ram.sv`), Memory BIST and scan insertion  

---

## 1. Architectural Role & Storage Organization

The `tide_ram` module is the primary 4 KB (1024 words × 32 bits) data scratchpad for the TIDE Extended Kalman Filter (EKF) coprocessor. In flight-critical avionics such as the OS0005 SoC, the estimator must maintain state continuity across intermittent sensor dropouts and manage out-of-sequence measurements (OOSM) without CPU intervention.

While the 128-entry register file (`tide_regfile`) holds only the active computational variables, `tide_ram` acts as the persistent on-chip repository for:
- Historical state and covariance checkpoints
- Measurement innovation journals
- Committed and shadow navigation contexts
- Dynamic scratch space for intermediate matrix calculations

```
┌────────────────────────────────────────────────────────────────────────┐
│                      TIDE-RAM Word Address Map                         │
├───────────────┬─────────────┬──────────────┬───────────────────────────┤
│ Word Range    │ Byte Range  │ Region Name  │ Capacity & Allocation     │
├───────────────┼─────────────┼──────────────┼───────────────────────────┤
│ `0x000–0x0FF` │ `0x000–0x3FF│ Checkpoint   │ 256 words: 8 slots × 32   │
│               │             │ Ring Buffer  │ committed context frames  │
├───────────────┼─────────────┼──────────────┼───────────────────────────┤
│ `0x100–0x23F` │ `0x400–0x8FF│ Measurement  │ 320 words: 64 entries × 5 │
│               │             │ Journal Ring │ words (t_meas + 4-z vars) │
├───────────────┼─────────────┼──────────────┼───────────────────────────┤
│ `0x240–0x25F` │ `0x900–0x97F│ Committed    │ 32 words: Current         │
│               │             │ Context      │ authoritative EKF state   │
├───────────────┼─────────────┼──────────────┼───────────────────────────┤
│ `0x260–0x27F` │ `0x980–0x9FF│ Shadow       │ 32 words: Working context │
│               │             │ Context      │ during OOSM replay loops  │
├───────────────┼─────────────┼──────────────┼───────────────────────────┤
│ `0x280–0x3FF` │ `0xA00–0xFFF│ Scratchpad   │ 384 words: General matrix │
│               │             │ & Reserved   │ scratch / expansion       │
└───────────────┴─────────────┴──────────────┴───────────────────────────┘
```

Each context frame is packed into 32 words (128-byte aligned) to optimize burst transfers:
- Word 0: `t_last` (microsecond timestamp of last incorporated measurement)
- Words 1–6: Kinematics state vector $\mathbf{x} = [p_n, p_e, p_d, v_n, v_e, v_d]^T$
- Words 7–27: Packed lower-triangular error covariance matrix $\mathbf{P} \in \mathbb{R}^{6 \times 6}$
- Word 28: `journal_ptr` (tail pointer of the measurement journal at snapshot time)
- Words 29–31: Reserved alignment padding

---

## 2. Multi-Requester Priority Arbitration & Virtual Multi-Porting

### 2.1 The Die Area Dilemma: True Dual-Port vs. Single-Port Macro
In rad-tolerant fabrication processes such as SCL 180nm CP8D, a true dual-port (2RW) SRAM macro incurs a massive area and routing penalty:
- A 2RW bitcell requires 8 transistors (8T) with dual wordlines and bitline pairs.
- In 180nm CMOS, a $1024 \times 32$ 2RW macro occupies $\sim 0.45\text{ mm}^2$, compared to $\sim 0.25\text{ mm}^2$ for an equivalent single-port (1RW) 6T macro—an area inflation of $1.8\times$.
- Furthermore, dual-port macros impose double the routing track blockages in Metal 1 through Metal 3.

To maximize silicon efficiency, the TIDE architecture utilizes a **single-port SRAM macro (`tide_sram_sp_32x1024`) wrapped by a 4-requester combinational priority arbiter**. Virtual multi-porting is achieved because the operational duty cycles of the four masters are largely non-overlapping:
1. **CPU Write Port (Port 3):** Used strictly at boot time or initialization to seed initial covariance matrices; permanently locked out during active coprocessor execution.
2. **Temporal Controller (Port 1):** Accesses RAM during state snapshotting and journal scanning between microcode execution blocks.
3. **Load/Store Unit (Port 2):** Accesses RAM during microcode single-word operations or context burst restorations.
4. **CPU Debug Read Port (Port 4):** Reads RAM opportunistically for telemetry monitoring.

### 2.2 Strict Fixed-Priority Arbitration Hierarchy
The arbiter resolves concurrent requests combinationally within the clock cycle:
$$\text{TC Direct (Port 1)} > \text{LSU Burst/Single (Port 2)} > \text{CPU Write (Port 3)} > \text{CPU Debug Read (Port 4)}$$

```systemverilog
always_comb begin
    g_tc  = 1'b0; g_lsu = 1'b0; g_cpu = 1'b0; g_dbg = 1'b0;
    m_ce  = 1'b0; m_we  = 1'b0; m_addr = 12'd0; m_wdata = 32'd0;
    
    if (tc_ce_i) begin
        g_tc    = 1'b1; m_ce = 1'b1; m_we = tc_we_i;
        m_addr  = tc_addr_i;  m_wdata = tc_wdata_i;
    end else if (lsu_ce_i) begin
        g_lsu   = 1'b1; m_ce = 1'b1; m_we = lsu_we_i;
        m_addr  = lsu_addr_i; m_wdata = lsu_wdata_i;
    end else if (cpu_req) begin
        g_cpu   = 1'b1; m_ce = 1'b1; m_we = 1'b1;
        m_addr  = cpu_wr_addr_i; m_wdata = cpu_wr_data_i;
    end else if (dbg_ce_i) begin
        g_dbg   = 1'b1; m_ce = 1'b1; m_we = 1'b0;
        m_addr  = dbg_addr_i;
    end
end
```

### 2.3 Contention and Conflict Reporting
Under normal firmware operation, software and microcode scheduling ensure that TC and LSU do not issue simultaneous accesses. However, to guarantee fail-safe flight telemetry, `tide_ram` detects and latches any access collisions:
```systemverilog
if ((tc_ce_i & lsu_ce_i) | (tc_ce_i & cpu_req) | (lsu_ce_i & cpu_req))
    conflict_o <= 1'b1;
```
The `conflict_o` flag is sticky (cleared only via global system reset), alerting the host processor that a non-deterministic timing collision occurred.

---

## 3. Address Boundary Protection & Read-Data Steering

### 3.1 12-Bit ISA Addressing and Out-of-Range Trapping
Per ISA decision ISA-10, the TIDE engine utilizes 12-bit word addresses (`addr[11:0]`), spanning a logical 4096-word space. However, the physical memory array contains only 1024 words (`0x000` to `0x3FF`).

To prevent address aliasing and corruption of checkpoint structures:
1. The arbiter continuously validates that the upper address bits are zero:
   ```systemverilog
   wire in_range = (m_addr[11:10] == 2'b00);
   wire mac_ce   = m_ce & in_range;
   ```
2. If an access targets an address $\ge 1024$ (`m_addr[11:10] != 2'b00`):
   - SRAM chip-enable (`mac_ce`) is forced low, suppressing writes and preventing spurious read cycles.
   - The sticky error flag `oor_o` (out-of-range) is latched high.
   - Out-of-range read responses return clean zero data (`32'd0`).

### 3.2 Registered Grant Tracking and Read Steering
Because the SRAM macro exhibits a 1-cycle synchronous read latency (data valid the cycle *after* CE assertion), read-data steering requires registered grant qualification:

```systemverilog
logic rd_dbg_q, rd_oor_q;
always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
        rd_dbg_q <= 1'b0;
        rd_oor_q <= 1'b0;
    end else begin
        rd_dbg_q <= g_dbg;
        rd_oor_q <= m_ce & ~in_range;
    end
end

assign tc_rdata_o  = rd_oor_q ? 32'd0 : mac_rdata;
assign lsu_rdata_o = rd_oor_q ? 32'd0 : mac_rdata;
assign dbg_rdata_o = (rd_dbg_q & ~rd_oor_q) ? mac_rdata : 32'd0;
assign dbg_valid_o = rd_dbg_q;
```

This prevents debug reads from sampling stale data when preempted by an LSU burst, while guaranteeing that ungranted debug accesses return zero rather than snooping internal coprocessor states.

---

## 4. Physical Memory Macro Wrapper (`tide_sram_sp_32x1024`)

The physical SRAM interface is cleanly separated into the submodule `tide_sram_sp_32x1024`, supporting seamless retargeting between simulation, commercial FinFET prototyping, and foundry tapeout.

### 4.1 Behavioral Simulation Model & Read-During-Write Poisoning
Standard foundry SRAM macros do not support simultaneous read and write to the same address. If `ce_i` and `we_i` are asserted together, the data present on the read data bus on the following clock edge is physically **undefined**.

To ensure that firmware and microcode never inadvertently rely on simulator-specific write-through behaviors, the behavioral model deliberately poisons the read output on write cycles:
```systemverilog
always_ff @(posedge clk_i) begin
    if (ce_i) begin
        if (we_i) begin
            mem[addr_i] <= wdata_i;
`ifndef SYNTHESIS
            rdata_o     <= 32'hDEAD_BEEF;   // Intentional poison per doc 32 §2.2.3
`endif
        end else begin
            rdata_o <= mem[addr_i];
        end
    end
end
```
Any downstream logic that attempts to consume read data during a write cycle immediately latches `32'hDEAD_BEEF`, causing IEEE-754 floating-point calculations to generate NaNs and alerting verification engineers to illegal dependencies.

### 4.2 ASAP7 FinFET Macro Implementation (`ifdef TIDE_MACROS`)
When synthesized for the OS0035 7nm processor, the 1024-word memory is constructed from four 256×32 `fakeram7_256x32` hard macros banked across address bits `[9:8]`:
```systemverilog
`ifdef TIDE_MACROS
    logic [1:0]  bank_q;
    logic [31:0] rd [0:3];
    always_ff @(posedge clk_i) if (ce_i) bank_q <= addr_i[9:8];
    for (genvar b = 0; b < 4; b++) begin : g_bank
        fakeram7_256x32 u_m (
            .clk(clk_i),
            .ce_in(ce_i & (addr_i[9:8] == b)),
            .we_in(we_i),
            .addr_in(addr_i[7:0]),
            .wd_in(wdata_i),
            .rd_out(rd[b])
        );
    end
    assign rdata_o = rd[bank_q];
`endif
```
Banking reduces dynamic power by activating only one 256-word SRAM slice per access, lowering switching capacitance by $\sim 70\%$.

### 4.3 SCL 180nm Physical Integration Constraints
For the OS0005 Avionics SoC tapeout at SCL:
1. **Absence of Hardware Reset:** SCL hard macros feature no reset pin. The internal bitcells power up in arbitrary states. System reset (`rst_ni`) connects to the wrapper to clear arbiter state flops (`conflict_o`, `oor_o`, `rd_dbg_q`), but the memory array is unreset.
2. **Periphery Placement & Metal Blockages:** The $1024 \times 32$ macro must be placed along the boundary of the core tile. Metals M1 through M4 are completely blocked inside the macro footprint. All over-the-macro signal routing is prohibited, requiring synthesis scripts to reserve Metal 5 for cross-core power straps.

---

## 5. Verification Anomalies, Silicon Defects, and Engineering Fixes

### 5.1 Defect RAM-01: Silent Address Aliasing into Checkpoint Buffers
- **Symptom:** During extended stress testing of the temporal journal ring, historical checkpoint slot 0 was intermittently corrupted with spurious measurement data.
- **Root Cause:** A microcode bug in the journal write pointer generated out-of-range addresses above `0x3FF` (specifically `0x400`). Because the underlying macro decoded only 10 address bits (`addr_i[9:0]`), address `0x400` wrapped around to `0x000`, overwriting the first word of the checkpoint ring.
- **Engineering Fix:** Added the hardware boundary check `wire in_range = (m_addr[11:10] == 2'b00);`. If `in_range` is low, `mac_ce` is clamped to zero, permanently isolating the memory array from out-of-bounds corruption, and setting `oor_o`.

### 5.2 Defect RAM-02: Private State Leakage onto Host Debug Bus
- **Symptom:** The host processor, polling the debug read interface while the TIDE engine executed a Joseph-form covariance update, observed valid internal EKF covariance data even when the debug access was rejected due to LSU contention.
- **Root Cause:** `dbg_rdata_o` was wired directly to `mac_rdata`. When an LSU read completed on the same cycle that a debug read request was denied, the LSU's data word appeared on the host debug bus without qualification.
- **Engineering Fix:** Introduced the registered grant tracker `rd_dbg_q` and gated the output bus:
  ```systemverilog
  assign dbg_rdata_o = (rd_dbg_q & ~rd_oor_q) ? mac_rdata : 32'd0;
  assign dbg_valid_o = rd_dbg_q;
  ```
  This guarantees that the debug port returns zero whenever arbitration is not granted.

### 5.3 Defect RAM-03: CPU Write Injection Hazard During Active Navigation
- **Symptom:** Asynchronous diagnostic software executing on the host CPU attempted to recalibrate noise matrices while the coprocessor was mid-propagation, causing numerical divergence in the EKF state.
- **Root Cause:** The arbiter evaluated `cpu_wr_ce_i` directly, allowing CPU writes to preempt the LSU whenever the Temporal Controller was momentarily idle.
- **Engineering Fix:** Hardened the CPU write path by introducing the top-level qualification `cpu_req = cpu_wr_en_i & cpu_wr_ce_i`, where `cpu_wr_en_i` is driven strictly by `~busy_o`. CPU writes are locked out whenever the TIDE engine is active.
