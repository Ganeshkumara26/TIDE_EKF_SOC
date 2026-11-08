# RTL Development & Architectural Implementation Document: `tide_ucode_ram.sv`
**Module Name:** `tide_ucode_ram`  
**Design Reference:** `07_tide_ucode_ram.md`, `18_storage_map.md`, `15_timing_contracts.md`, `STEP_0_DECISIONS.md`  
**Target Architectures:** OS0005 Avionics SoC (SCL 180nm CP8D), OS0035 Micro-Navigation Processor (ASAP7 FinFET)  
**Verification Coverage:** Formal assertions (`tide_ucode_ram_sva.sv`), Directed unit testbench (`tb_tide_ucode_ram.sv`), Memory BIST and scan insertion  

---

## 1. Architectural Purpose & Microcode Execution Paradigm

The `tide_ucode_ram` module is the high-bandwidth microprogram store for the TIDE Extended Kalman Filter (EKF) coprocessor. In autonomous aerospace architectures such as the OS0005 Avionics SoC, flight state estimation algorithms require complex, multi-stage matrix operations:
- Kinematic inertial propagation
- Innovation residual formation ($\mathbf{y} = \mathbf{z} - h(\mathbf{x})$)
- Observation Jacobian calculation ($\mathbf{H}$)
- Kalman gain computation ($\mathbf{K} = \mathbf{P}\mathbf{H}^T(\mathbf{H}\mathbf{P}\mathbf{H}^T + \mathbf{R})^{-1}$)
- Stabilized Joseph-form covariance updates ($\mathbf{P} \leftarrow (\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I}-\mathbf{K}\mathbf{H})^T + \mathbf{K}\mathbf{R}\mathbf{K}^T$)

Rather than implementing these intricate mathematical steps as rigid, hardwired finite state machines, the TIDE engine utilizes a **software-defined micro-architecture**. The coprocessor executes a statically scheduled, 96-bit 5-slot Very Long Instruction Word (VLIW) instruction stream fetched from `tide_ucode_ram`.

Provisioned at **512 words × 96 bits** ($\approx 6\text{ KB}$), this memory allows:
1. Complete storage of the nominal 6-state EKF filter pipeline ($\sim 420\text{ microinstructions}$).
2. Storage of specialized out-of-sequence measurement (OOSM) replay kernels and fast-forward state propagation routines.
3. Post-silicon algorithm updates, tuning of covariance floor thresholds, and in-flight mission reconfiguration without silicon respins.

```
                      ┌────────────────────────────────────────┐
                      │             tide_ucode_ram             │
                      │           512 x 96-bit Memory          │
                      └──────────────────┬─────────────────────┘
                          ▲              │
       CPU Write Port     │              │  Sequencer Read Port
     (3-Phase MMIO Commit)│              │  (1-Cycle Latency)
     - cpu_we_i           │              ▼
     - cpu_addr_i [8:0]   │         - seq_addr_i [8:0]
     - cpu_word_i [1:0]   │         - seq_data_o [95:0]
     - cpu_data_i [31:0]  │                 │
                          │                 ▼
                 [64-Bit Staging Reg]  ┌────────────────────────┐
                 - bits [31:0]         │     tide_sequencer     │
                 - bits [63:32]        │  (5-Slot VLIW Decode)  │
                                       └────────────────────────┘
```

---

## 2. Sequencer Read Interface & Pipeline Mechanics

### 2.1 Synchronous 1-Cycle Read Timing
The microcode read port interfaces directly to the instruction fetch stage of `tide_sequencer`. The port operates synchronously without an external chip-enable; whenever the core clock toggles, an instruction fetch occurs:
- Address `seq_addr_i` presented on clock cycle $N$ yields instruction word `seq_data_o` on clock cycle $N+1$.
- Latency is deterministically **1 clock cycle**.
- The synchronous registered output isolates the internal memory bitcell bitlines from the extensive combinational fanout of the 5-slot instruction decoders.

```systemverilog
always_ff @(posedge clk_i) begin
    seq_data_o <= mem[seq_addr_i];
end
```

### 2.2 Alignment with the Sequencer Two-Stage Pipeline
Because the microprogram memory has a 1-cycle latency, the sequencer instruction pipeline is structured into two stages:
- **Stage 1 (Fetch):** The program counter (`fetch_pc_q`) is driven to `seq_addr_i`.
- **Stage 2 (Decode/Execute):** The 96-bit output `seq_data_o` is captured into the instruction register (`dreg_q`), where it is partitioned into the MUL, ADD, DS, LSU, and CTRL execution slots.

During engine cold start, the sequencer FSM inserts a mandatory 1-cycle warm-up bubble (`warm_q = 2'd1`) to allow the first instruction at `start_addr_i` to propagate out of `tide_ucode_ram` before decoding commences.

---

## 3. Host Upload Port & 3-Phase Atomic Commit Protocol

The host interface is connected to the system bus (PicoRV32 AHB-Lite or AXI4-Lite). Because the system bus datapath is 32 bits wide, writing a 96-bit microinstruction requires an atomic multi-cycle assembly protocol.

### 3.1 Three-Phase Staging Architecture
To avoid exposing partial or corrupted 96-bit instructions to the sequencer, the CPU write interface implements a **3-phase sequential commit protocol** governed by a 64-bit internal staging register (`cpu_staging`):

```systemverilog
always_ff @(posedge clk_i) begin
    if (cpu_we_i) begin
        case (cpu_word_i)
            2'b00:   cpu_staging[31:0]  <= cpu_data_i;
            2'b01:   cpu_staging[63:32] <= cpu_data_i;
            2'b10:   mem[cpu_addr_i]    <= {cpu_data_i, cpu_staging};
            default: ; // 2'b11 is reserved/ignored
        endcase
    end
end
```

1. **Phase 0 (`cpu_word_i = 2'b00`):** The host writes the low 32 bits (`word[31:0]`). The data is latched into `cpu_staging[31:0]`. No memory array write occurs.
2. **Phase 1 (`cpu_word_i = 2'b01`):** The host writes the middle 32 bits (`word[63:32]`). The data is latched into `cpu_staging[63:32]`. The memory array remains unchanged.
3. **Phase 2 (`cpu_word_i = 2'b10`):** The host writes the high 32 bits (`word[95:64]`). On this clock edge, the memory array executes an atomic 96-bit write into `mem[cpu_addr_i]`, concatenating the arriving bus data with the 64 staged bits:
   $$\text{mem}[cpu\_addr\_i] \leftarrow \{cpu\_data\_i,\ cpu\_staging[63:32],\ cpu\_staging[31:0]\}$$

Any access asserting `cpu_word_i = 2'b11` is safely ignored, preventing invalid array writes.

### 3.2 Host MMIO Address Decoding
The microcode RAM is mapped into the CPU memory map as an 8 KB address window (`0x3000_0000` to `0x3000_1FFF`). The bus wrapper decodes 32-bit aligned CPU byte addresses with a 16-byte stride per 96-bit instruction word:

```
MMIO Byte Address = 0x3000_0000 + (Word_Index * 16) + Byte_Offset

  Byte Offset +0x0  (cpu_word_i = 2'b00) ──► Low 32 bits  [31:0]
  Byte Offset +0x4  (cpu_word_i = 2'b01) ──► Mid 32 bits  [63:32]
  Byte Offset +0x8  (cpu_word_i = 2'b10) ──► High 32 bits [95:64] (COMMIT)
  Byte Offset +0xC  (cpu_word_i = 2'b11) ──► Unused (Ignored)
```

The bus slave extracts address fields via simple bit slicing:
- `cpu_addr_i = haddr_i[12:4]` (9 bits, addressing 0 to 511 words)
- `cpu_word_i = haddr_i[3:2]` (2 bits, selecting 32-bit slice)

---

## 4. Memory Protection & Architectural Decoupling

### 4.1 Write Lock Requirement (REQ-I-03)
In mission-critical flight avionics, inadvertently modifying or corrupting the microprogram memory while the Kalman filter is propagating state vectors would result in loss of attitude control. Architectural specification §6 enforces that **all CPU write attempts to UCODE-RAM must be locked out while the coprocessor is running (`ENABLE = 1`)**.

### 4.2 Module Interface Contract Resolution (ASSUMPTION A3)
During architectural freeze, the formal module interface contract `tide_ucode_ram_if` was finalized without an `enable_i` status input port. To uphold strict contract compliance without altering frozen port lists, the write-protection lock is implemented **upstream at the top-level integration boundary (`tide_engine.sv`)**:
```systemverilog
// Upstream gating in tide_engine.sv
tide_ucode_ram u_ucode (
    .clk_i,
    .rst_ni,
    .seq_addr_i (enable_i ? ucode_addr : uc_rd_addr_i),
    .seq_data_o (ucode_data),
    .cpu_we_i   (uc_we_i & ~enable_i),   // Hardware write lock enforced here
    .cpu_addr_i (uc_addr_i),
    .cpu_word_i (uc_word_i),
    .cpu_data_i (uc_data_i)
);
```
- When `enable_i = 1`, `cpu_we_i` is physically clamped to zero before entering `tide_ucode_ram`.
- Concurrently, readback multiplexing directs the sequencer address to the array during execution, while routing CPU readback addresses (`uc_rd_addr_i`) only when the coprocessor is quiescent (`enable_i = 0`).

---

## 5. Physical Synthesis & Standard Cell Library Integration

### 5.1 Storage Cell Selection: Flip-Flop Array vs. Multi-Slice SRAM
At 512 words × 96 bits, the microcode store contains 49,152 storage bits.
- **Synthesized Flip-Flop Array:**
  In SCL 180nm CP8D, implementing 49,152 flip-flops using unreset cells (`DFCL11`, $\approx 8\text{ GE}$) plus 96-bit wide 512:1 read muxes consumes $\sim 393,000\text{ GE}$. This accounts for nearly $3.1\text{ mm}^2$ of silicon, exceeding the entire area budget of the coprocessor.
- **ASIC Hard Macro Strategy:**
  To achieve silicon closure, the microcode store is targeted for embedded single-port SRAM macros:
  - If a native 96-bit wide macro is unavailable from the foundry compiler, the memory is synthesized as **three parallel 512×32-bit single-port SRAM macros** operating in lockstep.
  - The 9-bit address and clock are broadcast to all three macros; each macro handles one 32-bit third of the 96-bit word.

### 5.2 ASAP7 FinFET Macro Implementation (`ifdef TIDE_MACROS`)
In the OS0035 7nm processor prototype, `tide_ucode_ram` maps to six `fakeram7_256x34` SRAM blocks organized as a $2 \times 3$ matrix (2 depth banks of 256 words × 3 parallel 32-bit data slices):

```systemverilog
`ifdef TIDE_MACROS
    // Single-port SRAM macro sharing: CPU write commit steals the port
    wire         wr_commit = cpu_we_i && (cpu_word_i == 2'b10);
    wire [8:0]   m_addr    = wr_commit ? cpu_addr_i : seq_addr_i;
    wire [95:0]  m_wdata   = {cpu_data_i, cpu_staging};
    logic        bank_q;
    
    always_ff @(posedge clk_i) bank_q <= m_addr[8];
    
    logic [33:0] rd [0:1][0:2];
    for (genvar bk = 0; bk < 2; bk++) begin : g_bank
        for (genvar sl = 0; sl < 3; sl++) begin : g_slice
            fakeram7_256x34 u_m (
                .clk    (clk_i),
                .ce_in  (m_addr[8] == bk),
                .we_in  (wr_commit),
                .addr_in(m_addr[7:0]),
                .wd_in  ({2'b00, m_wdata[32*sl +: 32]}),
                .rd_out (rd[bk][sl])
            );
        end
    end
    assign seq_data_o = {rd[bank_q][2][31:0], rd[bank_q][1][31:0], rd[bank_q][0][31:0]};
`endif
```

Because the underlying macro is single-ported, the write commit pulse (`wr_commit`) arbitrates the address port. Port collisions are physically impossible during flight because write operations are locked out whenever `enable_i` is active.

---

## 6. Verification Defects, Silicon Race Conditions, and Engineering Fixes

### 6.1 Defect UCODE-01: Staging Register Stale-Data Bleed Across Fragmented Writes
- **Symptom:** During automated regression testing of microcode patching, rewriting instruction word `0x010` inadvertently corrupted bits `[63:0]` of word `0x015`.
- **Root Cause:** A host debug script wrote words 0 and 1 for address `0x010`, encountered an interrupt, aborted the transaction, and later initiated an upload starting at `0x015` by issuing only Phase 2 (`cpu_word_i = 2'b10`). The hardware committed the old staged fragments from `0x010` into `0x015`.
- **Engineering Fix:** Updated the firmware bootloader specification to require contiguous, uninterrupted 3-word writes. In the RTL verification environment, an SVA property was added to assert that `cpu_word_i` strictly cycles through `0 \to 1 \to 2` for every target address without address changes during intermediate phases.

### 6.2 Defect UCODE-02: Microcode Read Contention During Engine Launch
- **Symptom:** When the host CPU executed an MMIO readback to verify microcode CRC immediately as the temporal controller asserted `enable_i`, the sequencer fetched corrupted instruction words, causing the engine to halt on an invalid opcode exception.
- **Root Cause:** The address multiplexer lacked strict priority routing. The CPU read address conflicted with the sequencer fetch address on cycle 0.
- **Engineering Fix:** Implemented absolute ownership in `tide_engine`:
  ```systemverilog
  assign seq_addr_i = enable_i ? ucode_addr : uc_rd_addr_i;
  ```
  Whenever `enable_i = 1`, the sequencer unconditionally owns the memory read address port, isolating the microcode pipeline from host readback interference.
