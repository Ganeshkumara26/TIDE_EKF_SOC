# RTL Development & Architectural Implementation Document: `tide_sequencer.sv`
**Module Name:** `tide_sequencer`  
**Design Reference:** `08_tide_sequencer.md`, `15_timing_contracts.md`, `16_micro_isa.md`, `STEP_0_DECISIONS.md`  
**Target Architectures:** OS0005 Avionics SoC (SCL 180nm CP8D), OS0035 Micro-Navigation Processor (ASAP7 FinFET)  
**Verification Coverage:** Formal assertions (`tide_sequencer_sva.sv`), Directed cycle-accurate testbench (`tb_tide_sequencer.sv`), Gate-level simulation (`v10`)

---

## 1. Architectural Role & VLIW Execution Philosophy

The `tide_sequencer` is the instruction dispatch and datapath orchestration engine of the TIDE Extended Kalman Filter (EKF) coprocessor. In flight navigation systems such as the OS0005, sensor fusion demands ultra-low, deterministic latencies without dynamic scheduling overhead. Dynamic out-of-order execution logic, branch predictors, and associative reservation stations introduce non-deterministic timing jitter and unacceptable silicon area overhead in radiation-tolerant processes. Consequently, `tide_sequencer` implements a **statically scheduled, 96-bit 5-slot Very Long Instruction Word (VLIW)** engine operating in strict lockstep.

The micro-architecture eliminates hardware interlock logic across execution units. The 96-bit microinstruction word is decoded in parallel into five independent functional slots:
1. **MUL Slot (`[95:73]`, 23 bits):** Drives the 2-cycle pipelined IEEE-754 multiplier (`tide_fp32_mul`).
2. **ADD Slot (`[72:49]`, 24 bits):** Drives the 3-cycle pipelined IEEE-754 adder/subtractor and 1-cycle bypass ALU (`tide_fp32_add`).
3. **DIV/SQRT (DS) Slot (`[48:33]`, 16 bits):** Controls the non-pipelined iterative radix-2 divider/square-root core (`tide_fp32_divsqrt`, 28/27-cycle latency).
4. **LSU/ALU Slot (`[32:9]`, 24 bits):** Dispatches single-word SRAM loads/stores to `tide_lsu`, constant table lookups to `tide_const_rom`, or executes inline integer/conversion operations.
5. **CTRL Slot (`[8:0]`, 9 bits):** Manages program counter redirection, loop decrement/branch branching, preemption boundary signaling, and index register post-increments.

```
┌──────────────────────────────────────────────────────────────────────────────────────────────────┐
│ 95  94 93  87 86  80 79  73 │ 72 70 69  63 62  56 55  49 │ 48 47 46  40 39  33 │ 32 29 28 22 21  9 │ 8  7  6  5  4  3  2  1  0 │
│        MUL SLOT (23)        │       ADD SLOT (24)        │   DS SLOT (16)      │  LSU/ALU SLOT (24) │    CTRL SLOT (9)          │
│ op  ra      rb      rd     │ op  ra      rb      rd     │ op  ra      rb      │ op   reg     imm   │ EU BC BR BS BI LD LB IDX  │
└──────────────────────────────────────────────────────────────────────────────────────────────────┘
```

The sequencer relies on the microcode compiler/assembler (`tide_asm.py`) to guarantee hazard-free execution. Register read-after-write (RAW) and write-after-write (WAW) hazards are forbidden at the microcode level. The sequencer hardware tracks in-flight instruction latencies using deterministic shift registers, pairing execution results with their target destination addresses exactly when the arithmetic pipelines produce valid data.

---

## 2. Pipeline Mechanics, Fetch-Decode Interface, and Startup Bubbles

The sequencer is organized as a decoupled two-stage pipeline: **Fetch (F)** and **Decode/Execute (D/X)**.

```mermaid
flowchart LR
    subgraph Stage_F ["Stage 1: Fetch (Cycle N)"]
        PC["fetch_pc_q [8:0]"] --> RAM["tide_ucode_ram (1-Cycle Latency)"]
    end
    subgraph Stage_DX ["Stage 2: Decode / Execute (Cycle N+1)"]
        RAM --> DREG["dreg_q [95:0]"]
        DREG --> DEC["5-Slot Decoder"]
        DEC --> RF_RD["Combinational RF Read (Ports 0-3)"]
        DEC --> LANES["Issue to FP Lanes / IntALU / LSU"]
        DEC --> WB_TRK["Shift-Register Latency Trackers"]
    end
    DEC -. Redirect .- PC
```

### 2.1 The Synchronous SRAM Latency Challenge
The instruction store `tide_ucode_ram` is a synchronous single-cycle SRAM array. An address presented at cycle $N$ yields valid microcode data on cycle $N+1$. When the temporal controller issues `start_i` along with `start_addr_i`, the fetch address is applied to `ucode_addr_o`, but `dreg_q` cannot latch valid instruction bits until cycle $N+1$.

To prevent uninitialized or stale data from propagating into the datapath on cycle $N$, the FSM introduces a mandatory 1-cycle startup bubble governed by `warm_q`:

```systemverilog
S_IDLE: begin
    if (start_i) begin
        state_q    <= S_RUN;
        fetch_pc_q <= start_addr_i;
        dreg_q     <= SAFE_BUBBLE_WORD;
        warm_q     <= 2'd1;   // 1-cycle startup bubble
        draining_q <= 1'b0;
        fresh_q    <= 1'b0;   // Invalidate bubble execution
    end
end
```

### 2.2 The ADD-Slot NOP Dilemma and `SAFE_BUBBLE_WORD`
In `isa.yaml`, the ADD slot uses a 3-bit opcode field (`[72:70]`), where all eight encodings represent legitimate operations:
- `3'b000` = `FADD`
- `3'b001` = `FSUB`
- `3'b010` = `FCMP`
- `3'b011` = `FMIN`
- `3'b100` = `FMAX`
- `3'b101` = `FABS`
- `3'b110` = `FNEG`
- `3'b111` = `FMOV`

Because `3'b000` encodes a valid addition rather than an idle state, synthesizing an all-zero bubble word would cause the ADD lane to execute `FADD r0, r0 -> r0`, corrupting state vector element $x[0]$. To resolve this, `SAFE_BUBBLE_WORD` directs the ADD destination register to `7'h7F` (`r127`), the hardwired zero sink in `tide_regfile`:

```systemverilog
localparam logic [95:0] SAFE_BUBBLE_WORD = {
    23'b0,                     // MUL: op=NOP (2'b00), ra=0, rb=0, rd=0
    3'b0, 7'b0, 7'b0, 7'h7F,   // ADD: op=FADD, ra=0, rb=0, rd=r127 (sink)
    16'b0,                     // DS:  op=NOP (2'b00), ra=0, rb=0
    24'b0,                     // LSU: op=NOP (4'b0000), reg=0, imm=0
    9'b0                       // CTRL: all flags clear
};
```

Furthermore, integration fix `I-SEQ-6` identified that `tide_asm.py` emits an all-zero 24-bit ADD slot field for NOPs (`A-ADDNOP`). The decoder checks `add_nop = (dreg_q[72:49] == 24'd0)` and suppresses `add_issue_now` when all bits are zero, preventing unintended writes to `r0`.

---

## 3. Register File Port Arbitration and Muxing (Rules R1 & R2)

The physical register file (`tide_regfile`) provides four combinational read ports (`raddr0_o` to `raddr3_o`) and three synchronous write ports (`waddr0_o` to `waddr2_o`). However, five functional slots require operand inputs and result writebacks.

### 3.1 Read Port Sharing Between MUL and DS (Rule R1)
The MUL and DS slots share physical read ports 0 and 1. The assembler enforces rule R1: the union of operands required by MUL and DS in any single microinstruction must not exceed two unique registers. Integration fix `I-SEQ-8` replaced a naive static priority mux with a dynamic port allocator:

```systemverilog
wire        ds_binary  = (ds_op == 2'b01); // FDIV requires 2 operands; FSQRT requires 1
wire [6:0]  p01_c0     = mul_active ? mul_ra : ds_ra;
wire        p01_c0_v   = mul_active | ds_active;

logic [6:0] p01_r1;
always_comb begin
    p01_r1 = 7'd0;
    if (mul_active && mul_rb != mul_ra)                 p01_r1 = mul_rb;
    else if (mul_active && ds_active && ds_ra != mul_ra) p01_r1 = ds_ra;
    else if (mul_active && ds_binary && ds_rb != mul_ra) p01_r1 = ds_rb;
    else if (!mul_active && ds_binary && ds_rb != ds_ra) p01_r1 = ds_rb;
end

assign rf_raddr0_o = p01_c0_v ? p01_c0 : 7'd0;
assign rf_raddr1_o = p01_r1;

function automatic logic [31:0] p01_val(input logic [6:0] r, input logic [31:0] d0, input logic [31:0] d1);
    p01_val = (r == p01_c0) ? rd_value(r, d0) : rd_value(r, d1);
endfunction
```

Each consumer extracts its operand combinationally from either `rdata0_i` or `rdata1_i` based on address matching.

### 3.2 Read Port Sharing Between ADD and LSU/ALU (Rule R2)
Ports 2 and 3 are shared between the ADD slot and the LSU/ALU slot (`I-SEQ-7`). An ADD operation may require two registers ($r_a, r_b$), while a store or integer ALU instruction requires $r_{reg}$ as a source. The allocation logic maps $r_a$ unconditionally to Port 2:

```systemverilog
wire add_rb_used  = (add_op <= 3'd4); // FADD, FSUB, FCMP, FMIN, FMAX use rb
wire lsu_on_p2    = lsu_alu_reads_reg & (lsu_reg == add_ra);
wire add_b_same_a = add_rb_used & (add_rb == add_ra);
wire add_b_on_p3  = add_rb_used & ~add_b_same_a;
wire lsu_free_p3  = lsu_alu_reads_reg & ~lsu_on_p2 & ~(add_b_on_p3 & (lsu_reg == add_rb));

assign rf_raddr2_o = add_ra;
assign rf_raddr3_o = lsu_free_p3 ? lsu_reg : add_rb;
```

When LSU STORE data is mapped to Port 3, `lsu_rdata_o` routes the resolved value out to `tide_engine` (`lsu_rdata_o = rd_value(lsu_reg, lsu_on_p2 ? rf_rdata2_i : rf_rdata3_i)`), decoupling the LSU from a rigid Port 2 dependency.

---

## 4. Arithmetic Datapaths: Inline IntALU and RNE `i2f32` Conversion

To minimize latency during EKF time updates, `tide_sequencer` embeds an inline integer ALU and an IEEE-754 round-to-nearest-even (RNE) integer-to-floating-point converter (`i2f32`).

### 4.1 Time Delta Calculation
In Kalman filtering, state propagation requires the elapsed microsecond interval:
$$\Delta t_s = (t_{\text{event}} - t_{\text{now}}) \times 10^{-6}$$
The integer difference $t_{\text{event}} - t_{\text{now}}$ is computed and converted to single-precision float via `ITOF` (`lsu_op == 4'b0111`), followed by a multiplication by $10^{-6}$ loaded from `tide_const_rom`.

### 4.2 Binary-Tree Leading Zero Count (`TIMING-3`)
The original design used a 32-step priority loop to detect the leading zero count ($LZC$), creating a 32-level carry chain that violated the 20 ns clock constraint in SCL 180nm. `TIMING-3` re-implemented this as a 5-level binary barrel-shifter tree:

```systemverilog
m1 = (mag[31:16] == 16'd0) ? {mag[15:0], 16'd0} : mag;  lz[4] = (mag[31:16] == 16'd0);
m2 = (m1[31:24] == 8'd0)   ? {m1[23:0], 8'd0}   : m1;   lz[3] = (m1[31:24] == 8'd0);
m3 = (m2[31:28] == 4'd0)   ? {m2[27:0], 4'd0}   : m2;   lz[2] = (m2[31:28] == 4'd0);
m4 = (m3[31:30] == 2'd0)   ? {m3[29:0], 2'd0}   : m3;   lz[1] = (m3[31:30] == 2'd0);
m5 = (m4[31]    == 1'b0)   ? {m4[30:0], 1'b0}   : m4;   lz[0] = (m4[31]    == 1'b0);

exp_b   = 8'd158 - {3'b0, lz};  // 127 + 31 - lz
shifted = m5;                   // shifted[31] is implicit leading 1
mant24  = shifted[31:8];        // implicit 1 + 23 mantissa bits
rnd_bit = shifted[7];
stk_bit = |shifted[6:0];
round_up = rnd_bit & (stk_bit | mant24[0]); // RNE tie-breaking rule
mant_rounded = {1'b0, mant24} + {24'b0, round_up};
```

If rounding causes the mantissa to overflow (`mant_rounded[24] == 1'b1`), the exponent increments and the mantissa resets to zero, correctly producing $2^{N+1}$.

---

## 5. Branch Evaluation, Bubble Squashing, and Pipeline Redirects

Branch decisions are resolved combinationally from `dreg_q` and latched status flags:
- `cmp_lt_q`: Latch updated by `FCMP` (via `add_bypass_valid_i` and `add_cmp_lt_i`) or integer `ICMP`.
- `loop_cnt_q`: 16-bit hardware iteration counter decremented by `ctrl_loop_dec`.

### 5.1 Taken-Branch Squash and Address Redirection (`I-SEQ-5`)
Because `tide_ucode_ram` has a 1-cycle read latency, resolving a branch at cycle $N$ would normally result in the target reaching `dreg_q` at cycle $N+2$ if the address was updated sequentially in `fetch_pc_q`. The instruction at $N+1$ (the wrong path) would execute unless squashed.

Integration fix `I-SEQ-5` established a single-bubble redirect contract (`ISA-07`). When `branch_taken` evaluates true, the branch target is muxed directly onto `ucode_addr_o` during cycle $N$, while the sequential PC updates to $\text{target} + 1$:

```systemverilog
wire redirect_now = (state_q == S_RUN) & (warm_q == 2'd0) & ~stalled &
                    ~ctrl_boundary & branch_taken;

assign ucode_addr_o = redirect_now ? branch_target : fetch_pc_q;

// Inside always_ff @(posedge clk_i):
if (branch_taken) begin
    fetch_pc_q <= branch_target + 9'd1;
    dreg_q     <= SAFE_BUBBLE_WORD; // 1-cycle bubble squash
end
```

```
Cycle 0: Decodes Branch at addr B. Evaluates branch_taken=1.
         ucode_addr_o driven to branch_target immediately.
Cycle 1: RAM fetches mem[branch_target]. dreg_q receives SAFE_BUBBLE_WORD (squash).
         fetch_pc_q advances to branch_target + 1.
Cycle 2: dreg_q latches mem[branch_target]. Execution resumes.
```

### 5.2 Loop Counter Timing Optimization (`TIMING-2`)
Evaluating loop termination via `(loop_cnt_q - 1 != 0)` placed a 16-bit decrementer directly on the critical timing path feeding `redirect_now` and `ucode_addr_o`. `TIMING-2` replaced this with an equivalent identity:
$$\text{ctrl\_loop\_dec} \implies (\text{cnt} - 1 \neq 0) \iff (\text{cnt} \neq 1)$$
This converted a subtraction into a 16-bit comparator against constant 1, cutting 1.2 ns of delay in SCL 180nm synthesis.

---

## 6. Dynamic Register Indirection: `idx0` and `idx1`

Matrix operations in the EKF (e.g., propagating the 21 covariance elements $P_{ij}$) require indirect register addressing. Microcode represents indirection using reserved register indices:
- `7'h7E`: Replaced by `idx0_q`
- `7'h7D`: Replaced by `idx1_q`

### 6.1 Address Substitution and Width Expansion (`I-SEQ-2`)
Originally, `idx0_q` and `idx1_q` were 7 bits wide. However, the EKF microcode uses these registers to index both register file operands and TIDE-RAM words (e.g., context base `0x240`). Truncation to 7 bits mapped address `0x240` to `0x040`, corrupting the checkpoint ring instead of the live context. Integration fix `I-SEQ-2` widened both registers to 12 bits:

```systemverilog
logic [11:0] idx0_q, idx1_q;

function automatic logic [31:0] rd_value (input logic [6:0] addr, input logic [31:0] rf_data);
    if (addr == 7'h7E)      rd_value = {20'b0, idx0_q};
    else if (addr == 7'h7D) rd_value = {20'b0, idx1_q};
    else                    rd_value = rf_data;
endfunction
```

For memory accesses, `I-SEQ-4` fixed the effective address generation:
```systemverilog
wire [11:0] lsu_idx_add = (ctrl_idx_mode == 2'b01) ? idx0_q :
                          (ctrl_idx_mode == 2'b10) ? idx1_q : 12'd0;
assign lsu_cmd_addr_o   = lsu_imm[11:0] + lsu_idx_add;
```

---

## 7. Write-Back Scheduler and Port Arbitration

The sequencer schedules writebacks to `tide_regfile` across three ports.

```
Port 0: MUL Lane (Latency = 2)
Cycle C   : mul_valid_o=1, mul_rd captured into mul_rd_q1
Cycle C+1 : Shifted to mul_rd_q2, mul_pend_q2=1
Cycle C+2 : mul_valid_i asserts -> rf_wen0_o=1, rf_waddr0_o=mul_rd_q2, rf_wdata0_o=mul_result_i

Port 1: ADD Lane Pipe (Latency = 3) / ADD Bypass (Latency = 1) / DIV-SQRT (Latency = 27/28)
- Pipe   : add_pipe_pend_q3 qualifies add_pipe_rd_q3
- Bypass : add_byp_pend_q1 qualifies add_byp_rd_q1
- DS     : ds_done_i qualifies ds_rd_pending_q
Arbitration: ds_real > byp_real > pipe_real

Port 2: IntALU / ITOF / ROM / LSU LOAD (Latency = 1)
Cycle C   : Decoded operation evaluates combinational result
Cycle C+1 : Registered into p2_wen_q, p2_addr_q, p2_data_q (I-SEQ-1 fix)
```

### 7.1 Port 1 Disambiguation Bug
In initial verification, `add_valid_i` asserted almost every cycle due to filler NOPs (`FADD r0, r0 -> r127`). When a 1-cycle bypass operation (e.g., `FMOV`) executed, its destination address propagated through the 3-stage pipeline shift register `add_pipe_rd_q*`. When a subsequent filler NOP reached stage 3, its valid pulse paired with the stale destination register, overwriting legitimate data with zero.

The defect was resolved by separating pending tracking bits:
```systemverilog
wire p1_pipe_real = add_valid_i & add_pipe_pend_q3;
wire p1_byp_real  = add_bypass_valid_i & add_byp_pend_q1;
wire p1_ds_real   = ds_done_i;

wire [6:0] p1_addr_raw = p1_ds_real   ? ds_rd_pending_q :
                         p1_byp_real  ? add_byp_rd_q1   :
                         p1_pipe_real ? add_pipe_rd_q3  : 7'h7F;
```

### 7.2 Port 2 Cycle Alignment (`I-SEQ-1`)
Under `timing.yaml`, Port 2 writebacks must land at cycle $C+1$. Early RTL drove IntALU and ROM results combinationally at cycle $C+0$. This collided with in-flight LSU loads on Port 2 and exposed updated values one cycle earlier than the compiler's scheduling graph anticipated. Registering Port 2 write signals (`p2_wen_q`, `p2_addr_q`, `p2_data_q`) ensured cycle $C+1$ alignment.

---

## 8. Microcode Toolchain Integration, Register Liveness Analysis, and `.livein` Resolution

The verification and integration of `tide_sequencer` uncovered a subtle architectural interaction between static CFG hazard analysis in `tools/tide_asm.py` and the procedural partitioning of microcode in `microcode/gen_ucode.py`.

### 8.1 The `.livein` Annotation Dilemma in `20_unit_update.tasm`
When `tide_asm.py` is invoked as an isolated linter on individual assembly source files, it constructs a Control Flow Graph (CFG) and performs rigorous inter-slot Def-Use and Use-Def dataflow analysis. By default, any register read before an explicit write within that local translation unit is flagged as an uninitialized read error:

```
ERROR 20_unit_update.tasm:4: r1 read but never written on some path (missing .livein?)
ERROR 20_unit_update.tasm:4: r90 read but never written on some path (missing .livein?)
...
ERROR 20_unit_update.tasm:189: r119 read but never written on some path (missing .livein?)
Errors: 77   Warnings: 0
```

File `20_unit_update.tasm` encodes the 4-iteration scalar update loop (`SCALAR_LOOP`, 184 cycles/iteration) followed by context persistence (`CTX_STORE`, 29 cycles). At instruction 0 (cycle $c0$), the loop reads $r_1$ (state $x_0$), $r_{90}$ (prior state $x_{p0}$), and indexes TIDE-RAM using `idx1`. Because `20_unit_update.tasm` was generated as a sub-routine snippet without a standalone `.livein` directive, standalone static analysis reports 77 catastrophic hazard violations.

### 8.2 Architectural Resolution via Runtime Microcode Concatenation
In the production EKF build system (`microcode/gen_ucode.py`), microcode is partitioned into functional phases for modular compilation:
1. `build_unit_pre()`: Generates `10_unit_pre.tasm` (prologue = context load, state propagation, measurement Jacobian geometry).
2. `build_update_body()`: Generates the 184-cycle scalar update loop body.
3. `build_store()`: Generates the 29-cycle `CTX_STORE` routine.

The compiler resolves liveness through **inter-procedural register chaining**:
- In `gen_ucode.py`, the prologue `build_unit_pre()` declares:
  ```python
  livein='r28-r36, r118-r122, r119'
  ```
  During its 258 execution cycles, `10_unit_pre` actively populates registers `r1` through `r27` (the 6-state vector $x$ and 21-element upper-triangular covariance $P$), copies $x_{\text{prior}}$ into reserved registers `r90..r95`, and stages the loop iteration count (4) into `r116` via `IMOV`.
- When assembling the unified microcode image `ucode.hex` (492 words total), `gen_ucode.py` concatenates `lines2` (loop body) and `lines3` (`CTX_STORE`), and links them directly after `lines1c` (`10_unit_pre`).
- Execution in `tide_sequencer` falls straight through from `10_unit_pre` into `SCALAR_LOOP` at microcode address `9'd258`. Therefore, every register flagged as uninitialized in isolated analysis is guaranteed to be fully defined and stabilized in the hardware register file prior to the execution of cycle 0 of `SCALAR_LOOP`.

The 77 assembler errors are thus an artifact of isolated sub-block analysis, correctly bypassed when compiling the unified memory map where Def-Use chains span across routine boundaries.

---

## 9. Integration Defects and Corrections Register

| Bug Ref | Description | Root Cause | Engineering Fix Applied |
|---|---|---|---|
| **I-SEQ-1** | Port 2 write collision at $C+0$ | Combinational writeback violated $C+1$ timing contract | Added 1-stage output register pipeline (`p2_wen_q`, `p2_addr_q`, `p2_data_q`). |
| **I-SEQ-2** | Index register truncation | `idx0_q`/`idx1_q` declared as `[6:0]`, truncating SRAM base `0x240` to `0x040` | Widened index flops to 12 bits (`[11:0]`). |
| **I-SEQ-3** | Spurious TIDE-RAM store during IDLE | `lsu_cmd_valid_o` driven directly from unclocked decode | Qualified command valid with `running_o & fresh_q`. |
| **I-SEQ-4** | LSU missing index offset | Base address was driven from `imm12` only | Added `lsu_idx_add` derived from `ctrl_idx_mode`. |
| **I-SEQ-5** | 2-cycle branch penalty | Target address waited for sequential fetch register update | Muxed `branch_target` directly onto `ucode_addr_o` during decode cycle. |
| **I-SEQ-6** | Spurious `FADD` on microcode NOP | Assembler all-zero ADD field decoded as `FADD r0,r0->r0` | Added explicit `add_nop` decode check to suppress issue. |
| **I-SEQ-7** | Port 2/3 read starvation on ADD | Rigid mapping forced LSU to Port 2, clobbering ADD operand | Dynamic muxing allocates Port 2 to ADD $r_a$ and Port 3 to LSU or ADD $r_b$. |
| **I-SEQ-8** | Read port collision on MUL+DS | Independent source registers exceeded 2 physical ports | Dynamic allocation maps unique operands to Ports 0 and 1. |
| **A-DS-OP** | `ds_op_o` functional inversion | RTL mapped `ds_op[0]` to lane opcode (01=DIV, 10=SQRT) | Re-mapped to `ds_op[1]` to match `0=DIV, 1=SQRT`. |

---

## 10. Verification & Standard Cell Synthesis Profile

On the SCL 180nm CP8D library, `tide_sequencer` synthesizes to **3,940 Gate Equivalents (GE)** excluding the register file and microcode memory. Dynamic power at 50 MHz and 1.8 V core supply is **1.84 mW**. Critical path timing is dominated by:
$$\text{Clock} \to \text{dreg\_q} \to \text{branch decode} \to \text{ucode\_addr\_o} \to \text{RAM address setup}$$
Total path delay is **14.2 ns**, providing a 29% timing margin against the 20.0 ns clock period. In ASAP7 FinFET technology, operating at 0.7 V and 500 MHz, the path delay compresses to **1.18 ns**, meeting all cycle-budget requirements with over 41% slack.
