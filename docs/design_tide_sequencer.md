# Design: `tide_sequencer.sv` — VLIW Microcode Sequencer

**Spec reference:** §6.5 ("Statically scheduled: no hardware interlocks; results appear at fixed cycles; the assembler checks hazards and port conflicts."), Appendix C
**Gate:** G5
**New file**

---

## 1. Purpose

The sequencer is the heart of the TIDE engine. It fetches 96-bit VLIW microinstructions from the microcode RAM, decodes the 5 parallel slots, drives the datapath (register file reads → lane inputs, lane outputs → register file writes), manages the program counter, loop counters, index registers, and signals event-boundary checkpoints to the temporal controller.

## 2. Interface

```systemverilog
module tide_sequencer (
    input  logic        clk_i,
    input  logic        rst_ni,

    // Control (from temporal controller)
    input  logic        start_i,      // Begin executing from start_addr
    input  logic [8:0]  start_addr_i, // Initial PC (entry point)
    output logic        done_o,       // Unit completed (at END_OF_UNIT instruction)
    output logic        boundary_o,   // At event boundary (for preemption check)
    output logic        running_o,    // Sequencer is actively executing

    // Microcode RAM read port
    output logic [8:0]  ucode_addr_o,
    input  logic [95:0] ucode_data_i, // 1-cycle read latency

    // Register file ports
    output logic [6:0]  rf_raddr0_o, rf_raddr1_o, rf_raddr2_o, rf_raddr3_o,
    input  logic [31:0] rf_rdata0_i, rf_rdata1_i, rf_rdata2_i, rf_rdata3_i,
    output logic        rf_wen0_o, rf_wen1_o, rf_wen2_o,
    output logic [6:0]  rf_waddr0_o, rf_waddr1_o, rf_waddr2_o,
    output logic [31:0] rf_wdata0_o, rf_wdata1_o, rf_wdata2_o,

    // MUL lane
    output logic        mul_valid_o,
    output logic [31:0] mul_a_o, mul_b_o,
    input  logic        mul_valid_i,
    input  logic [31:0] mul_result_i,

    // ADD lane
    output logic        add_valid_o,
    output logic [2:0]  add_op_o,
    output logic [31:0] add_a_o, add_b_o,
    input  logic        add_valid_i,
    input  logic [31:0] add_result_i,
    input  logic        add_bypass_valid_i,
    input  logic [31:0] add_bypass_result_i,
    input  logic        add_cmp_lt_i,

    // DIV/SQRT unit
    output logic        ds_start_o,
    output logic [31:0] ds_a_o, ds_b_o,
    output logic        ds_op_o,      // 0=div, 1=sqrt
    input  logic        ds_busy_i,
    input  logic        ds_done_i,
    input  logic [31:0] ds_result_i,

    // Integer ALU
    output logic        ialu_valid_o,
    output logic [3:0]  ialu_op_o,
    output logic [31:0] ialu_a_o, ialu_b_o,
    input  logic [31:0] ialu_result_i,
    input  logic        ialu_cmp_i,

    // Constant ROM
    output logic [5:0]  rom_addr_o,
    input  logic [31:0] rom_data_i,

    // LSU (Load/Store to TIDE-RAM)
    output logic        lsu_valid_o,
    output logic        lsu_we_o,
    output logic [11:0] lsu_addr_o,
    output logic [31:0] lsu_wdata_o,
    input  logic [31:0] lsu_rdata_i,
    input  logic        lsu_ready_i
);
```

## 3. Pipeline Structure

The sequencer has a **2-stage pipeline**: Fetch → Decode/Execute.

### Fetch (cycle N)
- Present `PC` to `ucode_addr_o`.
- Increment `PC` (or load branch target if previous instruction was a taken branch).
- Microcode RAM returns data on cycle N+1.

### Decode/Execute (cycle N+1)
- Decode the 96-bit instruction into 5 slots.
- Present register file read addresses (combinational read → data available same cycle).
- Drive lane inputs.
- Lane results arrive after their respective latencies (MUL: 2 cycles, ADD: 3 cycles, DIV/SQRT: 28/27 cycles). The assembler guarantees that the destination register write occurs at the exact cycle the result is ready.

### Write-back
- Write-back is not a separate pipeline stage. The sequencer tracks in-flight operations and writes results to the register file when they complete. This is managed by the **write-back scheduler** — a set of shift registers that track which destination register to write and when.

## 4. Microinstruction Decode (96 bits, Appendix C)

```
[95:73] MUL slot    — 23 bits: op(2) + ra(7) + rb(7) + rd(7)
[72:49] ADD slot    — 24 bits: op(3) + ra(7) + rb(7) + rd(7)
[48:33] DIV/SQRT    — 16 bits: op(2) + ra(7) + rb(7)
[32:9]  LSU/ALU/ITOF— 24 bits: op(4) + reg(7) + addr/imm(13)
[8:0]   Control     —  9 bits: see below
```

### MUL slot (bits [95:73])

| Bits | Name | Description |
|---|---|---|
| [95:94] | mul_op | 00=NOP, 01=MUL, 10=reserved, 11=reserved |
| [93:87] | mul_ra | Source register A (7-bit) |
| [86:80] | mul_rb | Source register B (7-bit) |
| [79:73] | mul_rd | Destination register (7-bit, write occurs 2 cycles later) |

### ADD slot (bits [72:49])

| Bits | Name | Description |
|---|---|---|
| [72:70] | add_op | 000=NOP, 001=ADD, 010=SUB, 011=CMP, 100=MIN, 101=MAX, 110=ABS, 111=MOV |
| [69:63] | add_ra | Source register A |
| [62:56] | add_rb | Source register B |
| [55:49] | add_rd | Destination register (write occurs 3 cycles later for ADD/SUB, 1 cycle for bypass) |

### DIV/SQRT slot (bits [48:33])

| Bits | Name | Description |
|---|---|---|
| [48:47] | ds_op | 00=NOP, 01=DIV, 10=SQRT, 11=reserved |
| [46:40] | ds_ra | Source register A (dividend/radicand) |
| [39:33] | ds_rb | Source register B (divisor, ignored for sqrt) |

The destination register for DIV/SQRT is a **pseudo-register**: the sequencer reads the result from `ds_result_i` when `ds_done_i` pulses and writes it to a pre-configured destination register. The destination is encoded in the `_rd` field of the instruction that issued the start (or in a following NOP instruction — **design decision: encode it in the LSU/ALU slot of the same instruction**).

### LSU/ALU/ITOF slot (bits [32:9])

| Bits | Name | Description |
|---|---|---|
| [32:29] | lsu_op | 0000=NOP, 0001=LOAD, 0010=STORE, 0011=LOAD_ROM, 0100=IADD, 0101=ISUB, 0110=ICMP, 0111=ITOF, 1000=LDLOOP, 1001=LDIDX0, 1010=LDIDX1, 1011=IMOV |
| [28:22] | lsu_reg | Register (source for STORE/LD*, destination for LOAD/ALU) |
| [21:9] | lsu_imm | 13-bit immediate (address for LOAD/STORE, immediate value for ALU) |

### Control slot (bits [8:0])

| Bits | Name | Description |
|---|---|---|
| [8] | end_of_unit | If 1, sequencer waits for all in-flight ops (MUL/ADD/DS) to retire (drain shift registers), then signals `done_o` and halts. |
| [7] | boundary_check | If 1, signal `boundary_o` (for preemption). Sequencer pauses until `start_i` resumes. |
| [6] | branch | If 1, the next PC is `branch_target` instead of PC+1. |
| [5] | branch_cond | If 1, branch is conditional on the last comparison result (`cmp_lt`). If 0, branch is unconditional. |
| [4] | branch_sense | If 1, branch taken when condition is TRUE. If 0, branch taken when condition is FALSE. |
| [3] | loop_dec | Decrement the loop counter. |
| [2] | loop_branch | Branch to `branch_target` if loop counter > 0 (after decrement). |
| [1:0] | idx_mode | 00=none, 01=post-increment index register 0, 10=post-increment index register 1, 11=both |

The branch target is the `lsu_imm[8:0]` field (9 bits, enough for 512 addresses). When `branch=1`, the LSU slot's immediate field is repurposed as the branch target.

## 5. Index Registers

Two 7-bit index registers (`idx0`, `idx1`) with post-increment. They are used for iterating over the packed covariance (21 elements) and the nonzero Jacobian columns.

- **Set:** via the LSU slot (`LDIDX0` or `LDIDX1`).
- **Post-increment:** when `idx_mode` is set, the addressed index register increments by 1 after the instruction executes.
- **Usage in register addresses:** When a register address field contains a special code (e.g., `7'h7E` = "use idx0", `7'h7D` = "use idx1"), the sequencer substitutes the index register's value as the actual register address.

## 6. Write-Back Scheduler

The sequencer must track in-flight operations and write their results to the RF at the correct cycle:

| Source | Latency | Write-back mechanism (Port) |
|---|---|---|
| MUL lane | 2 cycles | 2-stage shift register (Port 0) |
| ADD lane (add/sub) | 3 cycles | 3-stage shift register (Port 1) |
| ADD lane (bypass) | 1 cycle | Immediate write-back (Port 1) |
| DIV/SQRT | 28 or 27 cycles | Watch `ds_done_i`, write `ds_result_i` to stored `rd` (Port 1) |
| LSU LOAD / ROM | 1 cycle | Immediate write-back (Port 2) |
| IntALU | 1 cycle | Immediate write-back (Port 2) |

Multiple write-backs in the same cycle to different ports are fully supported. Same-port conflicts are avoided by the assembler.

## 7. Stall Conditions

The spec says "no hardware interlocks." However, the sequencer must stall if:
- **LSU is not ready** (`lsu_ready_i = 0`): the TIDE-RAM is busy with a burst transfer.
- **DIV/SQRT is busy** and a new DIV/SQRT is issued: this should never happen (assembler prevents it), but the hardware should ignore the start.

## 8. Verification Plan

| Test | Pass Criterion |
|---|---|
| NOP-only program (100 NOPs + END) | Sequencer starts, runs 100 cycles, asserts done_o |
| MUL-only: load two regs, multiply, store result | Correct result after 2 cycles |
| ADD-only: similar | Correct after 3 cycles |
| MUL+ADD parallel | Both results correct, no port conflict |
| DIV: start, wait 28 cycles, read result | Correct |
| Loop: decrement + branch back | Correct iteration count |
| Index register post-increment | Addresses increment correctly |
| Boundary check + resume | Sequencer pauses and resumes on start_i |
| End of unit | done_o asserted, sequencer halts |

**TEST REQUIRED:** Full integration test with loaded microcode running predict+update and comparing RF contents against golden model.
