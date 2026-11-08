# Design: `tide_regfile.sv` — 128×32, 4-Read / 3-Write Register File

**Spec reference:** §6.5 ("Register file: 128 × 32 bits (flops), 4 read / 3 write ports for the lanes and memory/ALU. Holds the active context only.")
**Gate:** G5
**New file**

---

## 1. Purpose

The register file is the central data store for the TIDE engine's active computation. It holds the current EKF state vector `x[6]`, the covariance lower triangle `P[21]`, temporaries, and constants loaded from the constant ROM. It does NOT hold checkpoints, journals, or shadow contexts — those live in TIDE-RAM.

## 2. Interface

```systemverilog
module tide_regfile (
    input  logic        clk_i,
    input  logic        rst_ni,       // Reset clears valid_o signals only

    // Read port 0 (MUL lane, operand A)
    input  logic [6:0]  raddr0_i,
    output logic [31:0] rdata0_o,

    // Read port 1 (MUL lane, operand B)
    input  logic [6:0]  raddr1_i,
    output logic [31:0] rdata1_o,

    // Read port 2 (ADD lane, operand A — shared with LSU)
    input  logic [6:0]  raddr2_i,
    output logic [31:0] rdata2_o,

    // Read port 3 (ADD lane, operand B — shared with LSU)
    input  logic [6:0]  raddr3_i,
    output logic [31:0] rdata3_o,

    // Write port 0 (MUL lane result)
    input  logic        wen0_i,
    input  logic [6:0]  waddr0_i,
    input  logic [31:0] wdata0_i,

    // Write port 1 (ADD lane result or DIV/SQRT)
    input  logic        wen1_i,
    input  logic [6:0]  waddr1_i,
    input  logic [31:0] wdata1_i,

    // Write port 2 (LSU load, IntALU, Const ROM)
    input  logic        wen2_i,
    input  logic [6:0]  waddr2_i,
    input  logic [31:0] wdata2_i
);
```

## 3. Register Map (Convention)

The microcode assembler uses symbolic names. The hardware just sees 7-bit addresses. This is the convention:

| Range | Count | Purpose |
|---|---|---|
| r0–r5 | 6 | State vector `x[0..5]` = `[pn, pe, pd, vn, ve, vd]` |
| r6–r26 | 21 | Covariance `P[0..20]` (lower triangle, row-major) |
| r27 | 1 | `t_now` (current estimator time, integer µs) |
| r28–r31 | 4 | Measurements `z[0..3]` = `[range, az, el, doppler]` |
| r32–r35 | 4 | Measurement noise `R[0..3]` |
| r36–r39 | 4 | Innovations `y[0..3]` |
| r40–r63 | 24 | Temporaries (Jacobian row, `u` vector, gains, etc.) |
| r64–r69 | 6 | Saved state `x_prior[0..5]` (for innovation correction) |
| r70–r79 | 10 | Geometry intermediates (`r, r_g, 1/r, 1/r_g², az, el, ...`) |
| r80–r95 | 16 | Scratch for Joseph update |
| r96–r111 | 16 | Additional scratch |
| r112–r119 | 8 | Loop/index temporaries |
| r120–r124 | 5 | Config / Temporaries |
| r125 | 1 | Index register 1 (idx1) |
| r126 | 1 | Index register 0 (idx0) |
| r127 | 1 | Zero register (hardwired 0, writes are ignored) |

> [!IMPORTANT]
> Register r127 is **hardwired to zero**. Reads always return `0x00000000`. Writes to r127 are silently discarded. This simplifies microcode (no need for a separate zero source).

## 4. Read/Write Behavior

- **Reads:** Synchronous. Address presented on cycle N, data available on cycle N (same cycle, combinational read). This is standard for flop-based register files.
  
  **ALTERNATIVE (if timing is tight):** Make reads registered (address on cycle N, data on cycle N+1). This changes the sequencer's pipeline depth. **The sequencer design assumes combinational reads. Do not change this without updating the sequencer.**

- **Writes:** Synchronous, on the rising clock edge. Data written on cycle N is available for read on cycle N+1.

- **Write-write conflict:** The assembler forbids Write-After-Write hazards in the same cycle. If it happens, the highest port number wins (Port 2 > Port 1 > Port 0). The hardware must handle it deterministically.

- **Read-write in same cycle:** A read and write to the same address in the same cycle returns the **old** value (read-before-write). The new value is available next cycle. The assembler accounts for this.

## 5. Implementation

```systemverilog
module tide_regfile (
    // ... ports as above ...
);
    // Storage: 128 × 32-bit flops
    logic [31:0] rf [0:127];

    // Reads (combinational)
    assign rdata0_o = (raddr0_i == 7'd127) ? 32'h0 : rf[raddr0_i];
    assign rdata1_o = (raddr1_i == 7'd127) ? 32'h0 : rf[raddr1_i];
    assign rdata2_o = (raddr2_i == 7'd127) ? 32'h0 : rf[raddr2_i];
    assign rdata3_o = (raddr3_i == 7'd127) ? 32'h0 : rf[raddr3_i];

    // Writes (synchronous, highest port wins on conflict)
    always_ff @(posedge clk_i) begin
        if (wen0_i && waddr0_i != 7'd127)
            rf[waddr0_i] <= wdata0_i;
        if (wen1_i && waddr1_i != 7'd127)
            rf[waddr1_i] <= wdata1_i;  // Port 1 overwrites port 0 if same address
        if (wen2_i && waddr2_i != 7'd127)
            rf[waddr2_i] <= wdata2_i;  // Port 2 overwrites port 1/0
    end
endmodule
```

## 6. Area Estimate

128 × 32 = 4096 flip-flops. At ~8 GE per flip-flop in 180nm: ~33k GE. The 4-read muxing adds ~16k GE (four 128:1 × 32-bit muxes). Total: **~50k GE ≈ 0.4–0.7 mm²** at SCL 180nm. This matches the spec's estimate [E].

## 7. Verification

| Test | Pass Criterion |
|---|---|
| Write port 0, read back from each read port | Correct data |
| Write port 1, read back | Correct data |
| Write-write conflict (both ports, same addr) | Port 1 wins |
| Read-write same cycle, same addr | Old value returned |
| r127 reads as zero | Always 0x00000000 |
| r127 write ignored | No side effects |
| All 128 addresses | Full coverage |
