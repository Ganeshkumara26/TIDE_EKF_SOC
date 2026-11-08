# Design: `tide_const_rom.sv` — 64×32-bit Constant ROM

**Spec reference:** §6.3 ("Constant ROM: 64 × 32 bits (π, π/2, 2π, atan coefficients, 1e-6, 0.41421356, floors, dt0⁻¹, …).")
**Gate:** G3
**New file**

---

## 1. Purpose

A small read-only memory that holds FP32 constants used by the microcode. The sequencer reads from this ROM using the LSU slot of the microinstruction. The ROM is synthesized as combinational logic (a big `case` statement) — it does not need an SRAM macro.

## 2. Interface

```systemverilog
module tide_const_rom (
    input  logic [5:0]  addr_i,       // 6-bit address (0–63)
    output logic [31:0] data_o        // Constant value
);
```

### Timing
- **Purely combinational.** Output is valid in the same cycle as the address. The sequencer/LSU registers the output on the next clock edge.
- No `clk_i` or `rst_ni` needed.

## 3. Contents

> [!NOTE]
> The polynomial coefficients have been frozen from the golden model reference.

| Index | Hex Value | Float Value | Name | Used By |
|---|---|---|---|---|
| 0 | `0x40490FDB` | 3.14159265 | π | GEOM (atan2 quadrant correction) |
| 1 | `0x3FC90FDB` | 1.57079633 | π/2 | GEOM (elevation) |
| 2 | `0x40C90FDB` | 6.28318531 | 2π | GEOM (azimuth wrap) |
| 3 | `0xC0490FDB` | -3.14159265 | -π | GEOM (atan2 negative quadrant) |
| 4 | `0x3F800000` | 1.0 | one | Various multiply identities |
| 5 | `0x00000000` | 0.0 | zero | Initialization, comparisons |
| 6 | `0xBF800000` | -1.0 | neg_one | Sign flips |
| 7 | `0x358637BD` | 1.0e-6 | us_to_s | PREDICT (dt_us × 1e-6 → dt_s) |
| 8 | `0x3ED413CD` | 0.41421356 | tan_pi_8 | ATAN2 (octant reduction threshold) |
| 9 | `0x401A827A` | 2.41421356 | cot_pi_8 | ATAN2 (octant reduction threshold) |
| 10 | `0x3F490FDB` | 0.78539816 | pi_4 | ATAN2 (octant reduction) |
| 11 | `0x3F7FFFF5` | 0.999999... | atan_c0 | ATAN2 polynomial coeff 0 |
| 12 | `0xBEAAA52E` | -0.333290... | atan_c1 | ATAN2 polynomial coeff 1 |
| 13 | `0x3E4C7B66` | 0.199690... | atan_c2 | ATAN2 polynomial coeff 2 |
| 14 | `0xBE084A28` | -0.133100... | atan_c3 | ATAN2 polynomial coeff 3 |
| 15 | `0x3727C5AC` | 1.0e-5 | floor_val | Covariance diagonal floor |
| 16 | `0x322BCC77` | 1.0e-8 | small_eps | Near-zero guard |
| 17 | `0x3F000000` | 0.5 | half | Various |
| 18 | `0x40000000` | 2.0 | two | Various |
| 19 | `0xC0000000` | -2.0 | neg_two | Various |
| 20–63 | `0x00000000` | 0.0 | reserved | Future use / padding |

## 4. Implementation

```systemverilog
module tide_const_rom (
    input  logic [5:0]  addr_i,
    output logic [31:0] data_o
);
    always_comb begin
        case (addr_i)
            6'd0:  data_o = 32'h40490FDB; // π
            6'd1:  data_o = 32'h3FC90FDB; // π/2
            6'd2:  data_o = 32'h40C90FDB; // 2π
            6'd3:  data_o = 32'hC0490FDB; // -π
            6'd4:  data_o = 32'h3F800000; // 1.0
            6'd5:  data_o = 32'h00000000; // 0.0
            6'd6:  data_o = 32'hBF800000; // -1.0
            6'd7:  data_o = 32'h358637BD; // 1e-6
            6'd8:  data_o = 32'h3ED413CD; // tan(π/8)
            6'd9:  data_o = 32'h401A827A; // cot(π/8)
            6'd10: data_o = 32'h3F490FDB; // π/4
            6'd11: data_o = 32'h3F7FFFF5; // c0
            6'd12: data_o = 32'hBEAAA52E; // c1
            6'd13: data_o = 32'h3E4C7B66; // c2
            6'd14: data_o = 32'hBE084A28; // c3
            6'd15: data_o = 32'h3727C5AC; // 1e-5 (floor)
            6'd16: data_o = 32'h322BCC77; // 1e-8
            6'd17: data_o = 32'h3F000000; // 0.5
            6'd18: data_o = 32'h40000000; // 2.0
            6'd19: data_o = 32'hC0000000; // -2.0
            default: data_o = 32'h00000000;
        endcase
    end
endmodule
```

## 5. Verification

Trivial — read every address and compare against the expected table. Add this as a sanity check at the start of any engine testbench.

**TEST REQUIRED:** Confirm that the test suite mathematically verifies the polynomial precision against the required accuracy threshold.
