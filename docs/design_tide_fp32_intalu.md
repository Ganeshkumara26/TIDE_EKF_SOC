# Design: `tide_fp32_intalu.sv` — Integer ALU + ITOF

**Spec reference:** §6.3 ("32-bit add/sub/compare/shift for timestamps, pointers, loop counters; ITOF (int32 → binary32, RNE) for dt.")
**Gate:** G3
**New file**

---

## 1. Purpose

Handles all integer arithmetic inside the TIDE engine: timestamp wrap-safe comparisons, loop counters for covariance iteration, pointer arithmetic for journal/checkpoint indexing, and the critical ITOF (integer-to-float) conversion used to turn `dt` (in integer microseconds) into FP32 for the predict step.

## 2. Interface

```systemverilog
module tide_fp32_intalu (
    input  logic        clk_i,
    input  logic        rst_ni,

    // Input
    input  logic        valid_i,
    input  logic [3:0]  op_i,         // Operation select
    input  logic [31:0] a_i,          // Operand A (integer)
    input  logic [31:0] b_i,          // Operand B (integer)

    // Output (single-cycle latency, registered)
    output logic        valid_o,
    output logic [31:0] result_o,     // Integer or FP32 result (for ITOF)
    output logic        cmp_result_o, // Comparison result (for branch)
    output logic        zero_o        // Result is zero
);
```

### Operation encoding (`op_i`)

| op_i | Operation | Description |
|---|---|---|
| 4'b0100 | IADD | `A + sign_ext(imm)` (32-bit wrap) |
| 4'b0101 | ISUB | `A - B` (reg-reg subtract) |
| 4'b0110 | ICMP | Signed `A < B` → `cmp_result_o` |
| 4'b0111 | ITOF | Convert signed int32 A to IEEE-754 FP32 (RNE) |
| 4'b1000 | LDLOOP | (Handled by Sequencer directly, but decoded here as passthrough) |
| 4'b1001 | LDIDX0 | (Handled by Sequencer directly) |
| 4'b1010 | LDIDX1 | (Handled by Sequencer directly) |
| 4'b1011 | IMOV | `result = sign_ext(imm)` |

### Timing
- **All operations:** 1 clock cycle (combinational logic, registered output).
- **Throughput:** 1/cycle.

## 3. ITOF Conversion (int32 → fp32, RNE)

This is the most complex operation in this unit.

### Algorithm
1. Handle zero: if `A == 0`, result = `0x00000000` (+0.0).
2. Determine sign: `sign = A[31]`. If negative, negate: `abs_val = -A`.
3. Find the position of the leading 1: `lz = CLZ(abs_val)`. The leading 1 is at bit position `31 - lz`.
4. IEEE exponent: `exp = 127 + (31 - lz) = 158 - lz`.
5. Mantissa: Left-shift `abs_val` by `lz` to normalize. The leading 1 is now at bit 31. The mantissa is bits `[30:8]` (23 bits).
6. **Rounding (RNE):**
   - Guard bit = `abs_val_shifted[7]`.
   - Round bit = `abs_val_shifted[6]`.
   - Sticky = `|abs_val_shifted[5:0]`.
   - Apply RNE: same logic as FP multiply/add.
   - If rounding overflows the mantissa: increment exponent.
7. Pack: `{sign, exp[7:0], mantissa[22:0]}`.

**TEST REQUIRED:** Verify ITOF against the C expression `(float)(int32_t)x` compiled with `-ffp-contract=off` for all 2³² values (or a representative random subset of 10⁶+ values). The ITOF result must be bit-exact.

### Critical use case
The spec says (§6.6 step 3): `dt = ITOF(Δt_µs) × 1e-6`. This means the ITOF converter is used every single predict call. The timer difference `Δt_µs` is a 32-bit integer in microseconds. The ITOF converts it to FP32, then the MUL lane multiplies by the constant `1e-6` to get `dt` in seconds.

## 4. Timestamp Comparison

The spec says (§6.6): "Timestamps use wrap-safe signed differences." This means event ordering is determined by `(int32_t)(t_event - t_now)`:
- If the signed difference is `>= 0` → in-order (or current).
- If the signed difference is `< 0` → late.

The ISUB + ICMP operations together implement this wrap-safe comparison.

## 5. Implementation Notes

1. **This is a simple, fully combinational unit** with a single output register. No pipeline stages needed.
2. **The CLZ (count leading zeros)** for ITOF is a 32-bit priority encoder. Same circuit as the LZC in the adder. In 180nm at 50 MHz this should close trivially.
3. **The barrel shifter** for ITOF normalization should easily meet timing.
4. **Reset:** `valid_o = 0` on reset. Everything else can be undefined.

## 6. Verification Plan

| Test | Method | Count | Pass Criterion |
|---|---|---|---|
| ITOF: powers of 2 | Verilator | 32 | Exact |
| ITOF: max positive/negative | Verilator | 10 | Bit-exact |
| ITOF: random int32 | Verilator | 10⁶ | Bit-exact vs C `(float)x` |
| ITOF: rounding boundaries | Verilator | ~500 | Correct RNE |
| ISUB wrap-around | Verilator | ~100 | Correct signed difference |
| All ops directed | Verilator | ~200 | Correct |
