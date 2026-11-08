# Design: `tide_fp32_add.sv` — IEEE-754 FP32 Adder / Subtractor

**Spec reference:** §6.3, Decision T5 (No FMA — add is a separate rounding stage)
**Gate:** G3
**New file** (does not exist yet)

---

## 1. Purpose

A pipelined IEEE-754 binary32 adder/subtractor. This is the second primary arithmetic lane. It performs add, subtract, compare, min, max, abs, neg, and select operations. The compare/abs/neg/select operations are single-cycle (bypass the pipeline).

## 2. Interface

```systemverilog
module tide_fp32_add (
    input  logic        clk_i,
    input  logic        rst_ni,

    // Pipeline input
    input  logic        valid_i,
    input  logic [2:0]  op_i,         // Operation select (see below)
    input  logic [31:0] a_i,          // Operand A
    input  logic [31:0] b_i,          // Operand B

    // Pipeline output
    output logic        valid_o,      // Result available (3 cycles after valid_i for add/sub)
    output logic [31:0] result_o,
    output logic [4:0]  flags_o,      // {invalid, div_by_zero(0), overflow, underflow, inexact}

    // Single-cycle bypass output (for cmp/abs/neg/sel)
    output logic        bypass_valid_o,
    output logic [31:0] bypass_result_o,
    output logic        cmp_lt_o,     // A < B (for select/branch)
    output logic        cmp_eq_o      // A == B
);
```

### Operation encoding (`op_i`)

| op_i | Operation | Latency | Description |
|---|---|---|---|
| 3'b000 | ADD | 3 cycles | `A + B` |
| 3'b001 | SUB | 3 cycles | `A - B` (negate B sign, then add) |
| 3'b010 | CMP | 1 cycle (bypass) | Compare A vs B, set `cmp_lt_o`, `cmp_eq_o` |
| 3'b011 | MIN | 1 cycle (bypass) | `min(A, B)` |
| 3'b100 | MAX | 1 cycle (bypass) | `max(A, B)` |
| 3'b101 | ABS | 1 cycle (bypass) | `|A|` (clear sign bit) |
| 3'b110 | NEG | 1 cycle (bypass) | `-A` (flip sign bit) |
| 3'b111 | SEL | 1 cycle (bypass) | Select A if `cmp_lt_o` from previous compare, else B |

### Timing contract
- **ADD/SUB latency:** 3 clock cycles.
- **CMP/MIN/MAX/ABS/NEG/SEL latency:** 1 cycle (combinational, registered output via bypass port).
- **Throughput:** 1 operation per cycle for all operations.

## 3. Pipeline Stages (ADD/SUB only)

### Stage 1 (cycle N → N+1): Alignment

1. **Unpack** both operands (same as multiplier: sign, exponent, mantissa with implicit 1, subnormal handling).

2. **For SUB:** Flip `sign_b`.

3. **Effective operation:** If `sign_a == sign_b` → effective add. If `sign_a != sign_b` → effective subtract.

4. **Exponent and Magnitude difference:** `exp_diff = exp_a - exp_b`.
   - If `exp_diff > 0` or (`exp_diff == 0` and `mant_a >= mant_b`): A is the larger magnitude. Right-shift B's mantissa by `exp_diff` positions. Result sign is `sign_a`.
   - If `exp_diff < 0` or (`exp_diff == 0` and `mant_a < mant_b`): B is the larger magnitude. Right-shift A's mantissa by `|exp_diff|` positions. Swap so A is always the larger. Result sign is `sign_b`.
   - **Maximum shift:** 26 positions (24 mantissa + guard + round). Beyond that, everything is sticky.

5. **Register:** aligned mantissas (26 bits each: 24 mantissa + guard + round), sticky bit, effective operation, result sign, result exponent, special-case flags.

### Stage 2 (cycle N+1 → N+2): Addition + Leading Zero Count

1. **Mantissa operation:**
   - Effective add: `mant_sum = mant_a + mant_b` (27-bit result, may have carry-out).
   - Effective subtract: `mant_diff = mant_a - mant_b` (26-bit result, may be very small).

2. **Leading zero count (LZC)** on the result:
   - For effective add with carry-out: LZC = 0 (will normalize by right-shifting 1).
   - For effective subtract: LZC may be 0 to 25. This is the costly operation. Use a priority encoder or a dedicated LZC circuit.

3. **Register:** mantissa result, LZC value, metadata.

### Stage 3 (cycle N+2 → N+3): Normalize + Round

1. **Normalize:**
   - Effective add with carry-out: right-shift mantissa by 1, increment exponent.
   - Effective subtract with leading zeros: left-shift mantissa by LZC, decrement exponent by LZC.
   - If exponent goes to 0 or below: result is subnormal. Clamp exponent to 0, shift mantissa accordingly.

2. **Round (RNE):** Same logic as the multiplier (guard, round, sticky → RNE decision).

3. **Pack** result: `{sign_r, exp_r[7:0], mant_r[22:0]}`.

4. **Special cases:**

| A | B | Eff. Op | Result |
|---|---|---|---|
| sNaN | any | any | NaN (0x7FC00000) | invalid |
| any | sNaN | any | NaN (0x7FC00000) | invalid |
| qNaN | any | any | NaN (0x7FC00000) | — |
| any | qNaN | any | NaN (0x7FC00000) | — |
| +Inf | +Inf | add | +Inf | — |
| +Inf | -Inf | add | NaN (0x7FC00000)| invalid |
| -Inf | +Inf | add | NaN (0x7FC00000)| invalid |
| +Inf | finite | add | +Inf | — |
| x | x | sub | +0 | — |
| ±0 | ±0 | sub, signs differ | +0 (RNE convention) | — |

## 4. Bypass Operations (Single-Cycle)

These do NOT go through the 3-stage pipeline. They are combinational with a single output register.

### CMP (Compare)
- Handle NaN: if either input is NaN, both `cmp_lt_o` and `cmp_eq_o` are 0 (unordered).
- Handle ±0: `+0 == -0`.
- Compare signs first, then exponents, then mantissas.
- `cmp_lt_o = 1` if A < B.
- `cmp_eq_o = 1` if A == B.

### MIN/MAX
- Use the comparator. Return the appropriate operand.
- If either is NaN, return the non-NaN operand (IEEE-754 2008 minNum/maxNum behavior).

### ABS
- `result = {1'b0, a_i[30:0]}`.

### NEG
- `result = {~a_i[31], a_i[30:0]}`.

### SEL (Conditional Select)
- The sequencer uses this after a CMP to implement conditional moves.
- `result = cmp_lt_o ? a_i : b_i` (or as configured by the microinstruction).

## 5. Subnormal Handling

Same as the multiplier: subnormals are supported as inputs and outputs. No flush-to-zero.

**TEST REQUIRED:** Test: subnormal + normal, subnormal - subnormal, large_normal + subnormal (catastrophic cancellation), subnormal result from normal subtraction.

## 6. Verification Plan (V1)

| Test | Method | Count | Pass Criterion |
|---|---|---|---|
| Directed special cases (NaN, Inf, zero) | Verilator | ~300 | Bit-exact |
| Catastrophic cancellation (nearly equal) | Verilator | ~500 | Bit-exact |
| Random add | Verilator + TestFloat | 10⁸ | Bit-exact vs SoftFloat |
| Random sub | Verilator + TestFloat | 10⁸ | Bit-exact |
| CMP/MIN/MAX directed | Verilator | ~200 | Correct |
| ABS/NEG | Verilator | ~50 | Trivial |
| Bypass timing | Verilator | ~20 | Result in 1 cycle |

## 7. Implementation Notes

1. **The LZC (leading zero counter)** is the critical path in Stage 2 for effective subtraction. In 180nm at 50 MHz (20 ns period), a 27-bit priority encoder should close easily (~4 gate delays). If it doesn't, consider a two-level LZC.

2. **Catastrophic cancellation** (subtracting two nearly-equal numbers) is the hardest case. The LZC may be up to 25, requiring a massive left-shift and exponent adjustment. Make sure the shifter is a barrel shifter, not a sequential one.

3. **The bypass path** must be independent of the pipeline. The bypass result appears on `bypass_valid_o` / `bypass_result_o` in the same cycle as `valid_i` (registered once). It does NOT conflict with a pipelined ADD/SUB in flight.

4. **SUB implementation:** Don't build a separate subtractor. Just flip `b_i[31]` and feed into the adder.
