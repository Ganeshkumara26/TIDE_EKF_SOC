# Design: `tide_fp32_divsqrt.sv` — IEEE-754 FP32 Divide / Square Root

**Spec reference:** §6.3 ("Iterative radix-2 digit recurrence, exact RNE. Divide ≈ 28 cycles, sqrt ≈ 27 cycles, not pipelined; one operation at a time; latency is fixed so the assembler can schedule around it.")
**Gate:** G3
**Replaces:** Current `tide_fp32_divsqrt.sv` which uses Newton-Raphson (WRONG ALGORITHM)

---

## 1. Purpose

An iterative FP32 divide and square root unit using **radix-2 restoring digit recurrence**. The spec demands fixed, deterministic latency so the static microcode scheduler knows exactly when the result is available.

**Why not Newton-Raphson:** NR uses the shared MUL/ADD lanes (which would create scheduling conflicts) and has variable convergence for edge cases. Digit recurrence is self-contained, uses only shift-and-subtract, and has guaranteed fixed latency.

## 2. Interface

```systemverilog
module tide_fp32_divsqrt (
    input  logic        clk_i,
    input  logic        rst_ni,

    // Command interface
    input  logic        start_i,      // Pulse: begin operation
    input  logic [31:0] a_i,          // Dividend (div) or Radicand (sqrt)
    input  logic [31:0] b_i,          // Divisor (div only; ignored for sqrt)
    input  logic        op_i,         // 0 = divide, 1 = sqrt

    // Result interface
    output logic        busy_o,       // High while computing
    output logic        done_o,       // Pulse: result ready
    output logic [31:0] result_o,     // IEEE-754 binary32 result
    output logic [4:0]  flags_o       // {invalid, div_by_zero, overflow, underflow, inexact}
);
```

### Timing contract
- **Divide latency:** Exactly 28 cycles from `start_i` to `done_o`.
- **Sqrt latency:** Exactly 27 cycles from `start_i` to `done_o`.
- **Not pipelined:** `start_i` is ignored while `busy_o` is high.
- **`done_o`** is a single-cycle pulse.

### Why these exact cycle counts
- 24 mantissa bits + 2 guard bits + 1 round bit = 27 quotient/root bits needed.
- Divide: 1 cycle setup + 26 recurrence iterations + 1 cycle finalize = 28.
- Sqrt: 1 cycle setup + 25 recurrence iterations + 1 cycle finalize = 27. (Sqrt converges 1 iteration faster because the first bit is always 1 for normalized inputs.)

## 3. Subnormal Handling

Subnormals must be supported for both inputs and outputs:
- **Inputs:** In the setup cycle, use an LZC to pre-normalize any subnormal inputs (shift left until leading 1, decrement exponent).
- **Outputs:** In the finalize cycle, if the computed exponent is $\le 0$, denormalize the result (shift right, clamp exponent to 0) before rounding.

## 4. Algorithm: Radix-2 Restoring Division

### Setup (cycle 1)
1. Unpack A and B.
2. Result sign = `sign_a ^ sign_b`.
3. Result exponent = `exp_a - exp_b + 127` (bias correction).
4. If `mant_a >= mant_b`: quotient will have leading 1 in bit position 0. Set initial partial remainder `R = mant_a - mant_b`, first quotient bit = 1.
5. If `mant_a < mant_b`: quotient will have leading 1 in bit position -1. Adjust exponent by -1. Set `R = mant_a`, first quotient bit = 0.

### Recurrence (cycles 2 through N-1)
Each cycle produces one quotient bit:
1. `R = 2 * R` (left-shift by 1).
2. If `R >= mant_b`: quotient bit = 1, `R = R - mant_b`.
3. If `R < mant_b`: quotient bit = 0, `R` unchanged.
4. Shift quotient bit into the quotient register.

This is the **restoring division** variant. It is the simplest to implement.

### Finalize (last cycle)
1. The last recurrence bit is the round bit.
2. Sticky bit = `(R != 0)` after the final iteration.
3. Apply RNE rounding.
4. Normalize if needed (the initial comparison guarantees at most 1 bit of normalization).
5. Pack result.

## 5. Algorithm: Radix-2 Square Root

### Setup (cycle 1)
1. Unpack A.
2. If A is negative (and not -0): result = NaN, set invalid flag, done immediately.
3. Result sign = 0 (sqrt is always non-negative).
4. Result exponent = `(exp_a - 127) / 2 + 127`. If `exp_a` is odd, pre-shift mantissa left by 1 to make the effective exponent even.
5. Initialize: partial remainder `R = mant_a` (possibly shifted), root accumulator `Q = 0`.

### Recurrence (cycles 2 through N-1)
Each cycle produces one root bit:
1. Form the test value: `T = (2*Q + 1) << (remaining_bits)` (the trial subtrahend).
   - More precisely: `trial = 2*R - (2*Q_shifted + bit_value)`.
2. If `trial >= 0`: root bit = 1, `R = trial`, `Q = Q with new bit set`.
3. If `trial < 0`: root bit = 0, `R` unchanged, `Q` unchanged.

### Finalize (last cycle)
1. Round bit and sticky bit from the final remainder.
2. Apply RNE rounding.
3. Pack result.

## 6. Special Cases

### Division

| A | B | Result | Flags |
|---|---|---|---|
| sNaN | any | NaN (0x7FC00000) | invalid |
| any | sNaN | NaN (0x7FC00000) | invalid |
| qNaN | any | NaN (0x7FC00000) | — |
| any | qNaN | NaN (0x7FC00000) | — |
| ±Inf | ±Inf | NaN | invalid |
| ±0 | ±0 | NaN | invalid |
| finite | ±0 | ±Inf | div_by_zero |
| ±0 | finite≠0 | ±0 | — |
| ±Inf | finite | ±Inf | — |
| finite | ±Inf | ±0 | — |

### Square Root

| A | Result | Flags |
|---|---|---|
| sNaN | NaN | invalid |
| qNaN | NaN | — |
| negative (≠ -0) | NaN | invalid |
| -0 | -0 | — |
| +0 | +0 | — |
| +Inf | +Inf | — |

## 7. Implementation Notes

1. **This unit does NOT use the MUL or ADD lanes.** It is entirely self-contained with its own shift-and-subtract datapath. This is critical — the spec says the DIV/SQRT unit "sits on the dependency chain" and the microcode scheduler plans around its fixed latency.

2. **The partial remainder `R` and divisor `B`** need to be at least 26 bits wide (24 mantissa + guard + round). Use 28-bit or 32-bit internal datapaths for safety.

3. **The cycle count must be EXACTLY 28 for divide and 27 for sqrt FOR ALL CASES.** The microcode assembler hardcodes these values. If your implementation takes fewer or more cycles, the entire microcode schedule breaks.

4. **Restoring:** Use restoring recurrence to avoid correction steps.

5. **Reset:** On reset, `busy_o = 0`, `done_o = 0`. Internal state can be undefined.

6. **Special cases** should be detected in the setup cycle. **CRITICAL:** For special cases, `done_o` MUST still assert at the exact specified cycle count (28 or 27) to maintain the fixed-latency contract. Do NOT short-circuit early.

## 8. Verification Plan (V1)

| Test | Method | Count | Pass Criterion |
|---|---|---|---|
| Directed special cases | Verilator | ~200 | Bit-exact |
| Division: exact results | Verilator | ~100 | No inexact flag |
| Division: rounding boundaries | Verilator | ~500 | Correct RNE |
| Division: random | Verilator + TestFloat | 10⁸ | Bit-exact vs SoftFloat |
| Sqrt: perfect squares | Verilator | ~50 | Exact |
| Sqrt: random | Verilator + TestFloat | 10⁸ | Bit-exact |
| Cycle count verification | Verilator | every test | Exactly 28 (div) or 27 (sqrt) |
| Back-to-back operations | Verilator | ~20 | No state leakage |

**TEST REQUIRED:** Verify that the cycle count is EXACTLY 28/27 for EVERY test case, including special cases. This is a hard architectural contract.
