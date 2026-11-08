# Design: `tide_fp32_mul.sv` — IEEE-754 FP32 Multiplier

**Spec reference:** §6.3, Decision T5 (No FMA), Decision T7 (shared datapath)
**Gate:** G3 (FP lanes RTL + V1 verification)
**Replaces:** `tide_fp32_fma.sv` (DELETE that file)

---

## 1. Purpose

A pipelined IEEE-754 binary32 multiplier. It is one of the two primary arithmetic lanes in the TIDE-EKF engine. The spec explicitly forbids FMA (fused multiply-add) to preserve bit-exactness with the C golden model compiled with `-ffp-contract=off`. Therefore multiply and add are **separate units with separate rounding**.

## 2. Interface

```systemverilog
module tide_fp32_mul (
    input  logic        clk_i,
    input  logic        rst_ni,

    // Pipeline input (from sequencer or PCPI arbiter)
    input  logic        valid_i,      // New operands available
    input  logic [31:0] a_i,          // Operand A (IEEE-754 binary32)
    input  logic [31:0] b_i,          // Operand B (IEEE-754 binary32)

    // Pipeline output (to register file write port)
    output logic        valid_o,      // Result available (2 cycles after valid_i)
    output logic [31:0] result_o,     // IEEE-754 binary32 result
    output logic [4:0]  flags_o       // {invalid, div_by_zero(0), overflow, underflow, inexact}
);
```

### Timing contract
- **Latency:** 2 clock cycles (result available on cycle N+2 when input accepted on cycle N).
- **Throughput:** 1 operation per cycle (fully pipelined, new inputs every cycle).
- **No backpressure:** The sequencer guarantees it will never issue faster than 1/cycle.

## 3. Pipeline Stages

### Stage 1 (cycle N → N+1): Partial Product Generation + Carry-Save Compression

1. **Unpack** both operands:
   - `sign_a = a_i[31]`, `exp_a = a_i[30:23]`, `mant_a = {1'b1, a_i[22:0]}` (24 bits, implicit leading 1).
   - Same for B.
   - Handle subnormals: if `exp == 0` and `mant != 0`, then `mant = {1'b0, a_i[22:0]}` and `exp = 1`.
   - Handle zero: if `exp == 0` and `mant == 0`, flag as zero.

2. **Result sign:** `sign_r = sign_a ^ sign_b`.

3. **Exponent sum:** `exp_sum = exp_a + exp_b - 127` (biased). Use a 10-bit intermediate to detect overflow/underflow.

4. **Mantissa multiply:** 24×24 → 48-bit product.
   - Use **Booth radix-4** encoding on operand B to generate 13 partial products.
   - Compress partial products using a carry-save adder (CSA) tree (Wallace or Dadda tree) down to two vectors (sum + carry).
   - **Register the two vectors** (sum, carry) and all metadata (sign, exponent, special-case flags) at the end of Stage 1.

### Stage 2 (cycle N+1 → N+2): Carry-Propagate Add + Normalize + Round

1. **Carry-propagate add** the sum and carry vectors to get the full 48-bit product mantissa.

2. **Normalize:**
   - Use a **Leading Zero Counter (LZC)** on the 48-bit product. If either input was subnormal, the leading 1 can be anywhere from bit 0 to 47.
   - Left-shift the product by the LZC count to normalize it (leading 1 at bit 47). Adjust the exponent by subtracting the LZC count (plus 1 if it was a standard 1-bit right shift for overflow).
   - If the adjusted exponent goes $\le 0$, the result is subnormal. Clamp the exponent to 0 and right-shift the mantissa (denormalize) accordingly.

3. **Round (RNE — Round to Nearest, ties to Even):**
   - Guard bit = product[23] (or shifted equivalent).
   - Round bit = product[22].
   - Sticky bit = OR of all bits below the round bit.
   - RNE rule:
     - If guard=0: truncate.
     - If guard=1, (round|sticky)=1: round up.
     - If guard=1, round=0, sticky=0 (exact tie): round to even (round up if LSB of result mantissa is 1).
   - If rounding causes mantissa overflow (0xFFFFFF + 1 = 0x1000000): right-shift, increment exponent.

4. **Pack** result: `{sign_r, exp_r[7:0], mant_r[22:0]}`.

5. **Special cases** (computed in Stage 1, applied in Stage 2):
   - NaN $\times$ anything = `0x7FC00000` (canonical quiet NaN). `invalid` flag set ONLY if the input was a signaling NaN (sNaN). Quiet NaNs propagate without setting flags.
   - $\pm$Inf $\times \pm$0 = NaN (invalid flag set).
   - ±Inf × non-zero finite = ±Inf.
   - ±0 × non-Inf = ±0.
   - Overflow → ±Inf, set overflow + inexact flags.
   - Underflow → subnormal or ±0, set underflow + inexact flags.

## 4. Special Case Truth Table

| A | B | Result | Flags |
|---|---|---|---|
| sNaN | any | NaN (0x7FC00000) | invalid |
| any | sNaN | NaN (0x7FC00000) | invalid |
| qNaN | any | NaN (0x7FC00000) | — |
| any | qNaN | NaN (0x7FC00000) | — |
| ±Inf | ±0 | NaN | invalid |
| ±0 | ±Inf | NaN | invalid |
| ±Inf | finite≠0 | ±Inf (sign = sa^sb) | — |
| finite≠0 | ±Inf | ±Inf (sign = sa^sb) | — |
| ±Inf | ±Inf | ±Inf (sign = sa^sb) | — |
| ±0 | finite | ±0 (sign = sa^sb) | — |
| finite | ±0 | ±0 (sign = sa^sb) | — |
| normal | normal | normal product | inexact if rounded |
| subnormal | normal | may underflow | underflow, inexact |

## 5. Subnormal Handling

The spec says "subnormals supported" (§6.3). This means:
- Subnormal inputs are valid and must be handled (not flushed to zero).
- Subnormal results are valid and must be produced when the true result is in the subnormal range.
- The mantissa for subnormals does NOT have an implicit leading 1.

**TEST REQUIRED:** Specifically test: subnormal × normal, subnormal × subnormal, normal producing subnormal result.

## 6. Area Estimate

The spec estimates MUL + ADD combined at 0.3–0.6 mm² in SCL 180nm. A 24×24 Booth radix-4 multiplier with CSA tree is approximately 15-20k gates. The pipeline registers and the Stage 2 LZC + barrel shifter add roughly 4-5k gates. The LZC may impact the Stage 2 critical path; ensure synthesis targets meet 50 MHz.

## 7. Verification Plan (V1)

| Test | Method | Count | Pass Criterion |
|---|---|---|---|
| Directed special cases | Verilator | ~200 | Bit-exact |
| Boundary: exp overflow/underflow | Verilator | ~100 | Correct saturation |
| Random normal × normal | Verilator + TestFloat | 10⁸ | Bit-exact vs Berkeley SoftFloat |
| Random with subnormals | Verilator + TestFloat | 10⁷ | Bit-exact |
| NaN propagation | Verilator | ~50 | Always 0x7FC00000 |

## 8. Implementation Notes for the Coder

1. **Do NOT use the `*` operator** for the mantissa multiply in synthesis-targeted code. Use explicit Booth encoding and a CSA tree. The `*` operator may or may not infer a structure that meets timing in 180nm at 50 MHz. If you choose to use `*` as a first pass, add a comment `// SYNTHESIS: verify timing` and flag it for review.

2. **Pipeline register naming:** Use `_s1_` suffix for signals registered at the Stage 1/2 boundary.

3. **The valid pipeline:** `valid_i` propagates through a 2-stage shift register → `valid_o`. No stalling.

4. **Import `tide_pkg`** for any shared type definitions, but this module should have NO dependencies on the sequencer, temporal controller, or any other TIDE module. It is a pure arithmetic unit.

5. **Reset behavior:** On reset, `valid_o = 0`, `result_o = 0`, `flags_o = 0`. Pipeline registers can be undefined (don't waste area resetting them).
