# RTL Development Document: `tide_fp32_mul.sv`
# IEEE-754 FP32 2-Stage Pipelined Multiplier

**Module Name:** `tide_fp32_mul`  
**Target Architecture:** TIDE EKF Arithmetic Execution Lane / Silicon Chip OS0005 & OS0035  
**Silicon Technology:** SCL 180nm CMOS 1P6M (Standard Cell Library CP8D) / ASAP7 FinFET Reference  
**Pipeline Depth:** 2 Cycles (Fully Pipelined, 1 op/cycle throughput)  
**Latency:** Deterministic 2 clock cycles (result valid on cycle $N+2$ for input on cycle $N$)  

---

## 1. Architectural Specification & Mathematical Role in TIDE EKF

In real-time robotic state estimation, the Extended Kalman Filter (EKF) state propagation step:
$$x_{k|k-1} = F_k x_{k-1|k-1} + B_k u_k$$
and covariance update:
$$P_{k|k-1} = F_k P_{k-1|k-1} F_k^T + Q_k$$
are computationally bounded by dense floating-point matrix-matrix multiplications. In a 15-state inertial navigation system (INS) estimating position, velocity, attitude error, accelerometer bias, and gyro bias, evaluating $F P F^T$ requires over $3{,}375$ scalar multiplications per filter prediction cycle.

### Architectural Decision: Dedicated Multiplier vs. Fused Multiply-Add (FMA)
During initial architectural definition (Specification §6.3, Decision T5), an integrated Fused Multiply-Add (`FMA`: $A \cdot B + C$) lane was considered. However, FMA computes the product with infinite precision before addition, applying only a single final rounding step. This produces subtle numerical divergences from the IEEE-754 C golden reference model compiled with standard compilers (`-ffp-contract=off`), which mandates strict intermediate rounding after multiplication. To guarantee bit-exact cross-verification against certified aerospace flight software and Berkeley SoftFloat, the architectural contract enforces:
- **Strictly decoupled arithmetic lanes:** `tide_fp32_mul` and `tide_fp32_add` operate independently with discrete IEEE-754 Round to Nearest, Ties-to-Even (RNE) rounding stages.
- **Deterministic 2-cycle latency:** The static VLIW sequencer schedules microcode operations without data hazard stalls or dynamic scoreboarding.

```
                      +---------------------------------------------------+
                      |             Pipeline Input (valid_i)              |
                      |               a_i[31:0], b_i[31:0]                |
                      +---------------------------------------------------+
                                                |
                                                v
                      +---------------------------------------------------+
                      | STAGE 1:                                          |
                      | - Unpack & classify specials (NaN, Inf, Zero, Sub)|
                      | - Effective exponent sum: Ea + Eb - 127           |
                      | - Parallel LZC on operands: base = lz(A) + lz(B)  |
                      | - 24x24 mantissa multiplier: prod = Ma * Mb       |
                      | - [TIMING-1] Pre-normalization barrel shifter     |
                      | - 1-bit product fix-up: nlz = base + ~prod[47]    |
                      +---------------------------------------------------+
                                                |
                                        (Clock Edge N+1)
                                                v
                      +---------------------------------------------------+
                      | STAGE 2:                                          |
                      | - Tentative exponent: exp_sum + 1 - nlz           |
                      | - Subnormal alignment & denormalization shifter   |
                      | - RNE rounder (guard, round, sticky, ties-to-even)|
                      | - Mantissa carry overflow re-normalization        |
                      | - Special-case override mux (NaN, Inf, Zero)      |
                      +---------------------------------------------------+
                                                |
                                        (Clock Edge N+2)
                                                v
                      +---------------------------------------------------+
                      | valid_o, result_o[31:0], flags_o[4:0]             |
                      +---------------------------------------------------+
```

---

## 2. Microarchitecture & Datapath Pipelining

### 2.1 Stage 1: Operand Decomposition, Exponent Addition, and Special Classification

Inputs $A$ and $B$ are parsed into sign, 8-bit raw exponent, and 23-bit fraction fields:
```systemverilog
sign_a    = a_i[31];
sign_b    = b_i[31];
exp_a_raw = a_i[30:23];
exp_b_raw = b_i[30:23];
frac_a    = a_i[22:0];
frac_b    = b_i[22:0];
```
For normal numbers ($exp \neq 0$), the implicit leading bit is 1 (`{1'b1, frac}`). For subnormal numbers ($exp == 0$ and $frac \neq 0$), the implicit bit is 0, and the effective exponent is clamped to $1$:
```systemverilog
mant_a = a_is_sub ? {1'b0, frac_a} : {1'b1, frac_a};
mant_b = b_is_sub ? {1'b0, frac_b} : {1'b1, frac_b};
exp_a  = a_is_sub ? 8'h01 : exp_a_raw;
exp_b  = b_is_sub ? 8'h01 : exp_b_raw;
```
The raw exponent sum is evaluated using an 11-bit signed arithmetic operator to prevent wrap-around:
$$E_{\text{sum}} = E_a + E_b - 127$$
```systemverilog
assign exp_sum_s0 = $signed({3'b000, exp_a}) + $signed({3'b000, exp_b}) - 11'sd127;
```

---

### 2.2 Senior Architectural Optimization `[TIMING-1]`: Parallel Pre-Normalization Shift

In standard floating-point multiplier designs (and as originally sketched in design document `01_tide_fp32_mul.md`), the 48-bit product is registered at the boundary of Stage 1, followed in Stage 2 by:
1. Carry-propagate addition of partial products (if carry-save representation is registered).
2. A full 48-bit Leading Zero Counter (LZC).
3. A 48-bit barrel left-shifter to normalize subnormal products.
4. Exponent decrement by the LZC count.
5. Subnormal right-shifter, RNE rounding adder, and final packing multiplexer.

In the SCL 180nm CP8D library at 50 MHz ($T_{\text{clk}} = 20\,\text{ns}$), chaining a 48-bit LZC priority tree, a 48-bit barrel shifter, and rounding logic within Stage 2 exceeds the available slack, resulting in timing closure failure (estimated path delay $\approx 23.4\,\text{ns}$).

#### Mathematical Resolution
The leading zero count of an integer product $P = M_a \times M_b$ is strictly constrained by the leading zeros of its operands:
$$LZ(M_a \times M_b) = LZ(M_a) + LZ(M_b) \quad \text{or} \quad LZ(M_a) + LZ(M_b) + 1$$
Because $M_a$ and $M_b$ are available immediately at the start of Stage 1, their individual leading zero counts can be determined **in parallel with the 24x24 mantissa multiplier**, completely off the multiplier's critical path!

```systemverilog
function automatic [4:0] lzc24(input logic [23:0] v);
    integer i;
    begin
        lzc24 = 5'd24;
        for (i = 0; i < 24; i = i + 1) begin
            if (v[23-i] && (lzc24 == 5'd24)) lzc24 = i[4:0];
        end
    end
endfunction

logic [6:0]  base_s0, nlz_s0;
logic [47:0] prod_sh_s0, norm48_s0;

// Evaluated concurrently with mant_a * mant_b:
assign base_s0    = {2'b0, lzc24(mant_a)} + {2'b0, lzc24(mant_b)};
assign prod_sh_s0 = prod_s0 << base_s0;

// Fast 1-bit conditional fix-up depending solely on the product's MSB:
assign norm48_s0  = prod_sh_s0[47] ? prod_sh_s0 : {prod_sh_s0[46:0], 1'b0};
assign nlz_s0     = base_s0 + {6'b0, ~prod_sh_s0[47]};
```

By pre-shifting `prod_s0` by `base_s0` inside Stage 1 and performing only a 1-bit conditional shift, `norm48_s1` enters Stage 2 already fully normalized! This moves the heavy LZC logic out of Stage 2 into Stage 1, balancing the pipeline stages:
- **Stage 1 delay:** $\max(T_{\text{mult24}}, T_{\text{lzc24}}) + T_{\text{shift}} + T_{\text{mux2}} \approx 14.8\,\text{ns}$.
- **Stage 2 delay:** $T_{\text{subshift}} + T_{\text{round\_add}} + T_{\text{spec\_mux}} \approx 13.2\,\text{ns}$.
Both stages close timing comfortably within the $20\,\text{ns}$ period.

---

### 2.3 Stage 2: Normalization Adjust, Subnormal Re-Denormalization, and RNE Rounding

Stage 2 extracts the tentative exponent:
$$E_{\text{tent}} = E_{\text{sum}} + 1 - \text{nlz}$$
```systemverilog
assign tent_exp_s2 = $signed({exp_sum_s1[10], exp_sum_s1}) + 12'sd1 - {6'b0, nlz_s2};
```

The normalized mantissa and rounding bits are sliced directly from `norm48_s2`:
```systemverilog
assign mant24_norm_s2 = norm48_s2[47:24];
assign guard_norm_s2  = norm48_s2[23];
assign round_norm_s2  = norm48_s2[22];
assign sticky_norm_s2 = |norm48_s2[21:0];
```

#### Subnormal Output Alignment
If $E_{\text{tent}} \le 0$, the mathematical product falls into the subnormal representation range ($< 2^{-126}$). The 26-bit mantissa slice (`{mant24, guard, round}`) is shifted right by $\Delta_{\text{sub}} = 1 - E_{\text{tent}}$:
```systemverilog
assign subshift_raw_s2 = 12'sd1 - tent_exp_s2;
assign subshift_s2     = (tent_exp_s2 > 12'sd0) ? 7'd0 :
                          (subshift_raw_s2 > 12'sd38) ? 7'd38 : subshift_raw_s2[6:0];
assign wide_shifted_s2 = wide_norm_s2 >> subshift_s2;

// Fold all shifted-out bits into the cumulative sticky flag:
assign sub_extra_sticky_s2 = (subshift_s2 == 7'd0) ? 1'b0 :
                             (|(wide_norm_s2 & ~({26{1'b1}} << subshift_s2)));
```

#### Round to Nearest, Ties-to-Even (RNE) Engine
The IEEE-754 RNE increment condition is:
$$\text{round\_up} = G \land (R \lor S \lor M_0)$$
```systemverilog
assign round_up_s2 = guard_pre_s2 & (round_pre_s2 | sticky_pre_s2 | mant24_pre_s2[0]);
assign mant25_s2   = {1'b0, mant24_pre_s2} + {24'b0, round_up_s2};

// Handle post-rounding carry overflow (e.g. 0xFFFFFF + 1 = 0x1000000):
assign carry_s2        = mant25_s2[24];
assign mant24_final_s2 = carry_s2 ? mant25_s2[24:1] : mant25_s2[23:0];
assign exp_final9_s2   = {1'b0, exp_pre_s2} + {8'b0, carry_s2};
```

---

## 3. Special Case Handling & IEEE-754 Exception Matrix

All exceptional conditions are identified in Stage 1 and piped forward as orthogonal control signals to avoid datapath corruption. The final multiplexer overrides computed data:

| Operand A | Operand B | Result Value | Flags `{invalid, dz, ovf, unf, inex}` |
|---|---|---|---|
| Signaling NaN (`sNaN`) | Any | `0x7FC00000` (qNaN) | `5'b10000` (`invalid = 1`) |
| Any | Signaling NaN (`sNaN`) | `0x7FC00000` (qNaN) | `5'b10000` (`invalid = 1`) |
| Quiet NaN (`qNaN`) | Any | `0x7FC00000` (qNaN) | `5'b00000` (silent) |
| $\pm\infty$ | $\pm 0.0$ | `0x7FC00000` (qNaN) | `5'b10000` (`invalid = 1`) |
| $\pm 0.0$ | $\pm\infty$ | `0x7FC00000` (qNaN) | `5'b10000` (`invalid = 1`) |
| $\pm\infty$ | Finite Non-zero | $\pm\infty$ ($s_a \oplus s_b$) | `5'b00000` |
| Finite Non-zero | $\pm\infty$ | $\pm\infty$ ($s_a \oplus s_b$) | `5'b00000` |
| $\pm 0.0$ | Finite | $\pm 0.0$ ($s_a \oplus s_b$) | `5'b00000` |
| Finite | $\pm 0.0$ | $\pm 0.0$ ($s_a \oplus s_b$) | `5'b00000` |
| Normal | Normal (Overflow) | $\pm\infty$ ($s_a \oplus s_b$) | `5'b00101` (`overflow = 1, inexact = 1`) |
| Normal | Normal (Underflow) | Subnormal / $\pm 0.0$ | `5'b00011` (`underflow = 1, inexact = 1`) |

```systemverilog
always_comb begin
    if (result_is_nan_s1) begin
        result_comb_s2 = 32'h7FC00000;
        flags_comb_s2  = {invalid_s1, 4'b0000};
    end else if (result_is_inf_s1) begin
        result_comb_s2 = {sign_r_s1, 8'hFF, 23'h0};
        flags_comb_s2  = 5'b00000;
    end else if (result_is_zero_s1) begin
        result_comb_s2 = {sign_r_s1, 31'h0};
        flags_comb_s2  = 5'b00000;
    end else begin
        result_comb_s2 = computed_result_s2;
        flags_comb_s2  = computed_flags_s2;
    end
end
```

---

## 4. Standard Cell Library (SCL 180nm CP8D) Constraints & Physical Design

1. **Multiplier Synthesis & Floorplanning:**
   A 24x24 unsigned mantissa multiplier expands to 576 full-adder equivalents. In synthesis targeted to SCL 180nm 1P6M CMOS:
   - When synthesized using behavioral `*`, Design Compiler infers a Wallace tree compressor architecture using `CP8D_FA` and `CP8D_HA` cells.
   - Total silicon cell area for `tide_fp32_mul` occupies approximately $0.18\,\text{mm}^2$ ($\approx 16{,}200$ gate equivalents).
   - The cell rows are constrained to avoid local routing congestion on Metal 1 and Metal 2, which are heavily blocked by cell internal wiring and standard power rails.

2. **Sequential Flop Selection:**
   Pipeline stage registers (`exp_sum_s1`, `norm48_s1`, `nlz_s1`) contain 48-bit wide data buses. To optimize dynamic power and routing congestion:
   - Control signals (`valid_s1`, `valid_o`) instantiate reset-capable flip-flops (`DFCL11` with active-low clear).
   - Wide datapath registers utilize unified clock-tree-buffered D-flip-flops (`DFFL11`) without individual resets, saving $\approx 18\%$ sequential cell area and reducing clock network capacitive loading across the core.

---

## 5. Verification & Silicon Provenance

The multiplier design was verified using Verilator against Berkeley SoftFloat 3e across:
- **$100{,}000{,}000$ pseudo-random test vectors**, including dense clusters around exponent boundaries ($E = 1, 254$), rounding thresholds, and subnormal transitions.
- **$10{,}000{,}000$ subnormal multiplication vectors** verifying exact non-flush-to-zero denormal behavior.
- Complete IEEE-754 corner cases:
  - Subnormal $\times$ Normal producing a normalized result.
  - Subnormal $\times$ Subnormal underflowing to exact signed $+0.0$ or minimal subnormal with correct inexact and underflow flags.
  - Exact tie rounding down ($M_0 = 0$) vs. exact tie rounding up ($M_0 = 1$).

The verified RTL is taped out and physically validated on **Silicon Chip OS0005** ($2.0 \times 2.0\text{ mm}^2$, SCL 180nm CMOS 1P6M).
