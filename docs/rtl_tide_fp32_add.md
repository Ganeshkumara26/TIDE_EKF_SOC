# RTL Development Document: `tide_fp32_add.sv`
# IEEE-754 FP32 Pipelined Adder / Subtractor & Single-Cycle Bypass Lane

**Module Name:** `tide_fp32_add`  
**Target Architecture:** TIDE EKF Arithmetic Execution Lane / Silicon Chip OS0005 & OS0035  
**Silicon Technology:** SCL 180nm CMOS 1P6M (Standard Cell Library CP8D) / ASAP7 FinFET Reference  
**Pipeline Depth:** 3 Cycles (ADD, SUB) / 1 Cycle Combinational Registered (FCMP, FMIN, FMAX, FABS, FNEG, FMOV)  
**Throughput:** 1 operation / cycle non-blocking  

---

## 1. Architectural Specification & Mathematical Role in TIDE EKF

The Extended Kalman Filter (EKF) and Out-of-Sequence Measurement (OOSM) reconciliation engine deployed on the TIDE coprocessor execute repetitive matrix-vector operations:
$$x_{k|k-1} = F_k x_{k-1|k-1} + G_k u_k$$
$$P_{k|k-1} = F_k P_{k-1|k-1} F_k^T + Q_k$$
$$S_k = H_k P_{k|k-1} H_k^T + R_k$$
$$K_k = P_{k|k-1} H_k^T S_k^{-1}$$

In high-rate state estimation, the innovation covariance evaluation ($S_k$) and residual updates ($\tilde{y}_k = z_k - H_k \hat{x}_{k|k-1}$) are dominated by floating-point additions and subtractions. Furthermore, catastrophic cancellation—where two nearly identical numbers are subtracted, yielding large loss of significance—is frequent when computing sensor innovations or diagonal covariance elements ($P_{ii} - K_i H_i P_{ii}$).

To satisfy IEEE-754 binary32 compliance without pipeline stalls, `tide_fp32_add` is partitioned into two distinct paths:
1. **A 3-stage pipelined datapath** for floating-point addition and subtraction (`FADD`, `FSUB`), handling operand alignment, mantissa addition/subtraction, leading zero detection, full normalization, subnormal numbers, and Round to Nearest, Ties-to-Even (RNE).
2. **A parallel single-cycle bypass execution path** for floating-point comparisons (`FCMP`), extrema selection (`FMIN`, `FMAX`), sign manipulation (`FABS`, `FNEG`), and operand routing (`FMOV`), eliminating pipeline bubble penalties during state gating, quaternion sign-flipping, and threshold clipping.

```
                  +-------------------------------------------------------------+
                  |                     Pipeline Input (valid_i)                |
                  |                a_i[31:0], b_i[31:0], op_i[2:0]              |
                  +-------------------------------------------------------------+
                                     |                                   |
                [is_pipelined_op]    v                                   v  [!is_pipelined_op]
             +-------------------------------+                 +--------------------+
             | STAGE 1: Unpack, Sign Logic,  |                 | Bypass Comparator  |
             | Magnitude Swap, Align Shifter |                 | & Special Unary    |
             | (GRS Bit-0 Borrow Folding)    |                 | (FCMP/FMIN/FMAX/   |
             +-------------------------------+                 |  FABS/FNEG/FMOV)   |
                             | (Clock Edge N+1)                +--------------------+
                             v                                           | (Clock Edge N+1)
             +-------------------------------+                           v
             | STAGE 2: 28-bit Add/Subtract, |                 +--------------------+
             | Unified 28-bit LZC Detection  |                 | bypass_valid_o     |
             +-------------------------------+                 | bypass_result_o    |
                             | (Clock Edge N+2)                | cmp_lt_o, cmp_eq_o |
                             v                                 +--------------------+
             +-------------------------------+
             | STAGE 3: Normalizer/Subnormal |
             | Shifter, RNE Rounder, Pack    |
             +-------------------------------+
                             | (Clock Edge N+3)
                             v
             +-------------------------------+
             | valid_o, result_o, flags_o    |
             +-------------------------------+
```

---

## 2. Microarchitecture & Pipelined Datapath Translation

### 2.1 Stage 1: Operand Decomposition, Alignment, and Borrow Preservation (A-ADD-1)

Stage 1 unpacks the IEEE-754 operands $A = (-1)^{s_a} \cdot 2^{e_a - 127} \cdot (1.f_a)$ and $B = (-1)^{s_b} \cdot 2^{e_b - 127} \cdot (1.f_b)$. For subnormal numbers ($e = 0, f \neq 0$), the implicit leading bit is forced to $0$ and the effective exponent is biased to $e = 1$.

```systemverilog
assign sign_b_eff = (op_i == OP_SUB) ? ~b_i[31] : b_i[31];
assign eff_add_s0 = (sign_a == sign_b_eff);

always_comb begin
    mant_a    = a_is_zero ? 24'h0 : (a_is_sub ? {1'b0, frac_a} : {1'b1, frac_a});
    mant_b    = b_is_zero ? 24'h0 : (b_is_sub ? {1'b0, frac_b} : {1'b1, frac_b});
    exp_a_eff = (a_is_zero || a_is_sub) ? 8'h01 : exp_a_raw;
    exp_b_eff = (b_is_zero || b_is_sub) ? 8'h01 : exp_b_raw;
end
```

The operands are sorted by magnitude:
$$A \ge B \iff (e_a > e_b) \lor ((e_a == e_b) \land (m_a \ge m_b))$$
The smaller mantissa is right-shifted by $\Delta e = e_{\text{big}} - e_{\text{small}}$ to align binary points. In classic floating-point implementations, mantissas are extended with Guard ($G$), Round ($R$), and Sticky ($S$) bits.

#### Critical Hardware Insight: GRS Borrow Propagation under Subtraction (Assumption A-ADD-1)
A conventional hardware trap in floating-point subtractors is treating the sticky bit as an external status flag OR'd into the final result after arithmetic execution. When calculating an effective subtract ($A - B$) with $\Delta e > 0$:
$$M_{\text{big}} - (M_{\text{small}} \gg \Delta e)$$
If bits shifted past the round position contain non-zero data, they must generate an arithmetic borrow across the round and guard bits into the low-order mantissa. If the sticky bit is merely OR'd into the post-subtraction result, the borrow never propagates, producing off-by-1 ULP errors in approximately 0.3% of random vectors under cancellation.

In `tide_fp32_add.sv`, both mantissas are extended to 27 bits ($24\text{ bits mantissa} + G + R + S$). The sticky bit $S$ is explicitly folded into bit 0 of the shifted operand:

```systemverilog
logic [26:0] big27_s0, small27_pad_s0;
assign big27_s0       = {mant_big_s0, 3'b000};
assign small27_pad_s0 = {mant_small_s0, 3'b000};

assign small27_shifted_raw_s0 = (algn_shift_s0 == 5'd0) ? small27_pad_s0
                                                        : (small27_pad_s0 >> algn_shift_s0);
assign sticky_forced_s0 = (algn_shift_s0 == 5'd0) ? 1'b0 :
    |(small27_pad_s0 & ~(27'h7FFFFFF << algn_shift_s0));

assign small27_shifted_s0 = {small27_shifted_raw_s0[26:1],
                              small27_shifted_raw_s0[0] | sticky_forced_s0};
```
Because the sticky bit resides inside `small27_shifted_s0[0]`, the two's complement subtractor (`big27_s1 - small27_shifted_s1`) naturally propagates borrows through bit 0, yielding bit-exact alignment before Stage 2.

---

### 2.2 Stage 2: Mantissa Operation & Unified 28-bit Leading Zero Counter (LZC)

The architectural design doc initially suggested two disparate paths:
1. Carry-out path for addition ($\text{LZC} = 0$, requiring a 1-bit right shift).
2. Cancellation path for subtraction (requiring a priority encoder over 26 bits).

However, in silicon, subnormal addition (e.g., $0\text{x}00000001 + 0\text{x}00000001 = 0\text{x}00000002$) also yields results with leading zeros without triggering subtraction. To prevent redundant mux trees and divergent verification paths, `tide_fp32_add.sv` forms a unified 28-bit unsigned vector:

```systemverilog
logic [27:0] valword28_s1;
always_comb begin
    if (eff_add_s1)
        valword28_s1 = {1'b0, big27_s1} + {1'b0, small27_shifted_s1};
    else
        valword28_s1 = {1'b0, big27_s1 - small27_shifted_s1};
end
```

The unified 28-bit priority encoder `lzc28` scans `valword28_s1`:
```systemverilog
function automatic [4:0] lzc28(input logic [27:0] v);
    integer i;
    begin
        lzc28 = 5'd28;
        for (i = 0; i < 28; i = i + 1) begin
            if (v[27-i] && (lzc28 == 5'd28)) lzc28 = i[4:0];
        end
    end
endfunction
```
Synthesized in SCL 180nm CP8D standard cells, this encodes into a balanced 4-level gate tree (NAND2/NOR2 and AOI22 gates) with a path delay under 1.8 ns, easily meeting the 20 ns timing envelope without requiring early LZC tree splitting.

---

### 2.3 Stage 3: Normalization, Subnormal Alignment, and Round to Nearest, Ties-to-Even (RNE)

Stage 3 receives `valword28_s2` and `nlz28_s2`. The tentative exponent is computed as:
$$e_{\text{tent}} = e_{\text{big}} + 1 - \text{nlz28}$$
A barrel left-shifter normalizes `valword28_s2` by `nlz28_s2`:
```systemverilog
assign shifted28_s3 = valword28_s2 << nlz28_s2;
assign mant24_norm_s3 = shifted28_s3[27:4];
assign guard_norm_s3  = shifted28_s3[3];
assign round_norm_s3  = shifted28_s3[2];
assign sticky_norm_s3 = shifted28_s3[1] | shifted28_s3[0];
```

#### Subnormal Re-Denormalization
If $e_{\text{tent}} \le 0$, the number cannot be represented as a normalized binary32 float. The unit enters the subnormal path, shifting the 26-bit mantissa right by $\Delta_{\text{sub}} = 1 - e_{\text{tent}}$:
```systemverilog
assign subshift_raw_s3 = 12'sd1 - tent_exp_s3;
assign subshift_s3     = (tent_exp_s3 > 12'sd0) ? 7'd0 :
                          (subshift_raw_s3 > 12'sd38) ? 7'd38 : subshift_raw_s3[6:0];
assign wide_shifted_s3 = wide_norm_s3 >> subshift_s3;
assign sub_extra_sticky_s3 = (subshift_s3 == 7'd0) ? 1'b0 :
                             (|(wide_norm_s3 & ~({26{1'b1}} << subshift_s3)));
```

#### RNE Rounding Logic
Rounding increments the mantissa if:
$$\text{round\_up} = G \land (R \lor S \lor M_0)$$
where $M_0$ is the least significant bit of the pre-rounded mantissa (`mant24_pre_s3[0]`), implementing exact ties-to-even:
```systemverilog
assign round_up_s3 = guard_pre_s3 & (round_pre_s3 | sticky_pre_s3 | mant24_pre_s3[0]);
assign mant25_s3   = {1'b0, mant24_pre_s3} + {24'b0, round_up_s3};

assign carry_s3        = mant25_s3[24];
assign mant24_final_s3 = carry_s3 ? mant25_s3[24:1] : mant25_s3[23:0];
assign exp_final9_s3   = {1'b0, exp_pre_s3} + {8'b0, carry_s3};
```

---

## 3. Standard Cell Library (SCL 180nm CP8D) Hardware Implementation Constraints

1. **Metal-1 Congestion in the Alignment Shifter:**
   In SCL 180nm CMOS (CP8D library), cell height is $8 \times \text{pitch} = 5.04\,\mu\text{m}$. Standard cells have internal horizontal M1 power rails (VDD/VSS) and intra-cell connections. The Stage 1 alignment shifter requires a 27-bit multi-stage multiplexer tree. Writing wide combinational barrel shifts (`small27_pad_s0 >> algn_shift_s0`) directly creates high pin density on M1/M2. By clamping `algn_shift_s0` strictly to $27$ ($5\text{ bits}$), synthesis infers compact 4-to-1 mux clusters (`MUX4D1`), avoiding track starvation on M2.

2. **Sequential Flop Cell Selection:**
   Across the 3 pipeline register boundaries, high fan-out reset lines can induce significant buffer trees. The design selectively employs `DFCL11` (D-flip-flop with active-low clear) for pipeline control registers (`valid_s1`, `valid_s2`) while datapath registers use non-resetting data storage or unified clock-gated cells where acceptable. For all verified registers in this RTL, asynchronous active-low reset (`negedge rst_ni`) is mapped to `DFFQ_X1` / `DFCL11` cells to ensure predictable initialization during coprocessor context switches.

---

## 4. Single-Cycle Bypass Lane Architecture & Integration Defects

The single-cycle bypass execution path runs completely in parallel with the 3-stage adder pipeline, servicing FCMP, FMIN, FMAX, FABS, FNEG, and FMOV. During full-chip co-simulation and silicon bring-up of the TIDE EKF coprocessor, four critical hardware bugs and contract discrepancies were identified and resolved.

```
                           +----------------------------------------+
                           |  Bypass Path Combinational Logic       |
                           +----------------------------------------+
                             /            |            |           \
                            /             |            |            \
                           v              v            v             v
                    +------------+ +------------+ +------------+ +------------+
                    | Signed-Zero| | Canonical  | | FMOV       | | FCMP       |
                    | Tie-Break  | | NaN Rectif.| | Passthrough| | Numeric    |
                    | (FMIN/MAX) | | (FABS/NEG) | | (isa.yaml) | | Result Gen |
                    +------------+ +------------+ +------------+ +------------+
                           \              |            |            /
                            \             |            |           /
                             v            v            v          v
                           +----------------------------------------+
                           |         bypass_result_comb             |
                           +----------------------------------------+
                                              |
                                      [always_ff @clk]
                                              v
                           +----------------------------------------+
                           |  bypass_valid_o, bypass_result_o       |
                           +----------------------------------------+
```

### 4.1 Bug Fix 1: MAX(-0.0, +0.0) Signed-Zero Tie-Breaking Defect

**Defect Manifestation:** In IEEE-754 arithmetic, $+0.0$ (`0x00000000`) and $-0.0$ (`0x80000000`) compare as numerical equals ($+0.0 == -0.0$). The shared comparator `cmp_lt_comb` evaluates:
```systemverilog
else if (a_zero_c && b_zero_c) begin
    cmp_lt_comb = 1'b0;
    cmp_eq_comb = 1'b1;
end
```
When FMIN/FMAX was initially implemented by reusing `cmp_lt_comb` directly:
```systemverilog
// DEFECTIVE IMPLEMENTATION:
assign min_result = cmp_lt_comb ? a_i : b_i;
assign max_result = cmp_lt_comb ? b_i : a_i;
```
For inputs $A = -0.0$ and $B = +0.0$, `cmp_lt_comb` evaluates to `0`. Consequently, `max_result` defaulted to $A$ ($-0.0$), violating IEEE-754-2008 `maxNum` semantics, which mandate selecting $+0.0$. This defect was caught during PCPI coprocessor cross-verification where the reference model failed vector `MAX(-0.0, +0.0)`.

**RTL Fix:** An explicit signed-zero arbitration block was inserted ahead of magnitude evaluation:
```systemverilog
else if (a_zero_c && b_zero_c) begin
    // Equal magnitude (both zero): sign-aware tie-break, NOT cmp_lt_comb.
    min_result_comb = (a_i[31] || b_i[31]) ? (a_i[31] ? a_i : b_i) : a_i;
    max_result_comb = (!a_i[31] || !b_i[31]) ? (!a_i[31] ? a_i : b_i) : a_i;
end else begin
    min_result_comb = minmax_a_lt_b_comb ? a_i : b_i;
    max_result_comb = minmax_a_lt_b_comb ? b_i : a_i;
end
```

### 4.2 Bug Fix 2: FABS and FNEG Canonical NaN Payload Rectification

**Defect Manifestation:** Earlier design specifications suggested implementing `FABS` as `{1'b0, a_i[30:0]}` and `FNEG` as `{~a_i[31], a_i[30:0]}`. However, if the incoming operand is a signaling NaN or a non-canonical quiet NaN with payload (e.g., `a_i = 0xFFD0FC80`), bit-flipping produced `0x7FD0FC80`. While still a NaN, the top-level architectural specification requires the TIDE coprocessor to enforce canonical Quiet NaN generation (`0x7FC00000` for positive NaN, `0xFFC00000` for negative NaN) to prevent payload leakage from uninitialized memory.

**RTL Fix:** Explicit NaN canonicalization was introduced into the bypass datapath:
```systemverilog
assign abs_result_comb = a_nan_c ? 32'h7FC00000 : {1'b0, a_i[30:0]};
assign neg_result_comb = a_nan_c ? 32'hFFC00000 : {~a_i[31], a_i[30:0]};
```

### 4.3 Bug Fix 3: Opcode 3'b111 Discrepancy — Deprecation of SEL in Favor of FMOV

**Defect Manifestation:** Design document `02_tide_fp32_add.md` specified opcode `3'b111` as a conditional select (`SEL`: `cmp_lt_o ? a : b`). However, during system integration with the assembler (`tide_asm.py`) and microcode kernel assembly (`10_unit_pre.tasm`), `3'b111` was formally bound in `isa.yaml` to `FMOV` (floating-point register passthrough: $R_d \leftarrow R_a$). In microcode execution, `FMOV` instructions erroneously returned $R_b$ whenever a prior comparison left `cmp_lt_hold` asserted.

**RTL Fix:** Opcode `3'b111` was renamed from `OP_SEL` to `OP_MOV`, and wired as an unconditional passthrough of `a_i`:
```systemverilog
localparam logic [2:0] OP_MOV = 3'b111;
assign fmov_result_comb = a_i;
```

### 4.4 Bug Fix 4: FCMP Numeric Register Write-Back for RISC-V PCPI

**Defect Manifestation:** In standalone VLIW sequencer operation, `FCMP` only outputs condition codes via side-channel wires `cmp_lt_o` and `cmp_eq_o`. However, when TIDE is integrated into the host RISC-V processor (PicoRV32) via the PCPI coprocessor interface, custom-1 R-type instructions require a valid 32-bit integer write-back value to register $R_d$. Leaving `bypass_result_o` tied to zero caused RISC-V conditional evaluations to always evaluate false.

**RTL Fix:** `FCMP` was updated to drive boolean comparison results onto the numeric bus:
```systemverilog
assign cmp_result_comb = {31'b0, cmp_lt_comb};
```

---

## 5. Verification & Silicon Provenance Summary

The module was verified using Verilator against Berkeley SoftFloat 3e across:
- **$100{,}000{,}000$ random stimulus vectors** covering normal, subnormal, zero, infinity, and NaN permutations with 0 mismatches.
- Directed edge test cases:
  - Exact cancellation: $1.000000119 - 1.0 = 2^{-23}$ (`0x3F800001 - 0x3F800000 = 0x34000000`).
  - Subnormal addition carry-out: `0x00400000 + 0x00400000 = 0x00800000`.
  - Max signed zero selection: `MAX(-0.0, +0.0) = +0.0`.
  - Catastrophic subtraction with large shift borrow propagation.

Physical silicon tapeout was achieved in Silicon Chip OS0005 ($2.0 \times 2.0\text{ mm}^2$, SCL 180nm CMOS 1P6M) with zero DRC/LVS violations and static timing closure at 50 MHz.
