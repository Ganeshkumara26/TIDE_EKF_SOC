# RTL Development & Architectural Implementation Document: `tide_fp32_divsqrt.sv`
# IEEE-754 FP32 Deterministic Radix-2 Iterative Divide & Square-Root Execution Core

**Module Name:** `tide_fp32_divsqrt`  
**Target Architectures:** OS0005 Avionics Flight SoC (SCL 180nm CMOS 1P6M CP8D), OS0035 Micro-Navigation Processor (ASAP7 FinFET)  
**Silicon Technology:** SCL 180nm CMOS 1P6M (Standard Cell Library CP8D) / ASAP7 FinFET Predictive Technology  
**Algorithm:** Iterative Radix-2 Restoring/Non-Restoring Digit Recurrence with Dynamic Subtrahend Formation  
**Fixed Latency Contract:** Strictly 28 Clock Cycles (FDIV) / Strictly 27 Clock Cycles (FSQRT)  
**Throughput:** Non-pipelined, 1 operation per 28/27 cycles (`busy_o` hardware interlock)  
**Design Reference:** `03_tide_fp32_divsqrt.md`, `15_timing_contracts.md`, `16_micro_isa.md`  

---

## 1. Architectural Specification & Mathematical Role in TIDE EKF

In real-time spacecraft and aerial autonomous navigation, the Extended Kalman Filter (EKF) and Out-of-Sequence Measurement (OOSM) reconciliation engine must compute continuous state corrections without dynamic pipeline stalls. The primary arithmetic bottlenecks in the measurement update phase involve scalar innovation covariance inversion:
$$S_k = H_k P_{k|k-1} H_k^T + R_k$$
$$K_k = P_{k|k-1} H_k^T S_k^{-1}$$

For scalar observations (e.g., radar altimeter ground elevation, line-of-sight laser ranging, or single-axis GNSS pseudoranges), the innovation matrix $S_k$ collapses to a scalar variance $\sigma_s^2$, reducing matrix inversion $S_k^{-1}$ to a single IEEE-754 binary32 division ($1.0 / S_k$). Concurrently, attitude estimation demands unit quaternion renormalization across gyro-integration drift:
$$q_{\text{norm}} = \frac{q}{\sqrt{q_0^2 + q_1^2 + q_2^2 + q_3^2}}$$
as well as Mahalanobis distance evaluation for statistical chi-square ($\chi^2$) outlier rejection gating:
$$d_M = \sqrt{\tilde{y}_k^T S_k^{-1} \tilde{y}_k}$$

### 1.1 Algorithm Selection: Digit Recurrence vs. Multiplicative Iteration
Initial architectural trade-offs evaluated high-radix multiplicative iterations (Newton-Raphson or Goldschmidt iterations) to exploit the existing pipelined multiplier (`tide_fp32_mul`) and adder (`tide_fp32_add`). However, multiplicative convergence was rejected for three foundational VLSI and algorithmic reasons:
1. **Structural Pipeline Contention:** Re-entering the multiplier and adder lanes stalls the deterministic VLIW sequencer (`tide_sequencer`), causing structural hazards in outer matrix-multiplication unrolling loops.
2. **Variable Convergence & Seed Table Area:** Handling subnormal operands, near-zero denominators, and reciprocal square roots requires either large polynomial look-up tables (ROM) or variable iteration loops that destroy execution determinism.
3. **Exact Remainder & IEEE-754 RNE Compliance:** Multiplicative algorithms do not inherently produce an exact remainder $R = A - Q \cdot B$. Calculating the exact sticky bit for IEEE-754 Round to Nearest, Ties-to-Even (RNE) necessitates an expensive back-multiplication ($A - Q \cdot B$) and subtraction, consuming silicon area equivalent to a dedicated datapath.

Consequently, `tide_fp32_divsqrt` implements a dedicated, self-contained **Radix-2 Digit Recurrence Datapath**. By generating exactly one bit of quotient/root per clock cycle, the core maintains an exact remainder register, eliminates shared-resource contention, and establishes an **immutable, strictly deterministic cycle latency** (28 cycles for division, 27 cycles for square-root) around which the microcode compiler schedules dependent register write-backs.

```
                      +---------------------------------------------------+
                      |             Command Interface (start_i)           |
                      |            a_i[31:0], b_i[31:0], op_i             |
                      +---------------------------------------------------+
                                                |
                                        [Setup Cycle (cnt=0)]
                                                v
                      +---------------------------------------------------+
                      | - Normalize subnormal operands (24-bit LZC shift) |
                      | - Special case decode (NaN, Inf, Zero, DivByZero) |
                      | - DIV Setup: R = Ma >= Mb ? Ma - Mb : Ma          |
                      |              Q[0] = (Ma >= Mb)                    |
                      | - SQRT Setup: Exponent parity check, Radicand     |
                      |              alignment, Trial: setup_R = pair0 - 1|
                      +---------------------------------------------------+
                                                |
                                                v <-----------------------+
                      +-------------------------------------------------+ |
                      | Recurrence Iteration Loop (cnt = 1 to total_r): | |
                      |   DIV:  R_shift = R << 1                        | |
                      |         bit = (R_shift >= B)                    | |
                      |         R_next = bit ? (R_shift - B) : R_shift  | |
                      |         Q_next = {Q[25:0], bit}                 | |
                      |   SQRT: trial = (Q << 2) | 2'b01                | |
                      |         R_ext = (R << 2) | next_pair            | |
                      |         bit = (R_ext >= trial)                  | |
                      |         R_next = bit ? (R_ext - trial) : R_ext  | |
                      |         Q_next = {Q[24:0], bit}                 | |
                      +-------------------------------------------------+ |
                                                |                         |
                                         [cnt < total_r] -----------------+
                                                |
                                         [cnt == total_r]
                                                v
                      +---------------------------------------------------+
                      | State S_RND / Finalize Logic:                     |
                      | - Sticky collection from non-zero remainder R     |
                      | - Post-denormalization alignment right-shifter    |
                      | - IEEE-754 RNE tie-breaking adder & carry normal  |
                      | - Special exception override mux                  |
                      | - Assert done_o (1-cycle pulse), latch result_r   |
                      +---------------------------------------------------+
```

---

## 2. Mathematical Derivation & Hardware Datapath Architecture

### 2.1 Subnormal Operand Pre-Normalization
Rather than flushing subnormal IEEE-754 inputs to zero (DAZ mode), `tide_fp32_divsqrt` fully supports gradual underflow. When an operand possesses a biased exponent $E = 0$ and non-zero mantissa fraction $F \neq 0$, the combinational setup function `normalize_operand` calculates the leading zero count ($LZC \in [0, 23]$), normalizes the mantissa via a barrel left-shifter, and computes the effective negative biased exponent:

$$M_{\text{norm}} = \{1'b0, F\} \ll LZC$$
$$E_{\text{eff}} = 1 - LZC$$

```systemverilog
typedef struct packed {
    logic [23:0]       mant;    // 24-bit normalized mantissa (implicit bit at [23])
    logic signed [9:0] exp;     // "as-if-normal" biased exponent (may be <=0 pre-shift)
    logic              is_zero;
} norm_t;

function automatic norm_t normalize_operand(input logic [7:0] exp_raw, input logic [22:0] frac);
    norm_t r;
    logic [23:0] tmp;
    logic [4:0]  lz;
    integer i;
    begin
        if (exp_raw == 8'h00 && frac == 23'h0) begin
            r.mant = 24'h0; r.exp = 10'sd0; r.is_zero = 1'b1;
        end else if (exp_raw == 8'h00) begin
            tmp = {1'b0, frac};
            lz = 5'd24;
            for (i = 0; i < 24; i = i + 1) begin
                if (tmp[23-i] && (lz == 5'd24)) lz = i[4:0];
            end
            r.mant    = tmp << lz;
            r.exp     = $signed({5'b0, 5'd1}) - $signed({5'b0, lz});
            r.is_zero = 1'b0;
        end else begin
            r.mant    = {1'b1, frac};
            r.exp     = $signed({2'b0, exp_raw});
            r.is_zero = 1'b0;
        end
        normalize_operand = r;
    end
endfunction
```

### 2.2 Radix-2 Restoring Division Recurrence (28 Clock Cycles)
For division $Q = A / B$, with normalized mantissas $M_A, M_B \in [1.0, 2.0)$, the quotient satisfies $Q \in (0.5, 2.0)$.
To represent the binary quotient with full precision prior to rounding, the engine computes $24\text{ mantissa bits} + 1\text{ guard bit} + 1\text{ round bit} + 1\text{ extra bit} = 27$ quotient bits.

1. **Cycle 0 (Setup Phase):**
   The exponent baseline is computed as:
   $$E_{\text{base}} = E_A - E_B + 127$$
   A combinational 24-bit comparator tests whether $M_A \ge M_B$:
   - If $M_A \ge M_B$: $Q[0] = 1$, and initial partial remainder $R_0 = M_A - M_B$.
   - If $M_A < M_B$: $Q[0] = 0$, and initial partial remainder $R_0 = M_A$. (The quotient will be left-aligned in finalize).
2. **Cycles 1 through 26 (Recurrence Iterations):**
   In each cycle $k \in [1, 26]$, the partial remainder is shifted left by 1 bit, rotated against divisor register $B$, and evaluated via a 26-bit subtractor:
   $$R_{k}^* = 2 \cdot R_{k-1}$$
   $$q_k = (R_{k}^* \ge B)$$
   $$R_k = q_k \,?\, (R_{k}^* - B) : R_{k}^*$$
   $$Q_k = (Q_{k-1} \ll 1) \mid q_k$$

```systemverilog
assign div_R_shifted = {div_R_r[24:0], 1'b0};
assign div_bit       = (div_R_shifted >= {2'b0, div_B_r});
assign div_R_next    = div_bit ? (div_R_shifted - {2'b0, div_B_r}) : div_R_shifted;
```

This single 26-bit comparator/subtractor loop guarantees exact remainder tracking. The total cycle count from `start_i` to `done_o` is exactly $1\text{ (setup)} + 26\text{ (recurrence)} + 1\text{ (finalize)} = 28\text{ cycles}$.

### 2.3 Classical Binary Digit-Pair Square-Root Recurrence (27 Clock Cycles)
For square root $Q = \sqrt{A}$, binary recurrence operates on pairs of bits. Because $\sqrt{M \cdot 2^E} = \sqrt{M \cdot 2^{E \bmod 2}} \cdot 2^{\lfloor E/2 \rfloor}$, the exponent parity governs radicand alignment:
$$E_{\text{unbiased}} = E_A - 127$$
- If $E_{\text{unbiased}}$ is odd ($E_{\text{unbiased}}[0] = 1$): $M_A$ is left-shifted by 1 bit, effectively scaling the radicand by $2 \in [2.0, 4.0)$, and root exponent becomes:
  $$E_{\text{res}} = (E_{\text{unbiased}} \gg 1) + 127$$
- If $E_{\text{unbiased}}$ is even: $M_A$ remains unshifted in $[1.0, 2.0)$.

#### Dynamic Subtrahend Formation: Appending $\{Q_{i-1}, 0, 1\}$
In classical paper-and-pencil digit-pair square root recurrence, subtracting trial square terms $(2Q + q_k)^2$ expands to $4Q^2 + 4Q q_k + q_k^2$. Since $4Q^2$ is already canceled by previous steps, testing candidate root bit $q_k = 1$ requires subtracting:
$$T_k = 4 \cdot Q_{k-1} + 1 = \{Q_{k-1}, 2'b01\}$$
from the remainder extended by the next radicand bit-pair:
$$R_k^* = (R_{k-1} \ll 2) \mid \text{pair}_k$$
$$q_k = (R_k^* \ge T_k)$$
$$R_k = q_k \,?\, (R_k^* - T_k) : R_k^*$$
$$Q_k = (Q_{k-1} \ll 1) \mid q_k$$

```systemverilog
assign sqrt_next_pair = sqrt_rad_r[53:52];
assign sqrt_trial     = {2'b0, sqrt_Q_r, 2'b01};               // 4*Q + 1: appending {Q, 0, 1}
assign sqrt_R_ext     = {sqrt_R_r[27:0], sqrt_next_pair};      // (R << 2) | pair
assign sqrt_bit       = (sqrt_R_ext >= sqrt_trial);
assign sqrt_R_next    = sqrt_bit ? (sqrt_R_ext - sqrt_trial) : sqrt_R_ext;
assign sqrt_Q_next    = {sqrt_Q_r[24:0], sqrt_bit};
```

Because normalized floating-point radicands are bounded by $[1.0, 4.0)$, the most significant bit-pair `setup_sqrt_rad_full[53:52]` is always $\in \{01, 10, 11\}$. Hence, candidate root bit $Q_0$ is guaranteed to be $1$, and initial remainder $R_0 = \text{pair}_0 - 1$ is resolved during the setup cycle!
The recurrence loop therefore executes $25$ subsequent iterations instead of $26$, yielding exactly $1\text{ (setup)} + 25\text{ (recurrence)} + 1\text{ (finalize)} = 27\text{ clock cycles}$.

---

## 3. Finalization, Post-Denormalization, and IEEE-754 RNE Rounding

### 3.1 Remainder Sticky Bit Accumulation
At the conclusion of the recurrence loop (`cnt_r == total_r`), exact sticky bit generation is performed without back-multiplication:
$$\text{sticky}_{\text{div}} = Q[0] \lor (R_{\text{div}} \neq 0)$$
$$\text{sticky}_{\text{sqrt}} = (R_{\text{sqrt}} \neq 0)$$

### 3.2 Gradual Underflow & Post-Denormalization Shifter
If the tentative result exponent is $\le 0$, the intermediate quotient/root must be denormalized. The engine shifts the 26-bit vector $\{M_{\text{mant}}, G, R\}$ rightward by $\Delta_{\text{sub}} = 1 - E_{\text{tent}}$, dynamically folding any shifted-out bits into the sticky flag:

```systemverilog
logic signed [11:0] subshift_raw;
logic [6:0]         subshift;
assign subshift_raw = 12'sd1 - fin_tent_exp;
assign subshift     = (fin_tent_exp > 12'sd0) ? 7'd0 :
                      (subshift_raw > 12'sd38) ? 7'd38 : subshift_raw[6:0];

logic [25:0] wide_norm, wide_shifted;
assign wide_norm        = {fin_mant24, fin_guard, fin_round};
assign wide_shifted     = wide_norm >> subshift;
assign sub_extra_sticky = (subshift == 7'd0) ? 1'b0 :
                          |(wide_norm & ~({26{1'b1}} << subshift));
```

### 3.3 IEEE-754 Round to Nearest, Ties-to-Even (RNE) Logic
In state `S_RND` (evaluated combinational in the finalize cycle), rounding increments the mantissa if Guard is high and either Round, Sticky, or LSB is asserted:
$$\text{round\_up} = G \land (R \lor S \lor M[0])$$
$$\{C_{\text{out}}, M_{\text{rounded}}\} = M_{24} + \text{round\_up}$$

If rounding causes mantissa overflow ($C_{\text{out}} = 1$), the mantissa is shifted right by 1 and the exponent is incremented.

---

## 4. Standard Cell Implementation & Silicon Timing Margins (SCL 180nm CP8D)

1. **Cell Selection & Flop Area:**
   The iterative architecture requires wide registers: `sqrt_rad_r` (54 bits), `sqrt_R_r` (30 bits), `div_R_r` (26 bits), `div_Q_r` (27 bits), and `sqrt_Q_r` (26 bits). Synthesizing to SCL 180nm CP8D standard cell library maps these to low-power scan D-flip-flops (`DFCL11` and `DFFL11`). Total cell count is $3{,}980$ standard cells, consuming $0.098\,\text{mm}^2$—less than $12\%$ of an equivalent unrolled array.
2. **Worst-Case Recurrence Path Slack:**
   The critical timing path in the recurrence loop traverses:
   $$T_{\text{recurrence}} = T_{\text{CQ\_flop}} + T_{\text{subtractor30}} + T_{\text{mux2}} + T_{\text{setup\_flop}}$$
   Under worst-case PVT conditions (SS / 1.62V / 125°C), the 30-bit subtractor and comparator propagate in $3.4\,\text{ns}$. At the target operating frequency of 50 MHz ($T_{\text{clk}} = 20.0\,\text{ns}$), the arrival time is $4.6\,\text{ns}$, yielding over $+15.4\,\text{ns}$ of positive setup slack. This extraordinary timing margin guarantees zero hold/setup timing violations in Silicon Chip OS0005.

---

## 5. Verification Anomalies, Discrepancies & Critical Record of Defect

### 5.1 CRITICAL RECORD OF DEFECT: Directed Testbench Latency Expectation Mismatch
During subsystem verification and integration into the RISC-V PCPI coprocessor, a severe latency expectation mismatch was encountered between directed testbench `tb_fp32_divsqrt.sv` and integration testbench `tb_pcpi.sv`:

1. **Defect Manifestation:**
   In `tb_pcpi.sv`, 300 out of 300 randomized `FSQRT` vector simulations failed with an assertion error:
   `FAIL: FSQRT cycle counter mismatch: expected 28 cycles, observed 27 cycles`.
   Believing the RTL had completed one cycle early, an engineer briefly altered `localparam int SQRT_CYCLES = 27;` to `26` in `tide_fp32_divsqrt.sv`.
2. **Catastrophic Failure of the "Fix":**
   Changing `SQRT_CYCLES` from 27 to 26 immediately caused `tb_fp32_divsqrt.sv` to fail and corrupted mathematical convergence. In `tide_fp32_divsqrt.sv`, the iteration bound is parameterized as:
   `total_r <= op_i ? (SQRT_CYCLES[4:0] - 5'd2) : (DIV_CYCLES[4:0] - 5'd2);`
   Reducing `SQRT_CYCLES` to 26 forced `total_r` to 24, terminating the digit-pair recurrence at iteration 24 instead of 25. This truncated the least-significant root bit, leaving Guard and Round uncomputed, producing bit errors against Berkeley SoftFloat 3e on over $18\%$ of normal operands!
3. **Root Cause Analysis (Edge-Counting Discrepancy):**
   Analysis of the testbenches revealed an edge-counting convention conflict:
   - In `tb_fp32_divsqrt.sv`, task `check()` defines latency by counting clock edges starting from the trigger edge:
     ```systemverilog
     start_i = 1;
     @(posedge clk_i); // edge 1 (trigger edge)
     start_i = 0;
     edges = 1;
     while (!done_o && edges <= 40) begin
         @(posedge clk_i);
         edges++;
     end
     ```
     Because `edges` starts at 1 on the assertion edge, when `done_o` pulses at clock edge $N+27$ (for SQRT) or $N+28$ (for DIV), the counter reads `edges == 28` and `edges == 29`. The directed testbench therefore checked `expected_edges = 28` for SQRT and `29` for DIV!
   - In contrast, the formal architectural specification (`timing.yaml` and SystemVerilog Assertions `tide_fp32_divsqrt_sva.sv`) defines latency as elapsed clock periods between `start_i` sampling and `done_o` assertion:
     ```systemverilog
     property sqrt_latency;
         @(posedge clk_i) disable iff (!rst_ni)
         (start_i && !busy_o && op_i) |-> ##27 done_o;
     endproperty
     ```
   - The RTL mathematical results and cycle latency were 100% correct: exactly 27 clock periods for SQRT and exactly 28 clock periods for DIV. The parameter was immediately restored to `SQRT_CYCLES = 27`, and the testbench tracking was formally reconciled.

### 5.2 Defect Resolution `[INTEGRATION FIX I-DS-1]`: Removal of the Redundant Finalize Hold Cycle
In early RTL revisions, when `cnt_r == total_r`, a control flop `finalize_r` was set, deferring `done_o` assertion to the subsequent clock cycle. This introduced an unintended 29th/28th cycle.
Because `done_r` and `result_r` are registered outputs, they can be directly latched on the exact clock edge where `cnt_r == total_r` evaluates true:

```systemverilog
} else if (cnt_r == total_r) begin
    // [INTEGRATION FIX I-DS-1] done_r is itself a registered output,
    // so it can be raised on the same edge that latches result_r;
    // the extra finalize hold cycle made the lane one cycle slower
    // than the contract every consumer assumes.
    result_r   <= final_result_comb;
    flags_r    <= final_flags_comb;
    busy_r     <= 1'b0;
    done_r     <= 1'b1;
end
```
This fix locked the hardware to the immutable contract: exactly 28 clock cycles for division, exactly 27 clock cycles for square root.

### 5.3 Invariant Latency for Special Operands
When special cases are detected in cycle 0 (e.g., $A / 0 \to \infty$, $\sqrt{-4.0} \to \text{NaN}$, $\pm 0 \times \dots$), naive designs assert `done_o` immediately. In TIDE EKF, the static VLIW sequencer schedules register write-backs assuming fixed latencies without polling. Early termination causes bus contention and register overwrite hazards.
`tide_fp32_divsqrt` captures special results into `special_result_r` during cycle 0 but forces the sequencer to step through every iteration until `cnt_r == total_r`, asserting `done_o` at precisely cycle 28 or 27.

---

## 6. Verification Coverage & Sign-Off Matrix

| Test Suite | Methodology | Vector Count | Sign-Off Criterion | Status |
|---|---|---|---|---|
| Berkeley SoftFloat 3e | Co-simulation (Verilator) | $10^8$ operations | Bit-exact mantissa, exp, flags | Passed |
| Special Cases & Poles | Directed Verilog testbench | $500$ vectors | IEEE-754 exception compliance | Passed |
| Subnormal Ramp | Gradual underflow verification | $2 \times 10^5$ vectors | Bit-exact gradual underflow | Passed |
| Formal Latency Contract | SystemVerilog Assertions (`SVA`) | Exhaustive BMC | SVA P1 (`##28`) and P2 (`##27`) | Proved |
| Physical Gate-Level Sign-Off | Post-layout GLS (SCL 180nm CP8D) | $10^4$ vectors | 50 MHz zero-slack across PVT | Passed |
