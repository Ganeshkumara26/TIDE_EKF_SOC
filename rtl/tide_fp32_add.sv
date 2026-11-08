// -------------------------------------------------------------------
// tide_fp32_add.sv — IEEE-754 binary32 pipelined adder/subtractor
//                     + single-cycle bypass ops (CMP/MIN/MAX/ABS/NEG/SEL)
// Source spec: doc/design/02_tide_fp32_add.md
// Owner: Agent 2
//
// ADD/SUB: 3-stage pipeline (align -> add+LZC -> normalize/round), 3-cycle
//          latency, 1 op/cycle throughput.
// CMP/MIN/MAX/ABS/NEG/SEL: single-cycle bypass path, independent of the
//          ADD/SUB pipeline (can be issued on any cycle without regard to
//          an in-flight ADD/SUB).
//
// Implementation note (deviation from the design doc's stage-2 pseudocode,
// logged in ASSUMPTIONS.md as [A-ADD-1]): rather than special-casing
// "carry-out => LZC=0" vs "cancellation => LZC via priority encoder"
// separately, this implementation forms a single unified 27-bit value
// (zero-extended for the subtract case, natural carry bit for the add
// case) and runs ONE leading-zero counter + left-shift over it. This
// generalizes correctly to the add path producing leading zeros too
// (subnormal + subnormal), which the doc's simplified two-case
// description does not explicitly cover, while still meeting the
// documented 3-cycle timing contract and producing bit-identical results
// to the doc's algorithm for all cases the doc does describe.
// -------------------------------------------------------------------
module tide_fp32_add (
    input  logic        clk_i,
    input  logic        rst_ni,

    // Pipeline input
    input  logic        valid_i,
    input  logic [2:0]  op_i,
    input  logic [31:0] a_i,
    input  logic [31:0] b_i,

    // Pipeline output
    output logic        valid_o,
    output logic [31:0] result_o,
    output logic [4:0]  flags_o,

    // Single-cycle bypass output
    output logic        bypass_valid_o,
    output logic [31:0] bypass_result_o,
    output logic        cmp_lt_o,
    output logic        cmp_eq_o
);

    localparam logic [2:0] OP_ADD = 3'b000;
    localparam logic [2:0] OP_SUB = 3'b001;
    localparam logic [2:0] OP_CMP = 3'b010;
    localparam logic [2:0] OP_MIN = 3'b011;
    localparam logic [2:0] OP_MAX = 3'b100;
    localparam logic [2:0] OP_ABS = 3'b101;
    localparam logic [2:0] OP_NEG = 3'b110;
    // Renamed from OP_SEL during integration -- see the FMOV/SEL note below
    // (encoding unchanged: 3'b111, matches isa.yaml add_ops FMOV).
    localparam logic [2:0] OP_MOV = 3'b111;

    logic is_pipelined_op;
    assign is_pipelined_op = (op_i == OP_ADD) || (op_i == OP_SUB);

    // =====================================================================
    // Shared classification (used by both pipeline and CMP/MIN/MAX paths)
    // =====================================================================
    logic        sign_a;
    logic [7:0]  exp_a_raw;
    logic [22:0] frac_a;
    logic        a_is_zero, a_is_inf, a_is_nan, a_is_snan, a_is_sub;

    // "Effective B": for SUB, negate the sign of B so SUB(a,b) == ADD(a,-b).
    logic        sign_b_eff;
    logic [7:0]  exp_b_raw;
    logic [22:0] frac_b;
    logic        b_is_zero, b_is_inf, b_is_nan, b_is_snan, b_is_sub;

    always_comb begin
        sign_a    = a_i[31];
        exp_a_raw = a_i[30:23];
        frac_a    = a_i[22:0];
        a_is_zero = (exp_a_raw == 8'h00) && (frac_a == 23'h0);
        a_is_inf  = (exp_a_raw == 8'hFF) && (frac_a == 23'h0);
        a_is_nan  = (exp_a_raw == 8'hFF) && (frac_a != 23'h0);
        a_is_snan = a_is_nan && !frac_a[22];
        a_is_sub  = (exp_a_raw == 8'h00) && (frac_a != 23'h0);

        sign_b_eff = (op_i == OP_SUB) ? ~b_i[31] : b_i[31];
        exp_b_raw  = b_i[30:23];
        frac_b     = b_i[22:0];
        b_is_zero  = (exp_b_raw == 8'h00) && (frac_b == 23'h0);
        b_is_inf   = (exp_b_raw == 8'hFF) && (frac_b == 23'h0);
        b_is_nan   = (exp_b_raw == 8'hFF) && (frac_b != 23'h0);
        b_is_snan  = b_is_nan && !frac_b[22];
        b_is_sub   = (exp_b_raw == 8'h00) && (frac_b != 23'h0);
    end

    // =====================================================================
    // Stage 1: unpack + align
    // =====================================================================
    logic [23:0] mant_a, mant_b;
    logic [7:0]  exp_a_eff, exp_b_eff;
    always_comb begin
        mant_a    = a_is_zero ? 24'h0 : (a_is_sub ? {1'b0, frac_a} : {1'b1, frac_a});
        mant_b    = b_is_zero ? 24'h0 : (b_is_sub ? {1'b0, frac_b} : {1'b1, frac_b});
        exp_a_eff = (a_is_zero || a_is_sub) ? 8'h01 : exp_a_raw;
        exp_b_eff = (b_is_zero || b_is_sub) ? 8'h01 : exp_b_raw;
    end

    logic eff_add_s0;      // effective operation: 1=add magnitudes, 0=subtract
    assign eff_add_s0 = (sign_a == sign_b_eff);

    // Which operand has the larger (or equal) magnitude?
    logic a_ge_b_s0;
    assign a_ge_b_s0 = (exp_a_eff > exp_b_eff) ||
                        ((exp_a_eff == exp_b_eff) && (mant_a >= mant_b));

    logic [7:0]  exp_big_s0;
    logic [23:0] mant_big_s0, mant_small_s0;
    logic        sign_result_s0;
    always_comb begin
        if (a_ge_b_s0) begin
            exp_big_s0     = exp_a_eff;
            mant_big_s0    = mant_a;
            mant_small_s0  = mant_b;
            sign_result_s0 = sign_a;
        end else begin
            exp_big_s0     = exp_b_eff;
            mant_big_s0    = mant_b;
            mant_small_s0  = mant_a;
            sign_result_s0 = sign_b_eff;
        end
    end

    logic [8:0] exp_diff9_s0; // exp_big - exp_small, always >= 0 by construction
    assign exp_diff9_s0 = a_ge_b_s0 ? ({1'b0, exp_a_eff} - {1'b0, exp_b_eff})
                                     : ({1'b0, exp_b_eff} - {1'b0, exp_a_eff});

    logic [4:0] algn_shift_s0; // clamped to 27 (max meaningful shift)
    assign algn_shift_s0 = (exp_diff9_s0 > 9'd27) ? 5'd27 : exp_diff9_s0[4:0];

    // Pad both mantissas to 27 bits (24 mantissa + guard + round + sticky).
    // IMPORTANT: the sticky bit must be folded into bit0 of the SHIFTED small
    // operand and take part in the add/sub itself (not OR'd into the result
    // afterward). This is the standard "G,R,S" technique required for a
    // subtractor to round correctly under cancellation: an externally-OR'd
    // sticky does not propagate borrows correctly and produces off-by-one-ULP
    // results whenever the truncated tail is nonzero. See ASSUMPTIONS.md
    // [A-ADD-1] for the writeup.
    logic [26:0] big27_s0, small27_pad_s0;
    assign big27_s0       = {mant_big_s0, 3'b000};
    assign small27_pad_s0 = {mant_small_s0, 3'b000};

    logic [26:0] small27_shifted_raw_s0;
    logic        sticky_forced_s0;
    assign small27_shifted_raw_s0 = (algn_shift_s0 == 5'd0) ? small27_pad_s0
                                                              : (small27_pad_s0 >> algn_shift_s0);
    assign sticky_forced_s0 = (algn_shift_s0 == 5'd0) ? 1'b0 :
        |(small27_pad_s0 & ~(27'h7FFFFFF << algn_shift_s0));

    logic [26:0] small27_shifted_s0;
    assign small27_shifted_s0 = {small27_shifted_raw_s0[26:1],
                                  small27_shifted_raw_s0[0] | sticky_forced_s0};

    // ---- Special-case classification (NaN / Inf), stage 1 ----
    logic any_nan_s0, invalid_nan_in_s0, inf_opp_sign_s0;
    logic result_is_nan_s0, result_is_inf_s0;
    logic inf_result_sign_s0;
    always_comb begin
        any_nan_s0        = a_is_nan || b_is_nan;
        invalid_nan_in_s0 = a_is_snan || b_is_snan;
        inf_opp_sign_s0   = a_is_inf && b_is_inf && (sign_a != sign_b_eff);
        result_is_nan_s0  = any_nan_s0 || inf_opp_sign_s0;
        result_is_inf_s0  = !result_is_nan_s0 && (a_is_inf || b_is_inf);
        inf_result_sign_s0= a_is_inf ? sign_a : sign_b_eff;
    end

    // ---- Stage 1 -> Stage 2 registers ----
    logic        valid_s1;
    logic        eff_add_s1;
    logic [26:0] big27_s1, small27_shifted_s1;
    logic [7:0]  exp_big_s1;
    logic        sign_result_s1;
    logic        result_is_nan_s1, result_is_inf_s1, invalid_nan_in_s1, inf_result_sign_s1;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            valid_s1            <= 1'b0;
            eff_add_s1          <= 1'b0;
            big27_s1            <= '0;
            small27_shifted_s1  <= '0;
            exp_big_s1          <= '0;
            sign_result_s1      <= 1'b0;
            result_is_nan_s1    <= 1'b0;
            result_is_inf_s1    <= 1'b0;
            invalid_nan_in_s1   <= 1'b0;
            inf_result_sign_s1  <= 1'b0;
        end else begin
            valid_s1            <= valid_i && is_pipelined_op;
            eff_add_s1          <= eff_add_s0;
            big27_s1            <= big27_s0;
            small27_shifted_s1  <= small27_shifted_s0;
            exp_big_s1          <= exp_big_s0;
            sign_result_s1      <= sign_result_s0;
            result_is_nan_s1    <= result_is_nan_s0;
            result_is_inf_s1    <= result_is_inf_s0;
            invalid_nan_in_s1   <= invalid_nan_in_s0;
            inf_result_sign_s1  <= inf_result_sign_s0;
        end
    end

    // =====================================================================
    // Stage 2: mantissa add/subtract + leading-zero count
    // =====================================================================
    logic [27:0] valword28_s1;
    always_comb begin
        if (eff_add_s1)
            valword28_s1 = {1'b0, big27_s1} + {1'b0, small27_shifted_s1};
        else
            valword28_s1 = {1'b0, big27_s1 - small27_shifted_s1};
    end

    function automatic [4:0] lzc28(input logic [27:0] v);
        integer i;
        begin
            lzc28 = 5'd28;
            for (i = 0; i < 28; i = i + 1) begin
                if (v[27-i] && (lzc28 == 5'd28)) lzc28 = i[4:0];
            end
        end
    endfunction

    logic [4:0] nlz28_s1;
    assign nlz28_s1 = lzc28(valword28_s1);
    logic is_exact_zero_s1;
    assign is_exact_zero_s1 = (valword28_s1 == 28'h0);

    // ---- Stage 2 -> Stage 3 registers ----
    logic        valid_s2;
    logic        eff_add_s2;
    logic [27:0] valword28_s2;
    logic [4:0]  nlz28_s2;
    logic        is_exact_zero_s2;
    logic [7:0]  exp_big_s2;
    logic        sign_result_s2;
    logic        result_is_nan_s2, result_is_inf_s2, invalid_nan_in_s2, inf_result_sign_s2;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            valid_s2           <= 1'b0;
            eff_add_s2         <= 1'b0;
            valword28_s2       <= '0;
            nlz28_s2           <= '0;
            is_exact_zero_s2   <= 1'b0;
            exp_big_s2         <= '0;
            sign_result_s2     <= 1'b0;
            result_is_nan_s2   <= 1'b0;
            result_is_inf_s2   <= 1'b0;
            invalid_nan_in_s2  <= 1'b0;
            inf_result_sign_s2 <= 1'b0;
        end else begin
            valid_s2           <= valid_s1;
            eff_add_s2         <= eff_add_s1;
            valword28_s2       <= valword28_s1;
            nlz28_s2           <= nlz28_s1;
            is_exact_zero_s2   <= is_exact_zero_s1;
            exp_big_s2         <= exp_big_s1;
            sign_result_s2     <= sign_result_s1;
            result_is_nan_s2   <= result_is_nan_s1;
            result_is_inf_s2   <= result_is_inf_s1;
            invalid_nan_in_s2  <= invalid_nan_in_s1;
            inf_result_sign_s2 <= inf_result_sign_s1;
        end
    end

    // =====================================================================
    // Stage 3: normalize, round (RNE), pack
    // =====================================================================
    logic [27:0] shifted28_s3;
    assign shifted28_s3 = valword28_s2 << nlz28_s2;

    logic [23:0] mant24_norm_s3;
    logic        guard_norm_s3, round_norm_s3, sticky_norm_s3;
    assign mant24_norm_s3 = shifted28_s3[27:4];
    assign guard_norm_s3  = shifted28_s3[3];
    assign round_norm_s3  = shifted28_s3[2];
    assign sticky_norm_s3 = shifted28_s3[1] | shifted28_s3[0];

    logic signed [11:0] tent_exp_s3;
    assign tent_exp_s3 = $signed({4'b0, exp_big_s2}) + 12'sd1 - {7'b0, nlz28_s2};

    logic pre_overflow_s3;
    assign pre_overflow_s3 = (tent_exp_s3 >= 12'sd255);

    logic signed [11:0] subshift_raw_s3;
    logic [6:0]          subshift_s3;
    assign subshift_raw_s3 = 12'sd1 - tent_exp_s3;
    assign subshift_s3 = (tent_exp_s3 > 12'sd0) ? 7'd0 :
                          (subshift_raw_s3 > 12'sd38) ? 7'd38 : subshift_raw_s3[6:0];

    logic is_subnormal_path_s3;
    assign is_subnormal_path_s3 = (tent_exp_s3 <= 12'sd0) && !pre_overflow_s3;

    logic [25:0] wide_norm_s3;
    assign wide_norm_s3 = {mant24_norm_s3, guard_norm_s3, round_norm_s3};

    logic [25:0] wide_shifted_s3;
    logic        sub_extra_sticky_s3;
    assign wide_shifted_s3 = wide_norm_s3 >> subshift_s3;
    assign sub_extra_sticky_s3 =
        (subshift_s3 == 7'd0) ? 1'b0 :
        (|(wide_norm_s3 & ~({26{1'b1}} << subshift_s3)));

    logic [23:0] mant24_pre_s3;
    logic        guard_pre_s3, round_pre_s3, sticky_pre_s3;
    logic [7:0]  exp_pre_s3;
    always_comb begin
        if (is_subnormal_path_s3) begin
            mant24_pre_s3 = wide_shifted_s3[25:2];
            guard_pre_s3  = wide_shifted_s3[1];
            round_pre_s3  = wide_shifted_s3[0];
            sticky_pre_s3 = sticky_norm_s3 | sub_extra_sticky_s3;
            exp_pre_s3    = 8'h00;
        end else begin
            mant24_pre_s3 = mant24_norm_s3;
            guard_pre_s3  = guard_norm_s3;
            round_pre_s3  = round_norm_s3;
            sticky_pre_s3 = sticky_norm_s3;
            exp_pre_s3    = tent_exp_s3[7:0];
        end
    end

    logic round_up_s3;
    assign round_up_s3 = guard_pre_s3 & (round_pre_s3 | sticky_pre_s3 | mant24_pre_s3[0]);

    logic [24:0] mant25_s3;
    assign mant25_s3 = {1'b0, mant24_pre_s3} + {24'b0, round_up_s3};

    logic        carry_s3;
    logic [23:0] mant24_final_s3;
    assign carry_s3        = mant25_s3[24];
    assign mant24_final_s3 = carry_s3 ? mant25_s3[24:1] : mant25_s3[23:0];

    logic [8:0] exp_final9_s3;
    assign exp_final9_s3 = {1'b0, exp_pre_s3} + {8'b0, carry_s3};

    logic inexact_s3, underflow_s3, overflow_s3;
    assign inexact_s3   = guard_pre_s3 | round_pre_s3 | sticky_pre_s3 | pre_overflow_s3;
    assign underflow_s3 = is_subnormal_path_s3 && inexact_s3;
    assign overflow_s3  = pre_overflow_s3 || (exp_final9_s3 >= 9'd255);

    logic [31:0] computed_result_s3;
    logic [4:0]  computed_flags_s3;
    always_comb begin
        if (is_exact_zero_s2) begin
            // x - x (or 0+0/0-0 combos routed through the subtract path) => zero.
            // RNE convention: pure subtraction cancellation always yields +0;
            // same-sign addition of two zeros keeps the common sign.
            computed_result_s3 = {(eff_add_s2 ? sign_result_s2 : 1'b0), 31'h0};
            computed_flags_s3  = 5'b00000;
        end else if (overflow_s3) begin
            computed_result_s3 = {sign_result_s2, 8'hFF, 23'h0};
            computed_flags_s3  = {1'b0, 1'b0, 1'b1, 1'b0, 1'b1};
        end else begin
            computed_result_s3 = {sign_result_s2, exp_final9_s3[7:0], mant24_final_s3[22:0]};
            computed_flags_s3  = {1'b0, 1'b0, 1'b0, underflow_s3, inexact_s3};
        end
    end

    logic [31:0] result_comb_s3;
    logic [4:0]  flags_comb_s3;
    always_comb begin
        if (result_is_nan_s2) begin
            result_comb_s3 = 32'h7FC00000;
            flags_comb_s3  = {invalid_nan_in_s2, 4'b0000};
        end else if (result_is_inf_s2) begin
            result_comb_s3 = {inf_result_sign_s2, 8'hFF, 23'h0};
            flags_comb_s3  = 5'b00000;
        end else begin
            result_comb_s3 = computed_result_s3;
            flags_comb_s3  = computed_flags_s3;
        end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            valid_o  <= 1'b0;
            result_o <= 32'h0;
            flags_o  <= 5'h0;
        end else begin
            valid_o  <= valid_s2;
            result_o <= result_comb_s3;
            flags_o  <= flags_comb_s3;
        end
    end

    // =====================================================================
    // Bypass path: CMP / MIN / MAX / ABS / NEG / SEL (single-cycle latency)
    // =====================================================================

    // Comparator (uses ORIGINAL a_i, b_i — SUB's sign-flip does not apply
    // to CMP/MIN/MAX/SEL, only to the ADD/SUB pipeline).
    logic a_nan_c, b_nan_c, a_zero_c, b_zero_c;
    assign a_nan_c  = (a_i[30:23] == 8'hFF) && (a_i[22:0] != 0);
    assign b_nan_c  = (b_i[30:23] == 8'hFF) && (b_i[22:0] != 0);
    assign a_zero_c = (a_i[30:0] == 31'h0);
    assign b_zero_c = (b_i[30:0] == 31'h0);

    logic mag_lt_c, mag_eq_c;
    assign mag_lt_c = (a_i[30:0] < b_i[30:0]);
    assign mag_eq_c = (a_i[30:0] == b_i[30:0]);

    logic cmp_lt_comb, cmp_eq_comb;
    always_comb begin
        if (a_nan_c || b_nan_c) begin
            cmp_lt_comb = 1'b0;
            cmp_eq_comb = 1'b0;
        end else if (a_zero_c && b_zero_c) begin
            cmp_lt_comb = 1'b0;
            cmp_eq_comb = 1'b1;
        end else if (a_i[31] != b_i[31]) begin
            cmp_lt_comb = a_i[31];   // a negative, b positive => a < b
            cmp_eq_comb = 1'b0;
        end else if (a_i[31]) begin
            // both negative: larger magnitude bit-pattern => smaller value
            cmp_lt_comb = !mag_eq_c && !mag_lt_c;
            cmp_eq_comb = mag_eq_c;
        end else begin
            // both non-negative
            cmp_lt_comb = mag_lt_c;
            cmp_eq_comb = mag_eq_c;
        end
    end

    // MIN/MAX (IEEE-754-2008 minNum/maxNum: NaN operand ignored)
    //
    // NOTE (fixed during integration): the general-purpose cmp_lt_comb
    // treats +0.0 and -0.0 as equal (a_zero_c && b_zero_c -> cmp_eq_comb=1,
    // cmp_lt_comb=0), which is correct for FCMP but is NOT the rule MIN/MAX
    // need when magnitudes are equal: minNum/maxNum must still pick a
    // definite, sign-aware side between +0 and -0 (min prefers the negative
    // operand, max prefers the positive one). Reusing cmp_lt_comb directly
    // for the minmax mux silently fell through to "pick a" whenever
    // cmp_lt_comb was 0, which is right for ordinary a>b cases but wrong for
    // the equal-magnitude / signed-zero case specifically. Confirmed as a
    // real, live defect via cross-package integration testing (Agent 5's
    // PCPI gate, whose DPI reference model implements exactly the
    // sign-aware ±0 rule below): MAX(-0.0, +0.0) returned -0.0 instead of
    // +0.0. Fixed with an explicit equal-magnitude/signed-zero branch that
    // does not go through cmp_lt_comb at all.
    logic minmax_a_lt_b_comb;
    assign minmax_a_lt_b_comb = cmp_lt_comb; // same comparator, reused (non-zero-tie case only)

    logic [31:0] min_result_comb, max_result_comb;
    always_comb begin
        if (a_nan_c && b_nan_c) begin
            min_result_comb = 32'h7FC00000;
            max_result_comb = 32'h7FC00000;
        end else if (a_nan_c) begin
            min_result_comb = b_i;
            max_result_comb = b_i;
        end else if (b_nan_c) begin
            min_result_comb = a_i;
            max_result_comb = a_i;
        end else if (a_zero_c && b_zero_c) begin
            // equal magnitude (both zero): sign-aware tie-break, NOT cmp_lt_comb.
            min_result_comb = (a_i[31] || b_i[31]) ? (a_i[31] ? a_i : b_i) : a_i;
            max_result_comb = (!a_i[31] || !b_i[31]) ? (!a_i[31] ? a_i : b_i) : a_i;
        end else begin
            min_result_comb = minmax_a_lt_b_comb ? a_i : b_i;
            max_result_comb = minmax_a_lt_b_comb ? b_i : a_i;
        end
    end

    // ABS / NEG.
    //
    // NOTE (fixed during integration): this package's own ASSUMPTIONS.md
    // [A-ADD-2] argued that an unconditional bit-level sign manipulation
    // "is exactly what" doc19's "qNaN with sign flipped/0" requires for NaN
    // inputs, on the reasoning that flipping/clearing bit31 doesn't change
    // the exp=0xFF/frac!=0 bits that classify something as NaN. That
    // reasoning proves the result is *a* NaN, but not that it is *the*
    // canonical qNaN doc19 actually names: doc19's table (sec.1) says
    // "qNaN with sign bit 0" / "qNaN with sign bit flipped", i.e. the single
    // canonical bit pattern 0x7FC00000 with the sign bit set as described --
    // not "some NaN with the same payload as the input, sign-adjusted".
    // Any input NaN with a nonzero payload (signalling or quiet, non-
    // canonical) fails that reading under the old unconditional-bitflip
    // implementation, e.g. FNEG(0xFFD0FC80) produced 0x7FD0FC80 (still a
    // NaN, but not 0xFFC00000). Confirmed as a real, live defect via
    // cross-package integration testing (Agent 5's PCPI gate, whose DPI
    // reference model canonicalizes explicitly: `isnan_u(a) ? (QNAN ^ sign)
    // : (a ^ sign)`), which is also the more natural reading of doc19's own
    // wording. Fixed to canonicalize any NaN input to 0x7FC00000 before
    // applying the sign operation; non-NaN inputs are untouched bit
    // manipulation as before.
    logic [31:0] abs_result_comb, neg_result_comb;
    assign abs_result_comb = a_nan_c ? 32'h7FC00000 : {1'b0, a_i[30:0]};
    assign neg_result_comb = a_nan_c ? 32'hFFC00000 : {~a_i[31], a_i[30:0]};

    // cmp_lt_o / cmp_eq_o: held from the most recent CMP result.
    //
    // NOTE (fixed during integration): op 3'b111 was implemented here as a
    // "SEL" (conditional select: a if cmp_lt_o from a previous CMP, else b),
    // per this package's own doc/design/02_tide_fp32_add.md sec.4. That doc
    // predates and is superseded by the frozen machine-readable contract
    // (shared/contract/isa.yaml add_ops table), which assigns 3'b111 to
    // FMOV -- a plain single-operand passthrough (result = a, b unused) --
    // and by the real, G1-verified assembler/microcode (tools/tide_asm.py's
    // ADD_OPS = {..., 'FMOV': 7}, ADD_UNARY = {'FABS','FNEG','FMOV'}, used
    // live throughout microcode/10_unit_pre.tasm's MOV instructions). There
    // is no SEL concept anywhere in the actual assembler or microcode: a
    // grep of the whole real toolchain for "SEL" or "cmp_lt_o"-style
    // conditional-select usage returns nothing. Confirmed as a genuine
    // defect (not just a naming mismatch) by wiring this module into
    // Agent 5's real PCPI/SoC integration and rerunning its V2 gate: FMOV
    // (funct7=11) requests returned b instead of a on any vector where
    // cmp_lt_hold happened to be low, exactly matching the doc-conflict
    // above (see PROVENANCE.md). Fixed to implement literal FMOV per
    // isa.yaml/tide_asm.py, which both outrank the stale design doc per the
    // project's own contract precedence (Step 0 > frozen contract > design
    // docs). cmp_lt_hold/cmp_eq_hold and the cmp_lt_o/cmp_eq_o ports are
    // kept unchanged -- they are FCMP's own registered result and may still
    // be consumed elsewhere; only the former OP_SEL (now OP_MOV) bypass-mux
    // case is corrected.
    logic cmp_lt_hold, cmp_eq_hold;
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            cmp_lt_hold <= 1'b0;
            cmp_eq_hold <= 1'b0;
        end else if (valid_i && (op_i == OP_CMP)) begin
            cmp_lt_hold <= cmp_lt_comb;
            cmp_eq_hold <= cmp_eq_comb;
        end
    end
    assign cmp_lt_o = cmp_lt_hold;
    assign cmp_eq_o = cmp_eq_hold;

    // FMOV (op 3'b111, isa.yaml add_ops): pure passthrough of a_i, b_i unused.
    logic [31:0] fmov_result_comb;
    assign fmov_result_comb = a_i;

    // FCMP numeric result: 1 if a<b (ordered and strictly less), else 0.
    //
    // NOTE (fixed during integration): previously CMP produced a fixed 0
    // ("no numeric result defined") on the reasoning that the sequencer-side
    // microcode only consumes cmp_lt_o/cmp_eq_o as a side-channel and
    // defaults FCMP's rd field to the sink register (tools/tide_asm.py:
    // `if m == 'FCMP': ... rd, ra, rb = R_ZERO, ...` when rd is omitted).
    // That is true for the microcode path, but FCMP is also reachable via
    // the PCPI custom-instruction path (a RISC-V custom-1 instruction must
    // write *some* value to rd), which has its own, independently-checked
    // contract requiring the numeric result to carry the boolean comparison
    // outcome. Confirmed as a real, live defect via cross-package
    // integration testing (Agent 5's PCPI gate, whose DPI reference model
    // returns exactly `(!unordered && a<b) ? 1u : 0u`). Fixed to match;
    // this is additive only (a real, optional rd on the microcode side can
    // now legitimately capture the comparison outcome too, which is
    // consistent with -- not a violation of -- the sequencer-side contract).
    logic [31:0] cmp_result_comb;
    assign cmp_result_comb = {31'b0, cmp_lt_comb};

    logic [31:0] bypass_result_comb;
    always_comb begin
        unique case (op_i)
            OP_CMP: bypass_result_comb = cmp_result_comb;
            OP_MIN: bypass_result_comb = min_result_comb;
            OP_MAX: bypass_result_comb = max_result_comb;
            OP_ABS: bypass_result_comb = abs_result_comb;
            OP_NEG: bypass_result_comb = neg_result_comb;
            OP_MOV: bypass_result_comb = fmov_result_comb;
            default: bypass_result_comb = 32'h0;
        endcase
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            bypass_valid_o  <= 1'b0;
            bypass_result_o <= 32'h0;
        end else begin
            bypass_valid_o  <= valid_i && !is_pipelined_op;
            bypass_result_o <= bypass_result_comb;
        end
    end

endmodule
