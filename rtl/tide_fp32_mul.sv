// -------------------------------------------------------------------
// tide_fp32_mul.sv — IEEE-754 binary32 pipelined multiplier
// Source spec: doc/design/01_tide_fp32_mul.md
// Owner: Agent 2
//
// 2-stage pipeline, fully pipelined, 1 op/cycle throughput, no stalling.
//   Stage 1 (posedge N   -> regs _s1_) : unpack, classify specials,
//                                        24x24 mantissa multiply
//   Stage 2 (posedge N+1 -> result_o)  : normalize (LZC + shift),
//                                        round (RNE), pack, specials mux
//
// result_o/valid_o are valid on cycle N+2 for an input accepted on cycle N.
// -------------------------------------------------------------------
module tide_fp32_mul (
    input  logic        clk_i,
    input  logic        rst_ni,

    input  logic        valid_i,      // New operands available
    input  logic [31:0] a_i,          // Operand A (IEEE-754 binary32)
    input  logic [31:0] b_i,          // Operand B (IEEE-754 binary32)

    output logic        valid_o,      // Result available (2 cycles after valid_i)
    output logic [31:0] result_o,     // IEEE-754 binary32 result
    output logic [4:0]  flags_o       // {invalid, div_by_zero(0), overflow, underflow, inexact}
);

    // =====================================================================
    // Stage 1: unpack, classify, multiply, compress
    // =====================================================================
    logic        sign_a, sign_b;
    logic [7:0]  exp_a_raw, exp_b_raw;
    logic [22:0] frac_a, frac_b;
    logic        a_is_zero, b_is_zero;
    logic        a_is_inf,  b_is_inf;
    logic        a_is_nan,  b_is_nan;
    logic        a_is_snan, b_is_snan;
    logic        a_is_sub,  b_is_sub;
    logic [23:0] mant_a, mant_b;   // implicit-bit-included (0 for subnormal)
    logic [7:0]  exp_a,  exp_b;    // effective exponent (forced to 1 for subnormal)

    always_comb begin
        sign_a    = a_i[31];
        sign_b    = b_i[31];
        exp_a_raw = a_i[30:23];
        exp_b_raw = b_i[30:23];
        frac_a    = a_i[22:0];
        frac_b    = b_i[22:0];

        a_is_zero = (exp_a_raw == 8'h00) && (frac_a == 23'h0);
        b_is_zero = (exp_b_raw == 8'h00) && (frac_b == 23'h0);
        a_is_inf  = (exp_a_raw == 8'hFF) && (frac_a == 23'h0);
        b_is_inf  = (exp_b_raw == 8'hFF) && (frac_b == 23'h0);
        a_is_nan  = (exp_a_raw == 8'hFF) && (frac_a != 23'h0);
        b_is_nan  = (exp_b_raw == 8'hFF) && (frac_b != 23'h0);
        a_is_snan = a_is_nan && !frac_a[22];   // MSB of fraction clear => signaling
        b_is_snan = b_is_nan && !frac_b[22];
        a_is_sub  = (exp_a_raw == 8'h00) && (frac_a != 23'h0);
        b_is_sub  = (exp_b_raw == 8'h00) && (frac_b != 23'h0);

        mant_a = a_is_sub ? {1'b0, frac_a} : {1'b1, frac_a};
        mant_b = b_is_sub ? {1'b0, frac_b} : {1'b1, frac_b};
        exp_a  = a_is_sub ? 8'h01 : exp_a_raw;
        exp_b  = b_is_sub ? 8'h01 : exp_b_raw;
    end

    logic sign_r_s0;
    assign sign_r_s0 = sign_a ^ sign_b;

    // Biased exponent sum, wide signed to avoid overflow (range roughly -253..382)
    logic signed [10:0] exp_sum_s0;
    assign exp_sum_s0 = $signed({3'b000, exp_a}) + $signed({3'b000, exp_b}) - 11'sd127;

    // 24x24 -> 48 mantissa multiply.
    // SYNTHESIS: verify timing. First-pass behavioral multiply; replace with
    // explicit Booth radix-4 + CSA tree if this does not close at 50 MHz in
    // SCL 180nm (see design doc §8.1).
    logic [47:0] prod_s0;
    assign prod_s0 = mant_a * mant_b;

    // Special-case classification
    logic any_nan_s0, invalid_s0;
    logic result_is_nan_s0, result_is_inf_s0, result_is_zero_s0;
    always_comb begin
        any_nan_s0       = a_is_nan || b_is_nan;
        invalid_s0       = a_is_snan || b_is_snan ||
                            (a_is_inf && b_is_zero) || (a_is_zero && b_is_inf);
        result_is_nan_s0 = any_nan_s0 || (a_is_inf && b_is_zero) || (a_is_zero && b_is_inf);
        result_is_inf_s0 = !result_is_nan_s0 && (a_is_inf || b_is_inf);
        result_is_zero_s0= !result_is_nan_s0 && !result_is_inf_s0 && (a_is_zero || b_is_zero);
    end

    // ---- Stage 1 -> Stage 2 pipeline registers ----
    logic               valid_s1;
    logic               sign_r_s1;
    logic signed [10:0] exp_sum_s1;
    logic [47:0]        prod_s1;
    logic               result_is_nan_s1, result_is_inf_s1, result_is_zero_s1, invalid_s1;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            valid_s1          <= 1'b0;
            sign_r_s1         <= 1'b0;
            exp_sum_s1        <= '0;
            prod_s1           <= '0;
            result_is_nan_s1  <= 1'b0;
            result_is_inf_s1  <= 1'b0;
            result_is_zero_s1 <= 1'b0;
            invalid_s1        <= 1'b0;
        end else begin
            valid_s1          <= valid_i;
            sign_r_s1         <= sign_r_s0;
            exp_sum_s1        <= exp_sum_s0;
            prod_s1           <= prod_s0;
            result_is_nan_s1  <= result_is_nan_s0;
            result_is_inf_s1  <= result_is_inf_s0;
            result_is_zero_s1 <= result_is_zero_s0;
            invalid_s1        <= invalid_s0;
        end
    end

    // =====================================================================
    // Stage 2: normalize, round (RNE), pack
    // =====================================================================

    // Leading-zero count of the 48-bit product (0 if prod_s1==0, but that
    // case is masked off by result_is_zero_s1 so it never reaches packing).
    function automatic [5:0] lzc48(input logic [47:0] v);
        integer i;
        begin
            lzc48 = 6'd48;
            for (i = 0; i < 48; i = i + 1) begin
                if (v[47-i] && (lzc48 == 6'd48)) lzc48 = i[5:0];
            end
        end
    endfunction

    logic [5:0]  nlz_s2;
    logic [47:0] norm48_s2;
    assign nlz_s2    = lzc48(prod_s1);
    assign norm48_s2 = prod_s1 << nlz_s2;

    logic [23:0] mant24_norm_s2;
    logic        guard_norm_s2, round_norm_s2, sticky_norm_s2;
    assign mant24_norm_s2 = norm48_s2[47:24];
    assign guard_norm_s2  = norm48_s2[23];
    assign round_norm_s2  = norm48_s2[22];
    assign sticky_norm_s2 = |norm48_s2[21:0];

    // tentative_biased_exp = k + exp_a + exp_b - 173, where k = 47-nlz,
    // folded into exp_sum_s1 (= exp_a+exp_b-127): tentative = exp_sum_s1 + (47-nlz) - 46
    //   = exp_sum_s1 + 1 - nlz
    logic signed [11:0] tent_exp_s2;
    assign tent_exp_s2 = $signed({exp_sum_s1[10], exp_sum_s1}) + 12'sd1 - {6'b0, nlz_s2};

    // Pre-overflow / pre-underflow classification (before rounding)
    logic pre_overflow_s2;
    assign pre_overflow_s2 = (tent_exp_s2 >= 12'sd255);

    // Subnormal alignment: if tent_exp <= 0, shift right by (1 - tent_exp)
    logic signed [11:0] subshift_raw_s2;
    logic [6:0]          subshift_s2;   // clamped shift amount (0..38 enough for 26-bit datapath)
    assign subshift_raw_s2 = 12'sd1 - tent_exp_s2;
    assign subshift_s2 = (tent_exp_s2 > 12'sd0) ? 7'd0 :
                          (subshift_raw_s2 > 12'sd38) ? 7'd38 : subshift_raw_s2[6:0];

    logic is_subnormal_path_s2;
    assign is_subnormal_path_s2 = (tent_exp_s2 <= 12'sd0) && !pre_overflow_s2;

    // Combined mantissa+guard+round (26 bits) for the subnormal shifter
    logic [25:0] wide_norm_s2;
    assign wide_norm_s2 = {mant24_norm_s2, guard_norm_s2, round_norm_s2};

    logic [25:0] wide_shifted_s2;
    logic        sub_extra_sticky_s2;
    assign wide_shifted_s2 = wide_norm_s2 >> subshift_s2;
    // Any 1-bit shifted out of the 26-bit window, or original sticky, feeds sticky.
    assign sub_extra_sticky_s2 =
        (subshift_s2 == 7'd0) ? 1'b0 :
        (|(wide_norm_s2 & ~({26{1'b1}} << subshift_s2)));

    logic [23:0] mant24_pre_s2;
    logic        guard_pre_s2, round_pre_s2, sticky_pre_s2;
    logic [7:0]  exp_pre_s2;

    always_comb begin
        if (is_subnormal_path_s2) begin
            mant24_pre_s2 = wide_shifted_s2[25:2];
            guard_pre_s2  = wide_shifted_s2[1];
            round_pre_s2  = wide_shifted_s2[0];
            sticky_pre_s2 = sticky_norm_s2 | sub_extra_sticky_s2;
            exp_pre_s2    = 8'h00;
        end else begin
            mant24_pre_s2 = mant24_norm_s2;
            guard_pre_s2  = guard_norm_s2;
            round_pre_s2  = round_norm_s2;
            sticky_pre_s2 = sticky_norm_s2;
            exp_pre_s2    = tent_exp_s2[7:0];
        end
    end

    // RNE rounding decision
    logic round_up_s2;
    assign round_up_s2 = guard_pre_s2 & (round_pre_s2 | sticky_pre_s2 | mant24_pre_s2[0]);

    logic [24:0] mant25_s2;
    assign mant25_s2 = {1'b0, mant24_pre_s2} + {24'b0, round_up_s2};

    logic        carry_s2;
    logic [23:0] mant24_final_s2;
    assign carry_s2 = mant25_s2[24];
    assign mant24_final_s2 = carry_s2 ? mant25_s2[24:1] : mant25_s2[23:0];

    logic [8:0] exp_final9_s2; // extra bit to catch overflow into 255+
    assign exp_final9_s2 = {1'b0, exp_pre_s2} + {8'b0, carry_s2};

    logic inexact_s2, underflow_s2, overflow_s2;
    assign inexact_s2   = guard_pre_s2 | round_pre_s2 | sticky_pre_s2 | pre_overflow_s2;
    assign underflow_s2 = is_subnormal_path_s2 && inexact_s2;
    assign overflow_s2  = pre_overflow_s2 || (exp_final9_s2 >= 9'd255);

    // Final packed (non-special) result
    logic [31:0] computed_result_s2;
    logic [4:0]  computed_flags_s2;
    always_comb begin
        if (overflow_s2) begin
            computed_result_s2 = {sign_r_s1, 8'hFF, 23'h0};                 // ±Inf
            computed_flags_s2  = {1'b0, 1'b0, 1'b1, 1'b0, 1'b1};            // overflow, inexact
        end else begin
            computed_result_s2 = {sign_r_s1, exp_final9_s2[7:0], mant24_final_s2[22:0]};
            computed_flags_s2  = {1'b0, 1'b0, 1'b0, underflow_s2, inexact_s2};
        end
    end

    // ---- Specials mux (NaN / Inf / Zero override everything above) ----
    logic [31:0] result_comb_s2;
    logic [4:0]  flags_comb_s2;
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

    // ---- Output register ----
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            valid_o  <= 1'b0;
            result_o <= 32'h0;
            flags_o  <= 5'h0;
        end else begin
            valid_o  <= valid_s1;
            result_o <= result_comb_s2;
            flags_o  <= flags_comb_s2;
        end
    end

endmodule
