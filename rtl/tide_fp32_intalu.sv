// -------------------------------------------------------------------
// tide_fp32_intalu.sv — Integer ALU + ITOF (int32 -> binary32, RNE)
// Source spec: doc/design/04_tide_fp32_intalu.md
// Owner: Agent 2
//
// Single-cycle latency, fully combinational datapath with a registered
// output (1 op/cycle throughput, no pipelining needed for this unit).
// -------------------------------------------------------------------
module tide_fp32_intalu (
    input  logic        clk_i,
    input  logic        rst_ni,

    input  logic        valid_i,
    input  logic [3:0]  op_i,
    input  logic [31:0] a_i,
    input  logic [31:0] b_i,
    input  logic [12:0] imm_i,        // 13-bit immediate (IADD/IMOV)

    output logic        valid_o,
    output logic [31:0] result_o,
    output logic        cmp_result_o,
    output logic        zero_o
);

    localparam logic [3:0] OP_IADD   = 4'b0100;
    localparam logic [3:0] OP_ISUB   = 4'b0101;
    localparam logic [3:0] OP_ICMP   = 4'b0110;
    localparam logic [3:0] OP_ITOF   = 4'b0111;
    localparam logic [3:0] OP_LDLOOP = 4'b1000;
    localparam logic [3:0] OP_LDIDX0 = 4'b1001;
    localparam logic [3:0] OP_LDIDX1 = 4'b1010;
    localparam logic [3:0] OP_IMOV   = 4'b1011;

    // Sign-extended 13-bit immediate
    logic [31:0] imm_sext;
    assign imm_sext = {{19{imm_i[12]}}, imm_i};

    // ---- ITOF (int32 -> binary32, round-to-nearest-even) ----
    logic        itof_sign;
    logic [31:0] itof_abs;
    assign itof_sign = a_i[31];
    assign itof_abs  = itof_sign ? (~a_i + 32'h1) : a_i; // two's-complement negate

    function automatic [5:0] clz32(input logic [31:0] v);
        integer i;
        begin
            clz32 = 6'd32;
            for (i = 0; i < 32; i = i + 1) begin
                if (v[31-i] && (clz32 == 6'd32)) clz32 = i[5:0];
            end
        end
    endfunction

    logic [5:0]  itof_lz;
    logic [31:0] itof_shifted;
    assign itof_lz      = clz32(itof_abs);
    assign itof_shifted = itof_abs << itof_lz;

    logic [8:0]  itof_exp_pre; // 158-lz, fits comfortably in 9 bits
    assign itof_exp_pre = 9'd158 - {3'b0, itof_lz};

    logic [22:0] itof_mant_pre;
    logic        itof_guard, itof_round, itof_sticky;
    assign itof_mant_pre = itof_shifted[30:8];
    assign itof_guard    = itof_shifted[7];
    assign itof_round    = itof_shifted[6];
    assign itof_sticky   = |itof_shifted[5:0];

    logic itof_round_up;
    assign itof_round_up = itof_guard & (itof_round | itof_sticky | itof_mant_pre[0]);

    logic [23:0] itof_mant24;
    assign itof_mant24 = {1'b0, itof_mant_pre} + {23'b0, itof_round_up};

    logic        itof_carry;
    logic [22:0] itof_mant_final;
    logic [8:0]  itof_exp_final;
    assign itof_carry      = itof_mant24[23];
    assign itof_mant_final = itof_carry ? 23'h0 : itof_mant24[22:0]; // carry: 0xFFFFFF+1 -> mant=0
    assign itof_exp_final  = itof_exp_pre + {8'b0, itof_carry};

    logic [31:0] itof_result;
    assign itof_result = (a_i == 32'h0) ? 32'h0
                                         : {itof_sign, itof_exp_final[7:0], itof_mant_final};

    // ---- Comparator (signed A < B) ----
    logic cmp_lt_comb;
    assign cmp_lt_comb = ($signed(a_i) < $signed(b_i));

    // ---- Result mux ----
    logic [31:0] result_comb;
    always_comb begin
        unique case (op_i)
            OP_IADD:   result_comb = a_i + imm_sext;
            OP_ISUB:   result_comb = a_i - b_i;
            OP_ICMP:   result_comb = {31'b0, cmp_lt_comb}; // no numeric result defined; see ASSUMPTIONS.md
            OP_ITOF:   result_comb = itof_result;
            OP_IMOV:   result_comb = imm_sext;
            OP_LDLOOP,
            OP_LDIDX0,
            OP_LDIDX1: result_comb = a_i; // passthrough; actual load handled by Sequencer (see ASSUMPTIONS.md)
            default:   result_comb = 32'h0;
        endcase
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            valid_o      <= 1'b0;
            result_o     <= 32'h0;
            cmp_result_o <= 1'b0;
            zero_o       <= 1'b0;
        end else begin
            valid_o      <= valid_i;
            result_o     <= result_comb;
            cmp_result_o <= cmp_lt_comb;
            zero_o       <= (result_comb == 32'h0);
        end
    end

endmodule
