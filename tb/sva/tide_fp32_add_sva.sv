// FILE: sva/tide_fp32_add_sva.sv
// Bind: bind tide_fp32_add tide_fp32_add_sva u_sva (.*);
// Source: doc/design/40_v1_fp_lanes_verification.md section 3 (reproduced verbatim;
// matches the tide_fp32_add.sv port list exactly).

module tide_fp32_add_sva (
    input logic clk_i, rst_ni,
    input logic valid_i, valid_o,
    input logic [2:0] op_i,
    input logic [31:0] a_i, b_i, result_o,
    input logic bypass_valid_o,
    input logic [31:0] bypass_result_o,
    input logic cmp_lt_o,
    input logic [4:0] flags_o
);

    // P1: ADD/SUB pipeline timing — valid_o follows valid_i by 3 cycles
    logic valid_d1, valid_d2, valid_d3;
    logic is_pipelined;
    assign is_pipelined = (op_i == 3'b000) || (op_i == 3'b001); // ADD or SUB

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            valid_d1 <= 0; valid_d2 <= 0; valid_d3 <= 0;
        end else begin
            valid_d1 <= valid_i && is_pipelined;
            valid_d2 <= valid_d1;
            valid_d3 <= valid_d2;
        end
    end

    assert property (@(posedge clk_i) disable iff (!rst_ni)
        (valid_i && is_pipelined) |-> ##3 valid_o
    ) else $error("ADD_SVA P1: ADD/SUB valid_o must follow valid_i by 3 cycles");

    // P2: Bypass timing — CMP/ABS/NEG/MIN/MAX produce result in 1 cycle
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        (valid_i && !is_pipelined) |-> ##1 bypass_valid_o
    ) else $error("ADD_SVA P2: bypass ops must produce result in 1 cycle");

    // P3: +Inf + (-Inf) = NaN
    wire a_pos_inf = (a_i == 32'h7F800000);
    wire b_neg_inf = (b_i == 32'hFF800000);
    wire a_neg_inf = (a_i == 32'hFF800000);
    wire b_pos_inf = (b_i == 32'h7F800000);

    assert property (@(posedge clk_i) disable iff (!rst_ni)
        (valid_i && op_i == 3'b000 && a_pos_inf && b_neg_inf) |-> ##3
            (result_o == 32'h7FC00000 && flags_o[4] == 1'b1)
    ) else $error("ADD_SVA P3: +Inf + (-Inf) must be NaN");

    // P4: ABS clears sign bit
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        (valid_i && op_i == 3'b101) |-> ##1 (bypass_result_o[31] == 1'b0)
    ) else $error("ADD_SVA P4: ABS must clear sign bit");

    // P5: NEG flips sign bit
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        (valid_i && op_i == 3'b110) |-> ##1 (bypass_result_o[31] == ~a_i[31])
    ) else $error("ADD_SVA P5: NEG must flip sign bit");

    // P6: +0 == -0 for CMP
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        (valid_i && op_i == 3'b010 && a_i == 32'h00000000 && b_i == 32'h80000000)
        |-> ##1 (cmp_lt_o == 1'b0)
    ) else $error("ADD_SVA P6: +0 and -0 must compare equal (neither is less than)");

endmodule
