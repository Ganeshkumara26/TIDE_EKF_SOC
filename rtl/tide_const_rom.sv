// -------------------------------------------------------------------
// tide_const_rom.sv — 64x32-bit combinational constant ROM
// Source spec: doc/design/05_tide_const_rom.md (fully specified there;
// this file reproduces it verbatim, no design decisions required).
// Owner: Agent 2
// -------------------------------------------------------------------
module tide_const_rom (
    input  logic [5:0]  addr_i,       // 6-bit address (0-63)
    output logic [31:0] data_o        // Constant value
);
    always_comb begin
        case (addr_i)
            6'd0:  data_o = 32'h40490FDB; // pi
            6'd1:  data_o = 32'h3FC90FDB; // pi/2
            6'd2:  data_o = 32'h40C90FDB; // 2*pi
            6'd3:  data_o = 32'hC0490FDB; // -pi
            6'd4:  data_o = 32'h3F800000; // 1.0
            6'd5:  data_o = 32'h00000000; // 0.0
            6'd6:  data_o = 32'hBF800000; // -1.0
            6'd7:  data_o = 32'h358637BD; // 1e-6
            6'd8:  data_o = 32'h3ED413CD; // tan(pi/8)
            6'd9:  data_o = 32'h401A827A; // cot(pi/8)
            6'd10: data_o = 32'h3F490FDB; // pi/4
            6'd11: data_o = 32'h3F7FFFF5; // atan_c0
            6'd12: data_o = 32'hBEAAA52E; // atan_c1
            6'd13: data_o = 32'h3E4C7B66; // atan_c2
            6'd14: data_o = 32'hBE084A28; // atan_c3
            6'd15: data_o = 32'h3727C5AC; // 1e-5 (floor)
            6'd16: data_o = 32'h322BCC77; // 1e-8
            6'd17: data_o = 32'h3F000000; // 0.5
            6'd18: data_o = 32'h40000000; // 2.0
            6'd19: data_o = 32'hC0000000; // -2.0
            default: data_o = 32'h00000000; // 20-63: reserved
        endcase
    end
endmodule
