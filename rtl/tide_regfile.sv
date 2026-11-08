// -------------------------------------------------------------------
// tide_regfile.sv — 128 x 32, 4-Read / 3-Write Register File
//
// Contract: shared/contract/tide_regfile.sv (tide_regfile_if) — port list is
// frozen and matched exactly below.
// Design ref: read-only/design/06_tide_regfile.md
//
// Behavior (frozen by contract + design doc 06):
//   - Reads are combinational: raddrN_i presented on cycle N -> rdataN_o
//     valid same cycle (no clock edge needed).
//   - Writes are synchronous on posedge clk_i when wenX_i==1. Data written
//     on cycle N is visible to reads starting cycle N+1.
//   - Register 127 (7'h7F) is hardwired zero: reads always return 0,
//     writes to it are silently discarded.
//   - Write-write conflict (two ports write same address, same cycle) is
//     illegal per the assembler contract (WAW hazard), but if it occurs the
//     hardware resolves it deterministically: Port 2 > Port 1 > Port 0
//     (highest port number wins), implemented simply by ordering the
//     non-blocking assignments Port0 -> Port1 -> Port2 so a later
//     assignment overwrites an earlier one to the same array element.
//   - Read/write same address, same cycle: read returns the OLD value
//     (read-before-write). This falls out naturally: the read is
//     assign-based combinational logic that samples the current flop
//     contents, which only update after the posedge.
//
// Registers 125 (7'h7D, idx1) and 126 (7'h7E, idx0) are dropped from the
// general RF map per STEP_0_DECISIONS (ISA-05). Per ASSUMPTIONS.md [A1],
// the substitution of idx0/idx1 values for addresses 0x7E/0x7D, and the
// discarding of writes to those two addresses, is implemented in
// tide_sequencer.sv (which owns the idx0/idx1 flops) rather than here,
// because the frozen tide_regfile_if has no idx0/idx1 ports. This module
// is therefore left exactly as specified in design doc 06: only address
// 127 is special-cased; addresses 125/126 of the underlying array are
// simply never presented to this module's ports (the sequencer intercepts
// them before they reach raddrX_i/waddrX_i).
// -------------------------------------------------------------------

module tide_regfile (
    input  logic        clk_i,
    input  logic        rst_ni,       // Reset clears valid_o signals only (no register state reset guarantee is made; see ASSUMPTIONS.md [A2])

    // Read port 0 (MUL lane, operand A)
    input  logic [6:0]  raddr0_i,
    output logic [31:0] rdata0_o,

    // Read port 1 (MUL lane, operand B)
    input  logic [6:0]  raddr1_i,
    output logic [31:0] rdata1_o,

    // Read port 2 (ADD lane, operand A — shared with LSU)
    input  logic [6:0]  raddr2_i,
    output logic [31:0] rdata2_o,

    // Read port 3 (ADD lane, operand B — shared with LSU)
    input  logic [6:0]  raddr3_i,
    output logic [31:0] rdata3_o,

    // Write port 0 (MUL lane result)
    input  logic        wen0_i,
    input  logic [6:0]  waddr0_i,
    input  logic [31:0] wdata0_i,

    // Write port 1 (ADD lane result or DIV/SQRT)
    input  logic        wen1_i,
    input  logic [6:0]  waddr1_i,
    input  logic [31:0] wdata1_i,

    // Write port 2 (LSU load, IntALU, Const ROM)
    input  logic        wen2_i,
    input  logic [6:0]  waddr2_i,
    input  logic [31:0] wdata2_i
);

    // Storage: 128 x 32-bit flops
    logic [31:0] rf [0:127];

    // Reads (combinational, r127 hardwired to zero)
    assign rdata0_o = (raddr0_i == 7'd127) ? 32'h0 : rf[raddr0_i];
    assign rdata1_o = (raddr1_i == 7'd127) ? 32'h0 : rf[raddr1_i];
    assign rdata2_o = (raddr2_i == 7'd127) ? 32'h0 : rf[raddr2_i];
    assign rdata3_o = (raddr3_i == 7'd127) ? 32'h0 : rf[raddr3_i];

    // Writes (synchronous). Ordering Port0 -> Port1 -> Port2 gives the
    // required "highest port wins" WAW resolution since a later
    // non-blocking assignment to the same array element in the same
    // always_ff block wins.
    always_ff @(posedge clk_i) begin
        if (wen0_i && waddr0_i != 7'd127)
            rf[waddr0_i] <= wdata0_i;
        if (wen1_i && waddr1_i != 7'd127)
            rf[waddr1_i] <= wdata1_i;  // Port 1 overwrites port 0 if same address
        if (wen2_i && waddr2_i != 7'd127)
            rf[waddr2_i] <= wdata2_i;  // Port 2 overwrites port 1/0
    end

endmodule
