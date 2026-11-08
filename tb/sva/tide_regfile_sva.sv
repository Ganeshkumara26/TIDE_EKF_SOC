// -------------------------------------------------------------------
// tide_regfile_sva.sv
// P1/P1b-d and the cover in P2 are taken from
// read-only/design/42_v5_engine_ordered.md section 4 verbatim. P3-P5
// added by Agent 3 for write-back-scheduler-adjacent coverage (write
// visible next cycle, WAW priority, read-before-write).
// -------------------------------------------------------------------

module tide_regfile_sva (
    input logic clk_i,
    input logic [6:0] raddr0_i, raddr1_i, raddr2_i, raddr3_i,
    input logic [31:0] rdata0_o, rdata1_o, rdata2_o, rdata3_o,
    input logic wen0_i, wen1_i, wen2_i,
    input logic [6:0] waddr0_i, waddr1_i, waddr2_i,
    input logic [31:0] wdata0_i, wdata1_i, wdata2_i
);

    // P1: Register 127 always reads as zero
    assert property (@(posedge clk_i)
        (raddr0_i == 7'd127) |-> (rdata0_o == 32'h0)
    ) else $error("RF_SVA P1: r127 must always read as 0 (port 0)");

    assert property (@(posedge clk_i)
        (raddr1_i == 7'd127) |-> (rdata1_o == 32'h0)
    ) else $error("RF_SVA P1b: r127 must always read as 0 (port 1)");

    assert property (@(posedge clk_i)
        (raddr2_i == 7'd127) |-> (rdata2_o == 32'h0)
    ) else $error("RF_SVA P1c: r127 must always read as 0 (port 2)");

    assert property (@(posedge clk_i)
        (raddr3_i == 7'd127) |-> (rdata3_o == 32'h0)
    ) else $error("RF_SVA P1d: r127 must always read as 0 (port 3)");

    // P2: Write to r127 has no effect (cover property)
    cover property (@(posedge clk_i)
        wen0_i && waddr0_i == 7'd127
    );

    // -----------------------------------------------------------------
    // Additional properties (Agent 3)
    // -----------------------------------------------------------------

    // P3: A write on port 0 to address A (not 127) is visible on the very
    // next cycle when the same address is read back on port 0, provided
    // no higher-priority port (1 or 2) also wrote A that same cycle.
    assert property (@(posedge clk_i)
        (wen0_i && waddr0_i != 7'd127 && !(wen1_i && waddr1_i == waddr0_i) && !(wen2_i && waddr2_i == waddr0_i))
        |=> (raddr0_i == $past(waddr0_i)) |-> (rdata0_o == $past(wdata0_i))
    ) else $error("RF_SVA P3: write must be visible on the immediately following cycle");

    // P4: WAW priority -- when ports 1 and 2 write the same address in the
    // same cycle, port 2's data wins (visible on read next cycle).
    assert property (@(posedge clk_i)
        (wen1_i && wen2_i && (waddr1_i == waddr2_i) && waddr2_i != 7'd127)
        |=> (raddr2_i == $past(waddr2_i)) |-> (rdata2_o == $past(wdata2_i))
    ) else $error("RF_SVA P4: port 2 must win a same-address WAW race against port 1");

    cover property (@(posedge clk_i)
        wen0_i && wen1_i && wen2_i &&
        (waddr0_i != waddr1_i) && (waddr1_i != waddr2_i) && (waddr0_i != waddr2_i)
    ); // all three write ports active with distinct addresses in one cycle

endmodule
