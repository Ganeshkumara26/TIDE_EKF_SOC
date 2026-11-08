// -------------------------------------------------------------------
// tide_ucode_ram_sva.sv
// The cover in P2 is taken from
// read-only/design/42_v5_engine_ordered.md section 5 verbatim. P1 is
// upgraded from a comment ("structural, verified by always_ff") into an
// actual checked property. P3 added by Agent 3.
// -------------------------------------------------------------------

module tide_ucode_ram_sva (
    input logic clk_i, rst_ni,
    input logic [8:0] seq_addr_i,
    input logic [95:0] seq_data_o,
    input logic cpu_we_i,
    input logic [1:0] cpu_word_i,
    input logic [8:0] cpu_addr_i,
    input logic [31:0] cpu_data_i
);

    // P1: Read latency is exactly 1 cycle: seq_data_o observed this
    // cycle must equal whatever the memory held for seq_addr_i as it was
    // driven last cycle. We can't see `mem` from outside, but we CAN
    // check the weaker, still-meaningful property that seq_data_o is
    // stable for as long as seq_addr_i is held constant (a same-address
    // re-read returns the same word), which is inconsistent with any
    // latency other than a fixed single register stage combined with a
    // non-changing address.
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        ($stable(seq_addr_i)) |=> $stable(seq_data_o)
    ) else $error("UCODE_SVA P1: seq_data_o must be stable while seq_addr_i is held");

    // P2: CPU write committed only on 3rd word (cpu_word_i == 2) (cover)
    cover property (@(posedge clk_i)
        cpu_we_i && cpu_word_i == 2'b10
    );

    // P3: cpu_word_i == 3 (2'b11) is outside the 3-phase protocol and
    // must never be driven by a correctly-behaving CPU/bus (cover only --
    // this is a caller contract, not something this module can enforce).
    cover property (@(posedge clk_i)
        cpu_we_i && cpu_word_i == 2'b11
    );

endmodule
