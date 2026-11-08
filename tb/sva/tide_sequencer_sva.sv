// -------------------------------------------------------------------
// tide_sequencer_sva.sv
// Base properties P1-P7 are taken verbatim (signal names aside) from
// read-only/design/42_v5_engine_ordered.md section 3. P8-P11 are
// additional properties added for Agent 3's own coverage of the
// sequencer's write-back scheduler and branch squash, per README.md
// "Deliverables" (V5 SVA test report).
//
// Bind this module against tide_sequencer using a `bind` statement in
// the testbench (see tb/tb_tide_engine.sv) so it can observe internal
// signals (fetch_pc_q, dreg_q-derived fields) without adding debug ports
// to the frozen tide_sequencer_if.
// -------------------------------------------------------------------

module tide_sequencer_sva (
    input logic clk_i, rst_ni,
    input logic start_i, done_o, running_o, boundary_o,
    input logic [8:0] ucode_addr_o,
    // internal signals bound in for P8-P11
    input logic mul_valid_i, add_valid_i, add_bypass_valid_i, ds_done_i,
    input logic rf_wen0_o, rf_wen1_o,
    input logic lsu_cmd_valid_o, lsu_cmd_ready_i,
    input logic ds_busy_i, ds_valid_o
);

    // P1: Sequencer is idle on reset
    assert property (@(posedge clk_i)
        !rst_ni |=> (!running_o && !done_o)
    ) else $error("SEQ_SVA P1: sequencer must be idle after reset");

    // P2: start_i causes running_o to assert
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        (start_i && !running_o) |=> running_o
    ) else $error("SEQ_SVA P2: start must cause running");

    // P3: done_o is a single-cycle pulse
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        done_o |=> !done_o
    ) else $error("SEQ_SVA P3: done_o must be a single-cycle pulse");

    // P4: done_o causes running_o to deassert
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        done_o |=> !running_o
    ) else $error("SEQ_SVA P4: done must stop running");

    // P5: PC stays in range [0, 511]
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        ucode_addr_o < 9'd512
    ) else $error("SEQ_SVA P5: PC out of range");

    // P6: boundary_o only when running
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        boundary_o |-> running_o
    ) else $error("SEQ_SVA P6: boundary only when running");

    // P7: After boundary, sequencer pauses until re-started
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        boundary_o |=> (!running_o || start_i)
    ) else $error("SEQ_SVA P7: boundary must pause sequencer");

    // -----------------------------------------------------------------
    // Additional properties (Agent 3)
    // -----------------------------------------------------------------

    // P8: done_o and boundary_o are mutually exclusive (timing.yaml
    // seq_tc_interface.mutual_exclusion)
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        !(done_o && boundary_o)
    ) else $error("SEQ_SVA P8: done_o and boundary_o must be mutually exclusive");

    // P9: at most one of the three port-1 sources fires in any cycle
    // (STEP_0 decision C-3: assembler must never schedule two on the
    // same cycle)
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        $onehot0({add_valid_i, add_bypass_valid_i, ds_done_i})
    ) else $error("SEQ_SVA P9: port-1 write sources must not collide");

    // P10: rf_wen0_o (MUL write) only ever follows a real mul_valid_i pulse
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        rf_wen0_o |-> mul_valid_i
    ) else $error("SEQ_SVA P10: port-0 write must be driven by mul_valid_i");

    // P11: an LSU command that isn't accepted must be re-asserted next
    // cycle (no dropped/silently-lost LSU commands during a stall) --
    // design doc 08 section 7 stall condition.
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        (lsu_cmd_valid_o && !lsu_cmd_ready_i) |=> lsu_cmd_valid_o
    ) else $error("SEQ_SVA P11: stalled LSU command must be retried, not dropped");

    // P12: DIV/SQRT exclusivity (REQ-A-03 / ISA ds_coupling.exclusivity)
    // -- a new DS issue is never asserted while the lane reports busy.
    assert property (@(posedge clk_i) disable iff (!rst_ni)
        ds_busy_i |-> !ds_valid_o
    ) else $error("SEQ_SVA P12: must not issue a new DIV/SQRT while lane is busy");

    // Cover: DS write-back actually happens at least once in a run
    // (sanity, not a functional requirement)
    cover property (@(posedge clk_i) disable iff (!rst_ni) ds_done_i);

endmodule
