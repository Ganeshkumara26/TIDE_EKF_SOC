// -----------------------------------------------------------------------------
// tide_temporal_sva.sv - SVA-equivalent properties for V7 (doc 44), scoped to Agent 4's
// RTL: tide_temporal (checkpoint ring, journal, commit atomicity, counters, reset),
// tide_event_fifo, and tide_ram (bound in one file since all three are small and all
// owned by this agent; see the three `bind` statements at the end of each section).
//
// Tooling note (see ASSUMPTIONS A10): this environment's `yosys` build has no Verific
// plugin, so full concurrent-SVA syntax (`assert property (@(posedge clk) disable iff
// (...) ...)`) is not parsable by its built-in frontend -- confirmed directly (a one-line
// `assert property(...)` fails with "syntax error, unexpected '@'" even in isolation).
// This is the standard limitation of the open-source (non-Verific) flow, and the
// standard workaround -- used throughout YosysHQ's own SymbiYosys examples -- is
// immediate assertions inside a clocked `always` block, using $past/$rose/$fell/$stable.
// This is semantically equivalent for BMC/k-induction purposes; only the surface syntax
// differs from doc 44's sketch.
//
// F17-F21 (PCPI/engine lane exclusivity) belong to tide_sequencer/tide_pcpi/tide_fp_ctrl,
// which are not part of this package (see ASSUMPTIONS A1) -- omitted here, not faked.
// F11 ("total_events_submitted == units+drop+fifo+processing") needs a top-level event
// counter that lives in tide_top (Agent 5); the FIFO-local and controller-local pieces of
// that invariant (F12-style monotonicity, FIFO occupancy bookkeeping) ARE checked below.
//
// Each checker is a small module bound (via `bind`) into the DUT so its ports can be
// connected implicitly (.*) to internal signals of the same name -- the standard
// bind-in-a-checker pattern, avoiding any modification of the RTL under test.
// -----------------------------------------------------------------------------

// ============================================================================
// SECTION 1+2+3: checkpoint ring, journal sortedness, commit atomicity
// (bound into tide_temporal; ports named to match its internal registers)
// ============================================================================
module formal_temporal_checks (
    input logic        clk_i,
    input logic        rst_ni,

    input logic [2:0]  ck_head,
    input logic [3:0]  ck_cnt,
    // CKPT_DEPTH is fixed at 8 below (ck_t0..ck_t7): yosys's built-in (non-Verific)
    // frontend does not accept unpacked-array ports, so the ring-order check is unrolled
    // explicitly instead of using a parametric array port.
    input logic [31:0] ck_t0, ck_t1, ck_t2, ck_t3, ck_t4, ck_t5, ck_t6, ck_t7,

    input logic [15:0] jhead,
    input logic [15:0] jtail,

    input logic        com_sel,
    input logic        rpl_active,
    input logic [5:0]  state,
    input logic [31:0] chain_t,
    input logic [31:0] t_now,

    input logic [31:0] cnt_units_o,
    input logic [31:0] cnt_drop_o,
    input logic [31:0] cnt_late_o,
    input logic [31:0] cnt_replay_o,
    input logic [31:0] cnt_query_o,
    input logic        busy_o,
    input logic        replay_pending_o,
    input logic        provisional_o
);
    localparam [5:0] S_COMMIT_B = 6'd34;
    localparam [5:0] S_IO_PK2   = 6'd18;
    localparam [5:0] S_LATE_C   = 6'd28;

    wire [31:0] ck_t [0:7];
    assign ck_t[0] = ck_t0; assign ck_t[1] = ck_t1; assign ck_t[2] = ck_t2; assign ck_t[3] = ck_t3;
    assign ck_t[4] = ck_t4; assign ck_t[5] = ck_t5; assign ck_t[6] = ck_t6; assign ck_t[7] = ck_t7;

    // F4: occupied checkpoint slots are strictly time-ordered, walking from the oldest
    // (ck_head - ck_cnt) to the newest (ck_head - 1). Unrolled for CKPT_DEPTH=8.
    genvar gi;
    generate
        for (gi = 0; gi < 7; gi = gi + 1) begin : gen_ckpt_order
            wire [2:0] oldest = ck_head - ck_cnt[2:0];
            wire [2:0] slot_a = oldest + gi[2:0];
            wire [2:0] slot_b = oldest + gi[2:0] + 3'd1;
            wire       both_occ = (gi + 1 < {28'd0, ck_cnt});
            always @(posedge clk_i) if (rst_ni) begin
                // Timestamps wrap (32-bit, per STEP_0 ISA-03/A5 in ASSUMPTIONS), so
                // ordering must be a signed DIFFERENCE, not a raw signed-magnitude
                // compare -- the same convention the RTL itself uses everywhere
                // (diff_pos, too_old, the insertion-sort compare). A magnitude compare
                // breaks exactly at the wrap boundary; confirmed by the wraparound V6
                // config firing this property before the fix.
                if (both_occ) assert ($signed(ck_t[slot_b] - ck_t[slot_a]) >= 0);   // non-decreasing: tied event timestamps may create equal-t checkpoints (M=1)
            end
        end
    endgenerate

    always @(posedge clk_i) if (rst_ni) begin
        // F3: ring occupancy never exceeds capacity (CKPT_DEPTH=8).
        assert (ck_cnt <= 4'd8);

        // F5-equiv: journal occupancy never exceeds capacity (JRNL_DEPTH=64).
        assert ((jtail - jhead) <= 16'd64);
        // journal head never runs ahead of the tail.
        assert (jhead <= jtail);

        // F8-equiv: com_sel (which context is "committed") changes only in S_COMMIT_B.
        if ($past(rst_ni) && $changed(com_sel)) assert ($past(state) == S_COMMIT_B);

        // F9-equiv: a commit-swap adopts exactly the replayed chain's final time.
        if ($past(rst_ni) && $past(state) == S_COMMIT_B) assert (t_now == $past(chain_t));

        // F10-equiv: rpl_active only ever clears at a commit (S_COMMIT_B) or one of the
        // two designed resets (emergency checkpoint retirement S_IO_PK2, late-insertion
        // restart S_LATE_C) -- never silently abandoned elsewhere.
        if ($past(rst_ni) && $past(rpl_active) && !rpl_active)
            assert ($past(state) == S_COMMIT_B || $past(state) == S_IO_PK2 || $past(state) == S_LATE_C);

        // F12-equiv: event counters are monotonic non-decreasing.
        if ($past(rst_ni)) begin
            assert (cnt_units_o  >= $past(cnt_units_o));
            assert (cnt_drop_o   >= $past(cnt_drop_o));
            assert (cnt_late_o   >= $past(cnt_late_o));
            assert (cnt_replay_o >= $past(cnt_replay_o));
            assert (cnt_query_o  >= $past(cnt_query_o));
        end
    end

    // F22-equiv: clean state the cycle after reset is released.
    always @(posedge clk_i) begin
        if ($past(rst_ni) === 1'b0 && rst_ni) begin
            assert (!busy_o && !replay_pending_o && !provisional_o);
            assert (cnt_units_o == 0 && cnt_drop_o == 0 && cnt_late_o == 0);
            assert (cnt_replay_o == 0 && cnt_query_o == 0);
        end
    end
endmodule
bind tide_temporal formal_temporal_checks u_formal_temporal_checks (
    .ck_t0(ck_t[0]), .ck_t1(ck_t[1]), .ck_t2(ck_t[2]), .ck_t3(ck_t[3]),
    .ck_t4(ck_t[4]), .ck_t5(ck_t[5]), .ck_t6(ck_t[6]), .ck_t7(ck_t[7]),
    .*
);


// ============================================================================
// SECTION 5: event FIFO safety (F13-F16), bound into tide_event_fifo
// ============================================================================
module formal_fifo_checks #(parameter integer DEPTH = 8) (
    input logic                     clk_i,
    input logic                     rst_ni,
    input logic [$clog2(DEPTH)-1:0] wr_ptr,
    input logic [$clog2(DEPTH)-1:0] rd_ptr,
    input logic [$clog2(DEPTH):0]   count,
    input logic                     full_o,
    input logic                     valid_o,
    input logic                     push_valid_i,
    input logic                     push_accepted_o,
    input logic                     overrun_o
);
    always @(posedge clk_i) if (rst_ni) begin
        // F14/F15-equiv: a push is accepted iff requested and the FIFO wasn't full.
        assert (push_accepted_o == (push_valid_i && !full_o));

        // F14-equiv: overrun_o is asserted exactly the cycle after a push is attempted
        // against a full FIFO.
        if ($past(rst_ni)) assert (overrun_o == $past(push_valid_i && full_o));

        // F16-equiv: occupancy bookkeeping matches pointer arithmetic (mod DEPTH, with the
        // wr_ptr==rd_ptr ambiguity between "empty" and "full" resolved by `count` itself).
        assert (count == ((wr_ptr - rd_ptr) & (DEPTH[$clog2(DEPTH):0] - 1)) ||
               (count == DEPTH[$clog2(DEPTH):0] && wr_ptr == rd_ptr));

        assert (full_o  == (count == DEPTH[$clog2(DEPTH):0]));
        assert (valid_o == (count != 0));
    end

    // F22-equiv for the FIFO: clean the cycle after reset is released.
    always @(posedge clk_i) begin
        if ($past(rst_ni) === 1'b0 && rst_ni) assert (count == 0 && !valid_o && !full_o);
    end
endmodule
bind tide_event_fifo formal_fifo_checks u_formal_fifo_checks (.*);


// ============================================================================
// Extra (not in doc 44's list, added for this implementation): LSU/RAM handshake
// safety, since they are also fully owned by Agent 4.
// ============================================================================
module formal_lsu_ram_checks (
    input logic clk_i,
    input logic rst_ni,
    input logic tc_ce_i, input logic lsu_ce_i,
    input logic cpu_wr_en_i, input logic cpu_wr_ce_i,
    input logic conflict_o, input logic oor_o
);
    wire cpu_req = cpu_wr_en_i & cpu_wr_ce_i;
    always @(posedge clk_i) if (rst_ni) begin
        // Priority is strict TC > LSU > CPU-write > DBG; conflict_o must latch exactly
        // when two or more of the top three requesters assert ce in the same cycle.
        if ($past(rst_ni) &&
            (($past(tc_ce_i) & $past(lsu_ce_i)) | ($past(tc_ce_i) & $past(cpu_req)) |
             ($past(lsu_ce_i) & $past(cpu_req))))
            assert (conflict_o);

        // conflict_o and oor_o are sticky: only an external reset clears them (no W1C
        // port exists on this module), so they must never fall on their own.
        if ($past(rst_ni)) begin
            if ($past(conflict_o)) assert (conflict_o);
            if ($past(oor_o))      assert (oor_o);
        end
    end
endmodule
bind tide_ram formal_lsu_ram_checks u_formal_lsu_ram_checks (.*);
