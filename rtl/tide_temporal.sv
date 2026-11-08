// -----------------------------------------------------------------------------
// tide_temporal.sv - Temporal controller for out-of-sequence measurement handling
// Owner: Agent 4.  Spec: doc 10, doc 17 (wins on conflict), memmap.yaml, STEP_0.
// See ASSUMPTIONS.md for every place where the spec set was inconsistent.
//
// What it does
//   * consumes events (t, z0..z3, flags) one at a time when it is at a
//     "consume point" (IDLE, or the boundary between two replay units)
//   * classifies: INIT / QUERY / IN-ORDER (t > t_now) / LATE (t_now >= t > t_oldest_ckpt)
//     / TOO-OLD (dropped).   Ties with t_now are LATE (doc 17 6.1).
//   * IN-ORDER: burst-load committed ctx -> RF, write event regs, run UNIT,
//     burst-store ctx, append journal, checkpoint every M events, publish output.
//   * LATE: insert into the sorted journal (insertion sort in TIDE-RAM, ties go
//     after existing equals), invalidate newer checkpoints, replay from the latest
//     checkpoint <= t_event through the journal tail, commit by swapping the
//     committed/shadow context pointer.
//   * In-order / query events preempt a replay at unit boundaries: the replay ctx
//     is saved to the shadow buffer (PREEMPT_SAVE), the event is served against the
//     committed ("tip") ctx, the replay ctx is restored (PREEMPT_RESTORE).
//   * Output snapshot is protected by a seqlock (odd = update in progress).
//
// RF ABI used towards the microcode (confirm with Agent 1, ASSUMPTIONS A4):
//   r0..r5 x, r6..r26 P (context burst window, 27 words), r27 t_now (int us, value
//   BEFORE the unit), r28 event t, r29..r32 z0..z3, r33..r36 R (selected R-set),
//   r37 dt_us = t_event - t_now (signed int32, STEP_0 ISA-03).
//   The unit must leave the updated ctx in r0..r26 and end with seq_done/boundary.
//
// TIDE-RAM word map (memmap.yaml): ckpt 0x000 (8 x 32), journal 0x100 (64 x 5),
//   ctx0 0x240, ctx1 0x260, query-result scratch 0x2C0 (inside reserved area).
//   Context slot layout: +0 t_last, +1..+6 x, +7..+27 P (lower-tri), +28 journal_ptr.
// -----------------------------------------------------------------------------
module tide_temporal #(
    parameter [8:0]   UNIT_ENTRY  = 9'd0,
    parameter [8:0]   QUERY_ENTRY = 9'd16,
    parameter integer CKPT_DEPTH  = 8,     // power of two
    parameter integer JRNL_DEPTH  = 64     // power of two, <= 64
) (
    input  logic         clk_i,
    input  logic         rst_ni,

    // ---- configuration (from MMIO) ----
    input  logic [4:0]   ckpt_interval_i,  // M = 1..16 (0 treated as 1)
    input  logic         enable_i,
    input  logic         irq_en_i,
    input  logic [8:0]   status_clr_i,     // W1C, bit positions as status_o
    input  logic         clr_lat_i,

    // ---- event interface (head of the event FIFO) ----
    input  logic         event_valid_i,
    input  logic [31:0]  event_t_i,
    input  logic [31:0]  event_tarr_i,     // unused: arrival order == FIFO order
    input  logic [31:0]  event_z0_i,
    input  logic [31:0]  event_z1_i,
    input  logic [31:0]  event_z2_i,
    input  logic [31:0]  event_z3_i,
    input  logic [4:0]   event_flags_i,    // [1:0] rset, [2] query, [3] no-gate, [4] init
    output logic         event_ready_o,
    output logic [4:0]   cur_flags_o,      // flags of the event being served
    output logic [1:0]   rset_sel_o,       // R-set selector for rset_r*_i
    input  logic [31:0]  rset_r0_i,
    input  logic [31:0]  rset_r1_i,
    input  logic [31:0]  rset_r2_i,
    input  logic [31:0]  rset_r3_i,

    // ---- sequencer ----
    output logic         seq_start_o,
    output logic [8:0]   seq_start_addr_o,
    input  logic         seq_running_i,
    input  logic         seq_done_i,
    input  logic         seq_boundary_i,

    // ---- LSU burst command ----
    output logic         lsu_burst_start_o,
    output logic         lsu_burst_we_o,
    output logic [6:0]   lsu_burst_rf_base_o,
    output logic [11:0]  lsu_burst_ram_base_o,
    output logic [5:0]   lsu_burst_len_o,
    input  logic         lsu_burst_done_i,

    // ---- register file write port ----
    output logic         rf_wen_o,
    output logic [6:0]   rf_waddr_o,
    output logic [31:0]  rf_wdata_o,

    // ---- TIDE-RAM direct port ----
    output logic         ram_ce_o,
    output logic         ram_we_o,
    output logic [11:0]  ram_addr_o,
    output logic [31:0]  ram_wdata_o,
    input  logic [31:0]  ram_rdata_i,

    // ---- status ----
    output logic         busy_o,
    output logic         replay_pending_o,
    output logic         provisional_o,
    output logic         irq_o,
    output logic [4:0]   irq_ev_o,         // [0]UNIT_DONE [1]CKPT_SAVED [3]ERR_JRNL [4]ERR_LATE
    output logic [8:0]   status_o,         // BUSY,REPLAY_PENDING,-,LATE_DROP,JRNL_FULL,ABORT,-,NAN,PROV
    output logic         gpio_engine_busy_o,
    output logic         gpio_late_drop_o,
    output logic         gpio_provisional_o,

    // ---- counters ----
    output logic [31:0]  cnt_units_o,
    output logic [31:0]  cnt_late_o,
    output logic [31:0]  cnt_drop_o,
    output logic [31:0]  cnt_replay_o,
    output logic [31:0]  cnt_abort_o,
    output logic [31:0]  cnt_query_o,
    output logic [31:0]  cyc_last_unit_o,
    output logic [31:0]  cyc_last_replay_o,
    output logic [31:0]  lat_last_wait_o,
    output logic [31:0]  lat_max_wait_o,

    // ---- output snapshot (seqlock) and query result ----
    output logic [31:0]  out_seq_o,
    output logic [31:0]  out_t_o,
    output logic [191:0] out_x_o,          // {x5..x0}
    output logic [191:0] out_pdiag_o,      // {P55,P44,P33,P22,P11,P00}
    output logic         out_committed_o,
    output logic [191:0] qout_x_o,         // {x5..x0} predicted
    output logic [31:0]  qout_t_o
);
    // ------------------------------------------------------------------
    // constants
    // ------------------------------------------------------------------
    localparam integer CW = $clog2(CKPT_DEPTH);
    localparam integer JW = $clog2(JRNL_DEPTH);
    localparam [11:0] CKPT_BASE = 12'h000;
    localparam [11:0] JRNL_BASE = 12'h100;
    localparam [11:0] CTX0      = 12'h240;
    localparam [11:0] CTX1      = 12'h260;
    localparam [11:0] QRES      = 12'h281;   // [I-TC-4] x0..x5 as written by microcode at ABI.RAM_QOUT(0x280)+1..6
    localparam [5:0]  CTXLEN    = 6'd27;
    // [INTEGRATION FIX I-TC-1] first RF register of the 27-word context burst.
    // The assembler ABI (tools/tide_abi.py R_XBASE/R_PBASE) keeps x in r1..r6 and
    // P in r7..r27; the burst used r0..r26 (the earlier placeholder ABI), so every
    // post-unit burst-store rewrote the context one word off the microcode's
    // own correct CTX_STORE (measured: ctx0+1 kept the old x0, x0..P20 shifted +1).
    localparam [6:0]  RF_CTX0   = 7'd1;

    // RF ABI -- reconciled against tools/tide_abi.py (the real ABI d01's
    // assembler/simulator/microcode actually use), see ASSUMPTIONS.md A4 and
    // PROVENANCE.md. Two things changed from the original placeholder:
    //   * RF_DT (dt_us) moves from r37 to r118 (ABI.R_DT_US): r37 is a live
    //     scratch register inside the real microcode's own PREDICT routine,
    //     so leaving dt_us at r37 would have the microcode overwrite its own
    //     input mid-routine.
    //   * RF_CTX (new, r119 / ABI.R_CTX) is added: the real microcode loads
    //     and stores the 27-word context itself, via LDIDX0 r119 followed by
    //     LOAD/STORE [n+idx0] -- it does NOT expect the context to already
    //     be sitting in r1..r27 at unit start. The RF_TNOW write (r27) is
    //     kept only because r0..r26 is still separately used by the
    //     emergency-checkpoint path in S_IO_PK (RAM -> RF -> RAM, entirely
    //     independent of the sequencer/microcode); the real microcode
    //     immediately overwrites r27 as scratch, so the RF_TNOW write is
    //     inert (but harmless) on the unit-start path.
    localparam [6:0] RF_TNOW = 7'd27, RF_EVT = 7'd28, RF_Z0 = 7'd29, RF_R0 = 7'd33,
                     RF_DT = 7'd118, RF_CTX = 7'd119;

    // states
    localparam [5:0]
        S_IDLE   = 6'd0,  S_CLS    = 6'd1,  S_DROP   = 6'd2,
        S_LOAD   = 6'd3,  S_SAVE   = 6'd4,  S_SAVEW  = 6'd5,
        S_INIT_A = 6'd6,  S_INIT_B = 6'd7,  S_INIT_C = 6'd8,
        S_IO_A   = 6'd9,  S_EVTW   = 6'd10, S_RUN0   = 6'd11, S_RUNW   = 6'd12,
        S_IO_B   = 6'd13, S_JAPP   = 6'd14, S_IO_C   = 6'd15, S_IO_D0  = 6'd16,
        S_IO_D   = 6'd17, S_IO_E   = 6'd18,
        S_QRY_A  = 6'd19, S_Q_RD   = 6'd20,
        S_LATE_A = 6'd21, S_LATE_B = 6'd22, S_JI_R0  = 6'd23, S_JI_C   = 6'd24,
        S_JI_CPR = 6'd25, S_JI_CPW = 6'd26, S_JI_NEW = 6'd27, S_LATE_C = 6'd28,
        S_RPL_PRIME = 6'd39, S_RCK_A = 6'd40, S_RCK_B = 6'd41, S_RCK_C = 6'd42, S_RPL_NEXT = 6'd29, S_RPL_RD = 6'd30, S_RPL_END = 6'd31, S_RPL_BND = 6'd32,
        S_COMMIT = 6'd33, S_COMMIT_B = 6'd34,
        S_OUT_RD = 6'd35, S_OUT_W  = 6'd36,
        S_IO_PK  = 6'd37, S_IO_PK2 = 6'd38;

    localparam [1:0] P_IO = 2'd0, P_QRY = 2'd1, P_RPL = 2'd2;

    // ------------------------------------------------------------------
    // registers
    // ------------------------------------------------------------------
    logic [5:0]  state, ret;
    logic [4:0]  cnt;
    logic        bph;
    logic [1:0]  path;

    // captured event
    logic [31:0] ev_t, ev_z0, ev_z1, ev_z2, ev_z3;
    logic [4:0]  ev_flags;
    wire  [1:0]  ev_rset = ev_flags[1:0];

    // event being loaded into RF
    logic [31:0] x_t, x_z0, x_z1, x_z2, x_z3, x_base;
    logic [1:0]  x_rset;

    // temporal state
    logic [31:0] t_now;
    logic        init_done;
    logic        com_sel;          // 0: committed = CTX0, 1: committed = CTX1
    logic        rpl_active;
    logic        out_owed;
    logic        deg_ev;           // current in-order event could not be journaled
    logic [1:0]  drop_kind;        // 0 too-old/no-init, 1 journal full
    logic        defer_q;          // event parked: draining a suspended replay before (re)classifying it

    // checkpoint ring
    logic [CW-1:0]   ck_head;
    logic [CW:0]     ck_cnt;
    logic [31:0]     ck_t [0:CKPT_DEPTH-1];
    logic [15:0]     ck_j [0:CKPT_DEPTH-1];
    logic [4:0]      ev_since;

    // journal (data in RAM; flags in flops)
    logic [15:0]     jhead, jtail;
    logic [4:0]      rpl_since;      // [I-TC-5] chain units since the last ckpt (replay-time checkpoints)
    logic [1:0]      jrs [0:JRNL_DEPTH-1];

    // replay chain
    logic [31:0]     chain_t;
    logic [15:0]     rpl_r;

    // insertion scratch
    logic [15:0]     ji_i, ji_left, ji_p;
    logic [2:0]      ji_w;

    // burst / save parameters
    logic [6:0]      bs_rfb;
    logic [11:0]     bs_ram;
    logic [5:0]      bs_len;
    logic            bs_we;
    logic [11:0]     sv_base;
    logic [31:0]     sv_t;
    logic [15:0]     sv_j;
    logic            sv_nowords;

    // output staging
    logic [31:0]     tmp [0:11];
    logic [31:0]     out_seq;
    logic [31:0]     out_t;
    logic [31:0]     out_x [0:5];
    logic [31:0]     out_pd [0:5];
    logic            out_committed;
    logic [31:0]     q_x [0:5];
    logic [31:0]     q_t;

    // counters / status
    logic [31:0]     c_units, c_late, c_drop, c_replay, c_abort, c_query;
    logic [31:0]     cyc_unit, cyc_replay, svc_cyc, rpl_cyc;
    logic [31:0]     wait_cnt, lat_last, lat_max;
    logic            st_late_drop, st_jrnl_full, st_abort, st_nan;
    logic [4:0]      irq_ev_q;

    // ------------------------------------------------------------------
    // derived values
    // ------------------------------------------------------------------
    wire [11:0] com_base = com_sel ? CTX1 : CTX0;
    wire [11:0] shd_base = com_sel ? CTX0 : CTX1;

    wire [CW-1:0] newest_idx = ck_head - {{(CW-1){1'b0}}, 1'b1};
    wire [CW-1:0] oldest_idx = ck_head - ck_cnt[CW-1:0];
    wire [CW-1:0] next_oldest_idx = oldest_idx + {{(CW-1){1'b0}}, 1'b1};

    wire [31:0] ck_oldest_t = ck_t[oldest_idx];
    wire [31:0] ck_newest_t = ck_t[newest_idx];

    wire        diff_pos  = ($signed(ev_t - t_now) > 0);
    wire        too_old   = ($signed(ev_t - ck_oldest_t) <= 0);
    wire [15:0] jocc      = jtail - jhead;
    wire        jfull     = (jocc == JRNL_DEPTH[15:0]);
    wire        caught_up = (rpl_r == jtail);
    wire [4:0]  m_eff     = (ckpt_interval_i == 5'd0) ? 5'd1 : ckpt_interval_i;
    // Retiring the oldest checkpoint frees the journal entries below the next checkpoint.
    // Safe when no replay runs, or when the replay chain has already consumed those entries.
    wire        can_retire = (ck_cnt >= 2) &&
                             (!rpl_active || ($signed(ck_j[next_oldest_idx] - rpl_r) <= 0));

    function [11:0] jaddr(input [15:0] idx, input [2:0] w);
        jaddr = JRNL_BASE + ({6'd0, idx[JW-1:0]} * 12'd5) + {9'd0, w};
    endfunction

    function [11:0] ckbase(input [CW-1:0] idx);
        ckbase = CKPT_BASE + {idx, 5'd0};
    endfunction

    // P lower-triangular diagonal offsets: P00, P11, P22, P33, P44, P55
    function [4:0] pdiag(input [2:0] k);
        case (k)
            3'd0: pdiag = 5'd0;  3'd1: pdiag = 5'd2;  3'd2: pdiag = 5'd5;
            3'd3: pdiag = 5'd9;  3'd4: pdiag = 5'd14; default: pdiag = 5'd20;
        endcase
    endfunction

    function is_nan(input [31:0] w);
        is_nan = (w[30:23] == 8'hFF) && (w[22:0] != 23'd0);
    endfunction

    wire take = event_valid_i & event_ready_o;

    // ------------------------------------------------------------------
    // combinational outputs
    // ------------------------------------------------------------------
    always_comb begin
        event_ready_o        = enable_i & ~defer_q & ((state == S_IDLE) | ((state == S_RPL_BND) & ~caught_up));
        cur_flags_o          = ev_flags;
        rset_sel_o           = x_rset;

        seq_start_o          = (state == S_RUN0);
        seq_start_addr_o     = (path == P_QRY) ? QUERY_ENTRY : UNIT_ENTRY;

        lsu_burst_start_o    = ((state == S_LOAD) | (state == S_SAVE)) & ~bph;
        lsu_burst_we_o       = bs_we;
        lsu_burst_rf_base_o  = bs_rfb;
        lsu_burst_ram_base_o = bs_ram;
        lsu_burst_len_o      = bs_len;

        rf_wen_o             = 1'b0;
        rf_waddr_o           = 7'd0;
        rf_wdata_o           = 32'd0;

        ram_ce_o             = 1'b0;
        ram_we_o             = 1'b0;
        ram_addr_o           = 12'd0;
        ram_wdata_o          = 32'd0;

        case (state)
            S_SAVEW: begin
                ram_ce_o    = 1'b1;
                ram_we_o    = 1'b1;
                ram_addr_o  = sv_base + ((cnt == 5'd0) ? 12'd0 : 12'd28);
                ram_wdata_o = (cnt == 5'd0) ? sv_t : {16'd0, sv_j};
            end
            S_EVTW: begin
                rf_wen_o = 1'b1;
                case (cnt)
                    5'd0:  begin rf_waddr_o = RF_TNOW;      rf_wdata_o = x_base; end
                    5'd1:  begin rf_waddr_o = RF_EVT;       rf_wdata_o = x_t;    end
                    5'd2:  begin rf_waddr_o = RF_Z0;        rf_wdata_o = x_z0;   end
                    5'd3:  begin rf_waddr_o = RF_Z0 + 7'd1; rf_wdata_o = x_z1;   end
                    5'd4:  begin rf_waddr_o = RF_Z0 + 7'd2; rf_wdata_o = x_z2;   end
                    5'd5:  begin rf_waddr_o = RF_Z0 + 7'd3; rf_wdata_o = x_z3;   end
                    5'd6:  begin rf_waddr_o = RF_R0;        rf_wdata_o = rset_r0_i; end
                    5'd7:  begin rf_waddr_o = RF_R0 + 7'd1; rf_wdata_o = rset_r1_i; end
                    5'd8:  begin rf_waddr_o = RF_R0 + 7'd2; rf_wdata_o = rset_r2_i; end
                    5'd9:  begin rf_waddr_o = RF_R0 + 7'd3; rf_wdata_o = rset_r3_i; end
                    5'd10: begin rf_waddr_o = RF_DT;        rf_wdata_o = x_t - x_base; end
                    // [ABI fix, see RF ABI comment above]: r119 = TIDE-RAM word
                    // address of t_last for the context this unit must load/
                    // store itself via idx0 (LDIDX0 r119). com_base/shd_base
                    // already point at t_last (the microcode's own [1+idx0]
                    // load for x[0] is relative to that word), so no +1 offset
                    // here -- only the burst-load path (which skips t_last)
                    // adds it. Selected by path: IO/QUERY read the committed
                    // ("tip") context, REPLAY reads/writes the shadow context
                    // (matches the bs_ram base already used to reach this
                    // state on each of those three entry points, see S_IO_A,
                    // S_QRY_A, S_RPL_RD above).
                    default: begin
                        rf_waddr_o = RF_CTX;
                        rf_wdata_o = {20'd0, (path == P_RPL) ? shd_base : com_base};
                    end
                endcase
            end
            S_JAPP: begin
                ram_ce_o   = 1'b1;
                ram_we_o   = 1'b1;
                ram_addr_o = jaddr(jtail, cnt[2:0]);
                case (cnt[2:0])
                    3'd0: ram_wdata_o = ev_t;
                    3'd1: ram_wdata_o = ev_z0;
                    3'd2: ram_wdata_o = ev_z1;
                    3'd3: ram_wdata_o = ev_z2;
                    default: ram_wdata_o = ev_z3;
                endcase
            end
            S_JI_R0: begin
                if (ji_left != 16'd0) begin
                    ram_ce_o   = 1'b1;
                    ram_addr_o = jaddr(ji_i, 3'd0);
                end
            end
            S_JI_C: begin
                if ($signed(ram_rdata_i - ev_t) > 0) begin
                    ram_ce_o    = 1'b1;
                    ram_we_o    = 1'b1;
                    ram_addr_o  = jaddr(ji_i + 16'd1, 3'd0);
                    ram_wdata_o = ram_rdata_i;
                end
            end
            S_JI_CPR: begin
                ram_ce_o   = 1'b1;
                ram_addr_o = jaddr(ji_i, ji_w);
            end
            S_JI_CPW: begin
                ram_ce_o    = 1'b1;
                ram_we_o    = 1'b1;
                ram_addr_o  = jaddr(ji_i + 16'd1, ji_w);
                ram_wdata_o = ram_rdata_i;
            end
            S_JI_NEW: begin
                ram_ce_o   = 1'b1;
                ram_we_o   = 1'b1;
                ram_addr_o = jaddr(ji_p, cnt[2:0]);
                case (cnt[2:0])
                    3'd0: ram_wdata_o = ev_t;
                    3'd1: ram_wdata_o = ev_z0;
                    3'd2: ram_wdata_o = ev_z1;
                    3'd3: ram_wdata_o = ev_z2;
                    default: ram_wdata_o = ev_z3;
                endcase
            end
            S_RPL_RD: begin
                if (cnt < 5'd5) begin
                    ram_ce_o   = 1'b1;
                    ram_addr_o = jaddr(rpl_r, cnt[2:0]);
                end
            end
            S_Q_RD: begin
                if (cnt < 5'd6) begin
                    ram_ce_o   = 1'b1;
                    ram_addr_o = QRES + {7'd0, cnt};
                end
            end
            S_OUT_RD: begin
                if (cnt < 5'd12) begin
                    ram_ce_o   = 1'b1;
                    ram_addr_o = (cnt < 5'd6) ? (com_base + 12'd1 + {7'd0, cnt})
                                              : (com_base + 12'd7 + {7'd0, pdiag(cnt[2:0] - 3'd6)});
                end
            end
            default: ;
        endcase

        // status / passthrough
        busy_o             = (state != S_IDLE) | out_owed;
        replay_pending_o   = rpl_active;
        provisional_o      = rpl_active;
        irq_ev_o           = irq_ev_q;
        irq_o              = irq_en_i & (|irq_ev_q);
        status_o           = {rpl_active, st_nan, 1'b0, st_abort, st_jrnl_full, st_late_drop, 1'b0,
                              rpl_active, busy_o};
        gpio_engine_busy_o = busy_o;
        gpio_late_drop_o   = st_late_drop;
        gpio_provisional_o = rpl_active;

        cnt_units_o        = c_units;
        cnt_late_o         = c_late;
        cnt_drop_o         = c_drop;
        cnt_replay_o       = c_replay;
        cnt_abort_o        = c_abort;
        cnt_query_o        = c_query;
        cyc_last_unit_o    = cyc_unit;
        cyc_last_replay_o  = cyc_replay;
        lat_last_wait_o    = lat_last;
        lat_max_wait_o     = lat_max;

        out_seq_o          = out_seq;
        out_t_o            = out_t;
        out_committed_o    = out_committed;
        out_x_o            = {out_x[5], out_x[4], out_x[3], out_x[2], out_x[1], out_x[0]};
        out_pdiag_o        = {out_pd[5], out_pd[4], out_pd[3], out_pd[2], out_pd[1], out_pd[0]};
        qout_x_o           = {q_x[5], q_x[4], q_x[3], q_x[2], q_x[1], q_x[0]};
        qout_t_o           = q_t;
    end

    // ------------------------------------------------------------------
    // sequential logic
    // ------------------------------------------------------------------
    integer k;
    always_ff @(posedge clk_i) begin
        if (!rst_ni) begin
            state <= S_IDLE; ret <= S_IDLE; cnt <= 5'd0; bph <= 1'b0; path <= P_IO;
            ev_t <= 32'd0; ev_z0 <= 32'd0; ev_z1 <= 32'd0; ev_z2 <= 32'd0; ev_z3 <= 32'd0; ev_flags <= 5'd0;
            x_t <= 32'd0; x_z0 <= 32'd0; x_z1 <= 32'd0; x_z2 <= 32'd0; x_z3 <= 32'd0; x_base <= 32'd0; x_rset <= 2'd0;
            t_now <= 32'd0; init_done <= 1'b0; com_sel <= 1'b0; rpl_active <= 1'b0; out_owed <= 1'b0;
            deg_ev <= 1'b0; drop_kind <= 2'd0; defer_q <= 1'b0;
            ck_head <= {CW{1'b0}}; ck_cnt <= {(CW+1){1'b0}}; ev_since <= 5'd0;
            jhead <= 16'd0; jtail <= 16'd0; rpl_since <= 5'd0;
            chain_t <= 32'd0; rpl_r <= 16'd0;
            ji_i <= 16'd0; ji_left <= 16'd0; ji_p <= 16'd0; ji_w <= 3'd0;
            bs_rfb <= 7'd0; bs_ram <= 12'd0; bs_len <= 6'd0; bs_we <= 1'b0;
            sv_base <= 12'd0; sv_t <= 32'd0; sv_j <= 16'd0; sv_nowords <= 1'b0;
            out_seq <= 32'd0; out_t <= 32'd0; out_committed <= 1'b0; q_t <= 32'd0;
            for (k = 0; k < 6; k = k + 1) begin out_x[k] <= 32'd0; out_pd[k] <= 32'd0; q_x[k] <= 32'd0; end
            c_units <= 32'd0; c_late <= 32'd0; c_drop <= 32'd0; c_replay <= 32'd0; c_abort <= 32'd0; c_query <= 32'd0;
            cyc_unit <= 32'd0; cyc_replay <= 32'd0; svc_cyc <= 32'd0; rpl_cyc <= 32'd0;
            wait_cnt <= 32'd0; lat_last <= 32'd0; lat_max <= 32'd0;
            st_late_drop <= 1'b0; st_jrnl_full <= 1'b0; st_abort <= 1'b0; st_nan <= 1'b0;
            irq_ev_q <= 5'd0;
        end else begin
            // ---------------- defaults ----------------
            irq_ev_q <= 5'd0;
            if (state != S_IDLE) svc_cyc <= svc_cyc + 32'd1;
            if (rpl_active)      rpl_cyc <= rpl_cyc + 32'd1;

            // sticky status W1C
            if (status_clr_i[3]) st_late_drop  <= 1'b0;
            if (status_clr_i[4]) st_jrnl_full  <= 1'b0;
            if (status_clr_i[5]) st_abort      <= 1'b0;
            if (status_clr_i[7]) st_nan        <= 1'b0;

            // head-of-line wait measurement
            if (clr_lat_i) begin lat_max <= 32'd0; lat_last <= 32'd0; end
            if (enable_i && event_valid_i && !event_ready_o) wait_cnt <= wait_cnt + 32'd1;
            else if (!event_valid_i)                          wait_cnt <= 32'd0;

            // ---------------- event capture (IDLE / replay boundary) ----------------
            if (take) begin
                ev_t <= event_t_i; ev_z0 <= event_z0_i; ev_z1 <= event_z1_i;
                ev_z2 <= event_z2_i; ev_z3 <= event_z3_i; ev_flags <= event_flags_i;
                lat_last <= wait_cnt;
                if (wait_cnt > lat_max && !clr_lat_i) lat_max <= wait_cnt;
                wait_cnt <= 32'd0;
                svc_cyc  <= 32'd0;
                state    <= S_CLS;
            end

            case (state)
            // ==============================================================
            S_IDLE: begin
                if (!take && out_owed) begin
                    cnt <= 5'd0; ret <= S_IDLE; state <= S_OUT_RD;
                end
            end

            // ==============================================================
            // classification
            S_CLS: begin
                if (ev_flags[2]) begin
                    // QUERY (overrides everything else); needs an initialised context
                    if (!init_done) begin
                        drop_kind <= 2'd0; state <= S_DROP;
                    end else if (rpl_active) begin
                        bs_we <= 1'b1; bs_rfb <= RF_CTX0; bs_ram <= shd_base + 12'd1; bs_len <= CTXLEN;
                        sv_base <= shd_base; sv_t <= chain_t; sv_j <= jtail; sv_nowords <= 1'b0;
                        bph <= 1'b0; ret <= S_QRY_A; state <= S_SAVE;
                    end else state <= S_QRY_A;
                end else if (ev_flags[4]) begin
                    // INIT: read the seed context from the fixed shadow area (CTX1)
                    bs_we <= 1'b0; bs_rfb <= RF_CTX0; bs_ram <= CTX1 + 12'd1; bs_len <= CTXLEN;
                    bph <= 1'b0; ret <= S_INIT_A; state <= S_LOAD;
                end else if (!init_done) begin
                    drop_kind <= 2'd0; state <= S_DROP;
                end else if (diff_pos) begin
                    // IN-ORDER
                    if (rpl_active) begin
                        // If the journal is exhausted and only the in-flight replay is holding
                        // it that way, do NOT preempt-and-reset (that would discard journal
                        // entries the replay still needs, silently losing whatever it was about
                        // to fold in).  Instead park this event and let the replay keep draining
                        // -- it is bounded and will either free enough journal (can_retire) or
                        // commit outright, at which point S_RPL_BND / S_COMMIT_B serve it.
                        if (jfull & ~can_retire) begin
                            defer_q <= 1'b1;
                            state <= S_RPL_NEXT;
                        end else begin
                            bs_we <= 1'b1; bs_rfb <= RF_CTX0; bs_ram <= shd_base + 12'd1; bs_len <= CTXLEN;
                            sv_base <= shd_base; sv_t <= chain_t; sv_j <= jtail; sv_nowords <= 1'b0;
                            bph <= 1'b0; ret <= S_IO_A; state <= S_SAVE;   // PREEMPT_SAVE
                        end
                    end else state <= S_IO_A;
                end else if (!too_old) begin
                    if (rpl_active & jfull & ~can_retire) begin
                        // same reasoning as the in-order case: don't force a drop just because
                        // the in-flight replay hasn't freed room yet -- drain it and retry.
                        defer_q <= 1'b1;
                        state   <= S_RPL_NEXT;
                    end else state <= S_LATE_A;
                end else begin
                    drop_kind <= 2'd0; state <= S_DROP;
                end
            end

            S_DROP: begin
                c_drop <= c_drop + 32'd1;
                if (drop_kind == 2'd1) begin
                    st_jrnl_full <= 1'b1; irq_ev_q[3] <= 1'b1;
                end else begin
                    irq_ev_q[4] <= 1'b1;
                end
                st_late_drop <= 1'b1;
                state <= rpl_active ? S_RPL_NEXT : S_IDLE;
            end

            // ==============================================================
            // sub-routines: burst load / burst store (+ ctx metadata words)
            S_LOAD: begin
                bph <= 1'b1;
                if (lsu_burst_done_i) begin bph <= 1'b0; state <= ret; end
            end
            S_SAVE: begin
                bph <= 1'b1;
                if (lsu_burst_done_i) begin
                    bph <= 1'b0; cnt <= 5'd0;
                    state <= sv_nowords ? ret : S_SAVEW;
                end
            end
            S_SAVEW: begin
                if (cnt == 5'd1) state <= ret; else cnt <= cnt + 5'd1;
            end

            // ==============================================================
            // INIT
            S_INIT_A: begin
                com_sel <= 1'b0;
                bs_we <= 1'b1; bs_rfb <= RF_CTX0; bs_ram <= CTX0 + 12'd1; bs_len <= CTXLEN;
                sv_base <= CTX0; sv_t <= ev_t; sv_j <= 16'd0; sv_nowords <= 1'b0;
                bph <= 1'b0; ret <= S_INIT_B; state <= S_SAVE;
            end
            S_INIT_B: begin
                bs_we <= 1'b1; bs_rfb <= RF_CTX0; bs_ram <= CKPT_BASE + 12'd1; bs_len <= CTXLEN;
                sv_base <= CKPT_BASE; sv_t <= ev_t; sv_j <= 16'd0; sv_nowords <= 1'b0;
                bph <= 1'b0; ret <= S_INIT_C; state <= S_SAVE;
            end
            S_INIT_C: begin
                ck_t[0] <= ev_t; ck_j[0] <= 16'd0;
                ck_head <= {{(CW-1){1'b0}}, 1'b1};
                ck_cnt  <= {{CW{1'b0}}, 1'b1};
                jhead <= 16'd0; jtail <= 16'd0;
                ev_since <= 5'd0; rpl_active <= 1'b0; init_done <= 1'b1;
                t_now <= ev_t; deg_ev <= 1'b0;
                out_owed <= 1'b1;
                state <= S_IDLE;
            end

            // ==============================================================
            // IN-ORDER
            S_IO_A: begin
                // journal-full protection.  An in-order event can never be left un-journaled
                // (deg_ev): a later replay commit would reconstruct a context that silently
                // forgets it.  So if retirement is blocked only by an in-flight replay, that
                // replay is aborted (its target range is about to be discarded anyway) and we
                // fall through to the same emergency-checkpoint reset used when idle.
                deg_ev <= 1'b0;
                x_t <= ev_t; x_z0 <= ev_z0; x_z1 <= ev_z1; x_z2 <= ev_z2; x_z3 <= ev_z3;
                x_rset <= ev_rset; x_base <= t_now; path <= P_IO;
                bs_we <= 1'b0; bs_rfb <= RF_CTX0; bs_ram <= com_base + 12'd1; bs_len <= CTXLEN;
                bph <= 1'b0; cnt <= 5'd0; ret <= S_EVTW; state <= S_LOAD;
                if (jfull) begin
                    if (can_retire) begin
                        jhead  <= ck_j[next_oldest_idx];
                        ck_cnt <= ck_cnt - 1'b1;
                    end else begin
                        if (rpl_active) begin
                            rpl_active <= 1'b0;
                            c_abort    <= c_abort + 32'd1;
                            st_abort   <= 1'b1;
                        end
                        ret <= S_IO_PK;          // emergency pre-event checkpoint of the tip
                    end
                end
            end
            // emergency checkpoint: RF holds the committed ctx (just loaded)
            S_IO_PK: begin
                bs_we <= 1'b1; bs_rfb <= RF_CTX0; bs_ram <= ckbase(ck_head) + 12'd1; bs_len <= CTXLEN;
                sv_base <= ckbase(ck_head); sv_t <= t_now; sv_j <= jtail; sv_nowords <= 1'b0;
                bph <= 1'b0; ret <= S_IO_PK2; state <= S_SAVE;
            end
            S_IO_PK2: begin
                // the old window cannot be kept (its journal is full): new ckpt becomes the only one
                ck_t[ck_head] <= t_now; ck_j[ck_head] <= jtail;
                ck_head <= ck_head + {{(CW-1){1'b0}}, 1'b1};
                jhead   <= jtail;
                ck_cnt  <= {{CW{1'b0}}, 1'b1};   // [I-TC-3] new ckpt is the only valid one
                ev_since <= 5'd0;
                irq_ev_q[1] <= 1'b1;
                st_jrnl_full <= 1'b1; irq_ev_q[3] <= 1'b1;   // window was reset: report it
                cnt <= 5'd0; state <= S_EVTW;
            end

            S_EVTW: begin
                // [ABI fix] threshold raised 10->11 to admit the extra R_CTX
                // write cycle added above (cnt now runs 0..10, 11 cycles).
                if (cnt == 5'd11) begin cnt <= 5'd0; state <= S_RUN0; end
                else cnt <= cnt + 5'd1;
            end
            S_RUN0: state <= S_RUNW;
            S_RUNW: begin
                if (seq_done_i | seq_boundary_i) begin
                    case (path)
                        P_IO: begin
                            bs_we <= 1'b1; bs_rfb <= RF_CTX0; bs_ram <= com_base + 12'd1; bs_len <= CTXLEN;
                            sv_base <= com_base; sv_t <= x_t;
                            sv_j <= deg_ev ? jtail : (jtail + 16'd1); sv_nowords <= 1'b0;
                            bph <= 1'b0; ret <= S_IO_B; state <= S_SAVE;
                        end
                        P_QRY: begin
                            // [I-TC-4] the query microcode already stored t_q,x0..x5 to RAM at QOUT;
                            // no RF burst needed, read the result straight from RAM.
                            cnt <= 5'd0; state <= S_Q_RD;
                        end
                        default: state <= S_RPL_END;
                    endcase
                end
            end
            S_IO_B: begin
                t_now <= x_t;
                cnt <= 5'd0;
                state <= deg_ev ? S_IO_C : S_JAPP;
            end
            S_JAPP: begin
                if (cnt == 5'd4) begin
                    jrs[jtail[JW-1:0]] <= ev_rset;
                    jtail <= jtail + 16'd1;
                    state <= S_IO_C;
                end else cnt <= cnt + 5'd1;
            end
            S_IO_C: begin
                if (!deg_ev && !rpl_active && ({1'b0, ev_since} + 6'd1 >= {1'b0, m_eff})) begin
                    bs_we <= 1'b1; bs_rfb <= RF_CTX0; bs_ram <= ckbase(ck_head) + 12'd1; bs_len <= CTXLEN;
                    sv_base <= ckbase(ck_head); sv_t <= ev_t; sv_j <= jtail; sv_nowords <= 1'b0;
                    bph <= 1'b0; ret <= S_IO_D0; state <= S_SAVE;
                end else begin
                    if (ev_since != 5'd31) ev_since <= ev_since + 5'd1;
                    state <= S_IO_D;
                end
            end
            S_IO_D0: begin
                ck_t[ck_head] <= ev_t; ck_j[ck_head] <= jtail;
                ck_head <= ck_head + {{(CW-1){1'b0}}, 1'b1};
                if (ck_cnt == CKPT_DEPTH[CW:0]) jhead <= ck_j[ck_head + {{(CW-1){1'b0}}, 1'b1}];
                else ck_cnt <= ck_cnt + 1'b1;
                ev_since <= 5'd0;
                irq_ev_q[1] <= 1'b1;
                state <= S_IO_D;
            end
            S_IO_D: begin
                c_units <= c_units + 32'd1;
                cyc_unit <= svc_cyc;
                irq_ev_q[0] <= irq_ev_q[0] | 1'b1;
                cnt <= 5'd0; ret <= S_IO_E; state <= S_OUT_RD;
            end
            S_IO_E: begin
                deg_ev <= 1'b0;
                if (rpl_active) begin
                    // PREEMPT_RESTORE: reload the replay ctx from the shadow buffer
                    bs_we <= 1'b0; bs_rfb <= RF_CTX0; bs_ram <= shd_base + 12'd1; bs_len <= CTXLEN;
                    bph <= 1'b0; ret <= S_RPL_BND; state <= S_LOAD;
                end else state <= S_IDLE;
            end

            // ==============================================================
            // QUERY (non destructive predict of the committed / tip context)
            S_QRY_A: begin
                x_t <= ev_t; x_z0 <= ev_z0; x_z1 <= ev_z1; x_z2 <= ev_z2; x_z3 <= ev_z3;
                x_rset <= ev_rset; x_base <= t_now; path <= P_QRY;
                bs_we <= 1'b0; bs_rfb <= RF_CTX0; bs_ram <= com_base + 12'd1; bs_len <= CTXLEN;
                bph <= 1'b0; cnt <= 5'd0; ret <= S_EVTW; state <= S_LOAD;
            end
            S_Q_RD: begin
                if (cnt >= 5'd1) q_x[cnt - 5'd1] <= ram_rdata_i;
                if (cnt == 5'd6) begin
                    q_t <= x_t; c_query <= c_query + 32'd1;
                    state <= S_IO_E;
                end else cnt <= cnt + 5'd1;
            end

            // ==============================================================
            // LATE
            S_LATE_A: begin
                if (jfull) begin
                    if (can_retire) begin
                        jhead  <= ck_j[next_oldest_idx];
                        ck_cnt <= ck_cnt - 1'b1;
                        state  <= S_LATE_B;
                    end else begin
                        drop_kind <= 2'd1; state <= S_DROP;
                    end
                end else state <= S_LATE_B;
            end
            S_LATE_B: begin
                if (too_old) begin
                    drop_kind <= 2'd0; state <= S_DROP;
                end else if (ck_cnt > 1 && $signed(ck_newest_t - ev_t) > 0) begin
                    ck_cnt  <= ck_cnt - 1'b1;
                    ck_head <= ck_head - {{(CW-1){1'b0}}, 1'b1};
                end else begin
                    c_late   <= c_late + 32'd1;
                    ji_i     <= jtail - 16'd1;
                    ji_left  <= jtail - jhead;
                    state    <= S_JI_R0;
                end
            end
            S_JI_R0: begin
                if (ji_left == 16'd0) begin
                    ji_p <= ji_i + 16'd1; cnt <= 5'd0; state <= S_JI_NEW;
                end else state <= S_JI_C;
            end
            S_JI_C: begin
                if ($signed(ram_rdata_i - ev_t) > 0) begin
                    ji_w <= 3'd1; state <= S_JI_CPR;
                end else begin
                    ji_p <= ji_i + 16'd1; cnt <= 5'd0; state <= S_JI_NEW;
                end
            end
            S_JI_CPR: state <= S_JI_CPW;
            S_JI_CPW: begin
                if (ji_w == 3'd4) begin
                    jrs[(ji_i + 16'd1) & (JRNL_DEPTH - 1)] <= jrs[ji_i[JW-1:0]];
                    ji_i    <= ji_i - 16'd1;
                    ji_left <= ji_left - 16'd1;
                    state   <= S_JI_R0;
                end else begin
                    ji_w <= ji_w + 3'd1; state <= S_JI_CPR;
                end
            end
            S_JI_NEW: begin
                if (cnt == 5'd4) begin
                    jrs[ji_p[JW-1:0]] <= ev_rset;
                    jtail <= jtail + 16'd1;
                    state <= S_LATE_C;
                end else cnt <= cnt + 5'd1;
            end
            S_LATE_C: begin
                if (!rpl_active || ($signed(ji_p - rpl_r) < 0)) begin
                    // start a new chain (or restart the running one) from the newest valid ckpt
                    if (rpl_active) begin c_abort <= c_abort + 32'd1; st_abort <= 1'b1; end
                    else rpl_cyc <= 32'd0;
                    rpl_active <= 1'b1;
                    rpl_r   <= ck_j[newest_idx];
                    chain_t <= ck_t[newest_idx];
                    rpl_since <= 5'd0;
                    bs_we <= 1'b0; bs_rfb <= RF_CTX0; bs_ram <= ckbase(newest_idx) + 12'd1; bs_len <= CTXLEN;
                    bph <= 1'b0; ret <= S_RPL_PRIME; state <= S_LOAD;
                end else begin
                    state <= S_RPL_NEXT;           // new entry lies ahead of the chain
                end
            end

            // ==============================================================
            // REPLAY
            // [INTEGRATION FIX I-TC-2] The real microcode keeps the context in
            // TIDE-RAM (LDIDX0 r119 ... in-place STORE), not in the RF.  A replay
            // chain therefore has to start with the restored checkpoint present in
            // the SHADOW context in RAM; the burst-load above only filled the RF.
            // Write it out (RF r1..r27 -> shadow+1.., t_last -> shadow+0) before the
            // first replay unit.  Subsequent units of the chain update the shadow
            // context in place.
            S_RPL_PRIME: begin
                bs_we <= 1'b1; bs_rfb <= RF_CTX0; bs_ram <= shd_base + 12'd1; bs_len <= CTXLEN;
                sv_base <= shd_base; sv_t <= chain_t; sv_j <= jtail; sv_nowords <= 1'b0;
                bph <= 1'b0; ret <= S_RPL_NEXT; state <= S_SAVE;
            end
            S_RPL_NEXT: begin
                if (caught_up) state <= S_COMMIT;
                else begin cnt <= 5'd0; state <= S_RPL_RD; end
            end
            S_RPL_RD: begin
                case (cnt)
                    5'd1: x_t  <= ram_rdata_i;
                    5'd2: x_z0 <= ram_rdata_i;
                    5'd3: x_z1 <= ram_rdata_i;
                    5'd4: x_z2 <= ram_rdata_i;
                    5'd5: x_z3 <= ram_rdata_i;
                    default: ;
                endcase
                if (cnt == 5'd5) begin
                    x_rset <= jrs[rpl_r[JW-1:0]];
                    x_base <= chain_t; path <= P_RPL; cnt <= 5'd0; state <= S_EVTW;
                end else cnt <= cnt + 5'd1;
            end
            S_RPL_END: begin
                chain_t <= x_t;
                rpl_r   <= rpl_r + 16'd1;
                // [I-TC-5] replay-time checkpoint: without it a stream that keeps the replay
                // engine busy never creates a checkpoint (in-order events during a replay run on
                // the stale committed ctx and must not checkpoint) and the journal saturates.
                if ({1'b0, rpl_since} + 6'd1 >= {1'b0, m_eff} && (rpl_r + 16'd1) != jtail) begin
                    state <= S_RCK_A;
                end else begin
                    if (rpl_since != 5'd31) rpl_since <= rpl_since + 5'd1;
                    state <= S_RPL_BND;
                end
            end
            S_RCK_A: begin      // shadow ctx (chain state after this unit) -> RF
                bs_we <= 1'b0; bs_rfb <= RF_CTX0; bs_ram <= shd_base + 12'd1; bs_len <= CTXLEN;
                bph <= 1'b0; ret <= S_RCK_B; state <= S_LOAD;
            end
            S_RCK_B: begin      // RF -> checkpoint slot
                bs_we <= 1'b1; bs_rfb <= RF_CTX0; bs_ram <= ckbase(ck_head) + 12'd1; bs_len <= CTXLEN;
                sv_base <= ckbase(ck_head); sv_t <= chain_t; sv_j <= rpl_r; sv_nowords <= 1'b0;
                bph <= 1'b0; ret <= S_RCK_C; state <= S_SAVE;
            end
            S_RCK_C: begin
                ck_t[ck_head] <= chain_t; ck_j[ck_head] <= rpl_r;
                ck_head <= ck_head + {{(CW-1){1'b0}}, 1'b1};
                if (ck_cnt == CKPT_DEPTH[CW:0]) jhead <= ck_j[ck_head + {{(CW-1){1'b0}}, 1'b1}];
                else ck_cnt <= ck_cnt + 1'b1;
                rpl_since <= 5'd0;
                irq_ev_q[1] <= 1'b1;
                state <= S_RPL_BND;
            end
            S_RPL_BND: begin
                // event (if any) was already taken by the capture block above
                if (defer_q) begin
                    if (caught_up) begin
                        state <= S_COMMIT;                    // finish the chain, then reclassify (S_COMMIT_B)
                    end else if (can_retire) begin
                        defer_q <= 1'b0;
                        state   <= S_CLS;                      // room exists now: reclassify from scratch
                    end else state <= S_RPL_NEXT;              // keep draining
                end else if (!take) state <= caught_up ? S_COMMIT : S_RPL_NEXT;
            end
            S_COMMIT: begin
                bs_we <= 1'b1; bs_rfb <= RF_CTX0; bs_ram <= shd_base + 12'd1; bs_len <= CTXLEN;
                sv_base <= shd_base; sv_t <= chain_t; sv_j <= jtail; sv_nowords <= 1'b0;
                bph <= 1'b0; ret <= S_COMMIT_B; state <= S_SAVE;
            end
            S_COMMIT_B: begin
                com_sel    <= ~com_sel;          // commit swap
                rpl_active <= 1'b0;
                t_now      <= chain_t;
                c_replay   <= c_replay + 32'd1;
                cyc_replay <= rpl_cyc;
                irq_ev_q[0] <= 1'b1;
                out_owed   <= 1'b1;
                if (defer_q) begin
                    defer_q <= 1'b0;
                    state <= S_CLS;                // no active replay left: reclassify and run it
                end else state <= S_IDLE;
            end

            // ==============================================================
            // OUTPUT UPDATE (seqlock)
            S_OUT_RD: begin
                if (cnt >= 5'd1) tmp[cnt - 5'd1] <= ram_rdata_i;
                if (cnt == 5'd12) begin cnt <= 5'd0; state <= S_OUT_W; end
                else cnt <= cnt + 5'd1;
            end
            S_OUT_W: begin
                case (cnt)
                    5'd0: begin
                        out_seq <= out_seq + 32'd1;       // odd: update in progress
                        out_committed <= 1'b0;
                        cnt <= 5'd1;
                    end
                    5'd1: begin
                        for (k = 0; k < 6; k = k + 1) begin
                            out_x[k]  <= tmp[k];
                            out_pd[k] <= tmp[6 + k];
                            if (is_nan(tmp[k]) || is_nan(tmp[6 + k])) st_nan <= 1'b1;
                        end
                        out_t <= t_now;
                        cnt <= 5'd2;
                    end
                    default: begin
                        out_seq <= out_seq + 32'd1;       // even: stable
                        out_committed <= ~rpl_active;
                        out_owed <= 1'b0;
                        state <= ret;
                    end
                endcase
            end

            default: state <= S_IDLE;
            endcase
        end
    end

    wire _unused = &{1'b0, event_tarr_i, seq_running_i};
endmodule
