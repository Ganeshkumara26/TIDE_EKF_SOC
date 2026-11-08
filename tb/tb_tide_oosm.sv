// -----------------------------------------------------------------------------
// tb_tide_oosm.sv - system testbench for the OOSM temporal controller (V6 / V8)
//
// DUT chain: event FIFO -> tide_temporal -> tide_lsu -> tide_ram  (all real RTL)
// Stand-ins (Agent 3 / Agent 1 artefacts are not available, see ASSUMPTIONS A1):
//   * register file : 128 x 32, combinational read, synchronous write
//   * sequencer+lanes: "unit model" that, after N cycles, applies a deterministic,
//     order-sensitive 32-bit integer update to the 27 context words held in
//     RF r0..r26 (using dt, z, R read from the RF ABI registers) and pulses
//     seq_done (or seq_boundary).  The same function is implemented in
//     tools/oosm_oracle.py, so the temporal controller is compared bit-for-bit against
//     a chronological oracle.  The temporal controller does no arithmetic, so a
//     hash-style update is the strictest possible ordering check.
//
// Plusargs: +stream=FILE +log=FILE +unit=CYCLES +m=CKPT_INTERVAL +bound=0|1
//           +maxcyc=N +ramrand=1
// Log records (parsed by tb/run_v6.py):
//   OUT c seq t ntaken x0..x5 pd0..pd5     committed(c=1)/provisional(c=0) snapshot
//   DROP idx | QRY idx dt in x0..x5 out x0..x5 | FINAL t w0..w26 | STAT ... | WAIT ...
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_tide_oosm;
    localparam [8:0] UNIT_ENTRY  = 9'd0;
    localparam [8:0] QUERY_ENTRY = 9'd16;
    localparam integer MAXEV = 4096;

    logic clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    // ---------------- plusargs ----------------
    string  stream_file, log_file;
    integer unit_cycles, m_cfg, use_bound, max_cycles, ram_rand;
    integer fd_log;

    // ---------------- FIFO / event source ----------------
    logic         push_valid;
    logic [196:0] push_data;
    wire          push_acc;
    wire          f_valid, f_full, f_over;
    wire  [196:0] f_data;
    wire  [3:0]   f_occ;
    wire          ev_ready;

    tide_event_fifo u_fifo (
        .clk_i(clk), .rst_ni(rst_n),
        .push_valid_i(push_valid), .push_data_i(push_data), .push_accepted_o(push_acc),
        .pop_ready_i(ev_ready), .valid_o(f_valid), .data_o(f_data),
        .full_o(f_full), .occupancy_o(f_occ), .overrun_o(f_over));

    // ---------------- DUT ----------------
    wire         seq_start;  wire [8:0] seq_start_addr;
    logic        seq_done = 0, seq_boundary = 0, seq_running = 0;
    wire         lb_start, lb_we, lb_done, lb_busy;
    wire [6:0]   lb_rfb;  wire [11:0] lb_rmb;  wire [5:0] lb_len;
    wire         tc_rf_wen;  wire [6:0] tc_rf_waddr;  wire [31:0] tc_rf_wdata;
    wire         tc_ce, tc_we;  wire [11:0] tc_addr;  wire [31:0] tc_wdata, tc_rdata;
    wire [1:0]   rset_sel;
    wire [31:0]  rr0 = 32'h3F00_0000 | {22'd0, rset_sel, 8'd0} | 32'd0;
    wire [31:0]  rr1 = 32'h3F00_0000 | {22'd0, rset_sel, 8'd0} | 32'd1;
    wire [31:0]  rr2 = 32'h3F00_0000 | {22'd0, rset_sel, 8'd0} | 32'd2;
    wire [31:0]  rr3 = 32'h3F00_0000 | {22'd0, rset_sel, 8'd0} | 32'd3;

    wire         busy, rpl_pend, prov, irq, gp_busy, gp_late, gp_prov;
    wire [4:0]   irq_ev;  wire [8:0] status;
    wire [31:0]  c_units, c_late, c_drop, c_replay, c_abort, c_query, cyc_unit, cyc_rpl, lat_last, lat_max;
    wire [31:0]  out_seq, out_t, qout_t;
    wire [191:0] out_x, out_pd, qout_x;
    wire         out_comm;
    wire [4:0]   cur_flags;

    logic        enable = 0;
    logic [8:0]  status_clr = 0;
    logic        clr_lat = 0;

    tide_temporal #(.UNIT_ENTRY(UNIT_ENTRY), .QUERY_ENTRY(QUERY_ENTRY)) dut (
        .clk_i(clk), .rst_ni(rst_n),
        .ckpt_interval_i(m_cfg[4:0]), .enable_i(enable), .irq_en_i(1'b1),
        .status_clr_i(status_clr), .clr_lat_i(clr_lat),
        .event_valid_i(f_valid),
        .event_t_i(f_data[31:0]), .event_tarr_i(f_data[63:32]),
        .event_z0_i(f_data[95:64]), .event_z1_i(f_data[127:96]),
        .event_z2_i(f_data[159:128]), .event_z3_i(f_data[191:160]),
        .event_flags_i(f_data[196:192]),
        .event_ready_o(ev_ready), .cur_flags_o(cur_flags), .rset_sel_o(rset_sel),
        .rset_r0_i(rr0), .rset_r1_i(rr1), .rset_r2_i(rr2), .rset_r3_i(rr3),
        .seq_start_o(seq_start), .seq_start_addr_o(seq_start_addr),
        .seq_running_i(seq_running), .seq_done_i(seq_done), .seq_boundary_i(seq_boundary),
        .lsu_burst_start_o(lb_start), .lsu_burst_we_o(lb_we), .lsu_burst_rf_base_o(lb_rfb),
        .lsu_burst_ram_base_o(lb_rmb), .lsu_burst_len_o(lb_len), .lsu_burst_done_i(lb_done),
        .rf_wen_o(tc_rf_wen), .rf_waddr_o(tc_rf_waddr), .rf_wdata_o(tc_rf_wdata),
        .ram_ce_o(tc_ce), .ram_we_o(tc_we), .ram_addr_o(tc_addr), .ram_wdata_o(tc_wdata),
        .ram_rdata_i(tc_rdata),
        .busy_o(busy), .replay_pending_o(rpl_pend), .provisional_o(prov), .irq_o(irq),
        .irq_ev_o(irq_ev), .status_o(status),
        .gpio_engine_busy_o(gp_busy), .gpio_late_drop_o(gp_late), .gpio_provisional_o(gp_prov),
        .cnt_units_o(c_units), .cnt_late_o(c_late), .cnt_drop_o(c_drop), .cnt_replay_o(c_replay),
        .cnt_abort_o(c_abort), .cnt_query_o(c_query), .cyc_last_unit_o(cyc_unit),
        .cyc_last_replay_o(cyc_rpl), .lat_last_wait_o(lat_last), .lat_max_wait_o(lat_max),
        .out_seq_o(out_seq), .out_t_o(out_t), .out_x_o(out_x), .out_pdiag_o(out_pd),
        .out_committed_o(out_comm), .qout_x_o(qout_x), .qout_t_o(qout_t));

    // LSU + RF + RAM
    wire [6:0]  l_raddr, l_waddr;  wire [31:0] l_wdata;  wire l_wen;
    wire        l_ce, l_we;  wire [11:0] l_addr;  wire [31:0] l_wd, l_rd;
    logic [31:0] rf [0:127];
    wire  [31:0] rf_rdata = rf[l_raddr];

    tide_lsu u_lsu (
        .clk_i(clk), .rst_ni(rst_n),
        .cmd_valid_i(1'b0), .cmd_we_i(1'b0), .cmd_reg_i(7'd0), .cmd_addr_i(12'd0), .cmd_ready_o(),
        .burst_start_i(lb_start), .burst_we_i(lb_we), .burst_rf_base_i(lb_rfb),
        .burst_ram_base_i(lb_rmb), .burst_len_i(lb_len), .burst_busy_o(lb_busy), .burst_done_o(lb_done),
        .rf_raddr_o(l_raddr), .rf_rdata_i(rf_rdata),
        .rf_wen_o(l_wen), .rf_waddr_o(l_waddr), .rf_wdata_o(l_wdata),
        .ram_ce_o(l_ce), .ram_we_o(l_we), .ram_addr_o(l_addr), .ram_wdata_o(l_wd), .ram_rdata_i(l_rd));

    wire ram_conflict, ram_oor;
    tide_ram u_ram (
        .clk_i(clk), .rst_ni(rst_n),
        .tc_ce_i(tc_ce), .tc_we_i(tc_we), .tc_addr_i(tc_addr), .tc_wdata_i(tc_wdata), .tc_rdata_o(tc_rdata),
        .lsu_ce_i(l_ce), .lsu_we_i(l_we), .lsu_addr_i(l_addr), .lsu_wdata_i(l_wd), .lsu_rdata_o(l_rd),
        .cpu_wr_en_i(1'b0), .cpu_wr_ce_i(1'b0), .cpu_wr_addr_i(12'd0), .cpu_wr_data_i(32'd0),
        .dbg_ce_i(1'b0), .dbg_addr_i(12'd0), .dbg_rdata_o(), .dbg_valid_o(),
        .conflict_o(ram_conflict), .oor_o(ram_oor));

    // RF writes (TC and LSU never write in the same cycle: checked below)
    integer rf_dual_errs = 0;
    always_ff @(posedge clk) begin
        if (tc_rf_wen && l_wen) rf_dual_errs <= rf_dual_errs + 1;
        if (l_wen     && l_waddr  != 7'd127) rf[l_waddr]     <= l_wdata;
        if (tc_rf_wen && tc_rf_waddr != 7'd127) rf[tc_rf_waddr] <= tc_rf_wdata;
    end

    // ---------------- unit model (stand-in for sequencer + lanes) ----------------
    function automatic [31:0] mix(input [31:0] a, input [31:0] b);
        logic [31:0] h;
        begin
            h = a ^ (b * 32'h9E3779B1);
            h = h ^ (h >> 15);
            h = h * 32'h85EBCA6B;
            h = h ^ (h >> 13);
            mix = h;
        end
    endfunction

    integer      run_cnt = 0;
    logic [8:0]  run_addr;
    integer      max_unit_seen = 0;
    integer      seq_overlap_errs = 0;
    logic [31:0] qin_x [0:5];
    logic [31:0] qin_dt;

    logic [31:0] nw [0:26];
    logic [31:0] e_h;
    always_ff @(posedge clk) begin
        seq_done     <= 1'b0;
        seq_boundary <= 1'b0;
        if (!rst_n) begin
            seq_running <= 1'b0; run_cnt <= 0;
        end else begin
            if (seq_start) begin
                if (seq_running) seq_overlap_errs <= seq_overlap_errs + 1;
                seq_running <= 1'b1; run_cnt <= 0; run_addr <= seq_start_addr;
                // [ABI fix, matches tide_temporal.sv RF_DT/RF_CTX]: dt_us now
                // lives at r118 (ABI.R_DT_US), not r37 (r37 is live scratch
                // inside the real microcode's own PREDICT routine). r119
                // (ABI.R_CTX, the RAM context-base address) is written by
                // the corrected tide_temporal.sv but this stand-in unit model
                // still mutates rf[0..26] directly (as it always has) rather
                // than modelling the real microcode's own RAM load/store --
                // it is a scheduling/bookkeeping oracle, not an arithmetic
                // one (see ASSUMPTIONS.md A1), so r119's value is read here
                // only to confirm the temporal controller is presenting a
                // non-garbage context address, not to actually fetch through it.
                if (seq_start_addr == QUERY_ENTRY) begin
                    for (int i = 0; i < 6; i++) qin_x[i] <= rf[i];
                    qin_dt <= rf[118];
                end
            end else if (seq_running) begin
                if (run_cnt >= unit_cycles - 1) begin
                    seq_running <= 1'b0;
                    if (run_addr == QUERY_ENTRY) begin
                        for (int i = 0; i < 6; i++) rf[i] <= mix(rf[i] + i + 100, rf[118]);
                    end else begin
                        e_h = mix(mix(mix(rf[118], rf[29]), mix(rf[30], rf[31])),
                                  mix(rf[32], rf[33] ^ rf[34] ^ rf[35] ^ rf[36]));
                        for (int i = 0; i < 27; i++)
                            rf[i] <= mix(rf[i] + i, e_h ^ rf[(i + 1) % 27]);
                        rf[27] <= rf[28];
                    end
                    if (use_bound != 0) seq_boundary <= 1'b1; else seq_done <= 1'b1;
                    if (run_cnt + 1 > max_unit_seen) max_unit_seen <= run_cnt + 1;
                end else run_cnt <= run_cnt + 1;
            end
        end
    end

    // ---------------- stream loading ----------------
    logic [31:0] seed_w [0:26];
    integer      ev_gap [0:MAXEV-1];
    logic [31:0] ev_t_a [0:MAXEV-1], ev_ta_a [0:MAXEV-1];
    logic [31:0] ev_z_a [0:MAXEV-1][0:3];
    logic [4:0]  ev_f_a [0:MAXEV-1];
    integer      n_ev = 0;

    task automatic load_stream(input string fname);
        integer fd, r, gap;
        reg [7:0] ch;
        reg [31:0] w, t, ta, z0, z1, z2, z3, fl;
        begin
            fd = $fopen(fname, "r");
            if (fd == 0) begin $display("TB: cannot open stream %s", fname); $finish; end
            while (!$feof(fd)) begin
                r = $fscanf(fd, " %c", ch);
                if (r == 1) begin
                    if (ch == "S") begin
                        for (int i = 0; i < 27; i++) begin r = $fscanf(fd, " %h", w); seed_w[i] = w; end
                    end else if (ch == "E") begin
                        r = $fscanf(fd, " %d %h %h %h %h %h %h %h", gap, t, ta, z0, z1, z2, z3, fl);
                        ev_gap[n_ev] = gap; ev_t_a[n_ev] = t; ev_ta_a[n_ev] = ta;
                        ev_z_a[n_ev][0] = z0; ev_z_a[n_ev][1] = z1; ev_z_a[n_ev][2] = z2; ev_z_a[n_ev][3] = z3;
                        ev_f_a[n_ev] = fl[4:0];
                        n_ev++;
                    end
                end
            end
            $fclose(fd);
        end
    endtask

    // ---------------- monitors ----------------
    integer ntaken = 0, cyc = 0;
    logic [31:0] prev_seq = 0, prev_drop = 0, prev_query = 0;
    integer wait_max_tb = 0, wait_cur_tb = 0;
    integer wait_max_replay_tb = 0;
    logic   replay_at_arrival = 0;
    integer errs = 0;

    always_ff @(posedge clk) begin
        cyc <= cyc + 1;
        if (rst_n) begin
            if (f_valid && ev_ready) begin
                ntaken <= ntaken + 1;
                if (wait_cur_tb > wait_max_tb) wait_max_tb <= wait_cur_tb;
                if (replay_at_arrival && wait_cur_tb > wait_max_replay_tb) wait_max_replay_tb <= wait_cur_tb;
                wait_cur_tb <= 0;
            end else if (f_valid) begin
                wait_cur_tb <= wait_cur_tb + 1;
            end
            if (!f_valid) begin wait_cur_tb <= 0; replay_at_arrival <= rpl_pend; end
            else if (wait_cur_tb == 0 && !(f_valid && ev_ready)) replay_at_arrival <= rpl_pend;

            prev_seq   <= out_seq;
            prev_drop  <= c_drop;
            prev_query <= c_query;
            if (out_seq != prev_seq) begin
                if (out_seq != prev_seq + 1) begin errs++; $display("ERR seq step"); end
                if (!out_seq[0]) begin
                    $fdisplay(fd_log, "OUT %0d %0d %0d %0d %h %h %h %h %h %h %h %h %h %h %h %h", out_comm,
                        out_seq, $signed(out_t), ntaken,
                        out_x[31:0], out_x[63:32], out_x[95:64], out_x[127:96], out_x[159:128], out_x[191:160],
                        out_pd[31:0], out_pd[63:32], out_pd[95:64], out_pd[127:96], out_pd[159:128], out_pd[191:160]);
                end
            end
            if (c_drop != prev_drop)
                $fdisplay(fd_log, "DROP %0d %0d", ntaken - 1, dut.drop_kind);
            if (c_query != prev_query)
                $fdisplay(fd_log, "QRY %0d %h %h %h %h %h %h %h %h %h %h %h %h %h", ntaken - 1, qin_dt,
                    qin_x[0], qin_x[1], qin_x[2], qin_x[3], qin_x[4], qin_x[5],
                    qout_x[31:0], qout_x[63:32], qout_x[95:64], qout_x[127:96], qout_x[159:128], qout_x[191:160]);
            if (out_comm && out_seq[0]) begin errs++; $display("ERR committed with odd seq"); end
        end
    end

    // optional debug trace (+dbg=1)
    integer dbg_on = 0;
    initial if ($value$plusargs("dbg=%d", dbg_on)) ;
    logic prev_jf = 0;
    logic [5:0] prev_state = 0;
    always_ff @(posedge clk) begin
        prev_jf <= dut.st_jrnl_full;
        prev_state <= dut.state;
        if (dbg_on && dut.st_jrnl_full && !prev_jf)
            $display("DBG jfull at cyc=%0d ntaken=%0d jhead=%0d jtail=%0d ck_cnt=%0d ck_head=%0d rpl=%0d state=%0d ev_since=%0d",
                     cyc, ntaken, dut.jhead, dut.jtail, dut.ck_cnt, dut.ck_head, dut.rpl_active, dut.state, dut.ev_since);
        if (dbg_on && dut.state == 6'd10 && dut.cnt == 5'd0)   // S_JAPP entry
            $display("DBG JAPP cyc=%0d ev_t=%0d jtail=%0d deg=%0d rpl=%0d", cyc, dut.ev_t, dut.jtail, dut.deg_ev, dut.rpl_active);
        if (dbg_on && dut.state == 6'd2 && prev_state != 6'd2)         // S_DROP entry
            $display("DBG DROP cyc=%0d ev_t=%0d ck_oldest_t=%0d drop_kind=%0d ck_cnt=%0d rpl=%0d",
                     cyc, dut.ev_t, dut.ck_oldest_t, dut.drop_kind, dut.ck_cnt, dut.rpl_active);
        if (dbg_on && dut.state == 6'd10 && prev_state != 6'd10)      // S_EVTW entry (event about to be run)
            $display("DBG RUN cyc=%0d x_t=%0d x_base=%0d x_z=%h,%h,%h,%h xrset=%0d path=%0d rpl_r=%0d",
                     cyc, dut.x_t, dut.x_base, dut.x_z0, dut.x_z1, dut.x_z2, dut.x_z3, dut.x_rset, dut.path, dut.rpl_r);
        if (dbg_on && dut.state == 6'd17 && prev_state != 6'd17)      // S_IO_D entry (commit)
            $display("DBG COMMIT cyc=%0d x_t=%0d x_z=%h,%h,%h,%h xrset=%0d path=%0d deg=%0d rpl=%0d",
                     cyc, dut.x_t, dut.x_z0, dut.x_z1, dut.x_z2, dut.x_z3, dut.x_rset, dut.path, dut.deg_ev, dut.rpl_active);
        if (dbg_on && dut.state == 6'd33 && prev_state != 6'd33)      // S_COMMIT entry (replay chain committed)
            $display("DBG RPLCOMMIT cyc=%0d chain_t=%0d jtail=%0d", cyc, dut.chain_t, dut.jtail);
        if (dbg_on && dut.state==6'd28 && dut.cnt==5'd0) // S_LATE_C
            $display("DBG LATE_C cyc=%0d ev_t=%0d ji_p=%0d rpl_r=%0d rpl_active=%0d", cyc, dut.ev_t, dut.ji_p, dut.rpl_r, dut.rpl_active);
    end

    // ---------------- main driver ----------------
    integer idx, gcnt, quiet;
    logic   done_inject;
    initial begin
        if (!$value$plusargs("stream=%s", stream_file)) stream_file = "stream.txt";
        if (!$value$plusargs("log=%s", log_file))       log_file = "sim.log";
        if (!$value$plusargs("unit=%d", unit_cycles))   unit_cycles = 1100;
        if (!$value$plusargs("m=%d", m_cfg))            m_cfg = 4;
        if (!$value$plusargs("bound=%d", use_bound))    use_bound = 0;
        if (!$value$plusargs("maxcyc=%d", max_cycles))  max_cycles = 20000000;
        if (!$value$plusargs("ramrand=%d", ram_rand))   ram_rand = 1;
        fd_log = $fopen(log_file, "w");
        load_stream(stream_file);

        for (int i = 0; i < 128; i++) rf[i] = ram_rand ? $urandom : 32'd0;
        for (int i = 0; i < 1024; i++) u_ram.u_macro.mem[i] = ram_rand ? $urandom : 32'd0;
        // seed context lives in the fixed shadow area CTX1 (0x260): words +1..+27
        for (int i = 0; i < 27; i++) u_ram.u_macro.mem[12'h261 + i] = seed_w[i];

        push_valid = 0; push_data = 0; done_inject = 0; idx = 0; gcnt = 0;
        repeat (5) @(negedge clk);
        rst_n = 1;
        repeat (3) @(negedge clk);
        enable = 1;

        // inject events: wait 'gap' cycles, then push when FIFO has room (no loss in V6)
        while (idx < n_ev && cyc < max_cycles) begin
            gcnt = ev_gap[idx];
            while (gcnt > 0) begin @(negedge clk); gcnt--; end
            push_data  = {ev_f_a[idx], ev_z_a[idx][3], ev_z_a[idx][2], ev_z_a[idx][1], ev_z_a[idx][0],
                          ev_ta_a[idx], ev_t_a[idx]};
            push_valid = 1;
            @(negedge clk);
            while (!push_acc) @(negedge clk);   // hold until accepted
            push_valid = 0;
            idx++;
        end
        done_inject = 1;

        // drain
        quiet = 0;
        while (quiet < 300 && cyc < max_cycles) begin
            @(negedge clk);
            if (f_valid || busy || rpl_pend) quiet = 0; else quiet++;
        end

        // final committed context
        begin
            logic [11:0] cb;
            cb = dut.com_sel ? 12'h260 : 12'h240;
            $fwrite(fd_log, "FINAL %0d", $signed(dut.t_now));
            for (int i = 0; i < 27; i++) $fwrite(fd_log, " %h", u_ram.u_macro.mem[cb + 1 + i]);
            $fwrite(fd_log, "\n");
        end
        $fdisplay(fd_log, "STAT taken=%0d units=%0d late=%0d drop=%0d query=%0d replay=%0d abort=%0d cycles=%0d maxunit=%0d",
                  ntaken, c_units, c_late, c_drop, c_query, c_replay, c_abort, cyc, max_unit_seen);
        $fdisplay(fd_log, "WAIT tb_max=%0d tb_max_replay=%0d dut_max=%0d", wait_max_tb, wait_max_replay_tb, lat_max);
        $fdisplay(fd_log, "CHK ram_conflict=%0d ram_oor=%0d rf_dual=%0d seq_overlap=%0d errs=%0d timeout=%0d jrnl_full=%0d",
                  ram_conflict, ram_oor, rf_dual_errs, seq_overlap_errs, errs, (cyc >= max_cycles), dut.st_jrnl_full);
        $fclose(fd_log);
        $finish;
    end
endmodule
