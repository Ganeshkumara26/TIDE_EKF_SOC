// -----------------------------------------------------------------------------
// tb_engine_oosm.sv -- out-of-sequence-measurement regression on the REAL engine
// (tide_engine: temporal + sequencer + RF + LSU + RAM + real FP32 lanes + microcode),
// replacing the stand-in "unit model" of 04_temporal_controller/tb/tb_tide_oosm.sv.
// Log format identical (see 04_temporal_controller/tb/run_v6.py); the oracle is
// tools/engine_oosm.py (tide_sim.py == golden C model, bit-exact).
// Plusargs: +stream=FILE +log=FILE +m=CKPT +maxcyc=N
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_engine_oosm #(parameter integer TB_C = 8, parameter integer TB_J = 64);
    localparam integer MAXEV = 4096;
    logic clk = 0, rst_n = 1;
    
    // RAM backdoor (works for the behavioural macro and, with +define+TIDE_MACROS, for the 4x fakeram7_256x32 banks)
    function automatic [31:0] pk(input integer w);
`ifdef TIDE_MACROS
        case (w[9:8]) 2'd0: pk = dut.u_ram.u_macro.g_bank[0].u_m.mem[w[7:0]]; 2'd1: pk = dut.u_ram.u_macro.g_bank[1].u_m.mem[w[7:0]];
                      2'd2: pk = dut.u_ram.u_macro.g_bank[2].u_m.mem[w[7:0]]; default: pk = dut.u_ram.u_macro.g_bank[3].u_m.mem[w[7:0]]; endcase
`else
        pk = dut.u_ram.u_macro.mem[w[9:0]];
`endif
    endfunction
    task automatic pw(input integer w, input [31:0] v);
`ifdef TIDE_MACROS
        case (w[9:8]) 2'd0: dut.u_ram.u_macro.g_bank[0].u_m.mem[w[7:0]] = v; 2'd1: dut.u_ram.u_macro.g_bank[1].u_m.mem[w[7:0]] = v;
                      2'd2: dut.u_ram.u_macro.g_bank[2].u_m.mem[w[7:0]] = v; default: dut.u_ram.u_macro.g_bank[3].u_m.mem[w[7:0]] = v; endcase
`else
        dut.u_ram.u_macro.mem[w[9:0]] = v;
`endif
    endtask
initial begin #1 rst_n = 0; end
    always #5 clk = ~clk;

    string  stream_file, log_file;
    integer unit_cycles = 0, m_cfg, use_bound = 0, max_cycles, ram_rand;
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

    // R-set table (must match tools/engine_oosm.py RSETS)
    wire [1:0]  rset_sel;
    logic [31:0] rr0, rr1, rr2, rr3;
    always_comb begin
        case (rset_sel)
            2'd0: begin rr0 = 32'h41c80000; rr1 = 32'h38d1b717; rr2 = 32'h38d1b717; rr3 = 32'h3f800000; end
            2'd1: begin rr0 = 32'h42480000; rr1 = 32'h3951b717; rr2 = 32'h3951b717; rr3 = 32'h40000000; end
            2'd2: begin rr0 = 32'h42960000; rr1 = 32'h399d4952; rr2 = 32'h399d4952; rr3 = 32'h40400000; end
            default: begin rr0 = 32'h42c80000; rr1 = 32'h39d1b717; rr2 = 32'h39d1b717; rr3 = 32'h40800000; end
        endcase
    end

    wire         busy, rpl_pend, prov, irq, gp_busy, gp_late, gp_prov;
    wire [4:0]   irq_ev;  wire [8:0] status;
    wire [31:0]  c_units, c_late, c_drop, c_replay, c_abort, c_query, cyc_unit, cyc_rpl, lat_last, lat_max;
    wire [31:0]  out_seq, out_t, qout_t;
    wire [191:0] out_x, out_pd, qout_x;
    wire         out_comm;
    wire [4:0]   cur_flags;
    logic        enable = 0;
    logic        uc_we = 0; logic [8:0] uc_addr = 0; logic [1:0] uc_word = 0; logic [31:0] uc_data = 0;
    logic [95:0] ucw [0:511];
    wire         ram_conflict, ram_oor, rf_conflict;

    tide_engine #(.UNIT_ENTRY(9'd0), .QUERY_ENTRY(9'd471), .CKPT_DEPTH(TB_C), .JRNL_DEPTH(TB_J)) dut (
        .clk_i(clk), .rst_ni(rst_n),
        .ckpt_interval_i(m_cfg[4:0]), .enable_i(enable), .irq_en_i(1'b1),
        .status_clr_i(9'd0), .clr_lat_i(1'b0),
        .event_valid_i(f_valid),
        .event_t_i(f_data[31:0]),
        .event_z0_i(f_data[95:64]), .event_z1_i(f_data[127:96]),
        .event_z2_i(f_data[159:128]), .event_z3_i(f_data[191:160]),
        .event_flags_i(f_data[196:192]),
        .event_ready_o(ev_ready), .cur_flags_o(cur_flags), .rset_sel_o(rset_sel),
        .rset_r0_i(rr0), .rset_r1_i(rr1), .rset_r2_i(rr2), .rset_r3_i(rr3),
        .qp_i(32'h3f800000), .qv_i(32'h3dcccccd), .dt0inv_i(32'h42c80000),
        .uc_we_i(uc_we), .uc_addr_i(uc_addr), .uc_word_i(uc_word), .uc_data_i(uc_data), .uc_rd_addr_i(9'd0), .uc_rd_data_o(),
        .ram_wr_ce_i(1'b0), .ram_wr_addr_i(12'd0), .ram_wr_data_i(32'd0),
        .ram_dbg_ce_i(1'b0), .ram_dbg_addr_i(12'd0), .ram_dbg_rdata_o(), .ram_dbg_valid_o(),
        .busy_o(busy), .replay_pending_o(rpl_pend), .provisional_o(prov), .irq_o(irq),
        .irq_ev_o(irq_ev), .status_o(status),
        .gpio_engine_busy_o(gp_busy), .gpio_late_drop_o(gp_late), .gpio_provisional_o(gp_prov),
        .cnt_units_o(c_units), .cnt_late_o(c_late), .cnt_drop_o(c_drop), .cnt_replay_o(c_replay),
        .cnt_abort_o(c_abort), .cnt_query_o(c_query), .cyc_last_unit_o(cyc_unit),
        .cyc_last_replay_o(cyc_rpl), .lat_last_wait_o(lat_last), .lat_max_wait_o(lat_max),
        .out_seq_o(out_seq), .out_t_o(out_t), .out_x_o(out_x), .out_pdiag_o(out_pd),
        .out_committed_o(out_comm), .qout_x_o(qout_x), .qout_t_o(qout_t),
        .ram_conflict_o(ram_conflict), .ram_oor_o(ram_oor), .rf_conflict_o(rf_conflict));

    integer rf_dual_errs; integer seq_overlap_errs = 0; integer max_unit_seen = 0;
    always_comb rf_dual_errs = rf_conflict ? 1 : 0;
    logic [31:0] qin_x [0:5];
    logic [31:0] qin_dt;

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
                $fdisplay(fd_log, "DROP %0d %0d", ntaken - 1, dut.u_tc.drop_kind);
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
        prev_jf <= dut.u_tc.st_jrnl_full;
        prev_state <= dut.u_tc.state;
        if (dut.u_tc.state == 6'd19) begin   // S_QRY_A: snapshot committed ctx + dt for the oracle
            qin_dt <= dut.u_tc.ev_t - dut.u_tc.t_now;
            for (int k = 0; k < 6; k++) qin_x[k] <= pk(dut.u_tc.com_base[9:0] + 1 + k);
        end
        if (dbg_on && dut.u_tc.st_jrnl_full && !prev_jf)
            $display("DBG jfull at cyc=%0d ntaken=%0d jhead=%0d jtail=%0d ck_cnt=%0d ck_head=%0d rpl=%0d state=%0d ev_since=%0d",
                     cyc, ntaken, dut.u_tc.jhead, dut.u_tc.jtail, dut.u_tc.ck_cnt, dut.u_tc.ck_head, dut.u_tc.rpl_active, dut.u_tc.state, dut.u_tc.ev_since);
        if (dbg_on && dut.u_tc.state == 6'd10 && dut.u_tc.cnt == 5'd0)   // S_JAPP entry
            $display("DBG JAPP cyc=%0d ev_t=%0d jtail=%0d deg=%0d rpl=%0d", cyc, dut.u_tc.ev_t, dut.u_tc.jtail, dut.u_tc.deg_ev, dut.u_tc.rpl_active);
        if (dbg_on && dut.u_tc.state == 6'd2 && prev_state != 6'd2)         // S_DROP entry
            $display("DBG DROP cyc=%0d ev_t=%0d ck_oldest_t=%0d drop_kind=%0d ck_cnt=%0d rpl=%0d",
                     cyc, dut.u_tc.ev_t, dut.u_tc.ck_oldest_t, dut.u_tc.drop_kind, dut.u_tc.ck_cnt, dut.u_tc.rpl_active);
        if (dbg_on && dut.u_tc.state == 6'd10 && prev_state != 6'd10)
            $display("DBG CTXRAM cyc=%0d com=%h shd=%h : com[0..2]=%h %h %h shd[0..2]=%h %h %h ckpt_newest=%h",
              cyc, dut.u_tc.com_base, dut.u_tc.shd_base,
              pk(dut.u_tc.com_base[9:0]), pk(dut.u_tc.com_base[9:0]+1), pk(dut.u_tc.com_base[9:0]+2),
              pk(dut.u_tc.shd_base[9:0]), pk(dut.u_tc.shd_base[9:0]+1), pk(dut.u_tc.shd_base[9:0]+2),
              pk({dut.u_tc.newest_idx,5'd0}+1));
        if (dbg_on && dut.u_tc.state == 6'd10 && prev_state != 6'd10)      // S_EVTW entry (event about to be run)
            $display("DBG RUN cyc=%0d x_t=%0d x_base=%0d x_z=%h,%h,%h,%h xrset=%0d path=%0d rpl_r=%0d",
                     cyc, dut.u_tc.x_t, dut.u_tc.x_base, dut.u_tc.x_z0, dut.u_tc.x_z1, dut.u_tc.x_z2, dut.u_tc.x_z3, dut.u_tc.x_rset, dut.u_tc.path, dut.u_tc.rpl_r);
        if (dbg_on && dut.u_tc.state == 6'd17 && prev_state != 6'd17)      // S_IO_D entry (commit)
            $display("DBG COMMIT cyc=%0d x_t=%0d x_z=%h,%h,%h,%h xrset=%0d path=%0d deg=%0d rpl=%0d",
                     cyc, dut.u_tc.x_t, dut.u_tc.x_z0, dut.u_tc.x_z1, dut.u_tc.x_z2, dut.u_tc.x_z3, dut.u_tc.x_rset, dut.u_tc.path, dut.u_tc.deg_ev, dut.u_tc.rpl_active);
        if (dbg_on && dut.u_tc.state == 6'd33 && prev_state != 6'd33)      // S_COMMIT entry (replay chain committed)
            $display("DBG RPLCOMMIT cyc=%0d chain_t=%0d jtail=%0d", cyc, dut.u_tc.chain_t, dut.u_tc.jtail);
        if (dbg_on && dut.u_tc.state==6'd28 && dut.u_tc.cnt==5'd0) // S_LATE_C
            $display("DBG LATE_C cyc=%0d ev_t=%0d ji_p=%0d rpl_r=%0d rpl_active=%0d", cyc, dut.u_tc.ev_t, dut.u_tc.ji_p, dut.u_tc.rpl_r, dut.u_tc.rpl_active);
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

        for (int i = 0; i < 1024; i++) pw(i, ram_rand ? $urandom : 32'd0);
        // seed context lives in the fixed shadow area CTX1 (0x260): words +1..+27
        for (int i = 0; i < 27; i++) pw(12'h261 + i, seed_w[i]);

        push_valid = 0; push_data = 0; done_inject = 0; idx = 0; gcnt = 0;
        repeat (5) @(negedge clk);
        rst_n = 1;
        repeat (3) @(negedge clk);
        // microcode load through the CPU port (3 writes per 96-bit word), engine disabled
        $readmemh("build/ucode.hex", ucw);
        for (int a = 0; a < 492; a++)
            for (int k = 0; k < 3; k++) begin
                uc_we = 1; uc_addr = a[8:0]; uc_word = k[1:0]; uc_data = ucw[a][32*k +: 32]; @(negedge clk);
            end
        uc_we = 0;
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
            cb = dut.u_tc.com_sel ? 12'h260 : 12'h240;
            $fwrite(fd_log, "FINAL %0d", $signed(dut.u_tc.t_now));
            for (int i = 0; i < 27; i++) $fwrite(fd_log, " %h", pk(cb + 1 + i));
            $fwrite(fd_log, "\n");
        end
        $fdisplay(fd_log, "STAT taken=%0d units=%0d late=%0d drop=%0d query=%0d replay=%0d abort=%0d cycles=%0d maxunit=%0d",
                  ntaken, c_units, c_late, c_drop, c_query, c_replay, c_abort, cyc, max_unit_seen);
        $fdisplay(fd_log, "WAIT tb_max=%0d tb_max_replay=%0d dut_max=%0d", wait_max_tb, wait_max_replay_tb, lat_max);
        $fdisplay(fd_log, "CHK ram_conflict=%0d ram_oor=%0d rf_dual=%0d seq_overlap=%0d errs=%0d timeout=%0d jrnl_full=%0d",
                  ram_conflict, ram_oor, rf_dual_errs, seq_overlap_errs, errs, (cyc >= max_cycles), dut.u_tc.st_jrnl_full);
        $fclose(fd_log);
        $finish;
    end
endmodule
