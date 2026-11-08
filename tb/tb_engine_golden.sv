// End-to-end: real tide_engine (real temporal/sequencer/RF/LSU/RAM/lanes/ROM)
// vs the C golden model's unit vectors. Bit-exact compare of out_x / out_pdiag.
`timescale 1ns/1ps
module tb_engine_golden;
    logic clk = 0, rst_ni = 1;
    initial begin #1 rst_ni = 0; end
    always #5 clk = ~clk;

    logic        enable = 0, event_valid = 0;
    logic [31:0] ev_t, ev_z0, ev_z1, ev_z2, ev_z3, r0, r1, r2, r3, qp, qv, dt0;
    logic [4:0]  ev_flags = 0;
    logic        ev_ready, busy;
    logic        uc_we = 0; logic [8:0] uc_addr = 0; logic [1:0] uc_word = 0; logic [31:0] uc_data = 0;
    logic        rw_ce = 0; logic [11:0] rw_addr = 0; logic [31:0] rw_data = 0;
    logic [31:0] cnt_units, out_seq, out_t, dbgd; logic [191:0] out_x, out_pd; logic [8:0] status;
    logic        ram_conflict, ram_oor, rf_conflict;
    logic [4:0]  irq_ev; logic irq;

    tide_engine dut (
        .clk_i(clk), .rst_ni(rst_ni),
        .ckpt_interval_i(5'd4), .enable_i(enable), .irq_en_i(1'b1), .status_clr_i(9'd0), .clr_lat_i(1'b0),
        .event_valid_i(event_valid), .event_t_i(ev_t),
        .event_z0_i(ev_z0), .event_z1_i(ev_z1), .event_z2_i(ev_z2), .event_z3_i(ev_z3),
        .event_flags_i(ev_flags), .event_ready_o(ev_ready), .cur_flags_o(), .rset_sel_o(),
        .rset_r0_i(r0), .rset_r1_i(r1), .rset_r2_i(r2), .rset_r3_i(r3),
        .qp_i(qp), .qv_i(qv), .dt0inv_i(dt0),
        .uc_we_i(uc_we), .uc_addr_i(uc_addr), .uc_word_i(uc_word), .uc_data_i(uc_data), .uc_rd_addr_i(9'd0), .uc_rd_data_o(),
        .ram_wr_ce_i(rw_ce), .ram_wr_addr_i(rw_addr), .ram_wr_data_i(rw_data),
        .ram_dbg_ce_i(1'b0), .ram_dbg_addr_i(12'd0), .ram_dbg_rdata_o(dbgd), .ram_dbg_valid_o(),
        .busy_o(busy), .replay_pending_o(), .provisional_o(), .irq_o(irq), .irq_ev_o(irq_ev), .status_o(status),
        .gpio_engine_busy_o(), .gpio_late_drop_o(), .gpio_provisional_o(),
        .cnt_units_o(cnt_units), .cnt_late_o(), .cnt_drop_o(), .cnt_replay_o(), .cnt_abort_o(), .cnt_query_o(),
        .cyc_last_unit_o(), .cyc_last_replay_o(), .lat_last_wait_o(), .lat_max_wait_o(),
        .out_seq_o(out_seq), .out_t_o(out_t), .out_x_o(out_x), .out_pdiag_o(out_pd), .out_committed_o(),
        .qout_x_o(), .qout_t_o(),
        .ram_conflict_o(ram_conflict), .ram_oor_o(ram_oor), .rf_conflict_o(rf_conflict)
    );

    `include "tb/dbg_probe.svh"
    logic [95:0] ucw [0:511];
    integer fd, rc, nsc = 0, nev = 0, nfail = 0, nbitfail = 0, i, to, nuc;
    reg [255:0] tag;
    reg [31:0] w [0:27];
    reg [31:0] ex [0:27];
    reg [31:0] c1, c2, c3;
    integer pdi [0:5];

    task automatic push(input [4:0] fl);
        begin
            @(posedge clk); #1; ev_flags = fl;
            to = 0; while (!ev_ready && to < 2000) begin @(posedge clk); #1; to++; end
            event_valid = 1; @(posedge clk); #1; event_valid = 0;
        end
    endtask

    task automatic wait_idle;   // wait until engine drained
        begin
            to = 0;
            repeat (20) @(posedge clk);
            while ((busy || !ev_ready) && to < 200000) begin @(posedge clk); to++; end
            repeat (5) @(posedge clk);
            if (to >= 200000) begin $display("TIMEOUT busy=%b", busy); nfail++; end
        end
    endtask

    initial begin
        pdi[0]=0; pdi[1]=2; pdi[2]=5; pdi[3]=9; pdi[4]=14; pdi[5]=20;
        $readmemh("build/ucode.hex", ucw);
        r0 = 0; r1 = 0; r2 = 0; r3 = 0; ev_t = 0; ev_z0 = 0; ev_z1 = 0; ev_z2 = 0; ev_z3 = 0;
        qp = 0; qv = 0; dt0 = 0;
        repeat (4) @(posedge clk); rst_ni = 1; repeat (3) @(posedge clk);
        // ---- load microcode through the CPU port (3 writes / word) ----
        for (nuc = 0; nuc < 492; nuc++)
            for (i = 0; i < 3; i++) begin
                #1; uc_we = 1; uc_addr = nuc[8:0]; uc_word = i[1:0]; uc_data = ucw[nuc][32*i +: 32];
                @(posedge clk);
            end
        #1; uc_we = 0;
        fd = $fopen("build/stim.txt", "r");
        while (!$feof(fd)) begin
            rc = $fscanf(fd, "%s", tag);
            if (rc != 1) break;
            if (tag == "B") begin
                rc = $fscanf(fd, "%h %h %h", c1, c2, c3);
                qp = c1; qv = c2; dt0 = c3; nsc++;
                // fresh engine per scenario
                enable = 0; rst_ni = 0; repeat (3) @(posedge clk); #1; rst_ni = 1; repeat (3) @(posedge clk);
                #1; enable = 1;
            end else if (tag == "I") begin
                for (i = 0; i < 28; i++) rc = $fscanf(fd, "%h", w[i]);
                // seed ctx1 (0x260): words +1..+27 (+0 t_last comes from the INIT event time)
                for (i = 1; i < 28; i++) begin
                    #1; rw_ce = 1; rw_addr = 12'h260 + i[11:0]; rw_data = w[i]; @(posedge clk);
                end
                #1; rw_ce = 0;
                ev_t = w[0]; ev_z0 = 0; ev_z1 = 0; ev_z2 = 0; ev_z3 = 0; r0 = 0; r1 = 0; r2 = 0; r3 = 0;
                push(5'b10000);
                wait_idle;
            end else if (tag == "E") begin
                rc = $fscanf(fd, "%h %h %h %h %h %h %h %h %h %h", ev_t, ev_z0, ev_z1, ev_z2, ev_z3, r0, r1, r2, r3, c1);
                push(5'b00000);
                wait_idle;
                nev++;
            end else if (tag == "X") begin
                for (i = 0; i < 28; i++) rc = $fscanf(fd, "%h", ex[i]);
                begin : chk
                    integer bad; bad = 0;
                    if (out_t !== ex[0]) begin bad++; $display("  t mismatch got %h exp %h", out_t, ex[0]); end
                    for (i = 0; i < 6; i++) if (out_x[32*i +: 32] !== ex[1+i]) begin bad++;
                        $display("  sc%0d ev%0d x%0d got %h exp %h", nsc, nev, i, out_x[32*i +: 32], ex[1+i]); end
                    for (i = 0; i < 6; i++) if (out_pd[32*i +: 32] !== ex[7+pdi[i]]) begin bad++;
                        $display("  sc%0d ev%0d Pd%0d got %h exp %h", nsc, nev, i, out_pd[32*i +: 32], ex[7+pdi[i]]); end
                    if (bad) begin nfail++; nbitfail += bad;
                        if (nfail == 1) begin
                            $display("  RAM ctx0 (0x240..) / ctx1 (0x260..) / expected:");
                            for (i = 0; i < 8; i++) $display("   +%0d ctx0=%h ctx1=%h exp=%h", i, dut.u_ram.u_macro.mem[10'h240+i], dut.u_ram.u_macro.mem[10'h260+i], ex[i]);
                            $display("   com_sel=%b com_base=%h", dut.u_tc.com_sel, dut.u_tc.com_base);
                        end
                    end
                end
            end
        end
        $display("RESULT scenarios=%0d events=%0d event_fails=%0d bad_words=%0d rf_conflict=%b ram_conflict=%b ram_oor=%b units=%0d",
                 nsc, nev, nfail, nbitfail, rf_conflict, ram_conflict, ram_oor, cnt_units);
        $finish;
    end
    initial begin #2000000000; $display("GLOBAL TIMEOUT"); $finish; end
endmodule
