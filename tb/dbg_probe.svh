    // debug probes (included by tb)
    logic _rfc_seen = 0, _ramc_seen = 0;
    always @(posedge clk) begin
        if (!_rfc_seen && !_ramc_seen && (((dut.p2_tc&dut.p2_seq)|(dut.p2_tc&dut.p2_lsu)|(dut.p2_seq&dut.p2_lsu))|(dut.seq_start&(dut.sw0|dut.sw1|dut.p2_tc|dut.p2_seq|dut.p2_lsu))))
            $display("[%0t] RFC-COND tc=%b(a%0d) seq2=%b(a%0d) lsu=%b(a%0d) start=%b sw0=%b sw1=%b tcstate=%0d running=%b", $time, dut.p2_tc, dut.tc_rf_waddr, dut.p2_seq, dut.sa2, dut.p2_lsu, dut.l_waddr, dut.seq_start, dut.sw0, dut.sw1, dut.u_tc.state, dut.seq_running);
        if (dut.rf_conflict_o && !_rfc_seen) begin _rfc_seen = 1;
            $display("[%0t] RF CONFLICT: tc=%b seq2=%b lsu=%b sw0=%b sw1=%b start=%b state=%0d tcaddr=%0d seqaddr=%0d lsuaddr=%0d",
               $time, dut.tc_rf_wen, dut.sw2, dut.l_wen, dut.sw0, dut.sw1, dut.seq_start, dut.u_tc.state, dut.tc_rf_waddr, dut.sa2, dut.l_waddr); end
        if (dut.ram_conflict_o && !_ramc_seen) begin _ramc_seen = 1;
            $display("[%0t] RAM CONFLICT: tc_ce=%b lsu_ce=%b cpu=%b tcstate=%0d tc_addr=%h lsu_addr=%h seq_running=%b",
               $time, dut.tc_ce, dut.l_ce, rw_ce, dut.u_tc.state, dut.tc_addr, dut.l_addr, dut.seq_running); end
    end
    integer _tr = 0; // trace disabled unless +trace
    always @(posedge clk) begin
        if (dut.seq_start && $test$plusargs("trace")) _tr = 14;
        if (_tr > 0) begin _tr--;
            $display("[%0t] start=%b run=%b uaddr=%0d lsu_v=%b we=%b reg=%0d addr=%0d l_ready=%b | l_wen=%b l_wa=%0d l_wd=%h | sw2=%b sa2=%0d sd2=%h | ram_rd=%h",
              $time, dut.seq_start, dut.seq_running, dut.ucode_addr, dut.s_lsu_v, dut.s_lsu_we, dut.s_lsu_reg, dut.s_lsu_addr, dut.l_cmd_ready,
              dut.l_wen, dut.l_waddr, dut.l_wdata, dut.sw2, dut.sa2, dut.sd2, dut.l_rd); end
    end
    always @(posedge clk) if ($time > 27370 && $time < 27420 && $test$plusargs("trace2"))
        $display("[%0t] tcstate=%0d tc_ce=%b tc_we=%b tc_a=%h | lsu_ce=%b l_we=%b l_a=%h busy=%b | s_lsu_v=%b s_we=%b s_addr=%h run=%b start=%b done=%b uaddr=%0d tc_bst=%b",
          $time, dut.u_tc.state, dut.tc_ce, dut.tc_we, dut.tc_addr, dut.l_ce, dut.l_we, dut.l_addr, dut.l_bbusy, dut.s_lsu_v, dut.s_lsu_we, dut.s_lsu_addr, dut.seq_running, dut.seq_start, dut.seq_done, dut.ucode_addr, dut.tc_bst);
    integer _t3 = 0;
    always @(posedge clk) begin
        if (dut.seq_start && $test$plusargs("trace3") && _t3 == 0) _t3 = 5000;
        if (_t3 > 0) begin _t3--;
            $display("t3 [%0t] uaddr=%0d idx0=%h lsu_v=%b we=%b reg=%0d addr=%h l_ce=%b l_we=%b l_a=%h l_wd=%h | l_wen=%b l_wa=%0d l_wd2=%h | sw2=%b sa2=%0d sd2=%h",
              $time, dut.ucode_addr, dut.u_seq.idx0_q, dut.s_lsu_v, dut.s_lsu_we, dut.s_lsu_reg, dut.s_lsu_addr, dut.l_ce, dut.l_we, dut.l_addr, dut.l_wd, dut.l_wen, dut.l_waddr, dut.l_wdata, dut.sw2, dut.sa2, dut.sd2); end
    end
    always @(posedge clk) if ($time > 21100 && $time < 21260 && $test$plusargs("trace4"))
        $display("t4 [%0t] uaddr=%0d fetchpc=%0d redirect=%b taken=%b warm=%0d stalled=%b loopcnt=%0d lsu_v=%b we=%b reg=%0d addr=%h idx0=%h idx1=%h fresh=%b ctrl_end=%b",
          $time, dut.ucode_addr, dut.u_seq.fetch_pc_q, dut.u_seq.redirect_now, dut.u_seq.branch_taken, dut.u_seq.warm_q, dut.u_seq.stalled, dut.u_seq.loop_cnt_q, dut.s_lsu_v, dut.s_lsu_we, dut.s_lsu_reg, dut.s_lsu_addr, dut.u_seq.idx0_q, dut.u_seq.idx1_q, dut.u_seq.fresh_q, dut.u_seq.ctrl_end);
    // differential-trace log: RF writes + RAM writes during first unit
    integer _lf = 0, _cyc = -1, _logging = 0, _units_logged = 0;
    initial if ($test$plusargs("rtllog")) _lf = $fopen("build/rtl_trace.txt", "w");
    always @(posedge clk) if (_lf != 0) begin
        if (dut.seq_start && _units_logged == 0) begin _logging = 1; _cyc = 0; end
        if (_logging) begin
            if (dut.w0_en) $fwrite(_lf, "W %0d 0 %0d %08x\n", _cyc, dut.w0_a, dut.w0_d);
            if (dut.w1_en) $fwrite(_lf, "W %0d 1 %0d %08x\n", _cyc, dut.w1_a, dut.w1_d);
            if (dut.w2_en) $fwrite(_lf, "W %0d 2 %0d %08x\n", _cyc, dut.w2_a, dut.w2_d);
            if (dut.l_ce && dut.l_we) $fwrite(_lf, "S %0d %0d %08x\n", _cyc, dut.l_addr, dut.l_wd);
            _cyc = _cyc + 1;
            if (dut.seq_done) begin _logging = 0; _units_logged = 1; $fclose(_lf); _lf = 0; end
        end
    end
    integer _t5 = 0;
    always @(posedge clk) begin
        if (dut.seq_start && $test$plusargs("trace5") && _t5 == 0) _t5 = 40;
        if (_t5 > 0) begin _t5--;
            $display("t5 [%0t] add_v_o=%b op=%0d a=%h b=%h | byp_v=%b byp_r=%h lt=%b | sw1=%b sa1=%0d sd1=%h | add_rd_q1=%0d byp_pend=%b byp_rd=%0d",
              $time, dut.add_v_o, dut.add_op, dut.add_a, dut.add_b, dut.add_byp_v, dut.add_byp_r, dut.add_lt, dut.sw1, dut.sa1, dut.sd1,
              dut.u_seq.add_byp_rd_q1, dut.u_seq.add_byp_pend_q1, dut.u_seq.add_byp_rd_q1); end
    end
    integer _c6 = 0, _ds0 = 0;
    always @(posedge clk) if ($test$plusargs("trace6")) begin
        _c6++;
        if (dut.ds_v) begin _ds0 = _c6; $display("DS start cyc=%0d op=%b busy=%b", _c6, dut.ds_op, dut.ds_busy); end
        if (dut.ds_done) $display("DS done cyc=%0d (delta %0d)", _c6, _c6-_ds0);
    end
