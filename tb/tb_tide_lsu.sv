// Self-checking LSU + RAM unit test (doc 09 section 7 test list).
`timescale 1ns/1ps
module tb_tide_lsu;
    logic clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    // LSU cmd / burst
    logic        cmd_valid = 0, cmd_we = 0;
    logic [6:0]  cmd_reg = 0;
    logic [11:0] cmd_addr = 0;
    wire         cmd_ready;
    logic        b_start = 0, b_we = 0;
    logic [6:0]  b_rfb = 0;
    logic [11:0] b_rmb = 0;
    logic [5:0]  b_len = 0;
    wire         b_busy, b_done;

    wire [6:0]  rf_raddr, rf_waddr;
    wire [31:0] rf_wdata;
    wire        rf_wen;
    wire        ram_ce, ram_we;
    wire [11:0] ram_addr;
    wire [31:0] ram_wdata, ram_rdata;

    // register file model (comb read, sync write)
    logic [31:0] rf [0:127];
    wire  [31:0] rf_rdata = rf[rf_raddr];
    always_ff @(posedge clk) if (rf_wen && rf_waddr != 7'd127) rf[rf_waddr] <= rf_wdata;

    tide_lsu dut (
        .clk_i(clk), .rst_ni(rst_n),
        .cmd_valid_i(cmd_valid), .cmd_we_i(cmd_we), .cmd_reg_i(cmd_reg), .cmd_addr_i(cmd_addr),
        .cmd_ready_o(cmd_ready),
        .burst_start_i(b_start), .burst_we_i(b_we), .burst_rf_base_i(b_rfb),
        .burst_ram_base_i(b_rmb), .burst_len_i(b_len),
        .burst_busy_o(b_busy), .burst_done_o(b_done),
        .rf_raddr_o(rf_raddr), .rf_rdata_i(rf_rdata),
        .rf_wen_o(rf_wen), .rf_waddr_o(rf_waddr), .rf_wdata_o(rf_wdata),
        .ram_ce_o(ram_ce), .ram_we_o(ram_we), .ram_addr_o(ram_addr),
        .ram_wdata_o(ram_wdata), .ram_rdata_i(ram_rdata));

    wire cf, oor;
    tide_ram u_ram (
        .clk_i(clk), .rst_ni(rst_n),
        .tc_ce_i(1'b0), .tc_we_i(1'b0), .tc_addr_i(12'd0), .tc_wdata_i(32'd0), .tc_rdata_o(),
        .lsu_ce_i(ram_ce), .lsu_we_i(ram_we), .lsu_addr_i(ram_addr), .lsu_wdata_i(ram_wdata),
        .lsu_rdata_o(ram_rdata),
        .cpu_wr_en_i(1'b0), .cpu_wr_ce_i(1'b0), .cpu_wr_addr_i(12'd0), .cpu_wr_data_i(32'd0),
        .dbg_ce_i(1'b0), .dbg_addr_i(12'd0), .dbg_rdata_o(), .dbg_valid_o(),
        .conflict_o(cf), .oor_o(oor));

    int errors = 0;
    int cyc = 0;
    always @(posedge clk) cyc <= cyc + 1;

    task automatic check(input bit ok, input string msg);
        if (!ok) begin errors++; $display("FAIL: %s (t=%0d)", msg, cyc); end
    endtask

    // Simpler explicit measurement: count cycles from start (inclusive) to done (inclusive).
    task automatic run_burst(input bit we, input [6:0] rfb, input [11:0] rmb, input [5:0] len,
                             output int cycles);
        int n;
        bit fin;
        @(negedge clk);
        b_start = 1; b_we = we; b_rfb = rfb; b_rmb = rmb; b_len = len;
        n = 1; fin = 0;
        #1; // let comb settle
        if (b_done) fin = 1;
        @(negedge clk);
        b_start = 0;
        while (!fin && n < 200) begin
            n++;            // now in cycle n-1 (0-based), i.e. n-th cycle
            #1;
            if (b_done) fin = 1;
            else @(negedge clk);
        end
        cycles = fin ? n : -1;
        @(negedge clk);
    endtask

    int c;
    initial begin
        for (int i = 0; i < 128; i++) rf[i] = 32'hFFFF_0000 + i;
        repeat (4) @(negedge clk);
        rst_n = 1;
        repeat (2) @(negedge clk);

        // ---- T1: burst STORE 27 words r0..r26 -> RAM 0x241.. ----
        for (int i = 0; i < 27; i++) rf[i] = 32'hA000_0000 + i * 7;
        run_burst(1, 7'd0, 12'h241, 6'd27, c);
        check(c == 27, $sformatf("burst store 27 cycles, got %0d", c));
        repeat (2) @(negedge clk);
        for (int i = 0; i < 27; i++)
            check(u_ram.u_macro.mem[12'h241 + i] == 32'hA000_0000 + i * 7, $sformatf("store word %0d", i));

        // ---- T2: burst LOAD 27 words back into r40.. ----
        run_burst(0, 7'd40, 12'h241, 6'd27, c);
        check(c == 28, $sformatf("burst load 27 -> 28 cycles, got %0d", c));
        repeat (2) @(negedge clk);
        for (int i = 0; i < 27; i++)
            check(rf[40 + i] == 32'hA000_0000 + i * 7, $sformatf("load word %0d = %h", i, rf[40 + i]));

        // ---- T3: burst len 1 ----
        rf[3] = 32'h1234_5678;
        run_burst(1, 7'd3, 12'h300, 6'd1, c);
        check(c == 1, $sformatf("burst store len1 = 1 cycle, got %0d", c));
        run_burst(0, 7'd90, 12'h300, 6'd1, c);
        check(c == 2, $sformatf("burst load len1 = 2 cycles, got %0d", c));
        repeat (2) @(negedge clk);
        check(rf[90] == 32'h1234_5678, "len1 roundtrip");

        // ---- T4: single load latency: RF write visible C+2 ----
        u_ram.u_macro.mem[12'h100] = 32'hCAFE_F00D;
        u_ram.u_macro.mem[12'h101] = 32'hBEEF_0001;
        @(negedge clk);
        cmd_valid = 1; cmd_we = 0; cmd_reg = 7'd50; cmd_addr = 12'h100;
        @(negedge clk);                       // C+1: write-back cycle
        cmd_reg = 7'd51; cmd_addr = 12'h101;  // back-to-back second load
        #1; check(rf_wen && rf_waddr == 7'd50 && rf_wdata == 32'hCAFE_F00D, "load#1 write-back at C+1");
        @(negedge clk);
        cmd_valid = 0;
        #1; check(rf_wen && rf_waddr == 7'd51 && rf_wdata == 32'hBEEF_0001, "load#2 write-back pipelined");
        @(negedge clk);
        check(rf[50] == 32'hCAFE_F00D && rf[51] == 32'hBEEF_0001, "loads landed in RF");

        // ---- T5: single store ----
        rf[60] = 32'h5555_AAAA;
        cmd_valid = 1; cmd_we = 1; cmd_reg = 7'd60; cmd_addr = 12'h102;
        @(negedge clk);
        cmd_valid = 0;
        @(negedge clk);
        check(u_ram.u_macro.mem[12'h102] == 32'h5555_AAAA, "single store lands");

        // ---- T6: cmd_ready low during burst ----
        @(negedge clk);
        b_start = 1; b_we = 1; b_rfb = 0; b_rmb = 12'h280; b_len = 6'd10;
        @(negedge clk); b_start = 0;
        #1; check(!cmd_ready && b_busy, "busy during burst");
        repeat (12) @(negedge clk);
        check(cmd_ready && !b_busy, "ready after burst");

        // ---- T7: out-of-range write ignored ----
        cmd_valid = 1; cmd_we = 1; cmd_reg = 7'd60; cmd_addr = 12'h400;
        @(negedge clk); cmd_valid = 0; @(negedge clk); @(negedge clk);
        check(oor, "out-of-range flagged");

        check(!cf, "no arbitration conflict");
        if (errors == 0) $display("TB_TIDE_LSU: PASS");
        else             $display("TB_TIDE_LSU: FAIL (%0d errors)", errors);
        $finish;
    end
endmodule
