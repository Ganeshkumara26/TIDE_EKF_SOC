// -----------------------------------------------------------------------------
// tide_engine.sv -- the real "tide_top": temporal controller + sequencer +
// register file + LSU + TIDE-RAM + microcode RAM + const ROM + real FP32 lanes.
//
// This is the first time all of these have been wired together; every block
// previously ran only against stand-ins. Integration decisions made here
// (each one is a thing the blocks' own ASSUMPTIONS files left to "tide_top"):
//
//  I1. RF write port 2 is shared by three producers that are mutually
//      exclusive by construction: temporal event/ABI writes (sequencer idle),
//      sequencer IntALU/ROM writes, LSU LOAD writeback. Priority tc > seq > lsu;
//      any same-cycle overlap latches sticky `rf_conflict_o`.
//  I2. The LSU has no RF read port of its own (regfile has exactly 4). While the
//      sequencer runs, its raddr2 already carries the STORE source register
//      (tide_sequencer.sv:294), so LSU store data is taken from rdata2; while the
//      sequencer is idle (temporal burst-store) the LSU drives raddr2 itself.
//  I3. Microcode ABI regs r120/r121/r122 (QP, QV, DT0INV; tide_abi.py R_QP..)
//      are not written by tide_temporal. On the seq_start_o cycle the three RF
//      write ports (idle then) each load one of them from the CSR inputs.
//  I4. Event FIFO omitted: the AXI register block pushes straight into
//      tide_temporal, gated by event_ready_o (see register map EVENT_READY).
//  I5. LSU burst command: temporal OR sequencer (sequencer's is tied off per
//      its ASSUMPTIONS A6; OR keeps the path open).
//  I6. CPU RAM-write is only granted while !busy; microcode RAM write is
//      ignored while enable_i is set (memmap.yaml lock_condition).
// -----------------------------------------------------------------------------
module tide_engine #(
    parameter [8:0] UNIT_ENTRY  = 9'd0,
    parameter [8:0] QUERY_ENTRY = 9'd471,
    parameter integer CKPT_DEPTH = 8,     // power of two (ring in TIDE-RAM 0x000..0x0FF: max 8)
    parameter integer JRNL_DEPTH = 64      // power of two <= 64 (journal in TIDE-RAM 0x100..0x23F)
) (
    input  logic         clk_i,
    input  logic         rst_ni,

    // ---- control / event (same meaning as tide_temporal) ----
    input  logic [4:0]   ckpt_interval_i,
    input  logic         enable_i,
    input  logic         irq_en_i,
    input  logic [8:0]   status_clr_i,
    input  logic         clr_lat_i,
    input  logic         event_valid_i,
    input  logic [31:0]  event_t_i,
    input  logic [31:0]  event_z0_i, event_z1_i, event_z2_i, event_z3_i,
    input  logic [4:0]   event_flags_i,
    output logic         event_ready_o,
    output logic [4:0]   cur_flags_o,
    output logic [1:0]   rset_sel_o,
    input  logic [31:0]  rset_r0_i, rset_r1_i, rset_r2_i, rset_r3_i,

    // ---- microcode ABI parameters (float32) ----
    input  logic [31:0]  qp_i, qv_i, dt0inv_i,

    // ---- microcode RAM load port (3 writes of 32b per 96b word) ----
    input  logic         uc_we_i,
    input  logic [8:0]   uc_addr_i,
    input  logic [1:0]   uc_word_i,
    input  logic [31:0]  uc_data_i,
    input  logic [8:0]   uc_rd_addr_i,     // microcode read mirror (valid only while enable_i=0), data 1 cycle later
    output logic [95:0]  uc_rd_data_o,

    // ---- TIDE-RAM CPU ports (seed / debug) ----
    input  logic         ram_wr_ce_i,
    input  logic [11:0]  ram_wr_addr_i,
    input  logic [31:0]  ram_wr_data_i,
    input  logic         ram_dbg_ce_i,
    input  logic [11:0]  ram_dbg_addr_i,
    output logic [31:0]  ram_dbg_rdata_o,
    output logic         ram_dbg_valid_o,

    // ---- status / irq ----
    output logic         busy_o, replay_pending_o, provisional_o,
    output logic         irq_o,
    output logic [4:0]   irq_ev_o,
    output logic [8:0]   status_o,
    output logic         gpio_engine_busy_o, gpio_late_drop_o, gpio_provisional_o,
    output logic [31:0]  cnt_units_o, cnt_late_o, cnt_drop_o, cnt_replay_o,
    output logic [31:0]  cnt_abort_o, cnt_query_o,
    output logic [31:0]  cyc_last_unit_o, cyc_last_replay_o,
    output logic [31:0]  lat_last_wait_o, lat_max_wait_o,
    output logic [31:0]  out_seq_o, out_t_o,
    output logic [191:0] out_x_o, out_pdiag_o,
    output logic         out_committed_o,
    output logic [191:0] qout_x_o,
    output logic [31:0]  qout_t_o,

    // ---- sticky integration error flags ----
    output logic         ram_conflict_o, ram_oor_o, rf_conflict_o
);
    // ------------------------------------------------------------------ nets
    // temporal <-> sequencer/LSU/RAM
    logic        seq_start; logic [8:0] seq_start_addr;
    logic        seq_running, seq_done, seq_boundary;
    logic        tc_bst, tc_bwe; logic [6:0] tc_brf; logic [11:0] tc_bram; logic [5:0] tc_blen;
    logic        tc_rf_wen; logic [6:0] tc_rf_waddr; logic [31:0] tc_rf_wdata;
    logic        tc_ce, tc_we; logic [11:0] tc_addr; logic [31:0] tc_wdata, tc_rdata;

    // sequencer
    logic [8:0]  ucode_addr; logic [95:0] ucode_data;
    logic [6:0]  ra0, ra1, ra2_seq, ra3; logic [31:0] rd0, rd1, rd2, rd3;
    logic        sw0, sw1, sw2; logic [6:0] sa0, sa1, sa2; logic [31:0] sd0, sd1, sd2;
    logic        s_lsu_v, s_lsu_we; logic [6:0] s_lsu_reg; logic [11:0] s_lsu_addr;
    logic        s_bst, s_bwe; logic [6:0] s_brf; logic [11:0] s_bram; logic [5:0] s_blen;
    logic        mul_v_o, mul_v_i; logic [31:0] mul_a, mul_b, mul_r;
    logic        add_v_o, add_v_i, add_byp_v, add_lt; logic [2:0] add_op;
    logic [31:0] add_a, add_b, add_r, add_byp_r;
    logic        ds_v, ds_op, ds_busy, ds_done; logic [31:0] ds_a, ds_b, ds_r;
    logic [5:0]  rom_addr; logic [31:0] rom_data;
    logic        engine_owns; logic [31:0] seq_lsu_rdata;

    // LSU
    logic        l_cmd_ready, l_bdone, l_bbusy;
    logic [6:0]  l_rf_raddr; logic        l_wen; logic [6:0] l_waddr; logic [31:0] l_wdata;
    logic        l_ce, l_we; logic [11:0] l_addr; logic [31:0] l_wd, l_rd;

    // ------------------------------------------------------------- temporal
    tide_temporal #(.UNIT_ENTRY(UNIT_ENTRY), .QUERY_ENTRY(QUERY_ENTRY), .CKPT_DEPTH(CKPT_DEPTH), .JRNL_DEPTH(JRNL_DEPTH)) u_tc (
        .clk_i, .rst_ni,
        .ckpt_interval_i, .enable_i, .irq_en_i, .status_clr_i, .clr_lat_i,
        .event_valid_i, .event_t_i, .event_tarr_i(32'd0),
        .event_z0_i, .event_z1_i, .event_z2_i, .event_z3_i, .event_flags_i,
        .event_ready_o, .cur_flags_o, .rset_sel_o,
        .rset_r0_i, .rset_r1_i, .rset_r2_i, .rset_r3_i,
        .seq_start_o(seq_start), .seq_start_addr_o(seq_start_addr),
        .seq_running_i(seq_running), .seq_done_i(seq_done), .seq_boundary_i(seq_boundary),
        .lsu_burst_start_o(tc_bst), .lsu_burst_we_o(tc_bwe), .lsu_burst_rf_base_o(tc_brf),
        .lsu_burst_ram_base_o(tc_bram), .lsu_burst_len_o(tc_blen), .lsu_burst_done_i(l_bdone),
        .rf_wen_o(tc_rf_wen), .rf_waddr_o(tc_rf_waddr), .rf_wdata_o(tc_rf_wdata),
        .ram_ce_o(tc_ce), .ram_we_o(tc_we), .ram_addr_o(tc_addr), .ram_wdata_o(tc_wdata),
        .ram_rdata_i(tc_rdata),
        .busy_o, .replay_pending_o, .provisional_o, .irq_o, .irq_ev_o, .status_o,
        .gpio_engine_busy_o, .gpio_late_drop_o, .gpio_provisional_o,
        .cnt_units_o, .cnt_late_o, .cnt_drop_o, .cnt_replay_o, .cnt_abort_o, .cnt_query_o,
        .cyc_last_unit_o, .cyc_last_replay_o, .lat_last_wait_o, .lat_max_wait_o,
        .out_seq_o, .out_t_o, .out_x_o, .out_pdiag_o, .out_committed_o,
        .qout_x_o, .qout_t_o
    );

    // ------------------------------------------------------------ sequencer
    tide_sequencer u_seq (
        .clk_i, .rst_ni,
        .start_i(seq_start), .start_addr_i(seq_start_addr),
        .running_o(seq_running), .done_o(seq_done), .boundary_o(seq_boundary),
        .ucode_addr_o(ucode_addr), .ucode_data_i(ucode_data),
        .rf_raddr0_o(ra0), .rf_raddr1_o(ra1), .rf_raddr2_o(ra2_seq), .rf_raddr3_o(ra3),
        .rf_rdata0_i(rd0), .rf_rdata1_i(rd1), .rf_rdata2_i(rd2), .rf_rdata3_i(rd3),
        .rf_wen0_o(sw0), .rf_waddr0_o(sa0), .rf_wdata0_o(sd0),
        .rf_wen1_o(sw1), .rf_waddr1_o(sa1), .rf_wdata1_o(sd1),
        .rf_wen2_o(sw2), .rf_waddr2_o(sa2), .rf_wdata2_o(sd2),
        .lsu_cmd_valid_o(s_lsu_v), .lsu_cmd_we_o(s_lsu_we),
        .lsu_cmd_reg_o(s_lsu_reg), .lsu_cmd_addr_o(s_lsu_addr),
        .lsu_cmd_ready_i(l_cmd_ready),
        .lsu_burst_start_o(s_bst), .lsu_burst_we_o(s_bwe),
        .lsu_burst_rf_base_o(s_brf), .lsu_burst_ram_base_o(s_bram), .lsu_burst_len_o(s_blen),
        .lsu_burst_done_i(l_bdone), .lsu_burst_busy_i(l_bbusy),
        .mul_valid_o(mul_v_o), .mul_a_o(mul_a), .mul_b_o(mul_b), .mul_ready_i(1'b1),
        .mul_valid_i(mul_v_i), .mul_result_i(mul_r),
        .add_valid_o(add_v_o), .add_op_o(add_op), .add_a_o(add_a), .add_b_o(add_b), .add_ready_i(1'b1),
        .add_valid_i(add_v_i), .add_result_i(add_r),
        .add_bypass_valid_i(add_byp_v), .add_bypass_result_i(add_byp_r), .add_cmp_lt_i(add_lt),
        .ds_valid_o(ds_v), .ds_op_o(ds_op), .ds_a_o(ds_a), .ds_b_o(ds_b),
        .ds_busy_i(ds_busy), .ds_done_i(ds_done), .ds_result_i(ds_r),
        .rom_addr_o(rom_addr), .rom_data_i(rom_data),
        .lsu_rdata_o(seq_lsu_rdata), .engine_owns_o(engine_owns)
    );

    tide_ucode_ram u_ucode (
        .clk_i, .rst_ni,
        .seq_addr_i(enable_i ? ucode_addr : uc_rd_addr_i), .seq_data_o(ucode_data),
        .cpu_we_i(uc_we_i & ~enable_i), .cpu_addr_i(uc_addr_i),
        .cpu_word_i(uc_word_i), .cpu_data_i(uc_data_i)
    );

    assign uc_rd_data_o = ucode_data;

    // ----------------------------------------------------------- FP32 lanes
    tide_fp32_mul u_mul (
        .clk_i, .rst_ni, .valid_i(mul_v_o), .a_i(mul_a), .b_i(mul_b),
        .valid_o(mul_v_i), .result_o(mul_r), .flags_o()
    );
    logic add_cmp_eq_unused;
    tide_fp32_add u_add (
        .clk_i, .rst_ni, .valid_i(add_v_o), .op_i(add_op), .a_i(add_a), .b_i(add_b),
        .valid_o(add_v_i), .result_o(add_r), .flags_o(),
        .bypass_valid_o(add_byp_v), .bypass_result_o(add_byp_r),
        .cmp_lt_o(add_lt), .cmp_eq_o(add_cmp_eq_unused)
    );
    tide_fp32_divsqrt u_ds (
        .clk_i, .rst_ni, .start_i(ds_v), .a_i(ds_a), .b_i(ds_b), .op_i(ds_op),
        .busy_o(ds_busy), .done_o(ds_done), .result_o(ds_r), .flags_o()
    );
    tide_const_rom u_rom (.addr_i(rom_addr), .data_o(rom_data));

    // ------------------------------------------------------------------ LSU
    tide_lsu u_lsu (
        .clk_i, .rst_ni,
        .cmd_valid_i(s_lsu_v), .cmd_we_i(s_lsu_we), .cmd_reg_i(s_lsu_reg), .cmd_addr_i(s_lsu_addr),
        .cmd_ready_o(l_cmd_ready),
        .burst_start_i(tc_bst | s_bst), .burst_we_i(tc_bst ? tc_bwe : s_bwe),
        .burst_rf_base_i(tc_bst ? tc_brf : s_brf),
        .burst_ram_base_i(tc_bst ? tc_bram : s_bram),
        .burst_len_i(tc_bst ? tc_blen : s_blen),
        .burst_busy_o(l_bbusy), .burst_done_o(l_bdone),
        .rf_raddr_o(l_rf_raddr), .rf_rdata_i(engine_owns ? seq_lsu_rdata : rd2),     // I2 (revised: sequencer resolves its own LSU source port)
        .rf_wen_o(l_wen), .rf_waddr_o(l_waddr), .rf_wdata_o(l_wdata),
        .ram_ce_o(l_ce), .ram_we_o(l_we), .ram_addr_o(l_addr), .ram_wdata_o(l_wd),
        .ram_rdata_i(l_rd)
    );

    tide_ram u_ram (
        .clk_i, .rst_ni,
        .tc_ce_i(tc_ce), .tc_we_i(tc_we), .tc_addr_i(tc_addr), .tc_wdata_i(tc_wdata), .tc_rdata_o(tc_rdata),
        .lsu_ce_i(l_ce), .lsu_we_i(l_we), .lsu_addr_i(l_addr), .lsu_wdata_i(l_wd), .lsu_rdata_o(l_rd),
        .cpu_wr_en_i(~busy_o), .cpu_wr_ce_i(ram_wr_ce_i), .cpu_wr_addr_i(ram_wr_addr_i), .cpu_wr_data_i(ram_wr_data_i),
        .dbg_ce_i(ram_dbg_ce_i), .dbg_addr_i(ram_dbg_addr_i),
        .dbg_rdata_o(ram_dbg_rdata_o), .dbg_valid_o(ram_dbg_valid_o),
        .conflict_o(ram_conflict_o), .oor_o(ram_oor_o)
    );

    // ----------------------------------------------------------- regfile
    // I2: LSU owns raddr2 while sequencer idle
    wire [6:0] ra2 = engine_owns ? ra2_seq : l_rf_raddr;

    // I1 + I3: write-port muxing
    wire        p2_tc  = tc_rf_wen;
    wire        p2_seq = sw2;
    wire        p2_lsu = l_wen;
    wire        w0_en  = sw0 | seq_start;
    wire        w1_en  = sw1 | seq_start;
    wire        w2_en  = p2_tc | p2_seq | p2_lsu | seq_start;
    wire [6:0]  w0_a   = seq_start ? 7'd120 : sa0;
    wire [31:0] w0_d   = seq_start ? qp_i    : sd0;
    wire [6:0]  w1_a   = seq_start ? 7'd121 : sa1;
    wire [31:0] w1_d   = seq_start ? qv_i    : sd1;
    wire [6:0]  w2_a   = seq_start ? 7'd122 : p2_tc ? tc_rf_waddr : p2_seq ? sa2 : l_waddr;
    wire [31:0] w2_d   = seq_start ? dt0inv_i : p2_tc ? tc_rf_wdata : p2_seq ? sd2 : l_wdata;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) rf_conflict_o <= 1'b0;
        else if (((p2_tc & p2_seq) | (p2_tc & p2_lsu) | (p2_seq & p2_lsu)) |
                 (seq_start & (sw0 | sw1 | p2_tc | p2_seq | p2_lsu)))
            rf_conflict_o <= 1'b1;
    end

    tide_regfile u_rf (
        .clk_i, .rst_ni,
        .raddr0_i(ra0), .rdata0_o(rd0), .raddr1_i(ra1), .rdata1_o(rd1),
        .raddr2_i(ra2), .rdata2_o(rd2), .raddr3_i(ra3), .rdata3_o(rd3),
        .wen0_i(w0_en), .waddr0_i(w0_a), .wdata0_i(w0_d),
        .wen1_i(w1_en), .waddr1_i(w1_a), .wdata1_i(w1_d),
        .wen2_i(w2_en), .waddr2_i(w2_a), .wdata2_i(w2_d)
    );
endmodule
