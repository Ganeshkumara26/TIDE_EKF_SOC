// -----------------------------------------------------------------------------
// tide_ram.sv - TIDE-RAM (4 KB = 1024 x 32) with 4-requester arbiter
// Owner: Agent 4.  Spec: doc 32 (macro wrapper), tide_ram.md, memmap.yaml
//
// Requesters (strict priority, highest first):
//   1. TC   (temporal controller direct port)
//   2. LSU  (burst / single access)
//   3. CPU  (optional write port, see ASSUMPTIONS A9: INIT needs a CPU write path)
//   4. DBG  (CPU debug read window, read-only)
//
// Timing (doc 32 2.2, frozen):
//   * synchronous read: data valid the cycle AFTER ce & ~we
//   * synchronous write: written at the posedge where ce & we
//   * after a write cycle the read-data register is UNDEFINED (poisoned in sim);
//     no requester may rely on it.
//
// Addressing: 12-bit WORD address (STEP_0 decision ISA-10); only 0..1023 exist.
// Out-of-range accesses (addr[11:10] != 0): writes ignored, reads return 0,
// sticky oor_o set.
// -----------------------------------------------------------------------------
module tide_sram_sp_32x1024 (
    input  logic        clk_i,
    input  logic        rst_ni,   // unused by real macros (no reset pin)
    input  logic        ce_i,
    input  logic        we_i,
    input  logic [9:0]  addr_i,
    input  logic [31:0] wdata_i,
    output logic [31:0] rdata_o
);
`ifdef TIDE_MACROS
    // ASAP7 fakeram7_256x32 x4, bank = addr[9:8]. Synchronous read (data valid the cycle after ce&~we).
    wire _unused_rst = rst_ni;
    logic [1:0]  bank_q;
    logic [31:0] rd [0:3];
    always_ff @(posedge clk_i) if (ce_i) bank_q <= addr_i[9:8];
    for (genvar b = 0; b < 4; b++) begin : g_bank
        fakeram7_256x32 u_m (.clk(clk_i), .ce_in(ce_i & (addr_i[9:8] == b)), .we_in(we_i), .addr_in(addr_i[7:0]),
                             .wd_in(wdata_i), .rd_out(rd[b]));
    end
    assign rdata_o = rd[bank_q];
`else
    // Behavioural model.  Replace by the foundry macro wrapper for ASIC.
    logic [31:0] mem [0:1023];
    wire _unused_rst = rst_ni;
    always_ff @(posedge clk_i) begin
        if (ce_i) begin
            if (we_i) begin
                mem[addr_i] <= wdata_i;
`ifndef SYNTHESIS
                rdata_o     <= 32'hDEAD_BEEF;   // doc 32 2.2.3: undefined after write -> poison
`endif
            end else begin
                rdata_o <= mem[addr_i];
            end
        end
    end
`endif
endmodule

module tide_ram (
    input  logic        clk_i,
    input  logic        rst_ni,

    // Port 1: temporal controller (highest priority)
    input  logic        tc_ce_i,
    input  logic        tc_we_i,
    input  logic [11:0] tc_addr_i,
    input  logic [31:0] tc_wdata_i,
    output logic [31:0] tc_rdata_o,

    // Port 2: LSU
    input  logic        lsu_ce_i,
    input  logic        lsu_we_i,
    input  logic [11:0] lsu_addr_i,
    input  logic [31:0] lsu_wdata_i,
    output logic [31:0] lsu_rdata_o,

    // Port 3: CPU write (INIT seeding); enabled only when cpu_wr_en_i (top gates on !busy)
    input  logic        cpu_wr_en_i,
    input  logic        cpu_wr_ce_i,
    input  logic [11:0] cpu_wr_addr_i,
    input  logic [31:0] cpu_wr_data_i,

    // Port 4: CPU debug read (read-only)
    input  logic        dbg_ce_i,
    input  logic [11:0] dbg_addr_i,
    output logic [31:0] dbg_rdata_o,
    output logic        dbg_valid_o,     // data valid this cycle (read was granted last cycle)

    // Error flags (sticky, cleared only by reset)
    output logic        conflict_o,      // a TC/LSU/CPU-write request lost arbitration
    output logic        oor_o            // out-of-range access attempted
);
    // ---------------- arbitration (combinational) ----------------
    logic        g_tc, g_lsu, g_cpu, g_dbg;
    logic        m_ce, m_we;
    logic [11:0] m_addr;
    logic [31:0] m_wdata;
    logic        cpu_req;

    assign cpu_req = cpu_wr_en_i & cpu_wr_ce_i;

    always_comb begin
        g_tc = 1'b0; g_lsu = 1'b0; g_cpu = 1'b0; g_dbg = 1'b0;
        m_ce = 1'b0; m_we = 1'b0; m_addr = 12'd0; m_wdata = 32'd0;
        if (tc_ce_i) begin
            g_tc = 1'b1; m_ce = 1'b1; m_we = tc_we_i; m_addr = tc_addr_i; m_wdata = tc_wdata_i;
        end else if (lsu_ce_i) begin
            g_lsu = 1'b1; m_ce = 1'b1; m_we = lsu_we_i; m_addr = lsu_addr_i; m_wdata = lsu_wdata_i;
        end else if (cpu_req) begin
            g_cpu = 1'b1; m_ce = 1'b1; m_we = 1'b1; m_addr = cpu_wr_addr_i; m_wdata = cpu_wr_data_i;
        end else if (dbg_ce_i) begin
            g_dbg = 1'b1; m_ce = 1'b1; m_we = 1'b0; m_addr = dbg_addr_i;
        end
    end

    wire in_range = (m_addr[11:10] == 2'b00);
    wire mac_ce   = m_ce & in_range;
    logic [31:0] mac_rdata;

    tide_sram_sp_32x1024 u_macro (
        .clk_i  (clk_i),
        .rst_ni (rst_ni),
        .ce_i   (mac_ce),
        .we_i   (m_we),
        .addr_i (m_addr[9:0]),
        .wdata_i(m_wdata),
        .rdata_o(mac_rdata)
    );

    // ---------------- read-data steering (registered grant info) ----------------
    logic rd_dbg_q, rd_oor_q;
    always_ff @(posedge clk_i) begin
        if (!rst_ni) begin
            rd_dbg_q <= 1'b0; rd_oor_q <= 1'b0;
            conflict_o <= 1'b0; oor_o <= 1'b0;
        end else begin
            rd_dbg_q <= g_dbg;
            rd_oor_q <= m_ce & ~in_range;
            if ((tc_ce_i & lsu_ce_i) | (tc_ce_i & cpu_req) | (lsu_ce_i & cpu_req))
                conflict_o <= 1'b1;
            if (m_ce & ~in_range) oor_o <= 1'b1;
        end
    end

    // All ports see the macro output; out-of-range reads return zero.
    assign tc_rdata_o  = rd_oor_q ? 32'd0 : mac_rdata;
    assign lsu_rdata_o = rd_oor_q ? 32'd0 : mac_rdata;
    // Debug: returns data only if its read was granted; otherwise 0 (spec: "old data or zero")
    assign dbg_rdata_o = (rd_dbg_q & ~rd_oor_q) ? mac_rdata : 32'd0;
    assign dbg_valid_o = rd_dbg_q;
endmodule
