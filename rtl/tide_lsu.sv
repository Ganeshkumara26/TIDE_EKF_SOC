// -----------------------------------------------------------------------------
// tide_lsu.sv - Load/Store Unit between the register file and TIDE-RAM
// Owner: Agent 4.  Spec: doc 09, timing.yaml, STEP_0 (word addresses, ISA-10)
//
// Single-word LOAD  : cmd at cycle C -> RAM read issued in C, RF write in C+1
//                     (value readable by consumers from C+2).  Fully pipelined.
// Single-word STORE : cmd at cycle C -> RF read + RAM write both in C.
// Burst STORE       : burst_start in cycle 0 (params are consumed combinationally
//                     in cycle 0).  One word per cycle, cycles 0..len-1.
//                     burst_done_o is high in cycle len-1.   => len cycles total.
// Burst LOAD        : cycle 0 issues RAM read of word 0; cycle k (1..len-1) issues
//                     read k and writes RF word k-1; cycle len writes RF word len-1.
//                     burst_done_o is high in cycle len.     => len+1 cycles total.
//
// Rules for the requester (asserted by sva/tide_lsu_sva.sv):
//   * do not issue cmd_valid_i together with burst_start_i
//   * do not issue burst_start_i / cmd while burst_busy_o
// burst_busy_o is registered: it is high from cycle 1 of a burst until the cycle
// AFTER done (cycle 0 of a burst is known to the requester, it issued the start).
//
// Addresses are 12-bit WORD addresses; the RAM ignores anything >= 1024.
// -----------------------------------------------------------------------------
module tide_lsu (
    input  logic        clk_i,
    input  logic        rst_ni,

    // single-word command (from sequencer LSU_ALU slot)
    input  logic        cmd_valid_i,
    input  logic        cmd_we_i,          // 1 = STORE (RF -> RAM), 0 = LOAD (RAM -> RF)
    input  logic [6:0]  cmd_reg_i,         // RF register index
    input  logic [11:0] cmd_addr_i,        // RAM word address
    output logic        cmd_ready_o,

    // burst command (sequencer CTX ops or temporal controller)
    input  logic        burst_start_i,
    input  logic        burst_we_i,        // 1 = RF -> RAM (save), 0 = RAM -> RF (restore)
    input  logic [6:0]  burst_rf_base_i,
    input  logic [11:0] burst_ram_base_i,
    input  logic [5:0]  burst_len_i,       // 1..63 (0 = no-op, done in cycle 0)
    output logic        burst_busy_o,
    output logic        burst_done_o,

    // register file
    output logic [6:0]  rf_raddr_o,        // combinational RF read address (store data)
    input  logic [31:0] rf_rdata_i,
    output logic        rf_wen_o,
    output logic [6:0]  rf_waddr_o,
    output logic [31:0] rf_wdata_o,

    // RAM
    output logic        ram_ce_o,
    output logic        ram_we_o,
    output logic [11:0] ram_addr_o,
    output logic [31:0] ram_wdata_o,
    input  logic [31:0] ram_rdata_i
);
    // ---------------- burst state ----------------
    logic        busy_q;
    logic [5:0]  k_q;            // cycle index within burst (valid while busy_q)
    logic        we_q;
    logic [6:0]  rfb_q;
    logic [11:0] rmb_q;
    logic [5:0]  len_q;

    // effective parameters: cycle 0 uses the inputs, later cycles use the registers
    wire         start_e = burst_start_i & ~busy_q;
    wire         act     = start_e | busy_q;
    wire         we_e    = busy_q ? we_q  : burst_we_i;
    wire [6:0]   rfb_e   = busy_q ? rfb_q : burst_rf_base_i;
    wire [11:0]  rmb_e   = busy_q ? rmb_q : burst_ram_base_i;
    wire [5:0]   len_e   = busy_q ? len_q : burst_len_i;
    wire [5:0]   k_e     = busy_q ? k_q   : 6'd0;

    wire         zero_len = (len_e == 6'd0);
    // last cycle of the burst
    wire         last = zero_len ? 1'b1 :
                        (we_e ? (k_e == len_e - 6'd1) : (k_e == len_e));

    // ---------------- single-word load write-back ----------------
    logic       pend_q;
    logic [6:0] pend_reg_q;

    wire cmd_fire = cmd_valid_i & cmd_ready_o;
    assign cmd_ready_o  = ~busy_q;
    assign burst_busy_o = busy_q;
    assign burst_done_o = act & last;

    // ---------------- datapath ----------------
    wire [11:0] burst_ram_addr = rmb_e + {6'd0, k_e};
    wire [6:0]  burst_rf_st    = rfb_e + {1'b0, k_e};            // store: RF word read
    wire [6:0]  burst_rf_ld    = rfb_e + {1'b0, k_e} - 7'd1;     // load : RF word written (k>=1)

    always_comb begin
        ram_ce_o    = 1'b0;
        ram_we_o    = 1'b0;
        ram_addr_o  = 12'd0;
        ram_wdata_o = rf_rdata_i;
        rf_raddr_o  = cmd_reg_i;
        rf_wen_o    = 1'b0;
        rf_waddr_o  = 7'd0;
        rf_wdata_o  = ram_rdata_i;

        // single-word load write-back (cycle C+1)
        if (pend_q) begin
            rf_wen_o   = 1'b1;
            rf_waddr_o = pend_reg_q;
        end

        if (act && !zero_len) begin
            if (we_e) begin
                // burst store: RF -> RAM, one word per cycle
                ram_ce_o   = 1'b1;
                ram_we_o   = 1'b1;
                ram_addr_o = burst_ram_addr;
                rf_raddr_o = burst_rf_st;
            end else begin
                // burst load: read issue for k < len, RF write-back for k >= 1
                if (k_e != len_e) begin
                    ram_ce_o   = 1'b1;
                    ram_we_o   = 1'b0;
                    ram_addr_o = burst_ram_addr;
                end
                if (k_e != 6'd0) begin
                    rf_wen_o   = 1'b1;
                    rf_waddr_o = burst_rf_ld;
                end
            end
        end else if (cmd_fire) begin
            ram_ce_o   = 1'b1;
            ram_we_o   = cmd_we_i;
            ram_addr_o = cmd_addr_i;
        end
    end

    // ---------------- sequential ----------------
    always_ff @(posedge clk_i) begin
        if (!rst_ni) begin
            busy_q     <= 1'b0;
            k_q        <= 6'd0;
            we_q       <= 1'b0;
            rfb_q      <= 7'd0;
            rmb_q      <= 12'd0;
            len_q      <= 6'd0;
            pend_q     <= 1'b0;
            pend_reg_q <= 7'd0;
        end else begin
            // single load bookkeeping
            pend_q <= 1'b0;
            if (cmd_fire && !cmd_we_i && !(act && !zero_len)) begin
                pend_q     <= 1'b1;
                pend_reg_q <= cmd_reg_i;
            end

            // burst bookkeeping
            if (act) begin
                if (last) begin
                    busy_q <= 1'b0;
                end else begin
                    busy_q <= 1'b1;
                    k_q    <= k_e + 6'd1;
                    if (start_e) begin
                        we_q  <= burst_we_i;
                        rfb_q <= burst_rf_base_i;
                        rmb_q <= burst_ram_base_i;
                        len_q <= burst_len_i;
                    end
                end
            end
        end
    end
endmodule
