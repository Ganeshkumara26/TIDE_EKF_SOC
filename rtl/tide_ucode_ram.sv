// -------------------------------------------------------------------
// tide_ucode_ram.sv — 512 x 96-bit Microcode RAM
//
// Contract: shared/contract/tide_ucode_ram.sv (tide_ucode_ram_if) — port
// list is frozen and matched exactly below.
// Design ref: read-only/design/07_tide_ucode_ram.md
//
// Behavior:
//   - Sequencer read port is synchronous (registered output): the address
//     presented on seq_addr_i on cycle N returns data on seq_data_o on
//     cycle N+1 (1-cycle read latency). There is no read-enable; the read
//     port is always active.
//   - CPU write port uses the frozen 3-phase protocol: word[31:0] and
//     word[63:32] are captured into a 64-bit staging register on
//     cpu_word_i == 0/1 respectively; the 96-bit word is committed to
//     mem[cpu_addr_i] only on cpu_word_i == 2 (using the just-arrived
//     32-bit chunk as bits [95:64] together with the staged [63:0]).
//   - Per ASSUMPTIONS.md [A3]: the design doc's §6 "write lock while
//     ENABLE=1" requirement (REQ-I-03 in 51_verification_matrix.md) has NO
//     corresponding port in the frozen tide_ucode_ram_if (no enable/engine
//     status input exists). Per the OVERVIEW.md precedence rule
//     (contract file > design doc), this module does NOT implement the
//     write lock itself; that gating must be done upstream (at the bus
//     decoder / tide_top level, which can independently know engine
//     status) before cpu_we_i reaches this module. This module always
//     honors cpu_we_i as presented.
// -------------------------------------------------------------------

module tide_ucode_ram (
    input  logic        clk_i,
    input  logic        rst_ni,

    // Sequencer read port (one instruction per cycle)
    input  logic [8:0]  seq_addr_i,   // 9-bit address (0-511)
    output logic [95:0] seq_data_o,   // 96-bit microinstruction

    // CPU write port (MMIO: three 32-bit writes fill one 96-bit word)
    input  logic        cpu_we_i,     // Write enable
    input  logic [8:0]  cpu_addr_i,   // Word address (0-511)
    input  logic [1:0]  cpu_word_i,   // Which third: 0=bits[31:0], 1=bits[63:32], 2=bits[95:64]
    input  logic [31:0] cpu_data_i    // 32-bit data chunk
);

`ifdef TIDE_MACROS
    // ASAP7 fakeram7_256x34: 2 depth banks (addr[8]) x 3 32-bit slices. ONE port: the CPU write (commit phase, engine
    // disabled) steals the port for one cycle; the sequencer read is "always on" otherwise (ce=1, we=0).
    logic [63:0] cpu_staging;
    always_ff @(posedge clk_i) if (cpu_we_i) begin
        if (cpu_word_i == 2'b00) cpu_staging[31:0]  <= cpu_data_i;
        if (cpu_word_i == 2'b01) cpu_staging[63:32] <= cpu_data_i;
    end
    wire         wr_commit = cpu_we_i && (cpu_word_i == 2'b10);
    wire [8:0]   m_addr    = wr_commit ? cpu_addr_i : seq_addr_i;
    wire [95:0]  m_wdata   = {cpu_data_i, cpu_staging};
    logic        bank_q;
    always_ff @(posedge clk_i) bank_q <= m_addr[8];
    logic [33:0] rd [0:1][0:2];
    for (genvar bk = 0; bk < 2; bk++) begin : g_bank
        for (genvar sl = 0; sl < 3; sl++) begin : g_slice
            fakeram7_256x34 u_m (.clk(clk_i), .ce_in(m_addr[8] == bk), .we_in(wr_commit), .addr_in(m_addr[7:0]),
                                 .wd_in({2'b00, m_wdata[32*sl +: 32]}), .rd_out(rd[bk][sl]));
        end
    end
    assign seq_data_o = {rd[bank_q][2][31:0], rd[bank_q][1][31:0], rd[bank_q][0][31:0]};
`else
    // Storage. Contents are undefined/uninitialized at power-up per
    // design doc §4; firmware must load microcode before enabling the
    // engine. (For simulation, initial value is 'x' via the tool's
    // default; no reset value is asserted, matching the spec.)
    logic [95:0] mem [0:511];

    // Staging register for CPU writes (holds the first two 32-bit thirds
    // until the third arrives and commits).
    logic [63:0] cpu_staging;

    // Sequencer read (synchronous, 1-cycle latency, always active)
    always_ff @(posedge clk_i) begin
        seq_data_o <= mem[seq_addr_i];
    end

    // CPU write (3-phase commit protocol)
    always_ff @(posedge clk_i) begin
        if (cpu_we_i) begin
            case (cpu_word_i)
                2'b00: cpu_staging[31:0]  <= cpu_data_i;
                2'b01: cpu_staging[63:32] <= cpu_data_i;
                2'b10: mem[cpu_addr_i]    <= {cpu_data_i, cpu_staging};
                default: ; // cpu_word_i == 2'b11 is not part of the protocol; ignored.
            endcase
        end
    end

`endif
endmodule
