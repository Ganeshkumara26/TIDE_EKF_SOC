// -------------------------------------------------------------------
// tide_sequencer.sv -- VLIW Microcode Sequencer
//
// Contract: shared/contract/tide_sequencer.sv (tide_sequencer_if)
// Design ref: read-only/design/08_tide_sequencer.md
// ISA: shared/contract/isa.yaml, Step-0 decisions: shared/contract/STEP_0_DECISIONS.md
//
// IMPORTANT -- see ASSUMPTIONS.md [A4]:
// The frozen tide_sequencer_if omits the MUL/ADD lane RESULT return paths
// (mul_valid_i/mul_result_i, add_valid_i/add_result_i,
// add_bypass_valid_i/add_bypass_result_i, add_cmp_lt_i) even though the
// lanes' own frozen interfaces (tide_fp32_mul_if, tide_fp32_add_if) do
// produce valid_o/result_o/bypass_valid_o/bypass_result_o/cmp_lt_o, and
// the sequencer is architecturally the only place that can pair a lane's
// result with its (already-consumed) destination register address. A
// module that cannot receive these signals cannot write MUL/ADD results
// to the register file at all, which would make the design non-functional
// and contradicts design doc 08 section 6 ("Write-Back Scheduler") and
// timing.yaml's write_port_allocation table. This is treated as a defect
// in the contract file (most likely dropped during the "task review
// fixes" mentioned in STEP_0_DECISIONS.md), not an intentional design.
// Resolution: the missing ports are restored below, using the exact
// names/widths from design doc 08's interface listing, and the
// discrepancy is logged in ASSUMPTIONS.md as pending cross-agent sign-off.
// All OTHER port names/widths match the frozen tide_sequencer_if exactly.
//
// See ASSUMPTIONS.md for additional logged decisions: [A1] idx0/idx1
// address substitution done here (not in tide_regfile), [A5] IntALU
// (IADD/ISUB/ICMP/ITOF/IMOV/LDLOOP/LDIDX0/LDIDX1) implemented inline
// (no ports exist for an external IntALU in the frozen interface),
// [A6] LSU burst signals tied off (ISA has no opcode assigned to trigger
// a burst), [A7] pipeline bubble/stall mechanics.
// -------------------------------------------------------------------

module tide_sequencer (
    input  logic        clk_i,
    input  logic        rst_ni,

    // Temporal controller
    input  logic        start_i,
    input  logic [8:0]  start_addr_i,
    output logic        running_o,
    output logic        done_o,
    output logic        boundary_o,

    // Microcode RAM
    output logic [8:0]  ucode_addr_o,
    input  logic [95:0] ucode_data_i,

    // Register file
    output logic [6:0]  rf_raddr0_o,
    output logic [6:0]  rf_raddr1_o,
    output logic [6:0]  rf_raddr2_o,
    output logic [6:0]  rf_raddr3_o,
    input  logic [31:0] rf_rdata0_i,
    input  logic [31:0] rf_rdata1_i,
    input  logic [31:0] rf_rdata2_i,
    input  logic [31:0] rf_rdata3_i,
    output logic        rf_wen0_o,
    output logic [6:0]  rf_waddr0_o,
    output logic [31:0] rf_wdata0_o,
    output logic        rf_wen1_o,
    output logic [6:0]  rf_waddr1_o,
    output logic [31:0] rf_wdata1_o,
    output logic        rf_wen2_o,
    output logic [6:0]  rf_waddr2_o,
    output logic [31:0] rf_wdata2_o,

    // LSU
    output logic        lsu_cmd_valid_o,
    output logic        lsu_cmd_we_o,
    output logic [6:0]  lsu_cmd_reg_o,
    output logic [11:0] lsu_cmd_addr_o,
    input  logic        lsu_cmd_ready_i,
    output logic        lsu_burst_start_o,
    output logic        lsu_burst_we_o,
    output logic [6:0]  lsu_burst_rf_base_o,
    output logic [11:0] lsu_burst_ram_base_o,
    output logic [5:0]  lsu_burst_len_o,
    input  logic        lsu_burst_done_i,
    input  logic        lsu_burst_busy_i,

    // FP Lanes
    output logic        mul_valid_o,
    output logic [31:0] mul_a_o,
    output logic [31:0] mul_b_o,
    input  logic        mul_ready_i,
    // -- restored per ASSUMPTIONS.md [A4] --
    input  logic        mul_valid_i,
    input  logic [31:0] mul_result_i,

    output logic        add_valid_o,
    output logic [2:0]  add_op_o,
    output logic [31:0] add_a_o,
    output logic [31:0] add_b_o,
    input  logic        add_ready_i,
    // -- restored per ASSUMPTIONS.md [A4] --
    input  logic        add_valid_i,
    input  logic [31:0] add_result_i,
    input  logic        add_bypass_valid_i,
    input  logic [31:0] add_bypass_result_i,
    input  logic        add_cmp_lt_i,

    output logic        ds_valid_o,
    output logic        ds_op_o,
    output logic [31:0] ds_a_o,
    output logic [31:0] ds_b_o,
    input  logic        ds_busy_i,
    input  logic        ds_done_i,
    input  logic [31:0] ds_result_i,

    // Const ROM
    output logic [5:0]  rom_addr_o,
    input  logic [31:0] rom_data_i,

    // PCPI arbiter mux control
    output logic [31:0] lsu_rdata_o,   // resolved LSU source-register value (STORE data)
    output logic        engine_owns_o
);

    // ===================================================================
    // 0. Small combinational helper functions
    // ===================================================================

    // Sign-extend the 13-bit LSU/ALU immediate field to 32 bits (two's
    // complement), used by IADD/ISUB/ICMP/IMOV.
    function automatic logic [31:0] sext13 (input logic [12:0] v);
        sext13 = {{19{v[12]}}, v};
    endfunction

    // Correctly-rounded (round-to-nearest, ties-to-even) signed 32-bit
    // integer -> IEEE-754 binary32 conversion, per
    // read-only/shared/19_numerical_semantics.md section 1 (RNE is the
    // frozen rounding mode for all TIDE FP operations, including the
    // ITOF conversion used for dt = ITOF(t_event - t_now) * 1e-6).
    function automatic logic [31:0] i2f32 (input logic signed [31:0] val);
        logic        sign;
        logic [31:0] mag;
        int          msb;
        int          i;
        logic [31:0] shifted, m1, m2, m3, m4, m5;
        logic [4:0]  lz;
        logic [7:0]  exp_b;
        logic [23:0] mant24;      // implicit bit + 23 mantissa bits, pre-round
        logic        rnd_bit, stk_bit;
        logic        round_up;
        logic [24:0] mant_rounded; // extra bit to catch mantissa-overflow-on-round
    begin
        sign = val[31];
        mag  = sign ? (~val + 32'd1) : val;

        if (mag == 32'd0) begin
            i2f32 = 32'h0000_0000;
        end else begin
            // [TIMING-3] binary-tree leading-zero normalise (was a 32-step priority chain)
            m1 = (mag[31:16] == 16'd0) ? {mag[15:0], 16'd0} : mag;       lz[4] = (mag[31:16] == 16'd0);
            m2 = (m1[31:24] == 8'd0)   ? {m1[23:0], 8'd0}   : m1;        lz[3] = (m1[31:24] == 8'd0);
            m3 = (m2[31:28] == 4'd0)   ? {m2[27:0], 4'd0}   : m2;        lz[2] = (m2[31:28] == 4'd0);
            m4 = (m3[31:30] == 2'd0)   ? {m3[29:0], 2'd0}   : m3;        lz[1] = (m3[31:30] == 2'd0);
            m5 = (m4[31] == 1'b0)      ? {m4[30:0], 1'b0}   : m4;        lz[0] = (m4[31] == 1'b0);
            msb     = 31 - lz;
            exp_b   = 8'd158 - {3'b0, lz};
            shifted = m5;                      // shifted[31] is the implicit '1'
            mant24  = shifted[31:8];           // bit31=implicit, [30:8]=23 mantissa bits
            rnd_bit = shifted[7];
            stk_bit = |shifted[6:0];
            round_up = rnd_bit & (stk_bit | mant24[0]);
            mant_rounded = {1'b0, mant24} + {24'b0, round_up};

            if (mant_rounded[24]) begin
                // Mantissa rounded up through all 1's: bump exponent,
                // mantissa becomes all-zero (implicit bit absorbs carry).
                exp_b  = exp_b + 8'd1;
                i2f32  = {sign, exp_b, 23'b0};
            end else begin
                i2f32  = {sign, exp_b, mant_rounded[22:0]};
            end
        end
    end
    endfunction

    // ===================================================================
    // 1. Decode-stage instruction register (dreg) and fetch PC
    // ===================================================================
    // See ASSUMPTIONS.md [A7] for the derivation of the 1-cycle startup
    // bubble and the stall/branch-squash mechanics below.

    localparam logic [95:0] NOP_WORD = 96'h0;

    // ADD slot has NO NOP encoding (isa.yaml add_ops: all 8 values of the
    // 3-bit op field are real ops; 3'b000 = FADD, not idle). See
    // ASSUMPTIONS.md [A10]. Any word we synthesize ourselves for a
    // startup bubble, branch squash, or forced-idle-after-boundary must
    // therefore park the ADD slot's destination at r127 (hardwired zero,
    // write discarded) so the always-issued FADD/FSUB/... is harmless.
    // All other slots (MUL/DS/LSU_ALU/CTRL) DO have a real all-zero NOP
    // encoding, so only the ADD.rd field needs to be forced.
    localparam logic [95:0] SAFE_BUBBLE_WORD =
        {23'b0,                 // MUL  [95:73] op=NOP
         3'b0, 7'b0, 7'b0, 7'h7F, // ADD [72:49] op=FADD, ra=rb=0, rd=r127 (discarded)
         16'b0,                 // DS   [48:33] op=NOP
         24'b0,                 // LSU_ALU [32:9] op=NOP
         9'b0};                 // CTRL [8:0] all clear

    logic [8:0]  fetch_pc_q;
    logic [95:0] dreg_q;          // currently executing instruction
    logic [1:0]  warm_q;          // startup-bubble counter (1 cycle; see [A7])
    logic        fresh_q;         // 1 = dreg_q was freshly latched this cycle
                                   // (not a stall/drain repeat); gates all
                                   // side effects so a held instruction is
                                   // never re-applied. See ASSUMPTIONS.md [A7].

    typedef enum logic { S_IDLE, S_RUN } state_e;
    state_e state_q;

    assign running_o    = (state_q == S_RUN);
    // ucode_addr_o assigned below (after `stalled`), see I-SEQ-5

    // ---- Decode of dreg_q into slot fields ----
    // MUL slot [95:73]
    wire [1:0] mul_op    = dreg_q[95:94];
    wire [6:0] mul_ra    = dreg_q[93:87];
    wire [6:0] mul_rb    = dreg_q[86:80];
    wire [6:0] mul_rd    = dreg_q[79:73];
    // ADD slot [72:49]
    wire [2:0] add_op    = dreg_q[72:70];
    wire [6:0] add_ra    = dreg_q[69:63];
    wire [6:0] add_rb    = dreg_q[62:56];
    wire [6:0] add_rd    = dreg_q[55:49];
    // DS slot [48:33]
    wire [1:0] ds_op     = dreg_q[48:47];
    wire [6:0] ds_ra     = dreg_q[46:40];
    wire [6:0] ds_rb     = dreg_q[39:33];
    // LSU/ALU slot [32:9]
    wire [3:0]  lsu_op   = dreg_q[32:29];
    wire [6:0]  lsu_reg  = dreg_q[28:22];
    wire [12:0] lsu_imm  = dreg_q[21:9];
    // CTRL slot [8:0]
    wire ctrl_end          = dreg_q[8];
    wire ctrl_boundary     = dreg_q[7];
    wire ctrl_branch       = dreg_q[6];
    wire ctrl_branch_cond  = dreg_q[5];
    wire ctrl_branch_sense = dreg_q[4];
    wire ctrl_loop_dec     = dreg_q[3];
    wire ctrl_loop_branch  = dreg_q[2];
    wire [1:0] ctrl_idx_mode = dreg_q[1:0];

    // ===================================================================
    // 2. idx0 / idx1 index registers, loop counter, cmp_lt latch
    // ===================================================================
    // [INTEGRATION FIX I-SEQ-2] idx0/idx1 are 12 bits wide (they hold TIDE-RAM
    // word addresses such as the context base 0x240; tide_sim.py masks with
    // 0xFFF). They were 7 bits, which truncated 0x240 to 0x40.
    logic [11:0] idx0_q, idx1_q;
    logic [15:0] loop_cnt_q;
    logic        cmp_lt_q;

    // ===================================================================
    // 3. Operand read/substitution
    // ===================================================================
    // Per ASSUMPTIONS.md [A1]: addresses 7'h7E (idx0) and 7'h7D (idx1) are
    // intercepted here and never presented as "real" reads/writes to
    // tide_regfile; 7'h7F (zero) is already handled inside tide_regfile
    // itself and simply passed through.
    function automatic logic [31:0] rd_value (input logic [6:0] addr, input logic [31:0] rf_data);
        if (addr == 7'h7E) rd_value = {20'b0, idx0_q};
        else if (addr == 7'h7D) rd_value = {20'b0, idx1_q};
        else rd_value = rf_data;
    endfunction

    // Read port 0/1: MUL ra/rb, shared with DS ra/rb (R1: assembler
    // guarantees equal addresses when both slots are simultaneously
    // active, so a simple priority mux is safe).
    wire mul_active = (mul_op == 2'b01);
    wire ds_active  = (ds_op  != 2'b00);
    // ADD has no NOP encoding (see ASSUMPTIONS.md [A10]): whatever is in
    // the ADD slot is always a real op whenever dreg_q is being retired.
    // "add_active" therefore just means "ADD lane should issue this
    // cycle", which is gated below by fresh_q (not repeated on holds).
    // LSU_ALU ops that need to *read* the `reg` field as a source operand
    // (as opposed to pure destinations like LOAD/LOAD_ROM/IMOV, or
    // no-operand ops like NOP/branch-target-only).
    wire lsu_alu_reads_reg = (lsu_op == 4'b0010) || // STORE (reg = data source)
                             (lsu_op == 4'b0100) || // IADD
                             (lsu_op == 4'b0101) || // ISUB
                             (lsu_op == 4'b0110) || // ICMP
                             (lsu_op == 4'b0111) || // ITOF
                             (lsu_op == 4'b1000) || // LDLOOP
                             (lsu_op == 4'b1001) || // LDIDX0
                             (lsu_op == 4'b1010);   // LDIDX1

    // [INTEGRATION FIX I-SEQ-8] ports 0/1 likewise (tide_asm rule R1: MUL and DS
    // may share the two ports with ANY assignment as long as their combined
    // source set has <= 2 distinct registers, e.g. MUL r42,r39,r39 with
    // SQRT r57,r33).  Distinct registers are assigned in order to port 0 then
    // port 1; each consumer picks the port holding its register.
    wire        ds_binary  = (ds_op == 2'b01);
    wire [6:0]  p01_c0     = mul_active ? mul_ra : ds_ra;
    wire        p01_c0_v   = mul_active | ds_active;
    wire        p01_c1_v   = mul_active & (mul_rb != mul_ra);
    wire        p01_ds_a_new = ds_active & ~(mul_active & (ds_ra == mul_ra)) & ~(mul_active & (ds_ra == mul_rb));
    wire        p01_ds_b_in  = ds_active & ds_binary;
    // second distinct register (if any)
    logic [6:0] p01_r1;
    always_comb begin
        p01_r1 = 7'd0;
        if (mul_active && mul_rb != mul_ra) p01_r1 = mul_rb;
        else if (mul_active && ds_active && ds_ra != mul_ra) p01_r1 = ds_ra;
        else if (mul_active && ds_binary && ds_rb != mul_ra) p01_r1 = ds_rb;
        else if (!mul_active && ds_binary && ds_rb != ds_ra) p01_r1 = ds_rb;
    end
    assign rf_raddr0_o = p01_c0_v ? p01_c0 : 7'd0;
    assign rf_raddr1_o = p01_r1;
    function automatic logic [31:0] p01_val(input logic [6:0] r, input logic [31:0] d0, input logic [31:0] d1);
        p01_val = (r == p01_c0) ? rd_value(r, d0) : rd_value(r, d1);
    endfunction
    // R2 (per timing.yaml read_port_allocation): ADD and LSU/ALU share
    // read ports 2/3. Since ADD is structurally always "in use" (no NOP
    // encoding, see [A10]) but LSU_ALU's need for a source read is the
    // exception rather than the rule, prioritize LSU_ALU's specific need
    // explicitly so its read is always correct regardless of the
    // assembler's choice of ADD.ra that cycle; fall back to ADD's own ra
    // otherwise. (LSU_ALU never needs two source registers, so port 3 is
    // always free for ADD's rb.)
    // [INTEGRATION FIX I-SEQ-7] read-port 2/3 allocation.  tide_asm.py rule R2
    // allows an ADD plus a register-reading LSU/ALU op as long as the union of
    // source registers is <= 2 distinct registers, in ANY assignment (e.g.
    // unary ABS r122,r49 together with ITOF r120).  The old fixed mapping
    // (LSU forced onto port 2) fed the ADD the LSU's register instead.  Now:
    // ADD ra -> port 2 always; ADD rb -> port 3 if binary; a register-reading
    // LSU op uses whichever port already carries its register, else the free
    // port 3.  The resolved LSU source value is exported (lsu_rdata_o) since the
    // LSU STORE data can no longer be assumed to be on port 2.
    wire add_rb_used  = (add_op <= 3'd4);
    wire lsu_on_p2    = lsu_alu_reads_reg & (lsu_reg == add_ra);
    wire add_b_same_a = add_rb_used & (add_rb == add_ra);      // both ADD operands are one register -> only port 2 needed
    wire add_b_on_p3  = add_rb_used & ~add_b_same_a;
    wire lsu_free_p3  = lsu_alu_reads_reg & ~lsu_on_p2 & ~(add_b_on_p3 & (lsu_reg == add_rb));
    assign rf_raddr2_o = add_ra;
    assign rf_raddr3_o = lsu_free_p3 ? lsu_reg : add_rb;

    wire [31:0] mul_a_val   = p01_val(mul_ra, rf_rdata0_i, rf_rdata1_i);
    wire [31:0] mul_b_val   = p01_val(mul_rb, rf_rdata0_i, rf_rdata1_i);
    wire [31:0] ds_a_val    = p01_val(ds_ra, rf_rdata0_i, rf_rdata1_i);
    wire [31:0] ds_b_val    = p01_val(ds_rb, rf_rdata0_i, rf_rdata1_i);
    wire [31:0] add_a_val   = rd_value(add_ra, rf_rdata2_i);
    wire [31:0] add_b_val   = rd_value(add_rb, add_b_same_a ? rf_rdata2_i : rf_rdata3_i);
    wire [31:0] lsu_reg_val = rd_value(lsu_reg, lsu_on_p2 ? rf_rdata2_i : rf_rdata3_i);

    // ===================================================================
    // 4. Branch / loop resolution (combinational, from dreg_q)
    // ===================================================================
    wire [8:0] branch_target = lsu_imm[8:0];
    wire branch_cond_taken = ctrl_branch_cond ? (ctrl_branch_sense ? cmp_lt_q : ~cmp_lt_q) : 1'b1;
    wire regular_branch_taken = ctrl_branch & branch_cond_taken;
    wire [15:0] loop_cnt_after_dec = ctrl_loop_dec ? (loop_cnt_q - 16'd1) : loop_cnt_q;
    // [TIMING-2] (cnt-1 != 0) == (cnt != 1): avoids the 16-bit decrement in the
    // loop-branch -> ucode-address path (I-SEQ-5).  Same function as before.
    wire loop_after_nz = ctrl_loop_dec ? (loop_cnt_q != 16'd1) : (loop_cnt_q != 16'd0);
    wire loop_branch_taken = ctrl_loop_branch & loop_after_nz;
    wire branch_taken = regular_branch_taken | loop_branch_taken;

    // ===================================================================
    // 5. MUL / ADD / DS issue and write-back tracking
    // ===================================================================
    // -- MUL: fixed 2-cycle latency, back-to-back issue legal. A 2-stage
    //    shift register carries the destination address alongside the
    //    lane's own pipeline; mul_valid_i (restored input, see [A4])
    //    tells us exactly which cycles a write is real.
    logic [6:0] mul_rd_q1, mul_rd_q2;
    logic       mul_pend_q1, mul_pend_q2;

    // -- ADD: two independent write-back paths sharing port 1.
    //    Pipeline (FADD=000/FSUB=001): 3-cycle latency.
    //    Bypass   (FCMP=010/FMIN=011/FMAX=100/FABS=101/FNEG=110/FMOV=111):
    //    1-cycle latency. NOTE: per isa.yaml add_ops, ALL 8 encodings are
    //    real ops -- there is no ADD NOP (see [A10]) -- so add_is_pipe /
    //    add_is_bypass partition the full op space with no "inactive"
    //    case; whether anything is actually ISSUED this cycle is decided
    //    solely by fresh_q below.
    wire add_is_pipe   = (add_op == 3'b000) || (add_op == 3'b001);
    wire add_is_bypass = ~add_is_pipe;

    logic [6:0] add_pipe_rd_q1, add_pipe_rd_q2, add_pipe_rd_q3;
    logic       add_pipe_pend_q1, add_pipe_pend_q2, add_pipe_pend_q3;
    logic [6:0] add_byp_rd_q1;
    logic       add_byp_pend_q1;

    // -- DS: only one in flight at a time (ds_busy_i enforces this).
    //    Destination register comes from the LSU/ALU slot's `reg` field
    //    of the SAME instruction that issues the DS op (STEP_0 decision
    //    "m-4": ds destination = reg field of LSU/ALU slot; lsu_op must
    //    be NOP whenever ds_op != NOP).
    logic [6:0] ds_rd_pending_q;

    // ---- Real issue conditions, gated by fresh_q so a held/repeated
    // dreg_q (LSU stall or end-of-unit drain) is never re-issued to a
    // lane or re-applied to idx0/idx1/loop_cnt/cmp_lt/port-2. See
    // ASSUMPTIONS.md [A7] / [A10].
    wire mul_issue_now = fresh_q & mul_active;
    // [INTEGRATION FIX I-SEQ-6] ADD-slot NOP.  The assembler (tide_asm.py,
    // A-ADDNOP) encodes "no ADD this cycle" as the all-zero 24-bit ADD field,
    // and tide_sim.py skips it.  This module's [A10] assumed instead that the
    // assembler pads with rd=r127; it does not, so every NOP cycle issued a
    // real FADD r0,r0 -> r0 (junk writes to r0 and spurious port-1 traffic).
    // An all-zero ADD field is now a NOP.  (An explicit ADD r0,r0,r0 is
    // rejected by the assembler, so the encoding is unambiguous.)
    wire add_nop       = (dreg_q[72:49] == 24'd0);
    wire add_issue_now = fresh_q & ~add_nop;
    wire ds_issue_now  = fresh_q & ds_active & ~ds_busy_i;
    wire lsu_alu_issue_now = fresh_q;          // gates internal IntALU/idx/loop side effects

    // in-flight tracking for end-of-unit drain (design doc 08 sec 8:
    // only MUL/ADD/DS are drained; LSU is excluded by the doc's own text)
    wire mul_inflight = mul_issue_now | mul_pend_q1 | mul_pend_q2;
    wire add_inflight = (add_issue_now & add_is_pipe) | add_pipe_pend_q1 | add_pipe_pend_q2 | add_pipe_pend_q3 |
                         (add_issue_now & add_is_bypass) | add_byp_pend_q1;
    wire ds_inflight  = ds_busy_i | ds_issue_now;
    wire inflight_any = mul_inflight | add_inflight | ds_inflight;

    // ===================================================================
    // 6. LSU issue (single LOAD/STORE only; see ASSUMPTIONS.md [A6] for
    //    why burst is tied off)
    // ===================================================================
    wire lsu_is_load  = (lsu_op == 4'b0001);
    wire lsu_is_store = (lsu_op == 4'b0010);
    wire lsu_issue    = lsu_is_load | lsu_is_store;

    // [INTEGRATION FIX I-SEQ-3] cmd_valid was combinational on the decoded word
    // regardless of state, so (a) an IDLE sequencer with a stale decoded STORE
    // kept writing TIDE-RAM[0] (corrupting checkpoint slot 0) and (b) a held
    // instruction during drain would re-issue every cycle. Gate by running_o &
    // fresh_q like every other side effect. LSU-not-ready retry is NOT
    // supported: the engine guarantees the LSU is idle while the sequencer
    // runs (temporal commands bursts only while sequencer idle); tide_engine
    // asserts lsu_cmd_ready on every fired command.
    assign lsu_cmd_valid_o = lsu_issue & running_o & fresh_q;
    assign lsu_cmd_we_o    = lsu_is_store;
    assign lsu_cmd_reg_o   = lsu_reg;
    // [INTEGRATION FIX I-SEQ-4] indexed addressing: effective address =
    // imm12 + idx0 (mode 01) / idx1 (mode 10), pre-increment value, exactly as
    // tools/tide_sim.py (addr = (imm & 0xFFF) + idx). Was imm only: every
    // "[1+idx0]" access hit checkpoint-slot RAM instead of the live context.
    wire [11:0] lsu_idx_add = (ctrl_idx_mode == 2'b01) ? idx0_q :
                              (ctrl_idx_mode == 2'b10) ? idx1_q : 12'd0;
    assign lsu_cmd_addr_o  = lsu_imm[11:0] + lsu_idx_add;

    // Burst LSU is never triggered by the current frozen ISA (no opcode
    // assigns it). Tie outputs off permanently.
    assign lsu_burst_start_o    = 1'b0;
    assign lsu_burst_we_o       = 1'b0;
    assign lsu_burst_rf_base_o  = 7'd0;
    assign lsu_burst_ram_base_o = 12'd0;
    assign lsu_burst_len_o      = 6'd0;

    wire lsu_stall = lsu_issue & ~lsu_cmd_ready_i;

    // ===================================================================
    // 7. Const ROM + inline IntALU (see ASSUMPTIONS.md [A5])
    // ===================================================================
    assign rom_addr_o = lsu_imm[5:0];

    wire lsu_is_load_rom = (lsu_op == 4'b0011);
    wire lsu_is_iadd     = (lsu_op == 4'b0100);
    wire lsu_is_isub     = (lsu_op == 4'b0101);
    wire lsu_is_icmp     = (lsu_op == 4'b0110);
    wire lsu_is_itof     = (lsu_op == 4'b0111);
    wire lsu_is_ldloop   = (lsu_op == 4'b1000);
    wire lsu_is_ldidx0   = (lsu_op == 4'b1001);
    wire lsu_is_ldidx1   = (lsu_op == 4'b1010);
    wire lsu_is_imov     = (lsu_op == 4'b1011);

    wire [31:0] iadd_result = lsu_reg_val + sext13(lsu_imm);
    wire [31:0] isub_result = lsu_reg_val - sext13(lsu_imm);
    wire        icmp_lt     = ($signed(lsu_reg_val) < $signed(sext13(lsu_imm)));
    wire [31:0] itof_result = i2f32(lsu_reg_val);
    wire [31:0] imov_result = sext13(lsu_imm);

    // Port-2 write source arbitration: at most one of these is active per
    // the assembler's contract (single lsu_op field). Gated by
    // lsu_alu_issue_now (==fresh_q) so a held/repeated instruction (e.g.
    // an end-of-unit drain hold) never re-applies the same write twice.
    wire        port2_wen_local   = lsu_alu_issue_now &
                                     (lsu_is_load_rom | lsu_is_iadd | lsu_is_isub |
                                      lsu_is_itof | lsu_is_imov);
    wire [31:0] port2_wdata_local = lsu_is_load_rom ? rom_data_i    :
                                     lsu_is_iadd     ? iadd_result  :
                                     lsu_is_isub     ? isub_result  :
                                     lsu_is_itof     ? itof_result  :
                                     imov_result; // lsu_is_imov

    // ===================================================================
    // 8. End-of-unit drain / boundary
    // ===================================================================
    logic draining_q;
    wire  drain_active = ctrl_end | draining_q;
    wire  drain_done   = drain_active & ~inflight_any;

    // ===================================================================
    // 9. Stall / hold condition for the whole fetch/decode pipeline
    // ===================================================================
    wire stalled = lsu_stall | drain_active;

    // [INTEGRATION FIX I-SEQ-5] taken-branch penalty.  tide_ucode_ram reads
    // synchronously (1 cycle) and the branch resolves from dreg_q, so with the
    // address always taken from fetch_pc_q the target word reaches dreg_q two
    // cycles after the branch, and the word *after the delay* (B+2, wrong
    // path) executes in between -- measured: the scalar loop's tail ran the
    // CTX_STORE prologue every iteration and wrote the context to the wrong
    // address.  The contract (ISA-07 and tide_sim.py "a taken branch costs one
    // bubble cycle") is: B+1 squashed, target decoded the very next cycle.
    // Achieved by presenting the branch target to the RAM in the resolve cycle
    // itself (RAM output = mem[target] one cycle later = exactly when dreg_q
    // is refilled after the single bubble) and continuing the sequential
    // fetch from target+1.  Cost: branch_taken (dreg_q decode + cmp_lt_q /
    // loop counter) now feeds the microcode-RAM address input -- a real timing
    // path to look at in synthesis.
    wire redirect_now = (state_q == S_RUN) & (warm_q == 2'd0) & ~stalled &
                        ~ctrl_boundary & branch_taken;
    assign ucode_addr_o = redirect_now ? branch_target : fetch_pc_q;

    assign engine_owns_o = running_o;
    assign lsu_rdata_o   = lsu_reg_val;

    // ===================================================================
    // 10. Sequential state update
    // ===================================================================
    always_ff @(posedge clk_i) begin
        if (!rst_ni) begin
            state_q          <= S_IDLE;
            fetch_pc_q       <= 9'd0;
            dreg_q           <= SAFE_BUBBLE_WORD;
            warm_q           <= 2'd0;
            fresh_q          <= 1'b0;
            idx0_q           <= 12'd0;
            idx1_q           <= 12'd0;
            loop_cnt_q       <= 16'd0;
            cmp_lt_q         <= 1'b0;
            mul_rd_q1        <= 7'd0;  mul_rd_q2        <= 7'd0;
            mul_pend_q1      <= 1'b0;  mul_pend_q2      <= 1'b0;
            add_pipe_rd_q1   <= 7'd0;  add_pipe_rd_q2   <= 7'd0;  add_pipe_rd_q3 <= 7'd0;
            add_pipe_pend_q1 <= 1'b0;  add_pipe_pend_q2 <= 1'b0;  add_pipe_pend_q3 <= 1'b0;
            add_byp_rd_q1    <= 7'd0;  add_byp_pend_q1  <= 1'b0;
            ds_rd_pending_q  <= 7'd0;
            draining_q       <= 1'b0;
            done_o           <= 1'b0;
            boundary_o       <= 1'b0;
        end else begin
            // Default single-cycle pulses; overridden below when they fire.
            done_o     <= 1'b0;
            boundary_o <= 1'b0;

            case (state_q)
                // -----------------------------------------------------
                S_IDLE: begin
                    if (start_i) begin
                        state_q    <= S_RUN;
                        fetch_pc_q <= start_addr_i;
                        dreg_q     <= SAFE_BUBBLE_WORD;
                        warm_q     <= 2'd1;   // 1-cycle startup bubble, [A7]
                        draining_q <= 1'b0;
                        fresh_q    <= 1'b0;   // the forced bubble is inert; nothing to issue
                    end
                end

                // -----------------------------------------------------
                S_RUN: begin
                    // ---- Part A: side effects, gated by fresh_q so a
                    // held/repeated dreg_q is never re-applied (ASSUMPTIONS.md
                    // [A7]). These execute every cycle unconditionally
                    // (independent of the warmup/stall/advance branching
                    // below) because the MUL/ADD lanes are separate,
                    // always-ticking pipelines: their completion pulses
                    // arrive a fixed number of REAL cycles after issue
                    // regardless of whether the sequencer itself stalls
                    // in between, so the tracking shift registers must
                    // keep shifting every cycle to stay aligned with them.

                    // idx0 / idx1 update (LDIDX* takes priority over
                    // post-increment; see ASSUMPTIONS.md [A9])
                    if (lsu_alu_issue_now & lsu_is_ldidx0) idx0_q <= lsu_reg_val[11:0];
                    else if (fresh_q & ctrl_idx_mode[0])   idx0_q <= idx0_q + 12'd1;

                    if (lsu_alu_issue_now & lsu_is_ldidx1) idx1_q <= lsu_reg_val[11:0];
                    else if (fresh_q & ctrl_idx_mode[1])   idx1_q <= idx1_q + 12'd1;

                    // loop counter update
                    if (lsu_alu_issue_now & lsu_is_ldloop) loop_cnt_q <= lsu_reg_val[15:0];
                    else if (fresh_q & ctrl_loop_dec)      loop_cnt_q <= loop_cnt_after_dec;

                    // cmp_lt latch update (naturally safe without explicit
                    // fresh_q gating: add_bypass_valid_i can only pulse
                    // following a real add_issue_now, which is already
                    // fresh_q-gated at its source)
                    if (add_bypass_valid_i)             cmp_lt_q <= add_cmp_lt_i;
                    else if (lsu_alu_issue_now & lsu_is_icmp) cmp_lt_q <= icmp_lt;

                    // MUL shift register -- shifts every cycle; only the
                    // newly-inserted pend bit depends on a real issue.
                    mul_rd_q2   <= mul_rd_q1;
                    mul_rd_q1   <= mul_rd;
                    mul_pend_q2 <= mul_pend_q1;
                    mul_pend_q1 <= mul_issue_now;

                    // ADD pipe shift register (3-stage)
                    add_pipe_rd_q3   <= add_pipe_rd_q2;
                    add_pipe_rd_q2   <= add_pipe_rd_q1;
                    add_pipe_rd_q1   <= add_rd;
                    add_pipe_pend_q3 <= add_pipe_pend_q2;
                    add_pipe_pend_q2 <= add_pipe_pend_q1;
                    add_pipe_pend_q1 <= add_issue_now & add_is_pipe;

                    // ADD bypass shift register (1-stage)
                    add_byp_rd_q1   <= add_rd;
                    add_byp_pend_q1 <= add_issue_now & add_is_bypass;

                    // DS destination capture at issue
                    if (ds_issue_now) ds_rd_pending_q <= lsu_reg;

                    // ---- Part B: fetch/decode pipeline control (branch
                    // squash, stall/hold, warmup, boundary, end-of-unit).
                    fresh_q <= ~stalled; // for use NEXT cycle

                    if (warm_q != 2'd0) begin
                        dreg_q     <= SAFE_BUBBLE_WORD;
                        fetch_pc_q <= fetch_pc_q + 9'd1;
                        warm_q     <= warm_q - 2'd1;
                    end else if (stalled) begin
                        // Hold everything; re-attempt next cycle.
                        dreg_q     <= dreg_q;
                        fetch_pc_q <= fetch_pc_q;
                        if (drain_active) begin
                            draining_q <= ~drain_done;
                            if (drain_done) begin
                                done_o  <= 1'b1;
                                state_q <= S_IDLE;
                            end
                        end
                    end else begin
                        // Normal advance: resolve branch/PC using the
                        // instruction retiring THIS cycle (dreg_q).
                        if (ctrl_boundary) begin
                            boundary_o <= 1'b1;
                            state_q    <= S_IDLE;
                            dreg_q     <= SAFE_BUBBLE_WORD;
                            // fetch_pc left as-is; a fresh start_i will redirect it.
                        end else if (branch_taken) begin
                            fetch_pc_q <= branch_target + 9'd1;   // I-SEQ-5: target itself was already presented to the RAM
                            dreg_q     <= SAFE_BUBBLE_WORD; // 1-cycle squash, ISA-07
                        end else begin
                            fetch_pc_q <= fetch_pc_q + 9'd1;
                            dreg_q     <= ucode_data_i;
                        end

                        // end-of-unit drain entry / immediate completion
                        if (ctrl_end) begin
                            if (inflight_any) begin
                                draining_q <= 1'b1;
                            end else begin
                                done_o  <= 1'b1;
                                state_q <= S_IDLE;
                            end
                        end
                    end
                end
            endcase
        end
    end

    // ===================================================================
    // 11. Register-file write ports
    // ===================================================================
    // Port 0: MUL result only.
    assign rf_wen0_o   = mul_valid_i & (mul_rd_q2 != 7'h7E) & (mul_rd_q2 != 7'h7D);
    assign rf_waddr0_o = mul_rd_q2;
    assign rf_wdata0_o = mul_result_i;

    // Port 1: ADD/SUB pipe done, ADD bypass done, or DS done.
    //
    // NOTE (fixed during integration): the assembler pads every unused ADD
    // slot with a filler FADD(0,0,r127) rather than a true bit-level NOP
    // (r127 is the hardwired-discard/sink register -- see tide_regfile.sv).
    // That filler issues almost every cycle, so add_valid_i (the raw
    // 3-cycle-delayed ADD/SUB pipe-path completion pulse) is asserted
    // almost every cycle too. Two bugs compounded on top of that:
    //   (1) add_pipe_rd_q1/q2/q3 (the pipe rd shift register) was loaded
    //       unconditionally from add_rd on every ADD-slot issue, including
    //       *bypass* issues (FMOV/FMIN/FMAX/...). A bypass instruction's rd
    //       would ride through this shift register and surface 3 cycles
    //       later, coinciding with a later filler's genuine pipe
    //       completion pulse and causing a spurious write of *zero* data
    //       (the filler's own result) to the bypass instruction's real
    //       destination register.
    //   (2) The arbiter previously used "rd != 127" as a proxy for "this
    //       source's completion this cycle is a real, assembler-intended
    //       write," which is both unreliable (see bug 1) and unnecessary:
    //       the design already carries purpose-built pend bits
    //       (add_pipe_pend_q3, add_byp_pend_q1, tracking add_issue_now &
    //       add_is_pipe / add_is_bypass through the same latency as the rd
    //       shift registers) that say directly whether the completion
    //       pulsing this cycle corresponds to an instruction the assembler
    //       actually issued on that path, as opposed to it merely being
    //       "not equal to the sink register by chance."
    // Fixed by qualifying each source's fire with its own pend bit rather
    // than with a same-cycle "!= 127" address heuristic. The assembler's
    // contract (STEP_0 decision C-3) guarantees at most one *real*
    // (pend-qualified) source targets port 1 per cycle, so no additional
    // priority is needed among genuine writers.
    wire        p1_pipe_fire   = add_valid_i;          // 3-cycle-delayed ADD/SUB completion
    wire        p1_byp_fire    = add_bypass_valid_i;   // 1-cycle-delayed bypass completion
    wire        p1_ds_fire     = ds_done_i;
    wire        p1_pipe_real   = p1_pipe_fire & add_pipe_pend_q3;
    wire        p1_byp_real    = p1_byp_fire  & add_byp_pend_q1;
    wire        p1_ds_real     = p1_ds_fire;           // DS: only one in flight, ds_busy_i-gated
    wire [6:0]  p1_addr_raw    = p1_ds_real   ? ds_rd_pending_q :
                                 p1_byp_real  ? add_byp_rd_q1   :
                                 p1_pipe_real ? add_pipe_rd_q3  :
                                 7'h7F;
    wire [31:0] p1_data_raw    = p1_ds_real   ? ds_result_i :
                                 p1_byp_real  ? add_bypass_result_i :
                                 p1_pipe_real ? add_result_i :
                                 32'b0;
    assign rf_wen1_o   = (p1_pipe_real | p1_byp_real | p1_ds_real) &
                         (p1_addr_raw != 7'h7E) & (p1_addr_raw != 7'h7D);
    assign rf_waddr1_o = p1_addr_raw;
    assign rf_wdata1_o = p1_data_raw;

    // Port 2: LOAD_ROM / IADD / ISUB / ITOF / IMOV (LSU LOAD writes back
    // via the LSU module's own direct RF connection at the top level, per
    // ASSUMPTIONS.md -- the sequencer only issues the command).
    //
    // [INTEGRATION FIX I-SEQ-1, found when tide_engine.sv wired the real
    // blocks together]: these results are registered one cycle so the
    // port-2 write lands at C+1, as shared/contract/timing.yaml
    // (port 2 write_cycle: C+1) and tools/tide_sim.py (_sched(t+1,...)) both
    // require -- and exactly where tide_lsu's single-LOAD write-back lands
    // (pend_q). Writing in the issue cycle (C+0) both collided with the
    // previous cycle's LOAD on port 2 and exposed the new value one cycle
    // earlier than the microcode scheduler's register allocator assumes.
    logic        p2_wen_q;
    logic [6:0]  p2_addr_q;
    logic [31:0] p2_data_q;
    always_ff @(posedge clk_i) begin
        if (!rst_ni) begin
            p2_wen_q <= 1'b0; p2_addr_q <= 7'd0; p2_data_q <= 32'd0;
        end else begin
            p2_wen_q  <= port2_wen_local & (lsu_reg != 7'h7E) & (lsu_reg != 7'h7D);
            p2_addr_q <= lsu_reg;
            p2_data_q <= port2_wdata_local;
        end
    end
    assign rf_wen2_o   = p2_wen_q;
    assign rf_waddr2_o = p2_addr_q;
    assign rf_wdata2_o = p2_data_q;

    // ===================================================================
    // 12. FP lane drive
    // ===================================================================
    assign mul_valid_o = mul_issue_now;
    assign mul_a_o      = mul_a_val;
    assign mul_b_o      = mul_b_val;

    assign add_valid_o = add_issue_now;
    assign add_op_o     = add_op;
    assign add_a_o      = add_a_val;
    assign add_b_o      = add_b_val;

    assign ds_valid_o = ds_issue_now;
    // ISA: 01=FDIV,10=FSQRT -> lane: 0=DIV,1=SQRT.
    // NOTE (fixed during integration): ds_op[0] was backwards (FDIV=01 has
    // bit0=1, FSQRT=10 has bit0=0 -- exactly inverted vs the lane's 0=DIV/
    // 1=SQRT convention). ds_op[1] gives the correct mapping (FDIV bit1=0,
    // FSQRT bit1=1). Confirmed against isa.yaml ds_ops table and verified
    // by RTL simulation (see PROVENANCE.md).
    assign ds_op_o     = ds_op[1];
    assign ds_a_o      = ds_a_val;
    assign ds_b_o      = ds_b_val;

endmodule
