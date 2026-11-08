# Design: `tide_top.sv` — TIDE EKF Coprocessor Top-Level Integration

**Spec reference:** §6 (Full TIDE Architecture), §6.5 (Engine Block Diagram)
**Gate:** G7 (Full TIDE Integration)
**New file** — instantiates all TIDE submodules

---

## 1. Purpose

`tide_top.sv` is the structural top-level module that instantiates and interconnects all TIDE EKF coprocessor submodules into a single, port-clean entity. It is the module that the SoC shell instantiates. From the SoC's perspective, TIDE is a black box with an AHB-Lite slave port for CPU access, an event FIFO input for sensor data, and a handful of status/interrupt outputs.

This module contains **no behavioral logic** of its own — only instances, wires, and the lane input muxes that implement the PCPI/sequencer arbitration. All computation happens inside the submodules. The top-level's job is routing.

## 2. Interface

```systemverilog
module tide_top (
    input  logic        clk_i,            // 50 MHz system clock
    input  logic        rst_ni,           // Active-low synchronous reset

    // === AHB-Lite Slave Interface (CPU access) ===
    input  logic        hsel_i,           // Chip select
    input  logic [11:0] haddr_i,          // Address (4 KB decode space)
    input  logic        hwrite_i,         // 1 = write, 0 = read
    input  logic [31:0] hwdata_i,         // Write data
    input  logic [1:0]  htrans_i,         // Transfer type
    input  logic [2:0]  hsize_i,          // Transfer size
    output logic [31:0] hrdata_o,         // Read data
    output logic        hready_o,         // Transfer complete
    output logic        hresp_o,          // Transfer response (0=OK)

    // === PicoRV32 PCPI Interface ===
    input  logic        pcpi_valid_i,
    input  logic [31:0] pcpi_insn_i,
    input  logic [31:0] pcpi_rs1_i,
    input  logic [31:0] pcpi_rs2_i,
    output logic        pcpi_wr_o,
    output logic [31:0] pcpi_rd_o,
    output logic        pcpi_wait_o,
    output logic        pcpi_ready_o,

    // === Event Input (sensor measurement packets) ===
    input  logic        event_valid_i,     // New measurement available
    output logic        event_ready_o,     // TIDE can accept measurement
    input  logic [31:0] event_timestamp_i, // Measurement timestamp (µs)
    input  logic [1:0]  event_type_i,      // Sensor type
    input  logic [127:0] event_data_i,     // 4 × 32-bit measurement words

    // === Interrupt & Status ===
    output logic        irq_done_o,        // EKF epoch complete interrupt
    output logic        irq_oosm_o,        // OOSM event detected interrupt
    output logic        busy_o,            // Engine busy status
    output logic [31:0] t_current_o        // Current estimation timestamp
);
```

## 3. Submodule Instantiation Hierarchy

```
tide_top
├── tide_regfile              (128×32, 4R/3W register file)
├── tide_fp32_mul             (2-cycle pipelined IEEE-754 multiplier)
├── tide_fp32_add             (3-cycle pipelined IEEE-754 adder/comparator)
├── tide_fp32_divsqrt         (28/27-cycle iterative divider/square-root)
├── tide_fp32_intalu          (combinational integer ALU)
├── tide_const_rom            (64×32 constant lookup table)
├── tide_ucode_ram            (512×96 microcode memory)
├── tide_sequencer            (VLIW fetch-decode-execute pipeline)
├── tide_lsu                  (load/store unit with burst engine)
├── tide_temporal             (temporal controller & OOSM coordinator)
├── tide_pcpi                 (PCPI arbiter for CPU FP access)
├── tide_fp_ctrl              (FP exception flags & config registers)
├── tide_ahb_regs             (AHB-Lite slave register decode)
└── tide_event_fifo           (4-deep event FIFO with timestamp)
```

## 4. Internal Wiring

### 4.1 Lane Input Multiplexing

The critical routing decision in `tide_top` is the lane input mux. Two masters (sequencer and PCPI arbiter) drive the same three lanes. The mux uses the `mode` register and the `tide_busy` signal:

```systemverilog
// Lane ownership: TIDE engine has absolute priority
logic tide_owns_lanes;
assign tide_owns_lanes = mode_reg | temporal_busy;

// MUL lane inputs
assign mul_valid = tide_owns_lanes ? seq_mul_valid : pcpi_mul_valid;
assign mul_a     = tide_owns_lanes ? seq_mul_a     : pcpi_mul_a;
assign mul_b     = tide_owns_lanes ? seq_mul_b     : pcpi_mul_b;

// ADD lane inputs
assign add_valid = tide_owns_lanes ? seq_add_valid : pcpi_add_valid;
assign add_op    = tide_owns_lanes ? seq_add_op    : pcpi_add_op;
assign add_a     = tide_owns_lanes ? seq_add_a     : pcpi_add_a;
assign add_b     = tide_owns_lanes ? seq_add_b     : pcpi_add_b;

// DIV/SQRT lane inputs
assign ds_start  = tide_owns_lanes ? seq_ds_start  : pcpi_ds_start;
assign ds_a      = tide_owns_lanes ? seq_ds_a      : pcpi_ds_a;
assign ds_b      = tide_owns_lanes ? seq_ds_b      : pcpi_ds_b;
assign ds_op     = tide_owns_lanes ? seq_ds_op     : pcpi_ds_op;
```

Lane outputs (results, valid signals) are broadcast to both the sequencer and the PCPI arbiter. Each ignores results when it is not the active owner.

### 4.2 Register File Port Routing

The register file has 4 read ports and 3 write ports. Port assignment is fixed:

| Port | Read Consumer | Write Producer |
|---|---|---|
| Read 0 | Sequencer → MUL operand A | — |
| Read 1 | Sequencer → MUL operand B / ADD operand A | — |
| Read 2 | Sequencer → ADD operand B / LSU store data | — |
| Read 3 | Sequencer → LSU address base (indexed modes) | — |
| Write 0 | — | MUL result (via sequencer write-back scheduler) |
| Write 1 | — | ADD result / DIV/SQRT result (via write-back scheduler) |
| Write 2 | — | LSU LOAD data / IntALU result / Const ROM / Temporal controller |

Write port 2 has a priority mux: temporal controller > LSU > IntALU > Const ROM. In practice, only one source drives port 2 per cycle because the temporal controller writes only when the sequencer is idle.

### 4.3 TIDE-RAM Dual-Port Routing

TIDE-RAM port A connects to `tide_ahb_regs` for CPU access. Port B connects to `tide_lsu` for engine-side access. The LSU also serves the temporal controller's journal reads by time-sharing port B when no burst is active.

### 4.4 Event FIFO

A small synchronous FIFO (4 entries deep, ~180 bits per entry) buffers incoming sensor events. The FIFO isolates the CPU-side write timing from the temporal controller's read timing:

```
CPU firmware writes to EVENT_DATA registers → FIFO push
Temporal controller reads from FIFO → FIFO pop
```

The FIFO depth of 4 is sufficient because the worst-case EKF epoch is ~800 cycles (16 µs at 50 MHz), and sensor measurements arrive at most every 20 ms (50 Hz GPS). Even with OOSM replays, the FIFO never fills.

## 5. AHB-Lite Register Map (tide_ahb_regs)

The AHB slave decoder provides the CPU with access to all TIDE control, status, configuration, and data registers. The full register map:

| Offset | Register | Access | Description |
|---|---|---|---|
| `0x000` | `CTRL` | W | Control: bit 0 = start, bit 1 = stop, bit 2 = reset |
| `0x004` | `STATUS` | R | Status: bit 0 = busy, bit 1 = done, bit 2 = oosm, bit 3 = error |
| `0x008` | `MODE` | R/W | bit 0 = mode (0=CPU, 1=TIDE) |
| `0x00C` | `T_CURRENT` | R | Current estimation timestamp (µs) |
| `0x010` | `FP_STATUS` | R | FP exception flags + lane status |
| `0x014` | `FP_FLAGS_CLR` | W | Write-1-to-clear FP exception flags |
| `0x018` | `FP_CONFIG` | R/W | Rounding mode, FTZ |
| `0x01C–0x02C` | `FP_COUNTERS` | R | Performance counters |
| `0x030` | `EVENT_DATA0` | W | Measurement word 0 (push to FIFO) |
| `0x034` | `EVENT_DATA1` | W | Measurement word 1 |
| `0x038` | `EVENT_DATA2` | W | Measurement word 2 |
| `0x03C` | `EVENT_DATA3` | W | Measurement word 3 |
| `0x040` | `EVENT_TS` | W | Measurement timestamp (triggers FIFO push) |
| `0x044` | `EVENT_TYPE` | W | Measurement type (2-bit) |
| `0x048` | `EVENT_STATUS` | R | FIFO status: bit 0 = full, bit 1 = empty, [4:2] = count |
| `0x050–0x07F` | `NOISE_CFG` | R/W | Measurement noise parameters (R matrix) |
| `0x080–0x0FF` | `reserved` | — | — |
| `0x100–0x4FF` | `UCODE_RAM` | R/W | Microcode RAM (96-bit words, 3-phase write) |
| `0x500–0x8FF` | `TIDE_RAM` | R/W | TIDE-RAM direct access (1024 × 32-bit) |
| `0x900–0x93F` | `REGFILE_DBG` | R | Register file read-only debug access (128 × 32) |

## 6. Interrupt Generation

Two interrupt outputs connect to the PicoRV32's IRQ input:

- **`irq_done_o`:** Pulses for 1 cycle when the temporal controller completes an EKF epoch (in-order or OOSM replay finished). The CPU's ISR reads the updated state from TIDE-RAM or the register file debug port.
- **`irq_oosm_o`:** Pulses when an OOSM event is detected (informational, not blocking). The CPU can log the OOSM lateness for telemetry.

Both interrupts are edge-triggered (1-cycle pulses). The PicoRV32's IRQ handler must be fast enough to capture them, or a small edge-to-level converter can be added.

## 7. Clock and Reset Distribution

All submodules share a single clock domain (`clk_i`) and a single active-low synchronous reset (`rst_ni`). There are no clock domain crossings inside TIDE. The event FIFO is synchronous (both push and pop sides use `clk_i`).

Reset deassertion order does not matter because all submodules independently initialize their state machines to IDLE on reset. After reset, the temporal controller waits in IDLE for firmware to configure measurement noise parameters and load microcode before accepting events.

## 8. Power Management

`tide_top` includes optional clock gating for area-sensitive ASIC targets:

```systemverilog
// Optional: gate clocks to idle lanes
logic mul_clk_en, add_clk_en, ds_clk_en;
assign mul_clk_en = tide_owns_lanes ? seq_mul_valid : pcpi_mul_valid;
assign add_clk_en = tide_owns_lanes ? seq_add_valid : pcpi_add_valid;
assign ds_clk_en  = tide_owns_lanes ? seq_ds_start  : pcpi_ds_start;
```

On FPGA, clock gating is replaced with clock enables on the pipeline registers inside each lane. This is handled by the lanes themselves (see docs 01–03).

## 9. Area Summary (All Submodules)

| Submodule | Gate Count | Percentage |
|---|---|---|
| `tide_fp32_mul` | ~20,000 | 22% |
| `tide_fp32_add` | ~18,000 | 20% |
| `tide_fp32_divsqrt` | ~12,000 | 13% |
| `tide_fp32_intalu` | ~2,000 | 2% |
| `tide_regfile` | ~50,000 | 27% (dominated by flops) |
| `tide_const_rom` | ~3,000 | 3% |
| `tide_ucode_ram` | ~8,000 | 9% (depends on SRAM vs flops) |
| `tide_sequencer` | ~4,000 | 4% |
| `tide_lsu` | ~700 | <1% |
| `tide_temporal` | ~1,800 | 2% |
| `tide_pcpi` | ~970 | 1% |
| `tide_fp_ctrl` | ~1,290 | 1% |
| `tide_ahb_regs` | ~1,500 | 2% |
| `tide_event_fifo` | ~800 | <1% |
| Interconnect muxes | ~1,200 | 1% |
| **Total TIDE** | **~92,000** | **100%** |

At SCL 180nm (8 GE/gate average): approximately **0.74 mm²**. This fits comfortably within the spec's 1.0 mm² budget for TIDE.

## 10. Verification Plan

| Test | Method | Pass Criterion |
|---|---|---|
| Structural lint: all ports connected, no floating wires | Verilator --lint-only | Zero warnings |
| Reset: all submodules reach IDLE state | Verilator | No X propagation after reset |
| CPU microcode load → sequencer execution | Verilator | Correct RF state after predict+update |
| PCPI FMUL → lane → CPU result | Verilator | Bit-exact result |
| Event FIFO: push 4 events, pop 4 events | Verilator | FIFO behavior correct |
| Full EKF epoch: event → temporal → sequencer → done | Verilator | State matches golden model |
| OOSM replay: late event → save → restore → correct → forward | Verilator | State matches golden model |
| AHB register read/write | Verilator + cocotb | All registers accessible |
| Interrupt generation: done and oosm IRQs | Verilator | Correct pulse timing |
| Dual-access: CPU reads TIDE-RAM while engine accesses via LSU | Verilator | No data corruption |

## 11. Implementation Notes for the Coder

1. **This module should be purely structural.** No `always` blocks, no combinational logic except the lane muxes and a few assign statements. If you find yourself writing behavioral code here, it belongs in a submodule.

2. **Wire naming convention:** Use `<source>_<signal>` for inter-module wires. Example: `seq_mul_valid`, `pcpi_mul_a`, `tc_burst_start`. This makes the netlist readable during debug.

3. **Parameter propagation:** `DATA_WIDTH`, `UCODE_DEPTH`, `RF_DEPTH`, `RAM_DEPTH` should be parameters on `tide_top` and passed down to submodules. Do not hardcode `128`, `512`, `1024` in submodules.

4. **The AHB slave (`tide_ahb_regs`)** handles the 3-phase microcode write protocol internally. `tide_top` just wires the AHB bus to `tide_ahb_regs` and the decoded signals to the appropriate submodule.

5. **Reset polarity:** All submodules use active-low synchronous reset (`rst_ni`). Do not invert the reset inside this module — pass it through unchanged.

6. **Synthesis boundary:** `tide_top` should be marked as a `dont_touch` hierarchy in synthesis scripts to preserve the module boundary for timing analysis and hierarchical P&R.
