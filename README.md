# TIDE: Temporal Instruction Dataflow Engine for Real-Time Robot State Estimation

## Overview
TIDE (Temporal Instruction Dataflow Engine) is a domain-specific coprocessor designed for high-rate Extended Kalman Filtering (EKF), multi-sensor fusion, and asynchronous Out-of-Sequence Measurement (OOSM) reconciliation in robotics and autonomous aerospace navigation platforms.

Traditional robotic state estimation software running on general-purpose CPUs experiences severe latency jitter, memory cache bottlenecks, and high computational overhead during matrix covariance propagation and correction steps ($O(N^3)$ matrix operations). TIDE resolves this via a dual-pipeline architecture:
1. **Spatial Execution Datapath**: Fully pipelined IEEE 754 single-precision floating-point (FP32) arithmetic execution lanes (FP32 Add/Sub, Multiplier, Divider/Square-Root, and Integer ALU) engineered for deterministic single-cycle throughput and multi-cycle execution latencies.
2. **Temporal Event Controller**: Hardware circular journal memory and time-ordered priority event queues that absorb out-of-order sensor observations (e.g., delayed visual odometry, LiDAR point clouds, or GNSS fixes) and dynamically roll back/replay state estimates without stalling host CPU threads.

---

## Processor Architecture & Subsystems

```
                                +---------------------------------------------------------------+
                                |                    TIDE COPROCESSOR TOP-LEVEL                 |
                                |                                                               |
  Microcode Memory              |  +--------------------+             +-----------------------+ |
  (ucode.hex) =================>|  |  VLIW Sequencer    |============>| Architectural Regfile | |
                                |  |  (64-bit Horiz.)   |             | (64x32 Dual-Ported)   | |
                                |  +--------------------+             +-----------------------+ |
                                |            |                                    |             |
                                |            v                                    v             |
                                |  +--------------------+             +-----------------------+ |
  Sensor Event FIFO             |  | Temporal Manager   |             | Arithmetic Lanes      | |
  (IMU, Radar, LiDAR) =========>|  | (Circular Journal) |             | (Add, Mul, Div, Sqrt) | |
                                |  +--------------------+             +-----------------------+ |
                                +---------------------------------------------------------------+
```

### 1. Arithmetic Execution Lanes (`rtl/`)
- `tide_fp32_add.sv`: 2-cycle pipelined IEEE 754 single-precision floating-point adder/subtractor with complete subnormal number representation support, near/far path cancellation, and IEEE 754 exception flags.
- `tide_fp32_mul.sv`: 2-cycle pipelined FP32 multiplier with sign-magnitude decomposition, rounding logic (round-to-nearest-even), and overflow/underflow detection.
- `tide_fp32_divsqrt.sv`: Radix-4 non-restoring divider and square-root unit with deterministic cycle latency, supporting exact IEEE 754 remainder generation.
- `tide_fp32_intalu.sv`: Single-cycle integer ALU for index calculation, circular buffer pointer arithmetic, and conditional branching.
- `tide_const_rom.sv`: On-chip read-only memory containing precomputed mathematical constants ($1.0, 0.5, \pi, \sqrt{2}$, zero vectors, and identity matrix diagonals).

### 2. VLIW Microcode Sequencer & Register File (`rtl/tide_sequencer.sv`, `rtl/tide_regfile.sv`)
- **64-bit VLIW Instruction Set**: Horizontal microcode encoding supporting simultaneous multi-issue dispatch across the arithmetic lanes, load-store unit, and branch logic.
- **Architectural Register File (`rtl/tide_regfile.sv`)**: Multi-ported register file supporting concurrent dual-read and dual-write operations for matrix-vector state transformations.
- **Microcode RAM (`rtl/tide_ucode_ram.sv`)**: Dual-port high-speed memory storing compiled EKF prediction and measurement update microcode programs.

### 3. Temporal Out-of-Sequence Event Controller (`rtl/tide_temporal.sv`)
- **OOSM Hardware Reconciliation**: Absorbs sensor observations that arrive out of chronological order due to variable perception and communication latencies.
- **Circular Journal SRAM (`rtl/tide_ram.sv`)**: Hardware ring buffer maintaining historical state estimates $\hat{x}(\tau)$ and covariance matrices $P(\tau)$.
- **Time-Tagged Event FIFO (`rtl/tide_event_fifo.sv`)**: Asynchronous dual-clock FIFO decoupling sensor input clocks from the internal core processing clock.
- **Load-Store Unit (`rtl/tide_lsu.sv`)**: Interfaces the register file directly with internal journal memory.

### 4. Engine Top-Level Integration (`rtl/tide_engine.sv`)
- Unifies the sequencer, register file, temporal manager, microcode memory, and arithmetic execution lanes into a single hardware coprocessor core with standard AXI/memory-mapped bus interfaces.

---

## Silicon Proven Submodules (SCL 180nm CMOS Tapeouts)

This repository incorporates the two physical silicon-proven SCL 180nm taped-out chips as submodules:
- **[`submodules/scl_subnorm_fp32_tapeout`](https://github.com/Ganeshkumara26/scl_subnorm_fp32_tapeout)**: Silicon Chip OS0005 - Dedicated Subnormal IEEE 754 FP32 arithmetic coprocessor ($2.0 \times 2.0\text{ mm}^2$, SCL 180nm CMOS 1P6M, clean DRC & LVS sign-off).
- **[`submodules/scl_tide_late_tapeout`](https://github.com/Ganeshkumara26/scl_tide_late_tapeout)**: Silicon Chip OS0035 - TIDE Out-of-Sequence Measurement temporal reconciliation engine ($2.0 \times 2.0\text{ mm}^2$, SCL 180nm CMOS 1P6M, clean DRC & LVS sign-off).

---

## Verification & Assembly Reports (`sim/results/`)
- `sim/results/verification_report.log`: Comprehensive Icarus Verilog regression report running directed bit-exact vector tests across all floating-point and integer execution units (Adder: 20/20 passed, Multiplier: 12/12 passed, IntALU: 16/16 passed with 0 errors).
- `sim/results/ucode_assembly_report.log`: Full VLIW microcode assembly, hazard detection, and pipeline timing schedule report (492 / 512 microcode words utilized, peak register pressure 67 live, estimated EKF unit cycle budget 1,023 cycles).

---

## Technical Documentation & Engineering Reports

Comprehensive micro-architectural design documents and detailed RTL development narratives capturing engineering decisions are cataloged in [`docs/`](docs/):

### Micro-Architectural Design Specifications
- [`docs/design_micro_isa.md`](docs/design_micro_isa.md) — 96-bit horizontal VLIW micro-instruction format, slot encoding, and execution semantics.
- [`docs/design_timing_contracts.md`](docs/design_timing_contracts.md) — Master cycle-by-cycle execution latency, pipeline contracts, and backpressure rules.
- [`docs/design_temporal_arbitration.md`](docs/design_temporal_arbitration.md) — Temporal reconciliation algorithms, OOSM mathematics, and state replay policies.
- [`docs/design_tide_engine.md`](docs/design_tide_engine.md) — Unified TIDE EKF coprocessor top-level structural integration.
- [`docs/design_tide_event_fifo.md`](docs/design_tide_event_fifo.md) — Asynchronous 197-bit measurement event staging FIFO.
- [`docs/design_tide_sequencer.md`](docs/design_tide_sequencer.md) — VLIW microcode sequencer, loop hardware, and branch prediction.
- [`docs/design_tide_temporal.md`](docs/design_tide_temporal.md) — Temporal controller and out-of-sequence measurement rollback manager.
- [`docs/design_tide_regfile.md`](docs/design_tide_regfile.md) — 128x32 4-read/3-write multi-ported register file architecture.
- [`docs/design_tide_lsu.md`](docs/design_tide_lsu.md) — Load/Store unit and circular scratchpad memory interface.
- [`docs/design_tide_ucode_ram.md`](docs/design_tide_ucode_ram.md) — 512x96 3-phase atomic microcode storage architecture.
- [`docs/design_tide_fp32_add.md`](docs/design_tide_fp32_add.md) — 3-cycle pipelined IEEE-754 FP32 adder/subtractor with subnormal support.
- [`docs/design_tide_fp32_mul.md`](docs/design_tide_fp32_mul.md) — 2-cycle pipelined IEEE-754 FP32 multiplier.
- [`docs/design_tide_fp32_divsqrt.md`](docs/design_tide_fp32_divsqrt.md) — Non-restoring radix-4 FP32 divider and square-root unit.
- [`docs/design_tide_fp32_intalu.md`](docs/design_tide_fp32_intalu.md) — Single-cycle combinational integer ALU and address generation unit.
- [`docs/design_tide_const_rom.md`](docs/design_tide_const_rom.md) — 64x32 hardwired mathematical constant ROM.

### RTL Implementation & Engineering Thought Process
- [`docs/rtl_tide_engine.md`](docs/rtl_tide_engine.md) — Structural top-level binding, write port priority arbitration, and fault handling.
- [`docs/rtl_tide_event_fifo.md`](docs/rtl_tide_event_fifo.md) — Fall-through combinational head readout, overrun guards, and unreset flop partitioning.
- [`docs/rtl_tide_sequencer.md`](docs/rtl_tide_sequencer.md) — VLIW instruction dispatch, hazard resolution, and microcode liveness analysis.
- [`docs/rtl_tide_temporal.md`](docs/rtl_tide_temporal.md) — Timestamp comparator datapath, circular pointer rollback, and OOSM replay state machine.
- [`docs/rtl_tide_regfile.md`](docs/rtl_tide_regfile.md) — Flop array layout vs SRAM, WAW write priority arbitration, and routing congestion management.
- [`docs/rtl_tide_lsu.md`](docs/rtl_tide_lsu.md) — Pipelined burst state save/restore, scratchpad memory protocol, and cycle budget guarantees.
- [`docs/rtl_tide_ram.md`](docs/rtl_tide_ram.md) — Circular state journal memory wrapper, out-of-range address trapping, and simulation poisoning.
- [`docs/rtl_tide_ucode_ram.md`](docs/rtl_tide_ucode_ram.md) — 3-phase atomic MMIO staging protocol and execution-lock write protection.
- [`docs/rtl_tide_fp32_add.md`](docs/rtl_tide_fp32_add.md) — Exponent difference alignment, 27-bit significand addition, and RNE rounding logic.
- [`docs/rtl_tide_fp32_mul.md`](docs/rtl_tide_fp32_mul.md) — 24x24 mantissa multiplier tree, exponent overflow detection, and pipeline register balance.
- [`docs/rtl_tide_fp32_divsqrt.md`](docs/rtl_tide_fp32_divsqrt.md) — 28/27-cycle non-restoring recurrence datapath, quotient normalization, and testbench latency sign-off.
- [`docs/rtl_tide_fp32_intalu.md`](docs/rtl_tide_fp32_intalu.md) — Fast adder/subtractor, barrel shifter, and branch condition flag evaluation.
- [`docs/rtl_tide_const_rom.md`](docs/rtl_tide_const_rom.md) — Synthesizable constant table lookup, zero propagation, and timing path verification.

---

## Repository Layout
```
TIDE_EKF_SOC/
├── .gitignore
├── .gitmodules
├── README.md
├── run_tests.sh
├── docs/                              # Engineering documentation suite
│   ├── design_*.md                    # Micro-architectural design specifications
│   └── rtl_*.md                       # Comprehensive RTL implementation narratives
├── submodules/
│   ├── scl_subnorm_fp32_tapeout/      # Silicon Chip OS0005 Tapeout
│   └── scl_tide_late_tapeout/         # Silicon Chip OS0035 Tapeout
├── sim/
│   ├── results/
│   │   ├── ucode_assembly_report.log  # Microcode compilation & cycle budget report
│   │   └── verification_report.log    # Arithmetic lanes directed testbench report
│   ├── picorv32_bench/                # PicoRV32 RISC-V SoC co-simulation benchmarks
│   ├── 10_unit_pre.hex                # Assembled state prediction microcode kernel
│   └── 20_unit_update.hex             # Assembled measurement update microcode kernel
├── rtl/
│   ├── tide_const_rom.sv              # Mathematical constants ROM
│   ├── tide_engine.sv                 # Unified coprocessor top-level
│   ├── tide_event_fifo.sv             # Dual-clock asynchronous event FIFO
│   ├── tide_fp32_add.sv               # Subnormal-capable FP32 adder/subtractor
│   ├── tide_fp32_divsqrt.sv           # Radix-4 FP32 divider and square-root unit
│   ├── tide_fp32_intalu.sv            # Integer ALU and pointer indexer
│   ├── tide_fp32_mul.sv               # 2-cycle pipelined FP32 multiplier
│   ├── tide_lsu.sv                    # Dedicated load-store unit
│   ├── tide_ram.sv                    # Circular state journal SRAM buffer
│   ├── tide_regfile.sv                # Multi-ported architectural register file
│   ├── tide_sequencer.sv              # 64-bit horizontal VLIW sequencer
│   ├── tide_temporal.sv               # Temporal reconciliation & rollback logic
│   └── tide_ucode_ram.sv              # Microcode RAM block
├── tb/
│   ├── dbg_probe.svh                  # Simulation debug probes
│   ├── tb_const_rom.cpp               # C++ testbench for constants ROM
│   ├── tb_engine_golden.sv            # Full engine golden verification testbench
│   ├── tb_engine_oosm.sv              # Out-of-sequence measurement testbench
│   ├── tb_fp32_add.sv                 # Directed testbench for FP32 adder
│   ├── tb_fp32_divsqrt.sv             # Directed testbench for divider/sqrt
│   ├── tb_fp32_intalu.sv              # Directed testbench for integer ALU
│   ├── tb_fp32_mul.sv                 # Directed testbench for multiplier
│   ├── tb_fp32_random.cpp             # Random test vector generator & runner
│   ├── tb_tide_lsu.sv                 # LSU interface testbench
│   ├── tb_tide_oosm.sv                # Temporal manager testbench
│   └── sva/
│       ├── tide_fp32_add_sva.sv       # SystemVerilog Assertions for adder
│       ├── tide_fp32_divsqrt_sva.sv   # SVA for divider/sqrt
│       ├── tide_fp32_mul_sva.sv       # SVA for multiplier
│       ├── tide_preemption_sva.sv     # SVA for temporal preemption
│       ├── tide_regfile_sva.sv        # SVA for register file
│       ├── tide_sequencer_sva.sv      # SVA for microcode sequencer
│       ├── tide_temporal_sva.sv       # SVA for temporal reconciliation
│       └── tide_ucode_ram_sva.sv      # SVA for microcode memory
├── src/
│   └── golden_model/
│       ├── ekf.h                      # Double-precision EKF header
│       ├── ekf64.c                    # Reference 64-bit floating-point EKF
│       ├── ekf64.h                    # Reference EKF headers
│       ├── ekf_paper.c                # Algorithmic paper baseline model
│       ├── ekf_pilot.c                # Production flight pilot model
│       ├── ekf_pilot.h                # Pilot model interfaces
│       ├── tide_core.c                # Bit-accurate C simulation core
│       ├── tide_core.h                # Core C interfaces
│       ├── tide_core64.h              # 64-bit state definitions
│       ├── tide_test.c                # C test regression runner
│       ├── verify64.c                 # Precision verification routines
│       ├── verify_oosm.c              # C verification for temporal OOSM
│       └── vectors/
│           ├── golden_trace.txt       # Golden trajectory trace vectors
│           ├── op_vectors.txt         # Arithmetic test vectors
│           ├── rom_final.hex          # Constant ROM hex file
│           └── unit_vectors.txt       # Unit test stimulus vectors
├── tools/
│   ├── tide_abi.py                    # Application Binary Interface specifications
│   ├── tide_asm.py                    # Microcode assembler (.tasm -> .hex)
│   └── tide_sim.py                    # Cycle-accurate architectural simulator
├── microcode/
│   ├── 10_unit_pre.tasm               # State and covariance prediction kernel
│   ├── 20_unit_update.tasm            # Measurement correction and Kalman gain kernel
│   ├── 90_query.tasm                  # State vector readout kernel
│   ├── gen_ucode.py                   # Automated microcode generation script
│   └── ucode.hex                      # Assembled 64-bit binary microcode image
└── submodules/
    ├── scl_subnorm_fp32_tapeout/       # SCL 180nm Silicon Proven Coprocessor (OS0005)
    └── scl_tide_late_tapeout/          # SCL 180nm Out-of-Sequence Engine (OS0035)
```

---

## Toolchain & Verification

### 1. Microcode Assembly & Simulation
```bash
# Assemble microcode kernels into binary machine images
python3 tools/tide_asm.py microcode/10_unit_pre.tasm -o /tmp/pre.hex
python3 tools/tide_asm.py microcode/20_unit_update.tasm -o /tmp/update.hex

# Run cycle-accurate architectural simulation
python3 tools/tide_sim.py --trace src/golden_model/vectors/golden_trace.txt
```

### 2. Golden C Model Regression
```bash
# Compile and execute bit-accurate C EKF verification
gcc -O2 -o tide_test src/golden_model/tide_test.c src/golden_model/tide_core.c -lm
./tide_test
```

### 3. Complete Regression Suite
```bash
# Execute full automated test regression
./run_tests.sh
```
