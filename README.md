# ICE-SoC v2 / TIDE-EKF — Time-Indexed, Preemptible, Bounded-Lag EKF Engine

**Target:** SCL 180 nm via ChipIN, 5×5 mm die, ≤100 pins, 50 MHz
**Architecture spec:** `TIDE_EKF_SoC_Architecture_and_Research_Spec.md` (v2, 20 Sep 2026)

## What this is

A PicoRV32 RV32IM SoC with a **TIDE engine**: a microcoded, statically scheduled EKF processor with custom FP32 arithmetic lanes (MUL, ADD, DIV/SQRT), a 128-word register file, loadable microcode, and a **temporal controller** that owns a checkpoint ring, measurement journal, shadow replay context, event-boundary preemption, and atomic commit. Late (out-of-sequence) radar measurements are re-integrated on-chip by replaying from the nearest checkpoint, without CPU involvement.

The same FP32 lanes are also accessible to the CPU via PCPI custom instructions (baseline/fallback mode).

## Directory structure

```
ICE_SoC/
├── rtl/                    # SystemVerilog RTL
│   ├── ice_soc_pkg.sv      # System-wide package (address map, constants)
│   ├── ice_soc_top.sv      # Top-level SoC integration
│   ├── ice_clk_rst.sv      # Clock/reset controller
│   ├── ice_bus_decode.sv    # Address decoder
│   ├── ice_mem.sv           # SRAM wrapper
│   ├── ice_sysctrl.sv       # System control registers
│   ├── ice_uart.sv          # UART with timestamped RX FIFO
│   ├── ice_timer.sv         # 64-bit timer + watchdog
│   ├── ice_gpio.sv          # 8-bit GPIO with alt functions
│   ├── ice_spi.sv           # SPI master
│   ├── ice_boot_rom.sv      # Synthesized boot ROM
│   └── fpu/                 # (empty — replaced by TIDE FP lanes)
├── tb/                      # Testbenches (to be created for v2)
│   ├── tb_ice_soc.sv        # System-level testbench
│   └── tb_verilator.cpp     # Verilator C++ harness
├── firmware/                # C/ASM firmware
│   ├── crt0.S               # C runtime startup
│   ├── link.ld              # Linker script
│   ├── mkhex.py             # Binary to hex converter
│   ├── support.c            # Support functions
│   └── drivers/             # Peripheral drivers (TBD)
├── constraints/             # Timing constraints
│   └── ice_soc.sdc          # SDC for 50 MHz
├── synth/                   # Synthesis scripts
│   └── ice_soc_yosys_lint.tcl
├── formal/                  # Formal verification (TBD — see doc/design/44)
├── doc/                     # Documentation
│   ├── TIDE_EKF_Implementation_Master_Plan.md  # Roadmap
│   ├── pad_bonding_worksheet.md                # ChipIN pad/bonding plan
│   └── design/              # 30 detailed design & verification specs
│       ├── 00_MODULE_INDEX.md        # Master index
│       ├── 01–14_*.md                # RTL module specs
│       ├── 20–21_*.md                # Tool specs (assembler, simulator)
│       ├── 30_tide_hal.md            # Firmware HAL spec
│       └── 39–50_*.md                # Verification specs (SVA, formal, UVM, coverage)
├── _archive_v1/             # Archived v1 files (old FMA, CORDIC, hardwired FSM)
├── Makefile                 # Build system
└── README.md                # This file
```

## Build gates

| Gate | Deliverable | Status |
|---|---|---|
| G0 | PDK inspection (SRAM, pads) | Not started |
| G1 | Microcode assembler + cycle-accurate simulator | Not started |
| G2 | Novelty search | Not started |
| G3 | FP lane RTL + V1 (10⁸ random vectors) | Completed |
| G4 | SoC shell + PCPI mode + baselines | Not started |
| G5 | Sequencer + microcode bit-exact (V5) | Not started |
| G6 | Temporal controller + formal (V6+V7+V8) | Not started |
| G7 | FPGA emulation + synthesis + timing closure | Not started |
| G8 | PnR + sign-off + ChipIN deliverables | Not started |

## Key dependencies

- PicoRV32: commit `ef203c2b0a3fb793280f5114941416c425c5b461` (ISC license)
- riscv64-unknown-elf-gcc 13.2.0, picolibc 1.8.6-2
- Verilator 5+ (simulation)
- SymbiYosys (formal verification)
- Berkeley SoftFloat 3 (FP verification reference)

## Documentation

All design and verification specifications are in `doc/design/`. Start with [00_MODULE_INDEX.md](doc/design/00_MODULE_INDEX.md).
