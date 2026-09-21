# TIDE EKF SoC Validation

**PicoRV32 | Bare-metal C | RISC-V (rv32im)**

This repository contains the golden model implementation and SoC simulation testbench for the TIDE Extended Kalman Filter engine.

## The Architecture

Inspired by radar-inertial georeferencing coprocessors, running complex EKF algorithms deterministically on edge hardware requires bypassing OS-level jitter. This project targets bare-metal execution to guarantee microsecond-level precision for sensor fusion.

To rigorously validate the DSP mathematics and hardware-software interaction prior to silicon deployment, we utilize a cycle-accurate Verilog RTL simulation of the PicoRV32 core.

## Current Progress

The codebase currently implements:

1. **Golden Model (src/golden_model)**: The structure-native engine model (packed 21-element P, block predict, implicit-H sequential update). It includes single and double-precision models, validated against a traditional dense EKF baseline.
2. **SoC Simulation (`sim/picorv32_bench`)**: Firmware testbenches (`bench_tide.c`, `bench_blk.c`) and RTL testbench (`tb.v`) for cycle-accurate performance measurements of the TIDE-EKF engine running natively on the PicoRV32 processor.

## Repository Structure

- src/golden_model/ — TIDE-EKF core engine, dense EKF reference, and regression tests.
- sim/picorv32_bench/ — PicoRV32 Verilog RTL testbench, linker scripts, and bare-metal entry code.
