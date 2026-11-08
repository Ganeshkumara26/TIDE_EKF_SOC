#!/usr/bin/env bash
# TIDE EKF Coprocessor Regression Suite
set -e

echo "=== Running TIDE RTL & Golden Model Verification ==="

# 1. Run Golden Model Verification
echo "[1/3] Verifying C Golden Model against trace vectors..."
gcc -O2 -o src/golden_model/tide_test src/golden_model/tide_test.c src/golden_model/tide_core.c -lm
./src/golden_model/tide_test
rm -f src/golden_model/tide_test

# 2. Assemble Microcode Kernels
echo "[2/3] Validating Microcode Assembly..."
python3 tools/tide_asm.py microcode/10_unit_pre.tasm -o /tmp/pre.hex
python3 tools/tide_asm.py microcode/20_unit_update.tasm -o /tmp/update.hex

# 3. Run RTL Simulation (if Icarus Verilog is available)
if command -v iverilog &> /dev/null; then
    echo "[3/3] Running Icarus Verilog Simulation on Arithmetic Lanes..."
    iverilog -g2012 -o /tmp/tb_fp32_add rtl/tide_fp32_add.sv rtl/tide_const_rom.sv tb/tb_fp32_add.sv
    vvp /tmp/tb_fp32_add
    rm -f /tmp/tb_fp32_add
fi

echo "=== All TIDE Verification Steps Passed Successfully ==="
