# Design: `tide_ucode_ram.sv` — 512×96-bit Microcode RAM

**Spec reference:** §6.5 ("Microcode RAM: ≈ 400–500 words × 96 bits provisioned as 512 words (≈ 6 KB), loaded by the CPU at boot.")
**Gate:** G5
**New file**

---

## 1. Purpose

Stores the microcode program that controls the TIDE engine. The CPU loads it at boot time (or after a microcode patch) via the MMIO write window at `0x3000_0000`. The sequencer reads one 96-bit instruction per clock cycle during engine operation.

## 2. Interface

```systemverilog
module tide_ucode_ram (
    input  logic        clk_i,
    input  logic        rst_ni,

    // Sequencer read port (one instruction per cycle)
    input  logic [8:0]  seq_addr_i,   // 9-bit address (0–511)
    output logic [95:0] seq_data_o,   // 96-bit microinstruction

    // CPU write port (MMIO: three 32-bit writes fill one 96-bit word)
    input  logic        cpu_we_i,     // Write enable
    input  logic [8:0]  cpu_addr_i,   // Word address (0–511)
    input  logic [1:0]  cpu_word_i,   // Which third: 0=bits[31:0], 1=bits[63:32], 2=bits[95:64]
    input  logic [31:0] cpu_data_i    // 32-bit data chunk
);
```

## 3. CPU Write Protocol

The CPU loads microcode via three sequential 32-bit MMIO writes per 96-bit word, with a 16-byte stride between words:

1. Write `word[31:0]` to `UCODE_BASE + (addr * 16) + 0` → `cpu_word_i = 2'b00`
2. Write `word[63:32]` to `UCODE_BASE + (addr * 16) + 4` → `cpu_word_i = 2'b01`
3. Write `word[95:64]` to `UCODE_BASE + (addr * 16) + 8` → `cpu_word_i = 2'b10`

The write is committed to the RAM array on the third write (when `cpu_word_i == 2'b10`). Internally, a 64-bit staging register holds the first two thirds until the third arrives.

**Alternative simpler protocol:** Write all three words to a shadow register set, then a fourth "commit" write pushes them into the RAM. This is more robust against interrupted writes. **Decision: use the 3-write protocol** as it matches the spec's description of "three 32-bit writes per word."

## 4. Memory Organization

- **Depth:** 512 words.
- **Width:** 96 bits (3 × 32 bits).
- **Total:** 512 × 96 = 49,152 bits ≈ 6 KB.
- **Implementation:** Flop array for simulation and initial synthesis. **If an SRAM macro is available** (PDK inspection, §6.9), replace with a macro instantiation behind an `ifdef`. If no macro and area is tight, implement as a synthesized ROM with a 64-word patch overlay RAM (fallback from §13).
- **Initialization:** UCODE-RAM contents are **random/uninitialized** at power-up. Firmware MUST explicitly load the microcode image before enabling the engine.

## 5. Read Port Behavior

- **Synchronous read** (registered output): address presented on cycle N, data available on cycle N+1.
- This means the sequencer has a 1-cycle instruction fetch latency. The sequencer must account for this (fetch is one pipeline stage ahead of decode/execute).
- The read port is always active. There is no read-enable.

## 6. Write Port Behavior

- Writes are synchronous, on the rising clock edge.
- The hardware **MUST enforce a write lock**. If the engine is enabled (`ENABLE = 1`), all CPU writes to the microcode RAM must be silently discarded. This prevents corruption during engine execution. Firmware must stop the engine before loading or patching microcode.

## 7. Implementation Sketch

```systemverilog
module tide_ucode_ram (
    // ... ports ...
);
    // Storage
    logic [95:0] mem [0:511];

    // Staging register for CPU writes
    logic [63:0] cpu_staging;

    // Sequencer read (synchronous)
    always_ff @(posedge clk_i) begin
        seq_data_o <= mem[seq_addr_i];
    end

    // CPU write (3-phase)
    always_ff @(posedge clk_i) begin
        if (cpu_we_i) begin
            case (cpu_word_i)
                2'b00: cpu_staging[31:0]  <= cpu_data_i;
                2'b01: cpu_staging[63:32] <= cpu_data_i;
                2'b10: mem[cpu_addr_i]    <= {cpu_data_i, cpu_staging};
                default: ; // ignore
            endcase
        end
    end
endmodule
```

## 8. MMIO Address Mapping

The MMIO window at `0x3000_0000` maps to the CPU write port:

| MMIO Address | Computation | Meaning |
|---|---|---|
| `0x3000_0000 + addr*16 + 0` | `cpu_addr = addr`, `cpu_word = 0` | Word N, bits [31:0] |
| `0x3000_0000 + addr*16 + 4` | `cpu_addr = addr`, `cpu_word = 1` | Word N, bits [63:32] |
| `0x3000_0000 + addr*16 + 8` | `cpu_addr = addr`, `cpu_word = 2` | Word N, bits [95:64] (commits) |

The bus decoder extracts `addr` and `word` from the MMIO offset:
```
cpu_addr_i = mmio_offset[12:4]  // bits [12:4] → 9 bits (0–511)
cpu_word_i = mmio_offset[3:2]   // bits [3:2] → 0, 1, or 2
```

**Memory Window:** 512 words × 16 bytes/word = 8192 bytes = 8 KB. The MMIO window `0x3000_0000` to `0x3000_1FFF` covers the entire microcode RAM write interface.

## 9. CRC Check (Boot Integrity)

The spec says (§7.4): "If microcode load fails its CRC, the engine is disabled (MODE = 0)." The CRC check is done in **firmware**, not hardware. The boot ROM loads the microcode, computes a CRC over the loaded data, and compares it to a stored CRC in the flash image. If it fails, firmware sets `MODE = 0` and the engine is never started.

## 10. Verification

| Test | Pass Criterion |
|---|---|
| Write all 512 words via CPU port, read back via sequencer port | Exact match |
| Write a single word (3-phase), verify staging works | Correct |
| Read port latency = exactly 1 cycle | Timing check |
| Write during sequencer read (different addresses) | Both succeed |
| Write during sequencer read (same address) | Read gets old or new (don't care, but no X) |
