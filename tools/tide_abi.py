"""tide_abi.py -- register / memory ABI shared by tide_sim.py and microcode/gen_ucode.py.

Contract items (memmap.yaml / Step 0) and Agent-1 ABI additions (ASSUMPTIONS.md A-ABI).
"""
# --- contract ----------------------------------------------------------------
R_EVENT_T = 28                 # temporal controller writes event t_event (u32 us)
R_Z = (29, 30, 31, 32)         # z[0..3]  range, azimuth, elevation, doppler
R_R = (33, 34, 35, 36)         # R[0..3]  measurement variances
R_XBASE = 1                    # x[0..5]  = r1..r6   (ctx words 1..6)
R_PBASE = 7                    # P[0..20] = r7..r27  (ctx words 7..27), packed lower triangle
R_IDX1, R_IDX0, R_ZERO = 125, 126, 127

# --- Agent-1 ABI additions (TC must write these before start_i) ---------------
R_DT_US = 118                  # int32 dt_us = t_event - t_last (Step 0: TC computes)
R_CTX = 119                    # TIDE-RAM word address of the context to update in place
R_QP, R_QV, R_DT0INV = 120, 121, 122   # MMIO QP / QV / DT0INV mirrored as float32

# --- TIDE-RAM (word addresses) -------------------------------------------------
RAM_COMMITTED = 0x240
RAM_SHADOW = 0x260
RAM_QOUT = 0x280               # QUERY result: t_q, x0..x5
RAM_ROWS = 0x300               # 4 rows x 8 words: h0..h5, R_k, y0_k

CTX_WORDS = 28                 # t_last, x[6], P[21]
POISON = 0xFFFFFFFF
SEQ_START_LATENCY = 2          # start_i -> first instruction issue (ucode RAM read latency + decode)
SEQ_DONE_LATENCY = 1           # last retire -> done_o
