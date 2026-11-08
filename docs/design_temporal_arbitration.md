# 17 — Temporal State Machine & Arbitration Specification

**Purpose:** This document mathematically defines the priority rules, race semantics, and exact state transitions for the Temporal Controller. It eliminates ambiguity in how the hardware handles simultaneous events, journal full conditions, and replay boundary logic.

**If this document conflicts with doc 10, this document wins.**

---

## 1. Global Priority Hierarchy

When multiple temporal conditions become true on the exact same clock cycle, the temporal controller evaluates them in the following strictly ranked priority order (1 is highest):

1. **Fatal / Reset:** `rst_ni == 0` or fatal internal error. Immediate transition to IDLE.
2. **Current In-Order Event Service:** A new in-order event is pending in the FIFO and the sequencer is at a preemption boundary.
3. **Journal-Full Protection:** The journal is exactly one slot away from full, and an in-order event is attempting to append.
4. **Commit & Snapshot Arbitration:** Shadow replay has caught up to the snapshot tail.
5. **Late Event Insertion:** A late event has been classified and needs insertion into the journal.
6. **Replay Continuation:** Shadow replay is active and has more events to process.

---

## 2. Exact State Machine Definition

The states described here replace the conceptual diagram in doc 10 with precise RTL logic states.

### Core States
- `ST_IDLE`: Engine enabled, waiting for FIFO valid.
- `ST_CLASSIFY`: Reading FIFO, classifying event (in-order, late, too old, query).
- `ST_IN_ORDER_EXEC`: Executing an in-order event (modifies committed context).
- `ST_LATE_SETUP`: Restoring checkpoint to shadow context.
- `ST_JRNL_INSERT`: Scanning and shifting journal to insert a late event.
- `ST_REPLAY_EXEC`: Executing a journal event on the shadow context.
- `ST_PREEMPT_SAVE`: Pausing replay to save shadow context to TIDE-RAM.
- `ST_PREEMPT_RESTORE`: Restoring shadow context from TIDE-RAM after preemption.

### The Moving Target Problem: Journal Snapshot Rule

**Problem:** In-order events keep arriving and appending to the journal while a late replay is active. If replay just runs until it hits the "current" journal tail, it might never catch up, or it might commit while a new event is being written.

**Atomic Rule (Frozen):**
When a late event transitions to `ST_LATE_SETUP`, the hardware takes a snapshot of the current journal tail pointer: `replay_target_tail = journal_tail`.
The replay loop *only* processes events up to and including `replay_target_tail`.

If new in-order events arrive during replay, they are appended to the journal at the *actual* `journal_tail`, which advances past `replay_target_tail`.

**When replay reaches `replay_target_tail`:**
1. The hardware performs the **Commit Swap** (committed context pointer ↔ shadow context pointer).
2. The hardware checks: does `journal_tail == replay_target_tail`?
   - **YES:** Replay is fully caught up. Transition to `ST_IDLE`.
   - **NO:** New events arrived during replay! The hardware takes a *new* snapshot: `replay_target_tail = journal_tail`. It does *not* transition to IDLE. Instead, it continues replay starting from the new events. (Note: Since the swap just happened, the new events are already in the "new" shadow context, which was just the committed context. This preserves correctness).

---

## 3. Preemption Arbitration (The "Dangerous Corner")

During `ST_REPLAY_EXEC`, the sequencer asserts `seq_boundary_i` between events. On that exact cycle, the hardware evaluates preemption.

**Scenario:** Replay active AND new event pending in FIFO AND journal is almost full.

**Decision Matrix at `seq_boundary_i`:**

| Condition | Action | Next State |
|---|---|---|
| `FIFO_VALID` == 1 | Preempt replay to serve in-order event | `ST_PREEMPT_SAVE` |
| Replay index == `replay_target_tail` | End of snapshot reached | Perform COMMIT, then check if fully caught up |
| `FIFO_VALID` == 0 AND more events | Continue replay | `ST_REPLAY_EXEC` |

**What if `FIFO_VALID` == 1 AND Replay index == `replay_target_tail`?**
Priority Rule 2 (In-Order Service) beats Rule 4 (Commit).
However, doing the commit takes 0 cycles (just swapping pointers).
**Atomic Resolution:** The hardware performs the COMMIT swap *and immediately* transitions to `ST_IN_ORDER_EXEC` to handle the new FIFO event using the newly committed context.

---

## 4. Journal-Full Protection Invariants

**Dangerous Policy:** "Oldest journal entries discarded."
If a late event needs a checkpoint from time T_ckpt, and the journal entries immediately following T_ckpt have been discarded to make room, the replay cannot be reconstructed.

**Invariant 1 (Frozen):**
No journal entry may be overwritten if its timestamp is strictly greater than the timestamp of the *oldest valid checkpoint*.
The hardware enforces this: if `journal_head` reaches the oldest checkpoint's timestamp, the checkpoint ring's `tail` pointer is forcefully advanced, invalidating that checkpoint.

**Invariant 2 (Frozen):**
If the journal is entirely full (head meets tail) AND all entries belong to the most recent checkpoint interval (so no checkpoint can be invalidated), the engine enters **Degraded Mode**:
- `FIFO_OVERRUN` / `JOURNAL_FULL` status bit is set.
- The new in-order event is processed directly against the committed context.
- It is *not* written to the journal.
- Replay across this time window is permanently impossible. The missing event flag is set.

---

## 5. Output Buffer (Seqlock) Race Semantics

The Output Buffer is read by the CPU asynchronously.

**Hardware Write Sequence (Indivisible by hardware):**
1. `OUT_SEQ` increments (becomes odd).
2. `OUT_T`, `OUT_X[0..5]`, `OUT_PDIAG[0..5]` are written simultaneously (within 2-3 cycles).
3. `OUT_SEQ` increments (becomes even).

**Race Condition:** CPU reads while hardware is writing.
- **Resolution:** The CPU software loops while `OUT_SEQ` is odd or if `OUT_SEQ` changes during the read (as defined in Doc 10).
- **Hardware constraint:** The temporal controller will *stall* the write to the output buffer if a CPU APB read transaction is actively addressing the `OUT_*` MMIO region on that exact clock cycle, preventing bus contention or torn APB reads. Wait, stalling hardware on APB reads breaks determinism (Property P1/P4).
- **Corrected Hardware constraint (Frozen):** The hardware **never stalls** for the CPU. The CPU's APB read returns whatever is in the flops on that cycle. It is 100% the CPU software's responsibility to use the Seqlock protocol to detect tearing.

---

## 6. Timestamp Tie-Breaker Rules

If two events have the exact same `t_event`:

1. **In-order ties:** If Event A is processed, and Event B arrives later but has `t_event_B == t_event_A`, Event B is treated as **Late** (because `t_now` has already advanced past `t_event_A`).
2. **Journal Insertion ties:** When inserting a late event with timestamp T into a journal containing existing events with timestamp T, the new late event is inserted **after** the existing events. Arrival order breaks ties.
3. **Checkpoint exact match:** If a late event has `t_event == t_ckpt`, it is inserted *after* the checkpoint. The checkpoint does not need to be restored; the system can just use the checkpoint as the starting point and replay the new event first.

---

## 7. Provisional Data Visibility

When preemption occurs (`ST_PREEMPT_SAVE` → `ST_IN_ORDER_EXEC`), the hardware processes an in-order event while a late replay is suspended.
The resulting state is written to the Output Buffer.

**Rule:** The hardware MUST set the `PROVISIONAL` flag (bit 1 of `OUT_FLAGS`) when updating the output buffer from a preempting in-order event.
This informs the CPU that this output does *not* include the effects of the late event currently being replayed.
When the replay eventually commits, the hardware updates the output buffer again and clears the `PROVISIONAL` flag.
