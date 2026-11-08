/*
 * verify_oosm.c -- checkpoint/replay equivalence proof for out-of-sequence
 * measurements (OOSM), per doc 17_temporal_arbitration.md.
 *
 * Scope: doc 17 specifies the Temporal Controller's checkpoint/journal/replay
 * STATE MACHINE (RTL, a different agent's deliverable).  What Agent 1 (the
 * EKF math owner) must guarantee is the property the whole scheme depends
 * on: replaying tide_unit() over a checkpointed context, in TEMPORAL order,
 * reproduces exactly the state you would get from serial in-order processing
 * -- no matter what order the events actually ARRIVED in.  This program
 * builds a small software model of §2/§4/§6 (checkpoint ring, journal,
 * commit swap, late-event insertion, exact-timestamp shortcut) around the
 * real tide_unit(), and checks that model against ground truth.
 *
 * It is NOT a model of the TC's RTL/FSM (states, cycle counts, seqlock) --
 * that belongs to the TC/sequencer RTL agent.  It IS the reference other
 * agents can use to check the RTL result at a given committed timestamp.
 *
 * Build:  gcc -O2 -std=c99 -ffp-contract=off -o verify_oosm.exe verify_oosm.c tide_core.c -lm
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "tide_core.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint64_t rs = 0xD1B54A32D192ED03ull;
static uint64_t rnd(void) { rs ^= rs >> 12; rs ^= rs << 25; rs ^= rs >> 27; return rs * 2685821657736338717ull; }
static double urand(void) { return (rnd() >> 11) * (1.0 / 9007199254740992.0); }
static double grand(void) { double u = urand() + 1e-300, v = urand(); return sqrt(-2.0 * log(u)) * cos(6.283185307179586 * v); }

static tide_cfg_t g_cfg = { 1.0f, 0.1f, 100.0f };

typedef struct { uint32_t t_event; float z[4], R[4]; } evt_t;

static void meas(const double tx[6], const double sd[4], float z[4])
{
    double r = sqrt(tx[0]*tx[0] + tx[1]*tx[1] + tx[2]*tx[2]);
    double rg = sqrt(tx[0]*tx[0] + tx[1]*tx[1]);
    z[0] = (float)(r + sd[0] * grand());
    z[1] = (float)(atan2(tx[1], tx[0]) + sd[1] * grand());
    z[2] = (float)(atan2(-tx[2], rg) + sd[2] * grand());
    z[3] = (float)((tx[3]*tx[0] + tx[4]*tx[1] + tx[5]*tx[2]) / r + sd[3] * grand());
}

static void make_stream(evt_t *ev, int n, uint32_t t0, uint32_t dt_mean, tide_ctx_t *seed)
{
    double tx[6] = { 1500, -800, -300, -15, 12, 1 }, sd[4] = { 5.0, 0.01, 0.01, 1.0 };
    const float RSET[4] = { 25.0f, 1e-4f, 1e-4f, 1.0f };
    uint32_t t = t0;
    for (int i = 0; i < n; i++) {
        uint32_t d = dt_mean / 2 + (uint32_t)(rnd() % dt_mean);
        for (int k = 0; k < 3; k++) tx[k] += tx[k + 3] * d * 1e-6;
        t += d;
        ev[i].t_event = t;
        meas(tx, sd, ev[i].z);
        memcpy(ev[i].R, RSET, sizeof RSET);
    }
    (void)seed;
}

static tide_ctx_t ref_process(const evt_t *ev, int n, tide_ctx_t init)
{
    tide_ctx_t c = init;
    for (int i = 0; i < n; i++) {
        tide_event_t e; e.t_event = ev[i].t_event; memcpy(e.z, ev[i].z, 16); memcpy(e.R, ev[i].R, 16);
        tide_unit(&c, &e, &g_cfg, NULL);
    }
    return c;
}

/* ---- §2/§4/§6 software model -------------------------------------------- */
/* Capacities per doc 18 sec 1.1: Checkpoint Ring = 8 slots, Journal = 64
 * entries.  (An earlier version of this file used invented values of 16/256
 * with checkpoint-ring capacity as the sole eviction trigger; that is a
 * materially different -- and looser -- policy than doc 17 sec 4 actually
 * specifies, see the eviction logic below.) */
#define CKPT_CAP 8
#define JRNL_CAP 64

typedef struct { int valid; uint32_t t; uint32_t seq; tide_ctx_t ctx; } ckpt_t;

typedef struct {
    tide_ctx_t committed;
    ckpt_t ring[CKPT_CAP]; int ring_head, ring_n;   /* ring_head = oldest slot index */
    evt_t journal[JRNL_CAP]; uint32_t jseq[JRNL_CAP]; int jn;   /* journal, kept sorted by t_event
        (ties: insertion order, doc 17 §6.2); jseq[i] is a monotonic insertion-sequence stamp,
        parallel to journal[i], used to tell "the entry a checkpoint already reflects" apart from
        "a same-timestamp entry inserted after that checkpoint" (doc 17 §6.3) -- timestamp alone
        cannot make that distinction when two entries tie exactly at the checkpoint's own t. */
    uint32_t next_seq;
    int ckpt_interval;                              /* M: checkpoint every M committed in-order events */
    int since_ckpt;
    int gap_n; uint32_t gap_t[JRNL_CAP];             /* timestamps of in-order events dropped under Invariant 2 (never journaled) */
} tc_model_t;

static void tc_init(tc_model_t *m, tide_ctx_t seed, int M)
{
    memset(m, 0, sizeof *m);
    m->committed = seed;
    m->ckpt_interval = M;
    m->ring[0].valid = 1; m->ring[0].t = seed.t_last; m->ring[0].ctx = seed;
    m->ring_n = 1;
}

/* Evict the OLDEST checkpoint (doc 17 sec 4 Invariant 1's "forcefully
 * advance the checkpoint ring's tail" -- a consequence of journal pressure,
 * not an independent capacity policy; see jrnl_make_room). */
static void ckpt_evict_oldest(tc_model_t *m)
{
    if (m->ring_n == 0) return;
    m->ring[m->ring_head].valid = 0;
    m->ring_head = (m->ring_head + 1) % CKPT_CAP;
    m->ring_n--;
}

static int ckpt_oldest_valid_t(tc_model_t *m, uint32_t *out_t)
{
    for (int k = 0; k < m->ring_n; k++) {
        int idx = (m->ring_head + k) % CKPT_CAP;
        if (m->ring[idx].valid) { *out_t = m->ring[idx].t; return 1; }
    }
    return 0;
}

static void ckpt_push(tc_model_t *m, uint32_t t, tide_ctx_t ctx)
{
    if (m->ring_n == CKPT_CAP) ckpt_evict_oldest(m);   /* ring itself is still physically finite */
    int slot = (m->ring_head + m->ring_n) % CKPT_CAP;
    m->ring[slot].valid = 1; m->ring[slot].t = t; m->ring[slot].seq = m->next_seq; m->ring[slot].ctx = ctx;
    m->ring_n++;
}

/* latest checkpoint with t_ckpt <= t_target; -1 if none survives (degraded, doc 17 Invariant 2).
 * Search NEWEST -> OLDEST: several checkpoints can share the same t (repeated
 * late-commits whose replay tail didn't move push a new entry at the same
 * committed t_last each time), and the newest one is the only one guaranteed
 * to reflect every correction known so far at that temporal position -- an
 * older same-t entry is stale.  Iterating newest-first and taking the first
 * t <= t_target match picks the correct (latest arrived, hence most
 * complete) checkpoint in one pass, for both distinct and tied timestamps. */
static int ckpt_find(tc_model_t *m, uint32_t t_target)
{
    for (int k = m->ring_n - 1; k >= 0; k--) {
        int idx = (m->ring_head + k) % CKPT_CAP;
        if (!m->ring[idx].valid) continue;
        if ((int32_t)(t_target - m->ring[idx].t) >= 0) return idx;
    }
    return -1;
}

/* Does any recorded gap (an in-order event dropped under Invariant 2, never
 * journaled) fall strictly between a checkpoint at t_ckpt and a replay
 * target up_to_t?  If so, replaying through that span is missing an event
 * and must not be trusted, even though a checkpoint nominally exists before
 * it -- this is the doc 17 sec 4 "replay across this time window is
 * permanently impossible" case, which ckpt_find's plain existence check
 * alone cannot detect. */
static int gap_crosses(tc_model_t *m, uint32_t t_ckpt, uint32_t up_to_t)
{
    for (int i = 0; i < m->gap_n; i++) {
        uint32_t g = m->gap_t[i];
        if ((int32_t)(g - t_ckpt) > 0 && (int32_t)(g - up_to_t) <= 0) return 1;
    }
    return 0;
}

/* Make room for one more journal entry, per doc 17 sec 4 (Invariants 1 & 2).
 * Journal-capacity pressure is the trigger; checkpoint eviction is a
 * CONSEQUENCE (sacrifice the oldest checkpoint to legitimize discarding
 * older journal entries), not an independent policy.  Returns 0 if room was
 * made, 1 if Invariant 2's degraded condition applies (only the newest
 * checkpoint is left and it still doesn't cover the entry that must go). */
static int jrnl_make_room(tc_model_t *m)
{
    while (m->jn >= JRNL_CAP) {
        uint32_t oldest_t = m->journal[0].t_event;    /* journal kept sorted; [0] is oldest */
        uint32_t ckpt_t;
        if (ckpt_oldest_valid_t(m, &ckpt_t) && (int32_t)(oldest_t - ckpt_t) > 0) {
            if (m->ring_n <= 1) return 1;              /* Invariant 2: can't sacrifice the last checkpoint */
            ckpt_evict_oldest(m);
            continue;                                   /* re-check against the new oldest checkpoint */
        }
        /* oldest_t <= oldest checkpoint's t (or no checkpoint at all): safe to drop, Invariant 1 */
        memmove(&m->journal[0], &m->journal[1], (size_t)(m->jn - 1) * sizeof m->journal[0]);
        memmove(&m->jseq[0], &m->jseq[1], (size_t)(m->jn - 1) * sizeof m->jseq[0]);
        m->jn--;
    }
    return 0;
}

static void jrnl_insert(tc_model_t *m, evt_t e)
{
    /* sorted insert; ties (§6.2) go AFTER existing entries with the same t_event */
    int i = m->jn;
    while (i > 0 && (int32_t)(m->journal[i - 1].t_event - e.t_event) > 0) {
        m->journal[i] = m->journal[i - 1]; m->jseq[i] = m->jseq[i - 1]; i--;
    }
    m->journal[i] = e;
    m->jseq[i] = m->next_seq++;
    m->jn++;
}

/* replay committed <- ckpt, applying every journal entry with t_event in
 * (ckpt.t, target_t] in order; §6.3 exact-match shortcut is just the ckpt.t
 * == e.t_event case of this same rule (no entry to skip, replay starts with
 * the late event itself) so no special-case code is needed for it. */
static tide_ctx_t replay_from(tc_model_t *m, int ckpt_idx, uint32_t up_to_t)
{
    tide_ctx_t c = m->ring[ckpt_idx].ctx;
    uint32_t t_ckpt = m->ring[ckpt_idx].t, seq_ckpt = m->ring[ckpt_idx].seq;
    for (int i = 0; i < m->jn; i++) {
        int32_t dt = (int32_t)(m->journal[i].t_event - t_ckpt);
        /* skip only the entry the checkpoint already reflects: strictly older, OR the exact
         * same timestamp but inserted before/at the checkpoint was taken (doc 17 §6.3 -- a
         * same-timestamp entry inserted AFTER the checkpoint must still be replayed). */
        if (dt < 0 || (dt == 0 && (int32_t)(m->jseq[i] - seq_ckpt) < 0)) continue;
        if ((int32_t)(m->journal[i].t_event - up_to_t) > 0) break;
        tide_event_t e; e.t_event = m->journal[i].t_event;
        memcpy(e.z, m->journal[i].z, 16); memcpy(e.R, m->journal[i].R, 16);
        tide_unit(&c, &e, &g_cfg, NULL);
    }
    return c;
}

/* Any checkpoint strictly newer than a late event's timestamp was built
 * without knowledge of that event, so its cached context is now provably
 * incomplete for its own declared position -- it must not be reused.  The
 * checkpoint pushed right after this late event's commit-swap (at the
 * current replay tail) supersedes it with fully up-to-date content. */
static void ckpt_invalidate_after(tc_model_t *m, uint32_t t_after_exclusive)
{
    for (int k = 0; k < m->ring_n; k++) {
        int idx = (m->ring_head + k) % CKPT_CAP;
        if (m->ring[idx].valid && (int32_t)(m->ring[idx].t - t_after_exclusive) > 0)
            m->ring[idx].valid = 0;
    }
}

/* process one event, arriving in whatever order.
 * Returns: 0 ok; 1 the requested reorder is unrecoverable (no valid
 * checkpoint, or a recorded gap lies in the replay span) -- Invariant 2,
 * "by design" degraded, excluded from correctness accounting by the caller. */
static int tc_feed(tc_model_t *m, evt_t e)
{
    /* doc 17 section 6.1: t_event == committed.t_last is a TIE, and ties are
     * classified LATE ("t_now has already advanced past t_event_A") -- only
     * a STRICTLY newer timestamp is in-order. */
    if ((int32_t)(e.t_event - m->committed.t_last) > 0) {
        tide_event_t ev; ev.t_event = e.t_event; memcpy(ev.z, e.z, 16); memcpy(ev.R, e.R, 16);
        tide_unit(&m->committed, &ev, &g_cfg, NULL);
        if (jrnl_make_room(m)) {
            /* Invariant 2: process against committed (already done above),
             * but do NOT journal it -- record the gap so any later replay
             * spanning this timestamp is correctly refused, not silently
             * wrong. */
            if (m->gap_n < JRNL_CAP) m->gap_t[m->gap_n++] = e.t_event;
        } else {
            jrnl_insert(m, e);
        }
        if (++m->since_ckpt >= m->ckpt_interval) { ckpt_push(m, m->committed.t_last, m->committed); m->since_ckpt = 0; }
        return 0;
    }
    /* late: find a checkpoint at or before e.t_event, insert into the journal, replay to the current tail */
    int idx = ckpt_find(m, e.t_event);
    if (idx < 0) return 1;                                     /* Invariant 2: no valid checkpoint at all */
    uint32_t tail_t_before = m->jn ? m->journal[m->jn - 1].t_event : m->ring[idx].t;
    if (gap_crosses(m, m->ring[idx].t, tail_t_before)) return 1; /* Invariant 2: a gap sits in the replay span */
    if (jrnl_make_room(m)) return 1;                            /* Invariant 2: no room and can't free any */
    jrnl_insert(m, e);
    ckpt_invalidate_after(m, e.t_event);
    uint32_t tail_t = m->jn ? m->journal[m->jn - 1].t_event : m->ring[idx].t;
    tide_ctx_t shadow = replay_from(m, idx, tail_t);
    m->committed = shadow;                                    /* commit swap (section 2) */
    ckpt_push(m, m->committed.t_last, m->committed);
    m->since_ckpt = 0;
    return 0;
}

/* ---- scenario runner ------------------------------------------------------ */
static void shuffle(int *idx, int n)
{
    for (int i = n - 1; i > 0; i--) { int j = (int)(rnd() % (uint32_t)(i + 1)); int t = idx[i]; idx[i] = idx[j]; idx[j] = t; }
}

static int run_scenario(const char *name, evt_t *ev, int n, tide_ctx_t seed, int *arrival, int M, int verbose)
{
    tide_ctx_t want = ref_process(ev, n, seed);
    tc_model_t m; tc_init(&m, seed, M);
    for (int i = 0; i < n; i++) {
        if (tc_feed(&m, ev[arrival[i]])) return 2;   /* ring/journal exhausted: degraded by design, see [V5]/[V6] */
    }
    (void)name;
    int match = memcmp(&m.committed, &want, sizeof want) == 0;
    if (!match && verbose) {
        for (int i = 0; i < 6; i++) if (f2u(m.committed.x[i]) != f2u(want.x[i]))
            printf("    x[%d]: got %08x want %08x\n", i, f2u(m.committed.x[i]), f2u(want.x[i]));
        for (int i = 0; i < 21; i++) if (f2u(m.committed.P[i]) != f2u(want.P[i]))
            printf("    P[%d]: got %08x want %08x\n", i, f2u(m.committed.P[i]), f2u(want.P[i]));
    }
    return match ? 0 : 1;
}

static tide_ctx_t seed_ctx(void)
{
    tide_ctx_t c; memset(&c, 0, sizeof c);
    c.t_last = 1000000;
    float x0[6] = { 1510, -790, -310, -12, 10, 0 };
    memcpy(c.x, x0, sizeof x0);
    for (int i = 0; i < 3; i++) c.P[TIDE_PIDX(i, i)] = 100.0f;
    for (int i = 3; i < 6; i++) c.P[TIDE_PIDX(i, i)] = 25.0f;
    return c;
}

int main(void)
{
    printf("=== TIDE OOSM checkpoint/replay verification (doc 17) ===\n");

    /* [V1] random permutations of arrival order, distinct timestamps */
    {
        printf("[V1] random reorder, %d trials\n", 300);
        int bad = 0, degraded = 0;
        for (int trial = 0; trial < 300; trial++) {
            int n = 5 + (int)(rnd() % 8);            /* small enough that a full shuffle usually stays within CKPT_CAP=8 */
            evt_t ev[64]; tide_ctx_t seed = seed_ctx();
            make_stream(ev, n, seed.t_last, 8000, &seed);
            int arrival[64]; for (int i = 0; i < n; i++) arrival[i] = i;
            shuffle(arrival, n);
            int M = 3 + (int)(rnd() % 6);
            int r = run_scenario("random", ev, n, seed, arrival, M, trial < 3);
            if (r == 1) bad++;
            if (r == 2) degraded++;
        }
        CHECK(bad == 0, "%d/300 random-reorder trials mismatched golden serial processing", bad);
        printf("     %d/300 legitimately degraded (Invariant 2: reorder skew exceeded what the %d-slot ring / %d-entry journal could recover, by design)\n", degraded, CKPT_CAP, JRNL_CAP);
        printf("     %s\n", bad == 0 ? "all remaining trials bit-exact vs serial in-order reference" : "see failures above");
    }
    /* [V2] one very late event (near-start timestamp, arrives dead last) -- deep replay.
     * n is chosen so the in-order-triggered checkpoint count (~(n-1)/M) stays under
     * CKPT_CAP=8 -- otherwise the seed checkpoint event 0 needs would itself have been
     * evicted by the time event 0 finally arrives, which is a legitimate Invariant-2
     * degraded case (see V5/V6), not a deep-replay correctness test. */
    {
        int n = 16; evt_t ev[64]; tide_ctx_t seed = seed_ctx();
        make_stream(ev, n, seed.t_last, 8000, &seed);
        int arrival[64]; for (int i = 1; i < n; i++) arrival[i - 1] = i;
        arrival[n - 1] = 0;                          /* event 0 (earliest) arrives last */
        int r = run_scenario("deep_replay", ev, n, seed, arrival, 4, 1);
        CHECK(r == 0, "deep replay (oldest event arrives last) mismatched reference");
        printf("[V2] deep replay (%d-event replay chain): %s\n", n - 1, r == 0 ? "PASS bit-exact" : "FAIL");
    }
    /* [V3] checkpoint-exact-match shortcut (doc 17 §6.3): late event t_event == an existing t_ckpt */
    {
        int n = 24; evt_t ev[64]; tide_ctx_t seed = seed_ctx();
        make_stream(ev, n, seed.t_last, 10000, &seed);
        int M = 4;
        /* run in-order up to event (M-1) so a checkpoint lands exactly at ev[M-1].t_event, then
         * feed a synthetic late event whose t_event equals that checkpoint's timestamp exactly */
        tc_model_t m; tc_init(&m, seed, M);
        int arrival_prefix[64]; int k = 0;
        for (int i = 0; i < M; i++) arrival_prefix[k++] = i;      /* exactly M events -> one checkpoint at ev[M-1] */
        for (int i = 0; i < k; i++) tc_feed(&m, ev[arrival_prefix[i]]);
        evt_t exact = ev[M - 1];
        exact.t_event = m.ring[(m.ring_head + m.ring_n - 1) % CKPT_CAP].t;   /* == committed t_last: exact match */
        exact.z[0] += 3.0f;                                                    /* a distinct (synthetic) measurement */
        int deg = tc_feed(&m, exact);
        for (int i = M; i < n; i++) tc_feed(&m, ev[i]);
        /* reference: same base stream but with an extra synthetic event inserted immediately
         * after ev[M-1] (identical t_event, arrival-order tiebreak place it after, doc17 6.2/6.3) */
        evt_t ref_ev[65]; int rn = 0;
        for (int i = 0; i < M; i++) ref_ev[rn++] = ev[i];
        ref_ev[rn++] = exact;
        for (int i = M; i < n; i++) ref_ev[rn++] = ev[i];
        tide_ctx_t want = ref_process(ref_ev, rn, seed);
        int match = !deg && memcmp(&m.committed, &want, sizeof want) == 0;
        CHECK(match, "checkpoint-exact-match (t_event == t_ckpt) shortcut mismatched reference");
        printf("[V3] checkpoint-exact-match shortcut: %s\n", match ? "PASS bit-exact" : "FAIL");
    }
    /* [V4] several late events, including late-vs-late reordering (a second late event
     * older than a first late event's insertion, i.e. re-reordering after a commit) */
    {
        printf("[V4] multiple interleaved late events, %d trials\n", 100);
        int bad = 0, degraded = 0;
        for (int trial = 0; trial < 100; trial++) {
            int n = 10 + (int)(rnd() % 10);
            evt_t ev[64]; tide_ctx_t seed = seed_ctx();
            make_stream(ev, n, seed.t_last, 6000, &seed);
            int arrival[64];
            /* build an arrival order with several independent LOCAL rotations (bounded span),
             * to create multiple, possibly-nested late-event episodes rather than one global
             * shuffle -- local rotations keep the reorder skew within real ring/journal
             * capacity far more often than a full-array shuffle would. */
            for (int i = 0; i < n; i++) arrival[i] = i;
            int rounds = 1 + (int)(rnd() % 3);
            for (int r = 0; r < rounds; r++) {
                int a = (int)(rnd() % n), len = 2 + (int)(rnd() % 4);
                if (a + len > n) len = n - a;
                for (int i = 0; i < len / 2; i++) { int t = arrival[a + i]; arrival[a + i] = arrival[a + len - 1 - i]; arrival[a + len - 1 - i] = t; }
            }
            int r = run_scenario("multi_late", ev, n, seed, arrival, 4, trial < 3);
            if (r == 1) bad++;
            if (r == 2) degraded++;
        }
        CHECK(bad == 0, "%d/100 multi-late trials mismatched golden serial processing", bad);
        printf("     %d/100 legitimately degraded (Invariant 2, by design)\n", degraded);
        printf("     %s\n", bad == 0 ? "all remaining trials bit-exact vs serial in-order reference" : "see failures above");
    }
    /* [V5] Invariant 2 sanity: when the reorder skew exceeds what the ring/journal can
     * recover, the model must SAY SO (degraded) rather than silently returning a wrong answer. */
    {
        int n = 20; evt_t ev[64]; tide_ctx_t seed = seed_ctx();
        make_stream(ev, n, seed.t_last, 8000, &seed);
        tc_model_t m; tc_init(&m, seed, 1);              /* checkpoint every event: only CKPT_CAP=8 events of skew survive */
        int arrival[64]; for (int i = 1; i < n; i++) arrival[i - 1] = i;
        arrival[n - 1] = 0;                                /* event 0 arrives after >8 checkpoints have cycled past it */
        int saw_degraded = 0;
        for (int i = 0; i < n; i++) if (tc_feed(&m, ev[arrival[i]])) saw_degraded = 1;
        CHECK(saw_degraded, "expected Invariant-2 degraded detection when skew exceeds ring capacity, got none");
        printf("[V5] checkpoint-ring exhaustion is detected (not silently wrong): %s\n", saw_degraded ? "PASS" : "FAIL");
    }
    /* [V6] Invariant 2 via JOURNAL pressure (not ring pressure): force jrnl_make_room to
     * sacrifice checkpoints down to the last one, then hit an in-order event old journal
     * entries can't make room for -- it must be dropped from the journal (a recorded gap),
     * not silently misprocessed, and a later late event whose replay would cross that gap
     * must be refused even though a checkpoint nominally exists before it. */
    {
        int n = 90; evt_t ev[128]; tide_ctx_t seed = seed_ctx();
        make_stream(ev, n, seed.t_last, 4000, &seed);
        /* M > JRNL_CAP: no in-order checkpoint is pushed before the journal fills, so once
         * it does, only the seed checkpoint survives -- jrnl_make_room's Invariant-2 branch
         * (ring_n <= 1) is the only way forward, not Invariant-1 eviction of a recent one. */
        tc_model_t m; tc_init(&m, seed, JRNL_CAP + 10);
        for (int i = 0; i < n; i++) tc_feed(&m, ev[i]);
        int gap_ok = m.gap_n > 0;                          /* with n=90 > JRNL_CAP=64, at least one drop must have occurred */
        CHECK(gap_ok, "expected jrnl_make_room to record at least one Invariant-2 gap over %d in-order events (journal cap %d)", n, JRNL_CAP);
        int refused = 0;
        if (m.gap_n > 0) {
            evt_t late = ev[0]; late.t_event = m.gap_t[0] > 0 ? m.gap_t[0] - 1 : 0;  /* just before the recorded gap */
            if (late.t_event > 0 && (int32_t)(late.t_event - m.committed.t_last) < 0)
                refused = tc_feed(&m, late) == 1;
        }
        printf("[V6] journal-pressure gap recorded (%d gap(s)) and a replay crossing it is refused: %s\n",
               m.gap_n, (gap_ok && (m.gap_n == 0 || refused)) ? "PASS" : "FAIL (see above)");
        CHECK(m.gap_n == 0 || refused, "a late event whose replay span crosses a recorded gap was NOT refused -- would have silently produced a wrong answer");
    }
    /* [V7] doc 17 sec 6.1 exact-tie boundary: an event with t_event == committed.t_last
     * must be classified LATE, not in-order (the ">= 0" vs "> 0" distinction fixed above). */
    {
        int n = 10; evt_t ev[16]; tide_ctx_t seed = seed_ctx();
        make_stream(ev, n, seed.t_last, 10000, &seed);
        tc_model_t m; tc_init(&m, seed, 3);
        for (int i = 0; i < 4; i++) tc_feed(&m, ev[i]);          /* advance committed.t_last */
        evt_t tie = ev[3];                                        /* exact same t_event as the last committed event */
        tie.z[0] += 1.5f;                                         /* distinct synthetic measurement so a wrong branch is visible */
        int r = tc_feed(&m, tie);
        for (int i = 4; i < n; i++) tc_feed(&m, ev[i]);
        evt_t ref_ev[17]; int rn = 0;
        for (int i = 0; i < 4; i++) ref_ev[rn++] = ev[i];
        ref_ev[rn++] = tie;                                       /* tie inserted after, per sec 6.2 */
        for (int i = 4; i < n; i++) ref_ev[rn++] = ev[i];
        tide_ctx_t want = ref_process(ref_ev, rn, seed);
        int match = !r && memcmp(&m.committed, &want, sizeof want) == 0;
        CHECK(match, "exact-timestamp tie was not classified/processed as LATE per doc 17 sec 6.1");
        printf("[V7] exact-tie (t_event == committed.t_last) classified LATE: %s\n", match ? "PASS bit-exact" : "FAIL");
    }
    printf("%s\n", g_fail ? "TIDE OOSM verification: FAIL" : "TIDE OOSM verification: PASS");
    return g_fail ? 1 : 0;
}
