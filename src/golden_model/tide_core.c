/*
 * tide_core.c -- TIDE-EKF golden model.  See tide_core.h.
 * Operation order here is NORMATIVE for microcode/gen_ucode.py.
 */
#include <math.h>
#include "tide_core.h"

/* ---------------------------------------------------------------------- */
/* Final constant ROM.  Entries 0..10, 15..19 are the memmap.yaml contract.
 * Entries 11..14 are PATCHED (A-ROM): the contract values (0x3F7FFFF5 ...)
 * are a t*(c0+c1 z+..) fit valid only for t < ~0.3 (max err 1.6e-5 rad with
 * the tan(pi/8) reduction) and do not meet doc 19's 2.8e-7 bound.  They are
 * replaced by doc 19's frozen Cephes atanf coefficients in Horner order.  */
const uint32_t TIDE_ROM[64] = {
    /* 0 */ 0x40490FDB, 0x3FC90FDB, 0x40C90FDB, 0xC0490FDB, 0x3F800000,
    /* 5 */ 0x00000000, 0xBF800000, 0x358637BD, 0x3ED413CD, 0x401A827A,
    /*10 */ 0x3F490FDB,
    /*11 */ 0x3DA4F0D1,  /*  8.05374449538e-2  (Horner coeff 0) */
    /*12 */ 0xBE0E1B85,  /* -1.38776856032e-1                   */
    /*13 */ 0x3E4C925F,  /*  1.99777106478e-1                   */
    /*14 */ 0xBEAAAA2A,  /* -3.33329491539e-1                   */
    /*15 */ 0x3727C5AC, 0x322BCC77, 0x3F000000, 0x40000000, 0xC0000000
    /* 20..63 = 0 */
};

float hw_div(float a, float b) { return hw_canon(a / b); }
float hw_sqrt(float a)         { return hw_canon(sqrtf(a)); }

/* ---------------------------------------------------------------------- */
/* atan2                                                                    */
/* ---------------------------------------------------------------------- */

/* shared reduced-argument polynomial: returns (p + t2) + a0, doc 19 sec 2.2 */
static float atan_poly(float t2, float a0)
{
    float z = hw_mul(t2, t2);
    float p = rom(ROM_ATAN_H0);
    p = hw_add(hw_mul(p, z), rom(ROM_ATAN_H1));
    p = hw_add(hw_mul(p, z), rom(ROM_ATAN_H2));
    p = hw_add(hw_mul(p, z), rom(ROM_ATAN_H3));
    p = hw_mul(hw_mul(p, z), t2);
    return hw_add(hw_add(p, t2), a0);
}

/* Branch-free range reduction: s = [t > tan(pi/8)];
 *   t2 = (t - s) / (1 + s*t)   (s=0: t/1 = t exactly; s=1: (t-1)/(t+1))
 *   a0 = s * pi/4              (exact)                                     */
static float atan_reduce_eval(float t)
{
    float s   = hw_lt(rom(ROM_TAN_PI_8), t);
    float num = hw_sub(t, s);
    float den = hw_add(rom(ROM_ONE), hw_mul(s, t));
    float t2  = hw_div(num, den);
    float a0  = hw_mul(s, rom(ROM_PI_4));
    return atan_poly(t2, a0);
}

/* octant select stage: out = c + m*a, c = s*K, m = 1 - 2s  (bit-exact to
 * "if (s) a = K - a" because s in {0,1}: 0+a = a, K + (-1*a) = K - a)     */
static float oct_stage(float s, float K, float a)
{
    float c = hw_mul(s, K);
    float m = hw_sub(rom(ROM_ONE), hw_add(s, s));
    return hw_add(c, hw_mul(m, a));
}

float tide_atan2_full(float y, float x)
{
    float ax = hw_fabs(x), ay = hw_fabs(y);
    float mx = hw_fmax(ax, ay), mn = hw_fmin(ax, ay);
    /* atan2(0,0) = +0 rule: mx==0 -> divide 0/1 instead of 0/0 */
    float nz  = hw_lt(rom(ROM_ZERO), mx);
    float e   = hw_sub(rom(ROM_ONE), nz);
    float mxg = hw_add(mx, e);
    float t   = hw_div(mn, mxg);
    float a   = atan_reduce_eval(t);
    float s1  = hw_lt(ax, ay);                       /* ay > ax */
    a = oct_stage(s1, rom(ROM_PI_2), a);
    float s2  = hw_lt(x, rom(ROM_ZERO));             /* x < 0   */
    a = oct_stage(s2, rom(ROM_PI), a);
    float s3  = hw_lt(y, rom(ROM_ZERO));             /* y < 0   */
    float m3  = hw_sub(rom(ROM_ONE), hw_add(s3, s3));
    return hw_mul(m3, a);
}

float tide_atan2_xpos(float y, float x)
{
    float ax = hw_fabs(x), ay = hw_fabs(y);   /* ax == x for x > 0 */
    float mx = hw_fmax(ax, ay), mn = hw_fmin(ax, ay);
    float t  = hw_div(mn, mx);
    float a  = atan_reduce_eval(t);
    float s1 = hw_lt(ax, ay);
    a = oct_stage(s1, rom(ROM_PI_2), a);
    float s3 = hw_lt(y, rom(ROM_ZERO));
    float m3 = hw_sub(rom(ROM_ONE), hw_add(s3, s3));
    return hw_mul(m3, a);
}

/* Branchy reference straight from doc 19 (tests only) */
float tide_atan2_ref(float y, float x)
{
    float ax = hw_fabs(x), ay = hw_fabs(y);
    float mx = hw_fmax(ax, ay), mn = hw_fmin(ax, ay);
    if (mx == 0.0f) return 0.0f;
    float t = hw_div(mn, mx), a0 = 0.0f;
    if (t > rom(ROM_TAN_PI_8)) {
        float num = hw_sub(t, 1.0f), den = hw_add(t, 1.0f);
        t = hw_div(num, den);
        a0 = rom(ROM_PI_4);
    }
    float a = atan_poly(t, a0);
    if (ay > ax) a = hw_sub(rom(ROM_PI_2), a);
    if (x < 0.0f) a = hw_sub(rom(ROM_PI), a);
    if (y < 0.0f) a = hw_mul(-1.0f, a);       /* == -a except NaN payload */
    return a;
}

/* ---------------------------------------------------------------------- */
/* PREDICT (doc 19 sec 3, block form)                                       */
/* ---------------------------------------------------------------------- */
#define PP(i, j)  P[TIDE_PIDX(i, j)]
/* symmetric access into the OLD covariance */
#define PS(i, j)  ((i) >= (j) ? P[TIDE_PIDX(i, j)] : P[TIDE_PIDX(j, i)])

void tide_predict(tide_ctx_t *c, int32_t dt_us, const tide_cfg_t *cfg)
{
    float dt  = hw_mul(hw_itof(dt_us), rom(ROM_US_TO_S));       /* ITOF, FMUL 1e-6 */
    float sc  = hw_mul(dt, cfg->dt0inv);                        /* dt/dt0 (A-SC)   */
    float qp  = hw_mul(cfg->qp, sc);
    float qv  = hw_mul(cfg->qv, sc);
    float dt2 = hw_mul(dt, dt);
    float *x = c->x;
    const float *P = c->P;           /* old values; new values go to N */
    float N[21];

    for (int i = 0; i < 3; i++)                                  /* state */
        x[i] = hw_add(x[i], hw_mul(x[i + 3], dt));

    /* dt * Pvv (6 unique products, lower triangle of the velocity block) */
    float dPvv[3][3];
    for (int a = 0; a < 3; a++)
        for (int b = 0; b <= a; b++)
            dPvv[a][b] = hw_mul(dt, PP(3 + a, 3 + b));

    /* position block: Ppp + dt*(Pvp[a][b]+Pvp[b][a]) + dt^2*Pvv (+Qp diag) */
    for (int a = 0; a < 3; a++)
        for (int b = 0; b <= a; b++) {
            float s  = hw_add(PP(3 + a, b), PP(3 + b, a));      /* P10 + P01 */
            float t1 = hw_mul(dt, s);
            float t2 = hw_mul(dt2, PP(3 + a, 3 + b));
            float n  = hw_add(hw_add(PP(a, b), t1), t2);
            if (a == b) n = hw_add(n, qp);
            N[TIDE_PIDX(a, b)] = n;
        }
    /* cross block: Pvp + dt*Pvv (Pvv symmetric) */
    for (int a = 0; a < 3; a++)
        for (int b = 0; b < 3; b++) {
            float d = (a >= b) ? dPvv[a][b] : dPvv[b][a];
            N[TIDE_PIDX(3 + a, b)] = hw_add(PP(3 + a, b), d);
        }
    /* velocity block: Pvv + Qv on the diagonal only */
    for (int a = 0; a < 3; a++)
        for (int b = 0; b <= a; b++)
            N[TIDE_PIDX(3 + a, 3 + b)] =
                (a == b) ? hw_add(PP(3 + a, 3 + a), qv) : PP(3 + a, 3 + b);

    memcpy(c->P, N, sizeof N);
}

/* ---------------------------------------------------------------------- */
/* GEOM: h(x_prior), Jacobian rows, corrected-later innovations y0          */
/* ---------------------------------------------------------------------- */
void tide_geom(const float x[6], const float z[4], const float R[4], tide_rows_t *out)
{
    float pn = x[0], pe = x[1], pd = x[2], vn = x[3], ve = x[4], vd = x[5];
    float ONE = rom(ROM_ONE);

    float pn2 = hw_mul(pn, pn), pe2 = hw_mul(pe, pe), pd2 = hw_mul(pd, pd);
    float rg2r = hw_add(pn2, pe2);
    float r2   = hw_add(rg2r, pd2);
    float r2c  = hw_fmax(r2,   rom(ROM_US_TO_S));     /* guard 1e-6 (pilot) */
    float rg2c = hw_fmax(rg2r, rom(ROM_US_TO_S));
    float r    = hw_sqrt(r2c);
    float rg   = hw_sqrt(rg2c);

    float rrg     = hw_mul(r, rg);
    float inv_rrg = hw_div(ONE, rrg);                 /* single DIV -> 1/r, 1/rg */
    float inv_r   = hw_mul(rg, inv_rrg);
    float inv_rg  = hw_mul(r, inv_rrg);

    float dv  = hw_add(hw_add(hw_mul(vn, pn), hw_mul(ve, pe)), hw_mul(vd, pd));
    float dop = hw_mul(dv, inv_r);

    float az = tide_atan2_full(pe, pn);
    float el = tide_atan2_xpos(hw_fneg(pd), rg);      /* NED: elevation up = -pd */

    float y0[4];
    y0[0] = hw_sub(z[0], r);
    float y1 = hw_sub(z[1], az);                      /* azimuth wrap to [-pi,pi] */
    float shi = hw_lt(rom(ROM_PI), y1);
    float slo = hw_lt(y1, rom(ROM_NEG_PI));
    y1 = hw_add(hw_sub(y1, hw_mul(shi, rom(ROM_TWO_PI))), hw_mul(slo, rom(ROM_TWO_PI)));
    y0[1] = y1;
    y0[2] = hw_sub(z[2], el);
    y0[3] = hw_sub(z[3], dop);

    float H[4][6];
    memset(H, 0, sizeof H);
    H[0][0] = hw_mul(pn, inv_r);
    H[0][1] = hw_mul(pe, inv_r);
    H[0][2] = hw_mul(pd, inv_r);

    float inv_rg2 = hw_mul(inv_rg, inv_rg);
    H[1][0] = hw_fneg(hw_mul(pe, inv_rg2));
    H[1][1] = hw_mul(pn, inv_rg2);

    float inv_r2   = hw_mul(inv_r, inv_r);
    float inv_r2rg = hw_mul(inv_r2, inv_rg);
    float pdpn = hw_mul(pd, pn), pdpe = hw_mul(pd, pe);
    H[2][0] = hw_mul(pdpn, inv_r2rg);
    H[2][1] = hw_mul(pdpe, inv_r2rg);
    H[2][2] = hw_fneg(hw_mul(rg, inv_r2));

    float vel[3] = { vn, ve, vd };
    for (int i = 0; i < 3; i++) {
        H[3][i]     = hw_mul(hw_sub(vel[i], hw_mul(dop, H[0][i])), inv_r);
        H[3][3 + i] = H[0][i];
    }
    for (int k = 0; k < 4; k++) {
        for (int j = 0; j < 6; j++) out->row[k][j] = H[k][j];
        out->row[k][6] = R[k];
        out->row[k][7] = y0[k];
    }
}

/* ---------------------------------------------------------------------- */
/* One sequential scalar update (Joseph form, rank-1 structured evaluation) */
/*   A = I - K h^T ;  P+ = A P A^T + R K K^T                                */
/*   u = A P = P - K p^T  (p = P h);  (u A^T)_ij = u_ij - c_i K_j,          */
/*   c = u h = p - K (h.p);  P+_ij = (u_ij - c_i K_j) + (R K_i) K_j         */
/* ---------------------------------------------------------------------- */
void tide_scalar(tide_ctx_t *c, const float row[8], const float xp[6])
{
    const float *h = row;
    float R = row[6], y0 = row[7];
    float *x = c->x;
    const float *P = c->P;

    /* innovation correction: y = y0 - h . (x_cur - x_prior)  (doc 19 sec 4.2) */
    float d[6], dot = 0.0f;
    for (int j = 0; j < 6; j++) d[j] = hw_sub(x[j], xp[j]);
    dot = hw_add(hw_mul(h[0], d[0]), hw_mul(h[1], d[1]));
    for (int j = 2; j < 6; j++) dot = hw_add(dot, hw_mul(h[j], d[j]));
    float y = hw_sub(y0, dot);

    /* p = P h */
    float p[6];
    for (int i = 0; i < 6; i++) {
        float acc = hw_add(hw_mul(PS(i, 0), h[0]), hw_mul(PS(i, 1), h[1]));
        for (int j = 2; j < 6; j++) acc = hw_add(acc, hw_mul(PS(i, j), h[j]));
        p[i] = acc;
    }
    /* hp = h . p ; s = hp + R */
    float hp = hw_add(hw_mul(h[0], p[0]), hw_mul(h[1], p[1]));
    for (int i = 2; i < 6; i++) hp = hw_add(hp, hw_mul(h[i], p[i]));
    float s   = hw_add(hp, R);
    float inv = hw_div(rom(ROM_ONE), s);

    float K[6];
    for (int i = 0; i < 6; i++) K[i] = hw_mul(p[i], inv);
    for (int i = 0; i < 6; i++) x[i] = hw_add(x[i], hw_mul(K[i], y));

    float cc[6], rk[6];
    for (int i = 0; i < 6; i++) {
        cc[i] = hw_sub(p[i], hw_mul(K[i], hp));
        rk[i] = hw_mul(R, K[i]);
    }
    float N[21];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j <= i; j++) {
            float u  = hw_sub(P[TIDE_PIDX(i, j)], hw_mul(K[i], p[j]));
            float w  = hw_sub(u, hw_mul(K[j], cc[i]));
            N[TIDE_PIDX(i, j)] = hw_add(w, hw_mul(rk[i], K[j]));
        }
    memcpy(c->P, N, sizeof N);
}

/* ---------------------------------------------------------------------- */
void tide_unit(tide_ctx_t *c, const tide_event_t *e, const tide_cfg_t *cfg, tide_dbg_t *dbg)
{
    int32_t dt_us = (int32_t)(e->t_event - c->t_last);   /* TC computes (Step 0) */
    tide_predict(c, dt_us, cfg);
    float xp[6];
    memcpy(xp, c->x, sizeof xp);
    tide_rows_t rows;
    tide_geom(c->x, e->z, e->R, &rows);
    if (dbg) { dbg->pre = *c; dbg->rows = rows; }
    for (int k = 0; k < 4; k++) tide_scalar(c, rows.row[k], xp);
    c->t_last = e->t_event;
}

void tide_query(const tide_ctx_t *c, uint32_t t_q, const tide_cfg_t *cfg, uint32_t out[7])
{
    (void)cfg;
    int32_t dt_us = (int32_t)(t_q - c->t_last);
    float dt = hw_mul(hw_itof(dt_us), rom(ROM_US_TO_S));
    out[0] = t_q;
    for (int i = 0; i < 3; i++) out[1 + i] = f2u(hw_add(c->x[i], hw_mul(c->x[i + 3], dt)));
    for (int i = 3; i < 6; i++) out[1 + i] = f2u(c->x[i]);
}
