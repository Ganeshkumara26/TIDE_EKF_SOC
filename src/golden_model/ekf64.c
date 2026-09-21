#include "ekf64.h"
#include <string.h>
#include <math.h>
int g64_simplified=0,g64_fix_alias=0;
#ifndef SIMULATION_BUILD
#include "arm_math.h"
#else
typedef struct { uint32_t numRows; uint32_t numCols; double *pData; } arm64_matrix_instance_f64;
static void arm64_mat_init_f64(arm64_matrix_instance_f64 *S, uint32_t nr, uint32_t nc, double *p) {
    S->numRows = nr; S->numCols = nc; S->pData = p;
}
static int arm64_mat_mult_f64(const arm64_matrix_instance_f64 *A, const arm64_matrix_instance_f64 *B, arm64_matrix_instance_f64 *C) {
    for (uint32_t i = 0; i < A->numRows; i++)
        for (uint32_t j = 0; j < B->numCols; j++) {
            double s = 0.0f;
            for (uint32_t k = 0; k < A->numCols; k++)
                s += A->pData[i*A->numCols+k] * B->pData[k*B->numCols+j];
            C->pData[i*C->numCols+j] = s;
        }
    return 0;
}
static int arm64_mat_inverse_f64(const arm64_matrix_instance_f64 *S, arm64_matrix_instance_f64 *D) {
    uint32_t n = S->numRows;
    double tmp[72]; /* 6x12 augmented */
    for (uint32_t i = 0; i < n; i++) {
        for (uint32_t j = 0; j < n; j++) {
            tmp[i*(2*n)+j] = S->pData[i*n+j];
            tmp[i*(2*n)+j+n] = (i==j) ? 1.0f : 0.0f;
        }
    }
    for (uint32_t i = 0; i < n; i++) {
        double piv = tmp[i*(2*n)+i];
        if (fabs(piv) < 1e-12f) return -1;
        for (uint32_t j = 0; j < 2*n; j++) tmp[i*(2*n)+j] /= piv;
        for (uint32_t k = 0; k < n; k++) {
            if (k == i) continue;
            double f = tmp[k*(2*n)+i];
            for (uint32_t j = 0; j < 2*n; j++) tmp[k*(2*n)+j] -= f*tmp[i*(2*n)+j];
        }
    }
    for (uint32_t i = 0; i < n; i++)
        for (uint32_t j = 0; j < n; j++)
            D->pData[i*n+j] = tmp[i*(2*n)+j+n];
    return 0;
}
static int arm64_mat_add_f64(const arm64_matrix_instance_f64 *A, const arm64_matrix_instance_f64 *B, arm64_matrix_instance_f64 *C) {
    for (uint32_t i = 0; i < A->numRows*A->numCols; i++) C->pData[i] = A->pData[i] + B->pData[i];
    return 0;
}
static int arm64_mat_trans_f64(const arm64_matrix_instance_f64 *S, arm64_matrix_instance_f64 *D) {
    for (uint32_t i = 0; i < S->numRows; i++)
        for (uint32_t j = 0; j < S->numCols; j++)
            D->pData[j*D->numCols+i] = S->pData[i*S->numCols+j];
    return 0;
}
#endif
static double scratch_F[EKF64_STATE_DIM * EKF64_STATE_DIM];
static double scratch_FT[EKF64_STATE_DIM * EKF64_STATE_DIM];
static double scratch_FP[EKF64_STATE_DIM * EKF64_STATE_DIM];
static double scratch_FPFt[EKF64_STATE_DIM * EKF64_STATE_DIM];
static double scratch_Pnew[EKF64_STATE_DIM * EKF64_STATE_DIM];
static double scratch_H[EKF64_MEAS_DIM * EKF64_STATE_DIM];
static double scratch_HT[EKF64_STATE_DIM * EKF64_MEAS_DIM];
static double scratch_HP[EKF64_MEAS_DIM * EKF64_STATE_DIM];
static double scratch_S[EKF64_MEAS_DIM * EKF64_MEAS_DIM];
static double scratch_Sinv[EKF64_MEAS_DIM * EKF64_MEAS_DIM];
static double scratch_K[EKF64_STATE_DIM * EKF64_MEAS_DIM];
static double scratch_PHT[EKF64_STATE_DIM * EKF64_MEAS_DIM];
static double scratch_KH[EKF64_STATE_DIM * EKF64_STATE_DIM];
static double scratch_I_KH[EKF64_STATE_DIM * EKF64_STATE_DIM];

void ekf64_init(ekf64_state_t *ekf, const ekf64_config_t *config) {
    uint8_t *p = (uint8_t *)ekf;
    for (uint32_t i = 0; i < sizeof(ekf64_state_t); i++) p[i] = 0;
    ekf->process_noise = config->process_noise;
    ekf->meas_noise_range = config->meas_noise_range;
    ekf->meas_noise_angle = config->meas_noise_angle;
    ekf->meas_noise_doppler = config->meas_noise_doppler;
    ekf->innovation_gate = 9.0f;
    ekf->initialized = 0;
    ekf->meas_noise_range = config->meas_noise_range;
    ekf->meas_noise_angle = config->meas_noise_angle;
    ekf->meas_noise_doppler = config->meas_noise_doppler;
    ekf->innovation_gate = 9.0f;
    ekf->initialized = 0;
    for (int i = 0; i < EKF64_STATE_DIM * EKF64_STATE_DIM; i++) ekf->Q[i] = 0.0f;
    ekf->Q[0] = ekf->Q[7] = ekf->Q[14] = config->process_noise;
    ekf->Q[21] = ekf->Q[28] = ekf->Q[35] = config->process_noise * 0.1f;
    for (int i = 0; i < EKF64_MEAS_DIM * EKF64_MEAS_DIM; i++) ekf->R[i] = 0.0f;
    ekf->R[0] = config->meas_noise_range;
    ekf->R[5] = config->meas_noise_angle;
    ekf->R[10] = config->meas_noise_angle;
    ekf->R[15] = config->meas_noise_doppler;
    for (int i = 0; i < EKF64_STATE_DIM * EKF64_STATE_DIM; i++) ekf->P[i] = 0.0f;
    ekf->P[0] = ekf->P[7] = ekf->P[14] = 100.0f;
    ekf->P[21] = ekf->P[28] = ekf->P[35] = 25.0f;
}

void ekf64_predict(ekf64_state_t *ekf, double dt) {
    ekf->x[0] += ekf->x[3] * dt;
    ekf->x[1] += ekf->x[4] * dt;
    ekf->x[2] += ekf->x[5] * dt;
    memset(scratch_F, 0, sizeof(scratch_F));
    scratch_F[0] = scratch_F[7] = scratch_F[14] = 1.0f;
    scratch_F[21] = scratch_F[28] = scratch_F[35] = 1.0f;
    scratch_F[3] = scratch_F[10] = scratch_F[17] = dt;
    arm64_matrix_instance_f64 matF, matFT, matP, matFP, matFPFt, matQ, matPnew;
    arm64_mat_init_f64(&matF, 6, 6, scratch_F);
    arm64_mat_init_f64(&matP, 6, 6, ekf->P);
    arm64_mat_init_f64(&matFT, 6, 6, scratch_FT);
    arm64_mat_trans_f64(&matF, &matFT);
    arm64_mat_init_f64(&matFP, 6, 6, scratch_FP);
    arm64_mat_mult_f64(&matF, &matP, &matFP);
    arm64_mat_init_f64(&matFPFt, 6, 6, scratch_FPFt);
    arm64_mat_mult_f64(&matFP, &matFT, &matFPFt);
    arm64_mat_init_f64(&matQ, 6, 6, ekf->Q);
    arm64_mat_init_f64(&matPnew, 6, 6, scratch_Pnew);
    arm64_mat_add_f64(&matFPFt, &matQ, &matPnew);
    memcpy(ekf->P, scratch_Pnew, sizeof(scratch_Pnew));
    ekf->predict_count++;
}

int ekf64_update_radar(ekf64_state_t *ekf, double range, double azimuth, double elevation, double doppler, double *innovation_out) {
    double pn = ekf->x[0], pe = ekf->x[1], pd = ekf->x[2];
    double vn = ekf->x[3], ve = ekf->x[4], vd = ekf->x[5];
    double r2 = pn*pn + pe*pe + pd*pd;
    if (r2 < 1e-6f) r2 = 1e-6f;
    double r = sqrt(r2);
    double range_pred = r;
    double rg2_raw = pn*pn + pe*pe;
    if (rg2_raw < 1e-6f) rg2_raw = 1e-6f;
    double r_ground = sqrt(rg2_raw);
    double azimuth_pred = atan2(pe, pn);
    double elevation_pred = atan2(-pd, r_ground);
    double doppler_pred = (vn*pn + ve*pe + vd*pd) / r;
    double y[EKF64_MEAS_DIM];
    y[0] = range - range_pred;
    y[1] = azimuth - azimuth_pred;
    y[2] = elevation - elevation_pred;
    y[3] = doppler - doppler_pred;
    memset(scratch_H, 0, sizeof(scratch_H));
    scratch_H[0] = pn / r;
    scratch_H[1] = pe / r;
    scratch_H[2] = pd / r;
    scratch_H[3] = 0.0f; scratch_H[4] = 0.0f; scratch_H[5] = 0.0f;
    double rg2 = r_ground * r_ground;
    scratch_H[6]  = -pe / rg2;
    scratch_H[7]  =  pn / rg2;
    scratch_H[8]  = 0.0f; scratch_H[9]  = 0.0f; scratch_H[10] = 0.0f; scratch_H[11] = 0.0f;
    double r2rg = r2 * r_ground;
    scratch_H[12] = (pd * pn) / r2rg;
    scratch_H[13] = (pd * pe) / r2rg;
    scratch_H[14] = -r_ground / r2;
    scratch_H[15] = 0.0f; scratch_H[16] = 0.0f; scratch_H[17] = 0.0f;
    double dot_vp = vn*pn + ve*pe + vd*pd;
    scratch_H[18] = (vn - doppler_pred * pn / r) / r;
    scratch_H[19] = (ve - doppler_pred * pe / r) / r;
    scratch_H[20] = (vd - doppler_pred * pd / r) / r;
    scratch_H[21] = pn / r;
    scratch_H[22] = pe / r;
    scratch_H[23] = pd / r;
    (void)dot_vp;
    arm64_matrix_instance_f64 matH, matP, matHT, matHP, matHPHT, matR, matS;
    arm64_mat_init_f64(&matH, 4, 6, scratch_H);
    arm64_mat_init_f64(&matP, 6, 6, ekf->P);
    arm64_mat_init_f64(&matHT, 6, 4, scratch_HT);
    arm64_mat_trans_f64(&matH, &matHT);
    arm64_mat_init_f64(&matHP, 4, 6, scratch_HP);
    arm64_mat_mult_f64(&matH, &matP, &matHP);
    arm64_mat_init_f64(&matHPHT, 4, 4, scratch_S);
    arm64_mat_mult_f64(&matHP, &matHT, &matHPHT);
    arm64_mat_init_f64(&matR, 4, 4, ekf->R);
    arm64_mat_init_f64(&matS, 4, 4, scratch_S);
    arm64_mat_add_f64(&matHPHT, &matR, &matS);
    double s_gate = scratch_S[0] + scratch_S[5] + scratch_S[10] + scratch_S[15];
    if (s_gate > ekf->innovation_gate * 100.0f) {
        ekf->reject_count++;
        return -1;
    }
    arm64_matrix_instance_f64 matSinv, matK;
    arm64_mat_init_f64(&matSinv, 4, 4, scratch_Sinv);
    if (arm64_mat_inverse_f64(&matS, &matSinv) != 0) {
        ekf->reject_count++;
        return -2;
    }
    arm64_matrix_instance_f64 matPHT;
    arm64_mat_init_f64(&matPHT, 6, 4, g64_fix_alias ? scratch_PHT : scratch_K);
    arm64_mat_mult_f64(&matP, &matHT, &matPHT);
    arm64_mat_init_f64(&matK, 6, 4, scratch_K);
    arm64_mat_mult_f64(&matPHT, &matSinv, &matK);
    for (int i = 0; i < EKF64_STATE_DIM; i++) {
        double ky = 0.0f;
        for (int j = 0; j < EKF64_MEAS_DIM; j++)
            ky += scratch_K[i*EKF64_MEAS_DIM+j] * y[j];
        ekf->x[i] += ky;
    }
    arm64_matrix_instance_f64 matKH;
    arm64_mat_init_f64(&matKH, 6, 6, scratch_KH);
    arm64_mat_mult_f64(&matK, &matH, &matKH);
    for (uint32_t i = 0; i < 36; i++) scratch_I_KH[i] = -scratch_KH[i];
    for (int i = 0; i < EKF64_STATE_DIM; i++) scratch_I_KH[i*6+i] += 1.0f;
    static double scratch_IKH_T[EKF64_STATE_DIM * EKF64_STATE_DIM];
    static double scratch_IKHP[EKF64_STATE_DIM * EKF64_STATE_DIM];
    static double scratch_IKHPIKHt[EKF64_STATE_DIM * EKF64_STATE_DIM];
    static double scratch_KR[EKF64_STATE_DIM * EKF64_MEAS_DIM];
    static double scratch_KRKt[EKF64_STATE_DIM * EKF64_STATE_DIM];
    arm64_matrix_instance_f64 matIKH, matIKHT, matIKHP, matIKHPIKHt;
    arm64_mat_init_f64(&matIKH, 6, 6, scratch_I_KH);
    arm64_mat_init_f64(&matIKHT, 6, 6, scratch_IKH_T);
    arm64_mat_trans_f64(&matIKH, &matIKHT);
    arm64_mat_init_f64(&matIKHP, 6, 6, scratch_IKHP);
    arm64_mat_mult_f64(&matIKH, &matP, &matIKHP);
    if (g64_simplified) { memcpy(scratch_Pnew, scratch_IKHP, sizeof(scratch_Pnew)); goto cov_done; }
    arm64_mat_init_f64(&matIKHPIKHt, 6, 6, scratch_IKHPIKHt);
    arm64_mat_mult_f64(&matIKHP, &matIKHT, &matIKHPIKHt);
    arm64_matrix_instance_f64 matR2, matKR, matKt, matKRKt;
    arm64_mat_init_f64(&matR2, 4, 4, ekf->R);
    arm64_mat_init_f64(&matKR, 6, 4, scratch_KR);
    arm64_mat_mult_f64(&matK, &matR2, &matKR);
    arm64_mat_init_f64(&matKt, 4, 6, scratch_HT);
    arm64_mat_trans_f64(&matK, &matKt);
    arm64_mat_init_f64(&matKRKt, 6, 6, scratch_KRKt);
    arm64_mat_mult_f64(&matKR, &matKt, &matKRKt);
    arm64_matrix_instance_f64 matPnew;
    arm64_mat_init_f64(&matPnew, 6, 6, scratch_Pnew);
    arm64_mat_add_f64(&matIKHPIKHt, &matKRKt, &matPnew);
    cov_done:;
    memcpy(ekf->P, scratch_Pnew, sizeof(scratch_Pnew));
    for (int i = 0; i < EKF64_STATE_DIM; i++)
        for (int j = i+1; j < EKF64_STATE_DIM; j++) {
            double avg = 0.5f * (ekf->P[i*EKF64_STATE_DIM+j] + ekf->P[j*EKF64_STATE_DIM+i]);
            ekf->P[i*EKF64_STATE_DIM+j] = avg;
            ekf->P[j*EKF64_STATE_DIM+i] = avg;
        }
    ekf->update_count++;
    if (innovation_out) { innovation_out[0]=y[0]; innovation_out[1]=y[1]; innovation_out[2]=y[2]; innovation_out[3]=y[3]; }
    return 0;
}
void ekf64_get_position(const ekf64_state_t *ekf, double *pn, double *pe, double *pd) { *pn = ekf->x[0]; *pe = ekf->x[1]; *pd = ekf->x[2]; }
void ekf64_get_velocity(const ekf64_state_t *ekf, double *vn, double *ve, double *vd) { *vn = ekf->x[3]; *ve = ekf->x[4]; *vd = ekf->x[5]; }
