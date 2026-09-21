#include "ekf.h"
#include <string.h>
#include <math.h>
int g_simplified=0,g_fix_alias=0;
#ifndef SIMULATION_BUILD
#include "arm_math.h"
#else
typedef struct { uint32_t numRows; uint32_t numCols; float *pData; } arm_matrix_instance_f32;
static void arm_mat_init_f32(arm_matrix_instance_f32 *S, uint32_t nr, uint32_t nc, float *p) {
    S->numRows = nr; S->numCols = nc; S->pData = p;
}
static int arm_mat_mult_f32(const arm_matrix_instance_f32 *A, const arm_matrix_instance_f32 *B, arm_matrix_instance_f32 *C) {
    for (uint32_t i = 0; i < A->numRows; i++)
        for (uint32_t j = 0; j < B->numCols; j++) {
            float s = 0.0f;
            for (uint32_t k = 0; k < A->numCols; k++)
                s += A->pData[i*A->numCols+k] * B->pData[k*B->numCols+j];
            C->pData[i*C->numCols+j] = s;
        }
    return 0;
}
static int arm_mat_inverse_f32(const arm_matrix_instance_f32 *S, arm_matrix_instance_f32 *D) {
    uint32_t n = S->numRows;
    float tmp[72]; /* 6x12 augmented */
    for (uint32_t i = 0; i < n; i++) {
        for (uint32_t j = 0; j < n; j++) {
            tmp[i*(2*n)+j] = S->pData[i*n+j];
            tmp[i*(2*n)+j+n] = (i==j) ? 1.0f : 0.0f;
        }
    }
    for (uint32_t i = 0; i < n; i++) {
        float piv = tmp[i*(2*n)+i];
        if (fabsf(piv) < 1e-12f) return -1;
        for (uint32_t j = 0; j < 2*n; j++) tmp[i*(2*n)+j] /= piv;
        for (uint32_t k = 0; k < n; k++) {
            if (k == i) continue;
            float f = tmp[k*(2*n)+i];
            for (uint32_t j = 0; j < 2*n; j++) tmp[k*(2*n)+j] -= f*tmp[i*(2*n)+j];
        }
    }
    for (uint32_t i = 0; i < n; i++)
        for (uint32_t j = 0; j < n; j++)
            D->pData[i*n+j] = tmp[i*(2*n)+j+n];
    return 0;
}
static int arm_mat_add_f32(const arm_matrix_instance_f32 *A, const arm_matrix_instance_f32 *B, arm_matrix_instance_f32 *C) {
    for (uint32_t i = 0; i < A->numRows*A->numCols; i++) C->pData[i] = A->pData[i] + B->pData[i];
    return 0;
}
static int arm_mat_trans_f32(const arm_matrix_instance_f32 *S, arm_matrix_instance_f32 *D) {
    for (uint32_t i = 0; i < S->numRows; i++)
        for (uint32_t j = 0; j < S->numCols; j++)
            D->pData[j*D->numCols+i] = S->pData[i*S->numCols+j];
    return 0;
}
#endif
static float scratch_F[EKF_STATE_DIM * EKF_STATE_DIM];
static float scratch_FT[EKF_STATE_DIM * EKF_STATE_DIM];
static float scratch_FP[EKF_STATE_DIM * EKF_STATE_DIM];
static float scratch_FPFt[EKF_STATE_DIM * EKF_STATE_DIM];
static float scratch_Pnew[EKF_STATE_DIM * EKF_STATE_DIM];
static float scratch_H[EKF_MEAS_DIM * EKF_STATE_DIM];
static float scratch_HT[EKF_STATE_DIM * EKF_MEAS_DIM];
static float scratch_HP[EKF_MEAS_DIM * EKF_STATE_DIM];
static float scratch_S[EKF_MEAS_DIM * EKF_MEAS_DIM];
static float scratch_Sinv[EKF_MEAS_DIM * EKF_MEAS_DIM];
static float scratch_K[EKF_STATE_DIM * EKF_MEAS_DIM];
static float scratch_PHT[EKF_STATE_DIM * EKF_MEAS_DIM];
static float scratch_KH[EKF_STATE_DIM * EKF_STATE_DIM];
static float scratch_I_KH[EKF_STATE_DIM * EKF_STATE_DIM];

void ekf_init(ekf_state_t *ekf, const ekf_config_t *config) {
    uint8_t *p = (uint8_t *)ekf;
    for (uint32_t i = 0; i < sizeof(ekf_state_t); i++) p[i] = 0;
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
    for (int i = 0; i < EKF_STATE_DIM * EKF_STATE_DIM; i++) ekf->Q[i] = 0.0f;
    ekf->Q[0] = ekf->Q[7] = ekf->Q[14] = config->process_noise;
    ekf->Q[21] = ekf->Q[28] = ekf->Q[35] = config->process_noise * 0.1f;
    for (int i = 0; i < EKF_MEAS_DIM * EKF_MEAS_DIM; i++) ekf->R[i] = 0.0f;
    ekf->R[0] = config->meas_noise_range;
    ekf->R[5] = config->meas_noise_angle;
    ekf->R[10] = config->meas_noise_angle;
    ekf->R[15] = config->meas_noise_doppler;
    for (int i = 0; i < EKF_STATE_DIM * EKF_STATE_DIM; i++) ekf->P[i] = 0.0f;
    ekf->P[0] = ekf->P[7] = ekf->P[14] = 100.0f;
    ekf->P[21] = ekf->P[28] = ekf->P[35] = 25.0f;
}

void ekf_predict(ekf_state_t *ekf, float dt) {
    ekf->x[0] += ekf->x[3] * dt;
    ekf->x[1] += ekf->x[4] * dt;
    ekf->x[2] += ekf->x[5] * dt;
    memset(scratch_F, 0, sizeof(scratch_F));
    scratch_F[0] = scratch_F[7] = scratch_F[14] = 1.0f;
    scratch_F[21] = scratch_F[28] = scratch_F[35] = 1.0f;
    scratch_F[3] = scratch_F[10] = scratch_F[17] = dt;
    arm_matrix_instance_f32 matF, matFT, matP, matFP, matFPFt, matQ, matPnew;
    arm_mat_init_f32(&matF, 6, 6, scratch_F);
    arm_mat_init_f32(&matP, 6, 6, ekf->P);
    arm_mat_init_f32(&matFT, 6, 6, scratch_FT);
    arm_mat_trans_f32(&matF, &matFT);
    arm_mat_init_f32(&matFP, 6, 6, scratch_FP);
    arm_mat_mult_f32(&matF, &matP, &matFP);
    arm_mat_init_f32(&matFPFt, 6, 6, scratch_FPFt);
    arm_mat_mult_f32(&matFP, &matFT, &matFPFt);
    arm_mat_init_f32(&matQ, 6, 6, ekf->Q);
    arm_mat_init_f32(&matPnew, 6, 6, scratch_Pnew);
    arm_mat_add_f32(&matFPFt, &matQ, &matPnew);
    memcpy(ekf->P, scratch_Pnew, sizeof(scratch_Pnew));
    ekf->predict_count++;
}

int ekf_update_radar(ekf_state_t *ekf, float range, float azimuth, float elevation, float doppler, float *innovation_out) {
    float pn = ekf->x[0], pe = ekf->x[1], pd = ekf->x[2];
    float vn = ekf->x[3], ve = ekf->x[4], vd = ekf->x[5];
    float r2 = pn*pn + pe*pe + pd*pd;
    if (r2 < 1e-6f) r2 = 1e-6f;
    float r = sqrtf(r2);
    float range_pred = r;
    float rg2_raw = pn*pn + pe*pe;
    if (rg2_raw < 1e-6f) rg2_raw = 1e-6f;
    float r_ground = sqrtf(rg2_raw);
    float azimuth_pred = atan2f(pe, pn);
    float elevation_pred = atan2f(-pd, r_ground);
    float doppler_pred = (vn*pn + ve*pe + vd*pd) / r;
    float y[EKF_MEAS_DIM];
    y[0] = range - range_pred;
    y[1] = azimuth - azimuth_pred;
    y[2] = elevation - elevation_pred;
    y[3] = doppler - doppler_pred;
    memset(scratch_H, 0, sizeof(scratch_H));
    scratch_H[0] = pn / r;
    scratch_H[1] = pe / r;
    scratch_H[2] = pd / r;
    scratch_H[3] = 0.0f; scratch_H[4] = 0.0f; scratch_H[5] = 0.0f;
    float rg2 = r_ground * r_ground;
    scratch_H[6]  = -pe / rg2;
    scratch_H[7]  =  pn / rg2;
    scratch_H[8]  = 0.0f; scratch_H[9]  = 0.0f; scratch_H[10] = 0.0f; scratch_H[11] = 0.0f;
    float r2rg = r2 * r_ground;
    scratch_H[12] = (pd * pn) / r2rg;
    scratch_H[13] = (pd * pe) / r2rg;
    scratch_H[14] = -r_ground / r2;
    scratch_H[15] = 0.0f; scratch_H[16] = 0.0f; scratch_H[17] = 0.0f;
    float dot_vp = vn*pn + ve*pe + vd*pd;
    scratch_H[18] = (vn - doppler_pred * pn / r) / r;
    scratch_H[19] = (ve - doppler_pred * pe / r) / r;
    scratch_H[20] = (vd - doppler_pred * pd / r) / r;
    scratch_H[21] = pn / r;
    scratch_H[22] = pe / r;
    scratch_H[23] = pd / r;
    (void)dot_vp;
    arm_matrix_instance_f32 matH, matP, matHT, matHP, matHPHT, matR, matS;
    arm_mat_init_f32(&matH, 4, 6, scratch_H);
    arm_mat_init_f32(&matP, 6, 6, ekf->P);
    arm_mat_init_f32(&matHT, 6, 4, scratch_HT);
    arm_mat_trans_f32(&matH, &matHT);
    arm_mat_init_f32(&matHP, 4, 6, scratch_HP);
    arm_mat_mult_f32(&matH, &matP, &matHP);
    arm_mat_init_f32(&matHPHT, 4, 4, scratch_S);
    arm_mat_mult_f32(&matHP, &matHT, &matHPHT);
    arm_mat_init_f32(&matR, 4, 4, ekf->R);
    arm_mat_init_f32(&matS, 4, 4, scratch_S);
    arm_mat_add_f32(&matHPHT, &matR, &matS);
    float s_gate = scratch_S[0] + scratch_S[5] + scratch_S[10] + scratch_S[15];
    if (s_gate > ekf->innovation_gate * 100.0f) {
        ekf->reject_count++;
        return -1;
    }
    arm_matrix_instance_f32 matSinv, matK;
    arm_mat_init_f32(&matSinv, 4, 4, scratch_Sinv);
    if (arm_mat_inverse_f32(&matS, &matSinv) != 0) {
        ekf->reject_count++;
        return -2;
    }
    arm_matrix_instance_f32 matPHT;
    arm_mat_init_f32(&matPHT, 6, 4, g_fix_alias ? scratch_PHT : scratch_K);
    arm_mat_mult_f32(&matP, &matHT, &matPHT);
    arm_mat_init_f32(&matK, 6, 4, scratch_K);
    arm_mat_mult_f32(&matPHT, &matSinv, &matK);
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        float ky = 0.0f;
        for (int j = 0; j < EKF_MEAS_DIM; j++)
            ky += scratch_K[i*EKF_MEAS_DIM+j] * y[j];
        ekf->x[i] += ky;
    }
    arm_matrix_instance_f32 matKH;
    arm_mat_init_f32(&matKH, 6, 6, scratch_KH);
    arm_mat_mult_f32(&matK, &matH, &matKH);
    for (uint32_t i = 0; i < 36; i++) scratch_I_KH[i] = -scratch_KH[i];
    for (int i = 0; i < EKF_STATE_DIM; i++) scratch_I_KH[i*6+i] += 1.0f;
    static float scratch_IKH_T[EKF_STATE_DIM * EKF_STATE_DIM];
    static float scratch_IKHP[EKF_STATE_DIM * EKF_STATE_DIM];
    static float scratch_IKHPIKHt[EKF_STATE_DIM * EKF_STATE_DIM];
    static float scratch_KR[EKF_STATE_DIM * EKF_MEAS_DIM];
    static float scratch_KRKt[EKF_STATE_DIM * EKF_STATE_DIM];
    arm_matrix_instance_f32 matIKH, matIKHT, matIKHP, matIKHPIKHt;
    arm_mat_init_f32(&matIKH, 6, 6, scratch_I_KH);
    arm_mat_init_f32(&matIKHT, 6, 6, scratch_IKH_T);
    arm_mat_trans_f32(&matIKH, &matIKHT);
    arm_mat_init_f32(&matIKHP, 6, 6, scratch_IKHP);
    arm_mat_mult_f32(&matIKH, &matP, &matIKHP);
    if (g_simplified) { memcpy(scratch_Pnew, scratch_IKHP, sizeof(scratch_Pnew)); goto cov_done; }
    arm_mat_init_f32(&matIKHPIKHt, 6, 6, scratch_IKHPIKHt);
    arm_mat_mult_f32(&matIKHP, &matIKHT, &matIKHPIKHt);
    arm_matrix_instance_f32 matR2, matKR, matKt, matKRKt;
    arm_mat_init_f32(&matR2, 4, 4, ekf->R);
    arm_mat_init_f32(&matKR, 6, 4, scratch_KR);
    arm_mat_mult_f32(&matK, &matR2, &matKR);
    arm_mat_init_f32(&matKt, 4, 6, scratch_HT);
    arm_mat_trans_f32(&matK, &matKt);
    arm_mat_init_f32(&matKRKt, 6, 6, scratch_KRKt);
    arm_mat_mult_f32(&matKR, &matKt, &matKRKt);
    arm_matrix_instance_f32 matPnew;
    arm_mat_init_f32(&matPnew, 6, 6, scratch_Pnew);
    arm_mat_add_f32(&matIKHPIKHt, &matKRKt, &matPnew);
    cov_done:;
    memcpy(ekf->P, scratch_Pnew, sizeof(scratch_Pnew));
    for (int i = 0; i < EKF_STATE_DIM; i++)
        for (int j = i+1; j < EKF_STATE_DIM; j++) {
            float avg = 0.5f * (ekf->P[i*EKF_STATE_DIM+j] + ekf->P[j*EKF_STATE_DIM+i]);
            ekf->P[i*EKF_STATE_DIM+j] = avg;
            ekf->P[j*EKF_STATE_DIM+i] = avg;
        }
    ekf->update_count++;
    if (innovation_out) { innovation_out[0]=y[0]; innovation_out[1]=y[1]; innovation_out[2]=y[2]; innovation_out[3]=y[3]; }
    return 0;
}
void ekf_get_position(const ekf_state_t *ekf, float *pn, float *pe, float *pd) { *pn = ekf->x[0]; *pe = ekf->x[1]; *pd = ekf->x[2]; }
void ekf_get_velocity(const ekf_state_t *ekf, float *vn, float *ve, float *vd) { *vn = ekf->x[3]; *ve = ekf->x[4]; *vd = ekf->x[5]; }
