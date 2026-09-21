/* RECONSTRUCTED header: inferred from field usage in the user's ekf.c */
#ifndef EKF_H
#define EKF_H
#include <stdint.h>
#define EKF_STATE_DIM 6
#define EKF_MEAS_DIM  4
typedef struct { float process_noise, meas_noise_range, meas_noise_angle, meas_noise_doppler; } ekf_config_t;
typedef struct {
  float x[EKF_STATE_DIM];
  float P[EKF_STATE_DIM*EKF_STATE_DIM];
  float Q[EKF_STATE_DIM*EKF_STATE_DIM];
  float R[EKF_MEAS_DIM*EKF_MEAS_DIM];
  float process_noise, meas_noise_range, meas_noise_angle, meas_noise_doppler, innovation_gate;
  uint32_t initialized, predict_count, update_count, reject_count;
} ekf_state_t;
void ekf_init(ekf_state_t*, const ekf_config_t*);
void ekf_predict(ekf_state_t*, float dt);
int  ekf_update_radar(ekf_state_t*, float range, float az, float el, float doppler, float *innov_out);
void ekf_get_position(const ekf_state_t*, float*, float*, float*);
void ekf_get_velocity(const ekf_state_t*, float*, float*, float*);
#endif
