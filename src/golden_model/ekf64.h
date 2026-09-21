/* RECONSTRUCTED header: inferred from field usage in the user's ekf.c */
#ifndef EKF64_H
#define EKF64_H
#include <stdint.h>
#define EKF64_STATE_DIM 6
#define EKF64_MEAS_DIM  4
typedef struct { double process_noise, meas_noise_range, meas_noise_angle, meas_noise_doppler; } ekf64_config_t;
typedef struct {
  double x[EKF64_STATE_DIM];
  double P[EKF64_STATE_DIM*EKF64_STATE_DIM];
  double Q[EKF64_STATE_DIM*EKF64_STATE_DIM];
  double R[EKF64_MEAS_DIM*EKF64_MEAS_DIM];
  double process_noise, meas_noise_range, meas_noise_angle, meas_noise_doppler, innovation_gate;
  uint32_t initialized, predict_count, update_count, reject_count;
} ekf64_state_t;
void ekf64_init(ekf64_state_t*, const ekf64_config_t*);
void ekf64_predict(ekf64_state_t*, double dt);
int  ekf64_update_radar(ekf64_state_t*, double range, double az, double el, double doppler, double *innov_out);
void ekf64_get_position(const ekf64_state_t*, double*, double*, double*);
void ekf64_get_velocity(const ekf64_state_t*, double*, double*, double*);
#endif
