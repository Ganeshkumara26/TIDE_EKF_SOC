/*
 * ekf.h — float32 Extended Kalman Filter for radar-inertial fusion
 * Host-buildable port of the internship EKF (original target: STM32F070RB, Cortex-M0 @ 48 MHz,
 * CMSIS-DSP f32). The EKF_Fixed_SCALE macro below is a leftover name; the filter itself is float32.
 *
 * 6-state EKF: position (3) + velocity (3) in NED frame.
 * Fuses MAVLink attitude/velocity with radar range-Doppler measurements.
 * Uses CMSIS-DSP matrix operations for deterministic timing.
 *
 * State vector: [pn, pe, pd, vn, ve, vd]^T
 * Measurement:  [range, azimuth, elevation, doppler] from radar
 */
#ifndef EKF_H
#define EKF_H

#include <stdint.h>

#define EKF_STATE_DIM    6
#define EKF_MEAS_DIM     4
#define EKF_Fixed_SCALE  1000  /* Q15.16 fixed-point scale factor */

/* EKF configuration */
typedef struct {
    float dt;              /* Prediction time step (seconds) */
    float process_noise;   /* Process noise covariance scalar */
    float meas_noise_range;   /* Range measurement noise (m^2) */
    float meas_noise_angle;   /* Angle measurement noise (rad^2) */
    float meas_noise_doppler; /* Doppler measurement noise (m/s)^2 */
} ekf_config_t;

/* EKF state */
typedef struct {
    /* State vector x[6] = [pn, pe, pd, vn, ve, vd] */
    float x[EKF_STATE_DIM];

    /* Covariance matrix P[6x6] */
    float P[EKF_STATE_DIM * EKF_STATE_DIM];

    /* Process noise Q[6x6] */
    float Q[EKF_STATE_DIM * EKF_STATE_DIM];

    /* Measurement noise R[4x4] */
    float R[EKF_MEAS_DIM * EKF_MEAS_DIM];

    /* Noise parameters (stored for runtime inspection) */
    float process_noise;
    float meas_noise_range;
    float meas_noise_angle;
    float meas_noise_doppler;

    /* Innovation gate threshold (Mahalanobis distance) */
    float innovation_gate;

    /* Status */
    uint32_t predict_count;
    uint32_t update_count;
    uint32_t reject_count;
    uint8_t  initialized;
} ekf_state_t;

void ekf_init(ekf_state_t *ekf, const ekf_config_t *config);
void ekf_predict(ekf_state_t *ekf, float dt);
int  ekf_update_radar(ekf_state_t *ekf,
                      float range, float azimuth, float elevation,
                      float doppler, float *innovation_out);
void ekf_get_position(const ekf_state_t *ekf, float *pn, float *pe, float *pd);
void ekf_get_velocity(const ekf_state_t *ekf, float *vn, float *ve, float *vd);

#endif /* EKF_H */
