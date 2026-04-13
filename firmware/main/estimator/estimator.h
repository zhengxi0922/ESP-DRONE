#pragma once

#include <stdbool.h>

#include "esp_drone_types.h"

typedef struct {
    uint64_t timestamp_us;

    /* Backward-compatible raw aliases used by existing rate/hang/UDP paths. */
    vec3f_t gyro_body_xyz_dps;
    vec3f_t acc_body_xyz_g;
    axis3f_t rate_rpy_dps;
    eulerf_t attitude_rpy_deg;
    quatf_t quat_body_to_world;
    bool attitude_valid;

    vec3f_t raw_gyro_body_xyz_dps;
    vec3f_t filtered_gyro_body_xyz_dps;
    vec3f_t raw_acc_body_xyz_g;
    vec3f_t filtered_acc_body_xyz_g;
    axis3f_t raw_rate_rpy_dps;
    axis3f_t filtered_rate_rpy_dps;
    eulerf_t raw_attitude_rpy_deg;
    quatf_t raw_quat_body_to_world;
    float kalman_roll_deg;
    float kalman_pitch_deg;
    bool kalman_valid;
} estimator_state_t;

void estimator_init(void);
void estimator_reset(void);
void estimator_update_from_imu(const imu_sample_t *sample, estimator_state_t *out_state);
bool estimator_get_latest(estimator_state_t *out_state);

axis3f_t estimator_project_rates_from_body_gyro(vec3f_t gyro_body_xyz_dps);
