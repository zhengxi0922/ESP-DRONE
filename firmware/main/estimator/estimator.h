#pragma once

#include <stdbool.h>

#include "esp_drone_types.h"

typedef struct {
    uint64_t timestamp_us;
    vec3f_t gyro_body_xyz_dps;
    vec3f_t acc_body_xyz_g;
    axis3f_t rate_rpy_dps;
    eulerf_t attitude_rpy_deg;
    quatf_t quat_body_to_world;
    bool attitude_valid;
} estimator_state_t;

void estimator_init(void);
void estimator_reset(void);
void estimator_update_from_imu(const imu_sample_t *sample, estimator_state_t *out_state);
axis3f_t estimator_project_rates_from_body_gyro(vec3f_t gyro_body_xyz_dps);

