#include "estimator.h"

#include <string.h>

axis3f_t estimator_project_rates_from_body_gyro(vec3f_t gyro_body_xyz_dps)
{
    return (axis3f_t){
        .roll = -gyro_body_xyz_dps.y,
        .pitch = gyro_body_xyz_dps.x,
        .yaw = -gyro_body_xyz_dps.z,
    };
}

void estimator_init(void)
{
}

void estimator_reset(void)
{
}

void estimator_update_from_imu(const imu_sample_t *sample, estimator_state_t *out_state)
{
    if (sample == NULL || out_state == NULL) {
        return;
    }

    memset(out_state, 0, sizeof(*out_state));
    out_state->timestamp_us = sample->timestamp_us;
    out_state->gyro_body_xyz_dps = sample->gyro_xyz_dps;
    out_state->acc_body_xyz_g = sample->acc_xyz_g;
    out_state->rate_rpy_dps = estimator_project_rates_from_body_gyro(sample->gyro_xyz_dps);
    out_state->attitude_rpy_deg = sample->roll_pitch_yaw_deg;
    out_state->quat_body_to_world = sample->quat_wxyz;
    out_state->attitude_valid = sample->has_attitude;
}
