#include "estimator.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include "params.h"

#define ESTIMATOR_DEG_PER_RAD 57.29577951308232f
#define ESTIMATOR_TWO_PI 6.283185307179586f

typedef struct {
    float angle_deg;
    float bias_dps;
    float p00;
    float p01;
    float p10;
    float p11;
    bool initialized;
} estimator_kalman_axis_t;

typedef struct {
    estimator_state_t latest;
    uint64_t last_timestamp_us;
    bool filters_initialized;
    estimator_kalman_axis_t kalman_roll;
    estimator_kalman_axis_t kalman_pitch;
} estimator_internal_state_t;

static portMUX_TYPE s_estimator_lock = portMUX_INITIALIZER_UNLOCKED;
static estimator_internal_state_t s_state;

static float estimator_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float estimator_lpf_alpha(float cutoff_hz, float dt_s)
{
    if (cutoff_hz <= 0.0f || dt_s <= 0.0f || !isfinite(cutoff_hz) || !isfinite(dt_s)) {
        return 1.0f;
    }
    const float rc = 1.0f / (ESTIMATOR_TWO_PI * cutoff_hz);
    return estimator_clampf(dt_s / (rc + dt_s), 0.0f, 1.0f);
}

static float estimator_lpf(float prev, float raw, float alpha)
{
    return prev + alpha * (raw - prev);
}

static vec3f_t estimator_lpf_vec3(vec3f_t prev, vec3f_t raw, float alpha)
{
    return (vec3f_t){
        .x = estimator_lpf(prev.x, raw.x, alpha),
        .y = estimator_lpf(prev.y, raw.y, alpha),
        .z = estimator_lpf(prev.z, raw.z, alpha),
    };
}

static axis3f_t estimator_lpf_axis3(axis3f_t prev, axis3f_t raw, float alpha)
{
    return (axis3f_t){
        .roll = estimator_lpf(prev.roll, raw.roll, alpha),
        .pitch = estimator_lpf(prev.pitch, raw.pitch, alpha),
        .yaw = estimator_lpf(prev.yaw, raw.yaw, alpha),
    };
}

static bool estimator_acc_tilt_from_filtered_acc(vec3f_t acc, float *out_roll_deg, float *out_pitch_deg)
{
    const float norm = sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    if (!isfinite(norm) || norm < 0.60f || norm > 1.40f || acc.z < 0.20f) {
        return false;
    }

    if (out_roll_deg != NULL) {
        *out_roll_deg = atan2f(acc.x, acc.z) * ESTIMATOR_DEG_PER_RAD;
    }
    if (out_pitch_deg != NULL) {
        *out_pitch_deg = atan2f(acc.y, acc.z) * ESTIMATOR_DEG_PER_RAD;
    }
    return true;
}

static float estimator_kalman_update(estimator_kalman_axis_t *axis,
                                     float measured_angle_deg,
                                     float measured_rate_dps,
                                     float dt_s,
                                     const params_store_t *params)
{
    if (!axis->initialized) {
        axis->angle_deg = measured_angle_deg;
        axis->bias_dps = 0.0f;
        axis->p00 = 1.0f;
        axis->p01 = 0.0f;
        axis->p10 = 0.0f;
        axis->p11 = 1.0f;
        axis->initialized = true;
        return axis->angle_deg;
    }

    const float rate_unbiased = measured_rate_dps - axis->bias_dps;
    axis->angle_deg += dt_s * rate_unbiased;

    axis->p00 += dt_s * (dt_s * axis->p11 - axis->p01 - axis->p10 + params->kalman_q_angle);
    axis->p01 -= dt_s * axis->p11;
    axis->p10 -= dt_s * axis->p11;
    axis->p11 += params->kalman_q_bias * dt_s;

    const float innovation = measured_angle_deg - axis->angle_deg;
    const float s = axis->p00 + params->kalman_r_measure;
    if (!isfinite(s) || s <= 1e-9f) {
        axis->initialized = false;
        return measured_angle_deg;
    }

    const float k0 = axis->p00 / s;
    const float k1 = axis->p10 / s;
    axis->angle_deg += k0 * innovation;
    axis->bias_dps += k1 * innovation;

    const float p00 = axis->p00;
    const float p01 = axis->p01;
    axis->p00 -= k0 * p00;
    axis->p01 -= k0 * p01;
    axis->p10 -= k1 * p00;
    axis->p11 -= k1 * p01;
    return axis->angle_deg;
}

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
    estimator_reset();
}

void estimator_reset(void)
{
    taskENTER_CRITICAL(&s_estimator_lock);
    memset(&s_state, 0, sizeof(s_state));
    taskEXIT_CRITICAL(&s_estimator_lock);
}

void estimator_update_from_imu(const imu_sample_t *sample, estimator_state_t *out_state)
{
    if (sample == NULL || out_state == NULL) {
        return;
    }

    const params_store_t *params = params_get();
    estimator_state_t next = {0};
    estimator_kalman_axis_t kalman_roll = {0};
    estimator_kalman_axis_t kalman_pitch = {0};
    bool filters_initialized = false;
    uint64_t last_timestamp_us = 0;

    taskENTER_CRITICAL(&s_estimator_lock);
    next = s_state.latest;
    kalman_roll = s_state.kalman_roll;
    kalman_pitch = s_state.kalman_pitch;
    filters_initialized = s_state.filters_initialized;
    last_timestamp_us = s_state.last_timestamp_us;
    taskEXIT_CRITICAL(&s_estimator_lock);

    float dt_s = 0.001f;
    if (last_timestamp_us != 0u && sample->timestamp_us > last_timestamp_us) {
        dt_s = (float)(sample->timestamp_us - last_timestamp_us) / 1000000.0f;
        if (dt_s < 0.001f) {
            dt_s = 0.001f;
        } else if (dt_s > 0.050f) {
            dt_s = 0.050f;
            kalman_roll.initialized = false;
            kalman_pitch.initialized = false;
        }
    }

    next.timestamp_us = sample->timestamp_us;
    next.raw_gyro_body_xyz_dps = sample->gyro_xyz_dps;
    next.raw_acc_body_xyz_g = sample->acc_xyz_g;
    next.raw_rate_rpy_dps = estimator_project_rates_from_body_gyro(sample->gyro_xyz_dps);
    next.raw_attitude_rpy_deg = sample->roll_pitch_yaw_deg;
    next.raw_quat_body_to_world = sample->quat_wxyz;
    next.attitude_valid = sample->has_attitude && sample->has_quaternion;

    if (!filters_initialized) {
        next.filtered_gyro_body_xyz_dps = next.raw_gyro_body_xyz_dps;
        next.filtered_acc_body_xyz_g = next.raw_acc_body_xyz_g;
        next.filtered_rate_rpy_dps = next.raw_rate_rpy_dps;
        filters_initialized = true;
    } else {
        const float gyro_alpha = estimator_lpf_alpha(params->gyro_lpf_hz, dt_s);
        const float accel_alpha = estimator_lpf_alpha(params->accel_lpf_hz, dt_s);
        const float rate_alpha = estimator_lpf_alpha(params->rate_lpf_hz, dt_s);
        next.filtered_gyro_body_xyz_dps =
            estimator_lpf_vec3(next.filtered_gyro_body_xyz_dps, next.raw_gyro_body_xyz_dps, gyro_alpha);
        next.filtered_acc_body_xyz_g =
            estimator_lpf_vec3(next.filtered_acc_body_xyz_g, next.raw_acc_body_xyz_g, accel_alpha);
        next.filtered_rate_rpy_dps =
            estimator_lpf_axis3(next.filtered_rate_rpy_dps, next.raw_rate_rpy_dps, rate_alpha);
    }

    float accel_roll_deg = 0.0f;
    float accel_pitch_deg = 0.0f;
    const bool accel_tilt_valid =
        estimator_acc_tilt_from_filtered_acc(next.filtered_acc_body_xyz_g, &accel_roll_deg, &accel_pitch_deg);
    next.kalman_valid = false;
    if (params->kalman_enable && accel_tilt_valid) {
        next.kalman_roll_deg = estimator_kalman_update(
            &kalman_roll, accel_roll_deg, next.filtered_rate_rpy_dps.roll, dt_s, params);
        next.kalman_pitch_deg = estimator_kalman_update(
            &kalman_pitch, accel_pitch_deg, next.filtered_rate_rpy_dps.pitch, dt_s, params);
        next.kalman_valid = true;
    } else {
        kalman_roll.initialized = false;
        kalman_pitch.initialized = false;
    }

    next.gyro_body_xyz_dps = next.raw_gyro_body_xyz_dps;
    next.acc_body_xyz_g = next.raw_acc_body_xyz_g;
    next.rate_rpy_dps = next.raw_rate_rpy_dps;
    next.attitude_rpy_deg = next.raw_attitude_rpy_deg;
    next.quat_body_to_world = next.raw_quat_body_to_world;

    taskENTER_CRITICAL(&s_estimator_lock);
    s_state.latest = next;
    s_state.kalman_roll = kalman_roll;
    s_state.kalman_pitch = kalman_pitch;
    s_state.filters_initialized = filters_initialized;
    s_state.last_timestamp_us = sample->timestamp_us;
    taskEXIT_CRITICAL(&s_estimator_lock);

    *out_state = next;
}

bool estimator_get_latest(estimator_state_t *out_state)
{
    if (out_state == NULL) {
        return false;
    }

    taskENTER_CRITICAL(&s_estimator_lock);
    *out_state = s_state.latest;
    const bool valid = s_state.latest.timestamp_us != 0u;
    taskEXIT_CRITICAL(&s_estimator_lock);
    return valid;
}
