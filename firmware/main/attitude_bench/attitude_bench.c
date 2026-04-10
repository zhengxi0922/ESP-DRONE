#include "attitude_bench.h"

#include <math.h>
#include <string.h>

#include "params.h"
#include "runtime_state.h"

#define ATTITUDE_BENCH_DEG_PER_RAD (57.29577951308232f)

static float attitude_bench_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static bool attitude_bench_quat_is_valid(quatf_t q)
{
    return isfinite(q.w) && isfinite(q.x) && isfinite(q.y) && isfinite(q.z);
}

static quatf_t attitude_bench_quat_normalize(quatf_t q, bool *out_valid)
{
    quatf_t normalized = {0};
    const float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

    if (out_valid != NULL) {
        *out_valid = false;
    }
    if (!attitude_bench_quat_is_valid(q) || norm <= 1e-6f || !isfinite(norm)) {
        return normalized;
    }

    normalized.w = q.w / norm;
    normalized.x = q.x / norm;
    normalized.y = q.y / norm;
    normalized.z = q.z / norm;
    if (out_valid != NULL) {
        *out_valid = true;
    }
    return normalized;
}

static quatf_t attitude_bench_quat_conjugate(quatf_t q)
{
    return (quatf_t){
        .w = q.w,
        .x = -q.x,
        .y = -q.y,
        .z = -q.z,
    };
}

static quatf_t attitude_bench_quat_multiply(quatf_t a, quatf_t b)
{
    return (quatf_t){
        .w = (a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z),
        .x = (a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y),
        .y = (a.w * b.y) - (a.x * b.z) + (a.y * b.w) + (a.z * b.x),
        .z = (a.w * b.z) + (a.x * b.y) - (a.y * b.x) + (a.z * b.w),
    };
}

static vec3f_t attitude_bench_quat_to_rotvec_rad(quatf_t q_rel)
{
    const float vec_norm = sqrtf(q_rel.x * q_rel.x + q_rel.y * q_rel.y + q_rel.z * q_rel.z);
    if (vec_norm <= 1e-6f) {
        return (vec3f_t){
            .x = 2.0f * q_rel.x,
            .y = 2.0f * q_rel.y,
            .z = 2.0f * q_rel.z,
        };
    }

    const float clamped_w = attitude_bench_clampf(q_rel.w, -1.0f, 1.0f);
    const float angle = 2.0f * atan2f(vec_norm, clamped_w);
    const float scale = angle / vec_norm;
    return (vec3f_t){
        .x = q_rel.x * scale,
        .y = q_rel.y * scale,
        .z = q_rel.z * scale,
    };
}

static float attitude_bench_apply_deadband(float value_deg, float deadband_deg)
{
    return (fabsf(value_deg) <= deadband_deg) ? 0.0f : value_deg;
}

bool attitude_bench_capture_reference(const imu_sample_t *sample)
{
    bool valid = false;
    quatf_t normalized = {0};

    if (sample == NULL || sample->health != IMU_HEALTH_OK || !sample->has_quaternion) {
        return false;
    }

    normalized = attitude_bench_quat_normalize(sample->quat_wxyz, &valid);
    if (!valid) {
        return false;
    }

    runtime_state_set_attitude_hang_state((attitude_hang_state_t){
        .ref_valid = true,
        .ref_q_body_to_world = normalized,
        .err_roll_deg = 0.0f,
        .err_pitch_deg = 0.0f,
        .rate_sp_roll_dps = 0.0f,
        .rate_sp_pitch_dps = 0.0f,
        .base_duty_active = 0.0f,
    });
    return true;
}

void attitude_bench_clear_reference(void)
{
    runtime_state_set_attitude_hang_state((attitude_hang_state_t){0});
}

void attitude_bench_reset_status(void)
{
    attitude_hang_state_t state = runtime_state_get_attitude_hang_state();
    state.err_roll_deg = 0.0f;
    state.err_pitch_deg = 0.0f;
    state.rate_sp_roll_dps = 0.0f;
    state.rate_sp_pitch_dps = 0.0f;
    state.base_duty_active = 0.0f;
    runtime_state_set_attitude_hang_state(state);
}

bool attitude_bench_compute(const estimator_state_t *estimator_state, axis3f_t *out_rate_setpoint_dps)
{
    bool current_valid = false;
    quatf_t q_now = {0};
    attitude_hang_state_t state = runtime_state_get_attitude_hang_state();
    const params_store_t *params = params_get();

    if (out_rate_setpoint_dps == NULL) {
        return false;
    }
    *out_rate_setpoint_dps = (axis3f_t){0};

    if (estimator_state == NULL || !state.ref_valid || !estimator_state->attitude_valid) {
        attitude_bench_reset_status();
        return false;
    }

    q_now = attitude_bench_quat_normalize(estimator_state->quat_body_to_world, &current_valid);
    if (!current_valid) {
        attitude_bench_reset_status();
        return false;
    }

    quatf_t q_ref = state.ref_q_body_to_world;
    quatf_t q_rel = attitude_bench_quat_multiply(attitude_bench_quat_conjugate(q_ref), q_now);
    bool rel_valid = false;
    q_rel = attitude_bench_quat_normalize(q_rel, &rel_valid);
    if (!rel_valid) {
        attitude_bench_reset_status();
        return false;
    }

    if (q_rel.w < 0.0f) {
        q_rel.w = -q_rel.w;
        q_rel.x = -q_rel.x;
        q_rel.y = -q_rel.y;
        q_rel.z = -q_rel.z;
    }

    const vec3f_t rotvec_rad = attitude_bench_quat_to_rotvec_rad(q_rel);
    float err_roll_deg = -rotvec_rad.y * ATTITUDE_BENCH_DEG_PER_RAD;
    float err_pitch_deg = rotvec_rad.x * ATTITUDE_BENCH_DEG_PER_RAD;

    err_roll_deg = attitude_bench_apply_deadband(err_roll_deg, params->attitude_error_deadband_deg);
    err_pitch_deg = attitude_bench_apply_deadband(err_pitch_deg, params->attitude_error_deadband_deg);

    out_rate_setpoint_dps->roll = attitude_bench_clampf(
        -params->attitude_kp_roll * err_roll_deg,
        -params->attitude_rate_limit_roll,
        params->attitude_rate_limit_roll);
    out_rate_setpoint_dps->pitch = attitude_bench_clampf(
        -params->attitude_kp_pitch * err_pitch_deg,
        -params->attitude_rate_limit_pitch,
        params->attitude_rate_limit_pitch);
    out_rate_setpoint_dps->yaw = 0.0f;

    state.err_roll_deg = err_roll_deg;
    state.err_pitch_deg = err_pitch_deg;
    state.rate_sp_roll_dps = out_rate_setpoint_dps->roll;
    state.rate_sp_pitch_dps = out_rate_setpoint_dps->pitch;
    state.base_duty_active = params->attitude_test_base_duty;
    runtime_state_set_attitude_hang_state(state);
    return true;
}
