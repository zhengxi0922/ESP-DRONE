#include "ground_tune.h"

#include <math.h>

#include "params.h"
#include "runtime_state.h"

#define GROUND_TUNE_DEG_PER_RAD 57.29577951308232f

static float ground_tune_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static bool ground_tune_quat_is_valid(quatf_t q)
{
    return isfinite(q.w) && isfinite(q.x) && isfinite(q.y) && isfinite(q.z);
}

static quatf_t ground_tune_quat_normalize(quatf_t q, bool *out_valid)
{
    quatf_t normalized = {0};
    const float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

    if (out_valid != NULL) {
        *out_valid = false;
    }
    if (!ground_tune_quat_is_valid(q) || norm <= 1e-6f || !isfinite(norm)) {
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

static quatf_t ground_tune_quat_conjugate(quatf_t q)
{
    return (quatf_t){.w = q.w, .x = -q.x, .y = -q.y, .z = -q.z};
}

static quatf_t ground_tune_quat_multiply(quatf_t a, quatf_t b)
{
    return (quatf_t){
        .w = (a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z),
        .x = (a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y),
        .y = (a.w * b.y) - (a.x * b.z) + (a.y * b.w) + (a.z * b.x),
        .z = (a.w * b.z) + (a.x * b.y) - (a.y * b.x) + (a.z * b.w),
    };
}

static vec3f_t ground_tune_quat_to_rotvec_rad(quatf_t q_rel)
{
    const float vec_norm = sqrtf(q_rel.x * q_rel.x + q_rel.y * q_rel.y + q_rel.z * q_rel.z);
    if (vec_norm <= 1e-6f) {
        return (vec3f_t){.x = 2.0f * q_rel.x, .y = 2.0f * q_rel.y, .z = 2.0f * q_rel.z};
    }

    const float clamped_w = ground_tune_clampf(q_rel.w, -1.0f, 1.0f);
    const float angle = 2.0f * atan2f(vec_norm, clamped_w);
    const float scale = angle / vec_norm;
    return (vec3f_t){.x = q_rel.x * scale, .y = q_rel.y * scale, .z = q_rel.z * scale};
}

static float ground_tune_apply_deadband(float value_deg, float deadband_deg)
{
    return (fabsf(value_deg) <= deadband_deg) ? 0.0f : value_deg;
}

static bool ground_tune_outer_forced(ground_tune_submode_t submode)
{
    return submode == GROUND_TUNE_SUBMODE_ATTITUDE_VERIFY ||
           submode == GROUND_TUNE_SUBMODE_LOW_RISK_LIFTOFF ||
           submode == GROUND_TUNE_SUBMODE_UDP_MANUAL;
}

static float ground_tune_apply_target_limit(float target_deg, const params_store_t *params)
{
    const float limit = params->ground_att_target_limit_deg;
    return ground_tune_clampf(target_deg, -limit, limit);
}

static float ground_tune_clamp_rate(float value, float limit, uint8_t flag, uint8_t *flags)
{
    const float clamped = ground_tune_clampf(value, -limit, limit);
    if (fabsf(clamped - value) > 0.001f && flags != NULL) {
        *flags |= flag;
    }
    return clamped;
}

bool ground_tune_capture_reference(const imu_sample_t *sample, const estimator_state_t *estimator_state)
{
    bool valid = false;
    const params_store_t *params = params_get();

    if (sample == NULL ||
        sample->health != IMU_HEALTH_OK ||
        !sample->has_gyro_acc ||
        !sample->has_quaternion ||
        estimator_state == NULL ||
        !estimator_state->attitude_valid) {
        return false;
    }

    if (params->ground_tune_use_kalman_attitude &&
        !estimator_state->kalman_valid) {
        ground_tune_set_trip_reason(GROUND_TUNE_TRIP_KALMAN_INVALID);
        return false;
    }

    const quatf_t normalized = ground_tune_quat_normalize(sample->quat_wxyz, &valid);
    if (!valid) {
        return false;
    }

    runtime_state_set_ground_tune_state((ground_tune_state_t){
        .ref_valid = true,
        .ref_q_body_to_world = normalized,
        .ref_kalman_roll_deg = estimator_state->kalman_roll_deg,
        .ref_kalman_pitch_deg = estimator_state->kalman_pitch_deg,
        .target_roll_deg = 0.0f,
        .target_pitch_deg = 0.0f,
        .target_yaw_deg = 0.0f,
        .measured_roll_deg = 0.0f,
        .measured_pitch_deg = 0.0f,
        .measured_yaw_deg = 0.0f,
        .error_roll_deg = 0.0f,
        .error_pitch_deg = 0.0f,
        .error_yaw_deg = 0.0f,
        .err_roll_deg = 0.0f,
        .err_pitch_deg = 0.0f,
        .rate_sp_roll_dps = 0.0f,
        .rate_sp_pitch_dps = 0.0f,
        .rate_sp_yaw_dps = 0.0f,
        .base_duty_active = 0.0f,
        .outer_clamp_flags = 0u,
        .inner_clamp_flags = 0u,
        .submode = GROUND_TUNE_SUBMODE_RATE_ONLY,
        .trip_reason = GROUND_TUNE_TRIP_NONE,
    });
    return true;
}

void ground_tune_clear_reference(void)
{
    runtime_state_set_ground_tune_state((ground_tune_state_t){0});
}

void ground_tune_reset_status(void)
{
    ground_tune_state_t state = runtime_state_get_ground_tune_state();
    state.target_roll_deg = 0.0f;
    state.target_pitch_deg = 0.0f;
    state.target_yaw_deg = 0.0f;
    state.measured_roll_deg = 0.0f;
    state.measured_pitch_deg = 0.0f;
    state.measured_yaw_deg = 0.0f;
    state.error_roll_deg = 0.0f;
    state.error_pitch_deg = 0.0f;
    state.error_yaw_deg = 0.0f;
    state.err_roll_deg = 0.0f;
    state.err_pitch_deg = 0.0f;
    state.rate_sp_roll_dps = 0.0f;
    state.rate_sp_pitch_dps = 0.0f;
    state.rate_sp_yaw_dps = 0.0f;
    state.base_duty_active = 0.0f;
    state.outer_clamp_flags = 0u;
    state.inner_clamp_flags = 0u;
    state.trip_reason = GROUND_TUNE_TRIP_NONE;
    runtime_state_set_ground_tune_state(state);
}

void ground_tune_set_trip_reason(ground_tune_trip_reason_t reason)
{
    ground_tune_state_t state = runtime_state_get_ground_tune_state();
    state.trip_reason = reason;
    runtime_state_set_ground_tune_state(state);
}

void ground_tune_set_submode(ground_tune_submode_t submode)
{
    ground_tune_state_t state = runtime_state_get_ground_tune_state();
    state.submode = submode;
    runtime_state_set_ground_tune_state(state);
}

void ground_tune_set_inner_clamp_flags(uint8_t flags)
{
    ground_tune_state_t state = runtime_state_get_ground_tune_state();
    state.inner_clamp_flags = flags;
    runtime_state_set_ground_tune_state(state);
}

bool ground_tune_set_angle_target(uint8_t axis_id, float target_deg)
{
    const params_store_t *params = params_get();
    if (!isfinite(target_deg) || params == NULL) {
        return false;
    }

    ground_tune_state_t state = runtime_state_get_ground_tune_state();
    const float limited = ground_tune_apply_target_limit(target_deg, params);
    switch (axis_id) {
    case 0:
        state.target_roll_deg = limited;
        break;
    case 1:
        state.target_pitch_deg = limited;
        break;
    case 2:
        state.target_yaw_deg = 0.0f;
        break;
    default:
        return false;
    }
    runtime_state_set_ground_tune_state(state);
    return true;
}

bool ground_tune_compute(const estimator_state_t *estimator_state, axis3f_t *out_rate_setpoint_dps)
{
    ground_tune_state_t state = runtime_state_get_ground_tune_state();
    const params_store_t *params = params_get();

    if (out_rate_setpoint_dps == NULL) {
        return false;
    }
    *out_rate_setpoint_dps = (axis3f_t){0};

    if (estimator_state == NULL || !state.ref_valid) {
        ground_tune_set_trip_reason(GROUND_TUNE_TRIP_REF_MISSING);
        return false;
    }

    const bool outer_enabled = params->ground_tune_enable_attitude_outer || ground_tune_outer_forced(state.submode);
    if (params->ground_tune_use_kalman_attitude && !estimator_state->kalman_valid) {
        ground_tune_set_trip_reason(GROUND_TUNE_TRIP_KALMAN_INVALID);
        return false;
    }

    bool current_valid = false;
    quatf_t q_now = ground_tune_quat_normalize(estimator_state->quat_body_to_world, &current_valid);
    if (!current_valid || !estimator_state->attitude_valid) {
        ground_tune_set_trip_reason(GROUND_TUNE_TRIP_IMU_STALE);
        return false;
    }

    quatf_t q_rel = ground_tune_quat_multiply(ground_tune_quat_conjugate(state.ref_q_body_to_world), q_now);
    bool rel_valid = false;
    q_rel = ground_tune_quat_normalize(q_rel, &rel_valid);
    if (!rel_valid) {
        ground_tune_set_trip_reason(GROUND_TUNE_TRIP_IMU_STALE);
        return false;
    }
    if (q_rel.w < 0.0f) {
        q_rel.w = -q_rel.w;
        q_rel.x = -q_rel.x;
        q_rel.y = -q_rel.y;
        q_rel.z = -q_rel.z;
    }

    const vec3f_t rotvec_rad = ground_tune_quat_to_rotvec_rad(q_rel);
    float measured_roll_deg = -rotvec_rad.y * GROUND_TUNE_DEG_PER_RAD;
    float measured_pitch_deg = rotvec_rad.x * GROUND_TUNE_DEG_PER_RAD;
    if (params->ground_tune_use_kalman_attitude) {
        measured_roll_deg = estimator_state->kalman_roll_deg - state.ref_kalman_roll_deg;
        measured_pitch_deg = estimator_state->kalman_pitch_deg - state.ref_kalman_pitch_deg;
    }

    const float target_roll_deg = ground_tune_apply_target_limit(state.target_roll_deg, params);
    const float target_pitch_deg = ground_tune_apply_target_limit(state.target_pitch_deg, params);
    const float target_yaw_deg = 0.0f;
    const float display_err_roll_deg =
        ground_tune_apply_deadband(measured_roll_deg, params->ground_att_error_deadband_deg);
    const float display_err_pitch_deg =
        ground_tune_apply_deadband(measured_pitch_deg, params->ground_att_error_deadband_deg);
    const float error_roll_deg =
        ground_tune_apply_deadband(target_roll_deg - measured_roll_deg, params->ground_att_error_deadband_deg);
    const float error_pitch_deg =
        ground_tune_apply_deadband(target_pitch_deg - measured_pitch_deg, params->ground_att_error_deadband_deg);
    const axis3f_t runtime_rate_request = runtime_state_get_rate_setpoint_request();
    uint8_t outer_clamp_flags = 0u;

    if (outer_enabled) {
        out_rate_setpoint_dps->roll = ground_tune_clamp_rate(
            params->ground_att_kp_roll * error_roll_deg,
            params->ground_att_rate_limit_roll,
            GROUND_TUNE_OUTER_CLAMP_ROLL,
            &outer_clamp_flags);
        out_rate_setpoint_dps->pitch = ground_tune_clamp_rate(
            params->ground_att_kp_pitch * error_pitch_deg,
            params->ground_att_rate_limit_pitch,
            GROUND_TUNE_OUTER_CLAMP_PITCH,
            &outer_clamp_flags);
        out_rate_setpoint_dps->yaw = runtime_rate_request.yaw;
    } else {
        *out_rate_setpoint_dps = (axis3f_t){0};
    }

    state.target_roll_deg = target_roll_deg;
    state.target_pitch_deg = target_pitch_deg;
    state.target_yaw_deg = target_yaw_deg;
    state.measured_roll_deg = measured_roll_deg;
    state.measured_pitch_deg = measured_pitch_deg;
    state.measured_yaw_deg = 0.0f;
    state.error_roll_deg = error_roll_deg;
    state.error_pitch_deg = error_pitch_deg;
    state.error_yaw_deg = 0.0f;
    state.err_roll_deg = display_err_roll_deg;
    state.err_pitch_deg = display_err_pitch_deg;
    state.rate_sp_roll_dps = out_rate_setpoint_dps->roll;
    state.rate_sp_pitch_dps = out_rate_setpoint_dps->pitch;
    state.rate_sp_yaw_dps = out_rate_setpoint_dps->yaw;
    state.outer_clamp_flags = outer_clamp_flags;
    state.trip_reason = GROUND_TUNE_TRIP_NONE;
    runtime_state_set_ground_tune_state(state);
    return true;
}

const char *ground_tune_trip_reason_text(ground_tune_trip_reason_t reason)
{
    switch (reason) {
    case GROUND_TUNE_TRIP_NONE:
        return "none";
    case GROUND_TUNE_TRIP_REF_MISSING:
        return "ground ref missing";
    case GROUND_TUNE_TRIP_KALMAN_INVALID:
        return "kalman invalid";
    case GROUND_TUNE_TRIP_ANGLE:
        return "angle trip";
    case GROUND_TUNE_TRIP_SATURATION:
        return "saturation trip";
    case GROUND_TUNE_TRIP_IMU_STALE:
        return "imu stale";
    case GROUND_TUNE_TRIP_WATCHDOG:
        return "watchdog timeout";
    case GROUND_TUNE_TRIP_RATE_JITTER:
        return "rate jitter trip";
    case GROUND_TUNE_TRIP_FAILSAFE:
        return "failsafe";
    case GROUND_TUNE_TRIP_STOP_NORMAL:
        return "ground tune stopped normally";
    case GROUND_TUNE_TRIP_BATTERY:
        return "battery invalid or critical";
    default:
        return "unknown";
    }
}
