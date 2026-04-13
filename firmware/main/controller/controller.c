#include "controller.h"

#include <math.h>
#include <string.h>

#include "params.h"

#define CONTROLLER_TWO_PI 6.283185307179586f

typedef struct {
    axis3f_t integral;
    axis3f_t prev_error;
    axis3f_t d_lpf;
    bool roll_initialized;
    bool pitch_initialized;
    bool yaw_initialized;
    bool armed;
    float base_duty;
    bool motor_saturated;
    bool trip_or_failsafe;
    rate_controller_status_t last_status;
} controller_state_t;

static controller_state_t s_state;

static float controller_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float controller_lpf_alpha(float cutoff_hz, float dt_s)
{
    if (cutoff_hz <= 0.0f || dt_s <= 0.0f || !isfinite(cutoff_hz) || !isfinite(dt_s)) {
        return 1.0f;
    }
    const float rc = 1.0f / (CONTROLLER_TWO_PI * cutoff_hz);
    return controller_clampf(dt_s / (rc + dt_s), 0.0f, 1.0f);
}

void controller_init(void)
{
    memset(&s_state, 0, sizeof(s_state));
}

void controller_reset(void)
{
    const bool armed = s_state.armed;
    const float base_duty = s_state.base_duty;
    const bool motor_saturated = s_state.motor_saturated;
    const bool trip_or_failsafe = s_state.trip_or_failsafe;
    memset(&s_state, 0, sizeof(s_state));
    s_state.armed = armed;
    s_state.base_duty = base_duty;
    s_state.motor_saturated = motor_saturated;
    s_state.trip_or_failsafe = trip_or_failsafe;
}

void controller_set_runtime_flags(bool armed,
                                  float base_duty,
                                  bool motor_saturated,
                                  bool trip_or_failsafe)
{
    s_state.armed = armed;
    s_state.base_duty = base_duty;
    s_state.motor_saturated = motor_saturated;
    s_state.trip_or_failsafe = trip_or_failsafe;
    s_state.last_status.motor_saturation = motor_saturated;
    s_state.last_status.integrator_freeze =
        !armed ||
        base_duty <= (params_get()->motor_idle_duty + 0.002f) ||
        motor_saturated ||
        trip_or_failsafe;
}

static bool controller_integrator_should_freeze(const params_store_t *params)
{
    return !s_state.armed ||
           s_state.base_duty <= (params->motor_idle_duty + 0.002f) ||
           s_state.motor_saturated ||
           s_state.trip_or_failsafe;
}

static void controller_update_axis(float setpoint,
                                   float measured,
                                   float dt_s,
                                   float kp,
                                   float ki,
                                   float kd,
                                   float integral_limit,
                                   float output_limit,
                                   float derivative_alpha,
                                   bool freeze_integrator,
                                   float *integral_state,
                                   float *prev_error,
                                   float *d_lpf_state,
                                   bool *axis_initialized,
                                   float *out_error,
                                   float *out_p,
                                   float *out_i,
                                   float *out_d,
                                   float *out_total)
{
    const float error = setpoint - measured;
    const float p_term = kp * error;

    if (freeze_integrator) {
        if (fabsf(setpoint) <= 0.001f && fabsf(measured) <= 0.5f) {
            *integral_state = 0.0f;
        }
    } else {
        *integral_state += error * dt_s;
        *integral_state = controller_clampf(*integral_state, -integral_limit, integral_limit);
    }
    const float i_term = ki * (*integral_state);

    float derivative_raw = 0.0f;
    if (*axis_initialized && dt_s > 0.0f) {
        derivative_raw = (error - *prev_error) / dt_s;
    }
    *d_lpf_state += derivative_alpha * (derivative_raw - *d_lpf_state);
    const float d_term = kd * (*d_lpf_state);

    *prev_error = error;
    *axis_initialized = true;

    const float total = controller_clampf(p_term + i_term + d_term, -output_limit, output_limit);
    *out_error = error;
    *out_p = p_term;
    *out_i = i_term;
    *out_d = d_term;
    *out_total = total;
}

rate_controller_status_t controller_update_rate(const axis3f_t *rate_setpoint_dps, const axis3f_t *measured_rate_dps, float dt_s)
{
    const params_store_t *params = params_get();
    rate_controller_status_t status = {0};

    if (rate_setpoint_dps == NULL || measured_rate_dps == NULL || dt_s <= 0.0f) {
        return s_state.last_status;
    }

    const bool freeze_integrator = controller_integrator_should_freeze(params);
    const float derivative_alpha = controller_lpf_alpha(params->rate_lpf_hz, dt_s);

    status.measured_rate_dps = *measured_rate_dps;
    status.motor_saturation = s_state.motor_saturated;
    status.integrator_freeze = freeze_integrator;

    controller_update_axis(rate_setpoint_dps->roll,
                           measured_rate_dps->roll,
                           dt_s,
                           params->rate_kp_roll,
                           params->rate_ki_roll,
                           params->rate_kd_roll,
                           params->rate_integral_limit,
                           params->rate_output_limit,
                           derivative_alpha,
                           freeze_integrator,
                           &s_state.integral.roll,
                           &s_state.prev_error.roll,
                           &s_state.d_lpf.roll,
                           &s_state.roll_initialized,
                           &status.error.roll,
                           &status.p_term.roll,
                           &status.i_term.roll,
                           &status.d_term.roll,
                           &status.output.roll);
    controller_update_axis(rate_setpoint_dps->pitch,
                           measured_rate_dps->pitch,
                           dt_s,
                           params->rate_kp_pitch,
                           params->rate_ki_pitch,
                           params->rate_kd_pitch,
                           params->rate_integral_limit,
                           params->rate_output_limit,
                           derivative_alpha,
                           freeze_integrator,
                           &s_state.integral.pitch,
                           &s_state.prev_error.pitch,
                           &s_state.d_lpf.pitch,
                           &s_state.pitch_initialized,
                           &status.error.pitch,
                           &status.p_term.pitch,
                           &status.i_term.pitch,
                           &status.d_term.pitch,
                           &status.output.pitch);
    controller_update_axis(rate_setpoint_dps->yaw,
                           measured_rate_dps->yaw,
                           dt_s,
                           params->rate_kp_yaw,
                           params->rate_ki_yaw,
                           params->rate_kd_yaw,
                           params->rate_integral_limit,
                           params->rate_output_limit,
                           derivative_alpha,
                           freeze_integrator,
                           &s_state.integral.yaw,
                           &s_state.prev_error.yaw,
                           &s_state.d_lpf.yaw,
                           &s_state.yaw_initialized,
                           &status.error.yaw,
                           &status.p_term.yaw,
                           &status.i_term.yaw,
                           &status.d_term.yaw,
                           &status.output.yaw);

    s_state.last_status = status;
    return status;
}

rate_controller_status_t controller_get_last_rate_status(void)
{
    return s_state.last_status;
}
