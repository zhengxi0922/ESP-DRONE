/**
 * @file controller.c
 * @brief 速率环控制器实现。
 */

#include "controller.h"

#include <string.h>

#include "params.h"

typedef struct {
    axis3f_t integral;
    axis3f_t prev_error;
    bool initialized;
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

void controller_init(void)
{
    memset(&s_state, 0, sizeof(s_state));
}

void controller_reset(void)
{
    memset(&s_state, 0, sizeof(s_state));
}

static void controller_update_axis(float setpoint,
                                   float measured,
                                   float dt_s,
                                   float kp,
                                   float ki,
                                   float kd,
                                   float integral_limit,
                                   float output_limit,
                                   float *integral_state,
                                   float *prev_error,
                                   bool *axis_initialized,
                                   float *out_p,
                                   float *out_i,
                                   float *out_d,
                                   float *out_total)
{
    const float error = setpoint - measured;
    const float p_term = kp * error;

    *integral_state += error * dt_s;
    *integral_state = controller_clampf(*integral_state, -integral_limit, integral_limit);
    const float i_term = ki * (*integral_state);

    float d_term = 0.0f;
    if (*axis_initialized && dt_s > 0.0f) {
        d_term = kd * ((error - *prev_error) / dt_s);
    }
    *prev_error = error;
    *axis_initialized = true;

    const float total = controller_clampf(p_term + i_term + d_term, -output_limit, output_limit);
    *out_p = p_term;
    *out_i = i_term;
    *out_d = d_term;
    *out_total = total;
}

rate_controller_status_t controller_update_rate(const axis3f_t *rate_setpoint_dps, const axis3f_t *measured_rate_dps, float dt_s)
{
    const params_store_t *params = params_get();
    rate_controller_status_t status = {0};
    bool axis_initialized = s_state.initialized;

    if (rate_setpoint_dps == NULL || measured_rate_dps == NULL || dt_s <= 0.0f) {
        return s_state.last_status;
    }

    controller_update_axis(rate_setpoint_dps->roll,
                           measured_rate_dps->roll,
                           dt_s,
                           params->rate_kp_roll,
                           params->rate_ki_roll,
                           params->rate_kd_roll,
                           params->rate_integral_limit,
                           params->rate_output_limit,
                           &s_state.integral.roll,
                           &s_state.prev_error.roll,
                           &axis_initialized,
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
                           &s_state.integral.pitch,
                           &s_state.prev_error.pitch,
                           &axis_initialized,
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
                           &s_state.integral.yaw,
                           &s_state.prev_error.yaw,
                           &axis_initialized,
                           &status.p_term.yaw,
                           &status.i_term.yaw,
                           &status.d_term.yaw,
                           &status.output.yaw);

    s_state.initialized = axis_initialized;
    s_state.last_status = status;
    return status;
}

rate_controller_status_t controller_get_last_rate_status(void)
{
    return s_state.last_status;
}
