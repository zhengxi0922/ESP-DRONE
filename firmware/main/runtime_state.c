/**
 * @file runtime_state.c
 * @brief 跨任务运行时状态实现。
 */

#include "runtime_state.h"

#include <string.h>
#include <stdatomic.h>

static _Atomic int g_arm_state;
static _Atomic int g_failsafe_reason;
static _Atomic uint32_t g_loop_dt_us;
static _Atomic uint32_t g_max_loop_dt_us;
static _Atomic uint32_t g_loop_overrun_count;
static _Atomic bool g_stream_enabled;
static _Atomic int g_motor_test_logical;
static _Atomic uint32_t g_motor_test_duty_bits;
static _Atomic int g_control_mode;
static _Atomic uint32_t g_axis_test_roll_bits;
static _Atomic uint32_t g_axis_test_pitch_bits;
static _Atomic uint32_t g_axis_test_yaw_bits;
static _Atomic uint32_t g_rate_setpoint_roll_bits;
static _Atomic uint32_t g_rate_setpoint_pitch_bits;
static _Atomic uint32_t g_rate_setpoint_yaw_bits;
static _Atomic bool g_attitude_ref_valid;
static _Atomic uint32_t g_attitude_ref_qw_bits;
static _Atomic uint32_t g_attitude_ref_qx_bits;
static _Atomic uint32_t g_attitude_ref_qy_bits;
static _Atomic uint32_t g_attitude_ref_qz_bits;
static _Atomic uint32_t g_attitude_err_roll_bits;
static _Atomic uint32_t g_attitude_err_pitch_bits;
static _Atomic uint32_t g_attitude_rate_sp_roll_bits;
static _Atomic uint32_t g_attitude_rate_sp_pitch_bits;
static _Atomic uint32_t g_base_duty_active_bits;
static _Atomic bool g_ground_ref_valid;
static _Atomic uint32_t g_ground_ref_qw_bits;
static _Atomic uint32_t g_ground_ref_qx_bits;
static _Atomic uint32_t g_ground_ref_qy_bits;
static _Atomic uint32_t g_ground_ref_qz_bits;
static _Atomic uint32_t g_ground_ref_kalman_roll_bits;
static _Atomic uint32_t g_ground_ref_kalman_pitch_bits;
static _Atomic uint32_t g_ground_err_roll_bits;
static _Atomic uint32_t g_ground_err_pitch_bits;
static _Atomic uint32_t g_ground_rate_sp_roll_bits;
static _Atomic uint32_t g_ground_rate_sp_pitch_bits;
static _Atomic uint32_t g_ground_base_duty_active_bits;
static _Atomic int g_ground_trip_reason;

static uint32_t runtime_state_float_bits(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static float runtime_state_bits_float(uint32_t bits)
{
    float value = 0.0f;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

void runtime_state_init(void)
{
    atomic_store(&g_arm_state, ARM_STATE_DISARMED);
    atomic_store(&g_failsafe_reason, FAILSAFE_REASON_NONE);
    atomic_store(&g_loop_dt_us, 0);
    atomic_store(&g_max_loop_dt_us, 0);
    atomic_store(&g_loop_overrun_count, 0);
    atomic_store(&g_stream_enabled, false);
    atomic_store(&g_motor_test_logical, -1);
    atomic_store(&g_motor_test_duty_bits, 0u);
    atomic_store(&g_control_mode, CONTROL_MODE_IDLE);
    atomic_store(&g_axis_test_roll_bits, 0u);
    atomic_store(&g_axis_test_pitch_bits, 0u);
    atomic_store(&g_axis_test_yaw_bits, 0u);
    atomic_store(&g_rate_setpoint_roll_bits, 0u);
    atomic_store(&g_rate_setpoint_pitch_bits, 0u);
    atomic_store(&g_rate_setpoint_yaw_bits, 0u);
    atomic_store(&g_attitude_ref_valid, false);
    atomic_store(&g_attitude_ref_qw_bits, 0u);
    atomic_store(&g_attitude_ref_qx_bits, 0u);
    atomic_store(&g_attitude_ref_qy_bits, 0u);
    atomic_store(&g_attitude_ref_qz_bits, 0u);
    atomic_store(&g_attitude_err_roll_bits, 0u);
    atomic_store(&g_attitude_err_pitch_bits, 0u);
    atomic_store(&g_attitude_rate_sp_roll_bits, 0u);
    atomic_store(&g_attitude_rate_sp_pitch_bits, 0u);
    atomic_store(&g_base_duty_active_bits, 0u);
    atomic_store(&g_ground_ref_valid, false);
    atomic_store(&g_ground_ref_qw_bits, 0u);
    atomic_store(&g_ground_ref_qx_bits, 0u);
    atomic_store(&g_ground_ref_qy_bits, 0u);
    atomic_store(&g_ground_ref_qz_bits, 0u);
    atomic_store(&g_ground_ref_kalman_roll_bits, 0u);
    atomic_store(&g_ground_ref_kalman_pitch_bits, 0u);
    atomic_store(&g_ground_err_roll_bits, 0u);
    atomic_store(&g_ground_err_pitch_bits, 0u);
    atomic_store(&g_ground_rate_sp_roll_bits, 0u);
    atomic_store(&g_ground_rate_sp_pitch_bits, 0u);
    atomic_store(&g_ground_base_duty_active_bits, 0u);
    atomic_store(&g_ground_trip_reason, GROUND_TUNE_TRIP_NONE);
}

void runtime_state_set_arm_state(arm_state_t state)
{
    atomic_store(&g_arm_state, (int)state);
}

arm_state_t runtime_state_get_arm_state(void)
{
    return (arm_state_t)atomic_load(&g_arm_state);
}

void runtime_state_set_failsafe_reason(failsafe_reason_t reason)
{
    atomic_store(&g_failsafe_reason, (int)reason);
}

failsafe_reason_t runtime_state_get_failsafe_reason(void)
{
    return (failsafe_reason_t)atomic_load(&g_failsafe_reason);
}

void runtime_state_set_loop_stats(loop_stats_t stats)
{
    atomic_store(&g_loop_dt_us, stats.loop_dt_us);
    atomic_store(&g_max_loop_dt_us, stats.max_loop_dt_us);
    atomic_store(&g_loop_overrun_count, stats.loop_overrun_count);
}

loop_stats_t runtime_state_get_loop_stats(void)
{
    loop_stats_t stats = {
        .loop_dt_us = atomic_load(&g_loop_dt_us),
        .max_loop_dt_us = atomic_load(&g_max_loop_dt_us),
        .loop_overrun_count = atomic_load(&g_loop_overrun_count),
    };
    return stats;
}

void runtime_state_set_stream_enabled(bool enabled)
{
    atomic_store(&g_stream_enabled, enabled);
}

bool runtime_state_get_stream_enabled(void)
{
    return atomic_load(&g_stream_enabled);
}

void runtime_state_set_motor_test(int logical_motor, float duty)
{
    atomic_store(&g_motor_test_logical, logical_motor);
    atomic_store(&g_motor_test_duty_bits, runtime_state_float_bits(duty));
}

void runtime_state_get_motor_test(int *out_logical_motor, float *out_duty)
{
    if (out_logical_motor != NULL) {
        *out_logical_motor = atomic_load(&g_motor_test_logical);
    }
    if (out_duty != NULL) {
        *out_duty = runtime_state_bits_float(atomic_load(&g_motor_test_duty_bits));
    }
}

void runtime_state_set_control_mode(control_mode_t mode)
{
    atomic_store(&g_control_mode, (int)mode);
}

control_mode_t runtime_state_get_control_mode(void)
{
    return (control_mode_t)atomic_load(&g_control_mode);
}

void runtime_state_set_axis_test_request(axis3f_t request)
{
    atomic_store(&g_axis_test_roll_bits, runtime_state_float_bits(request.roll));
    atomic_store(&g_axis_test_pitch_bits, runtime_state_float_bits(request.pitch));
    atomic_store(&g_axis_test_yaw_bits, runtime_state_float_bits(request.yaw));
}

axis3f_t runtime_state_get_axis_test_request(void)
{
    return (axis3f_t){
        .roll = runtime_state_bits_float(atomic_load(&g_axis_test_roll_bits)),
        .pitch = runtime_state_bits_float(atomic_load(&g_axis_test_pitch_bits)),
        .yaw = runtime_state_bits_float(atomic_load(&g_axis_test_yaw_bits)),
    };
}

void runtime_state_set_rate_setpoint_request(axis3f_t request)
{
    atomic_store(&g_rate_setpoint_roll_bits, runtime_state_float_bits(request.roll));
    atomic_store(&g_rate_setpoint_pitch_bits, runtime_state_float_bits(request.pitch));
    atomic_store(&g_rate_setpoint_yaw_bits, runtime_state_float_bits(request.yaw));
}

axis3f_t runtime_state_get_rate_setpoint_request(void)
{
    return (axis3f_t){
        .roll = runtime_state_bits_float(atomic_load(&g_rate_setpoint_roll_bits)),
        .pitch = runtime_state_bits_float(atomic_load(&g_rate_setpoint_pitch_bits)),
        .yaw = runtime_state_bits_float(atomic_load(&g_rate_setpoint_yaw_bits)),
    };
}

void runtime_state_set_attitude_reference(bool valid, quatf_t ref_q_body_to_world)
{
    atomic_store(&g_attitude_ref_valid, valid);
    atomic_store(&g_attitude_ref_qw_bits, runtime_state_float_bits(ref_q_body_to_world.w));
    atomic_store(&g_attitude_ref_qx_bits, runtime_state_float_bits(ref_q_body_to_world.x));
    atomic_store(&g_attitude_ref_qy_bits, runtime_state_float_bits(ref_q_body_to_world.y));
    atomic_store(&g_attitude_ref_qz_bits, runtime_state_float_bits(ref_q_body_to_world.z));
}

void runtime_state_clear_attitude_reference(void)
{
    runtime_state_set_attitude_reference(false, (quatf_t){0});
}

attitude_hang_state_t runtime_state_get_attitude_hang_state(void)
{
    return (attitude_hang_state_t){
        .ref_valid = atomic_load(&g_attitude_ref_valid),
        .ref_q_body_to_world =
            {
                .w = runtime_state_bits_float(atomic_load(&g_attitude_ref_qw_bits)),
                .x = runtime_state_bits_float(atomic_load(&g_attitude_ref_qx_bits)),
                .y = runtime_state_bits_float(atomic_load(&g_attitude_ref_qy_bits)),
                .z = runtime_state_bits_float(atomic_load(&g_attitude_ref_qz_bits)),
            },
        .err_roll_deg = runtime_state_bits_float(atomic_load(&g_attitude_err_roll_bits)),
        .err_pitch_deg = runtime_state_bits_float(atomic_load(&g_attitude_err_pitch_bits)),
        .rate_sp_roll_dps = runtime_state_bits_float(atomic_load(&g_attitude_rate_sp_roll_bits)),
        .rate_sp_pitch_dps = runtime_state_bits_float(atomic_load(&g_attitude_rate_sp_pitch_bits)),
        .base_duty_active = runtime_state_bits_float(atomic_load(&g_base_duty_active_bits)),
    };
}

void runtime_state_set_attitude_hang_state(attitude_hang_state_t state)
{
    runtime_state_set_attitude_reference(state.ref_valid, state.ref_q_body_to_world);
    atomic_store(&g_attitude_err_roll_bits, runtime_state_float_bits(state.err_roll_deg));
    atomic_store(&g_attitude_err_pitch_bits, runtime_state_float_bits(state.err_pitch_deg));
    atomic_store(&g_attitude_rate_sp_roll_bits, runtime_state_float_bits(state.rate_sp_roll_dps));
    atomic_store(&g_attitude_rate_sp_pitch_bits, runtime_state_float_bits(state.rate_sp_pitch_dps));
    atomic_store(&g_base_duty_active_bits, runtime_state_float_bits(state.base_duty_active));
}

void runtime_state_set_ground_reference(bool valid,
                                        quatf_t ref_q_body_to_world,
                                        float ref_kalman_roll_deg,
                                        float ref_kalman_pitch_deg)
{
    atomic_store(&g_ground_ref_valid, valid);
    atomic_store(&g_ground_ref_qw_bits, runtime_state_float_bits(ref_q_body_to_world.w));
    atomic_store(&g_ground_ref_qx_bits, runtime_state_float_bits(ref_q_body_to_world.x));
    atomic_store(&g_ground_ref_qy_bits, runtime_state_float_bits(ref_q_body_to_world.y));
    atomic_store(&g_ground_ref_qz_bits, runtime_state_float_bits(ref_q_body_to_world.z));
    atomic_store(&g_ground_ref_kalman_roll_bits, runtime_state_float_bits(ref_kalman_roll_deg));
    atomic_store(&g_ground_ref_kalman_pitch_bits, runtime_state_float_bits(ref_kalman_pitch_deg));
}

void runtime_state_clear_ground_reference(void)
{
    runtime_state_set_ground_tune_state((ground_tune_state_t){0});
}

ground_tune_state_t runtime_state_get_ground_tune_state(void)
{
    return (ground_tune_state_t){
        .ref_valid = atomic_load(&g_ground_ref_valid),
        .ref_q_body_to_world =
            {
                .w = runtime_state_bits_float(atomic_load(&g_ground_ref_qw_bits)),
                .x = runtime_state_bits_float(atomic_load(&g_ground_ref_qx_bits)),
                .y = runtime_state_bits_float(atomic_load(&g_ground_ref_qy_bits)),
                .z = runtime_state_bits_float(atomic_load(&g_ground_ref_qz_bits)),
            },
        .ref_kalman_roll_deg = runtime_state_bits_float(atomic_load(&g_ground_ref_kalman_roll_bits)),
        .ref_kalman_pitch_deg = runtime_state_bits_float(atomic_load(&g_ground_ref_kalman_pitch_bits)),
        .err_roll_deg = runtime_state_bits_float(atomic_load(&g_ground_err_roll_bits)),
        .err_pitch_deg = runtime_state_bits_float(atomic_load(&g_ground_err_pitch_bits)),
        .rate_sp_roll_dps = runtime_state_bits_float(atomic_load(&g_ground_rate_sp_roll_bits)),
        .rate_sp_pitch_dps = runtime_state_bits_float(atomic_load(&g_ground_rate_sp_pitch_bits)),
        .base_duty_active = runtime_state_bits_float(atomic_load(&g_ground_base_duty_active_bits)),
        .trip_reason = (ground_tune_trip_reason_t)atomic_load(&g_ground_trip_reason),
    };
}

void runtime_state_set_ground_tune_state(ground_tune_state_t state)
{
    runtime_state_set_ground_reference(state.ref_valid,
                                       state.ref_q_body_to_world,
                                       state.ref_kalman_roll_deg,
                                       state.ref_kalman_pitch_deg);
    atomic_store(&g_ground_err_roll_bits, runtime_state_float_bits(state.err_roll_deg));
    atomic_store(&g_ground_err_pitch_bits, runtime_state_float_bits(state.err_pitch_deg));
    atomic_store(&g_ground_rate_sp_roll_bits, runtime_state_float_bits(state.rate_sp_roll_dps));
    atomic_store(&g_ground_rate_sp_pitch_bits, runtime_state_float_bits(state.rate_sp_pitch_dps));
    atomic_store(&g_ground_base_duty_active_bits, runtime_state_float_bits(state.base_duty_active));
    atomic_store(&g_ground_trip_reason, (int)state.trip_reason);
}
