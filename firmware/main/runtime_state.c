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
    uint32_t bits = 0;
    memcpy(&bits, &duty, sizeof(bits));
    atomic_store(&g_motor_test_logical, logical_motor);
    atomic_store(&g_motor_test_duty_bits, bits);
}

void runtime_state_get_motor_test(int *out_logical_motor, float *out_duty)
{
    if (out_logical_motor != NULL) {
        *out_logical_motor = atomic_load(&g_motor_test_logical);
    }
    if (out_duty != NULL) {
        uint32_t bits = atomic_load(&g_motor_test_duty_bits);
        memcpy(out_duty, &bits, sizeof(bits));
    }
}
