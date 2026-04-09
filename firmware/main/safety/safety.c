/**
 * @file safety.c
 * @brief 安全状态机实现。
 */

#include "safety.h"

#include <stdatomic.h>

#include "params.h"
#include "runtime_state.h"

#define SAFETY_BATTERY_CRITICAL_DEBOUNCE_CYCLES 100u

static _Atomic int s_arm_state;
static _Atomic int s_failsafe_reason;
static _Atomic bool s_pending_arm;
static _Atomic bool s_pending_disarm;
static _Atomic bool s_pending_kill;
static _Atomic bool s_pending_cli_override;
static _Atomic bool s_cli_override_active;
static _Atomic uint32_t s_battery_critical_streak;

static bool safety_battery_critical_active(const safety_inputs_t *inputs)
{
    const params_store_t *params = params_get();

    if (inputs == NULL || !(inputs->battery_voltage > 0.1f)) {
        atomic_store(&s_battery_critical_streak, 0u);
        return false;
    }

    if (inputs->battery_voltage <= params->battery_critical_v) {
        uint32_t streak = atomic_load(&s_battery_critical_streak);
        if (streak < SAFETY_BATTERY_CRITICAL_DEBOUNCE_CYCLES) {
            streak++;
            atomic_store(&s_battery_critical_streak, streak);
        }
        return streak >= SAFETY_BATTERY_CRITICAL_DEBOUNCE_CYCLES;
    }

    atomic_store(&s_battery_critical_streak, 0u);
    return false;
}

/* failsafe 原因在这里统一收口。
 * 这样 arm/disarm/kill、IMU 超时、低压、loop overrun 不会分散在多个模块里各自判定。 */
static failsafe_reason_t safety_detect_reason(const safety_inputs_t *inputs)
{
    if (inputs == NULL) {
        return FAILSAFE_REASON_NONE;
    }

    /* Constrained bench tests can produce short PWM-induced battery dips.
     * Require a short low-voltage streak so transient sag does not trip rate bench steps immediately. */
    if (safety_battery_critical_active(inputs)) {
        return FAILSAFE_REASON_BATTERY_CRITICAL;
    }
    const params_store_t *params = params_get();
    if (inputs->imu_health == IMU_HEALTH_TIMEOUT) {
        return FAILSAFE_REASON_IMU_TIMEOUT;
    }
    if (inputs->imu_health == IMU_HEALTH_PARSE_ERROR ||
        inputs->imu_stats.consecutive_parse_errors >= params->imu_parse_error_limit) {
        return FAILSAFE_REASON_IMU_PARSE;
    }
    if (inputs->loop_stats.loop_overrun_count >= params->loop_overrun_limit) {
        return FAILSAFE_REASON_LOOP_OVERRUN;
    }
    if (atomic_load(&s_arm_state) == ARM_STATE_ARMED &&
        !inputs->rc_link_online &&
        !atomic_load(&s_cli_override_active)) {
        return FAILSAFE_REASON_RC_TIMEOUT;
    }

    return FAILSAFE_REASON_NONE;
}

static bool safety_arm_conditions_met(const safety_inputs_t *inputs, bool cli_override_allowed)
{
    const params_store_t *params = params_get();

    if (inputs == NULL) {
        return false;
    }
    if (inputs->throttle_norm >= 0.05f) {
        return false;
    }
    if (inputs->imu_health != IMU_HEALTH_OK) {
        return false;
    }
    if (inputs->battery_voltage > 0.1f && inputs->battery_voltage <= params->battery_arm_v) {
        return false;
    }
    /* 允许 CLI override 的唯一目的，是在 bench bring-up 阶段没有 RC 链路时仍可做受控台架测试。 */
    if (!inputs->rc_link_online && !cli_override_allowed) {
        return false;
    }

    return true;
}

void safety_init(void)
{
    atomic_store(&s_arm_state, ARM_STATE_DISARMED);
    atomic_store(&s_failsafe_reason, FAILSAFE_REASON_NONE);
    atomic_store(&s_pending_arm, false);
    atomic_store(&s_pending_disarm, false);
    atomic_store(&s_pending_kill, false);
    atomic_store(&s_pending_cli_override, false);
    atomic_store(&s_cli_override_active, false);
    atomic_store(&s_battery_critical_streak, 0u);

    runtime_state_set_arm_state(ARM_STATE_DISARMED);
    runtime_state_set_failsafe_reason(FAILSAFE_REASON_NONE);
}

bool safety_request_arm(bool cli_override_allowed)
{
    if (atomic_load(&s_arm_state) == ARM_STATE_FAULT_LOCK) {
        return false;
    }

    atomic_store(&s_pending_cli_override, cli_override_allowed);
    atomic_store(&s_pending_arm, true);
    return true;
}

void safety_request_disarm(void)
{
    atomic_store(&s_pending_disarm, true);
}

void safety_request_kill(void)
{
    atomic_store(&s_pending_kill, true);
}

void safety_update(const safety_inputs_t *inputs, safety_status_t *out_status)
{
    arm_state_t state = (arm_state_t)atomic_load(&s_arm_state);
    failsafe_reason_t reason = (failsafe_reason_t)atomic_load(&s_failsafe_reason);

    /* kill 优先级最高，进入 fault lock 后必须人工干预，不能被普通 arm 覆盖。 */
    if (atomic_exchange(&s_pending_kill, false)) {
        state = ARM_STATE_FAULT_LOCK;
        reason = FAILSAFE_REASON_KILL;
        atomic_store(&s_pending_arm, false);
        atomic_store(&s_pending_disarm, false);
        atomic_store(&s_cli_override_active, false);
        atomic_store(&s_battery_critical_streak, 0u);
    }

    if (state != ARM_STATE_FAULT_LOCK) {
        const failsafe_reason_t detected = safety_detect_reason(inputs);
        if (detected != FAILSAFE_REASON_NONE) {
            state = ARM_STATE_FAILSAFE;
            reason = detected;
        }
    }

    if (atomic_exchange(&s_pending_disarm, false) && state != ARM_STATE_FAULT_LOCK) {
        state = ARM_STATE_DISARMED;
        reason = FAILSAFE_REASON_NONE;
        atomic_store(&s_cli_override_active, false);
        atomic_store(&s_battery_critical_streak, 0u);
    }

    if (atomic_exchange(&s_pending_arm, false) && state == ARM_STATE_DISARMED) {
        const bool cli_override_allowed = atomic_exchange(&s_pending_cli_override, false);
        if (safety_arm_conditions_met(inputs, cli_override_allowed)) {
            state = ARM_STATE_ARMED;
            reason = FAILSAFE_REASON_NONE;
            atomic_store(&s_cli_override_active, cli_override_allowed);
        }
    } else {
        atomic_store(&s_pending_cli_override, false);
    }

    atomic_store(&s_arm_state, state);
    atomic_store(&s_failsafe_reason, reason);
    runtime_state_set_arm_state(state);
    runtime_state_set_failsafe_reason(reason);

    if (out_status != NULL) {
        out_status->arm_state = state;
        out_status->failsafe_reason = reason;
        out_status->motors_should_stop = (state != ARM_STATE_ARMED);
    }
}
