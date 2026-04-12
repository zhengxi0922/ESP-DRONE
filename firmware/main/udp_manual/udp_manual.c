#include "udp_manual.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include "esp_timer.h"

#include "console.h"
#include "motor.h"
#include "params.h"
#include "runtime_state.h"
#include "safety.h"

#define UDP_MANUAL_RAMP_DUTY_PER_S 0.25f
#define UDP_MANUAL_LONG_TIMEOUT_MULTIPLIER 3u

typedef enum {
    UDP_MANUAL_STAGE_DISABLED = 0,
    UDP_MANUAL_STAGE_IDLE = 1,
    UDP_MANUAL_STAGE_MANUAL = 2,
    UDP_MANUAL_STAGE_TAKEOFF = 3,
    UDP_MANUAL_STAGE_LAND = 4,
} udp_manual_stage_t;

typedef struct {
    bool enabled;
    bool timeout_event_sent;
    bool disarm_event_sent;
    udp_manual_stage_t stage;
    uint64_t last_frame_us;
    float current_throttle;
    float desired_throttle;
    axis3f_t desired_axis;
} udp_manual_state_t;

static portMUX_TYPE s_udp_manual_lock = portMUX_INITIALIZER_UNLOCKED;
static udp_manual_state_t s_udp_manual_state;

static float udp_manual_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static bool udp_manual_all_finite(float a, float b, float c, float d)
{
    return isfinite(a) && isfinite(b) && isfinite(c) && isfinite(d);
}

static void udp_manual_reset_locked(void)
{
    memset(&s_udp_manual_state, 0, sizeof(s_udp_manual_state));
    s_udp_manual_state.stage = UDP_MANUAL_STAGE_DISABLED;
}

void udp_manual_init(void)
{
    taskENTER_CRITICAL(&s_udp_manual_lock);
    udp_manual_reset_locked();
    taskEXIT_CRITICAL(&s_udp_manual_lock);
}

void udp_manual_reset(void)
{
    taskENTER_CRITICAL(&s_udp_manual_lock);
    udp_manual_reset_locked();
    taskEXIT_CRITICAL(&s_udp_manual_lock);
}

console_cmd_status_t udp_manual_enable(void)
{
    const arm_state_t arm_state = runtime_state_get_arm_state();
    const control_mode_t mode = runtime_state_get_control_mode();

    if (arm_state == ARM_STATE_FAILSAFE || arm_state == ARM_STATE_FAULT_LOCK) {
        return CMD_STATUS_REJECTED;
    }
    if (mode == CONTROL_MODE_UDP_MANUAL) {
        return CMD_STATUS_OK;
    }
    if (mode != CONTROL_MODE_IDLE) {
        return CMD_STATUS_CONFLICT;
    }
    if (arm_state != ARM_STATE_DISARMED) {
        return CMD_STATUS_DISARM_REQUIRED;
    }

    runtime_state_set_motor_test(-1, 0.0f);
    runtime_state_set_axis_test_request((axis3f_t){0});
    runtime_state_set_rate_setpoint_request((axis3f_t){0});

    taskENTER_CRITICAL(&s_udp_manual_lock);
    udp_manual_reset_locked();
    s_udp_manual_state.enabled = true;
    s_udp_manual_state.stage = UDP_MANUAL_STAGE_IDLE;
    s_udp_manual_state.last_frame_us = (uint64_t)esp_timer_get_time();
    taskEXIT_CRITICAL(&s_udp_manual_lock);

    runtime_state_set_control_mode(CONTROL_MODE_UDP_MANUAL);
    console_send_event_text("udp manual enabled: experimental open-loop control");
    return CMD_STATUS_OK;
}

console_cmd_status_t udp_manual_disable(void)
{
    udp_manual_reset();
    runtime_state_set_motor_test(-1, 0.0f);
    runtime_state_set_axis_test_request((axis3f_t){0});
    runtime_state_set_rate_setpoint_request((axis3f_t){0});
    runtime_state_set_control_mode(CONTROL_MODE_IDLE);
    safety_request_disarm();
    motor_stop_all();
    console_send_event_text("udp manual disabled");
    return CMD_STATUS_OK;
}

console_cmd_status_t udp_manual_stop(void)
{
    udp_manual_reset();
    runtime_state_set_motor_test(-1, 0.0f);
    runtime_state_set_axis_test_request((axis3f_t){0});
    runtime_state_set_rate_setpoint_request((axis3f_t){0});
    runtime_state_set_control_mode(CONTROL_MODE_IDLE);
    safety_request_disarm();
    motor_stop_all();
    console_send_event_text("udp manual stop: motors commanded safe");
    return CMD_STATUS_OK;
}

console_cmd_status_t udp_manual_takeoff(void)
{
    if (runtime_state_get_control_mode() != CONTROL_MODE_UDP_MANUAL) {
        return CMD_STATUS_CONFLICT;
    }
    if (runtime_state_get_arm_state() == ARM_STATE_FAILSAFE ||
        runtime_state_get_arm_state() == ARM_STATE_FAULT_LOCK) {
        return CMD_STATUS_REJECTED;
    }

    taskENTER_CRITICAL(&s_udp_manual_lock);
    if (!s_udp_manual_state.enabled) {
        taskEXIT_CRITICAL(&s_udp_manual_lock);
        return CMD_STATUS_CONFLICT;
    }
    s_udp_manual_state.stage = UDP_MANUAL_STAGE_TAKEOFF;
    s_udp_manual_state.desired_axis = (axis3f_t){0};
    s_udp_manual_state.last_frame_us = (uint64_t)esp_timer_get_time();
    s_udp_manual_state.timeout_event_sent = false;
    s_udp_manual_state.disarm_event_sent = false;
    taskEXIT_CRITICAL(&s_udp_manual_lock);

    safety_request_arm(true);
    console_send_event_text("udp takeoff requested: open-loop ramp only");
    return CMD_STATUS_OK;
}

console_cmd_status_t udp_manual_land(void)
{
    if (runtime_state_get_control_mode() != CONTROL_MODE_UDP_MANUAL) {
        return CMD_STATUS_CONFLICT;
    }

    taskENTER_CRITICAL(&s_udp_manual_lock);
    if (!s_udp_manual_state.enabled) {
        taskEXIT_CRITICAL(&s_udp_manual_lock);
        return CMD_STATUS_CONFLICT;
    }
    s_udp_manual_state.stage = UDP_MANUAL_STAGE_LAND;
    s_udp_manual_state.desired_throttle = 0.0f;
    s_udp_manual_state.desired_axis = (axis3f_t){0};
    s_udp_manual_state.last_frame_us = (uint64_t)esp_timer_get_time();
    s_udp_manual_state.timeout_event_sent = false;
    taskEXIT_CRITICAL(&s_udp_manual_lock);

    console_send_event_text("udp land requested: open-loop ramp down");
    return CMD_STATUS_OK;
}

console_cmd_status_t udp_manual_setpoint(float throttle, float pitch, float roll, float yaw)
{
    if (!udp_manual_all_finite(throttle, pitch, roll, yaw)) {
        return CMD_STATUS_INVALID_ARGUMENT;
    }
    if (runtime_state_get_control_mode() != CONTROL_MODE_UDP_MANUAL) {
        return CMD_STATUS_CONFLICT;
    }

    const params_store_t *params = params_get();
    const float axis_limit = params->udp_manual_axis_limit;
    const float max_pwm = params->udp_manual_max_pwm;

    taskENTER_CRITICAL(&s_udp_manual_lock);
    if (!s_udp_manual_state.enabled) {
        taskEXIT_CRITICAL(&s_udp_manual_lock);
        return CMD_STATUS_CONFLICT;
    }
    s_udp_manual_state.desired_throttle = udp_manual_clampf(throttle, 0.0f, max_pwm);
    s_udp_manual_state.desired_axis.roll = udp_manual_clampf(roll, -axis_limit, axis_limit);
    s_udp_manual_state.desired_axis.pitch = udp_manual_clampf(pitch, -axis_limit, axis_limit);
    s_udp_manual_state.desired_axis.yaw = udp_manual_clampf(yaw, -axis_limit, axis_limit);
    s_udp_manual_state.stage = UDP_MANUAL_STAGE_MANUAL;
    s_udp_manual_state.last_frame_us = (uint64_t)esp_timer_get_time();
    s_udp_manual_state.timeout_event_sent = false;
    s_udp_manual_state.disarm_event_sent = false;
    taskEXIT_CRITICAL(&s_udp_manual_lock);
    return CMD_STATUS_OK;
}

bool udp_manual_get_control(uint64_t now_us, uint32_t loop_dt_us, udp_manual_control_t *out_control)
{
    if (out_control == NULL || runtime_state_get_control_mode() != CONTROL_MODE_UDP_MANUAL) {
        return false;
    }

    const params_store_t *params = params_get();
    const uint64_t timeout_us = (uint64_t)params->udp_manual_timeout_ms * 1000u;
    const uint64_t long_timeout_us = timeout_us * UDP_MANUAL_LONG_TIMEOUT_MULTIPLIER;
    const float max_pwm = params->udp_manual_max_pwm;
    const float takeoff_pwm = udp_manual_clampf(params->udp_takeoff_pwm, 0.0f, max_pwm);
    const float land_min_pwm = udp_manual_clampf(params->udp_land_min_pwm, 0.0f, max_pwm);
    const float dt_s = udp_manual_clampf((float)loop_dt_us / 1000000.0f, 0.001f, 0.050f);
    const float max_delta = UDP_MANUAL_RAMP_DUTY_PER_S * dt_s;
    const char *event_text = NULL;

    taskENTER_CRITICAL(&s_udp_manual_lock);
    if (!s_udp_manual_state.enabled) {
        taskEXIT_CRITICAL(&s_udp_manual_lock);
        return false;
    }

    float target_throttle = 0.0f;
    axis3f_t target_axis = s_udp_manual_state.desired_axis;
    bool timed_out = false;
    bool should_disarm = false;
    const uint64_t age_us = (now_us >= s_udp_manual_state.last_frame_us)
                                ? (now_us - s_udp_manual_state.last_frame_us)
                                : 0u;

    if (age_us > timeout_us) {
        timed_out = true;
        target_axis = (axis3f_t){0};
        target_throttle = land_min_pwm;
        if (!s_udp_manual_state.timeout_event_sent) {
            s_udp_manual_state.timeout_event_sent = true;
            event_text = "udp manual watchdog timeout: zero axes and reduce throttle";
        }
        if (age_us > long_timeout_us) {
            target_throttle = 0.0f;
            should_disarm = true;
        }
    } else if (s_udp_manual_state.stage == UDP_MANUAL_STAGE_TAKEOFF) {
        target_axis = (axis3f_t){0};
        target_throttle = takeoff_pwm;
    } else if (s_udp_manual_state.stage == UDP_MANUAL_STAGE_LAND) {
        target_axis = (axis3f_t){0};
        target_throttle = 0.0f;
        if (s_udp_manual_state.current_throttle <= (land_min_pwm + 0.001f)) {
            should_disarm = true;
        }
    } else if (s_udp_manual_state.stage == UDP_MANUAL_STAGE_MANUAL) {
        target_throttle = udp_manual_clampf(s_udp_manual_state.desired_throttle, 0.0f, max_pwm);
    }

    if (target_throttle > s_udp_manual_state.current_throttle + max_delta) {
        s_udp_manual_state.current_throttle += max_delta;
    } else if (target_throttle < s_udp_manual_state.current_throttle - max_delta) {
        s_udp_manual_state.current_throttle -= max_delta;
    } else {
        s_udp_manual_state.current_throttle = target_throttle;
    }
    s_udp_manual_state.current_throttle = udp_manual_clampf(s_udp_manual_state.current_throttle, 0.0f, max_pwm);

    if (should_disarm && !s_udp_manual_state.disarm_event_sent) {
        s_udp_manual_state.disarm_event_sent = true;
        event_text = timed_out ? "udp manual long timeout: auto disarm" : "udp manual landing complete: auto disarm";
    }

    *out_control = (udp_manual_control_t){
        .throttle = s_udp_manual_state.current_throttle,
        .axis = target_axis,
        .timed_out = timed_out,
        .should_disarm = should_disarm,
    };
    taskEXIT_CRITICAL(&s_udp_manual_lock);

    if (event_text != NULL) {
        console_send_event_text(event_text);
    }
    return true;
}
