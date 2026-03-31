#pragma once

#include <stdbool.h>

#include "esp_drone_types.h"
#include "imu.h"

typedef struct {
    float throttle_norm;
    bool rc_link_online;
    float battery_voltage;
    imu_health_t imu_health;
    imu_stats_t imu_stats;
    loop_stats_t loop_stats;
} safety_inputs_t;

typedef struct {
    arm_state_t arm_state;
    failsafe_reason_t failsafe_reason;
    bool motors_should_stop;
} safety_status_t;

void safety_init(void);
bool safety_request_arm(bool cli_override_allowed);
void safety_request_disarm(void);
void safety_request_kill(void);
void safety_update(const safety_inputs_t *inputs, safety_status_t *out_status);

