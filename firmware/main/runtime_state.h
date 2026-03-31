#pragma once

#include "esp_drone_types.h"

void runtime_state_init(void);

void runtime_state_set_arm_state(arm_state_t state);
arm_state_t runtime_state_get_arm_state(void);

void runtime_state_set_failsafe_reason(failsafe_reason_t reason);
failsafe_reason_t runtime_state_get_failsafe_reason(void);

void runtime_state_set_loop_stats(loop_stats_t stats);
loop_stats_t runtime_state_get_loop_stats(void);

void runtime_state_set_stream_enabled(bool enabled);
bool runtime_state_get_stream_enabled(void);

void runtime_state_set_motor_test(int logical_motor, float duty);
void runtime_state_get_motor_test(int *out_logical_motor, float *out_duty);
