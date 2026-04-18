#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "estimator.h"
#include "imu.h"

bool ground_tune_capture_reference(const imu_sample_t *sample, const estimator_state_t *estimator_state);
void ground_tune_clear_reference(void);
void ground_tune_reset_status(void);
void ground_tune_set_trip_reason(ground_tune_trip_reason_t reason);
void ground_tune_set_submode(ground_tune_submode_t submode);
void ground_tune_set_inner_clamp_flags(uint8_t flags);
bool ground_tune_set_angle_target(uint8_t axis_id, float target_deg);
bool ground_tune_compute(const estimator_state_t *estimator_state, axis3f_t *out_rate_setpoint_dps);
const char *ground_tune_trip_reason_text(ground_tune_trip_reason_t reason);
