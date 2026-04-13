#pragma once

#include <stdbool.h>

#include "estimator.h"
#include "imu.h"

bool ground_tune_capture_reference(const imu_sample_t *sample, const estimator_state_t *estimator_state);
void ground_tune_clear_reference(void);
void ground_tune_reset_status(void);
void ground_tune_set_trip_reason(ground_tune_trip_reason_t reason);
bool ground_tune_compute(const estimator_state_t *estimator_state, axis3f_t *out_rate_setpoint_dps);
const char *ground_tune_trip_reason_text(ground_tune_trip_reason_t reason);
