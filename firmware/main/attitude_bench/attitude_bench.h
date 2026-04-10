#pragma once

#include <stdbool.h>

#include "estimator.h"
#include "imu.h"

bool attitude_bench_capture_reference(const imu_sample_t *sample);
void attitude_bench_clear_reference(void);
void attitude_bench_reset_status(void);
bool attitude_bench_compute(const estimator_state_t *estimator_state, axis3f_t *out_rate_setpoint_dps);
