#pragma once

#include <stdbool.h>

#include "esp_drone_types.h"

typedef struct {
    axis3f_t p_term;
    axis3f_t i_term;
    axis3f_t d_term;
    axis3f_t output;
    axis3f_t error;
    axis3f_t measured_rate_dps;
    bool motor_saturation;
    bool integrator_freeze;
} rate_controller_status_t;

void controller_init(void);
void controller_reset(void);
void controller_set_runtime_flags(bool armed,
                                  float base_duty,
                                  bool motor_saturated,
                                  bool trip_or_failsafe);
rate_controller_status_t controller_update_rate(const axis3f_t *rate_setpoint_dps,
                                                const axis3f_t *measured_rate_dps,
                                                float dt_s);
rate_controller_status_t controller_get_last_rate_status(void);
