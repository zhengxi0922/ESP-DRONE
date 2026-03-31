#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define MOTOR_COUNT 4

esp_err_t motor_init(void);
esp_err_t motor_reconfigure_from_params(void);

void motor_set_armed_outputs(const float normalized[MOTOR_COUNT], bool kill_override);
void motor_set_test_output(uint8_t logical_motor, float duty);
void motor_stop_all(void);
void motor_get_outputs(float out_outputs[MOTOR_COUNT]);
