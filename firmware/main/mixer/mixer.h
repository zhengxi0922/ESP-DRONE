#pragma once

#include <stdbool.h>

#include "esp_drone_types.h"

#define MIXER_MOTOR_COUNT 4

typedef struct {
    float throttle;
    axis3f_t axis;
} mixer_input_t;

typedef struct {
    float roll_coeff;
    float pitch_coeff;
    float yaw_coeff;
} mixer_coeffs_t;

void mixer_init(void);
bool mixer_build_coeffs(mixer_coeffs_t out_coeffs[MIXER_MOTOR_COUNT]);
void mixer_mix(const mixer_coeffs_t coeffs[MIXER_MOTOR_COUNT], const mixer_input_t *input, float out_outputs[MIXER_MOTOR_COUNT]);
bool mixer_self_test(void);

