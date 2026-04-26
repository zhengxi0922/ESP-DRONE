/**
 * @file motor.c
 * @brief 电机 PWM 输出实现。
 */

#include "motor.h"

#include <string.h>

#include "driver/ledc.h"

#include "board_config.h"
#include "params.h"

#define MOTOR_PWM_RESOLUTION LEDC_TIMER_8_BIT
#define MOTOR_PWM_MAX_COUNT ((1u << MOTOR_PWM_RESOLUTION) - 1u)

static ledc_channel_t s_channels[MOTOR_COUNT] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
};

static float s_last_outputs[MOTOR_COUNT];
static bool s_initialized;

static float motor_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static esp_err_t motor_configure_timer(uint32_t freq_hz)
{
    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .freq_hz = (int)freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    return ledc_timer_config(&timer_config);
}

esp_err_t motor_init(void)
{
    const params_store_t *params = params_get();
    esp_err_t err = motor_configure_timer(params->motor_pwm_freq_hz);
    if (err != ESP_OK) {
        return err;
    }

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        const board_motor_config_t *cfg = board_get_motor_config((board_motor_id_t)i);
        const ledc_channel_config_t ch = {
            .gpio_num = cfg->gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = s_channels[i],
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
            .flags.output_invert = 0,
        };
        err = ledc_channel_config(&ch);
        if (err != ESP_OK) {
            return err;
        }
        s_last_outputs[i] = 0.0f;
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t motor_reconfigure_from_params(void)
{
    return motor_configure_timer(params_get()->motor_pwm_freq_hz);
}

static void motor_write_single(int logical_motor, float normalized_duty)
{
    const uint8_t physical_motor = params_get()->motor_output_map[logical_motor];
    const uint32_t count = (uint32_t)(motor_clampf(normalized_duty, 0.0f, 1.0f) * (float)MOTOR_PWM_MAX_COUNT);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, s_channels[physical_motor], count);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, s_channels[physical_motor]);
    s_last_outputs[logical_motor] = motor_clampf(normalized_duty, 0.0f, 1.0f);
}

void motor_set_armed_outputs(const float normalized[MOTOR_COUNT], bool kill_override)
{
    const params_store_t *params = params_get();

    if (!s_initialized) {
        return;
    }

    if (kill_override) {
        motor_stop_all();
        return;
    }

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        float duty = normalized != NULL ? normalized[i] : 0.0f;
        if (duty > 0.0f) {
            duty = duty * params->motor_trim_scale[i] + params->motor_trim_offset[i];
        }
        duty = motor_clampf(duty, 0.0f, params->motor_max_duty);

        if (duty > 0.0f && duty < params->motor_idle_duty) {
            duty = params->motor_idle_duty;
        }

        if (s_last_outputs[i] <= 0.0f && duty > 0.0f && duty < params->motor_startup_boost_duty) {
            duty = params->motor_startup_boost_duty;
        }

        const float delta = duty - s_last_outputs[i];
        const float slew = params->motor_slew_limit_per_tick;
        if (delta > slew) {
            duty = s_last_outputs[i] + slew;
        } else if (delta < -slew) {
            duty = s_last_outputs[i] - slew;
        }

        motor_write_single(i, duty);
    }
}

void motor_set_test_output(uint8_t logical_motor, float duty)
{
    float outputs[MOTOR_COUNT] = {0};
    if (logical_motor < MOTOR_COUNT) {
        outputs[logical_motor] = motor_clampf(duty, 0.0f, 1.0f);
    }
    motor_set_armed_outputs(outputs, false);
}

void motor_stop_all(void)
{
    if (!s_initialized) {
        return;
    }

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        motor_write_single(i, 0.0f);
    }
}

void motor_get_outputs(float out_outputs[MOTOR_COUNT])
{
    if (out_outputs == NULL) {
        return;
    }
    memcpy(out_outputs, s_last_outputs, sizeof(s_last_outputs));
}
