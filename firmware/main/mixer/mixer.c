#include "mixer.h"

#include <math.h>
#include <string.h>

#include "board_config.h"
#include "params.h"

static mixer_coeffs_t s_coeffs[MIXER_MOTOR_COUNT];

static float mixer_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void mixer_init(void)
{
    mixer_build_coeffs(s_coeffs);
}

bool mixer_build_coeffs(mixer_coeffs_t out_coeffs[MIXER_MOTOR_COUNT])
{
    if (out_coeffs == NULL) {
        return false;
    }

    const params_store_t *params = params_get();
    for (int i = 0; i < MIXER_MOTOR_COUNT; ++i) {
        const board_motor_config_t *cfg = board_get_motor_config((board_motor_id_t)i);
        if (cfg == NULL) {
            return false;
        }

        out_coeffs[i] = (mixer_coeffs_t){
            /* mixer 方向完全按项目机体系重新推导：
             * roll_coeff=-x, pitch_coeff=-y, yaw_coeff=CCW:+1 / CW:-1。
             * 这里不能偷用旧仓库或常见 +X 机头假设。 */
            .roll_coeff = (float)(-cfg->body_x_sign),
            .pitch_coeff = (float)(-cfg->body_y_sign),
            .yaw_coeff = params->motor_spin_is_cw[i] ? -1.0f : 1.0f,
        };
    }

    return true;
}

void mixer_mix(const mixer_coeffs_t coeffs[MIXER_MOTOR_COUNT], const mixer_input_t *input, float out_outputs[MIXER_MOTOR_COUNT])
{
    if (coeffs == NULL || input == NULL || out_outputs == NULL) {
        return;
    }

    for (int i = 0; i < MIXER_MOTOR_COUNT; ++i) {
        const float output =
            input->throttle +
            coeffs[i].roll_coeff * input->axis.roll +
            coeffs[i].pitch_coeff * input->axis.pitch +
            coeffs[i].yaw_coeff * input->axis.yaw;
        out_outputs[i] = mixer_clampf(output, 0.0f, 1.0f);
    }
}

static bool mixer_check_direction(const float outputs[MIXER_MOTOR_COUNT], int inc_a, int inc_b, int dec_a, int dec_b)
{
    return outputs[inc_a] > outputs[dec_a] &&
           outputs[inc_a] > outputs[dec_b] &&
           outputs[inc_b] > outputs[dec_a] &&
           outputs[inc_b] > outputs[dec_b];
}

bool mixer_self_test(void)
{
    mixer_coeffs_t coeffs[MIXER_MOTOR_COUNT] = {0};
    float outputs[MIXER_MOTOR_COUNT] = {0};
    const float base = 0.20f;
    const float cmd = 0.05f;

    if (!mixer_build_coeffs(coeffs)) {
        return false;
    }

    /* 下面 6 组自检直接对应 docs/motor_map.md 中的方向真值表。 */
    mixer_mix(coeffs, &(mixer_input_t){.throttle = base, .axis = {.roll = +cmd}}, outputs);
    if (!mixer_check_direction(outputs, 0, 3, 1, 2)) {
        return false;
    }

    mixer_mix(coeffs, &(mixer_input_t){.throttle = base, .axis = {.roll = -cmd}}, outputs);
    if (!mixer_check_direction(outputs, 1, 2, 0, 3)) {
        return false;
    }

    mixer_mix(coeffs, &(mixer_input_t){.throttle = base, .axis = {.pitch = +cmd}}, outputs);
    if (!mixer_check_direction(outputs, 2, 3, 0, 1)) {
        return false;
    }

    mixer_mix(coeffs, &(mixer_input_t){.throttle = base, .axis = {.pitch = -cmd}}, outputs);
    if (!mixer_check_direction(outputs, 0, 1, 2, 3)) {
        return false;
    }

    mixer_mix(coeffs, &(mixer_input_t){.throttle = base, .axis = {.yaw = +cmd}}, outputs);
    if (!mixer_check_direction(outputs, 0, 2, 1, 3)) {
        return false;
    }

    mixer_mix(coeffs, &(mixer_input_t){.throttle = base, .axis = {.yaw = -cmd}}, outputs);
    if (!mixer_check_direction(outputs, 1, 3, 0, 2)) {
        return false;
    }

    return true;
}
