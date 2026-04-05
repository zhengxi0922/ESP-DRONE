/**
 * @file mixer.h
 * @brief 四轴混控接口。
 */

#pragma once

#include <stdbool.h>

#include "esp_drone_types.h"

/**
 * @brief Mixer 输出的电机通道数。
 *
 * @note 当前实现固定适用于四轴机型。
 */
#define MIXER_MOTOR_COUNT 4

/**
 * @brief 一次混控计算的输入。
 */
typedef struct {
    float throttle; /**< 油门基值，推荐范围为 `0.0f` 到 `1.0f`。 */
    axis3f_t axis;  /**< 项目语义下的三轴控制量。 */
} mixer_input_t;

/**
 * @brief 单个电机的混控系数。
 *
 * @note 系数由板级位置与旋向推导，不应直接替换成旧仓库经验公式。
 */
typedef struct {
    float roll_coeff;  /**< Roll 轴系数。 */
    float pitch_coeff; /**< Pitch 轴系数。 */
    float yaw_coeff;   /**< Yaw 轴系数。 */
} mixer_coeffs_t;

/**
 * @brief 初始化 mixer 缓存。
 */
void mixer_init(void);

/**
 * @brief 构建当前板级映射下的混控系数。
 *
 * @param[out] out_coeffs 输出系数数组，长度必须为 `MIXER_MOTOR_COUNT`。
 * @retval true 系数构建成功。
 * @retval false 输出指针为空或板级配置不完整。
 */
bool mixer_build_coeffs(mixer_coeffs_t out_coeffs[MIXER_MOTOR_COUNT]);

/**
 * @brief 执行一次四轴混控。
 *
 * @param[in] coeffs 混控系数数组。
 * @param[in] input 本次混控输入。
 * @param[out] out_outputs 输出数组，长度必须为 `MIXER_MOTOR_COUNT`。
 * @note 每路输出都会被裁剪到 `0.0f` 到 `1.0f`。
 */
void mixer_mix(const mixer_coeffs_t coeffs[MIXER_MOTOR_COUNT], const mixer_input_t *input, float out_outputs[MIXER_MOTOR_COUNT]);

/**
 * @brief 执行混控方向自检。
 *
 * @retval true `+/-roll`、`+/-pitch`、`+/-yaw` 方向均与文档约定一致。
 * @retval false 当前系数与文档真值表不一致。
 */
bool mixer_self_test(void);
