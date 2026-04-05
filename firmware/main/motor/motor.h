/**
 * @file motor.h
 * @brief 电机输出接口。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/**
 * @brief 电机输出通道数量。
 *
 * @note 当前实现固定适用于四轴机型。
 */
#define MOTOR_COUNT 4

/**
 * @brief 初始化 PWM 定时器与四路电机通道。
 *
 * @return `ESP_OK` 表示初始化成功。
 */
esp_err_t motor_init(void);

/**
 * @brief 根据参数重新配置电机 PWM 频率。
 *
 * @return `ESP_OK` 表示重配置成功。
 */
esp_err_t motor_reconfigure_from_params(void);

/**
 * @brief 输出闭环控制电机命令。
 *
 * @param[in] normalized 四路归一化输出，范围建议为 `0.0f` 到 `1.0f`；
 *                      传入 `NULL` 时等效为全零。
 * @param[in] kill_override 为 `true` 时立即停桨。
 * @note 当前实现会按参数施加最大占空比、空转下限、起转提升和斜率限制。
 */
void motor_set_armed_outputs(const float normalized[MOTOR_COUNT], bool kill_override);

/**
 * @brief 输出单电机点动测试命令。
 *
 * @param[in] logical_motor 逻辑电机编号，范围为 `0` 到 `MOTOR_COUNT - 1`。
 * @param[in] duty 点动占空比，范围建议为 `0.0f` 到 `1.0f`。
 * @note 非法编号会导致四路输出均为零。
 */
void motor_set_test_output(uint8_t logical_motor, float duty);

/**
 * @brief 立即停止所有电机。
 */
void motor_stop_all(void);

/**
 * @brief 获取最近一次逻辑电机输出值。
 *
 * @param[out] out_outputs 输出数组，长度必须为 `MOTOR_COUNT`。
 */
void motor_get_outputs(float out_outputs[MOTOR_COUNT]);
