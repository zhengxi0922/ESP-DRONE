/**
 * @file motor.h
 * @brief ESP-DRONE ???????
 * @details ?????????????????????????
 * @author Codex
 * @date 2026-04-05
 * @version 1.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define MOTOR_COUNT 4

/**
 * @brief 初始化电机输出模块
 * @return esp_err_t ESP_OK 表示成功
 */
esp_err_t motor_init(void);

/**
 * @brief 基于参数系统重新配置电机输出模块
 * @return esp_err_t ESP_OK 表示成功
 */
esp_err_t motor_reconfigure_from_params(void);

/**
 * @brief 输出闭环控制电机命令
 * @param normalized 四路 0~1 归一化输出
 * @param kill_override 为 true 时强制停桨
 */
void motor_set_armed_outputs(const float normalized[MOTOR_COUNT], bool kill_override);

/**
 * @brief 输出单电机点动测试命令
 * @param logical_motor 逻辑电机编号
 * @param duty 点动占空比，0~1
 */
void motor_set_test_output(uint8_t logical_motor, float duty);

/**
 * @brief 立即停止所有电机
 */
void motor_stop_all(void);

/**
 * @brief 获取最近一次电机输出值
 * @param[out] out_outputs 四路输出快照
 */
void motor_get_outputs(float out_outputs[MOTOR_COUNT]);
