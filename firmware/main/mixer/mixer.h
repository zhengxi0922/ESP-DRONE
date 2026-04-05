/**
 * @file mixer.h
 * @brief ESP-DRONE Mixer ???
 * @details ?????????????????????
 * @author Codex
 * @date 2026-04-05
 * @version 1.0
 */

#pragma once

#include <stdbool.h>

#include "esp_drone_types.h"

#define MIXER_MOTOR_COUNT 4

/**
 * @brief 混控输入
 * @details 包含油门基值和项目语义下的三轴控制量。
 */
typedef struct {
    float throttle;
    axis3f_t axis;
} mixer_input_t;

/**
 * @brief 单个电机的混控系数
 * @details 系数由物理位置和旋向推导，不能直接写死为旧仓库公式。
 */
typedef struct {
    float roll_coeff;
    float pitch_coeff;
    float yaw_coeff;
} mixer_coeffs_t;

/**
 * @brief 初始化 mixer 模块
 */
void mixer_init(void);

/**
 * @brief 构建当前参数和板级映射下的 mixer 系数
 * @param[out] out_coeffs 输出系数数组
 * @return bool true 表示构建成功
 */
bool mixer_build_coeffs(mixer_coeffs_t out_coeffs[MIXER_MOTOR_COUNT]);

/**
 * @brief 执行一次混控计算
 * @param coeffs 混控系数
 * @param input 本次混控输入
 * @param[out] out_outputs 归一化四电机输出
 */
void mixer_mix(const mixer_coeffs_t coeffs[MIXER_MOTOR_COUNT], const mixer_input_t *input, float out_outputs[MIXER_MOTOR_COUNT]);

/**
 * @brief 运行 mixer 方向自检
 * @return bool true 表示 `+/-roll`、`+/-pitch`、`+/-yaw` 方向均与文档一致
 */
bool mixer_self_test(void);

