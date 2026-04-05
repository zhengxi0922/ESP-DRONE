/**
 * @file runtime_state.h
 * @brief ESP-DRONE ????????
 * @details ????????????????console?safety ??????
 * @author Codex
 * @date 2026-04-05
 * @version 1.0
 */

#pragma once

#include "esp_drone_types.h"

/**
 * @brief 初始化运行时状态模块
 */
void runtime_state_init(void);

/**
 * @brief 设置 arm 状态快照
 */
void runtime_state_set_arm_state(arm_state_t state);

/**
 * @brief 获取 arm 状态快照
 */
arm_state_t runtime_state_get_arm_state(void);

/**
 * @brief 设置 failsafe 原因快照
 */
void runtime_state_set_failsafe_reason(failsafe_reason_t reason);

/**
 * @brief 获取 failsafe 原因快照
 */
failsafe_reason_t runtime_state_get_failsafe_reason(void);

/**
 * @brief 设置循环统计信息
 */
void runtime_state_set_loop_stats(loop_stats_t stats);

/**
 * @brief 获取循环统计信息
 */
loop_stats_t runtime_state_get_loop_stats(void);

/**
 * @brief 设置遥测流开关状态
 */
void runtime_state_set_stream_enabled(bool enabled);

/**
 * @brief 获取遥测流开关状态
 */
bool runtime_state_get_stream_enabled(void);

/**
 * @brief 设置电机点动测试请求
 */
void runtime_state_set_motor_test(int logical_motor, float duty);

/**
 * @brief 获取电机点动测试请求
 */
void runtime_state_get_motor_test(int *out_logical_motor, float *out_duty);

/**
 * @brief 设置当前控制模式
 */
void runtime_state_set_control_mode(control_mode_t mode);

/**
 * @brief 获取当前控制模式
 */
control_mode_t runtime_state_get_control_mode(void);

/**
 * @brief 设置 axis-test 请求
 */
void runtime_state_set_axis_test_request(axis3f_t request);

/**
 * @brief 获取 axis-test 请求
 */
axis3f_t runtime_state_get_axis_test_request(void);

/**
 * @brief 设置 rate-test 目标值
 */
void runtime_state_set_rate_setpoint_request(axis3f_t request);

/**
 * @brief 获取 rate-test 目标值
 */
axis3f_t runtime_state_get_rate_setpoint_request(void);
