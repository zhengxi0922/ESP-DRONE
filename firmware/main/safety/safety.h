/**
 * @file safety.h
 * @brief ESP-DRONE ????????
 * @details ???????????arm ???failsafe ???????
 * @author Codex
 * @date 2026-04-05
 * @version 1.0
 */

#pragma once

#include <stdbool.h>

#include "esp_drone_types.h"
#include "imu.h"

/**
 * @brief 安全状态机输入
 * @details 汇总油门、链路、电池、IMU 和循环时序等安全判据。
 */
typedef struct {
    float throttle_norm;
    bool rc_link_online;
    float battery_voltage;
    imu_health_t imu_health;
    imu_stats_t imu_stats;
    loop_stats_t loop_stats;
} safety_inputs_t;

/**
 * @brief 安全状态机输出
 * @details 输出 arm 状态、failsafe 原因以及当前是否必须停桨。
 */
typedef struct {
    arm_state_t arm_state;
    failsafe_reason_t failsafe_reason;
    bool motors_should_stop;
} safety_status_t;

/**
 * @brief 初始化安全状态机
 */
void safety_init(void);

/**
 * @brief 请求解锁
 * @param cli_override_allowed 是否允许 bench CLI override
 * @return bool true 表示请求已接受
 */
bool safety_request_arm(bool cli_override_allowed);

/**
 * @brief 请求上锁
 */
void safety_request_disarm(void);

/**
 * @brief 请求急停
 */
void safety_request_kill(void);

/**
 * @brief 更新安全状态机
 * @param inputs 输入判据
 * @param[out] out_status 输出状态
 */
void safety_update(const safety_inputs_t *inputs, safety_status_t *out_status);

