/**
 * @file safety.h
 * @brief 安全状态机接口。
 */

#pragma once

#include <stdbool.h>

#include "esp_drone_types.h"
#include "imu.h"

/**
 * @brief 安全状态机输入。
 */
typedef struct {
    float throttle_norm;    /**< 归一化油门，范围通常为 `0.0f` 到 `1.0f`。 */
    bool rc_link_online;    /**< 遥控链路是否在线。 */
    float battery_voltage;  /**< 当前电池电压，单位为 V。 */
    imu_health_t imu_health; /**< IMU 健康状态。 */
    imu_stats_t imu_stats;   /**< IMU 运行统计快照。 */
    loop_stats_t loop_stats; /**< 控制循环统计快照。 */
} safety_inputs_t;

/**
 * @brief 安全状态机输出。
 */
typedef struct {
    arm_state_t arm_state;             /**< 当前 arm 状态。 */
    failsafe_reason_t failsafe_reason; /**< 当前 failsafe 原因。 */
    bool motors_should_stop;           /**< 为 `true` 时调用方必须停桨。 */
} safety_status_t;

/**
 * @brief 初始化安全状态机。
 */
void safety_init(void);

/**
 * @brief 请求进入 ARM 状态。
 *
 * @param[in] cli_override_allowed 是否允许在无 RC 链路时使用台架覆盖模式。
 * @retval true 请求已登记，最终是否解锁取决于下一次 `safety_update()`。
 * @retval false 当前处于 `ARM_STATE_FAULT_LOCK`，请求被拒绝。
 */
bool safety_request_arm(bool cli_override_allowed);

/**
 * @brief 请求上锁。
 */
void safety_request_disarm(void);

/**
 * @brief 请求急停。
 */
void safety_request_kill(void);

/**
 * @brief 用最新判据推进安全状态机。
 *
 * @param[in] inputs 输入判据，建议每个控制周期提供最新快照。
 * @param[out] out_status 输出状态缓存，可为 `NULL`。
 */
void safety_update(const safety_inputs_t *inputs, safety_status_t *out_status);
