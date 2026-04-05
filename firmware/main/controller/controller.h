/**
 * @file controller.h
 * @brief 速率环控制器接口。
 */

#pragma once

#include <stdbool.h>

#include "esp_drone_types.h"

/**
 * @brief 一次速率环更新后的输出快照。
 *
 * @note 各分量均为送往 mixer 的归一化控制量，不是最终 PWM 占空比。
 */
typedef struct {
    axis3f_t p_term; /**< P 项输出。 */
    axis3f_t i_term; /**< I 项输出。 */
    axis3f_t d_term; /**< D 项输出。 */
    axis3f_t output; /**< 三轴总输出。 */
} rate_controller_status_t;

/**
 * @brief 初始化速率环控制器状态。
 */
void controller_init(void);

/**
 * @brief 重置速率环内部状态。
 *
 * @note 会清空积分项、历史误差和上次输出快照。
 */
void controller_reset(void);

/**
 * @brief 执行一次三轴速率环更新。
 *
 * @param[in] rate_setpoint_dps 目标角速度，单位为 deg/s。
 * @param[in] measured_rate_dps 实测角速度，单位为 deg/s。
 * @param[in] dt_s 控制周期，单位为 s，必须大于 0。
 * @return 本次更新后的 P/I/D 分量和总输出。
 * @note 当输入指针为空或 `dt_s <= 0` 时，返回上一次有效输出快照。
 */
rate_controller_status_t controller_update_rate(const axis3f_t *rate_setpoint_dps, const axis3f_t *measured_rate_dps, float dt_s);

/**
 * @brief 获取最近一次速率环输出快照。
 *
 * @return 最近一次有效更新后的控制器状态。
 */
rate_controller_status_t controller_get_last_rate_status(void);
