/**
 * @file controller.h
 * @brief ESP-DRONE ??????
 * @details ????????????????????
 * @author Codex
 * @date 2026-04-05
 * @version 1.0
 */

#pragma once

#include <stdbool.h>

#include "esp_drone_types.h"

/**
 * @brief 速率环控制器输出状态
 * @details 保存三轴 P/I/D 分量和最终输出，供遥测与 bench 调试直接查看。
 */
typedef struct {
    axis3f_t p_term;
    axis3f_t i_term;
    axis3f_t d_term;
    axis3f_t output;
} rate_controller_status_t;

/**
 * @brief 初始化速率控制器
 */
void controller_init(void);

/**
 * @brief 重置速率控制器内部状态
 * @details 会清空积分项、上次误差等状态，常用于模式切换或重新解锁前。
 */
void controller_reset(void);

/**
 * @brief 更新一次速率环控制器
 * @param rate_setpoint_dps 目标角速度（项目 roll/pitch/yaw 语义）
 * @param measured_rate_dps 实测角速度
 * @param dt_s 控制周期，单位秒
 * @return rate_controller_status_t 本次更新后的 P/I/D 和总输出
 */
rate_controller_status_t controller_update_rate(const axis3f_t *rate_setpoint_dps, const axis3f_t *measured_rate_dps, float dt_s);

/**
 * @brief 获取上一次速率控制器状态
 * @return rate_controller_status_t 最近一次控制输出快照
 */
rate_controller_status_t controller_get_last_rate_status(void);

