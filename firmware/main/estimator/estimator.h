/**
 * @file estimator.h
 * @brief ESP-DRONE ???????
 * @details ?????????? IMU ???????
 * @author Codex
 * @date 2026-04-05
 * @version 1.0
 */

#pragma once

#include <stdbool.h>

#include "esp_drone_types.h"

/**
 * @brief 估计器输出状态
 * @details 当前阶段主要为 rate-loop 服务，保存时间戳、机体系陀螺/加速度、项目语义 rate 和姿态。
 */
typedef struct {
    uint64_t timestamp_us;
    vec3f_t gyro_body_xyz_dps;
    vec3f_t acc_body_xyz_g;
    axis3f_t rate_rpy_dps;
    eulerf_t attitude_rpy_deg;
    quatf_t quat_body_to_world;
    bool attitude_valid;
} estimator_state_t;

/**
 * @brief 初始化估计器模块
 */
void estimator_init(void);

/**
 * @brief 重置估计器内部状态
 */
void estimator_reset(void);

/**
 * @brief 基于 IMU 样本更新估计器状态
 * @param sample 统一 IMU 样本
 * @param[out] out_state 输出估计状态
 */
void estimator_update_from_imu(const imu_sample_t *sample, estimator_state_t *out_state);

/**
 * @brief 将机体系陀螺角速度投影为项目 roll/pitch/yaw 速率
 * @param gyro_body_xyz_dps 机体系角速度
 * @return axis3f_t 项目语义下的三轴角速度
 */
axis3f_t estimator_project_rates_from_body_gyro(vec3f_t gyro_body_xyz_dps);

