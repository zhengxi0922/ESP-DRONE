/**
 * @file estimator.h
 * @brief 姿态与速率估计接口。
 */

#pragma once

#include <stdbool.h>

#include "esp_drone_types.h"

/**
 * @brief 估计器输出快照。
 *
 * @note 当前实现主要负责把 IMU 样本投影为项目语义下的速率与姿态。
 */
typedef struct {
    uint64_t timestamp_us;      /**< 样本时间戳，单位为 us。 */
    vec3f_t gyro_body_xyz_dps;  /**< 机体系角速度，单位为 deg/s。 */
    vec3f_t acc_body_xyz_g;     /**< 机体系加速度，单位为 g。 */
    axis3f_t rate_rpy_dps;      /**< 项目语义下的 roll/pitch/yaw 角速度，单位为 deg/s。 */
    eulerf_t attitude_rpy_deg;  /**< 项目语义下的姿态角，单位为 deg。 */
    quatf_t quat_body_to_world; /**< 机体系到世界系的四元数。 */
    bool attitude_valid;        /**< 当前姿态解是否可信。 */
} estimator_state_t;

/**
 * @brief 初始化估计器模块。
 */
void estimator_init(void);

/**
 * @brief 重置估计器内部状态。
 */
void estimator_reset(void);

/**
 * @brief 用一帧 IMU 样本更新估计状态。
 *
 * @param[in] sample 统一 IMU 样本，不能为空。
 * @param[out] out_state 输出状态缓存，不能为空。
 */
void estimator_update_from_imu(const imu_sample_t *sample, estimator_state_t *out_state);

/**
 * @brief 将机体系陀螺角速度投影为项目语义速率。
 *
 * @param[in] gyro_body_xyz_dps 机体系角速度，单位为 deg/s。
 * @return 项目语义下的 roll/pitch/yaw 角速度。
 * @note 当前项目约定 `+roll=-body_y`、`+pitch=+body_x`、`+yaw=-body_z`。
 */
axis3f_t estimator_project_rates_from_body_gyro(vec3f_t gyro_body_xyz_dps);
