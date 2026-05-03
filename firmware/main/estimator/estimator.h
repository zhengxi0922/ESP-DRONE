#pragma once

#include <stdbool.h>

#include "esp_drone_types.h"

/**
 * @brief 控制回路使用的统一姿态源。
 *
 * @note 不同控制模式下统一通过该枚举选择姿态，避免分散判断。
 */
typedef enum {
    ESTIMATOR_ATTITUDE_SOURCE_RAW_MODULE = 0,      /**< 直接来自 IMU 模块输出。 */
    ESTIMATOR_ATTITUDE_SOURCE_FILTERED_RAW = 1,     /**< 经过低通滤波的 raw 姿态。 */
    ESTIMATOR_ATTITUDE_SOURCE_KALMAN_RP_RAW_YAW = 2, /**< Kalman roll/pitch + raw yaw。 */
} estimator_attitude_source_t;

typedef struct {
    uint64_t timestamp_us;

    /* Backward-compatible raw aliases used by existing rate/hang/UDP paths. */
    vec3f_t gyro_body_xyz_dps;
    vec3f_t acc_body_xyz_g;
    axis3f_t rate_rpy_dps;
    eulerf_t attitude_rpy_deg;
    quatf_t quat_body_to_world;
    bool attitude_valid;

    vec3f_t raw_gyro_body_xyz_dps;
    vec3f_t filtered_gyro_body_xyz_dps;
    vec3f_t raw_acc_body_xyz_g;
    vec3f_t filtered_acc_body_xyz_g;
    axis3f_t raw_rate_rpy_dps;
    axis3f_t filtered_rate_rpy_dps;
    eulerf_t raw_attitude_rpy_deg;
    quatf_t raw_quat_body_to_world;
    float kalman_roll_deg;
    float kalman_pitch_deg;
    bool kalman_valid;
} estimator_state_t;

void estimator_init(void);
void estimator_reset(void);
void estimator_update_from_imu(const imu_sample_t *sample, estimator_state_t *out_state);
bool estimator_get_latest(estimator_state_t *out_state);

axis3f_t estimator_project_rates_from_body_gyro(vec3f_t gyro_body_xyz_dps);

/**
 * @brief 根据指定源获取控制回路使用的姿态。
 *
 * @param[in] state 当前 estimator 状态快照。
 * @param[in] source 姿态源选择。
 * @param[out] attitude_rpy_deg 输出的控制用欧拉角 (roll/pitch/yaw, deg)。可为 NULL。
 * @param[out] quat_body_to_world 输出的控制用四元数。可为 NULL。
 * @param[out] attitude_valid 姿态有效性标志。可为 NULL。
 * @retval true  成功获取姿态。
 * @retval false 姿态源当前无效，输出未定义。
 *
 * @note 默认 control-loop 路径应调用本函数而不是直接读取 state 内部字段。
 *       yaw 当前始终来自 IMU 模块输出，Kalman 只覆盖 roll/pitch。
 */
bool estimator_get_control_attitude(const estimator_state_t *state,
                                    estimator_attitude_source_t source,
                                    eulerf_t *attitude_rpy_deg,
                                    quatf_t *quat_body_to_world,
                                    bool *attitude_valid);
