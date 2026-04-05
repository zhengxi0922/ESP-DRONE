/**
 * @file imu.h
 * @brief IMU 驱动与样本发布接口。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "esp_drone_types.h"

/**
 * @brief IMU 运行统计信息。
 *
 * @note 主要用于遥测、故障诊断和安全状态机判定。
 */
typedef struct {
    uint32_t published_seq;            /**< 已发布样本序号。 */
    uint32_t good_frames;              /**< 成功解析并接收的帧数。 */
    uint32_t parse_errors;             /**< 累计解析错误数。 */
    uint32_t consecutive_parse_errors; /**< 连续解析错误数。 */
    uint64_t last_frame_us;            /**< 最近一次有效帧时间戳，单位为 us。 */
} imu_stats_t;

/**
 * @brief 初始化 IMU 串口驱动并装载当前参数配置。
 *
 * @return `ESP_OK` 表示成功。
 * @note 依赖 `board_config_get()` 提供 UART 端口与引脚定义。
 */
esp_err_t imu_init(void);

/**
 * @brief 根据当前参数重新写入 IMU 回传配置。
 *
 * @return `ESP_OK` 表示成功，未初始化时返回 `ESP_ERR_INVALID_STATE`。
 * @note 当前实现只更新模块寄存器，不重建 UART 驱动。
 */
esp_err_t imu_reconfigure_from_params(void);

/**
 * @brief 采集当前角速度并累加到陀螺零偏补偿。
 *
 * @return `ESP_OK` 表示成功，当前样本无效时返回 `ESP_ERR_INVALID_STATE`。
 * @warning 该实现每调用一次都会继续累加补偿量，而不是自动求平均。
 */
esp_err_t imu_calibrate_gyro(void);

/**
 * @brief 采集当前姿态并累加到水平修正量。
 *
 * @return `ESP_OK` 表示成功，当前姿态无效时返回 `ESP_ERR_INVALID_STATE`。
 * @note 当前实现只修正 roll/pitch，yaw 修正始终清零。
 */
esp_err_t imu_calibrate_level(void);

/**
 * @brief 轮询串口接收并推进协议解析。
 *
 * @note 该函数应在高优先级常驻任务中持续调用，以维持样本健康状态刷新。
 */
void imu_service_rx(void);

/**
 * @brief 获取最新 IMU 样本。
 *
 * @param[out] out_sample 输出样本缓存，不能为空。
 * @param[out] out_seq 输出样本序号，可为 `NULL`。
 * @retval true 成功返回快照。
 * @retval false 驱动未初始化或 `out_sample` 为空。
 */
bool imu_get_latest(imu_sample_t *out_sample, uint32_t *out_seq);

/**
 * @brief 获取 IMU 统计快照。
 *
 * @return 当前统计信息。
 */
imu_stats_t imu_get_stats(void);

/**
 * @brief 将 IMU 健康状态转换为只读字符串。
 *
 * @param[in] health 健康状态枚举值。
 * @return 对应的静态字符串；未知值返回 `"UNKNOWN"`。
 */
const char *imu_health_to_string(imu_health_t health);
