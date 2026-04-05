/**
 * @file imu.h
 * @brief ESP-DRONE IMU ?????
 * @details ?? IMU ???????????????????
 * @author Codex
 * @date 2026-04-05
 * @version 1.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "esp_drone_types.h"

/**
 * @brief IMU 运行统计信息
 * @details 主要用于遥测、故障诊断和安全状态机中的超时/解析错误判定。
 */
typedef struct {
    uint32_t published_seq;
    uint32_t good_frames;
    uint32_t parse_errors;
    uint32_t consecutive_parse_errors;
    uint64_t last_frame_us;
} imu_stats_t;

/**
 * @brief 初始化 IMU 驱动
 * @return esp_err_t ESP_OK 表示成功
 */
esp_err_t imu_init(void);

/**
 * @brief 根据参数系统重配置 IMU 模式和回传内容
 * @return esp_err_t ESP_OK 表示成功
 */
esp_err_t imu_reconfigure_from_params(void);

/**
 * @brief 执行陀螺仪零偏校准
 * @return esp_err_t ESP_OK 表示成功
 */
esp_err_t imu_calibrate_gyro(void);

/**
 * @brief 执行水平校准
 * @return esp_err_t ESP_OK 表示成功
 */
esp_err_t imu_calibrate_level(void);

/**
 * @brief 轮询串口接收并推进协议解析
 * @details 该函数通常在高优先级 IMU 接收任务中持续调用。
 */
void imu_service_rx(void);

/**
 * @brief 获取最新 IMU 样本
 * @param[out] out_sample 输出样本
 * @param[out] out_seq 输出样本序号
 * @return bool true 表示获取成功
 */
bool imu_get_latest(imu_sample_t *out_sample, uint32_t *out_seq);

/**
 * @brief 获取 IMU 统计信息
 * @return imu_stats_t 当前统计快照
 */
imu_stats_t imu_get_stats(void);

/**
 * @brief 将 IMU 健康状态枚举转换为字符串
 * @param health 健康状态枚举
 * @return const char* 只读字符串
 */
const char *imu_health_to_string(imu_health_t health);
