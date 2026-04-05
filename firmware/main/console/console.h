/**
 * @file console.h
 * @brief ESP-DRONE ?????????
 * @details ????????????????????????
 * @author Codex
 * @date 2026-04-05
 * @version 1.0
 */

#pragma once

#include "esp_err.h"

#include "esp_drone_types.h"

/**
 * @brief 初始化 USB CDC 二进制控制台
 * @return esp_err_t ESP_OK 表示初始化成功
 */
esp_err_t console_init(void);

/**
 * @brief 轮询处理控制台收发
 * @details 在 service/telemetry 路径中周期调用，用于驱动协议接收和命令分发。
 */
void console_service(void);

/**
 * @brief 发送一条文本事件日志
 * @param text UTF-8 文本内容
 */
void console_send_event_text(const char *text);

/**
 * @brief 发送当前遥测样本
 * @param imu_sample IMU 统一样本
 * @param baro_state 气压计状态快照
 * @param battery_voltage 当前电池电压
 * @param battery_raw 当前电池 ADC 原始值
 */
void console_send_telemetry(const imu_sample_t *imu_sample,
                            const barometer_state_t *baro_state,
                            float battery_voltage,
                            int battery_raw);
