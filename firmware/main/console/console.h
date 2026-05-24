/**
 * @file console.h
 * @brief USB CDC 控制台接口。
 */

#pragma once

#include "esp_err.h"

#include "esp_drone_types.h"

/**
 * @brief 初始化 USB CDC 控制台收发链路。
 *
 * @return `ESP_OK` 表示初始化成功。
 */
esp_err_t console_init(void);

/**
 * @brief 轮询控制台输入并分发协议消息。
 *
 * @note 应在常驻服务任务中周期调用。
 */
void console_service(void);

/**
 * @brief 发送一条文本事件日志。
 *
 * @param[in] text UTF-8 文本指针。
 * @note 文本会按协议上限截断；当日志开关关闭时该调用为空操作。
 */
void console_send_event_text(const char *text);

/**
 * @brief FreeRTOS 任务入口：异步执行 OTA 更新。
 *
 * @param[in] arg 指向 malloc 分配的 URL 字符串，任务负责释放。
 */
void console_ota_task(void *arg);

/**
 * @brief 通过 HTTP 下载固件并执行 OTA 升级。
 *
 * @param[in] url 固件二进制文件的 HTTP URL。
 * @return `ESP_OK` 表示下载和写入成功（随后会重启）。
 */
esp_err_t console_ota_update(const char *url);

/**
 * @brief 发送一帧遥测样本。
 *
 * @param[in] imu_sample IMU 样本，不能为空。
 * @param[in] baro_state 气压计状态快照，可为 `NULL`。
 * @param[in] battery_voltage 当前电池电压，单位为 V。
 * @param[in] battery_raw 当前电池 ADC 原始读数。
 * @note 仅当控制台已初始化且遥测流开关已打开时才会实际发送。
 */
void console_send_telemetry(const imu_sample_t *imu_sample,
                            uint32_t sample_seq,
                            const barometer_state_t *baro_state,
                            float battery_voltage,
                            int battery_raw);
