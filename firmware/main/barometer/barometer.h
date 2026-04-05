/**
 * @file barometer.h
 * @brief ESP-DRONE 气压计状态框架
 * @details 当前阶段只负责保存、滤波、转换和输出 ATK-MS901M 的气压计数据，
 *          不参与现有姿态 / 速率控制或油门闭环。
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
 * @brief 初始化气压计状态模块
 * @return esp_err_t ESP_OK 表示成功
 */
esp_err_t barometer_init(void);

/**
 * @brief 用模块 baro 帧更新最新气压计状态
 * @param pressure_pa 模块上报的原始气压值，单位 Pa
 * @param altitude_cm 模块上报的高度值，单位 cm
 * @param temperature_c 模块上报的温度值，单位摄氏度
 * @param timestamp_us 接收时间戳，单位 us
 */
void barometer_update_from_module_frame(int32_t pressure_pa,
                                        int32_t altitude_cm,
                                        float temperature_c,
                                        uint64_t timestamp_us);

/**
 * @brief 获取最新气压计状态快照
 * @param[out] out_state 输出状态
 * @return bool true 表示成功获取
 */
bool barometer_get_latest(barometer_state_t *out_state);

/**
 * @brief 获取未来定高闭环的预留状态
 * @param[out] out_state 输出预留状态
 * @return bool true 表示成功获取
 */
bool barometer_get_altitude_hold_reserved_state(altitude_hold_reserved_state_t *out_state);

/**
 * @brief 将气压计健康状态枚举转为字符串
 * @param health 健康状态
 * @return const char* 只读字符串
 */
const char *barometer_health_to_string(baro_health_t health);
