/**
 * @file led_status.h
 * @brief LED 状态机接口。
 */

#pragma once

#include "esp_err.h"

#include "esp_drone_types.h"

/**
 * @brief 初始化 LED GPIO 与状态机输出。
 *
 * @return `ESP_OK` 表示初始化成功。
 */
esp_err_t led_status_init(void);

/**
 * @brief 设置当前 LED 逻辑状态。
 *
 * @param[in] state 逻辑状态枚举值。
 */
void led_status_set_state(led_state_t state);

/**
 * @brief 获取当前 LED 逻辑状态。
 *
 * @return 最近一次设置的逻辑状态。
 */
led_state_t led_status_get_state(void);

/**
 * @brief 按当前时间推进 LED 输出。
 *
 * @param[in] now_ms 单调递增系统时间，单位为 ms。
 */
void led_status_service(uint32_t now_ms);
