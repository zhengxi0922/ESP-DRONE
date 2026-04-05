/**
 * @file led_status.h
 * @brief ESP-DRONE LED ??????
 * @details ?? LED ????????????
 * @author Codex
 * @date 2026-04-05
 * @version 1.0
 */

#pragma once

#include "esp_err.h"

#include "esp_drone_types.h"

/**
 * @brief 初始化 LED 状态机模块
 * @return esp_err_t ESP_OK 表示成功
 */
esp_err_t led_status_init(void);

/**
 * @brief 设置当前 LED 逻辑状态
 * @param state 逻辑状态枚举
 */
void led_status_set_state(led_state_t state);

/**
 * @brief 获取当前 LED 逻辑状态
 * @return led_state_t 当前状态
 */
led_state_t led_status_get_state(void);

/**
 * @brief 推进 LED 状态机输出
 * @param now_ms 当前系统时间，单位毫秒
 */
void led_status_service(uint32_t now_ms);
