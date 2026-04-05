/**
 * @file barometer.h
 * @brief 气压计状态接口。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "esp_drone_types.h"

/**
 * @brief 初始化气压计状态缓存。
 *
 * @return `ESP_OK` 表示状态已复位完成。
 */
esp_err_t barometer_init(void);

/**
 * @brief 用一帧模块气压计数据更新内部快照。
 *
 * @param[in] pressure_pa 模块原始气压，单位为 Pa。
 * @param[in] altitude_cm 模块原始高度，单位为 cm。
 * @param[in] temperature_c 模块原始温度，单位为摄氏度。
 * @param[in] timestamp_us 单调递增时间戳，单位为 us。
 * @note 当前实现会在相邻时间戳相差 `5 ms` 到 `1 s` 之间时估算垂向速度。
 */
void barometer_update_from_module_frame(int32_t pressure_pa,
                                        int32_t altitude_cm,
                                        float temperature_c,
                                        uint64_t timestamp_us);

/**
 * @brief 获取最新气压计状态快照。
 *
 * @param[out] out_state 输出状态缓存，不能为空。
 * @retval true 成功返回快照。
 * @retval false 模块未初始化或 `out_state` 为空。
 * @note 即使尚未收到任何气压计数据，也会返回 `true`，此时 `health` 为
 *       `BARO_HEALTH_INIT`。
 */
bool barometer_get_latest(barometer_state_t *out_state);

/**
 * @brief 获取定高闭环预留状态快照。
 *
 * @param[out] out_state 输出状态缓存，不能为空。
 * @retval true 成功返回快照。
 * @retval false 模块未初始化或 `out_state` 为空。
 * @note 当前实现只复用最近一次气压计高度与垂向速度，不直接驱动控制环。
 */
bool barometer_get_altitude_hold_reserved_state(altitude_hold_reserved_state_t *out_state);

/**
 * @brief 将气压计健康状态转换为只读字符串。
 *
 * @param[in] health 健康状态枚举值。
 * @return 对应的静态字符串；未知值返回 `"UNKNOWN"`。
 */
const char *barometer_health_to_string(baro_health_t health);
