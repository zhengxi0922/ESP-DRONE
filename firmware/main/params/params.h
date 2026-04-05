/**
 * @file params.h
 * @brief ESP-DRONE ???????
 * @details ????????????????????????
 * @author Codex
 * @date 2026-04-05
 * @version 1.0
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "esp_drone_types.h"

#define PARAMS_NAMESPACE "esp_drone"
#define PARAMS_BLOB_KEY "registry_v1"
#define PARAMS_BLOB_MAGIC 0x31504445u
#define PARAMS_SCHEMA_VERSION 1u

/**
 * @brief 参数值类型
 * @details Python 工具链与固件在参数交互时共享这一类型编号。
 */
typedef enum {
    PARAM_TYPE_BOOL = 0,
    PARAM_TYPE_U8 = 1,
    PARAM_TYPE_U32 = 2,
    PARAM_TYPE_I32 = 3,
    PARAM_TYPE_FLOAT = 4,
} param_type_t;

/**
 * @brief 通用参数值联合体
 */
typedef union {
    bool b;
    uint8_t u8;
    uint32_t u32;
    int32_t i32;
    float f32;
} param_value_t;

/**
 * @brief 单个参数的描述信息
 * @details 用于参数列表枚举、CLI/GUI 展示和运行时读写。
 */
typedef struct {
    const char *name;
    param_type_t type;
    size_t offset;
} param_descriptor_t;

/**
 * @brief 全量参数存储结构
 * @details 集中保存电机、IMU、遥测、电池和 rate PID 等关键参数默认值与运行值。
 */
typedef struct {
    uint32_t motor_pwm_freq_hz;
    float motor_idle_duty;
    float motor_max_duty;
    float motor_startup_boost_duty;
    uint32_t motor_startup_boost_ms;
    float motor_slew_limit_per_tick;
    float bringup_test_base_duty;

    uint32_t rc_timeout_ms;
    uint32_t imu_timeout_ms;
    uint32_t imu_parse_error_limit;
    uint32_t loop_overrun_limit;

    float battery_warn_v;
    float battery_limit_v;
    float battery_critical_v;
    float battery_arm_v;

    uint32_t telemetry_usb_hz;
    uint32_t telemetry_udp_hz;
    uint32_t ring_buffer_seconds;

    imu_mode_t imu_mode;
    uint32_t imu_return_rate_code;
    body_axis_selector_t imu_map_x;
    body_axis_selector_t imu_map_y;
    body_axis_selector_t imu_map_z;
    bool imu_mag_enable;

    uint8_t motor_output_map[4];
    bool motor_spin_is_cw[4];

    float rate_kp_roll;
    float rate_ki_roll;
    float rate_kd_roll;
    float rate_kp_pitch;
    float rate_ki_pitch;
    float rate_kd_pitch;
    float rate_kp_yaw;
    float rate_ki_yaw;
    float rate_kd_yaw;
    float rate_integral_limit;
    float rate_output_limit;

    bool log_event_text_enabled;
} params_store_t;

/**
 * @brief 初始化参数系统
 * @return esp_err_t ESP_OK 表示成功
 */
esp_err_t params_init(void);

/**
 * @brief 获取当前参数只读指针
 * @return const params_store_t* 参数存储结构指针
 */
const params_store_t *params_get(void);

/**
 * @brief 恢复默认参数
 */
void params_reset_to_defaults(void);

/**
 * @brief 将当前参数保存到 NVS
 * @return esp_err_t ESP_OK 表示成功
 */
esp_err_t params_save(void);

/**
 * @brief 获取参数总数
 * @return size_t 参数条目数量
 */
size_t params_count(void);

/**
 * @brief 按索引获取参数描述
 * @param index 参数索引
 * @return const param_descriptor_t* 描述指针；失败返回 NULL
 */
const param_descriptor_t *params_get_descriptor(size_t index);

/**
 * @brief 按名称读取参数值
 * @param name 参数名
 * @param[out] out_value 输出参数值
 * @param[out] out_type 输出参数类型
 * @return bool true 表示成功
 */
bool params_try_get(const char *name, param_value_t *out_value, param_type_t *out_type);

/**
 * @brief 按名称写入参数值
 * @details 该接口会执行完整的范围/组合合法性校验，非法值不会写入。
 * @param name 参数名
 * @param value 待写入参数值
 * @param type 参数类型
 * @return bool true 表示写入成功
 */
bool params_try_set(const char *name, param_value_t value, param_type_t type);
