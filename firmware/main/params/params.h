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

typedef enum {
    PARAM_TYPE_BOOL = 0,
    PARAM_TYPE_U8 = 1,
    PARAM_TYPE_U32 = 2,
    PARAM_TYPE_I32 = 3,
    PARAM_TYPE_FLOAT = 4,
} param_type_t;

typedef union {
    bool b;
    uint8_t u8;
    uint32_t u32;
    int32_t i32;
    float f32;
} param_value_t;

typedef struct {
    const char *name;
    param_type_t type;
    size_t offset;
} param_descriptor_t;

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

esp_err_t params_init(void);
const params_store_t *params_get(void);
void params_reset_to_defaults(void);
esp_err_t params_save(void);

size_t params_count(void);
const param_descriptor_t *params_get_descriptor(size_t index);

bool params_try_get(const char *name, param_value_t *out_value, param_type_t *out_type);
bool params_try_set(const char *name, param_value_t value, param_type_t type);
