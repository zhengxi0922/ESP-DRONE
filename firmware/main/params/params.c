/**
 * @file params.c
 * @brief 参数默认值、校验与持久化实现。
 */

#include "params.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "nvs.h"
#include "nvs_flash.h"
#include "runtime_state.h"

typedef struct {
    uint32_t magic;
    uint32_t schema_version;
    uint32_t payload_len;
    uint32_t crc32;
} params_blob_header_t;

typedef struct {
    params_blob_header_t header;
    params_store_t payload;
} params_blob_t;

static params_store_t s_params;

#define PARAMS_RATE_KP_ROLL_DEFAULT 0.0007f
#define PARAMS_RATE_KI_ROLL_DEFAULT 0.0f
#define PARAMS_RATE_KD_ROLL_DEFAULT 0.0f
#define PARAMS_RATE_KP_PITCH_DEFAULT 0.0007f
#define PARAMS_RATE_KI_PITCH_DEFAULT 0.0f
#define PARAMS_RATE_KD_PITCH_DEFAULT 0.0f
#define PARAMS_RATE_KP_YAW_DEFAULT 0.0007f
#define PARAMS_RATE_KI_YAW_DEFAULT 0.0f
#define PARAMS_RATE_KD_YAW_DEFAULT 0.0f
#define PARAMS_SCHEMA_VERSION_BEFORE_MOTOR_REMAP 4u
#define PARAMS_SCHEMA_VERSION_BEFORE_GROUND_TUNE 5u
#define PARAMS_SCHEMA_VERSION_BEFORE_RATE_FIRST_GROUND_TUNE 6u

/* 参数系统采用“单 blob + schema_version + CRC32”保存策略。
 * 运行时所有参数写入都必须先过 params_try_set() 的合法性校验。 */
static uint32_t params_crc32(const void *data, size_t len)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;

    for (size_t i = 0; i < len; ++i) {
        crc ^= bytes[i];
        for (int bit = 0; bit < 8; ++bit) {
            const uint32_t mask = -(crc & 1u);
            crc = (crc >> 1u) ^ (0xEDB88320u & mask);
        }
    }

    return ~crc;
}

static const param_descriptor_t s_param_descs[] = {
    {"motor_pwm_freq_hz", PARAM_TYPE_U32, offsetof(params_store_t, motor_pwm_freq_hz)},
    {"motor_idle_duty", PARAM_TYPE_FLOAT, offsetof(params_store_t, motor_idle_duty)},
    {"motor_max_duty", PARAM_TYPE_FLOAT, offsetof(params_store_t, motor_max_duty)},
    {"motor_startup_boost_duty", PARAM_TYPE_FLOAT, offsetof(params_store_t, motor_startup_boost_duty)},
    {"motor_startup_boost_ms", PARAM_TYPE_U32, offsetof(params_store_t, motor_startup_boost_ms)},
    {"motor_slew_limit_per_tick", PARAM_TYPE_FLOAT, offsetof(params_store_t, motor_slew_limit_per_tick)},
    {"bringup_test_base_duty", PARAM_TYPE_FLOAT, offsetof(params_store_t, bringup_test_base_duty)},
    {"udp_manual_max_pwm", PARAM_TYPE_FLOAT, offsetof(params_store_t, udp_manual_max_pwm)},
    {"udp_takeoff_pwm", PARAM_TYPE_FLOAT, offsetof(params_store_t, udp_takeoff_pwm)},
    {"udp_land_min_pwm", PARAM_TYPE_FLOAT, offsetof(params_store_t, udp_land_min_pwm)},
    {"udp_manual_timeout_ms", PARAM_TYPE_U32, offsetof(params_store_t, udp_manual_timeout_ms)},
    {"udp_manual_axis_limit", PARAM_TYPE_FLOAT, offsetof(params_store_t, udp_manual_axis_limit)},
    {"rc_timeout_ms", PARAM_TYPE_U32, offsetof(params_store_t, rc_timeout_ms)},
    {"imu_timeout_ms", PARAM_TYPE_U32, offsetof(params_store_t, imu_timeout_ms)},
    {"imu_parse_error_limit", PARAM_TYPE_U32, offsetof(params_store_t, imu_parse_error_limit)},
    {"loop_overrun_limit", PARAM_TYPE_U32, offsetof(params_store_t, loop_overrun_limit)},
    {"battery_warn_v", PARAM_TYPE_FLOAT, offsetof(params_store_t, battery_warn_v)},
    {"battery_limit_v", PARAM_TYPE_FLOAT, offsetof(params_store_t, battery_limit_v)},
    {"battery_critical_v", PARAM_TYPE_FLOAT, offsetof(params_store_t, battery_critical_v)},
    {"battery_arm_v", PARAM_TYPE_FLOAT, offsetof(params_store_t, battery_arm_v)},
    {"telemetry_usb_hz", PARAM_TYPE_U32, offsetof(params_store_t, telemetry_usb_hz)},
    {"telemetry_udp_hz", PARAM_TYPE_U32, offsetof(params_store_t, telemetry_udp_hz)},
    {"ring_buffer_seconds", PARAM_TYPE_U32, offsetof(params_store_t, ring_buffer_seconds)},
    {"wifi_ap_enable", PARAM_TYPE_BOOL, offsetof(params_store_t, wifi_ap_enable)},
    {"wifi_ap_channel", PARAM_TYPE_U8, offsetof(params_store_t, wifi_ap_channel)},
    {"wifi_udp_port", PARAM_TYPE_U32, offsetof(params_store_t, wifi_udp_port)},
    {"imu_mode", PARAM_TYPE_I32, offsetof(params_store_t, imu_mode)},
    {"imu_return_rate_code", PARAM_TYPE_U32, offsetof(params_store_t, imu_return_rate_code)},
    {"imu_map_x", PARAM_TYPE_I32, offsetof(params_store_t, imu_map_x)},
    {"imu_map_y", PARAM_TYPE_I32, offsetof(params_store_t, imu_map_y)},
    {"imu_map_z", PARAM_TYPE_I32, offsetof(params_store_t, imu_map_z)},
    {"imu_mag_enable", PARAM_TYPE_BOOL, offsetof(params_store_t, imu_mag_enable)},
    {"motor_output_map0", PARAM_TYPE_U8, offsetof(params_store_t, motor_output_map[0])},
    {"motor_output_map1", PARAM_TYPE_U8, offsetof(params_store_t, motor_output_map[1])},
    {"motor_output_map2", PARAM_TYPE_U8, offsetof(params_store_t, motor_output_map[2])},
    {"motor_output_map3", PARAM_TYPE_U8, offsetof(params_store_t, motor_output_map[3])},
    {"motor_spin_is_cw0", PARAM_TYPE_BOOL, offsetof(params_store_t, motor_spin_is_cw[0])},
    {"motor_spin_is_cw1", PARAM_TYPE_BOOL, offsetof(params_store_t, motor_spin_is_cw[1])},
    {"motor_spin_is_cw2", PARAM_TYPE_BOOL, offsetof(params_store_t, motor_spin_is_cw[2])},
    {"motor_spin_is_cw3", PARAM_TYPE_BOOL, offsetof(params_store_t, motor_spin_is_cw[3])},
    {"rate_kp_roll", PARAM_TYPE_FLOAT, offsetof(params_store_t, rate_kp_roll)},
    {"rate_ki_roll", PARAM_TYPE_FLOAT, offsetof(params_store_t, rate_ki_roll)},
    {"rate_kd_roll", PARAM_TYPE_FLOAT, offsetof(params_store_t, rate_kd_roll)},
    {"rate_kp_pitch", PARAM_TYPE_FLOAT, offsetof(params_store_t, rate_kp_pitch)},
    {"rate_ki_pitch", PARAM_TYPE_FLOAT, offsetof(params_store_t, rate_ki_pitch)},
    {"rate_kd_pitch", PARAM_TYPE_FLOAT, offsetof(params_store_t, rate_kd_pitch)},
    {"rate_kp_yaw", PARAM_TYPE_FLOAT, offsetof(params_store_t, rate_kp_yaw)},
    {"rate_ki_yaw", PARAM_TYPE_FLOAT, offsetof(params_store_t, rate_ki_yaw)},
    {"rate_kd_yaw", PARAM_TYPE_FLOAT, offsetof(params_store_t, rate_kd_yaw)},
    {"rate_integral_limit", PARAM_TYPE_FLOAT, offsetof(params_store_t, rate_integral_limit)},
    {"rate_output_limit", PARAM_TYPE_FLOAT, offsetof(params_store_t, rate_output_limit)},
    {"log_event_text_enabled", PARAM_TYPE_BOOL, offsetof(params_store_t, log_event_text_enabled)},
    {"attitude_kp_roll", PARAM_TYPE_FLOAT, offsetof(params_store_t, attitude_kp_roll)},
    {"attitude_kp_pitch", PARAM_TYPE_FLOAT, offsetof(params_store_t, attitude_kp_pitch)},
    {"attitude_rate_limit_roll", PARAM_TYPE_FLOAT, offsetof(params_store_t, attitude_rate_limit_roll)},
    {"attitude_rate_limit_pitch", PARAM_TYPE_FLOAT, offsetof(params_store_t, attitude_rate_limit_pitch)},
    {"attitude_error_deadband_deg", PARAM_TYPE_FLOAT, offsetof(params_store_t, attitude_error_deadband_deg)},
    {"attitude_trip_deg", PARAM_TYPE_FLOAT, offsetof(params_store_t, attitude_trip_deg)},
    {"attitude_test_base_duty", PARAM_TYPE_FLOAT, offsetof(params_store_t, attitude_test_base_duty)},
    {"gyro_lpf_hz", PARAM_TYPE_FLOAT, offsetof(params_store_t, gyro_lpf_hz)},
    {"accel_lpf_hz", PARAM_TYPE_FLOAT, offsetof(params_store_t, accel_lpf_hz)},
    {"rate_lpf_hz", PARAM_TYPE_FLOAT, offsetof(params_store_t, rate_lpf_hz)},
    {"kalman_enable", PARAM_TYPE_BOOL, offsetof(params_store_t, kalman_enable)},
    {"kalman_q_angle", PARAM_TYPE_FLOAT, offsetof(params_store_t, kalman_q_angle)},
    {"kalman_q_bias", PARAM_TYPE_FLOAT, offsetof(params_store_t, kalman_q_bias)},
    {"kalman_r_measure", PARAM_TYPE_FLOAT, offsetof(params_store_t, kalman_r_measure)},
    {"ground_tune_enable_attitude_outer", PARAM_TYPE_BOOL, offsetof(params_store_t, ground_tune_enable_attitude_outer)},
    {"ground_tune_use_kalman_attitude", PARAM_TYPE_BOOL, offsetof(params_store_t, ground_tune_use_kalman_attitude)},
    {"ground_tune_use_filtered_rate", PARAM_TYPE_BOOL, offsetof(params_store_t, ground_tune_use_filtered_rate)},
    {"ground_att_kp_roll", PARAM_TYPE_FLOAT, offsetof(params_store_t, ground_att_kp_roll)},
    {"ground_att_kp_pitch", PARAM_TYPE_FLOAT, offsetof(params_store_t, ground_att_kp_pitch)},
    {"ground_att_rate_limit_roll", PARAM_TYPE_FLOAT, offsetof(params_store_t, ground_att_rate_limit_roll)},
    {"ground_att_rate_limit_pitch", PARAM_TYPE_FLOAT, offsetof(params_store_t, ground_att_rate_limit_pitch)},
    {"ground_att_error_deadband_deg", PARAM_TYPE_FLOAT, offsetof(params_store_t, ground_att_error_deadband_deg)},
    {"ground_att_trip_deg", PARAM_TYPE_FLOAT, offsetof(params_store_t, ground_att_trip_deg)},
    {"ground_test_base_duty", PARAM_TYPE_FLOAT, offsetof(params_store_t, ground_test_base_duty)},
    {"ground_test_max_extra_duty", PARAM_TYPE_FLOAT, offsetof(params_store_t, ground_test_max_extra_duty)},
    {"ground_test_motor_balance_limit", PARAM_TYPE_FLOAT, offsetof(params_store_t, ground_test_motor_balance_limit)},
    {"ground_test_auto_disarm_ms", PARAM_TYPE_U32, offsetof(params_store_t, ground_test_auto_disarm_ms)},
    {"ground_test_ramp_duty_per_s", PARAM_TYPE_FLOAT, offsetof(params_store_t, ground_test_ramp_duty_per_s)},
};

static const param_descriptor_t s_runtime_param_descs[] = {
    {"attitude_ref_valid", PARAM_TYPE_BOOL, 0u},
    {"ground_ref_valid", PARAM_TYPE_BOOL, 0u},
};

/* Defaults are used only for factory reset and schema migration gaps. Saved rate PID values are preserved. */
static void params_apply_rate_pid_defaults(params_store_t *store)
{
    store->rate_kp_roll = PARAMS_RATE_KP_ROLL_DEFAULT;
    store->rate_ki_roll = PARAMS_RATE_KI_ROLL_DEFAULT;
    store->rate_kd_roll = PARAMS_RATE_KD_ROLL_DEFAULT;
    store->rate_kp_pitch = PARAMS_RATE_KP_PITCH_DEFAULT;
    store->rate_ki_pitch = PARAMS_RATE_KI_PITCH_DEFAULT;
    store->rate_kd_pitch = PARAMS_RATE_KD_PITCH_DEFAULT;
    store->rate_kp_yaw = PARAMS_RATE_KP_YAW_DEFAULT;
    store->rate_ki_yaw = PARAMS_RATE_KI_YAW_DEFAULT;
    store->rate_kd_yaw = PARAMS_RATE_KD_YAW_DEFAULT;
}

static void params_apply_motor_output_map_defaults(params_store_t *store)
{
    store->motor_output_map[0] = 3;
    store->motor_output_map[1] = 0;
    store->motor_output_map[2] = 1;
    store->motor_output_map[3] = 2;
}

static void params_apply_defaults(params_store_t *store)
{
    memset(store, 0, sizeof(*store));

    /* 默认值同时服务于 bring-up 和后续 bench 调试，所以倾向保守而不是激进。 */
    store->motor_pwm_freq_hz = 15000;
    store->motor_idle_duty = 0.05f;
    store->motor_max_duty = 0.95f;
    store->motor_startup_boost_duty = 0.16f;
    store->motor_startup_boost_ms = 25;
    store->motor_slew_limit_per_tick = 0.03f;
    store->bringup_test_base_duty = 0.15f;
    store->udp_manual_max_pwm = 0.12f;
    store->udp_takeoff_pwm = 0.10f;
    store->udp_land_min_pwm = 0.05f;
    store->udp_manual_timeout_ms = 1000;
    store->udp_manual_axis_limit = 0.05f;

    store->rc_timeout_ms = 300;
    store->imu_timeout_ms = 50;
    store->imu_parse_error_limit = 20;
    store->loop_overrun_limit = 10;

    store->battery_warn_v = 3.60f;
    store->battery_limit_v = 3.50f;
    store->battery_critical_v = 3.30f;
    store->battery_arm_v = 3.50f;

    store->telemetry_usb_hz = 200;
    store->telemetry_udp_hz = 100;
    store->ring_buffer_seconds = 10;
    store->wifi_ap_enable = true;
    store->wifi_ap_channel = 6;
    store->wifi_udp_port = 2391;

    store->imu_mode = IMU_MODE_DIRECT;
    store->imu_return_rate_code = 0x01;
    store->imu_map_x = BODY_AXIS_POS_X;
    store->imu_map_y = BODY_AXIS_POS_Y;
    store->imu_map_z = BODY_AXIS_POS_Z;
    store->imu_mag_enable = false;

    params_apply_motor_output_map_defaults(store);
    store->motor_spin_is_cw[0] = false;
    store->motor_spin_is_cw[1] = true;
    store->motor_spin_is_cw[2] = false;
    store->motor_spin_is_cw[3] = true;

    params_apply_rate_pid_defaults(store);
    store->rate_integral_limit = 100.0f;
    store->rate_output_limit = 0.20f;
    store->attitude_kp_roll = 2.0f;
    store->attitude_kp_pitch = 2.0f;
    store->attitude_rate_limit_roll = 25.0f;
    store->attitude_rate_limit_pitch = 25.0f;
    store->attitude_error_deadband_deg = 1.0f;
    store->attitude_trip_deg = 30.0f;
    store->attitude_test_base_duty = 0.05f;

    store->gyro_lpf_hz = 40.0f;
    store->accel_lpf_hz = 20.0f;
    store->rate_lpf_hz = 30.0f;
    store->kalman_enable = true;
    /* Gentle 1D angle Kalman defaults for low-throttle ground tuning. */
    store->kalman_q_angle = 0.0025f;
    store->kalman_q_bias = 0.0030f;
    store->kalman_r_measure = 0.0800f;
    store->ground_tune_enable_attitude_outer = false;
    store->ground_tune_use_kalman_attitude = true;
    store->ground_tune_use_filtered_rate = true;

    store->ground_att_kp_roll = 1.2f;
    store->ground_att_kp_pitch = 1.2f;
    store->ground_att_rate_limit_roll = 12.0f;
    store->ground_att_rate_limit_pitch = 12.0f;
    store->ground_att_error_deadband_deg = 0.8f;
    store->ground_att_trip_deg = 12.0f;
    store->ground_test_base_duty = 0.08f;
    store->ground_test_max_extra_duty = 0.05f;
    store->ground_test_motor_balance_limit = 0.08f;
    store->ground_test_auto_disarm_ms = 15000u;
    store->ground_test_ramp_duty_per_s = 0.30f;

    store->log_event_text_enabled = true;
}

static bool params_selector_is_valid(body_axis_selector_t selector)
{
    return selector >= BODY_AXIS_POS_X && selector <= BODY_AXIS_NEG_Z;
}

static int params_selector_abs_axis(body_axis_selector_t selector)
{
    switch (selector) {
    case BODY_AXIS_POS_X:
    case BODY_AXIS_NEG_X:
        return 0;
    case BODY_AXIS_POS_Y:
    case BODY_AXIS_NEG_Y:
        return 1;
    case BODY_AXIS_POS_Z:
    case BODY_AXIS_NEG_Z:
        return 2;
    default:
        return -1;
    }
}

static int params_selector_sign(body_axis_selector_t selector)
{
    switch (selector) {
    case BODY_AXIS_POS_X:
    case BODY_AXIS_POS_Y:
    case BODY_AXIS_POS_Z:
        return +1;
    case BODY_AXIS_NEG_X:
    case BODY_AXIS_NEG_Y:
    case BODY_AXIS_NEG_Z:
        return -1;
    default:
        return 0;
    }
}

static bool params_validate_motor_output_map(const params_store_t *store)
{
    bool seen[4] = {0};
    for (size_t i = 0; i < 4; ++i) {
        const uint8_t value = store->motor_output_map[i];
        if (value >= 4u || seen[value]) {
            return false;
        }
        seen[value] = true;
    }
    return true;
}

static bool params_validate_imu_map(const params_store_t *store)
{
    const body_axis_selector_t selectors[3] = {
        store->imu_map_x,
        store->imu_map_y,
        store->imu_map_z,
    };
    int matrix[3][3] = {{0}};

    /* IMU 轴映射必须构成合法的正交、右手系机体系，避免出现重复轴或镜像映射。 */
    for (size_t row = 0; row < 3; ++row) {
        if (!params_selector_is_valid(selectors[row])) {
            return false;
        }

        const int axis = params_selector_abs_axis(selectors[row]);
        const int sign = params_selector_sign(selectors[row]);
        if (axis < 0 || sign == 0) {
            return false;
        }

        for (size_t prev = 0; prev < row; ++prev) {
            if (params_selector_abs_axis(selectors[prev]) == axis) {
                return false;
            }
        }

        matrix[row][axis] = sign;
    }

    const int det =
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
        matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
        matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);

    return det == 1;
}

static bool params_validate_telemetry_rates(const params_store_t *store)
{
    return store->telemetry_usb_hz >= 1u &&
           store->telemetry_usb_hz <= 200u &&
           store->telemetry_udp_hz >= 1u &&
           store->telemetry_udp_hz <= 100u;
}

static bool params_validate_network(const params_store_t *store)
{
    return store->wifi_ap_channel >= 1u &&
           store->wifi_ap_channel <= 13u &&
           store->wifi_udp_port >= 1u &&
           store->wifi_udp_port <= 65535u;
}

static bool params_validate_battery_thresholds(const params_store_t *store)
{
    return store->battery_warn_v > 0.0f &&
           store->battery_limit_v > 0.0f &&
           store->battery_critical_v > 0.0f &&
           store->battery_arm_v > 0.0f &&
           store->battery_warn_v > store->battery_limit_v &&
           store->battery_limit_v > store->battery_critical_v &&
           store->battery_arm_v >= store->battery_limit_v &&
           store->battery_arm_v <= store->battery_warn_v;
}

static bool params_validate_motor_duty_limits(const params_store_t *store)
{
    return store->motor_idle_duty >= 0.0f &&
           store->motor_idle_duty <= 1.0f &&
           store->motor_max_duty >= 0.0f &&
           store->motor_max_duty <= 1.0f &&
           store->motor_startup_boost_duty >= 0.0f &&
           store->motor_startup_boost_duty <= 1.0f &&
           store->bringup_test_base_duty >= 0.0f &&
           store->bringup_test_base_duty <= 1.0f &&
           store->udp_manual_max_pwm >= 0.0f &&
           store->udp_manual_max_pwm <= 1.0f &&
           store->udp_takeoff_pwm >= 0.0f &&
           store->udp_takeoff_pwm <= 1.0f &&
           store->udp_land_min_pwm >= 0.0f &&
           store->udp_land_min_pwm <= 1.0f &&
           store->attitude_test_base_duty >= 0.0f &&
           store->attitude_test_base_duty <= 1.0f &&
           store->ground_test_base_duty >= 0.0f &&
           store->ground_test_base_duty <= 1.0f &&
           store->motor_idle_duty <= store->motor_max_duty &&
           store->motor_startup_boost_duty >= store->motor_idle_duty &&
           store->motor_startup_boost_duty <= store->motor_max_duty &&
           store->bringup_test_base_duty >= store->motor_idle_duty &&
           store->bringup_test_base_duty <= store->motor_max_duty &&
           store->udp_manual_max_pwm >= store->motor_idle_duty &&
           store->udp_manual_max_pwm <= store->motor_max_duty &&
           store->udp_land_min_pwm <= store->udp_takeoff_pwm &&
           store->udp_takeoff_pwm <= store->udp_manual_max_pwm &&
           store->attitude_test_base_duty >= store->motor_idle_duty &&
           store->attitude_test_base_duty <= store->motor_max_duty &&
           store->ground_test_base_duty >= store->motor_idle_duty &&
           store->ground_test_base_duty <= store->motor_max_duty;
}

static bool params_float_in_range(float value, float min_value, float max_value)
{
    return isfinite(value) && value >= min_value && value <= max_value;
}

static bool params_validate_rate_pid_limits(const params_store_t *store)
{
    const float max_kp = 0.050f;
    const float max_ki = 0.020f;
    const float max_kd = 0.010f;

    if (!params_float_in_range(store->rate_kp_roll, 0.0f, max_kp) ||
        !params_float_in_range(store->rate_ki_roll, 0.0f, max_ki) ||
        !params_float_in_range(store->rate_kd_roll, 0.0f, max_kd) ||
        !params_float_in_range(store->rate_kp_pitch, 0.0f, max_kp) ||
        !params_float_in_range(store->rate_ki_pitch, 0.0f, max_ki) ||
        !params_float_in_range(store->rate_kd_pitch, 0.0f, max_kd) ||
        !params_float_in_range(store->rate_kp_yaw, 0.0f, max_kp) ||
        !params_float_in_range(store->rate_ki_yaw, 0.0f, max_ki) ||
        !params_float_in_range(store->rate_kd_yaw, 0.0f, max_kd) ||
        !params_float_in_range(store->rate_integral_limit, 0.0f, 300.0f) ||
        !params_float_in_range(store->rate_output_limit, 0.0f, 0.35f)) {
        return false;
    }

    return (store->bringup_test_base_duty + store->rate_output_limit) <= store->motor_max_duty;
}

static bool params_validate_attitude_bench_limits(const params_store_t *store)
{
    if (!params_float_in_range(store->attitude_kp_roll, 0.0f, 10.0f) ||
        !params_float_in_range(store->attitude_kp_pitch, 0.0f, 10.0f) ||
        !params_float_in_range(store->attitude_rate_limit_roll, 0.0f, 180.0f) ||
        !params_float_in_range(store->attitude_rate_limit_pitch, 0.0f, 180.0f) ||
        !params_float_in_range(store->attitude_error_deadband_deg, 0.0f, 10.0f) ||
        !params_float_in_range(store->attitude_trip_deg, 1.0f, 90.0f)) {
        return false;
    }

    if (store->attitude_error_deadband_deg > store->attitude_trip_deg) {
        return false;
    }

    return (store->attitude_test_base_duty + store->rate_output_limit) <= store->motor_max_duty;
}

static bool params_validate_udp_manual_limits(const params_store_t *store)
{
    return store->udp_manual_timeout_ms >= 200u &&
           store->udp_manual_timeout_ms <= 1000u &&
           params_float_in_range(store->udp_manual_axis_limit, 0.0f, 0.15f);
}

static bool params_validate_filter_kalman_limits(const params_store_t *store)
{
    return params_float_in_range(store->gyro_lpf_hz, 0.0f, 250.0f) &&
           params_float_in_range(store->accel_lpf_hz, 0.0f, 120.0f) &&
           params_float_in_range(store->rate_lpf_hz, 0.0f, 250.0f) &&
           params_float_in_range(store->kalman_q_angle, 0.000001f, 1.0f) &&
           params_float_in_range(store->kalman_q_bias, 0.0000001f, 0.1f) &&
           params_float_in_range(store->kalman_r_measure, 0.00001f, 10.0f);
}

static bool params_validate_ground_tune_limits(const params_store_t *store)
{
    if (!params_float_in_range(store->ground_att_kp_roll, 0.0f, 10.0f) ||
        !params_float_in_range(store->ground_att_kp_pitch, 0.0f, 10.0f) ||
        !params_float_in_range(store->ground_att_rate_limit_roll, 0.0f, 60.0f) ||
        !params_float_in_range(store->ground_att_rate_limit_pitch, 0.0f, 60.0f) ||
        !params_float_in_range(store->ground_att_error_deadband_deg, 0.0f, 5.0f) ||
        !params_float_in_range(store->ground_att_trip_deg, 5.0f, 30.0f) ||
        !params_float_in_range(store->ground_test_max_extra_duty, 0.0f, 0.20f) ||
        !params_float_in_range(store->ground_test_motor_balance_limit, 0.0f, 0.30f) ||
        !params_float_in_range(store->ground_test_ramp_duty_per_s, 0.01f, 2.0f) ||
        store->ground_test_auto_disarm_ms < 1000u ||
        store->ground_test_auto_disarm_ms > 120000u) {
        return false;
    }

    if (store->ground_att_error_deadband_deg > store->ground_att_trip_deg) {
        return false;
    }

    return (store->ground_test_base_duty + store->ground_test_max_extra_duty) <= store->motor_max_duty &&
           (store->ground_test_base_duty + store->rate_output_limit) <= store->motor_max_duty;
}

static bool params_validate_store(const params_store_t *store)
{
    if (store == NULL) {
        return false;
    }

    return params_validate_motor_output_map(store) &&
           params_validate_imu_map(store) &&
           store->imu_return_rate_code <= 0x09u &&
           params_validate_telemetry_rates(store) &&
           params_validate_network(store) &&
           params_validate_battery_thresholds(store) &&
           params_validate_motor_duty_limits(store) &&
           params_validate_rate_pid_limits(store) &&
           params_validate_attitude_bench_limits(store) &&
           params_validate_udp_manual_limits(store) &&
           params_validate_filter_kalman_limits(store) &&
           params_validate_ground_tune_limits(store);
}

static bool params_try_load_from_nvs(params_store_t *store)
{
    nvs_handle_t handle;
    uint8_t blob_buf[sizeof(params_blob_t)] = {0};
    size_t len = sizeof(blob_buf);

    if (nvs_open(PARAMS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    esp_err_t err = nvs_get_blob(handle, PARAMS_BLOB_KEY, blob_buf, &len);
    nvs_close(handle);
    if (err != ESP_OK || len < sizeof(params_blob_header_t)) {
        return false;
    }

    params_blob_header_t header = {0};
    memcpy(&header, blob_buf, sizeof(header));

    const bool current_schema = header.schema_version == PARAMS_SCHEMA_VERSION;
    const bool pre_motor_remap_schema = header.schema_version == PARAMS_SCHEMA_VERSION_BEFORE_MOTOR_REMAP;
    const bool pre_ground_tune_schema = header.schema_version == PARAMS_SCHEMA_VERSION_BEFORE_GROUND_TUNE;
    const bool pre_rate_first_ground_tune_schema =
        header.schema_version == PARAMS_SCHEMA_VERSION_BEFORE_RATE_FIRST_GROUND_TUNE;
    const size_t payload_len = len - sizeof(header);
    if (header.magic != PARAMS_BLOB_MAGIC ||
        (!current_schema && !pre_motor_remap_schema && !pre_ground_tune_schema && !pre_rate_first_ground_tune_schema) ||
        header.payload_len != payload_len ||
        header.payload_len > sizeof(params_store_t)) {
        return false;
    }

    const uint8_t *payload_bytes = blob_buf + sizeof(header);
    const uint32_t crc = params_crc32(payload_bytes, header.payload_len);
    if (crc != header.crc32) {
        return false;
    }

    params_store_t payload = {0};
    params_apply_defaults(&payload);
    memcpy(&payload, payload_bytes, header.payload_len);

    if (pre_motor_remap_schema) {
        params_apply_motor_output_map_defaults(&payload);
    }

    if (!params_validate_store(&payload)) {
        return false;
    }

    *store = payload;
    return true;
}

esp_err_t params_init(void)
{
    params_apply_defaults(&s_params);

    if (params_try_load_from_nvs(&s_params)) {
        return ESP_OK;
    }

    return ESP_OK;
}

const params_store_t *params_get(void)
{
    return &s_params;
}

void params_reset_to_defaults(void)
{
    params_apply_defaults(&s_params);
}

esp_err_t params_save(void)
{
    nvs_handle_t handle;
    params_store_t payload = s_params;

    params_blob_t blob = {
        .header = {
            .magic = PARAMS_BLOB_MAGIC,
            .schema_version = PARAMS_SCHEMA_VERSION,
            .payload_len = sizeof(payload),
            .crc32 = params_crc32(&payload, sizeof(payload)),
        },
        .payload = payload,
    };

    esp_err_t err = nvs_open(PARAMS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(handle, PARAMS_BLOB_KEY, &blob, sizeof(blob));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

size_t params_count(void)
{
    return (sizeof(s_param_descs) / sizeof(s_param_descs[0])) +
           (sizeof(s_runtime_param_descs) / sizeof(s_runtime_param_descs[0]));
}

const param_descriptor_t *params_get_descriptor(size_t index)
{
    const size_t store_count = sizeof(s_param_descs) / sizeof(s_param_descs[0]);
    const size_t runtime_count = sizeof(s_runtime_param_descs) / sizeof(s_runtime_param_descs[0]);

    if (index >= (store_count + runtime_count)) {
        return NULL;
    }

    if (index < store_count) {
        return &s_param_descs[index];
    }

    return &s_runtime_param_descs[index - store_count];
}

static void *params_ptr_from_desc(const param_descriptor_t *desc)
{
    return ((uint8_t *)&s_params) + desc->offset;
}

bool params_try_get(const char *name, param_value_t *out_value, param_type_t *out_type)
{
    if (name == NULL || out_value == NULL || out_type == NULL) {
        return false;
    }

    if (strcmp(name, "attitude_ref_valid") == 0) {
        *out_type = PARAM_TYPE_BOOL;
        out_value->b = runtime_state_get_attitude_hang_state().ref_valid;
        return true;
    }
    if (strcmp(name, "ground_ref_valid") == 0) {
        *out_type = PARAM_TYPE_BOOL;
        out_value->b = runtime_state_get_ground_tune_state().ref_valid;
        return true;
    }

    const size_t store_count = sizeof(s_param_descs) / sizeof(s_param_descs[0]);
    for (size_t i = 0; i < store_count; ++i) {
        const param_descriptor_t *desc = &s_param_descs[i];
        if (strcmp(desc->name, name) != 0) {
            continue;
        }

        const void *ptr = params_ptr_from_desc(desc);
        *out_type = desc->type;
        switch (desc->type) {
        case PARAM_TYPE_BOOL:
            out_value->b = *(const bool *)ptr;
            return true;
        case PARAM_TYPE_U8:
            out_value->u8 = *(const uint8_t *)ptr;
            return true;
        case PARAM_TYPE_U32:
            out_value->u32 = *(const uint32_t *)ptr;
            return true;
        case PARAM_TYPE_I32:
            out_value->i32 = *(const int32_t *)ptr;
            return true;
        case PARAM_TYPE_FLOAT:
            out_value->f32 = *(const float *)ptr;
            return true;
        }
    }

    return false;
}

bool params_try_set(const char *name, param_value_t value, param_type_t type)
{
    if (name == NULL) {
        return false;
    }

    if (strcmp(name, "attitude_ref_valid") == 0) {
        (void)value;
        (void)type;
        return false;
    }
    if (strcmp(name, "ground_ref_valid") == 0) {
        (void)value;
        (void)type;
        return false;
    }

    const size_t store_count = sizeof(s_param_descs) / sizeof(s_param_descs[0]);
    for (size_t i = 0; i < store_count; ++i) {
        const param_descriptor_t *desc = &s_param_descs[i];
        if (strcmp(desc->name, name) != 0 || desc->type != type) {
            continue;
        }

        params_store_t candidate = s_params;
        void *ptr = ((uint8_t *)&candidate) + desc->offset;
        switch (desc->type) {
        case PARAM_TYPE_BOOL:
            *(bool *)ptr = value.b;
            break;
        case PARAM_TYPE_U8:
            *(uint8_t *)ptr = value.u8;
            break;
        case PARAM_TYPE_U32:
            *(uint32_t *)ptr = value.u32;
            break;
        case PARAM_TYPE_I32:
            *(int32_t *)ptr = value.i32;
            break;
        case PARAM_TYPE_FLOAT:
            *(float *)ptr = value.f32;
            break;
        }

        if (!params_validate_store(&candidate)) {
            return false;
        }

        s_params = candidate;
        return true;
    }

    return false;
}
