/**
 * @file esp_drone_types.h
 * @brief ESP-DRONE ?????????
 * @details ?????????????????????????/?????
 * @author Codex
 * @date 2026-04-05
 * @version 1.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 三维向量
 */
typedef struct {
    float x;
    float y;
    float z;
} vec3f_t;

/**
 * @brief 项目语义下的 roll / pitch / yaw 三轴量
 */
typedef struct {
    float roll;
    float pitch;
    float yaw;
} axis3f_t;

/**
 * @brief 四元数
 */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} quatf_t;

/**
 * @brief 欧拉角姿态
 */
typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} eulerf_t;

/**
 * @brief 机体系轴选择器
 * @details 用于描述 IMU 模块坐标到项目机体系的有符号轴映射。
 */
typedef enum {
    BODY_AXIS_POS_X = 0,
    BODY_AXIS_NEG_X = 1,
    BODY_AXIS_POS_Y = 2,
    BODY_AXIS_NEG_Y = 3,
    BODY_AXIS_POS_Z = 4,
    BODY_AXIS_NEG_Z = 5,
} body_axis_selector_t;

/**
 * @brief IMU 模式枚举
 */
typedef enum {
    IMU_MODE_RAW = 0,
    IMU_MODE_DIRECT = 1,
} imu_mode_t;

typedef enum {
    IMU_HEALTH_INIT = 0,
    IMU_HEALTH_OK = 1,
    IMU_HEALTH_DEGRADED = 2,
    IMU_HEALTH_TIMEOUT = 3,
    IMU_HEALTH_PARSE_ERROR = 4,
} imu_health_t;

typedef enum {
    ARM_STATE_DISARMED = 0,
    ARM_STATE_ARMED = 1,
    ARM_STATE_FAILSAFE = 2,
    ARM_STATE_FAULT_LOCK = 3,
} arm_state_t;

typedef enum {
    FAILSAFE_REASON_NONE = 0,
    FAILSAFE_REASON_KILL = 1,
    FAILSAFE_REASON_RC_TIMEOUT = 2,
    FAILSAFE_REASON_IMU_TIMEOUT = 3,
    FAILSAFE_REASON_IMU_PARSE = 4,
    FAILSAFE_REASON_BATTERY_CRITICAL = 5,
    FAILSAFE_REASON_LOOP_OVERRUN = 6,
} failsafe_reason_t;

typedef enum {
    LED_STATE_INIT_WAIT_IMU = 0,
    LED_STATE_DISARMED_READY = 1,
    LED_STATE_ARMED_HEALTHY = 2,
    LED_STATE_LOW_BAT = 3,
    LED_STATE_FAILSAFE = 4,
    LED_STATE_IMU_ERROR = 5,
    LED_STATE_RC_LOSS = 6,
    LED_STATE_FAULT_LOCK = 7,
    LED_STATE_CALIBRATING = 8,
    LED_STATE_PARAM_SAVE = 9,
} led_state_t;

typedef enum {
    CONTROL_MODE_IDLE = 0,
    CONTROL_MODE_AXIS_TEST = 1,
    CONTROL_MODE_RATE_TEST = 2,
    CONTROL_MODE_HEIGHT_HOLD_RESERVED = 3,
} control_mode_t;

/**
 * @brief 气压计健康状态
 * @details 当前阶段只用于“数据链路是否可用”的健康判定，不参与油门闭环。
 */
typedef enum {
    BARO_HEALTH_INIT = 0,
    BARO_HEALTH_OK = 1,
    BARO_HEALTH_STALE = 2,
    BARO_HEALTH_INVALID = 3,
} baro_health_t;

/**
 * @brief 气压计状态快照
 * @details 统一承载 pressure / temperature / altitude / vertical speed，
 *          供 telemetry、CLI、GUI 和未来的定高估计扩展共同使用。
 */
typedef struct {
    uint64_t timestamp_us;
    float pressure_pa;
    float temperature_c;
    float altitude_m;
    float vertical_speed_mps;
    bool has_baro;
    bool valid;
    baro_health_t health;
    uint32_t update_age_us;
} barometer_state_t;

/**
 * @brief 未来定高闭环预留状态
 * @details 当前阶段只保留状态结构和模块边界，不参与任何油门/推力闭环。
 */
typedef struct {
    float target_altitude_m;
    float estimated_altitude_m;
    float estimated_vz_mps;
    bool altitude_hold_reserved_enabled;
} altitude_hold_reserved_state_t;

typedef struct {
    uint64_t timestamp_us;
    vec3f_t gyro_xyz_dps;
    vec3f_t acc_xyz_g;
    quatf_t quat_wxyz;
    eulerf_t roll_pitch_yaw_deg;
    imu_health_t health;
    uint32_t update_age_us;
    bool has_attitude;
    bool has_quaternion;
    bool has_gyro_acc;
} imu_sample_t;

/**
 * @brief 控制循环统计信息
 */
typedef struct {
    uint32_t loop_dt_us;
    uint32_t max_loop_dt_us;
    uint32_t loop_overrun_count;
} loop_stats_t;
