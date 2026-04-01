#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float x;
    float y;
    float z;
} vec3f_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} axis3f_t;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quatf_t;

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} eulerf_t;

typedef enum {
    BODY_AXIS_POS_X = 0,
    BODY_AXIS_NEG_X = 1,
    BODY_AXIS_POS_Y = 2,
    BODY_AXIS_NEG_Y = 3,
    BODY_AXIS_POS_Z = 4,
    BODY_AXIS_NEG_Z = 5,
} body_axis_selector_t;

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
} control_mode_t;

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

typedef struct {
    uint32_t loop_dt_us;
    uint32_t max_loop_dt_us;
    uint32_t loop_overrun_count;
} loop_stats_t;
