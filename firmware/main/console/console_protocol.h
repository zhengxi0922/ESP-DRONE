#pragma once

#include <stdint.h>

#define CONSOLE_FRAME_MAGIC 0xA5u
#define CONSOLE_FRAME_VERSION 0x01u
#define CONSOLE_PROTOCOL_VERSION 0x09u
#define CONSOLE_BUILD_GIT_HASH_LEN 16u
#define CONSOLE_BUILD_TIME_LEN 24u

#define CONSOLE_FEATURE_PARAMS (1u << 0)
#define CONSOLE_FEATURE_STREAMING (1u << 1)
#define CONSOLE_FEATURE_MOTOR_AXIS_TEST (1u << 2)
#define CONSOLE_FEATURE_RATE_TEST (1u << 3)
#define CONSOLE_FEATURE_BARO_TELEMETRY (1u << 4)
#define CONSOLE_FEATURE_ATTITUDE_HANG_BENCH (1u << 5)
#define CONSOLE_FEATURE_UDP_MANUAL_CONTROL (1u << 6)
#define CONSOLE_FEATURE_GROUND_TUNE (1u << 7)
#define CONSOLE_FEATURE_ATTITUDE_GROUND_VERIFY (1u << 8)
#define CONSOLE_FEATURE_LOW_RISK_LIFTOFF_VERIFY (1u << 9)
#define CONSOLE_FEATURE_ALL_MOTOR_TEST (1u << 10)
#define CONSOLE_FEATURE_BITMAP_CURRENT \
    (CONSOLE_FEATURE_PARAMS | \
     CONSOLE_FEATURE_STREAMING | \
     CONSOLE_FEATURE_MOTOR_AXIS_TEST | \
     CONSOLE_FEATURE_RATE_TEST | \
     CONSOLE_FEATURE_BARO_TELEMETRY | \
     CONSOLE_FEATURE_ATTITUDE_HANG_BENCH | \
     CONSOLE_FEATURE_UDP_MANUAL_CONTROL | \
     CONSOLE_FEATURE_GROUND_TUNE | \
     CONSOLE_FEATURE_ATTITUDE_GROUND_VERIFY | \
     CONSOLE_FEATURE_LOW_RISK_LIFTOFF_VERIFY | \
     CONSOLE_FEATURE_ALL_MOTOR_TEST)

typedef enum {
    MSG_HELLO_REQ = 0x01,
    MSG_HELLO_RESP = 0x02,
    MSG_CMD_REQ = 0x10,
    MSG_CMD_RESP = 0x11,
    MSG_PARAM_GET = 0x20,
    MSG_PARAM_VALUE = 0x21,
    MSG_PARAM_SET = 0x22,
    MSG_PARAM_LIST_REQ = 0x23,
    MSG_PARAM_LIST_ITEM = 0x24,
    MSG_PARAM_LIST_END = 0x25,
    MSG_PARAM_SAVE = 0x26,
    MSG_PARAM_RESET = 0x27,
    MSG_STREAM_CTRL = 0x30,
    MSG_TELEMETRY_SAMPLE = 0x31,
    MSG_EVENT_LOG_TEXT = 0x40,
    MSG_UDP_MANUAL_SETPOINT = 0x50,
} console_msg_type_t;

typedef enum {
    CMD_ARM = 1,
    CMD_DISARM = 2,
    CMD_KILL = 3,
    CMD_REBOOT = 4,
    CMD_MOTOR_TEST = 5,
    CMD_CALIB_GYRO = 6,
    CMD_CALIB_LEVEL = 7,
    CMD_AXIS_TEST = 8,
    CMD_RATE_TEST = 9,
    CMD_ATTITUDE_CAPTURE_REF = 10,
    CMD_ATTITUDE_TEST_START = 11,
    CMD_ATTITUDE_TEST_STOP = 12,
    CMD_UDP_MANUAL_ENABLE = 13,
    CMD_UDP_MANUAL_DISABLE = 14,
    CMD_UDP_MANUAL_SETPOINT = 15,
    CMD_UDP_TAKEOFF = 16,
    CMD_UDP_LAND = 17,
    CMD_UDP_MANUAL_STOP = 18,
    CMD_GROUND_CAPTURE_REF = 19,
    CMD_GROUND_TEST_START = 20,
    CMD_GROUND_TEST_STOP = 21,
    CMD_ATTITUDE_GROUND_VERIFY_START = 22,
    CMD_ATTITUDE_GROUND_VERIFY_STOP = 23,
    CMD_LIFTOFF_VERIFY_START = 24,
    CMD_LIFTOFF_VERIFY_STOP = 25,
    CMD_ATTITUDE_GROUND_SET_TARGET = 26,
    CMD_ALL_MOTOR_TEST_START = 27,
    CMD_ALL_MOTOR_TEST_STOP = 28,
} console_cmd_id_t;

typedef enum {
    CMD_STATUS_OK = 0,
    CMD_STATUS_REJECTED = 1,
    CMD_STATUS_UNSUPPORTED = 2,
    CMD_STATUS_INVALID_ARGUMENT = 3,
    CMD_STATUS_ARM_REQUIRED = 4,
    CMD_STATUS_DISARM_REQUIRED = 5,
    CMD_STATUS_IMU_NOT_READY = 6,
    CMD_STATUS_CONFLICT = 7,
    CMD_STATUS_STORAGE_ERROR = 8,
    CMD_STATUS_REF_REQUIRED = 9,
} console_cmd_status_t;

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t version;
    uint8_t msg_type;
    uint8_t flags;
    uint16_t seq;
    uint16_t payload_len;
} console_frame_header_t;

typedef struct __attribute__((packed)) {
    uint8_t cmd_id;
    uint8_t arg_u8;
    uint16_t reserved;
    float arg_f32;
} console_cmd_req_t;

typedef struct __attribute__((packed)) {
    uint8_t cmd_id;
    uint8_t status;
    uint16_t reserved;
} console_cmd_resp_t;

typedef struct __attribute__((packed)) {
    float throttle;
    float pitch;
    float roll;
    float yaw;
} console_udp_manual_setpoint_t;

typedef struct __attribute__((packed)) {
    uint8_t protocol_version;
    uint8_t imu_mode;
    uint8_t arm_state;
    uint8_t stream_enabled;
    uint32_t feature_bitmap;
    char build_git_hash[CONSOLE_BUILD_GIT_HASH_LEN];
    char build_time_utc[CONSOLE_BUILD_TIME_LEN];
} console_hello_resp_t;

typedef struct __attribute__((packed)) {
    uint64_t timestamp_us;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float setpoint_roll;
    float setpoint_pitch;
    float setpoint_yaw;
    float rate_setpoint_roll;
    float rate_setpoint_pitch;
    float rate_setpoint_yaw;
    float rate_pid_p_roll;
    float rate_pid_p_pitch;
    float rate_pid_p_yaw;
    float rate_pid_i_roll;
    float rate_pid_i_pitch;
    float rate_pid_i_yaw;
    float rate_pid_d_roll;
    float rate_pid_d_pitch;
    float rate_pid_d_yaw;
    float pid_out_roll;
    float pid_out_pitch;
    float pid_out_yaw;
    float motor1;
    float motor2;
    float motor3;
    float motor4;
    float battery_voltage;
    uint32_t battery_adc_raw;
    uint32_t loop_dt_us;
    uint32_t imu_age_us;
    uint8_t imu_mode;
    uint8_t imu_health;
    uint8_t arm_state;
    uint8_t failsafe_reason;
    uint8_t control_mode;
    uint8_t reserved[3];
    float baro_pressure_pa;
    float baro_temperature_c;
    float baro_altitude_m;
    float baro_vspeed_mps;
    uint32_t baro_update_age_us;
    uint8_t baro_valid;
    uint8_t baro_health;
    uint8_t baro_reserved[2];
    float attitude_err_roll_deg;
    float attitude_err_pitch_deg;
    float attitude_rate_sp_roll;
    float attitude_rate_sp_pitch;
    float attitude_ref_qw;
    float attitude_ref_qx;
    float attitude_ref_qy;
    float attitude_ref_qz;
    float base_duty_active;
    uint8_t attitude_ref_valid;
    uint8_t attitude_reserved[3];
    float filtered_gyro_x;
    float filtered_gyro_y;
    float filtered_gyro_z;
    float filtered_acc_x;
    float filtered_acc_y;
    float filtered_acc_z;
    float kalman_roll_deg;
    float kalman_pitch_deg;
    float rate_meas_roll_raw;
    float rate_meas_pitch_raw;
    float rate_meas_yaw_raw;
    float rate_meas_roll_filtered;
    float rate_meas_pitch_filtered;
    float rate_meas_yaw_filtered;
    float rate_err_roll;
    float rate_err_pitch;
    float rate_err_yaw;
    uint32_t sample_seq;
    uint8_t attitude_valid;
    uint8_t kalman_valid;
    uint8_t motor_saturation_flag;
    uint8_t integrator_freeze_flag;
    uint8_t ground_ref_valid;
    uint8_t reference_valid;
    uint8_t ground_trip_reason;
    uint8_t battery_valid;
    float angle_target_roll;
    float angle_target_pitch;
    float angle_target_yaw;
    float angle_measured_roll;
    float angle_measured_pitch;
    float angle_measured_yaw;
    float angle_error_roll;
    float angle_error_pitch;
    float angle_error_yaw;
    float outer_loop_rate_target_roll;
    float outer_loop_rate_target_pitch;
    float outer_loop_rate_target_yaw;
    uint8_t outer_loop_clamp_flag;
    uint8_t inner_loop_clamp_flag;
    uint8_t control_submode;
    uint8_t telemetry_reserved_v5;
} console_telemetry_sample_t;
