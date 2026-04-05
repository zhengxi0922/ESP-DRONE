/**
 * @file console_protocol.h
 * @brief ESP-DRONE ??????????
 * @details ?? USB CDC ? UDP ????????????????????
 * @author Codex
 * @date 2026-04-05
 * @version 1.0
 */

#pragma once

#include <stdint.h>

#define CONSOLE_FRAME_MAGIC 0xA5u
#define CONSOLE_FRAME_VERSION 0x01u
#define CONSOLE_PROTOCOL_VERSION 0x02u

/**
 * @brief 控制台消息类型
 * @details USB CDC 与 UDP 新协议共用同一套消息类型编号。
 */
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
} console_msg_type_t;

/**
 * @brief 设备命令编号
 * @details 通过 `MSG_CMD_REQ/RESP` 承载，用于 arm、点动测试、校准等动作。
 */
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
} console_cmd_id_t;

/**
 * @brief 通用帧头
 * @details 所有二进制消息都以该帧头开头，后接 payload 和 CRC。
 */
typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t version;
    uint8_t msg_type;
    uint8_t flags;
    uint16_t seq;
    uint16_t payload_len;
} console_frame_header_t;

/**
 * @brief 命令请求载荷
 */
typedef struct __attribute__((packed)) {
    uint8_t cmd_id;
    uint8_t arg_u8;
    uint16_t reserved;
    float arg_f32;
} console_cmd_req_t;

/**
 * @brief 命令响应载荷
 */
typedef struct __attribute__((packed)) {
    uint8_t cmd_id;
    uint8_t status;
    uint16_t reserved;
} console_cmd_resp_t;

/**
 * @brief HELLO 响应载荷
 * @details 用于主机侧确认协议版本、IMU 模式、arm 状态和功能位图。
 */
typedef struct __attribute__((packed)) {
    uint8_t protocol_version;
    uint8_t imu_mode;
    uint8_t arm_state;
    uint8_t stream_enabled;
    uint32_t feature_bitmap;
} console_hello_resp_t;

/**
 * @brief 统一遥测样本载荷
 * @details 同时服务于 CLI、GUI、CSV 导出和 bench 调试图表。
 */
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
} console_telemetry_sample_t;
