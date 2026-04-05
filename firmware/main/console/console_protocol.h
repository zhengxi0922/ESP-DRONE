/**
 * @file console_protocol.h
 * @brief 控制台二进制协议定义。
 */

#pragma once

#include <stdint.h>

/**
 * @brief 控制台原始帧魔数。
 *
 * @note 该值出现在 COBS 解码后的 `console_frame_header_t::magic` 字段。
 */
#define CONSOLE_FRAME_MAGIC 0xA5u

/**
 * @brief 当前原始帧布局版本号。
 *
 * @note 帧头、CRC 或 payload 编排发生不兼容变化时应递增。
 */
#define CONSOLE_FRAME_VERSION 0x01u

/**
 * @brief 对外握手暴露的协议版本号。
 *
 * @note 主机可通过 `MSG_HELLO_RESP` 检查该值以决定兼容策略。
 */
#define CONSOLE_PROTOCOL_VERSION 0x02u

/**
 * @brief 控制台消息类型。
 */
typedef enum {
    MSG_HELLO_REQ = 0x01,       /**< 主机请求设备握手信息。 */
    MSG_HELLO_RESP = 0x02,      /**< 设备返回握手信息。 */
    MSG_CMD_REQ = 0x10,         /**< 主机下发命令请求。 */
    MSG_CMD_RESP = 0x11,        /**< 设备返回命令执行结果。 */
    MSG_PARAM_GET = 0x20,       /**< 读取单个参数。 */
    MSG_PARAM_VALUE = 0x21,     /**< 返回单个参数值。 */
    MSG_PARAM_SET = 0x22,       /**< 写入单个参数。 */
    MSG_PARAM_LIST_REQ = 0x23,  /**< 枚举全部参数。 */
    MSG_PARAM_LIST_ITEM = 0x24, /**< 预留的参数列表项类型，当前固件未发送。 */
    MSG_PARAM_LIST_END = 0x25,  /**< 参数枚举结束。 */
    MSG_PARAM_SAVE = 0x26,      /**< 持久化当前参数。 */
    MSG_PARAM_RESET = 0x27,     /**< 恢复默认参数。 */
    MSG_STREAM_CTRL = 0x30,      /**< 控制遥测流开关。 */
    MSG_TELEMETRY_SAMPLE = 0x31, /**< 输出一帧遥测样本。 */
    MSG_EVENT_LOG_TEXT = 0x40,  /**< 输出文本事件日志。 */
} console_msg_type_t;

/**
 * @brief 设备命令编号。
 */
typedef enum {
    CMD_ARM = 1,         /**< 请求解锁。 */
    CMD_DISARM = 2,      /**< 请求上锁。 */
    CMD_KILL = 3,        /**< 请求急停并进入故障锁。 */
    CMD_REBOOT = 4,      /**< 请求软件重启。 */
    CMD_MOTOR_TEST = 5,  /**< 单电机点动测试。 */
    CMD_CALIB_GYRO = 6,  /**< 采集当前陀螺零偏。 */
    CMD_CALIB_LEVEL = 7, /**< 采集当前水平修正。 */
    CMD_AXIS_TEST = 8,   /**< 发送开环轴向测试命令。 */
    CMD_RATE_TEST = 9,   /**< 发送速率环测试目标。 */
} console_cmd_id_t;

/**
 * @brief 所有二进制消息共用的帧头。
 */
typedef struct __attribute__((packed)) {
    uint8_t magic;       /**< 固定为 `CONSOLE_FRAME_MAGIC`。 */
    uint8_t version;     /**< 固定为 `CONSOLE_FRAME_VERSION`。 */
    uint8_t msg_type;    /**< `console_msg_type_t` 枚举值。 */
    uint8_t flags;       /**< 预留标志位，当前固定为 0。 */
    uint16_t seq;        /**< 发送序号。 */
    uint16_t payload_len; /**< payload 长度，单位为字节。 */
} console_frame_header_t;

/**
 * @brief 命令请求载荷。
 *
 * @note `arg_u8` 与 `arg_f32` 的具体语义由 `cmd_id` 决定。
 */
typedef struct __attribute__((packed)) {
    uint8_t cmd_id;      /**< `console_cmd_id_t` 枚举值。 */
    uint8_t arg_u8;      /**< 命令整型参数。 */
    uint16_t reserved;   /**< 保留字段，发送方应置 0。 */
    float arg_f32;       /**< 命令浮点参数。 */
} console_cmd_req_t;

/**
 * @brief 命令响应载荷。
 *
 * @note `status` 约定为 `0=成功`、`1=拒绝`、`2=不支持`。
 */
typedef struct __attribute__((packed)) {
    uint8_t cmd_id;    /**< 对应的命令编号。 */
    uint8_t status;    /**< 执行结果。 */
    uint16_t reserved; /**< 保留字段，当前固定为 0。 */
} console_cmd_resp_t;

/**
 * @brief 握手响应载荷。
 */
typedef struct __attribute__((packed)) {
    uint8_t protocol_version; /**< `CONSOLE_PROTOCOL_VERSION`。 */
    uint8_t imu_mode;         /**< 当前 `imu_mode_t`。 */
    uint8_t arm_state;        /**< 当前 `arm_state_t`。 */
    uint8_t stream_enabled;   /**< 遥测流是否打开。 */
    uint32_t feature_bitmap;  /**< 功能位图。 */
} console_hello_resp_t;

/**
 * @brief 统一遥测样本载荷。
 *
 * @note 时间字段单位为 us。
 * @note 姿态角单位为 deg，角速度单位为 deg/s，加速度单位为 g。
 * @note 电池电压单位为 V，气压单位为 Pa，气压计温度单位为摄氏度。
 * @note 结构体按协议直接打包发送，主机侧不得假设可随意改序。
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
