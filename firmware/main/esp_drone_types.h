/**
 * @file esp_drone_types.h
 * @brief 全局共享类型定义。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 三维向量。
 */
typedef struct {
    float x; /**< X 分量。 */
    float y; /**< Y 分量。 */
    float z; /**< Z 分量。 */
} vec3f_t;

/**
 * @brief 项目语义下的 roll/pitch/yaw 三轴量。
 */
typedef struct {
    float roll;  /**< Roll 分量。 */
    float pitch; /**< Pitch 分量。 */
    float yaw;   /**< Yaw 分量。 */
} axis3f_t;

/**
 * @brief 四元数。
 *
 * @note 字段顺序固定为 `w,x,y,z`。
 */
typedef struct {
    float w; /**< 实部。 */
    float x; /**< X 虚部。 */
    float y; /**< Y 虚部。 */
    float z; /**< Z 虚部。 */
} quatf_t;

/**
 * @brief 欧拉角姿态。
 *
 * @note 所有字段单位均为 deg。
 */
typedef struct {
    float roll_deg;  /**< Roll 角。 */
    float pitch_deg; /**< Pitch 角。 */
    float yaw_deg;   /**< Yaw 角。 */
} eulerf_t;

/**
 * @brief IMU 模块坐标到项目机体系的轴选择器。
 */
typedef enum {
    BODY_AXIS_POS_X = 0, /**< 取模块 `+X`。 */
    BODY_AXIS_NEG_X = 1, /**< 取模块 `-X`。 */
    BODY_AXIS_POS_Y = 2, /**< 取模块 `+Y`。 */
    BODY_AXIS_NEG_Y = 3, /**< 取模块 `-Y`。 */
    BODY_AXIS_POS_Z = 4, /**< 取模块 `+Z`。 */
    BODY_AXIS_NEG_Z = 5, /**< 取模块 `-Z`。 */
} body_axis_selector_t;

/**
 * @brief IMU 工作模式。
 */
typedef enum {
    IMU_MODE_RAW = 0,    /**< 以原始陀螺/加速度数据为主。 */
    IMU_MODE_DIRECT = 1, /**< 直接消费模块姿态输出。 */
} imu_mode_t;

/**
 * @brief IMU 健康状态。
 */
typedef enum {
    IMU_HEALTH_INIT = 0,        /**< 尚未收到有效数据。 */
    IMU_HEALTH_OK = 1,          /**< 数据新鲜且满足当前模式要求。 */
    IMU_HEALTH_DEGRADED = 2,    /**< 数据部分可用，但缺少当前模式要求的关键量。 */
    IMU_HEALTH_TIMEOUT = 3,     /**< 超过超时阈值未收到新数据。 */
    IMU_HEALTH_PARSE_ERROR = 4, /**< 连续解析错误达到阈值。 */
} imu_health_t;

/**
 * @brief Arm 状态。
 */
typedef enum {
    ARM_STATE_DISARMED = 0,   /**< 上锁。 */
    ARM_STATE_ARMED = 1,      /**< 已解锁。 */
    ARM_STATE_FAILSAFE = 2,   /**< 进入 failsafe。 */
    ARM_STATE_FAULT_LOCK = 3, /**< 急停后的故障锁。 */
} arm_state_t;

/**
 * @brief Failsafe 原因。
 */
typedef enum {
    FAILSAFE_REASON_NONE = 0,             /**< 无 failsafe。 */
    FAILSAFE_REASON_KILL = 1,             /**< 人工急停。 */
    FAILSAFE_REASON_RC_TIMEOUT = 2,       /**< RC 链路丢失。 */
    FAILSAFE_REASON_IMU_TIMEOUT = 3,      /**< IMU 超时。 */
    FAILSAFE_REASON_IMU_PARSE = 4,        /**< IMU 解析错误。 */
    FAILSAFE_REASON_BATTERY_CRITICAL = 5, /**< 电池电压过低。 */
    FAILSAFE_REASON_LOOP_OVERRUN = 6,     /**< 控制循环连续超时。 */
} failsafe_reason_t;

/**
 * @brief LED 逻辑状态。
 */
typedef enum {
    LED_STATE_INIT_WAIT_IMU = 0,  /**< 等待 IMU 就绪。 */
    LED_STATE_DISARMED_READY = 1, /**< 已上锁且可进入测试。 */
    LED_STATE_ARMED_HEALTHY = 2,  /**< 已解锁且状态正常。 */
    LED_STATE_LOW_BAT = 3,       /**< 低电量告警。 */
    LED_STATE_FAILSAFE = 4,      /**< 一般 failsafe。 */
    LED_STATE_IMU_ERROR = 5,     /**< IMU 异常。 */
    LED_STATE_RC_LOSS = 6,       /**< RC 链路丢失。 */
    LED_STATE_FAULT_LOCK = 7,    /**< 急停故障锁。 */
    LED_STATE_CALIBRATING = 8,   /**< 校准进行中。 */
    LED_STATE_PARAM_SAVE = 9,    /**< 参数保存进行中。 */
} led_state_t;

/**
 * @brief 控制模式。
 */
typedef enum {
    CONTROL_MODE_IDLE = 0,                 /**< 空闲模式。 */
    CONTROL_MODE_AXIS_TEST = 1,            /**< 开环轴向测试。 */
    CONTROL_MODE_RATE_TEST = 2,            /**< 速率环测试。 */
    CONTROL_MODE_HEIGHT_HOLD_RESERVED = 3, /**< 预留定高模式。 */
    CONTROL_MODE_ATTITUDE_HANG_TEST = 4,   /**< 鍦嗘/鍚婃灦/鍙楅檺鍙版灦涓撶敤 attitude 澶栫幆 bring-up銆?*/
    CONTROL_MODE_UDP_MANUAL = 5,           /**< Experimental UDP manual control with attitude roll/pitch and rate-PID yaw. */
} control_mode_t;

/**
 * @brief 气压计健康状态。
 */
typedef enum {
    BARO_HEALTH_INIT = 0,    /**< 尚未收到有效气压计数据。 */
    BARO_HEALTH_OK = 1,      /**< 数据有效且新鲜。 */
    BARO_HEALTH_STALE = 2,   /**< 数据格式有效但已过陈旧阈值。 */
    BARO_HEALTH_INVALID = 3, /**< 数据超出当前有效范围。 */
} baro_health_t;

/**
 * @brief 气压计状态快照。
 */
typedef struct {
    uint64_t timestamp_us;      /**< 最近一次更新的时间戳，单位为 us。 */
    float pressure_pa;          /**< 气压，单位为 Pa。 */
    float temperature_c;        /**< 温度，单位为摄氏度。 */
    float altitude_m;           /**< 高度，单位为 m。 */
    float vertical_speed_mps;   /**< 垂向速度，单位为 m/s。 */
    bool has_baro;              /**< 是否已经收到过气压计数据。 */
    bool valid;                 /**< 当前数值是否落在有效范围内。 */
    baro_health_t health;       /**< 当前健康状态。 */
    uint32_t update_age_us;     /**< 数据年龄，单位为 us。 */
} barometer_state_t;

/**
 * @brief 预留的定高闭环状态。
 *
 * @note 当前阶段只保留边界与状态承载，不直接参与油门闭环。
 */
typedef struct {
    float target_altitude_m;             /**< 目标高度，单位为 m。 */
    float estimated_altitude_m;          /**< 当前估计高度，单位为 m。 */
    float estimated_vz_mps;              /**< 当前估计垂向速度，单位为 m/s。 */
    bool altitude_hold_reserved_enabled; /**< 预留定高模式开关。 */
} altitude_hold_reserved_state_t;

/**
 * @brief IMU 样本快照。
 *
 * @note 所有向量与姿态字段都已经映射到项目机体系语义。
 */
typedef struct {
    uint64_t timestamp_us;         /**< 样本时间戳，单位为 us。 */
    vec3f_t gyro_xyz_dps;          /**< 机体系角速度，单位为 deg/s。 */
    vec3f_t acc_xyz_g;             /**< 机体系加速度，单位为 g。 */
    quatf_t quat_wxyz;             /**< 机体系到世界系四元数。 */
    eulerf_t roll_pitch_yaw_deg;   /**< 项目语义姿态角，单位为 deg。 */
    imu_health_t health;           /**< 当前样本健康状态。 */
    uint32_t update_age_us;        /**< 数据年龄，单位为 us。 */
    bool has_attitude;             /**< 是否包含姿态角。 */
    bool has_quaternion;           /**< 是否包含四元数。 */
    bool has_gyro_acc;             /**< 是否包含陀螺与加速度。 */
} imu_sample_t;

/**
 * @brief 控制循环统计信息。
 */
typedef struct {
    uint32_t loop_dt_us;         /**< 最近一个控制周期时长，单位为 us。 */
    uint32_t max_loop_dt_us;     /**< 记录期内最大控制周期时长，单位为 us。 */
    uint32_t loop_overrun_count; /**< 连续超时计数。 */
} loop_stats_t;

typedef struct {
    bool ref_valid;
    quatf_t ref_q_body_to_world;
    float err_roll_deg;
    float err_pitch_deg;
    float rate_sp_roll_dps;
    float rate_sp_pitch_dps;
    float base_duty_active;
} attitude_hang_state_t;
