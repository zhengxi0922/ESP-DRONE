/**
 * @file params.h
 * @brief 参数系统接口。
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "esp_drone_types.h"

/**
 * @brief 参数在 NVS 中使用的命名空间。
 *
 * @note 仅用于持久化存储隔离，不参与协议字段名。
 */
#define PARAMS_NAMESPACE "esp_drone"

/**
 * @brief 参数整块 blob 在 NVS 中的键名。
 *
 * @note 当前参数系统采用单 blob 持久化，而不是逐项键值存储。
 */
#define PARAMS_BLOB_KEY "registry_v1"

/**
 * @brief 参数 blob 头部魔数。
 *
 * @note 用于识别当前持久化结构是否属于本项目参数格式。
 */
#define PARAMS_BLOB_MAGIC 0x31504445u

/**
 * @brief 参数持久化结构版本号。
 *
 * @note 参数布局发生不兼容变化时应递增。
 */
#define PARAMS_SCHEMA_VERSION 9u

/**
 * @brief 参数值类型编号。
 *
 * @note 主机工具链与固件共享该编号。
 */
typedef enum {
    PARAM_TYPE_BOOL = 0,  /**< 布尔值。 */
    PARAM_TYPE_U8 = 1,    /**< 8 位无符号整数。 */
    PARAM_TYPE_U32 = 2,   /**< 32 位无符号整数。 */
    PARAM_TYPE_I32 = 3,   /**< 32 位有符号整数。 */
    PARAM_TYPE_FLOAT = 4, /**< 32 位浮点数。 */
} param_type_t;

/**
 * @brief 通用参数值联合体。
 */
typedef union {
    bool b;       /**< 布尔值。 */
    uint8_t u8;   /**< 8 位无符号整数。 */
    uint32_t u32; /**< 32 位无符号整数。 */
    int32_t i32;  /**< 32 位有符号整数。 */
    float f32;    /**< 32 位浮点数。 */
} param_value_t;

/**
 * @brief 单个参数的描述信息。
 */
typedef struct {
    const char *name;  /**< 参数名，协议与日志均使用该字符串。 */
    param_type_t type; /**< 参数值类型。 */
    size_t offset;     /**< 字段在 `params_store_t` 中的偏移量。 */
} param_descriptor_t;

/**
 * @brief 全量参数存储结构。
 *
 * @note 该结构既承载默认值，也承载运行期当前值。
 */
typedef struct {
    uint32_t motor_pwm_freq_hz;       /**< 电机 PWM 频率，单位为 Hz。 */
    float motor_idle_duty;            /**< 非零输出时的最小占空比，范围为 `0.0f` 到 `1.0f`。 */
    float motor_max_duty;             /**< 电机输出上限，范围为 `0.0f` 到 `1.0f`。 */
    float motor_startup_boost_duty;   /**< 起转提升占空比，范围为 `0.0f` 到 `1.0f`。 */
    uint32_t motor_startup_boost_ms;  /**< 起转提升时长，单位为 ms；当前实现尚未消费该参数。 */
    float motor_slew_limit_per_tick;  /**< 每个控制周期允许的最大占空比变化量。 */
    float bringup_test_base_duty;     /**< 台架 bring-up 测试的基础油门，占空比范围为 `0.0f` 到 `1.0f`。 */
    float udp_manual_max_pwm;         /**< Experimental UDP manual max duty/PWM fraction. */
    float udp_takeoff_pwm;            /**< Experimental UDP takeoff target duty/PWM fraction. */
    float udp_land_min_pwm;           /**< Experimental UDP landing and timeout safe duty floor. */
    uint32_t udp_manual_timeout_ms;   /**< UDP manual watchdog timeout in ms. */
    float udp_manual_axis_limit;      /**< UDP manual normalized command limit; yaw maps to rate PID while roll/pitch use attitude hold. */

    uint32_t rc_timeout_ms;           /**< RC 超时阈值，单位为 ms；当前主循环尚未直接消费该参数。 */
    uint32_t imu_timeout_ms;          /**< IMU 超时阈值，单位为 ms。 */
    uint32_t imu_parse_error_limit;   /**< 连续 IMU 解析错误上限。 */
    uint32_t loop_overrun_limit;      /**< 控制循环连续超时上限。 */

    float battery_warn_v;             /**< 低电量告警阈值，单位为 V。 */
    float battery_limit_v;            /**< 预留限功率阈值，单位为 V；当前仅参与阈值合法性校验。 */
    float battery_critical_v;         /**< 触发 failsafe 的临界阈值，单位为 V。 */
    float battery_arm_v;              /**< 允许解锁的最低电压阈值，单位为 V。 */

    uint32_t telemetry_usb_hz;        /**< USB 遥测发送频率，单位为 Hz。 */
    uint32_t telemetry_udp_hz;        /**< 预留 UDP 遥测频率，单位为 Hz；当前未直接消费。 */
    uint32_t ring_buffer_seconds;     /**< 预留环形缓冲区时长，单位为 s；当前未直接消费。 */
    bool wifi_ap_enable;              /**< Enable ESP32 SoftAP on boot. */
    uint8_t wifi_ap_channel;          /**< ESP32 SoftAP channel, 1..13. */
    uint32_t wifi_udp_port;           /**< Binary CLI/GUI UDP protocol listening port. */

    imu_mode_t imu_mode;              /**< IMU 工作模式。 */
    uint32_t imu_return_rate_code;    /**< IMU 模块回传频率编码，范围为 `0x00` 到 `0x09`。 */
    body_axis_selector_t imu_map_x;   /**< 项目机体系 X 轴对应的模块轴。 */
    body_axis_selector_t imu_map_y;   /**< 项目机体系 Y 轴对应的模块轴。 */
    body_axis_selector_t imu_map_z;   /**< 项目机体系 Z 轴对应的模块轴。 */
    bool imu_mag_enable;              /**< 是否请求模块回传磁力计帧。 */

    uint8_t motor_output_map[4];      /**< 逻辑电机到物理 PWM 通道的排列，必须是 `0..3` 的全排列。 */
    bool motor_spin_is_cw[4];         /**< 逻辑电机旋向表，`true` 表示顺时针。 */

    float rate_kp_roll;               /**< Roll 轴速率环比例增益。 */
    float rate_ki_roll;               /**< Roll 轴速率环积分增益。 */
    float rate_kd_roll;               /**< Roll 轴速率环微分增益。 */
    float rate_kp_pitch;              /**< Pitch 轴速率环比例增益。 */
    float rate_ki_pitch;              /**< Pitch 轴速率环积分增益。 */
    float rate_kd_pitch;              /**< Pitch 轴速率环微分增益。 */
    float rate_kp_yaw;                /**< Yaw 轴速率环比例增益。 */
    float rate_ki_yaw;                /**< Yaw 轴速率环积分增益。 */
    float rate_kd_yaw;                /**< Yaw 轴速率环微分增益。 */
    float rate_integral_limit;        /**< 速率环积分限幅。 */
    float rate_output_limit;          /**< 速率环总输出限幅。 */

    bool log_event_text_enabled;      /**< 是否允许发送文本事件日志。 */
    float attitude_kp_roll;
    float attitude_kp_pitch;
    float attitude_rate_limit_roll;
    float attitude_rate_limit_pitch;
    float attitude_error_deadband_deg;
    float attitude_trip_deg;
    float attitude_test_base_duty;

    float gyro_lpf_hz;
    float accel_lpf_hz;
    float rate_lpf_hz;
    bool kalman_enable;
    float kalman_q_angle;
    float kalman_q_bias;
    float kalman_r_measure;
    bool ground_tune_enable_attitude_outer;
    bool ground_tune_use_kalman_attitude;
    bool ground_tune_use_filtered_rate;

    float ground_att_kp_roll;
    float ground_att_kp_pitch;
    float ground_att_rate_limit_roll;
    float ground_att_rate_limit_pitch;
    float ground_att_target_limit_deg;
    float ground_att_error_deadband_deg;
    float ground_att_trip_deg;
    float ground_test_base_duty;
    float ground_test_max_extra_duty;
    float ground_test_motor_balance_limit;
    uint32_t ground_test_auto_disarm_ms;
    float ground_test_ramp_duty_per_s;
    float liftoff_verify_base_duty;
    float liftoff_verify_max_extra_duty;
    uint32_t liftoff_verify_auto_disarm_ms;
    float liftoff_verify_ramp_duty_per_s;
    float liftoff_verify_att_trip_deg;
    float motor_trim_scale[4];
    float motor_trim_offset[4];
} params_store_t;

/**
 * @brief 初始化参数系统。
 *
 * @return `ESP_OK` 表示初始化完成。
 * @note 当前实现总是先装载默认值；NVS 中无有效数据时会静默回退到默认值。
 */
esp_err_t params_init(void);

/**
 * @brief 获取当前参数只读指针。
 *
 * @return 指向当前参数存储的静态只读指针。
 */
const params_store_t *params_get(void);

/**
 * @brief 恢复默认参数。
 *
 * @note 该操作只修改 RAM 中的当前值，不会自动写回 NVS。
 */
void params_reset_to_defaults(void);

/**
 * @brief 将当前参数保存到 NVS。
 *
 * @return `ESP_OK` 表示保存成功。
 */
esp_err_t params_save(void);

/**
 * @brief 获取参数总数。
 *
 * @return 当前可枚举的参数条目数量。
 */
size_t params_count(void);

/**
 * @brief 按索引获取参数描述。
 *
 * @param[in] index 参数索引，范围为 `0` 到 `params_count() - 1`。
 * @return 对应描述指针；索引越界时返回 `NULL`。
 */
const param_descriptor_t *params_get_descriptor(size_t index);

/**
 * @brief 按名称读取参数值。
 *
 * @param[in] name 参数名，不能为空。
 * @param[out] out_value 输出参数值，不能为空。
 * @param[out] out_type 输出参数类型，不能为空。
 * @retval true 成功读取参数。
 * @retval false 参数不存在或输出指针为空。
 */
bool params_try_get(const char *name, param_value_t *out_value, param_type_t *out_type);

/**
 * @brief 按名称写入参数值。
 *
 * @param[in] name 参数名，不能为空。
 * @param[in] value 待写入的参数值。
 * @param[in] type 参数类型，必须与目标参数完全匹配。
 * @retval true 写入成功。
 * @retval false 参数不存在、类型不匹配或候选参数集未通过完整合法性校验。
 * @note 该接口只更新 RAM 中的当前值，不会自动持久化到 NVS。
 */
bool params_try_set(const char *name, param_value_t value, param_type_t type);
