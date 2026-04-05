/**
 * @file board_config.h
 * @brief 板级硬件映射接口。
 */

#pragma once

#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_drone_types.h"

/**
 * @brief 项目机体系约定。
 *
 * @note `+Y` 为机头前方，`+X` 为机体右侧，`+Z` 为向上。
 * @note 项目正方向命名为 `+pitch=抬头`、`+roll=右侧下沉`、`+yaw=机头向右`。
 * @warning 方向敏感代码修改前，应同时核对 `docs/axis_truth_table.md` 与
 *          `docs/motor_map.md`。
 */

/**
 * @brief 板级定义的电机通道数量。
 *
 * @note 当前适用于四轴机型；数组接口和逻辑电机编号均以该值为上限。
 */
#define BOARD_MOTOR_COUNT 4

/**
 * @brief 固件内部统一使用的逻辑电机编号。
 */
typedef enum {
    BOARD_MOTOR_M1 = 0, /**< 逻辑电机 1。 */
    BOARD_MOTOR_M2 = 1, /**< 逻辑电机 2。 */
    BOARD_MOTOR_M3 = 2, /**< 逻辑电机 3。 */
    BOARD_MOTOR_M4 = 3, /**< 逻辑电机 4。 */
} board_motor_id_t;

/**
 * @brief 电机的物理安装位置。
 */
typedef enum {
    MOTOR_POS_LEFT_FRONT = 0,  /**< 左前。 */
    MOTOR_POS_RIGHT_FRONT = 1, /**< 右前。 */
    MOTOR_POS_RIGHT_REAR = 2,  /**< 右后。 */
    MOTOR_POS_LEFT_REAR = 3,   /**< 左后。 */
} motor_physical_position_t;

/**
 * @brief 单个电机的板级配置。
 */
typedef struct {
    board_motor_id_t logical_id;            /**< 固件内部逻辑编号。 */
    motor_physical_position_t position;     /**< 电机物理位置。 */
    gpio_num_t gpio;                        /**< PWM 输出 GPIO。 */
    bool spin_is_cw;                        /**< 物理旋向，`true` 表示顺时针。 */
    int8_t body_x_sign;                     /**< 机体系 X 位置符号，只取 `-1` 或 `+1`。 */
    int8_t body_y_sign;                     /**< 机体系 Y 位置符号，只取 `-1` 或 `+1`。 */
} board_motor_config_t;

/**
 * @brief 板级共享硬件资源配置。
 */
typedef struct {
    gpio_num_t led_g;         /**< 绿色 LED GPIO。 */
    gpio_num_t led_r;         /**< 红色 LED GPIO。 */
    gpio_num_t led_y;         /**< 黄色 LED GPIO。 */
    gpio_num_t bat_adc;       /**< 电池分压采样 GPIO。 */
    uart_port_t imu_uart_port; /**< IMU 所在 UART 端口。 */
    gpio_num_t imu_uart_tx;   /**< IMU UART TX GPIO。 */
    gpio_num_t imu_uart_rx;   /**< IMU UART RX GPIO。 */
    float bat_divider_ratio;  /**< 电池分压还原系数，单位为电池电压/ADC 端电压。 */
} board_config_t;

/**
 * @brief 获取全板硬件配置。
 *
 * @return 指向只读板级配置的静态指针。
 */
const board_config_t *board_config_get(void);

/**
 * @brief 获取指定逻辑电机的板级配置。
 *
 * @param[in] logical_id 逻辑电机编号，范围为 `0` 到 `BOARD_MOTOR_COUNT - 1`。
 * @return 指向只读电机配置的静态指针；编号非法时返回 `NULL`。
 */
const board_motor_config_t *board_get_motor_config(board_motor_id_t logical_id);

/**
 * @brief 初始化电池采样硬件。
 *
 * @return `ESP_OK` 表示初始化成功。
 * @note 当前实现即使 ADC 曲线校准句柄创建失败，也会保留近似换算回退路径。
 */
esp_err_t board_battery_init(void);

/**
 * @brief 读取当前电池采样值。
 *
 * @param[out] out_raw 原始 ADC 读数，可为 `NULL`。
 * @param[out] out_mv ADC 端电压，单位为 mV，可为 `NULL`。
 * @param[out] out_voltage 经过分压还原后的电池电压，单位为 V，可为 `NULL`。
 * @return `ESP_OK` 表示读取成功；未初始化时返回 `ESP_ERR_INVALID_STATE`。
 */
esp_err_t board_battery_read(int *out_raw, int *out_mv, float *out_voltage);
