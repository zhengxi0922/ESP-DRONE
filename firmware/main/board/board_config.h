/**
 * @file board_config.h
 * @brief ESP-DRONE ???????
 * @details ??????????????????????????????????? GPIO?
 * @author Codex
 * @date 2026-04-05
 * @version 1.0
 */

#pragma once

#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_drone_types.h"

/************************** 机体系与板级规则 **************************/
/*
 * Project body frame:
 *   +Y = nose / front
 *   +X = right side
 *   +Z = up
 *
 * Positive attitude names used everywhere in this repository:
 *   +pitch = nose up
 *   +roll  = right side down
 *   +yaw   = nose right, clockwise viewed from above
 *
 * Important:
 *   The body frame is right-handed, but the positive names for roll and yaw
 *   are project-defined and do not match the default mathematical positive
 *   rotation around +Y and +Z. Direction-sensitive code must reference
 *   docs/axis_truth_table.md and docs/motor_map.md before changing signs.
 */

#define BOARD_MOTOR_COUNT 4

/**
 * @brief 逻辑电机编号
 * @details 固件内部所有电机输出、mixer 和日志都必须以该逻辑编号为准，
 *          再通过 board 层映射到物理 GPIO。
 */
typedef enum {
    BOARD_MOTOR_M1 = 0,
    BOARD_MOTOR_M2 = 1,
    BOARD_MOTOR_M3 = 2,
    BOARD_MOTOR_M4 = 3,
} board_motor_id_t;

/**
 * @brief 电机物理位置定义
 * @details 物理位置与逻辑编号、GPIO、旋向分离，避免在 mixer 中把这些概念写死。
 */
typedef enum {
    MOTOR_POS_LEFT_FRONT = 0,
    MOTOR_POS_RIGHT_FRONT = 1,
    MOTOR_POS_RIGHT_REAR = 2,
    MOTOR_POS_LEFT_REAR = 3,
} motor_physical_position_t;

/**
 * @brief 单个电机的板级配置
 * @details 同时包含逻辑编号、物理位置、GPIO、旋向，以及在项目机体系中的 X/Y 位置符号。
 */
typedef struct {
    board_motor_id_t logical_id;
    motor_physical_position_t position;
    gpio_num_t gpio;
    bool spin_is_cw;
    int8_t body_x_sign;
    int8_t body_y_sign;
} board_motor_config_t;

/**
 * @brief 全板统一硬件配置
 * @details 集中描述 LED、电池 ADC、IMU UART 等共享硬件资源，是业务模块的只读真源。
 */
typedef struct {
    gpio_num_t led_g;
    gpio_num_t led_r;
    gpio_num_t led_y;
    gpio_num_t bat_adc;
    uart_port_t imu_uart_port;
    gpio_num_t imu_uart_tx;
    gpio_num_t imu_uart_rx;
    float bat_divider_ratio;
} board_config_t;

/**
 * @brief 获取全板硬件配置
 * @return const board_config_t* 指向只读板级配置结构的指针
 */
const board_config_t *board_config_get(void);

/**
 * @brief 获取指定逻辑电机的板级配置
 * @param logical_id 逻辑电机编号
 * @return const board_motor_config_t* 指向指定电机配置的只读指针；失败返回 NULL
 */
const board_motor_config_t *board_get_motor_config(board_motor_id_t logical_id);

/**
 * @brief 初始化电池采样硬件
 * @return esp_err_t ESP_OK 表示成功，其它值表示 ADC 初始化失败
 */
esp_err_t board_battery_init(void);

/**
 * @brief 读取当前电池采样值
 * @param[out] out_raw 原始 ADC 读数
 * @param[out] out_mv 校准后的毫伏值
 * @param[out] out_voltage 分压还原后的实际电池电压
 * @return esp_err_t ESP_OK 表示成功
 */
esp_err_t board_battery_read(int *out_raw, int *out_mv, float *out_voltage);
