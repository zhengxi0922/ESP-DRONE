#pragma once

#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_drone_types.h"

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

typedef enum {
    BOARD_MOTOR_M1 = 0,
    BOARD_MOTOR_M2 = 1,
    BOARD_MOTOR_M3 = 2,
    BOARD_MOTOR_M4 = 3,
} board_motor_id_t;

typedef enum {
    MOTOR_POS_LEFT_FRONT = 0,
    MOTOR_POS_RIGHT_FRONT = 1,
    MOTOR_POS_RIGHT_REAR = 2,
    MOTOR_POS_LEFT_REAR = 3,
} motor_physical_position_t;

typedef struct {
    board_motor_id_t logical_id;
    motor_physical_position_t position;
    gpio_num_t gpio;
    bool spin_is_cw;
    int8_t body_x_sign;
    int8_t body_y_sign;
} board_motor_config_t;

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

const board_config_t *board_config_get(void);
const board_motor_config_t *board_get_motor_config(board_motor_id_t logical_id);

esp_err_t board_battery_init(void);
esp_err_t board_battery_read(int *out_raw, int *out_mv, float *out_voltage);
