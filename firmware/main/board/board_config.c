/**
 * @file board_config.c
 * @brief 板级硬件映射与电池采样实现。
 */

#include "board_config.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_adc_cali_handle;
static bool s_battery_adc_ready;

static const board_motor_config_t s_motor_configs[BOARD_MOTOR_COUNT] = {
    {
        .logical_id = BOARD_MOTOR_M1,
        .position = MOTOR_POS_LEFT_FRONT,
        .gpio = GPIO_NUM_5,
        .spin_is_cw = false,
        .body_x_sign = -1,
        .body_y_sign = +1,
    },
    {
        .logical_id = BOARD_MOTOR_M2,
        .position = MOTOR_POS_RIGHT_FRONT,
        .gpio = GPIO_NUM_6,
        .spin_is_cw = true,
        .body_x_sign = +1,
        .body_y_sign = +1,
    },
    {
        .logical_id = BOARD_MOTOR_M3,
        .position = MOTOR_POS_RIGHT_REAR,
        .gpio = GPIO_NUM_3,
        .spin_is_cw = false,
        .body_x_sign = +1,
        .body_y_sign = -1,
    },
    {
        .logical_id = BOARD_MOTOR_M4,
        .position = MOTOR_POS_LEFT_REAR,
        .gpio = GPIO_NUM_4,
        .spin_is_cw = true,
        .body_x_sign = -1,
        .body_y_sign = -1,
    },
};

static const board_config_t s_board = {
    .led_g = GPIO_NUM_46,
    .led_r = GPIO_NUM_8,
    .led_y = GPIO_NUM_7,
    .bat_adc = GPIO_NUM_2,
    .imu_uart_port = UART_NUM_0,
    .imu_uart_tx = GPIO_NUM_43,
    .imu_uart_rx = GPIO_NUM_44,
    .bat_divider_ratio = 2.0f,
};

const board_config_t *board_config_get(void)
{
    return &s_board;
}

const board_motor_config_t *board_get_motor_config(board_motor_id_t logical_id)
{
    if ((unsigned)logical_id >= BOARD_MOTOR_COUNT) {
        return NULL;
    }

    return &s_motor_configs[logical_id];
}

esp_err_t board_battery_init(void)
{
    if (s_battery_adc_ready) {
        return ESP_OK;
    }

    const adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc_handle);
    if (err != ESP_OK) {
        return err;
    }

    adc_channel_t channel = ADC_CHANNEL_1;
    adc_unit_t unit = ADC_UNIT_1;
    err = adc_oneshot_io_to_channel(s_board.bat_adc, &unit, &channel);
    if (err != ESP_OK) {
        return err;
    }

    const adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc_handle, channel, &chan_cfg);
    if (err != ESP_OK) {
        return err;
    }

    const adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = unit,
        .atten = chan_cfg.atten,
        .bitwidth = chan_cfg.bitwidth,
    };
    err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali_handle);
    if (err != ESP_OK) {
        s_adc_cali_handle = NULL;
    }

    s_battery_adc_ready = true;
    return ESP_OK;
}

esp_err_t board_battery_read(int *out_raw, int *out_mv, float *out_voltage)
{
    if (!s_battery_adc_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    adc_channel_t channel = ADC_CHANNEL_1;
    adc_unit_t unit = ADC_UNIT_1;
    esp_err_t err = adc_oneshot_io_to_channel(s_board.bat_adc, &unit, &channel);
    if (err != ESP_OK) {
        return err;
    }

    int raw = 0;
    err = adc_oneshot_read(s_adc_handle, channel, &raw);
    if (err != ESP_OK) {
        return err;
    }

    int mv = 0;
    if (s_adc_cali_handle != NULL) {
        err = adc_cali_raw_to_voltage(s_adc_cali_handle, raw, &mv);
        if (err != ESP_OK) {
            return err;
        }
    } else {
        mv = (raw * 2800) / 4095;
    }

    if (out_raw != NULL) {
        *out_raw = raw;
    }
    if (out_mv != NULL) {
        *out_mv = mv;
    }
    if (out_voltage != NULL) {
        *out_voltage = ((float)mv / 1000.0f) * s_board.bat_divider_ratio;
    }

    return ESP_OK;
}
