#include "imu.h"

#include <stdatomic.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_timer.h"

#include "board_config.h"
#include "params.h"

#define IMU_FRAME_HEAD_LOW 0x55
#define IMU_FRAME_HEAD_UPLOAD_HIGH 0x55
#define IMU_FRAME_HEAD_ACK_HIGH 0xAF
#define IMU_MAX_PAYLOAD_LEN 28

#define IMU_FRAME_ATTITUDE 0x01
#define IMU_FRAME_QUATERNION 0x02
#define IMU_FRAME_GYRO_ACC 0x03
#define IMU_FRAME_MAG 0x04

#define IMU_REG_RETURNSET 0x08
#define IMU_REG_RETURNRATE 0x0A

typedef enum {
    PARSER_WAIT_HEAD_LOW = 0,
    PARSER_WAIT_HEAD_HIGH = 1,
    PARSER_WAIT_ID = 2,
    PARSER_WAIT_LEN = 3,
    PARSER_WAIT_PAYLOAD = 4,
    PARSER_WAIT_CHECKSUM = 5,
} parser_state_t;

typedef struct {
    parser_state_t state;
    uint8_t head_high;
    uint8_t frame_id;
    uint8_t data_len;
    uint8_t payload[IMU_MAX_PAYLOAD_LEN];
    uint8_t payload_index;
    uint8_t checksum;
} frame_parser_t;

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    quatf_t quat;
    vec3f_t gyro;
    vec3f_t acc;
    bool have_attitude;
    bool have_quat;
    bool have_gyro_acc;
} partial_sample_t;

static frame_parser_t s_parser;
static partial_sample_t s_partial;
static imu_sample_t s_samples[2];
static _Atomic uint32_t s_sample_seq;
static _Atomic uint32_t s_sample_index;
static _Atomic uint32_t s_good_frames;
static _Atomic uint32_t s_parse_errors;
static _Atomic uint32_t s_consecutive_parse_errors;
static _Atomic uint64_t s_last_frame_us;
static bool s_initialized;

static int16_t imu_read_s16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static float imu_norm_angle(float deg)
{
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

static uint8_t imu_return_rate_code_from_params(void)
{
    const uint32_t code = params_get()->imu_return_rate_code;
    return (uint8_t)((code > 0x09u) ? 0x01u : code);
}

static uint8_t imu_return_set_from_params(void)
{
    const params_store_t *params = params_get();
    if (params->imu_mode == IMU_MODE_RAW) {
        return params->imu_mag_enable ? 0x0C : 0x04;
    }
    return 0x03;
}

static void imu_write_reg_u8(uint8_t reg_id, uint8_t value)
{
    const board_config_t *board = board_config_get();
    uint8_t frame[6] = {
        IMU_FRAME_HEAD_LOW,
        IMU_FRAME_HEAD_ACK_HIGH,
        reg_id,
        0x01,
        value,
        0x00,
    };
    frame[5] = (uint8_t)(frame[0] + frame[1] + frame[2] + frame[3] + frame[4]);
    uart_write_bytes(board->imu_uart_port, frame, sizeof(frame));
}

static void imu_apply_param_configuration(void)
{
    imu_write_reg_u8(IMU_REG_RETURNSET, imu_return_set_from_params());
    imu_write_reg_u8(IMU_REG_RETURNRATE, imu_return_rate_code_from_params());
}

static void imu_publish_sample(imu_sample_t sample)
{
    const uint64_t now_us = esp_timer_get_time();
    sample.timestamp_us = now_us;
    sample.update_age_us = 0;

    const uint32_t next_index = atomic_load(&s_sample_index) ^ 1u;
    s_samples[next_index] = sample;
    atomic_store(&s_sample_index, next_index);
    atomic_fetch_add(&s_sample_seq, 1u);
}

static void imu_handle_completed_frame(uint8_t frame_id, const uint8_t *payload, uint8_t len)
{
    imu_sample_t sample = {
        .quat_wxyz = {1.0f, 0.0f, 0.0f, 0.0f},
        .health = IMU_HEALTH_OK,
    };

    switch (frame_id) {
    case IMU_FRAME_ATTITUDE:
        if (len >= 6) {
            s_partial.roll_deg = ((float)imu_read_s16_le(&payload[0]) / 32768.0f) * 180.0f;
            s_partial.pitch_deg = ((float)imu_read_s16_le(&payload[2]) / 32768.0f) * 180.0f;
            s_partial.yaw_deg = ((float)imu_read_s16_le(&payload[4]) / 32768.0f) * 180.0f;
            s_partial.have_attitude = true;
        }
        break;
    case IMU_FRAME_QUATERNION:
        if (len >= 8) {
            s_partial.quat.w = (float)imu_read_s16_le(&payload[0]) / 32768.0f;
            s_partial.quat.x = (float)imu_read_s16_le(&payload[2]) / 32768.0f;
            s_partial.quat.y = (float)imu_read_s16_le(&payload[4]) / 32768.0f;
            s_partial.quat.z = (float)imu_read_s16_le(&payload[6]) / 32768.0f;
            s_partial.have_quat = true;
        }
        break;
    case IMU_FRAME_GYRO_ACC:
        if (len >= 12) {
            s_partial.acc.x = ((float)imu_read_s16_le(&payload[0]) / 32768.0f) * 4.0f;
            s_partial.acc.y = ((float)imu_read_s16_le(&payload[2]) / 32768.0f) * 4.0f;
            s_partial.acc.z = ((float)imu_read_s16_le(&payload[4]) / 32768.0f) * 4.0f;
            s_partial.gyro.x = ((float)imu_read_s16_le(&payload[6]) / 32768.0f) * 2000.0f;
            s_partial.gyro.y = ((float)imu_read_s16_le(&payload[8]) / 32768.0f) * 2000.0f;
            s_partial.gyro.z = ((float)imu_read_s16_le(&payload[10]) / 32768.0f) * 2000.0f;
            s_partial.have_gyro_acc = true;
        }
        break;
    case IMU_FRAME_MAG:
    default:
        break;
    }

    sample.gyro_xyz_dps = s_partial.gyro;
    sample.acc_xyz_g = s_partial.acc;
    sample.quat_wxyz = s_partial.have_quat ? s_partial.quat : sample.quat_wxyz;
    sample.roll_pitch_yaw_deg.roll_deg = imu_norm_angle(s_partial.roll_deg);
    sample.roll_pitch_yaw_deg.pitch_deg = imu_norm_angle(s_partial.pitch_deg);
    sample.roll_pitch_yaw_deg.yaw_deg = imu_norm_angle(s_partial.yaw_deg);
    sample.has_attitude = s_partial.have_attitude || s_partial.have_quat;

    const params_store_t *params = params_get();
    if (params->imu_mode == IMU_MODE_RAW && !s_partial.have_gyro_acc) {
        return;
    }
    if (params->imu_mode == IMU_MODE_DIRECT && !sample.has_attitude) {
        return;
    }

    imu_publish_sample(sample);
    atomic_fetch_add(&s_good_frames, 1u);
    atomic_store(&s_consecutive_parse_errors, 0u);
    atomic_store(&s_last_frame_us, (uint64_t)esp_timer_get_time());
}

static void imu_parser_reset(void)
{
    memset(&s_parser, 0, sizeof(s_parser));
    s_parser.state = PARSER_WAIT_HEAD_LOW;
}

static void imu_parser_feed_byte(uint8_t byte)
{
    switch (s_parser.state) {
    case PARSER_WAIT_HEAD_LOW:
        if (byte == IMU_FRAME_HEAD_LOW) {
            s_parser.checksum = byte;
            s_parser.state = PARSER_WAIT_HEAD_HIGH;
        }
        break;
    case PARSER_WAIT_HEAD_HIGH:
        if (byte == IMU_FRAME_HEAD_UPLOAD_HIGH || byte == IMU_FRAME_HEAD_ACK_HIGH) {
            s_parser.head_high = byte;
            s_parser.checksum = (uint8_t)(s_parser.checksum + byte);
            s_parser.state = PARSER_WAIT_ID;
        } else {
            imu_parser_reset();
        }
        break;
    case PARSER_WAIT_ID:
        s_parser.frame_id = byte;
        s_parser.checksum = (uint8_t)(s_parser.checksum + byte);
        s_parser.state = PARSER_WAIT_LEN;
        break;
    case PARSER_WAIT_LEN:
        if (byte > IMU_MAX_PAYLOAD_LEN) {
            atomic_fetch_add(&s_parse_errors, 1u);
            atomic_fetch_add(&s_consecutive_parse_errors, 1u);
            imu_parser_reset();
            break;
        }
        s_parser.data_len = byte;
        s_parser.payload_index = 0;
        s_parser.checksum = (uint8_t)(s_parser.checksum + byte);
        s_parser.state = (byte == 0) ? PARSER_WAIT_CHECKSUM : PARSER_WAIT_PAYLOAD;
        break;
    case PARSER_WAIT_PAYLOAD:
        s_parser.payload[s_parser.payload_index++] = byte;
        s_parser.checksum = (uint8_t)(s_parser.checksum + byte);
        if (s_parser.payload_index >= s_parser.data_len) {
            s_parser.state = PARSER_WAIT_CHECKSUM;
        }
        break;
    case PARSER_WAIT_CHECKSUM:
        if (byte == s_parser.checksum && s_parser.head_high == IMU_FRAME_HEAD_UPLOAD_HIGH) {
            imu_handle_completed_frame(s_parser.frame_id, s_parser.payload, s_parser.data_len);
        } else if (byte != s_parser.checksum) {
            atomic_fetch_add(&s_parse_errors, 1u);
            atomic_fetch_add(&s_consecutive_parse_errors, 1u);
        }
        imu_parser_reset();
        break;
    }
}

esp_err_t imu_init(void)
{
    const board_config_t *board = board_config_get();
    const uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    imu_parser_reset();
    memset(&s_partial, 0, sizeof(s_partial));

    esp_err_t err = uart_driver_install(board->imu_uart_port, 2048, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        return err;
    }
    err = uart_param_config(board->imu_uart_port, &uart_cfg);
    if (err != ESP_OK) {
        return err;
    }
    err = uart_set_pin(board->imu_uart_port, board->imu_uart_tx, board->imu_uart_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        return err;
    }

    imu_apply_param_configuration();

    atomic_store(&s_sample_index, 0u);
    atomic_store(&s_sample_seq, 0u);
    atomic_store(&s_good_frames, 0u);
    atomic_store(&s_parse_errors, 0u);
    atomic_store(&s_consecutive_parse_errors, 0u);
    atomic_store(&s_last_frame_us, 0u);

    s_samples[0] = (imu_sample_t){
        .quat_wxyz = {1.0f, 0.0f, 0.0f, 0.0f},
        .health = IMU_HEALTH_INIT,
    };
    s_samples[1] = s_samples[0];
    s_initialized = true;
    return ESP_OK;
}

esp_err_t imu_reconfigure_from_params(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    imu_apply_param_configuration();
    return ESP_OK;
}

void imu_service_rx(void)
{
    const board_config_t *board = board_config_get();
    uint8_t buf[128];
    const int read_count = uart_read_bytes(board->imu_uart_port, buf, sizeof(buf), 10 / portTICK_PERIOD_MS);
    for (int i = 0; i < read_count; ++i) {
        imu_parser_feed_byte(buf[i]);
    }

    if (!s_initialized) {
        return;
    }

    const uint64_t now_us = (uint64_t)esp_timer_get_time();
    const uint64_t last_frame_us = atomic_load(&s_last_frame_us);
    const uint32_t sample_index = atomic_load(&s_sample_index);
    imu_sample_t sample = s_samples[sample_index];
    if (last_frame_us == 0) {
        sample.health = IMU_HEALTH_INIT;
        sample.update_age_us = 0;
    } else {
        const uint32_t age_us = (uint32_t)(now_us - last_frame_us);
        sample.update_age_us = age_us;
        sample.health = (age_us > (params_get()->imu_timeout_ms * 1000u)) ? IMU_HEALTH_TIMEOUT : sample.health;
        s_samples[sample_index] = sample;
    }
}

bool imu_get_latest(imu_sample_t *out_sample, uint32_t *out_seq)
{
    if (!s_initialized || out_sample == NULL) {
        return false;
    }

    uint32_t seq_before;
    uint32_t seq_after;
    do {
        seq_before = atomic_load(&s_sample_seq);
        const uint32_t index = atomic_load(&s_sample_index);
        *out_sample = s_samples[index];
        seq_after = atomic_load(&s_sample_seq);
    } while (seq_before != seq_after);

    if (out_seq != NULL) {
        *out_seq = seq_after;
    }
    return true;
}

imu_stats_t imu_get_stats(void)
{
    imu_stats_t stats = {
        .published_seq = atomic_load(&s_sample_seq),
        .good_frames = atomic_load(&s_good_frames),
        .parse_errors = atomic_load(&s_parse_errors),
        .consecutive_parse_errors = atomic_load(&s_consecutive_parse_errors),
        .last_frame_us = atomic_load(&s_last_frame_us),
    };
    return stats;
}

const char *imu_health_to_string(imu_health_t health)
{
    switch (health) {
    case IMU_HEALTH_INIT:
        return "INIT";
    case IMU_HEALTH_OK:
        return "OK";
    case IMU_HEALTH_DEGRADED:
        return "DEGRADED";
    case IMU_HEALTH_TIMEOUT:
        return "TIMEOUT";
    case IMU_HEALTH_PARSE_ERROR:
        return "PARSE_ERROR";
    default:
        return "UNKNOWN";
    }
}
