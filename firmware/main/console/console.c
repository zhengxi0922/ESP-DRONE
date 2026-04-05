/**
 * @file console.c
 * @brief ESP-DRONE ??????????
 * @details ?? USB CDC ??????????????????????????????
 * @author Codex
 * @date 2026-04-05
 * @version 1.0
 */

#include "console.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_vfs_cdcacm.h"

#include "console_protocol.h"
#include "controller.h"
#include "imu.h"
#include "motor.h"
#include "params.h"
#include "runtime_state.h"
#include "safety.h"

#define CONSOLE_RX_BUF_SIZE 512
#define CONSOLE_FRAME_BUF_SIZE 384
#define CONSOLE_EVENT_TEXT_MAX 120

static SemaphoreHandle_t s_console_tx_mutex;
static uint8_t s_rx_buf[CONSOLE_RX_BUF_SIZE];
static size_t s_rx_len;
static uint16_t s_tx_seq;
static bool s_initialized;

static uint16_t console_crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static size_t console_cobs_encode(const uint8_t *input, size_t length, uint8_t *output)
{
    uint8_t *start = output;
    const uint8_t *end = input + length;
    uint8_t *code_ptr = output++;
    uint8_t code = 1;

    while (input < end) {
        if (*input == 0) {
            *code_ptr = code;
            code_ptr = output++;
            code = 1;
            ++input;
        } else {
            *output++ = *input++;
            ++code;
            if (code == 0xFF) {
                *code_ptr = code;
                code_ptr = output++;
                code = 1;
            }
        }
    }

    *code_ptr = code;
    return (size_t)(output - start);
}

static bool console_cobs_decode(const uint8_t *input, size_t length, uint8_t *output, size_t *out_len)
{
    size_t write_len = 0;
    size_t index = 0;

    while (index < length) {
        const uint8_t code = input[index++];
        if (code == 0 || index + code - 1 > length + 1) {
            return false;
        }
        for (uint8_t i = 1; i < code; ++i) {
            if (index >= length) {
                return false;
            }
            output[write_len++] = input[index++];
        }
        if (code != 0xFF && index < length) {
            output[write_len++] = 0;
        }
    }

    *out_len = write_len;
    return true;
}

static bool console_send_frame(uint8_t msg_type, const void *payload, uint16_t payload_len)
{
    uint8_t raw_buf[CONSOLE_FRAME_BUF_SIZE];
    uint8_t encoded_buf[CONSOLE_FRAME_BUF_SIZE + 8];

    if (payload_len + sizeof(console_frame_header_t) + sizeof(uint16_t) > sizeof(raw_buf)) {
        return false;
    }

    console_frame_header_t header = {
        .magic = CONSOLE_FRAME_MAGIC,
        .version = CONSOLE_FRAME_VERSION,
        .msg_type = msg_type,
        .flags = 0,
        .seq = s_tx_seq++,
        .payload_len = payload_len,
    };

    memcpy(raw_buf, &header, sizeof(header));
    if (payload_len > 0 && payload != NULL) {
        memcpy(raw_buf + sizeof(header), payload, payload_len);
    }

    const uint16_t crc = console_crc16_ccitt(raw_buf, sizeof(header) + payload_len);
    memcpy(raw_buf + sizeof(header) + payload_len, &crc, sizeof(crc));

    const size_t raw_len = sizeof(header) + payload_len + sizeof(crc);
    const size_t encoded_len = console_cobs_encode(raw_buf, raw_len, encoded_buf);

    if (s_console_tx_mutex != NULL) {
        xSemaphoreTake(s_console_tx_mutex, portMAX_DELAY);
    }
    fwrite(encoded_buf, 1, encoded_len, stdout);
    fputc(0, stdout);
    fflush(stdout);
    fsync(fileno(stdout));
    if (s_console_tx_mutex != NULL) {
        xSemaphoreGive(s_console_tx_mutex);
    }

    return true;
}

static void console_send_cmd_resp(uint8_t cmd_id, uint8_t status)
{
    const console_cmd_resp_t resp = {
        .cmd_id = cmd_id,
        .status = status,
        .reserved = 0,
    };
    console_send_frame(MSG_CMD_RESP, &resp, sizeof(resp));
}

static bool console_decode_axis_request(uint8_t axis_id, float value, axis3f_t *out_axis)
{
    if (out_axis == NULL) {
        return false;
    }
    *out_axis = (axis3f_t){0};
    switch (axis_id) {
    case 0:
        out_axis->roll = value;
        return true;
    case 1:
        out_axis->pitch = value;
        return true;
    case 2:
        out_axis->yaw = value;
        return true;
    default:
        return false;
    }
}

static void console_send_param_value(const char *name)
{
    uint8_t payload[128];
    param_value_t value = {0};
    param_type_t type = PARAM_TYPE_U8;
    if (!params_try_get(name, &value, &type)) {
        return;
    }

    const size_t name_len = strlen(name);
    size_t value_len = 0;
    payload[0] = (uint8_t)type;
    payload[1] = (uint8_t)name_len;
    memcpy(payload + 2, name, name_len);
    size_t offset = 2 + name_len;

    switch (type) {
    case PARAM_TYPE_BOOL:
        payload[offset] = value.b ? 1 : 0;
        value_len = 1;
        break;
    case PARAM_TYPE_U8:
        payload[offset] = value.u8;
        value_len = 1;
        break;
    case PARAM_TYPE_U32:
        memcpy(payload + offset, &value.u32, sizeof(value.u32));
        value_len = sizeof(value.u32);
        break;
    case PARAM_TYPE_I32:
        memcpy(payload + offset, &value.i32, sizeof(value.i32));
        value_len = sizeof(value.i32);
        break;
    case PARAM_TYPE_FLOAT:
        memcpy(payload + offset, &value.f32, sizeof(value.f32));
        value_len = sizeof(value.f32);
        break;
    }

    console_send_frame(MSG_PARAM_VALUE, payload, (uint16_t)(offset + value_len));
}

static void console_handle_cmd_req(const uint8_t *payload, size_t len)
{
    if (len < sizeof(console_cmd_req_t)) {
        return;
    }

    console_cmd_req_t req = {0};
    memcpy(&req, payload, sizeof(req));

    switch (req.cmd_id) {
    case CMD_ARM:
        runtime_state_set_motor_test(-1, 0.0f);
        console_send_cmd_resp(req.cmd_id, safety_request_arm(true) ? 0 : 1);
        break;
    case CMD_DISARM:
        safety_request_disarm();
        runtime_state_set_motor_test(-1, 0.0f);
        runtime_state_set_control_mode(CONTROL_MODE_IDLE);
        runtime_state_set_axis_test_request((axis3f_t){0});
        runtime_state_set_rate_setpoint_request((axis3f_t){0});
        motor_stop_all();
        console_send_cmd_resp(req.cmd_id, 0);
        break;
    case CMD_KILL:
        safety_request_kill();
        runtime_state_set_motor_test(-1, 0.0f);
        runtime_state_set_control_mode(CONTROL_MODE_IDLE);
        runtime_state_set_axis_test_request((axis3f_t){0});
        runtime_state_set_rate_setpoint_request((axis3f_t){0});
        motor_stop_all();
        console_send_cmd_resp(req.cmd_id, 0);
        break;
    case CMD_REBOOT:
        runtime_state_set_stream_enabled(false);
        runtime_state_set_motor_test(-1, 0.0f);
        runtime_state_set_control_mode(CONTROL_MODE_IDLE);
        runtime_state_set_axis_test_request((axis3f_t){0});
        runtime_state_set_rate_setpoint_request((axis3f_t){0});
        motor_stop_all();
        console_send_cmd_resp(req.cmd_id, 0);
        fflush(stdout);
        /*
         * Give the host one short USB frame window to receive the ACK before
         * tearing down the CDC session via software restart.
         */
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
        break;
    case CMD_MOTOR_TEST:
        if (runtime_state_get_arm_state() != ARM_STATE_DISARMED) {
            console_send_cmd_resp(req.cmd_id, 1);
            break;
        }
        if (req.arg_u8 >= MOTOR_COUNT) {
            console_send_cmd_resp(req.cmd_id, 1);
            break;
        }
        if (req.arg_f32 <= 0.0f) {
            runtime_state_set_motor_test(-1, 0.0f);
            motor_stop_all();
        } else {
            runtime_state_set_control_mode(CONTROL_MODE_IDLE);
            runtime_state_set_axis_test_request((axis3f_t){0});
            runtime_state_set_rate_setpoint_request((axis3f_t){0});
            runtime_state_set_motor_test(req.arg_u8, req.arg_f32);
        }
        console_send_cmd_resp(req.cmd_id, 0);
        break;
    case CMD_AXIS_TEST: {
        axis3f_t request = {0};
        if (!console_decode_axis_request(req.arg_u8, req.arg_f32, &request)) {
            console_send_cmd_resp(req.cmd_id, 1);
            break;
        }
        runtime_state_set_motor_test(-1, 0.0f);
        runtime_state_set_rate_setpoint_request((axis3f_t){0});
        runtime_state_set_axis_test_request(request);
        runtime_state_set_control_mode((req.arg_f32 == 0.0f) ? CONTROL_MODE_IDLE : CONTROL_MODE_AXIS_TEST);
        console_send_cmd_resp(req.cmd_id, 0);
        break;
    }
    case CMD_RATE_TEST: {
        axis3f_t request = {0};
        if (!console_decode_axis_request(req.arg_u8, req.arg_f32, &request)) {
            console_send_cmd_resp(req.cmd_id, 1);
            break;
        }
        runtime_state_set_motor_test(-1, 0.0f);
        runtime_state_set_axis_test_request((axis3f_t){0});
        runtime_state_set_rate_setpoint_request(request);
        runtime_state_set_control_mode((req.arg_f32 == 0.0f) ? CONTROL_MODE_IDLE : CONTROL_MODE_RATE_TEST);
        console_send_cmd_resp(req.cmd_id, 0);
        break;
    }
    case CMD_CALIB_GYRO:
        console_send_cmd_resp(req.cmd_id, (imu_calibrate_gyro() == ESP_OK) ? 0 : 1);
        break;
    case CMD_CALIB_LEVEL:
        console_send_cmd_resp(req.cmd_id, (imu_calibrate_level() == ESP_OK) ? 0 : 1);
        break;
    default:
        console_send_cmd_resp(req.cmd_id, 2);
        break;
    }
}

static void console_handle_param_set(const uint8_t *payload, size_t len)
{
    if (len < 2) {
        return;
    }

    const param_type_t type = (param_type_t)payload[0];
    const uint8_t name_len = payload[1];
    if ((size_t)(2 + name_len) > len || name_len >= 64) {
        return;
    }

    char name[64] = {0};
    memcpy(name, payload + 2, name_len);
    const uint8_t *value_ptr = payload + 2 + name_len;
    param_value_t value = {0};

    switch (type) {
    case PARAM_TYPE_BOOL:
        if ((size_t)(3 + name_len) > len) {
            return;
        }
        value.b = value_ptr[0] != 0;
        break;
    case PARAM_TYPE_U8:
        if ((size_t)(3 + name_len) > len) {
            return;
        }
        value.u8 = value_ptr[0];
        break;
    case PARAM_TYPE_U32:
        if ((size_t)(2 + name_len + sizeof(uint32_t)) > len) {
            return;
        }
        memcpy(&value.u32, value_ptr, sizeof(uint32_t));
        break;
    case PARAM_TYPE_I32:
        if ((size_t)(2 + name_len + sizeof(int32_t)) > len) {
            return;
        }
        memcpy(&value.i32, value_ptr, sizeof(int32_t));
        break;
    case PARAM_TYPE_FLOAT:
        if ((size_t)(2 + name_len + sizeof(float)) > len) {
            return;
        }
        memcpy(&value.f32, value_ptr, sizeof(float));
        break;
    default:
        return;
    }

    if (!params_try_set(name, value, type)) {
        char msg[96];
        snprintf(msg, sizeof(msg), "param set rejected: %s", name);
        console_send_event_text(msg);
        console_send_param_value(name);
        return;
    }

    if (strcmp(name, "motor_pwm_freq_hz") == 0) {
        motor_reconfigure_from_params();
    }
    if (strcmp(name, "imu_mode") == 0 ||
        strcmp(name, "imu_return_rate_code") == 0 ||
        strcmp(name, "imu_mag_enable") == 0) {
        imu_reconfigure_from_params();
    }
    console_send_param_value(name);
}

static void console_handle_message(uint8_t msg_type, const uint8_t *payload, size_t len)
{
    switch ((console_msg_type_t)msg_type) {
    case MSG_HELLO_REQ: {
        const console_hello_resp_t resp = {
            .protocol_version = CONSOLE_PROTOCOL_VERSION,
            .imu_mode = (uint8_t)params_get()->imu_mode,
            .arm_state = (uint8_t)runtime_state_get_arm_state(),
            .stream_enabled = runtime_state_get_stream_enabled() ? 1u : 0u,
            .feature_bitmap = 0x0000001Fu,
        };
        console_send_frame(MSG_HELLO_RESP, &resp, sizeof(resp));
        break;
    }
    case MSG_CMD_REQ:
        console_handle_cmd_req(payload, len);
        break;
    case MSG_PARAM_GET:
        if (len >= 1) {
            char name[64] = {0};
            const uint8_t name_len = payload[0];
            if ((size_t)(1 + name_len) <= len && name_len < sizeof(name)) {
                memcpy(name, payload + 1, name_len);
                console_send_param_value(name);
            }
        }
        break;
    case MSG_PARAM_SET:
        console_handle_param_set(payload, len);
        break;
    case MSG_PARAM_LIST_REQ:
        for (size_t i = 0; i < params_count(); ++i) {
            const param_descriptor_t *desc = params_get_descriptor(i);
            if (desc != NULL) {
                console_send_param_value(desc->name);
            }
        }
        console_send_frame(MSG_PARAM_LIST_END, NULL, 0);
        break;
    case MSG_PARAM_SAVE:
        params_save();
        console_send_frame(MSG_PARAM_SAVE, NULL, 0);
        break;
    case MSG_PARAM_RESET:
        params_reset_to_defaults();
        motor_reconfigure_from_params();
        console_send_frame(MSG_PARAM_RESET, NULL, 0);
        break;
    case MSG_STREAM_CTRL:
        runtime_state_set_stream_enabled(len > 0 && payload[0] != 0);
        console_send_frame(MSG_STREAM_CTRL, payload, (uint16_t)len);
        break;
    default:
        break;
    }
}

static void console_try_process_frame(const uint8_t *encoded_frame, size_t encoded_len)
{
    uint8_t raw_buf[CONSOLE_FRAME_BUF_SIZE];
    size_t raw_len = 0;
    if (!console_cobs_decode(encoded_frame, encoded_len, raw_buf, &raw_len)) {
        return;
    }

    if (raw_len < sizeof(console_frame_header_t) + sizeof(uint16_t)) {
        return;
    }

    console_frame_header_t header = {0};
    memcpy(&header, raw_buf, sizeof(header));
    if (header.magic != CONSOLE_FRAME_MAGIC || header.version != CONSOLE_FRAME_VERSION) {
        return;
    }

    const size_t expected_len = sizeof(header) + header.payload_len + sizeof(uint16_t);
    if (raw_len != expected_len) {
        return;
    }

    uint16_t expected_crc = 0;
    memcpy(&expected_crc, raw_buf + raw_len - sizeof(uint16_t), sizeof(uint16_t));
    const uint16_t actual_crc = console_crc16_ccitt(raw_buf, raw_len - sizeof(uint16_t));
    if (expected_crc != actual_crc) {
        return;
    }

    console_handle_message(header.msg_type, raw_buf + sizeof(header), header.payload_len);
}

esp_err_t console_init(void)
{
    esp_vfs_dev_cdcacm_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
    esp_vfs_dev_cdcacm_set_tx_line_endings(ESP_LINE_ENDINGS_LF);

    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    s_console_tx_mutex = xSemaphoreCreateMutex();
    s_rx_len = 0;
    s_tx_seq = 0;
    s_initialized = true;
    return ESP_OK;
}

void console_service(void)
{
    if (!s_initialized) {
        return;
    }

    uint8_t byte = 0;
    while (read(STDIN_FILENO, &byte, 1) == 1) {
        if (byte == 0) {
            if (s_rx_len > 0) {
                console_try_process_frame(s_rx_buf, s_rx_len);
            }
            s_rx_len = 0;
            continue;
        }

        if (s_rx_len < sizeof(s_rx_buf)) {
            s_rx_buf[s_rx_len++] = byte;
        } else {
            s_rx_len = 0;
        }
    }

    if (errno != EAGAIN && errno != EWOULDBLOCK) {
        errno = 0;
    }
}

void console_send_event_text(const char *text)
{
    if (!s_initialized || !params_get()->log_event_text_enabled || text == NULL) {
        return;
    }

    uint8_t payload[CONSOLE_EVENT_TEXT_MAX + 2];
    const size_t text_len = strnlen(text, CONSOLE_EVENT_TEXT_MAX);
    payload[0] = 0;
    payload[1] = (uint8_t)text_len;
    memcpy(payload + 2, text, text_len);
    console_send_frame(MSG_EVENT_LOG_TEXT, payload, (uint16_t)(text_len + 2));
}

void console_send_telemetry(const imu_sample_t *imu_sample,
                            const barometer_state_t *baro_state,
                            float battery_voltage,
                            int battery_raw)
{
    if (!s_initialized || !runtime_state_get_stream_enabled() || imu_sample == NULL) {
        return;
    }

    float motor_outputs[MOTOR_COUNT] = {0};
    motor_get_outputs(motor_outputs);
    const loop_stats_t loop_stats = runtime_state_get_loop_stats();
    const axis3f_t axis_test_request = runtime_state_get_axis_test_request();
    const axis3f_t rate_setpoint_request = runtime_state_get_rate_setpoint_request();
    const rate_controller_status_t rate_status = controller_get_last_rate_status();
    const control_mode_t control_mode = runtime_state_get_control_mode();

    const console_telemetry_sample_t sample = {
        .timestamp_us = imu_sample->timestamp_us,
        .gyro_x = imu_sample->gyro_xyz_dps.x,
        .gyro_y = imu_sample->gyro_xyz_dps.y,
        .gyro_z = imu_sample->gyro_xyz_dps.z,
        .acc_x = imu_sample->acc_xyz_g.x,
        .acc_y = imu_sample->acc_xyz_g.y,
        .acc_z = imu_sample->acc_xyz_g.z,
        .quat_w = imu_sample->quat_wxyz.w,
        .quat_x = imu_sample->quat_wxyz.x,
        .quat_y = imu_sample->quat_wxyz.y,
        .quat_z = imu_sample->quat_wxyz.z,
        .roll_deg = imu_sample->roll_pitch_yaw_deg.roll_deg,
        .pitch_deg = imu_sample->roll_pitch_yaw_deg.pitch_deg,
        .yaw_deg = imu_sample->roll_pitch_yaw_deg.yaw_deg,
        .setpoint_roll = axis_test_request.roll,
        .setpoint_pitch = axis_test_request.pitch,
        .setpoint_yaw = axis_test_request.yaw,
        .rate_setpoint_roll = rate_setpoint_request.roll,
        .rate_setpoint_pitch = rate_setpoint_request.pitch,
        .rate_setpoint_yaw = rate_setpoint_request.yaw,
        .rate_pid_p_roll = rate_status.p_term.roll,
        .rate_pid_p_pitch = rate_status.p_term.pitch,
        .rate_pid_p_yaw = rate_status.p_term.yaw,
        .rate_pid_i_roll = rate_status.i_term.roll,
        .rate_pid_i_pitch = rate_status.i_term.pitch,
        .rate_pid_i_yaw = rate_status.i_term.yaw,
        .rate_pid_d_roll = rate_status.d_term.roll,
        .rate_pid_d_pitch = rate_status.d_term.pitch,
        .rate_pid_d_yaw = rate_status.d_term.yaw,
        .pid_out_roll = rate_status.output.roll,
        .pid_out_pitch = rate_status.output.pitch,
        .pid_out_yaw = rate_status.output.yaw,
        .motor1 = motor_outputs[0],
        .motor2 = motor_outputs[1],
        .motor3 = motor_outputs[2],
        .motor4 = motor_outputs[3],
        .battery_voltage = battery_voltage,
        .battery_adc_raw = (uint32_t)((battery_raw < 0) ? 0 : battery_raw),
        .loop_dt_us = loop_stats.loop_dt_us,
        .imu_age_us = imu_sample->update_age_us,
        .imu_mode = (uint8_t)params_get()->imu_mode,
        .imu_health = (uint8_t)imu_sample->health,
        .arm_state = (uint8_t)runtime_state_get_arm_state(),
        .failsafe_reason = (uint8_t)runtime_state_get_failsafe_reason(),
        .control_mode = (uint8_t)control_mode,
        .reserved = {0, 0, 0},
        .baro_pressure_pa = (baro_state != NULL) ? baro_state->pressure_pa : 0.0f,
        .baro_temperature_c = (baro_state != NULL) ? baro_state->temperature_c : 0.0f,
        .baro_altitude_m = (baro_state != NULL) ? baro_state->altitude_m : 0.0f,
        .baro_vspeed_mps = (baro_state != NULL) ? baro_state->vertical_speed_mps : 0.0f,
        .baro_update_age_us = (baro_state != NULL) ? baro_state->update_age_us : 0u,
        .baro_valid = (baro_state != NULL && baro_state->valid) ? 1u : 0u,
        .baro_health = (baro_state != NULL) ? (uint8_t)baro_state->health : (uint8_t)BARO_HEALTH_INIT,
        .baro_reserved = {0, 0},
    };

    console_send_frame(MSG_TELEMETRY_SAMPLE, &sample, sizeof(sample));
}
