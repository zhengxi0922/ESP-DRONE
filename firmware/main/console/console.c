/**
 * @file console.c
 * @brief USB CDC 控制台协议实现。
 */

#include "console.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_vfs_cdcacm.h"

#include "attitude_bench.h"
#include "console_protocol.h"
#include "controller.h"
#include "estimator.h"
#include "ground_tune.h"
#include "imu.h"
#include "motor.h"
#include "params.h"
#include "runtime_state.h"
#include "safety.h"
#include "udp_manual.h"

#define CONSOLE_RX_BUF_SIZE 512
#define CONSOLE_FRAME_BUF_SIZE 384
#define CONSOLE_EVENT_TEXT_MAX 120
#define CONSOLE_AXIS_TEST_ABS_MAX 0.25f
#define CONSOLE_RATE_TEST_ABS_MAX_DPS 200.0f

#ifndef ESP_DRONE_BUILD_GIT_HASH
#define ESP_DRONE_BUILD_GIT_HASH "unknown"
#endif

#ifndef ESP_DRONE_BUILD_TIME_UTC
#define ESP_DRONE_BUILD_TIME_UTC "unknown"
#endif

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
    if (msg_type != MSG_TELEMETRY_SAMPLE) {
        fsync(fileno(stdout));
    }
    if (s_console_tx_mutex != NULL) {
        xSemaphoreGive(s_console_tx_mutex);
    }

    return true;
}

static void console_send_cmd_resp(uint8_t cmd_id, console_cmd_status_t status)
{
    const console_cmd_resp_t resp = {
        .cmd_id = cmd_id,
        .status = (uint8_t)status,
        .reserved = 0,
    };
    console_send_frame(MSG_CMD_RESP, &resp, sizeof(resp));
}

static bool console_has_active_motor_test(void)
{
    int logical_motor = -1;
    float duty = 0.0f;
    runtime_state_get_motor_test(&logical_motor, &duty);
    return logical_motor >= 0 && duty > 0.0f;
}

static bool console_sample_supports_attitude_ref(const imu_sample_t *sample)
{
    return sample != NULL &&
           sample->health == IMU_HEALTH_OK &&
           sample->has_gyro_acc &&
           sample->has_quaternion;
}

static void console_reset_attitude_state(bool clear_reference)
{
    if (clear_reference) {
        attitude_bench_clear_reference();
    } else {
        attitude_bench_reset_status();
    }
}

static void console_reset_ground_state(bool clear_reference)
{
    if (clear_reference) {
        ground_tune_clear_reference();
    } else {
        ground_tune_reset_status();
    }
}

static void console_stop_active_control(bool clear_attitude_reference)
{
    runtime_state_set_control_mode(CONTROL_MODE_IDLE);
    runtime_state_set_axis_test_request((axis3f_t){0});
    runtime_state_set_rate_setpoint_request((axis3f_t){0});
    console_reset_attitude_state(clear_attitude_reference);
    console_reset_ground_state(false);
    udp_manual_reset();
}

static bool console_value_is_finite_in_range(float value, float min_value, float max_value)
{
    return isfinite(value) && value >= min_value && value <= max_value;
}

static bool console_axis_test_value_is_valid(float value)
{
    return console_value_is_finite_in_range(value, -CONSOLE_AXIS_TEST_ABS_MAX, CONSOLE_AXIS_TEST_ABS_MAX);
}

static bool console_rate_test_value_is_valid(float value_dps)
{
    return console_value_is_finite_in_range(value_dps, -CONSOLE_RATE_TEST_ABS_MAX_DPS, CONSOLE_RATE_TEST_ABS_MAX_DPS);
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
        console_send_cmd_resp(req.cmd_id, safety_request_arm(true) ? CMD_STATUS_OK : CMD_STATUS_REJECTED);
        break;
    case CMD_DISARM:
        safety_request_disarm();
        runtime_state_set_motor_test(-1, 0.0f);
        console_stop_active_control(false);
        motor_stop_all();
        console_send_cmd_resp(req.cmd_id, CMD_STATUS_OK);
        break;
    case CMD_KILL:
        safety_request_kill();
        runtime_state_set_motor_test(-1, 0.0f);
        console_stop_active_control(false);
        motor_stop_all();
        console_send_cmd_resp(req.cmd_id, CMD_STATUS_OK);
        break;
    case CMD_REBOOT:
        runtime_state_set_stream_enabled(false);
        runtime_state_set_motor_test(-1, 0.0f);
        console_stop_active_control(true);
        motor_stop_all();
        console_send_cmd_resp(req.cmd_id, CMD_STATUS_OK);
        fflush(stdout);
        /* 软件重启前保留一个短 USB 帧窗口，让主机先收到 ACK。 */
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
        break;
    case CMD_MOTOR_TEST:
        if (runtime_state_get_arm_state() != ARM_STATE_DISARMED) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_DISARM_REQUIRED);
            break;
        }
        if (req.arg_u8 >= MOTOR_COUNT) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_INVALID_ARGUMENT);
            break;
        }
        if (!console_value_is_finite_in_range(req.arg_f32, 0.0f, 1.0f)) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_INVALID_ARGUMENT);
            break;
        }
        if (req.arg_f32 <= 0.0f) {
            runtime_state_set_motor_test(-1, 0.0f);
            motor_stop_all();
        } else {
            console_stop_active_control(false);
            runtime_state_set_motor_test(req.arg_u8, req.arg_f32);
        }
        console_send_cmd_resp(req.cmd_id, CMD_STATUS_OK);
        break;
    case CMD_AXIS_TEST: {
        axis3f_t request = {0};
        if (runtime_state_get_arm_state() != ARM_STATE_DISARMED) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_DISARM_REQUIRED);
            break;
        }
        if (!console_axis_test_value_is_valid(req.arg_f32)) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_INVALID_ARGUMENT);
            break;
        }
        if (!console_decode_axis_request(req.arg_u8, req.arg_f32, &request)) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_INVALID_ARGUMENT);
            break;
        }
        runtime_state_set_motor_test(-1, 0.0f);
        runtime_state_set_rate_setpoint_request((axis3f_t){0});
        console_reset_attitude_state(false);
        runtime_state_set_axis_test_request(request);
        runtime_state_set_control_mode((req.arg_f32 == 0.0f) ? CONTROL_MODE_IDLE : CONTROL_MODE_AXIS_TEST);
        console_send_cmd_resp(req.cmd_id, CMD_STATUS_OK);
        break;
    }
    case CMD_RATE_TEST: {
        axis3f_t request = {0};
        imu_sample_t sample = {0};

        if (!console_rate_test_value_is_valid(req.arg_f32)) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_INVALID_ARGUMENT);
            break;
        }
        if (!console_decode_axis_request(req.arg_u8, req.arg_f32, &request)) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_INVALID_ARGUMENT);
            break;
        }
        if (req.arg_f32 != 0.0f && runtime_state_get_arm_state() != ARM_STATE_ARMED) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_ARM_REQUIRED);
            break;
        }
        if (req.arg_f32 != 0.0f &&
            (!imu_get_latest(&sample, NULL) || sample.health != IMU_HEALTH_OK || !sample.has_gyro_acc)) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_IMU_NOT_READY);
            break;
        }
        runtime_state_set_motor_test(-1, 0.0f);
        runtime_state_set_axis_test_request((axis3f_t){0});
        console_reset_attitude_state(false);
        runtime_state_set_rate_setpoint_request(request);
        runtime_state_set_control_mode((req.arg_f32 == 0.0f) ? CONTROL_MODE_IDLE : CONTROL_MODE_RATE_TEST);
        console_send_cmd_resp(req.cmd_id, CMD_STATUS_OK);
        break;
    }
    case CMD_ATTITUDE_CAPTURE_REF: {
        imu_sample_t sample = {0};
        const arm_state_t arm_state = runtime_state_get_arm_state();

        if (runtime_state_get_control_mode() != CONTROL_MODE_IDLE || console_has_active_motor_test()) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_CONFLICT);
            break;
        }
        if (arm_state == ARM_STATE_FAILSAFE || arm_state == ARM_STATE_FAULT_LOCK) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_REJECTED);
            break;
        }
        if (!imu_get_latest(&sample, NULL) || !console_sample_supports_attitude_ref(&sample)) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_IMU_NOT_READY);
            break;
        }
        if (!attitude_bench_capture_reference(&sample)) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_IMU_NOT_READY);
            break;
        }
        console_send_cmd_resp(req.cmd_id, CMD_STATUS_OK);
        break;
    }
    case CMD_ATTITUDE_TEST_START: {
        imu_sample_t sample = {0};

        if (runtime_state_get_control_mode() != CONTROL_MODE_IDLE || console_has_active_motor_test()) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_CONFLICT);
            break;
        }
        if (runtime_state_get_arm_state() != ARM_STATE_ARMED) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_ARM_REQUIRED);
            break;
        }
        if (!runtime_state_get_attitude_hang_state().ref_valid) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_REF_REQUIRED);
            break;
        }
        if (!imu_get_latest(&sample, NULL) || !console_sample_supports_attitude_ref(&sample)) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_IMU_NOT_READY);
            break;
        }
        runtime_state_set_motor_test(-1, 0.0f);
        runtime_state_set_axis_test_request((axis3f_t){0});
        runtime_state_set_rate_setpoint_request((axis3f_t){0});
        console_reset_attitude_state(false);
        runtime_state_set_control_mode(CONTROL_MODE_ATTITUDE_HANG_TEST);
        console_send_cmd_resp(req.cmd_id, CMD_STATUS_OK);
        break;
    }
    case CMD_ATTITUDE_TEST_STOP:
        runtime_state_set_motor_test(-1, 0.0f);
        console_stop_active_control(false);
        motor_stop_all();
        console_send_cmd_resp(req.cmd_id, CMD_STATUS_OK);
        break;
    case CMD_GROUND_CAPTURE_REF: {
        imu_sample_t sample = {0};
        estimator_state_t estimator_state = {0};
        const arm_state_t arm_state = runtime_state_get_arm_state();

        if (runtime_state_get_control_mode() != CONTROL_MODE_IDLE || console_has_active_motor_test()) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_CONFLICT);
            break;
        }
        if (arm_state == ARM_STATE_FAILSAFE || arm_state == ARM_STATE_FAULT_LOCK) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_REJECTED);
            break;
        }
        if (!imu_get_latest(&sample, NULL) || !console_sample_supports_attitude_ref(&sample)) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_IMU_NOT_READY);
            break;
        }
        if (!estimator_get_latest(&estimator_state) || estimator_state.timestamp_us != sample.timestamp_us) {
            estimator_update_from_imu(&sample, &estimator_state);
        }
        if (!ground_tune_capture_reference(&sample, &estimator_state)) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_IMU_NOT_READY);
            break;
        }
        controller_reset();
        console_send_event_text("ground ref captured");
        console_send_cmd_resp(req.cmd_id, CMD_STATUS_OK);
        break;
    }
    case CMD_GROUND_TEST_START: {
        imu_sample_t sample = {0};
        estimator_state_t estimator_state = {0};

        if (runtime_state_get_control_mode() != CONTROL_MODE_IDLE || console_has_active_motor_test()) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_CONFLICT);
            break;
        }
        if (runtime_state_get_arm_state() != ARM_STATE_ARMED) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_ARM_REQUIRED);
            break;
        }
        if (!runtime_state_get_ground_tune_state().ref_valid) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_REF_REQUIRED);
            break;
        }
        if (!imu_get_latest(&sample, NULL) || !console_sample_supports_attitude_ref(&sample)) {
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_IMU_NOT_READY);
            break;
        }
        if (!estimator_get_latest(&estimator_state) || estimator_state.timestamp_us != sample.timestamp_us) {
            estimator_update_from_imu(&sample, &estimator_state);
        }
        if (params_get()->ground_tune_use_kalman_attitude && !estimator_state.kalman_valid) {
            ground_tune_set_trip_reason(GROUND_TUNE_TRIP_KALMAN_INVALID);
            console_send_cmd_resp(req.cmd_id, CMD_STATUS_IMU_NOT_READY);
            break;
        }
        if (req.arg_f32 > 0.0f) {
            param_value_t value = {.f32 = req.arg_f32};
            if (!params_try_set("ground_test_base_duty", value, PARAM_TYPE_FLOAT)) {
                console_send_cmd_resp(req.cmd_id, CMD_STATUS_INVALID_ARGUMENT);
                break;
            }
        }
        runtime_state_set_motor_test(-1, 0.0f);
        runtime_state_set_axis_test_request((axis3f_t){0});
        runtime_state_set_rate_setpoint_request((axis3f_t){0});
        ground_tune_reset_status();
        controller_reset();
        runtime_state_set_control_mode(CONTROL_MODE_ATTITUDE_GROUND_TUNE);
        console_send_event_text("ground tune started");
        console_send_cmd_resp(req.cmd_id, CMD_STATUS_OK);
        break;
    }
    case CMD_GROUND_TEST_STOP:
        runtime_state_set_motor_test(-1, 0.0f);
        runtime_state_set_control_mode(CONTROL_MODE_IDLE);
        runtime_state_set_axis_test_request((axis3f_t){0});
        runtime_state_set_rate_setpoint_request((axis3f_t){0});
        ground_tune_set_trip_reason(GROUND_TUNE_TRIP_STOP_NORMAL);
        motor_stop_all();
        console_send_event_text("ground tune stopped normally");
        console_send_cmd_resp(req.cmd_id, CMD_STATUS_OK);
        break;
    case CMD_UDP_MANUAL_ENABLE:
        console_send_cmd_resp(req.cmd_id, udp_manual_enable());
        break;
    case CMD_UDP_MANUAL_DISABLE:
        console_send_cmd_resp(req.cmd_id, udp_manual_disable());
        break;
    case CMD_UDP_TAKEOFF:
        console_send_cmd_resp(req.cmd_id, udp_manual_takeoff());
        break;
    case CMD_UDP_LAND:
        console_send_cmd_resp(req.cmd_id, udp_manual_land());
        break;
    case CMD_UDP_MANUAL_STOP:
        console_send_cmd_resp(req.cmd_id, udp_manual_stop());
        break;
    case CMD_CALIB_GYRO:
        console_send_cmd_resp(req.cmd_id, (imu_calibrate_gyro() == ESP_OK) ? CMD_STATUS_OK : CMD_STATUS_IMU_NOT_READY);
        break;
    case CMD_CALIB_LEVEL:
        console_send_cmd_resp(req.cmd_id, (imu_calibrate_level() == ESP_OK) ? CMD_STATUS_OK : CMD_STATUS_IMU_NOT_READY);
        break;
    default:
        console_send_cmd_resp(req.cmd_id, CMD_STATUS_UNSUPPORTED);
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
        console_hello_resp_t resp = {
            .protocol_version = CONSOLE_PROTOCOL_VERSION,
            .imu_mode = (uint8_t)params_get()->imu_mode,
            .arm_state = (uint8_t)runtime_state_get_arm_state(),
            .stream_enabled = runtime_state_get_stream_enabled() ? 1u : 0u,
            .feature_bitmap = CONSOLE_FEATURE_BITMAP_CURRENT,
        };
        snprintf(resp.build_git_hash, sizeof(resp.build_git_hash), "%s", ESP_DRONE_BUILD_GIT_HASH);
        snprintf(resp.build_time_utc, sizeof(resp.build_time_utc), "%s", ESP_DRONE_BUILD_TIME_UTC);
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
    case MSG_UDP_MANUAL_SETPOINT: {
        if (len < sizeof(console_udp_manual_setpoint_t)) {
            console_send_cmd_resp(CMD_UDP_MANUAL_SETPOINT, CMD_STATUS_INVALID_ARGUMENT);
            break;
        }
        console_udp_manual_setpoint_t setpoint = {0};
        memcpy(&setpoint, payload, sizeof(setpoint));
        console_send_cmd_resp(
            CMD_UDP_MANUAL_SETPOINT,
            udp_manual_setpoint(setpoint.throttle, setpoint.pitch, setpoint.roll, setpoint.yaw));
        break;
    }
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
                            uint32_t sample_seq,
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
    const attitude_hang_state_t hang_state = runtime_state_get_attitude_hang_state();
    const ground_tune_state_t ground_state = runtime_state_get_ground_tune_state();
    estimator_state_t estimator_state = {0};
    if (!estimator_get_latest(&estimator_state) || estimator_state.timestamp_us != imu_sample->timestamp_us) {
        estimator_state.timestamp_us = imu_sample->timestamp_us;
        estimator_state.raw_gyro_body_xyz_dps = imu_sample->gyro_xyz_dps;
        estimator_state.filtered_gyro_body_xyz_dps = imu_sample->gyro_xyz_dps;
        estimator_state.raw_acc_body_xyz_g = imu_sample->acc_xyz_g;
        estimator_state.filtered_acc_body_xyz_g = imu_sample->acc_xyz_g;
        estimator_state.raw_rate_rpy_dps = estimator_project_rates_from_body_gyro(imu_sample->gyro_xyz_dps);
        estimator_state.filtered_rate_rpy_dps = estimator_state.raw_rate_rpy_dps;
        estimator_state.raw_attitude_rpy_deg = imu_sample->roll_pitch_yaw_deg;
        estimator_state.raw_quat_body_to_world = imu_sample->quat_wxyz;
        estimator_state.attitude_valid = imu_sample->has_attitude && imu_sample->has_quaternion;
    }
    const bool ground_active = control_mode == CONTROL_MODE_ATTITUDE_GROUND_TUNE;
    const bool reference_valid = ground_active ? ground_state.ref_valid : hang_state.ref_valid;

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
        .attitude_err_roll_deg = ground_active ? ground_state.err_roll_deg : hang_state.err_roll_deg,
        .attitude_err_pitch_deg = ground_active ? ground_state.err_pitch_deg : hang_state.err_pitch_deg,
        .attitude_rate_sp_roll = ground_active ? ground_state.rate_sp_roll_dps : hang_state.rate_sp_roll_dps,
        .attitude_rate_sp_pitch = ground_active ? ground_state.rate_sp_pitch_dps : hang_state.rate_sp_pitch_dps,
        .attitude_ref_qw = ground_active ? ground_state.ref_q_body_to_world.w : hang_state.ref_q_body_to_world.w,
        .attitude_ref_qx = ground_active ? ground_state.ref_q_body_to_world.x : hang_state.ref_q_body_to_world.x,
        .attitude_ref_qy = ground_active ? ground_state.ref_q_body_to_world.y : hang_state.ref_q_body_to_world.y,
        .attitude_ref_qz = ground_active ? ground_state.ref_q_body_to_world.z : hang_state.ref_q_body_to_world.z,
        .base_duty_active = ground_active ? ground_state.base_duty_active : hang_state.base_duty_active,
        .attitude_ref_valid = reference_valid ? 1u : 0u,
        .attitude_reserved = {0, 0, 0},
        .filtered_gyro_x = estimator_state.filtered_gyro_body_xyz_dps.x,
        .filtered_gyro_y = estimator_state.filtered_gyro_body_xyz_dps.y,
        .filtered_gyro_z = estimator_state.filtered_gyro_body_xyz_dps.z,
        .filtered_acc_x = estimator_state.filtered_acc_body_xyz_g.x,
        .filtered_acc_y = estimator_state.filtered_acc_body_xyz_g.y,
        .filtered_acc_z = estimator_state.filtered_acc_body_xyz_g.z,
        .kalman_roll_deg = estimator_state.kalman_roll_deg,
        .kalman_pitch_deg = estimator_state.kalman_pitch_deg,
        .rate_meas_roll_raw = estimator_state.raw_rate_rpy_dps.roll,
        .rate_meas_pitch_raw = estimator_state.raw_rate_rpy_dps.pitch,
        .rate_meas_yaw_raw = estimator_state.raw_rate_rpy_dps.yaw,
        .rate_meas_roll_filtered = estimator_state.filtered_rate_rpy_dps.roll,
        .rate_meas_pitch_filtered = estimator_state.filtered_rate_rpy_dps.pitch,
        .rate_meas_yaw_filtered = estimator_state.filtered_rate_rpy_dps.yaw,
        .rate_err_roll = rate_status.error.roll,
        .rate_err_pitch = rate_status.error.pitch,
        .rate_err_yaw = rate_status.error.yaw,
        .sample_seq = sample_seq,
        .attitude_valid = estimator_state.attitude_valid ? 1u : 0u,
        .kalman_valid = estimator_state.kalman_valid ? 1u : 0u,
        .motor_saturation_flag = rate_status.motor_saturation ? 1u : 0u,
        .integrator_freeze_flag = rate_status.integrator_freeze ? 1u : 0u,
        .ground_ref_valid = ground_state.ref_valid ? 1u : 0u,
        .reference_valid = reference_valid ? 1u : 0u,
        .ground_trip_reason = (uint8_t)ground_state.trip_reason,
        .telemetry_reserved = 0u,
    };

    console_send_frame(MSG_TELEMETRY_SAMPLE, &sample, sizeof(sample));
}
