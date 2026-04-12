#include "udp_protocol.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "console_protocol.h"
#include "controller.h"
#include "imu.h"
#include "motor.h"
#include "params.h"
#include "runtime_state.h"
#include "safety.h"
#include "udp_manual.h"

#define UDP_PROTOCOL_PORT 2391
#define UDP_PROTOCOL_RX_BUF_SIZE 512
#define UDP_PROTOCOL_FRAME_BUF_SIZE 384

#ifndef ESP_DRONE_BUILD_GIT_HASH
#define ESP_DRONE_BUILD_GIT_HASH "unknown"
#endif

#ifndef ESP_DRONE_BUILD_TIME_UTC
#define ESP_DRONE_BUILD_TIME_UTC "unknown"
#endif

static portMUX_TYPE s_udp_protocol_lock = portMUX_INITIALIZER_UNLOCKED;
static int s_udp_socket = -1;
static struct sockaddr_storage s_stream_client;
static socklen_t s_stream_client_len;
static bool s_stream_client_valid;
static bool s_udp_stream_enabled;
static uint16_t s_udp_tx_seq;

static uint16_t udp_crc16_ccitt(const uint8_t *data, size_t len)
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

static bool udp_send_frame_to(int sock,
                              const struct sockaddr_storage *addr,
                              socklen_t addr_len,
                              uint8_t msg_type,
                              const void *payload,
                              uint16_t payload_len)
{
    uint8_t raw_buf[UDP_PROTOCOL_FRAME_BUF_SIZE];

    if (sock < 0 || addr == NULL ||
        payload_len + sizeof(console_frame_header_t) + sizeof(uint16_t) > sizeof(raw_buf)) {
        return false;
    }

    console_frame_header_t header = {
        .magic = CONSOLE_FRAME_MAGIC,
        .version = CONSOLE_FRAME_VERSION,
        .msg_type = msg_type,
        .flags = 0,
        .seq = s_udp_tx_seq++,
        .payload_len = payload_len,
    };

    memcpy(raw_buf, &header, sizeof(header));
    if (payload_len > 0 && payload != NULL) {
        memcpy(raw_buf + sizeof(header), payload, payload_len);
    }

    const uint16_t crc = udp_crc16_ccitt(raw_buf, sizeof(header) + payload_len);
    memcpy(raw_buf + sizeof(header) + payload_len, &crc, sizeof(crc));
    const size_t raw_len = sizeof(header) + payload_len + sizeof(crc);

    return sendto(sock, raw_buf, raw_len, 0, (const struct sockaddr *)addr, addr_len) == (ssize_t)raw_len;
}

static void udp_send_cmd_resp_to(int sock,
                                 const struct sockaddr_storage *addr,
                                 socklen_t addr_len,
                                 uint8_t cmd_id,
                                 console_cmd_status_t status)
{
    const console_cmd_resp_t resp = {
        .cmd_id = cmd_id,
        .status = (uint8_t)status,
        .reserved = 0,
    };
    udp_send_frame_to(sock, addr, addr_len, MSG_CMD_RESP, &resp, sizeof(resp));
}

static void udp_send_param_value_to(int sock,
                                    const struct sockaddr_storage *addr,
                                    socklen_t addr_len,
                                    const char *name)
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

    udp_send_frame_to(sock, addr, addr_len, MSG_PARAM_VALUE, payload, (uint16_t)(offset + value_len));
}

static console_cmd_status_t udp_handle_command(uint8_t cmd_id)
{
    switch ((console_cmd_id_t)cmd_id) {
    case CMD_ARM:
        runtime_state_set_motor_test(-1, 0.0f);
        return safety_request_arm(true) ? CMD_STATUS_OK : CMD_STATUS_REJECTED;
    case CMD_DISARM:
        safety_request_disarm();
        runtime_state_set_motor_test(-1, 0.0f);
        runtime_state_set_axis_test_request((axis3f_t){0});
        runtime_state_set_rate_setpoint_request((axis3f_t){0});
        runtime_state_set_control_mode(CONTROL_MODE_IDLE);
        udp_manual_reset();
        motor_stop_all();
        return CMD_STATUS_OK;
    case CMD_KILL:
        safety_request_kill();
        runtime_state_set_motor_test(-1, 0.0f);
        runtime_state_set_axis_test_request((axis3f_t){0});
        runtime_state_set_rate_setpoint_request((axis3f_t){0});
        runtime_state_set_control_mode(CONTROL_MODE_IDLE);
        udp_manual_reset();
        motor_stop_all();
        return CMD_STATUS_OK;
    case CMD_UDP_MANUAL_ENABLE:
        return udp_manual_enable();
    case CMD_UDP_MANUAL_DISABLE:
        return udp_manual_disable();
    case CMD_UDP_TAKEOFF:
        return udp_manual_takeoff();
    case CMD_UDP_LAND:
        return udp_manual_land();
    case CMD_UDP_MANUAL_STOP:
        return udp_manual_stop();
    default:
        return CMD_STATUS_UNSUPPORTED;
    }
}

static bool udp_decode_frame(const uint8_t *data,
                             size_t len,
                             console_frame_header_t *out_header,
                             const uint8_t **out_payload)
{
    if (data == NULL || out_header == NULL || out_payload == NULL ||
        len < sizeof(console_frame_header_t) + sizeof(uint16_t)) {
        return false;
    }

    memcpy(out_header, data, sizeof(*out_header));
    if (out_header->magic != CONSOLE_FRAME_MAGIC || out_header->version != CONSOLE_FRAME_VERSION) {
        return false;
    }

    const size_t expected_len = sizeof(*out_header) + out_header->payload_len + sizeof(uint16_t);
    if (len != expected_len) {
        return false;
    }

    uint16_t expected_crc = 0;
    memcpy(&expected_crc, data + len - sizeof(uint16_t), sizeof(expected_crc));
    if (expected_crc != udp_crc16_ccitt(data, len - sizeof(uint16_t))) {
        return false;
    }

    *out_payload = data + sizeof(*out_header);
    return true;
}

static void udp_record_stream_client(const struct sockaddr_storage *addr, socklen_t addr_len, bool enabled)
{
    taskENTER_CRITICAL(&s_udp_protocol_lock);
    s_udp_stream_enabled = enabled;
    if (enabled && addr != NULL) {
        memcpy(&s_stream_client, addr, sizeof(*addr));
        s_stream_client_len = addr_len;
        s_stream_client_valid = true;
    }
    if (!enabled) {
        s_stream_client_valid = false;
    }
    taskEXIT_CRITICAL(&s_udp_protocol_lock);
    runtime_state_set_stream_enabled(enabled);
}

static void udp_handle_param_set(int sock,
                                 const struct sockaddr_storage *addr,
                                 socklen_t addr_len,
                                 const uint8_t *payload,
                                 size_t len)
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

    if (params_try_set(name, value, type)) {
        if (strcmp(name, "motor_pwm_freq_hz") == 0) {
            motor_reconfigure_from_params();
        }
        if (strcmp(name, "imu_mode") == 0 ||
            strcmp(name, "imu_return_rate_code") == 0 ||
            strcmp(name, "imu_mag_enable") == 0) {
            imu_reconfigure_from_params();
        }
    }
    udp_send_param_value_to(sock, addr, addr_len, name);
}

static void udp_handle_message(int sock,
                               const struct sockaddr_storage *addr,
                               socklen_t addr_len,
                               uint8_t msg_type,
                               const uint8_t *payload,
                               size_t len)
{
    switch ((console_msg_type_t)msg_type) {
    case MSG_HELLO_REQ: {
        console_hello_resp_t resp = {
            .protocol_version = CONSOLE_PROTOCOL_VERSION,
            .imu_mode = (uint8_t)params_get()->imu_mode,
            .arm_state = (uint8_t)runtime_state_get_arm_state(),
            .stream_enabled = s_udp_stream_enabled ? 1u : 0u,
            .feature_bitmap = CONSOLE_FEATURE_BITMAP_CURRENT,
        };
        snprintf(resp.build_git_hash, sizeof(resp.build_git_hash), "%s", ESP_DRONE_BUILD_GIT_HASH);
        snprintf(resp.build_time_utc, sizeof(resp.build_time_utc), "%s", ESP_DRONE_BUILD_TIME_UTC);
        udp_send_frame_to(sock, addr, addr_len, MSG_HELLO_RESP, &resp, sizeof(resp));
        break;
    }
    case MSG_CMD_REQ: {
        if (len < sizeof(console_cmd_req_t)) {
            return;
        }
        console_cmd_req_t req = {0};
        memcpy(&req, payload, sizeof(req));
        const console_cmd_status_t status = udp_handle_command(req.cmd_id);
        udp_send_cmd_resp_to(sock, addr, addr_len, req.cmd_id, status);
        if (req.cmd_id == CMD_REBOOT && status == CMD_STATUS_OK) {
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
        }
        break;
    }
    case MSG_PARAM_GET:
        if (len >= 1) {
            char name[64] = {0};
            const uint8_t name_len = payload[0];
            if ((size_t)(1 + name_len) <= len && name_len < sizeof(name)) {
                memcpy(name, payload + 1, name_len);
                udp_send_param_value_to(sock, addr, addr_len, name);
            }
        }
        break;
    case MSG_PARAM_SET:
        udp_handle_param_set(sock, addr, addr_len, payload, len);
        break;
    case MSG_PARAM_LIST_REQ:
        for (size_t i = 0; i < params_count(); ++i) {
            const param_descriptor_t *desc = params_get_descriptor(i);
            if (desc != NULL) {
                udp_send_param_value_to(sock, addr, addr_len, desc->name);
            }
        }
        udp_send_frame_to(sock, addr, addr_len, MSG_PARAM_LIST_END, NULL, 0);
        break;
    case MSG_PARAM_SAVE:
        params_save();
        udp_send_frame_to(sock, addr, addr_len, MSG_PARAM_SAVE, NULL, 0);
        break;
    case MSG_PARAM_RESET:
        params_reset_to_defaults();
        motor_reconfigure_from_params();
        imu_reconfigure_from_params();
        udp_send_frame_to(sock, addr, addr_len, MSG_PARAM_RESET, NULL, 0);
        break;
    case MSG_STREAM_CTRL:
        udp_record_stream_client(addr, addr_len, len > 0 && payload[0] != 0);
        udp_send_frame_to(sock, addr, addr_len, MSG_STREAM_CTRL, payload, (uint16_t)len);
        break;
    case MSG_UDP_MANUAL_SETPOINT: {
        if (len < sizeof(console_udp_manual_setpoint_t)) {
            udp_send_cmd_resp_to(sock, addr, addr_len, CMD_UDP_MANUAL_SETPOINT, CMD_STATUS_INVALID_ARGUMENT);
            break;
        }
        console_udp_manual_setpoint_t setpoint = {0};
        memcpy(&setpoint, payload, sizeof(setpoint));
        udp_send_cmd_resp_to(
            sock,
            addr,
            addr_len,
            CMD_UDP_MANUAL_SETPOINT,
            udp_manual_setpoint(setpoint.throttle, setpoint.pitch, setpoint.roll, setpoint.yaw));
        break;
    }
    default:
        break;
    }
}

void udp_protocol_task(void *arg)
{
    (void)arg;

    uint8_t rx_buf[UDP_PROTOCOL_RX_BUF_SIZE];
    while (1) {
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        const struct sockaddr_in bind_addr = {
            .sin_family = AF_INET,
            .sin_port = htons(UDP_PROTOCOL_PORT),
            .sin_addr.s_addr = htonl(INADDR_ANY),
        };

        if (bind(sock, (const struct sockaddr *)&bind_addr, sizeof(bind_addr)) != 0) {
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        taskENTER_CRITICAL(&s_udp_protocol_lock);
        s_udp_socket = sock;
        taskEXIT_CRITICAL(&s_udp_protocol_lock);

        while (1) {
            struct sockaddr_storage source_addr = {0};
            socklen_t source_len = sizeof(source_addr);
            const int rx_len = recvfrom(sock, rx_buf, sizeof(rx_buf), 0, (struct sockaddr *)&source_addr, &source_len);
            if (rx_len < 0) {
                if (errno == EINTR) {
                    continue;
                }
                break;
            }

            console_frame_header_t header = {0};
            const uint8_t *payload = NULL;
            if (!udp_decode_frame(rx_buf, (size_t)rx_len, &header, &payload)) {
                continue;
            }

            udp_handle_message(sock, &source_addr, source_len, header.msg_type, payload, header.payload_len);
        }

        taskENTER_CRITICAL(&s_udp_protocol_lock);
        if (s_udp_socket == sock) {
            s_udp_socket = -1;
            s_stream_client_valid = false;
            s_udp_stream_enabled = false;
        }
        taskEXIT_CRITICAL(&s_udp_protocol_lock);
        close(sock);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void udp_protocol_send_telemetry(const imu_sample_t *imu_sample,
                                 const barometer_state_t *baro_state,
                                 float battery_voltage,
                                 int battery_raw)
{
    if (imu_sample == NULL) {
        return;
    }

    int sock = -1;
    struct sockaddr_storage target = {0};
    socklen_t target_len = 0;
    bool enabled = false;

    taskENTER_CRITICAL(&s_udp_protocol_lock);
    sock = s_udp_socket;
    enabled = s_udp_stream_enabled && s_stream_client_valid;
    if (enabled) {
        memcpy(&target, &s_stream_client, sizeof(target));
        target_len = s_stream_client_len;
    }
    taskEXIT_CRITICAL(&s_udp_protocol_lock);

    if (!enabled || sock < 0) {
        return;
    }

    float motor_outputs[MOTOR_COUNT] = {0};
    motor_get_outputs(motor_outputs);
    const loop_stats_t loop_stats = runtime_state_get_loop_stats();
    const axis3f_t axis_test_request = runtime_state_get_axis_test_request();
    const axis3f_t rate_setpoint_request = runtime_state_get_rate_setpoint_request();
    const rate_controller_status_t rate_status = controller_get_last_rate_status();
    const control_mode_t control_mode = runtime_state_get_control_mode();
    const attitude_hang_state_t attitude_state = runtime_state_get_attitude_hang_state();

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
        .attitude_err_roll_deg = attitude_state.err_roll_deg,
        .attitude_err_pitch_deg = attitude_state.err_pitch_deg,
        .attitude_rate_sp_roll = attitude_state.rate_sp_roll_dps,
        .attitude_rate_sp_pitch = attitude_state.rate_sp_pitch_dps,
        .attitude_ref_qw = attitude_state.ref_q_body_to_world.w,
        .attitude_ref_qx = attitude_state.ref_q_body_to_world.x,
        .attitude_ref_qy = attitude_state.ref_q_body_to_world.y,
        .attitude_ref_qz = attitude_state.ref_q_body_to_world.z,
        .base_duty_active = attitude_state.base_duty_active,
        .attitude_ref_valid = attitude_state.ref_valid ? 1u : 0u,
        .attitude_reserved = {0, 0, 0},
    };

    if (!udp_send_frame_to(sock, &target, target_len, MSG_TELEMETRY_SAMPLE, &sample, sizeof(sample))) {
        taskENTER_CRITICAL(&s_udp_protocol_lock);
        s_udp_stream_enabled = false;
        s_stream_client_valid = false;
        taskEXIT_CRITICAL(&s_udp_protocol_lock);
    }
}
