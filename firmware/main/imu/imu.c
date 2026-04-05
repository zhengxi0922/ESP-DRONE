/**
 * @file imu.c
 * @brief ATK-MS901M 接收、映射与样本发布实现。
 */

#include "imu.h"

#include <math.h>
#include <stdatomic.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_timer.h"

#include "barometer.h"
#include "board_config.h"
#include "params.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define IMU_FRAME_HEAD_LOW 0x55
#define IMU_FRAME_HEAD_UPLOAD_HIGH 0x55
#define IMU_FRAME_HEAD_ACK_HIGH 0xAF
#define IMU_MAX_PAYLOAD_LEN 28

#define IMU_FRAME_ATTITUDE 0x01
#define IMU_FRAME_QUATERNION 0x02
#define IMU_FRAME_GYRO_ACC 0x03
#define IMU_FRAME_MAG 0x04
#define IMU_FRAME_BARO 0x05

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
    quatf_t quat_module;
    vec3f_t gyro_module;
    vec3f_t acc_module;
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
static vec3f_t s_gyro_bias_body_dps;
static eulerf_t s_level_trim_deg;

typedef struct {
    float m[3][3];
} mat3f_t;

static int16_t imu_read_s16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static int32_t imu_read_s32_le(const uint8_t *data)
{
    return (int32_t)((uint32_t)data[0] |
                     ((uint32_t)data[1] << 8) |
                     ((uint32_t)data[2] << 16) |
                     ((uint32_t)data[3] << 24));
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

static float imu_deg_from_rad(float rad)
{
    return rad * (180.0f / (float)M_PI);
}

static bool imu_map_is_identity(void)
{
    const params_store_t *params = params_get();
    return params->imu_map_x == BODY_AXIS_POS_X &&
           params->imu_map_y == BODY_AXIS_POS_Y &&
           params->imu_map_z == BODY_AXIS_POS_Z;
}

static float imu_select_axis_component(const vec3f_t *module_vec, body_axis_selector_t selector)
{
    switch (selector) {
    case BODY_AXIS_POS_X:
        return module_vec->x;
    case BODY_AXIS_NEG_X:
        return -module_vec->x;
    case BODY_AXIS_POS_Y:
        return module_vec->y;
    case BODY_AXIS_NEG_Y:
        return -module_vec->y;
    case BODY_AXIS_POS_Z:
        return module_vec->z;
    case BODY_AXIS_NEG_Z:
        return -module_vec->z;
    default:
        return 0.0f;
    }
}

static vec3f_t imu_map_module_vec_to_body(const vec3f_t *module_vec)
{
    const params_store_t *params = params_get();
    return (vec3f_t){
        .x = imu_select_axis_component(module_vec, params->imu_map_x),
        .y = imu_select_axis_component(module_vec, params->imu_map_y),
        .z = imu_select_axis_component(module_vec, params->imu_map_z),
    };
}

static void imu_fill_basis_row(float row[3], body_axis_selector_t selector)
{
    row[0] = 0.0f;
    row[1] = 0.0f;
    row[2] = 0.0f;
    switch (selector) {
    case BODY_AXIS_POS_X:
        row[0] = 1.0f;
        break;
    case BODY_AXIS_NEG_X:
        row[0] = -1.0f;
        break;
    case BODY_AXIS_POS_Y:
        row[1] = 1.0f;
        break;
    case BODY_AXIS_NEG_Y:
        row[1] = -1.0f;
        break;
    case BODY_AXIS_POS_Z:
        row[2] = 1.0f;
        break;
    case BODY_AXIS_NEG_Z:
        row[2] = -1.0f;
        break;
    }
}

static mat3f_t imu_module_to_body_basis_matrix(void)
{
    const params_store_t *params = params_get();
    mat3f_t basis = {0};
    /* 这里构造的是“body 轴在 module 坐标中的基底矩阵”。
     * 后续 quaternion 和姿态映射都依赖这一个方向真源。 */
    imu_fill_basis_row(basis.m[0], params->imu_map_x);
    imu_fill_basis_row(basis.m[1], params->imu_map_y);
    imu_fill_basis_row(basis.m[2], params->imu_map_z);
    return basis;
}

static mat3f_t imu_mat3_transpose(mat3f_t input)
{
    mat3f_t out = {0};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.m[r][c] = input.m[c][r];
        }
    }
    return out;
}

static mat3f_t imu_mat3_mul(mat3f_t a, mat3f_t b)
{
    mat3f_t out = {0};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            float value = 0.0f;
            for (int k = 0; k < 3; ++k) {
                value += a.m[r][k] * b.m[k][c];
            }
            out.m[r][c] = value;
        }
    }
    return out;
}

static quatf_t imu_quat_normalize(quatf_t q)
{
    const float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm <= 1e-6f) {
        return (quatf_t){1.0f, 0.0f, 0.0f, 0.0f};
    }
    return (quatf_t){
        .w = q.w / norm,
        .x = q.x / norm,
        .y = q.y / norm,
        .z = q.z / norm,
    };
}

static mat3f_t imu_mat3_from_quat(quatf_t q)
{
    q = imu_quat_normalize(q);
    const float ww = q.w * q.w;
    const float xx = q.x * q.x;
    const float yy = q.y * q.y;
    const float zz = q.z * q.z;
    const float wx = q.w * q.x;
    const float wy = q.w * q.y;
    const float wz = q.w * q.z;
    const float xy = q.x * q.y;
    const float xz = q.x * q.z;
    const float yz = q.y * q.z;

    return (mat3f_t){
        .m = {
            {ww + xx - yy - zz, 2.0f * (xy - wz), 2.0f * (xz + wy)},
            {2.0f * (xy + wz), ww - xx + yy - zz, 2.0f * (yz - wx)},
            {2.0f * (xz - wy), 2.0f * (yz + wx), ww - xx - yy + zz},
        },
    };
}

static quatf_t imu_quat_from_mat3(mat3f_t m)
{
    quatf_t q = {0};
    const float trace = m.m[0][0] + m.m[1][1] + m.m[2][2];
    if (trace > 0.0f) {
        const float s = sqrtf(trace + 1.0f) * 2.0f;
        q.w = 0.25f * s;
        q.x = (m.m[2][1] - m.m[1][2]) / s;
        q.y = (m.m[0][2] - m.m[2][0]) / s;
        q.z = (m.m[1][0] - m.m[0][1]) / s;
    } else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2]) {
        const float s = sqrtf(1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2]) * 2.0f;
        q.w = (m.m[2][1] - m.m[1][2]) / s;
        q.x = 0.25f * s;
        q.y = (m.m[0][1] + m.m[1][0]) / s;
        q.z = (m.m[0][2] + m.m[2][0]) / s;
    } else if (m.m[1][1] > m.m[2][2]) {
        const float s = sqrtf(1.0f + m.m[1][1] - m.m[0][0] - m.m[2][2]) * 2.0f;
        q.w = (m.m[0][2] - m.m[2][0]) / s;
        q.x = (m.m[0][1] + m.m[1][0]) / s;
        q.y = 0.25f * s;
        q.z = (m.m[1][2] + m.m[2][1]) / s;
    } else {
        const float s = sqrtf(1.0f + m.m[2][2] - m.m[0][0] - m.m[1][1]) * 2.0f;
        q.w = (m.m[1][0] - m.m[0][1]) / s;
        q.x = (m.m[0][2] + m.m[2][0]) / s;
        q.y = (m.m[1][2] + m.m[2][1]) / s;
        q.z = 0.25f * s;
    }
    return imu_quat_normalize(q);
}

static void imu_apply_runtime_calibration(imu_sample_t *sample)
{
    if (sample == NULL) {
        return;
    }

    sample->gyro_xyz_dps.x -= s_gyro_bias_body_dps.x;
    sample->gyro_xyz_dps.y -= s_gyro_bias_body_dps.y;
    sample->gyro_xyz_dps.z -= s_gyro_bias_body_dps.z;

    if (sample->has_attitude) {
        sample->roll_pitch_yaw_deg.roll_deg =
            imu_norm_angle(sample->roll_pitch_yaw_deg.roll_deg - s_level_trim_deg.roll_deg);
        sample->roll_pitch_yaw_deg.pitch_deg =
            imu_norm_angle(sample->roll_pitch_yaw_deg.pitch_deg - s_level_trim_deg.pitch_deg);
        sample->roll_pitch_yaw_deg.yaw_deg =
            imu_norm_angle(sample->roll_pitch_yaw_deg.yaw_deg - s_level_trim_deg.yaw_deg);
    }
}

static eulerf_t imu_project_euler_from_body_to_world(mat3f_t r_body_to_world)
{
    const float pitch = atan2f(r_body_to_world.m[2][1], r_body_to_world.m[2][2]);
    float sin_theta = -r_body_to_world.m[2][0];
    if (sin_theta > 1.0f) {
        sin_theta = 1.0f;
    } else if (sin_theta < -1.0f) {
        sin_theta = -1.0f;
    }
    const float theta = asinf(sin_theta);
    const float psi = atan2f(r_body_to_world.m[1][0], r_body_to_world.m[0][0]);

    return (eulerf_t){
        .roll_deg = imu_norm_angle(imu_deg_from_rad(-theta)),
        .pitch_deg = imu_norm_angle(imu_deg_from_rad(pitch)),
        .yaw_deg = imu_norm_angle(imu_deg_from_rad(-psi)),
    };
}

static bool imu_map_module_quat_to_body(quatf_t module_quat, quatf_t *out_body_quat, eulerf_t *out_body_rpy)
{
    const mat3f_t body_from_module = imu_module_to_body_basis_matrix();
    const mat3f_t module_from_body = imu_mat3_transpose(body_from_module);
    const mat3f_t world_from_module = imu_mat3_from_quat(module_quat);
    const mat3f_t world_from_body = imu_mat3_mul(world_from_module, module_from_body);

    if (out_body_quat != NULL) {
        *out_body_quat = imu_quat_from_mat3(world_from_body);
    }
    if (out_body_rpy != NULL) {
        *out_body_rpy = imu_project_euler_from_body_to_world(world_from_body);
    }
    return true;
}

/* “ATK-MS901M 模块坐标 -> 项目机体系”的唯一方向真源。
 * 所有方向敏感的 gyro/acc、四元数和姿态映射都必须经过这里，
 * 以保持与 docs/axis_truth_table.md 一致。 */
static void imu_map_partial_to_project_sample(const partial_sample_t *partial, imu_sample_t *out_sample)
{
    if (partial == NULL || out_sample == NULL) {
        return;
    }

    out_sample->gyro_xyz_dps = imu_map_module_vec_to_body(&partial->gyro_module);
    out_sample->acc_xyz_g = imu_map_module_vec_to_body(&partial->acc_module);
    out_sample->has_gyro_acc = partial->have_gyro_acc;
    out_sample->has_quaternion = false;
    out_sample->has_attitude = false;

    if (partial->have_quat) {
        imu_map_module_quat_to_body(partial->quat_module, &out_sample->quat_wxyz, &out_sample->roll_pitch_yaw_deg);
        out_sample->has_quaternion = true;
        out_sample->has_attitude = true;
        return;
    }

    if (!imu_map_is_identity()) {
        return;
    }

    /* 模块姿态角自身的符号约定不能直接当成项目 roll/yaw 真值。
     * 只有在轴映射为恒等且四元数缺失时，才临时退回这条 bring-up 路径。 */
    out_sample->roll_pitch_yaw_deg.roll_deg = imu_norm_angle(-partial->roll_deg);
    out_sample->roll_pitch_yaw_deg.pitch_deg = imu_norm_angle(partial->pitch_deg);
    out_sample->roll_pitch_yaw_deg.yaw_deg = imu_norm_angle(-partial->yaw_deg);
    out_sample->has_attitude = true;
}

static uint8_t imu_return_rate_code_from_params(void)
{
    const uint32_t code = params_get()->imu_return_rate_code;
    return (uint8_t)((code > 0x09u) ? 0x01u : code);
}

static uint8_t imu_return_set_from_params(void)
{
    const params_store_t *params = params_get();
    const uint8_t baro_bit = 0x10u;
    if (params->imu_mode == IMU_MODE_RAW) {
        /* RAW 模式继续以 gyro/acc 为主，但当前阶段需要一并请求 baro 帧，
         * 以便打通“模块气压计 -> firmware -> telemetry -> host”链路。 */
        return (params->imu_mag_enable ? 0x0Cu : 0x04u) | baro_bit;
    }
    /* DIRECT 模式仍需请求 gyro/acc 帧，这样遥测可输出实时原始量，
     * 单轴速率测试有新鲜反馈，台架陀螺校准也无需切回 RAW。
     * 帧位图：0x01 attitude，0x02 quaternion，0x04 gyro+acc，0x08 mag。 */
    return (params->imu_mag_enable ? 0x0Fu : 0x07u) | baro_bit;
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
    if (frame_id == IMU_FRAME_BARO) {
        if (len >= 10) {
            const int32_t pressure_pa = imu_read_s32_le(&payload[0]);
            const int32_t altitude_cm = imu_read_s32_le(&payload[4]);
            const float temperature_c = (float)imu_read_s16_le(&payload[8]) / 100.0f;
            barometer_update_from_module_frame(pressure_pa,
                                               altitude_cm,
                                               temperature_c,
                                               (uint64_t)esp_timer_get_time());
            atomic_fetch_add(&s_good_frames, 1u);
        }
        return;
    }

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
        }
        break;
    case IMU_FRAME_QUATERNION:
        if (len >= 8) {
            s_partial.quat_module.w = (float)imu_read_s16_le(&payload[0]) / 32768.0f;
            s_partial.quat_module.x = (float)imu_read_s16_le(&payload[2]) / 32768.0f;
            s_partial.quat_module.y = (float)imu_read_s16_le(&payload[4]) / 32768.0f;
            s_partial.quat_module.z = (float)imu_read_s16_le(&payload[6]) / 32768.0f;
            s_partial.have_quat = true;
        }
        break;
    case IMU_FRAME_GYRO_ACC:
        if (len >= 12) {
            s_partial.acc_module.x = ((float)imu_read_s16_le(&payload[0]) / 32768.0f) * 4.0f;
            s_partial.acc_module.y = ((float)imu_read_s16_le(&payload[2]) / 32768.0f) * 4.0f;
            s_partial.acc_module.z = ((float)imu_read_s16_le(&payload[4]) / 32768.0f) * 4.0f;
            s_partial.gyro_module.x = ((float)imu_read_s16_le(&payload[6]) / 32768.0f) * 2000.0f;
            s_partial.gyro_module.y = ((float)imu_read_s16_le(&payload[8]) / 32768.0f) * 2000.0f;
            s_partial.gyro_module.z = ((float)imu_read_s16_le(&payload[10]) / 32768.0f) * 2000.0f;
            s_partial.have_gyro_acc = true;
        }
        break;
    case IMU_FRAME_MAG:
    default:
        break;
    }

    imu_map_partial_to_project_sample(&s_partial, &sample);
    imu_apply_runtime_calibration(&sample);

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
    s_gyro_bias_body_dps = (vec3f_t){0};
    s_level_trim_deg = (eulerf_t){0};
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

esp_err_t imu_calibrate_gyro(void)
{
    imu_sample_t sample = {0};
    if (!imu_get_latest(&sample, NULL) || sample.health != IMU_HEALTH_OK || !sample.has_gyro_acc) {
        return ESP_ERR_INVALID_STATE;
    }

    s_gyro_bias_body_dps.x += sample.gyro_xyz_dps.x;
    s_gyro_bias_body_dps.y += sample.gyro_xyz_dps.y;
    s_gyro_bias_body_dps.z += sample.gyro_xyz_dps.z;
    return ESP_OK;
}

esp_err_t imu_calibrate_level(void)
{
    imu_sample_t sample = {0};
    if (!imu_get_latest(&sample, NULL) || sample.health != IMU_HEALTH_OK || !sample.has_attitude) {
        return ESP_ERR_INVALID_STATE;
    }

    s_level_trim_deg.roll_deg += sample.roll_pitch_yaw_deg.roll_deg;
    s_level_trim_deg.pitch_deg += sample.roll_pitch_yaw_deg.pitch_deg;
    s_level_trim_deg.yaw_deg = 0.0f;
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
        if (age_us > (params_get()->imu_timeout_ms * 1000u)) {
            sample.health = IMU_HEALTH_TIMEOUT;
        } else if (atomic_load(&s_consecutive_parse_errors) >= params_get()->imu_parse_error_limit) {
            sample.health = IMU_HEALTH_PARSE_ERROR;
        } else if (params_get()->imu_mode == IMU_MODE_DIRECT && !sample.has_quaternion) {
            sample.health = IMU_HEALTH_DEGRADED;
        } else {
            sample.health = IMU_HEALTH_OK;
        }
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
