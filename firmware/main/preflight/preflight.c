#include "preflight.h"

#include <math.h>
#include <stdio.h>

#include "board_config.h"
#include "console_protocol.h"
#include "estimator.h"
#include "esp_err.h"
#include "imu.h"
#include "mixer.h"
#include "motor.h"
#include "params.h"
#include "runtime_state.h"

#ifndef ESP_DRONE_BUILD_GIT_HASH
#define ESP_DRONE_BUILD_GIT_HASH "unknown"
#endif

static void preflight_emit(preflight_report_fn_t report_fn, void *ctx, bool ok, const char *detail)
{
    if (report_fn == NULL || detail == NULL) {
        return;
    }

    char line[120];
    snprintf(line, sizeof(line), "preflight %s %s", ok ? "OK" : "FAIL", detail);
    report_fn(line, ctx);
}

static bool preflight_float_abs_le(float value, float limit)
{
    return isfinite(value) && fabsf(value) <= limit;
}

static bool preflight_motor_output_map_valid(const params_store_t *params)
{
    bool seen[MOTOR_COUNT] = {0};
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
        const uint8_t physical = params->motor_output_map[i];
        if (physical >= MOTOR_COUNT || seen[physical]) {
            return false;
        }
        seen[physical] = true;
    }
    return true;
}

static bool preflight_rate_pid_p_only(const params_store_t *params)
{
    const float eps = 0.0000001f;
    return fabsf(params->rate_ki_roll) <= eps &&
           fabsf(params->rate_kd_roll) <= eps &&
           fabsf(params->rate_ki_pitch) <= eps &&
           fabsf(params->rate_kd_pitch) <= eps &&
           fabsf(params->rate_ki_yaw) <= eps &&
           fabsf(params->rate_kd_yaw) <= eps;
}

bool preflight_check_stabilize_min(preflight_link_t link,
                                   preflight_report_fn_t report_fn,
                                   void *report_ctx)
{
    const params_store_t *params = params_get();
    bool ok = true;

    const bool build_ok = ESP_DRONE_BUILD_GIT_HASH[0] != '\0';
    const bool protocol_ok = CONSOLE_PROTOCOL_VERSION >= 10u;
    const bool capability_ok =
        (CONSOLE_FEATURE_BITMAP_CURRENT & CONSOLE_FEATURE_STABILIZE_MIN) != 0u &&
        (CONSOLE_FEATURE_BITMAP_CURRENT & CONSOLE_FEATURE_PREFLIGHT_CHECK) != 0u;
    char detail[96];
    snprintf(detail,
             sizeof(detail),
             "build/protocol/cap protocol=%u features=0x%08lx build=%s",
             (unsigned)CONSOLE_PROTOCOL_VERSION,
             (unsigned long)CONSOLE_FEATURE_BITMAP_CURRENT,
             ESP_DRONE_BUILD_GIT_HASH);
    preflight_emit(report_fn, report_ctx, build_ok && protocol_ok && capability_ok, detail);
    ok = ok && build_ok && protocol_ok && capability_ok;

    const params_rate_pid_source_t pid_source = params_get_rate_pid_source();
    const bool pid_source_ok =
        pid_source == PARAM_RATE_PID_SOURCE_FIRMWARE_DEFAULT ||
        pid_source == PARAM_RATE_PID_SOURCE_NVS ||
        pid_source == PARAM_RATE_PID_SOURCE_RAM;
    snprintf(detail,
             sizeof(detail),
             "pid_source=%s p=[%.7f %.7f %.7f]",
             params_rate_pid_source_text(pid_source),
             params->rate_kp_roll,
             params->rate_kp_pitch,
             params->rate_kp_yaw);
    preflight_emit(report_fn, report_ctx, pid_source_ok, detail);
    ok = ok && pid_source_ok;

    const bool p_only_ok = preflight_rate_pid_p_only(params);
    snprintf(detail,
             sizeof(detail),
             "rate_pid_p_only I/D=[%.6f %.6f %.6f %.6f %.6f %.6f]",
             params->rate_ki_roll,
             params->rate_kd_roll,
             params->rate_ki_pitch,
             params->rate_kd_pitch,
             params->rate_ki_yaw,
             params->rate_kd_yaw);
    preflight_emit(report_fn, report_ctx, p_only_ok, detail);
    ok = ok && p_only_ok;

    imu_sample_t sample = {0};
    uint32_t sample_seq = 0;
    const bool got_sample = imu_get_latest(&sample, &sample_seq);
    const bool imu_ok =
        got_sample &&
        sample_seq != 0u &&
        sample.health == IMU_HEALTH_OK &&
        sample.timestamp_us != 0u &&
        sample.has_gyro_acc &&
        sample.has_quaternion &&
        sample.has_attitude &&
        sample.update_age_us <= (params->imu_timeout_ms * 1000u);
    snprintf(detail,
             sizeof(detail),
             "imu health=%d seq=%lu ts=%llu age_us=%lu gyro_acc=%u quat=%u attitude=%u",
             (int)sample.health,
             (unsigned long)sample_seq,
             (unsigned long long)sample.timestamp_us,
             (unsigned long)sample.update_age_us,
             sample.has_gyro_acc ? 1u : 0u,
             sample.has_quaternion ? 1u : 0u,
             sample.has_attitude ? 1u : 0u);
    preflight_emit(report_fn, report_ctx, imu_ok, detail);
    ok = ok && imu_ok;

    estimator_state_t estimator_state = {0};
    if (got_sample &&
        (!estimator_get_latest(&estimator_state) ||
         estimator_state.timestamp_us != sample.timestamp_us)) {
        estimator_update_from_imu(&sample, &estimator_state);
    }
    const bool estimator_ok =
        estimator_state.timestamp_us == sample.timestamp_us &&
        estimator_state.attitude_valid;
    snprintf(detail,
             sizeof(detail),
             "estimator attitude_valid=%u ts_match=%u",
             estimator_state.attitude_valid ? 1u : 0u,
             estimator_state.timestamp_us == sample.timestamp_us ? 1u : 0u);
    preflight_emit(report_fn, report_ctx, estimator_ok, detail);
    ok = ok && estimator_ok;

    const eulerf_t level_trim = imu_get_level_trim_deg();
    const bool level_trim_ok =
        preflight_float_abs_le(level_trim.roll_deg, 15.0f) &&
        preflight_float_abs_le(level_trim.pitch_deg, 15.0f);
    snprintf(detail,
             sizeof(detail),
             "level_trim roll=%.3f pitch=%.3f yaw=%.3f",
             level_trim.roll_deg,
             level_trim.pitch_deg,
             level_trim.yaw_deg);
    preflight_emit(report_fn, report_ctx, level_trim_ok, detail);
    ok = ok && level_trim_ok;

    const vec3f_t gyro_bias = imu_get_gyro_bias_body_dps();
    const bool gyro_bias_ok =
        preflight_float_abs_le(gyro_bias.x, 50.0f) &&
        preflight_float_abs_le(gyro_bias.y, 50.0f) &&
        preflight_float_abs_le(gyro_bias.z, 50.0f);
    snprintf(detail,
             sizeof(detail),
             "gyro_bias x=%.3f y=%.3f z=%.3f",
             gyro_bias.x,
             gyro_bias.y,
             gyro_bias.z);
    preflight_emit(report_fn, report_ctx, gyro_bias_ok, detail);
    ok = ok && gyro_bias_ok;

    const bool map_ok = preflight_motor_output_map_valid(params);
    snprintf(detail,
             sizeof(detail),
             "motor_output_map=[%u %u %u %u]",
             (unsigned)params->motor_output_map[0],
             (unsigned)params->motor_output_map[1],
             (unsigned)params->motor_output_map[2],
             (unsigned)params->motor_output_map[3]);
    preflight_emit(report_fn, report_ctx, map_ok, detail);
    ok = ok && map_ok;

    const bool mixer_ok = mixer_self_test();
    preflight_emit(report_fn, report_ctx, mixer_ok, "mixer_self_test");
    ok = ok && mixer_ok;

    int battery_raw = 0;
    int battery_mv = 0;
    float battery_v = 0.0f;
    const bool battery_read_ok = board_battery_read(&battery_raw, &battery_mv, &battery_v) == ESP_OK;
    const bool battery_ok = battery_read_ok && battery_v > params->battery_arm_v;
    snprintf(detail,
             sizeof(detail),
             "battery voltage=%.3f arm_min=%.3f raw=%d",
             battery_v,
             params->battery_arm_v,
             battery_raw);
    preflight_emit(report_fn, report_ctx, battery_ok, detail);
    ok = ok && battery_ok;

    const bool failsafe_ok =
        runtime_state_get_failsafe_reason() == FAILSAFE_REASON_NONE &&
        params->udp_manual_timeout_ms >= 200u &&
        params->udp_manual_timeout_ms <= 1000u &&
        params->imu_timeout_ms > 0u;
    snprintf(detail,
             sizeof(detail),
             "failsafe timeout imu_ms=%lu manual_ms=%lu reason=%d",
             (unsigned long)params->imu_timeout_ms,
             (unsigned long)params->udp_manual_timeout_ms,
             (int)runtime_state_get_failsafe_reason());
    preflight_emit(report_fn, report_ctx, failsafe_ok, detail);
    ok = ok && failsafe_ok;

    const bool link_ok = link == PREFLIGHT_LINK_USB || link == PREFLIGHT_LINK_UDP;
    snprintf(detail,
             sizeof(detail),
             "%s_freshness command_frame=fresh",
             link == PREFLIGHT_LINK_UDP ? "udp" : "usb");
    preflight_emit(report_fn, report_ctx, link_ok, detail);
    ok = ok && link_ok;

    const bool limits_ok =
        params->stabilize_min_max_angle_deg <= 5.0f &&
        params->stabilize_min_angle_kp_roll >= 1.0f &&
        params->stabilize_min_angle_kp_roll <= 1.5f &&
        params->stabilize_min_angle_kp_pitch >= 1.0f &&
        params->stabilize_min_angle_kp_pitch <= 1.5f &&
        params->stabilize_min_max_rate_dps <= 20.0f &&
        params->stabilize_min_max_yaw_rate_dps <= 30.0f;
    snprintf(detail,
             sizeof(detail),
             "stabilize_min_limits angle=%.1f kp=[%.2f %.2f] rate=%.1f yaw=%.1f",
             params->stabilize_min_max_angle_deg,
             params->stabilize_min_angle_kp_roll,
             params->stabilize_min_angle_kp_pitch,
             params->stabilize_min_max_rate_dps,
             params->stabilize_min_max_yaw_rate_dps);
    preflight_emit(report_fn, report_ctx, limits_ok, detail);
    ok = ok && limits_ok;

    preflight_emit(report_fn, report_ctx, ok, ok ? "result allow_stabilize_min" : "result block_stabilize_min");
    return ok;
}
