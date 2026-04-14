#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "nvs_flash.h"

#include "attitude_bench.h"
#include "barometer.h"
#include "board_config.h"
#include "controller.h"
#include "console.h"
#include "estimator.h"
#include "ground_tune.h"
#include "imu.h"
#include "led_status.h"
#include "mixer.h"
#include "motor.h"
#include "params.h"
#include "runtime_state.h"
#include "safety.h"
#include "udp_manual.h"
#include "udp_protocol.h"
#include "wifi_ap.h"

#define IMU_UART_RX_TASK_PRIO 23
#define FLIGHT_CONTROL_TASK_PRIO 22
#define RC_UDP_TASK_PRIO 18
#define TELEMETRY_TASK_PRIO 17
#define SERVICE_TASK_PRIO 10

#define TASK_STACK_WORDS 4096
#define UDP_MANUAL_FULL_YAW_RATE_DPS 45.0f
#define GROUND_SATURATION_TRIP_TICKS 150u
#define GROUND_RATE_JITTER_TRIP_TICKS 80u

static void flight_control_set_base_duty_active(float base_duty)
{
    attitude_hang_state_t state = runtime_state_get_attitude_hang_state();
    state.base_duty_active = base_duty;
    runtime_state_set_attitude_hang_state(state);
}

static void flight_control_set_ground_base_duty_active(float base_duty)
{
    ground_tune_state_t state = runtime_state_get_ground_tune_state();
    state.base_duty_active = base_duty;
    runtime_state_set_ground_tune_state(state);
}

static void flight_control_stop_attitude_hang_test(float command_outputs[MOTOR_COUNT], const char *reason)
{
    runtime_state_set_control_mode(CONTROL_MODE_IDLE);
    runtime_state_set_axis_test_request((axis3f_t){0});
    runtime_state_set_rate_setpoint_request((axis3f_t){0});
    attitude_bench_reset_status();
    flight_control_set_base_duty_active(0.0f);

    if (command_outputs != NULL) {
        memset(command_outputs, 0, sizeof(float) * MOTOR_COUNT);
    }
    motor_stop_all();
    if (reason != NULL) {
        console_send_event_text(reason);
    }
}

static void flight_control_stop_udp_manual(float command_outputs[MOTOR_COUNT], const char *reason, bool request_disarm)
{
    if (request_disarm) {
        safety_request_disarm();
    }
    udp_manual_reset();
    runtime_state_set_control_mode(CONTROL_MODE_IDLE);
    runtime_state_set_axis_test_request((axis3f_t){0});
    runtime_state_set_rate_setpoint_request((axis3f_t){0});
    attitude_bench_reset_status();
    flight_control_set_base_duty_active(0.0f);

    if (command_outputs != NULL) {
        memset(command_outputs, 0, sizeof(float) * MOTOR_COUNT);
    }
    motor_stop_all();
    if (reason != NULL) {
        console_send_event_text(reason);
    }
}

static void flight_control_stop_ground_tune(float command_outputs[MOTOR_COUNT],
                                            ground_tune_trip_reason_t trip_reason,
                                            const char *reason,
                                            bool request_disarm)
{
    if (request_disarm) {
        safety_request_disarm();
    }
    runtime_state_set_control_mode(CONTROL_MODE_IDLE);
    runtime_state_set_axis_test_request((axis3f_t){0});
    runtime_state_set_rate_setpoint_request((axis3f_t){0});
    ground_tune_set_trip_reason(trip_reason);
    flight_control_set_ground_base_duty_active(0.0f);
    controller_set_runtime_flags(false, 0.0f, false, true);

    if (command_outputs != NULL) {
        memset(command_outputs, 0, sizeof(float) * MOTOR_COUNT);
    }
    motor_stop_all();
    if (reason != NULL) {
        console_send_event_text(reason);
    }
}

static bool flight_control_attitude_sample_ready(const imu_sample_t *sample)
{
    return sample != NULL &&
           sample->health == IMU_HEALTH_OK &&
           sample->has_gyro_acc &&
           sample->has_quaternion;
}

static bool flight_control_attitude_reference_ready(void)
{
    return runtime_state_get_attitude_hang_state().ref_valid;
}

static bool flight_control_attitude_trip_exceeded(const params_store_t *params)
{
    const attitude_hang_state_t updated_state = runtime_state_get_attitude_hang_state();
    return fabsf(updated_state.err_roll_deg) > params->attitude_trip_deg ||
           fabsf(updated_state.err_pitch_deg) > params->attitude_trip_deg;
}

static bool flight_control_ground_trip_exceeded(const params_store_t *params)
{
    const ground_tune_state_t state = runtime_state_get_ground_tune_state();
    return fabsf(state.err_roll_deg) > params->ground_att_trip_deg ||
           fabsf(state.err_pitch_deg) > params->ground_att_trip_deg;
}

static bool flight_control_ground_limit_outputs(const params_store_t *params, float outputs[MOTOR_COUNT])
{
    bool saturated = false;
    const float lower = fmaxf(0.0f, params->ground_test_base_duty - params->ground_test_max_extra_duty);
    const float upper = fminf(params->motor_max_duty, params->ground_test_base_duty + params->ground_test_max_extra_duty);
    float min_output = 1.0f;
    float max_output = 0.0f;

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        const float before = outputs[i];
        if (outputs[i] < lower) {
            outputs[i] = lower;
        } else if (outputs[i] > upper) {
            outputs[i] = upper;
        }
        if (fabsf(outputs[i] - before) > 0.0005f) {
            saturated = true;
        }
        if (outputs[i] <= lower + 0.001f || outputs[i] >= upper - 0.001f) {
            saturated = true;
        }
        if (outputs[i] < min_output) {
            min_output = outputs[i];
        }
        if (outputs[i] > max_output) {
            max_output = outputs[i];
        }
    }

    if ((max_output - min_output) > params->ground_test_motor_balance_limit) {
        saturated = true;
    }
    return saturated;
}

static bool flight_control_ground_rate_jitter(axis3f_t previous, axis3f_t current)
{
    const float threshold = 0.025f;
    const bool roll_flip =
        fabsf(previous.roll) > threshold &&
        fabsf(current.roll) > threshold &&
        previous.roll * current.roll < 0.0f;
    const bool pitch_flip =
        fabsf(previous.pitch) > threshold &&
        fabsf(current.pitch) > threshold &&
        previous.pitch * current.pitch < 0.0f;
    return roll_flip || pitch_flip;
}

static void flight_control_apply_led_state(const safety_status_t *safety_status,
                                           const imu_sample_t *sample,
                                           bool battery_initialized,
                                           float battery_filtered_v)
{
    if (safety_status->arm_state == ARM_STATE_FAULT_LOCK) {
        led_status_set_state(LED_STATE_FAULT_LOCK);
    } else if (safety_status->arm_state == ARM_STATE_FAILSAFE) {
        if (safety_status->failsafe_reason == FAILSAFE_REASON_RC_TIMEOUT) {
            led_status_set_state(LED_STATE_RC_LOSS);
        } else if (safety_status->failsafe_reason == FAILSAFE_REASON_IMU_TIMEOUT ||
                   safety_status->failsafe_reason == FAILSAFE_REASON_IMU_PARSE) {
            led_status_set_state(LED_STATE_IMU_ERROR);
        } else {
            led_status_set_state(LED_STATE_FAILSAFE);
        }
    } else if (battery_initialized && battery_filtered_v <= params_get()->battery_warn_v) {
        led_status_set_state(LED_STATE_LOW_BAT);
    } else if (safety_status->arm_state == ARM_STATE_ARMED && sample->health == IMU_HEALTH_OK) {
        led_status_set_state(LED_STATE_ARMED_HEALTHY);
    } else if (sample->health == IMU_HEALTH_INIT) {
        led_status_set_state(LED_STATE_INIT_WAIT_IMU);
    } else {
        led_status_set_state(LED_STATE_DISARMED_READY);
    }
}

static axis3f_t flight_control_udp_manual_yaw_rate_setpoint(axis3f_t axis_command)
{
    const float axis_limit = params_get()->udp_manual_axis_limit;
    if (axis_limit <= 0.0001f) {
        return (axis3f_t){0};
    }

    const float scale = UDP_MANUAL_FULL_YAW_RATE_DPS / axis_limit;
    return (axis3f_t){
        .roll = 0.0f,
        .pitch = 0.0f,
        .yaw = axis_command.yaw * scale,
    };
}

static void imu_uart_rx_task(void *arg)
{
    (void)arg;
    while (1) {
        imu_service_rx();
    }
}

static void flight_control_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    uint64_t last_loop_us = (uint64_t)esp_timer_get_time();
    uint32_t max_loop_dt_us = 0;
    uint32_t overrun_count = 0;
    float battery_filtered_v = 0.0f;
    bool battery_initialized = false;
    safety_status_t safety_status = {0};
    estimator_state_t estimator_state = {0};
    mixer_coeffs_t mixer_coeffs[MIXER_MOTOR_COUNT] = {0};
    float command_outputs[MOTOR_COUNT] = {0};
    uint32_t last_sample_seq = 0;
    uint64_t last_rate_update_us = 0;
    arm_state_t last_arm_state = ARM_STATE_DISARMED;
    control_mode_t last_control_mode = CONTROL_MODE_IDLE;
    uint32_t ground_saturation_ticks = 0;
    uint32_t ground_jitter_ticks = 0;
    uint64_t ground_mode_start_us = 0;
    bool ground_saturated_for_freeze = false;
    axis3f_t previous_ground_pid_axis = {0};

    mixer_build_coeffs(mixer_coeffs);

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));

        const uint64_t now_us = (uint64_t)esp_timer_get_time();
        const uint32_t loop_dt_us = (uint32_t)(now_us - last_loop_us);
        last_loop_us = now_us;
        if (loop_dt_us > max_loop_dt_us) {
            max_loop_dt_us = loop_dt_us;
        }
        if (loop_dt_us > 1500u) {
            overrun_count++;
        } else {
            overrun_count = 0;
        }
        runtime_state_set_loop_stats((loop_stats_t){
            .loop_dt_us = loop_dt_us,
            .max_loop_dt_us = max_loop_dt_us,
            .loop_overrun_count = overrun_count,
        });

        int battery_raw = 0;
        int battery_mv = 0;
        float battery_v = 0.0f;
        if (board_battery_read(&battery_raw, &battery_mv, &battery_v) == ESP_OK) {
            if (!battery_initialized) {
                battery_filtered_v = battery_v;
                battery_initialized = true;
            } else {
                battery_filtered_v = 0.9f * battery_filtered_v + 0.1f * battery_v;
            }
        }

        imu_sample_t sample = {0};
        uint32_t sample_seq = 0;
        imu_get_latest(&sample, &sample_seq);
        const imu_stats_t imu_stats = imu_get_stats();
        const bool fresh_sample = (sample_seq != 0u && sample_seq != last_sample_seq);
        if (fresh_sample) {
            last_sample_seq = sample_seq;
        }

        safety_update(&(safety_inputs_t){
            .throttle_norm = 0.0f,
            .rc_link_online = false,
            .battery_voltage = battery_filtered_v,
            .imu_health = sample.health,
            .imu_stats = imu_stats,
            .loop_stats = runtime_state_get_loop_stats(),
        }, &safety_status);

        flight_control_apply_led_state(&safety_status, &sample, battery_initialized, battery_filtered_v);
        led_status_service((uint32_t)(now_us / 1000u));

        const control_mode_t control_mode = runtime_state_get_control_mode();
        if (safety_status.arm_state != last_arm_state || control_mode != last_control_mode) {
            estimator_reset();
            controller_reset();
            memset(command_outputs, 0, sizeof(command_outputs));
            last_rate_update_us = 0;
            attitude_bench_reset_status();
            ground_saturation_ticks = 0;
            ground_jitter_ticks = 0;
            ground_saturated_for_freeze = false;
            previous_ground_pid_axis = (axis3f_t){0};
            ground_mode_start_us = (control_mode == CONTROL_MODE_ATTITUDE_GROUND_TUNE) ? now_us : 0u;
            last_arm_state = safety_status.arm_state;
            last_control_mode = control_mode;
        }

        if (fresh_sample && sample.has_gyro_acc) {
            estimator_update_from_imu(&sample, &estimator_state);
        }

        int test_motor = -1;
        float test_duty = 0.0f;
        runtime_state_get_motor_test(&test_motor, &test_duty);
        if (test_motor >= 0 && test_duty > 0.0f && safety_status.arm_state == ARM_STATE_DISARMED) {
            flight_control_set_base_duty_active(0.0f);
            flight_control_set_ground_base_duty_active(0.0f);
            motor_set_test_output((uint8_t)test_motor, test_duty);
            continue;
        }

        if (control_mode == CONTROL_MODE_ATTITUDE_HANG_TEST &&
            (safety_status.arm_state != ARM_STATE_ARMED ||
             safety_status.failsafe_reason != FAILSAFE_REASON_NONE)) {
            flight_control_stop_attitude_hang_test(command_outputs,
                                                   "attitude hang test stopped: arm state or failsafe");
            continue;
        }

        if (control_mode == CONTROL_MODE_UDP_MANUAL &&
            (safety_status.arm_state == ARM_STATE_FAILSAFE ||
             safety_status.arm_state == ARM_STATE_FAULT_LOCK ||
             safety_status.failsafe_reason != FAILSAFE_REASON_NONE)) {
            flight_control_stop_udp_manual(command_outputs,
                                           "udp manual stopped: arm state or failsafe",
                                           false);
            continue;
        }

        if (control_mode == CONTROL_MODE_ATTITUDE_GROUND_TUNE &&
            (safety_status.arm_state != ARM_STATE_ARMED ||
             safety_status.failsafe_reason != FAILSAFE_REASON_NONE)) {
            const bool request_disarm =
                safety_status.failsafe_reason == FAILSAFE_REASON_BATTERY_CRITICAL ||
                safety_status.arm_state == ARM_STATE_FAILSAFE ||
                safety_status.arm_state == ARM_STATE_FAULT_LOCK;
            flight_control_stop_ground_tune(command_outputs,
                                            GROUND_TUNE_TRIP_FAILSAFE,
                                            "ground tune stopped: battery critical or failsafe",
                                            request_disarm);
            continue;
        }

        if (control_mode == CONTROL_MODE_AXIS_TEST && safety_status.arm_state == ARM_STATE_DISARMED) {
            const axis3f_t axis_request = runtime_state_get_axis_test_request();
            flight_control_set_base_duty_active(params_get()->bringup_test_base_duty);
            mixer_mix(mixer_coeffs,
                      &(mixer_input_t){
                          .throttle = params_get()->bringup_test_base_duty,
                          .axis = axis_request,
                      },
                      command_outputs);
            motor_set_armed_outputs(command_outputs, false);
            continue;
        }

        if (safety_status.arm_state == ARM_STATE_ARMED && control_mode == CONTROL_MODE_RATE_TEST) {
            flight_control_set_base_duty_active(params_get()->bringup_test_base_duty);
            if (fresh_sample && sample.has_gyro_acc) {
                const axis3f_t rate_setpoint = runtime_state_get_rate_setpoint_request();
                if (last_rate_update_us != 0 && sample.timestamp_us > last_rate_update_us) {
                    float controller_dt_s = (float)(sample.timestamp_us - last_rate_update_us) / 1000000.0f;
                    if (controller_dt_s < 0.001f) {
                        controller_dt_s = 0.001f;
                    } else if (controller_dt_s > 0.050f) {
                        controller_dt_s = 0.050f;
                    }

                    controller_set_runtime_flags(true, params_get()->bringup_test_base_duty, false, false);
                    const rate_controller_status_t rate_status =
                        controller_update_rate(&rate_setpoint, &estimator_state.rate_rpy_dps, controller_dt_s);
                    mixer_mix(mixer_coeffs,
                              &(mixer_input_t){
                                  .throttle = params_get()->bringup_test_base_duty,
                                  .axis = rate_status.output,
                              },
                              command_outputs);
                }
                last_rate_update_us = sample.timestamp_us;
            }
            motor_set_armed_outputs(command_outputs, false);
            continue;
        }

        if (safety_status.arm_state == ARM_STATE_ARMED && control_mode == CONTROL_MODE_ATTITUDE_HANG_TEST) {
            const params_store_t *params = params_get();

            if (!flight_control_attitude_sample_ready(&sample)) {
                flight_control_stop_attitude_hang_test(command_outputs,
                                                       "attitude hang test stopped: imu not ready");
                continue;
            }
            if (!flight_control_attitude_reference_ready()) {
                flight_control_stop_attitude_hang_test(command_outputs,
                                                       "attitude hang test stopped: reference missing");
                continue;
            }

            flight_control_set_base_duty_active(params->attitude_test_base_duty);
            if (fresh_sample) {
                axis3f_t rate_setpoint = {0};

                if (!attitude_bench_compute(&estimator_state, &rate_setpoint)) {
                    flight_control_stop_attitude_hang_test(command_outputs,
                                                           "attitude hang test stopped: estimator/ref invalid");
                    continue;
                }

                if (flight_control_attitude_trip_exceeded(params)) {
                    flight_control_stop_attitude_hang_test(command_outputs,
                                                           "attitude hang test stopped: attitude trip");
                    continue;
                }

                runtime_state_set_rate_setpoint_request(rate_setpoint);
                if (last_rate_update_us != 0 && sample.timestamp_us > last_rate_update_us) {
                    float controller_dt_s = (float)(sample.timestamp_us - last_rate_update_us) / 1000000.0f;
                    if (controller_dt_s < 0.001f) {
                        controller_dt_s = 0.001f;
                    } else if (controller_dt_s > 0.050f) {
                        controller_dt_s = 0.050f;
                    }

                    controller_set_runtime_flags(true, params->attitude_test_base_duty, false, false);
                    const rate_controller_status_t rate_status =
                        controller_update_rate(&rate_setpoint, &estimator_state.rate_rpy_dps, controller_dt_s);
                    mixer_mix(mixer_coeffs,
                              &(mixer_input_t){
                                  .throttle = params->attitude_test_base_duty,
                                  .axis = rate_status.output,
                              },
                              command_outputs);
                }
                last_rate_update_us = sample.timestamp_us;
            }

            motor_set_armed_outputs(command_outputs, false);
            continue;
        }

        if (safety_status.arm_state == ARM_STATE_ARMED && control_mode == CONTROL_MODE_ATTITUDE_GROUND_TUNE) {
            const params_store_t *params = params_get();

            if (ground_mode_start_us == 0u) {
                ground_mode_start_us = now_us;
            }
            if (!flight_control_attitude_sample_ready(&sample)) {
                flight_control_stop_ground_tune(command_outputs,
                                                GROUND_TUNE_TRIP_IMU_STALE,
                                                "ground tune stopped: imu stale",
                                                false);
                continue;
            }
            if (sample.update_age_us > (params->imu_timeout_ms * 1000u)) {
                flight_control_stop_ground_tune(command_outputs,
                                                GROUND_TUNE_TRIP_IMU_STALE,
                                                "ground tune stopped: imu stale",
                                                false);
                continue;
            }
            if (!runtime_state_get_ground_tune_state().ref_valid) {
                flight_control_stop_ground_tune(command_outputs,
                                                GROUND_TUNE_TRIP_REF_MISSING,
                                                "ground tune stopped: ground ref missing",
                                                false);
                continue;
            }
            if (params->ground_tune_use_kalman_attitude && !estimator_state.kalman_valid) {
                flight_control_stop_ground_tune(command_outputs,
                                                GROUND_TUNE_TRIP_KALMAN_INVALID,
                                                "ground tune stopped: kalman invalid",
                                                false);
                continue;
            }
            if ((now_us - ground_mode_start_us) >= ((uint64_t)params->ground_test_auto_disarm_ms * 1000u)) {
                flight_control_stop_ground_tune(command_outputs,
                                                GROUND_TUNE_TRIP_WATCHDOG,
                                                "ground tune stopped: watchdog timeout",
                                                true);
                continue;
            }

            flight_control_set_ground_base_duty_active(params->ground_test_base_duty);
            if (fresh_sample) {
                axis3f_t rate_setpoint = {0};
                if (!ground_tune_compute(&estimator_state, &rate_setpoint)) {
                    const ground_tune_trip_reason_t trip =
                        runtime_state_get_ground_tune_state().trip_reason;
                    const char *reason = "ground tune stopped: estimator/ref invalid";
                    if (trip == GROUND_TUNE_TRIP_REF_MISSING) {
                        reason = "ground tune stopped: ground ref missing";
                    } else if (trip == GROUND_TUNE_TRIP_KALMAN_INVALID) {
                        reason = "ground tune stopped: kalman invalid";
                    } else if (trip == GROUND_TUNE_TRIP_IMU_STALE) {
                        reason = "ground tune stopped: imu stale";
                    }
                    flight_control_stop_ground_tune(command_outputs, trip, reason, false);
                    continue;
                }
                if (flight_control_ground_trip_exceeded(params)) {
                    flight_control_stop_ground_tune(command_outputs,
                                                    GROUND_TUNE_TRIP_ANGLE,
                                                    "ground tune stopped: angle trip",
                                                    false);
                    continue;
                }

                runtime_state_set_rate_setpoint_request(rate_setpoint);
                if (last_rate_update_us != 0 && sample.timestamp_us > last_rate_update_us) {
                    float controller_dt_s = (float)(sample.timestamp_us - last_rate_update_us) / 1000000.0f;
                    if (controller_dt_s < 0.001f) {
                        controller_dt_s = 0.001f;
                    } else if (controller_dt_s > 0.050f) {
                        controller_dt_s = 0.050f;
                    }

                    const axis3f_t measured_rate =
                        params->ground_tune_use_filtered_rate ? estimator_state.filtered_rate_rpy_dps : estimator_state.rate_rpy_dps;
                    controller_set_runtime_flags(true,
                                                 params->ground_test_base_duty,
                                                 ground_saturated_for_freeze,
                                                 false);
                    const rate_controller_status_t rate_status =
                        controller_update_rate(&rate_setpoint, &measured_rate, controller_dt_s);
                    mixer_mix(mixer_coeffs,
                              &(mixer_input_t){
                                  .throttle = params->ground_test_base_duty,
                                  .axis = rate_status.output,
                              },
                              command_outputs);

                    const bool saturated = flight_control_ground_limit_outputs(params, command_outputs);
                    ground_saturated_for_freeze = saturated;
                    controller_set_runtime_flags(true, params->ground_test_base_duty, saturated, false);
                    ground_saturation_ticks = saturated ? (ground_saturation_ticks + 1u) : 0u;
                    ground_jitter_ticks =
                        flight_control_ground_rate_jitter(previous_ground_pid_axis, rate_status.output)
                            ? (ground_jitter_ticks + 1u)
                            : 0u;
                    previous_ground_pid_axis = rate_status.output;

                    if (ground_saturation_ticks >= GROUND_SATURATION_TRIP_TICKS) {
                        flight_control_stop_ground_tune(command_outputs,
                                                        GROUND_TUNE_TRIP_SATURATION,
                                                        "ground tune stopped: saturation trip",
                                                        false);
                        continue;
                    }
                    if (ground_jitter_ticks >= GROUND_RATE_JITTER_TRIP_TICKS) {
                        flight_control_stop_ground_tune(command_outputs,
                                                        GROUND_TUNE_TRIP_RATE_JITTER,
                                                        "ground tune stopped: rate output jitter trip",
                                                        false);
                        continue;
                    }
                }
                last_rate_update_us = sample.timestamp_us;
            }

            motor_set_armed_outputs(command_outputs, false);
            continue;
        }

        if (safety_status.arm_state == ARM_STATE_ARMED && control_mode == CONTROL_MODE_UDP_MANUAL) {
            const params_store_t *params = params_get();
            if (!flight_control_attitude_sample_ready(&sample)) {
                flight_control_stop_udp_manual(command_outputs,
                                               "udp manual stopped: attitude imu not ready",
                                               false);
                continue;
            }
            if (!flight_control_attitude_reference_ready()) {
                flight_control_stop_udp_manual(command_outputs,
                                               "udp manual stopped: attitude reference missing",
                                               false);
                continue;
            }

            udp_manual_control_t manual = {0};
            if (!udp_manual_get_control(now_us, loop_dt_us, &manual) || manual.should_disarm) {
                flight_control_stop_udp_manual(command_outputs, NULL, true);
                continue;
            }

            axis3f_t rate_setpoint = runtime_state_get_rate_setpoint_request();
            const axis3f_t yaw_rate_setpoint = flight_control_udp_manual_yaw_rate_setpoint(manual.axis);
            rate_setpoint.yaw = yaw_rate_setpoint.yaw;
            runtime_state_set_axis_test_request((axis3f_t){
                .roll = 0.0f,
                .pitch = 0.0f,
                .yaw = manual.axis.yaw,
            });
            runtime_state_set_rate_setpoint_request(rate_setpoint);
            flight_control_set_base_duty_active(manual.throttle);

            axis3f_t pid_axis = controller_get_last_rate_status().output;
            if (fresh_sample && sample.has_gyro_acc) {
                axis3f_t attitude_rate_setpoint = {0};
                if (!attitude_bench_compute(&estimator_state, &attitude_rate_setpoint)) {
                    flight_control_stop_udp_manual(command_outputs,
                                                   "udp manual stopped: estimator/ref invalid",
                                                   false);
                    continue;
                }
                if (flight_control_attitude_trip_exceeded(params)) {
                    flight_control_stop_udp_manual(command_outputs,
                                                   "udp manual stopped: attitude trip",
                                                   false);
                    continue;
                }

                rate_setpoint.roll = attitude_rate_setpoint.roll;
                rate_setpoint.pitch = attitude_rate_setpoint.pitch;
                rate_setpoint.yaw = yaw_rate_setpoint.yaw;
                runtime_state_set_rate_setpoint_request(rate_setpoint);
                flight_control_set_base_duty_active(manual.throttle);

                if (last_rate_update_us != 0 && sample.timestamp_us > last_rate_update_us) {
                    float controller_dt_s = (float)(sample.timestamp_us - last_rate_update_us) / 1000000.0f;
                    if (controller_dt_s < 0.001f) {
                        controller_dt_s = 0.001f;
                    } else if (controller_dt_s > 0.050f) {
                        controller_dt_s = 0.050f;
                    }

                    controller_set_runtime_flags(true, manual.throttle, false, false);
                    const rate_controller_status_t rate_status =
                        controller_update_rate(&rate_setpoint, &estimator_state.rate_rpy_dps, controller_dt_s);
                    pid_axis = rate_status.output;
                }
                last_rate_update_us = sample.timestamp_us;
            }

            mixer_mix(mixer_coeffs,
                      &(mixer_input_t){
                          .throttle = manual.throttle,
                          .axis = pid_axis,
                      },
                      command_outputs);
            motor_set_armed_outputs(command_outputs, false);
            continue;
        }

        flight_control_set_base_duty_active(0.0f);
        flight_control_set_ground_base_duty_active(0.0f);
        if (safety_status.arm_state == ARM_STATE_ARMED) {
            float zeros[MOTOR_COUNT] = {0};
            motor_set_armed_outputs(zeros, false);
        } else {
            motor_stop_all();
        }
    }
}

static void rc_udp_task(void *arg)
{
    udp_protocol_task(arg);
}

static void telemetry_task(void *arg)
{
    (void)arg;

    uint64_t last_send_us = 0;
    bool last_battery_valid = false;
    int last_battery_raw = 0;
    float last_battery_v = 0.0f;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5));

        const uint64_t now_us = (uint64_t)esp_timer_get_time();
        const uint32_t period_us = 1000000u / (params_get()->telemetry_usb_hz ? params_get()->telemetry_usb_hz : 1u);
        if ((now_us - last_send_us) < period_us) {
            continue;
        }
        last_send_us = now_us;

        imu_sample_t sample = {0};
        uint32_t sample_seq = 0;
        if (!imu_get_latest(&sample, &sample_seq)) {
            continue;
        }
        barometer_state_t baro_state = {0};
        barometer_get_latest(&baro_state);

        int battery_raw = last_battery_raw;
        int battery_mv = 0;
        float battery_v = last_battery_v;
        if (board_battery_read(&battery_raw, &battery_mv, &battery_v) == ESP_OK) {
            last_battery_valid = true;
            last_battery_raw = battery_raw;
            last_battery_v = battery_v;
        } else if (!last_battery_valid) {
            battery_raw = 0;
            battery_v = 0.0f;
        }
        console_send_telemetry(&sample, sample_seq, &baro_state, battery_v, battery_raw);
        udp_protocol_send_telemetry(&sample, sample_seq, &baro_state, battery_v, battery_raw);
    }
}

static void service_task(void *arg)
{
    (void)arg;

    imu_health_t last_imu_health = IMU_HEALTH_INIT;
    arm_state_t last_arm_state = ARM_STATE_DISARMED;
    uint64_t last_periodic_log_us = 0;

    while (1) {
        console_service();

        imu_sample_t sample = {0};
        imu_get_latest(&sample, NULL);
        const arm_state_t arm_state = runtime_state_get_arm_state();

        if (sample.health != last_imu_health) {
            char msg[96];
            snprintf(msg, sizeof(msg), "imu health -> %s", imu_health_to_string(sample.health));
            console_send_event_text(msg);
            last_imu_health = sample.health;
        }
        if (arm_state != last_arm_state) {
            char msg[96];
            snprintf(msg, sizeof(msg), "arm state -> %d reason=%d", (int)arm_state, (int)runtime_state_get_failsafe_reason());
            console_send_event_text(msg);
            last_arm_state = arm_state;
        }

        const uint64_t now_us = (uint64_t)esp_timer_get_time();
        if ((now_us - last_periodic_log_us) >= 1000000u) {
            const imu_stats_t stats = imu_get_stats();
            char msg[120];
            snprintf(msg,
                     sizeof(msg),
                     "imu seq=%lu good=%lu err=%lu age_us=%lu",
                     (unsigned long)stats.published_seq,
                     (unsigned long)stats.good_frames,
                     (unsigned long)stats.parse_errors,
                     (unsigned long)sample.update_age_us);
            console_send_event_text(msg);
            last_periodic_log_us = now_us;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    runtime_state_init();
    params_init();
    safety_init();
    estimator_init();
    controller_init();
    board_battery_init();
    barometer_init();
    led_status_init();
    motor_init();
    mixer_init();
    imu_init();
    console_init();
    wifi_ap_start();
    udp_manual_init();
    runtime_state_set_motor_test(-1, 0.0f);
    runtime_state_set_control_mode(CONTROL_MODE_IDLE);
    runtime_state_set_axis_test_request((axis3f_t){0});
    runtime_state_set_rate_setpoint_request((axis3f_t){0});
    attitude_bench_clear_reference();
    ground_tune_clear_reference();
    if (!mixer_self_test()) {
        runtime_state_set_arm_state(ARM_STATE_FAULT_LOCK);
        runtime_state_set_failsafe_reason(FAILSAFE_REASON_NONE);
        console_send_event_text("mixer self-test failed");
    }
    console_send_event_text("esp-drone bottom-layer framework ready");

    xTaskCreatePinnedToCore(imu_uart_rx_task, "imu_uart_rx_task", TASK_STACK_WORDS, NULL, IMU_UART_RX_TASK_PRIO, NULL, 1);
    xTaskCreatePinnedToCore(flight_control_task, "flight_control_task", TASK_STACK_WORDS, NULL, FLIGHT_CONTROL_TASK_PRIO, NULL, 1);
    xTaskCreatePinnedToCore(rc_udp_task, "rc_udp_task", TASK_STACK_WORDS, NULL, RC_UDP_TASK_PRIO, NULL, 0);
    xTaskCreatePinnedToCore(telemetry_task, "telemetry_task", TASK_STACK_WORDS, NULL, TELEMETRY_TASK_PRIO, NULL, 0);
    xTaskCreatePinnedToCore(service_task, "service_task", TASK_STACK_WORDS, NULL, SERVICE_TASK_PRIO, NULL, 0);
}
