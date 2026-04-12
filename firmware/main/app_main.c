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
#define UDP_MANUAL_FULL_AXIS_RATE_DPS 45.0f

static void flight_control_set_base_duty_active(float base_duty)
{
    attitude_hang_state_t state = runtime_state_get_attitude_hang_state();
    state.base_duty_active = base_duty;
    runtime_state_set_attitude_hang_state(state);
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

static axis3f_t flight_control_udp_manual_rate_setpoint(axis3f_t axis_command)
{
    const float axis_limit = params_get()->udp_manual_axis_limit;
    if (axis_limit <= 0.0001f) {
        return (axis3f_t){0};
    }

    const float scale = UDP_MANUAL_FULL_AXIS_RATE_DPS / axis_limit;
    return (axis3f_t){
        .roll = axis_command.roll * scale,
        .pitch = axis_command.pitch * scale,
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
            last_arm_state = safety_status.arm_state;
            last_control_mode = control_mode;
        }

        int test_motor = -1;
        float test_duty = 0.0f;
        runtime_state_get_motor_test(&test_motor, &test_duty);
        if (test_motor >= 0 && test_duty > 0.0f && safety_status.arm_state == ARM_STATE_DISARMED) {
            flight_control_set_base_duty_active(0.0f);
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
            udp_manual_reset();
            runtime_state_set_control_mode(CONTROL_MODE_IDLE);
            memset(command_outputs, 0, sizeof(command_outputs));
            motor_stop_all();
            console_send_event_text("udp manual stopped: arm state or failsafe");
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
                estimator_update_from_imu(&sample, &estimator_state);
                const axis3f_t rate_setpoint = runtime_state_get_rate_setpoint_request();
                if (last_rate_update_us != 0 && sample.timestamp_us > last_rate_update_us) {
                    float controller_dt_s = (float)(sample.timestamp_us - last_rate_update_us) / 1000000.0f;
                    if (controller_dt_s < 0.001f) {
                        controller_dt_s = 0.001f;
                    } else if (controller_dt_s > 0.050f) {
                        controller_dt_s = 0.050f;
                    }

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

            if (sample.health != IMU_HEALTH_OK || !sample.has_gyro_acc || !sample.has_quaternion) {
                flight_control_stop_attitude_hang_test(command_outputs,
                                                       "attitude hang test stopped: imu not ready");
                continue;
            }
            if (!runtime_state_get_attitude_hang_state().ref_valid) {
                flight_control_stop_attitude_hang_test(command_outputs,
                                                       "attitude hang test stopped: reference missing");
                continue;
            }

            flight_control_set_base_duty_active(params->attitude_test_base_duty);
            if (fresh_sample) {
                axis3f_t rate_setpoint = {0};

                estimator_update_from_imu(&sample, &estimator_state);
                if (!attitude_bench_compute(&estimator_state, &rate_setpoint)) {
                    flight_control_stop_attitude_hang_test(command_outputs,
                                                           "attitude hang test stopped: estimator/ref invalid");
                    continue;
                }

                const attitude_hang_state_t updated_state = runtime_state_get_attitude_hang_state();
                if (fabsf(updated_state.err_roll_deg) > params->attitude_trip_deg ||
                    fabsf(updated_state.err_pitch_deg) > params->attitude_trip_deg) {
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

        if (safety_status.arm_state == ARM_STATE_ARMED && control_mode == CONTROL_MODE_UDP_MANUAL) {
            udp_manual_control_t manual = {0};
            if (!udp_manual_get_control(now_us, loop_dt_us, &manual) || manual.should_disarm) {
                safety_request_disarm();
                udp_manual_reset();
                runtime_state_set_control_mode(CONTROL_MODE_IDLE);
                memset(command_outputs, 0, sizeof(command_outputs));
                motor_stop_all();
                continue;
            }

            const axis3f_t rate_setpoint = flight_control_udp_manual_rate_setpoint(manual.axis);
            runtime_state_set_axis_test_request(manual.axis);
            runtime_state_set_rate_setpoint_request(rate_setpoint);
            flight_control_set_base_duty_active(manual.throttle);

            axis3f_t pid_axis = controller_get_last_rate_status().output;
            if (fresh_sample && sample.has_gyro_acc) {
                estimator_update_from_imu(&sample, &estimator_state);
                if (last_rate_update_us != 0 && sample.timestamp_us > last_rate_update_us) {
                    float controller_dt_s = (float)(sample.timestamp_us - last_rate_update_us) / 1000000.0f;
                    if (controller_dt_s < 0.001f) {
                        controller_dt_s = 0.001f;
                    } else if (controller_dt_s > 0.050f) {
                        controller_dt_s = 0.050f;
                    }

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
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5));

        const uint64_t now_us = (uint64_t)esp_timer_get_time();
        const uint32_t period_us = 1000000u / (params_get()->telemetry_usb_hz ? params_get()->telemetry_usb_hz : 1u);
        if ((now_us - last_send_us) < period_us) {
            continue;
        }
        last_send_us = now_us;

        imu_sample_t sample = {0};
        if (!imu_get_latest(&sample, NULL)) {
            continue;
        }
        barometer_state_t baro_state = {0};
        barometer_get_latest(&baro_state);

        int battery_raw = 0;
        int battery_mv = 0;
        float battery_v = 0.0f;
        board_battery_read(&battery_raw, &battery_mv, &battery_v);
        console_send_telemetry(&sample, &baro_state, battery_v, battery_raw);
        udp_protocol_send_telemetry(&sample, &baro_state, battery_v, battery_raw);
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
