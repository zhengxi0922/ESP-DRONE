#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_timer.h"

#include "board_config.h"
#include "console.h"
#include "imu.h"
#include "led_status.h"
#include "motor.h"
#include "params.h"
#include "runtime_state.h"
#include "safety.h"

#define IMU_UART_RX_TASK_PRIO 23
#define FLIGHT_CONTROL_TASK_PRIO 22
#define RC_UDP_TASK_PRIO 18
#define TELEMETRY_TASK_PRIO 17
#define SERVICE_TASK_PRIO 10

#define TASK_STACK_WORDS 4096

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
        imu_get_latest(&sample, NULL);
        const imu_stats_t imu_stats = imu_get_stats();

        safety_update(&(safety_inputs_t){
            .throttle_norm = 0.0f,
            .rc_link_online = false,
            .battery_voltage = battery_filtered_v,
            .imu_health = sample.health,
            .imu_stats = imu_stats,
            .loop_stats = runtime_state_get_loop_stats(),
        }, &safety_status);

        if (safety_status.arm_state == ARM_STATE_FAULT_LOCK) {
            led_status_set_state(LED_STATE_FAULT_LOCK);
        } else if (safety_status.arm_state == ARM_STATE_FAILSAFE) {
            if (safety_status.failsafe_reason == FAILSAFE_REASON_RC_TIMEOUT) {
                led_status_set_state(LED_STATE_RC_LOSS);
            } else if (safety_status.failsafe_reason == FAILSAFE_REASON_IMU_TIMEOUT ||
                       safety_status.failsafe_reason == FAILSAFE_REASON_IMU_PARSE) {
                led_status_set_state(LED_STATE_IMU_ERROR);
            } else {
                led_status_set_state(LED_STATE_FAILSAFE);
            }
        } else if (battery_initialized && battery_filtered_v <= params_get()->battery_warn_v) {
            led_status_set_state(LED_STATE_LOW_BAT);
        } else if (safety_status.arm_state == ARM_STATE_ARMED && sample.health == IMU_HEALTH_OK) {
            led_status_set_state(LED_STATE_ARMED_HEALTHY);
        } else if (sample.health == IMU_HEALTH_INIT) {
            led_status_set_state(LED_STATE_INIT_WAIT_IMU);
        } else {
            led_status_set_state(LED_STATE_DISARMED_READY);
        }
        led_status_service((uint32_t)(now_us / 1000u));

        int test_motor = -1;
        float test_duty = 0.0f;
        runtime_state_get_motor_test(&test_motor, &test_duty);
        if (safety_status.arm_state != ARM_STATE_ARMED) {
            if (test_motor >= 0 && test_duty > 0.0f) {
                motor_set_test_output((uint8_t)test_motor, test_duty);
            } else {
                motor_stop_all();
            }
        } else {
            float zeros[MOTOR_COUNT] = {0};
            motor_set_armed_outputs(zeros, false);
        }
    }
}

static void rc_udp_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
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

        int battery_raw = 0;
        int battery_mv = 0;
        float battery_v = 0.0f;
        board_battery_read(&battery_raw, &battery_mv, &battery_v);
        console_send_telemetry(&sample, battery_v, battery_raw);
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
            snprintf(msg, sizeof(msg),
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
    board_battery_init();
    led_status_init();
    motor_init();
    imu_init();
    console_init();
    runtime_state_set_motor_test(-1, 0.0f);
    console_send_event_text("esp-drone bottom-layer framework ready");

    xTaskCreatePinnedToCore(imu_uart_rx_task, "imu_uart_rx_task", TASK_STACK_WORDS, NULL, IMU_UART_RX_TASK_PRIO, NULL, 1);
    xTaskCreatePinnedToCore(flight_control_task, "flight_control_task", TASK_STACK_WORDS, NULL, FLIGHT_CONTROL_TASK_PRIO, NULL, 1);
    xTaskCreatePinnedToCore(rc_udp_task, "rc_udp_task", TASK_STACK_WORDS, NULL, RC_UDP_TASK_PRIO, NULL, 0);
    xTaskCreatePinnedToCore(telemetry_task, "telemetry_task", TASK_STACK_WORDS, NULL, TELEMETRY_TASK_PRIO, NULL, 0);
    xTaskCreatePinnedToCore(service_task, "service_task", TASK_STACK_WORDS, NULL, SERVICE_TASK_PRIO, NULL, 0);
}
