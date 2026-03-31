#pragma once

#include "esp_err.h"

#include "esp_drone_types.h"

esp_err_t console_init(void);
void console_service(void);
void console_send_event_text(const char *text);
void console_send_telemetry(const imu_sample_t *imu_sample, float battery_voltage, int battery_raw);
