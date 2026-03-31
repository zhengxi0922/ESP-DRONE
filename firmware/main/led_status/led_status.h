#pragma once

#include "esp_err.h"

#include "esp_drone_types.h"

esp_err_t led_status_init(void);
void led_status_set_state(led_state_t state);
led_state_t led_status_get_state(void);
void led_status_service(uint32_t now_ms);
