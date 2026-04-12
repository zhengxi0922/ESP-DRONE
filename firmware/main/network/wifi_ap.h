#pragma once

#include "esp_err.h"

#define WIFI_AP_DEFAULT_SSID "ESP-DRONE"
#define WIFI_AP_DEFAULT_PASSWORD "12345678"
#define WIFI_AP_DEFAULT_IP "192.168.4.1"

esp_err_t wifi_ap_start(void);
