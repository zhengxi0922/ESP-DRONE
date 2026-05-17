#include "wifi_ap.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/inet.h"

#include "attitude_bench.h"
#include "console.h"
#include "ground_tune.h"
#include "motor.h"
#include "params.h"
#include "runtime_state.h"
#include "safety.h"
#include "udp_manual.h"

#define WIFI_AP_MAX_CONNECTIONS 2
#define WIFI_STA_MAX_RETRY 5
#define WIFI_FALLBACK_TASK_STACK 4096

typedef enum {
    WIFI_DEBUG_MODE_SOFTAP,
    WIFI_DEBUG_MODE_STA,
    WIFI_DEBUG_MODE_APSTA,
} wifi_debug_mode_t;

static const char *TAG = "wifi_ap";
static bool s_wifi_started;
static bool s_softap_active;
static bool s_sta_active;
static bool s_fallback_task_running;
static int s_sta_retry_count;
static wifi_debug_mode_t s_configured_mode = WIFI_DEBUG_MODE_SOFTAP;
static esp_netif_t *s_ap_netif;
static esp_netif_t *s_sta_netif;
static wifi_config_t s_ap_wifi_config;

static void wifi_ap_event_log(const char *message)
{
    ESP_LOGI(TAG, "%s", message);
    console_send_event_text(message);
}

static wifi_debug_mode_t wifi_debug_mode_from_params(const params_store_t *params)
{
    if (params != NULL && strcmp(params->wifi_mode, "sta") == 0) {
        return WIFI_DEBUG_MODE_STA;
    }
    if (params != NULL && strcmp(params->wifi_mode, "apsta") == 0) {
        return WIFI_DEBUG_MODE_APSTA;
    }
    return WIFI_DEBUG_MODE_SOFTAP;
}

static const char *wifi_debug_mode_text(wifi_debug_mode_t mode)
{
    switch (mode) {
    case WIFI_DEBUG_MODE_STA:
        return "sta";
    case WIFI_DEBUG_MODE_APSTA:
        return "apsta";
    case WIFI_DEBUG_MODE_SOFTAP:
    default:
        return "softap";
    }
}

static void wifi_copy_text_field(void *dest, size_t dest_len, const char *src)
{
    if (dest == NULL || dest_len == 0u) {
        return;
    }
    memset(dest, 0, dest_len);
    if (src == NULL) {
        return;
    }
    size_t len = strlen(src);
    if (len > dest_len) {
        len = dest_len;
    }
    memcpy(dest, src, len);
}

static void wifi_prepare_ap_config(const params_store_t *params, wifi_config_t *out_config)
{
    memset(out_config, 0, sizeof(*out_config));
    wifi_copy_text_field(out_config->ap.ssid, sizeof(out_config->ap.ssid), WIFI_AP_DEFAULT_SSID);
    wifi_copy_text_field(out_config->ap.password, sizeof(out_config->ap.password), WIFI_AP_DEFAULT_PASSWORD);
    out_config->ap.ssid_len = strlen(WIFI_AP_DEFAULT_SSID);
    out_config->ap.channel = params != NULL ? params->wifi_ap_channel : 6u;
    out_config->ap.max_connection = WIFI_AP_MAX_CONNECTIONS;
    out_config->ap.authmode = strlen(WIFI_AP_DEFAULT_PASSWORD) >= 8 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    out_config->ap.pmf_cfg.required = false;
}

static void wifi_prepare_sta_config(const params_store_t *params, wifi_config_t *out_config)
{
    memset(out_config, 0, sizeof(*out_config));
    if (params == NULL) {
        return;
    }
    wifi_copy_text_field(out_config->sta.ssid, sizeof(out_config->sta.ssid), params->sta_ssid);
    wifi_copy_text_field(out_config->sta.password, sizeof(out_config->sta.password), params->sta_password);
    out_config->sta.threshold.authmode = params->sta_password[0] == '\0' ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    out_config->sta.pmf_cfg.capable = true;
    out_config->sta.pmf_cfg.required = false;
}

static bool wifi_parse_ip4(const char *text, esp_ip4_addr_t *out_addr)
{
    if (text == NULL || text[0] == '\0' || out_addr == NULL) {
        return false;
    }

    struct in_addr parsed = {0};
    if (inet_aton(text, &parsed) == 0) {
        return false;
    }
    out_addr->addr = parsed.s_addr;
    return true;
}

static esp_err_t wifi_configure_sta_ip(esp_netif_t *sta_netif, const params_store_t *params)
{
    if (sta_netif == NULL || params == NULL || params->sta_static_ip[0] == '\0') {
        return ESP_OK;
    }

    esp_netif_ip_info_t ip_info = {0};
    if (!wifi_parse_ip4(params->sta_static_ip, &ip_info.ip) ||
        !wifi_parse_ip4(params->sta_gateway, &ip_info.gw) ||
        !wifi_parse_ip4(params->sta_netmask, &ip_info.netmask)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = esp_netif_dhcpc_stop(sta_netif);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        return err;
    }
    return esp_netif_set_ip_info(sta_netif, &ip_info);
}

static esp_err_t wifi_log_and_return(const char *step, esp_err_t err)
{
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
        return ESP_OK;
    }

    char message[128];
    snprintf(message, sizeof(message), "wifi start failed at %s: %s", step, esp_err_to_name(err));
    ESP_LOGE(TAG, "%s", message);
    console_send_event_text(message);
    return err;
}

static void wifi_clear_active_control(void)
{
    runtime_state_set_motor_test(-1, 0.0f);
    runtime_state_clear_all_motor_test();
    runtime_state_set_axis_test_request((axis3f_t){0});
    runtime_state_set_rate_setpoint_request((axis3f_t){0});
    runtime_state_set_control_mode(CONTROL_MODE_IDLE);
    runtime_state_set_stream_enabled(false);
    runtime_state_clear_attitude_reference();
    runtime_state_clear_ground_reference();
    attitude_bench_clear_reference();
    ground_tune_clear_reference();
    (void)udp_manual_stop();
    safety_request_disarm();
    motor_stop_all();
}

static void wifi_enter_safe_state(const char *reason)
{
    wifi_clear_active_control();
    char message[160];
    snprintf(message, sizeof(message), "wifi link lost: %s; motors stopped and tests cleared", reason);
    wifi_ap_event_log(message);
}

static void wifi_log_softap_ready(const char *prefix)
{
    const params_store_t *params = params_get();
    esp_netif_ip_info_t ip_info = {0};
    char ip_text[16];
    snprintf(ip_text, sizeof(ip_text), "%s", WIFI_AP_DEFAULT_IP);
    if (s_ap_netif != NULL && esp_netif_get_ip_info(s_ap_netif, &ip_info) == ESP_OK) {
        snprintf(ip_text, sizeof(ip_text), IPSTR, IP2STR(&ip_info.ip));
    }

    char message[192];
    snprintf(message,
             sizeof(message),
             "%s ssid=%s channel=%u ip=%s udp_port=%lu",
             prefix,
             WIFI_AP_DEFAULT_SSID,
             (unsigned)s_ap_wifi_config.ap.channel,
             ip_text,
             (unsigned long)params->wifi_udp_port);
    wifi_ap_event_log(message);
}

static void wifi_softap_fallback_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(100));

    const params_store_t *params = params_get();
    if (s_ap_netif == NULL) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
    }
    wifi_prepare_ap_config(params, &s_ap_wifi_config);

    (void)esp_wifi_stop();
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err == ESP_OK) {
        err = esp_wifi_set_config(WIFI_IF_AP, &s_ap_wifi_config);
    }
    if (err == ESP_OK) {
        err = esp_wifi_start();
    }

    if (err == ESP_OK) {
        s_softap_active = true;
        s_sta_active = false;
        wifi_log_softap_ready("sta fallback softap started");
    } else {
        char message[128];
        snprintf(message, sizeof(message), "sta fallback softap failed: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "%s", message);
        console_send_event_text(message);
    }

    s_fallback_task_running = false;
    vTaskDelete(NULL);
}

static void wifi_schedule_softap_fallback(void)
{
    if (s_fallback_task_running) {
        return;
    }

    s_fallback_task_running = true;
    if (xTaskCreate(wifi_softap_fallback_task,
                    "wifi_softap_fallback",
                    WIFI_FALLBACK_TASK_STACK,
                    NULL,
                    4,
                    NULL) != pdPASS) {
        s_fallback_task_running = false;
        wifi_ap_event_log("sta fallback softap task create failed");
    }
}

static void wifi_handle_sta_disconnected(const wifi_event_sta_disconnected_t *event)
{
    const int reason = event != NULL ? event->reason : -1;
    char message[128];
    snprintf(message, sizeof(message), "sta disconnected reason=%d", reason);
    wifi_enter_safe_state(message);

    if (!s_sta_active || s_fallback_task_running) {
        return;
    }

    if (s_sta_retry_count < WIFI_STA_MAX_RETRY) {
        ++s_sta_retry_count;
        esp_err_t err = esp_wifi_connect();
        snprintf(message,
                 sizeof(message),
                 "sta reconnect attempt %d/%d: %s",
                 s_sta_retry_count,
                 WIFI_STA_MAX_RETRY,
                 esp_err_to_name(err));
        wifi_ap_event_log(message);
        return;
    }

    if (s_configured_mode == WIFI_DEBUG_MODE_STA) {
        wifi_ap_event_log("sta reconnect failed; falling back to softap");
        wifi_schedule_softap_fallback();
    } else {
        wifi_ap_event_log("sta reconnect failed; softap remains active");
    }
}

static void wifi_ap_event_handler(void *arg,
                                  esp_event_base_t event_base,
                                  int32_t event_id,
                                  void *event_data)
{
    (void)arg;

    char message[160];
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
        s_sta_retry_count = 0;
        snprintf(message,
                 sizeof(message),
                 "sta connected ip=" IPSTR " udp_port=%lu",
                 IP2STR(&event->ip_info.ip),
                 (unsigned long)params_get()->wifi_udp_port);
        wifi_ap_event_log(message);
        return;
    }

    if (event_base != WIFI_EVENT) {
        return;
    }

    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        const wifi_event_ap_staconnected_t *event = (const wifi_event_ap_staconnected_t *)event_data;
        snprintf(message,
                 sizeof(message),
                 "softap station connected aid=%d",
                 event != NULL ? event->aid : -1);
        wifi_ap_event_log(message);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        const wifi_event_ap_stadisconnected_t *event = (const wifi_event_ap_stadisconnected_t *)event_data;
        snprintf(message,
                 sizeof(message),
                 "softap station disconnected aid=%d",
                 event != NULL ? event->aid : -1);
        wifi_ap_event_log(message);
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_handle_sta_disconnected((const wifi_event_sta_disconnected_t *)event_data);
    }
}

esp_err_t wifi_ap_start(void)
{
    if (s_wifi_started) {
        return ESP_OK;
    }

    const params_store_t *params = params_get();
    wifi_debug_mode_t mode = wifi_debug_mode_from_params(params);
    bool start_ap = mode == WIFI_DEBUG_MODE_SOFTAP || mode == WIFI_DEBUG_MODE_APSTA;
    bool start_sta = mode == WIFI_DEBUG_MODE_STA || mode == WIFI_DEBUG_MODE_APSTA;

    /* Diagnostic: print WiFi configuration summary (no plaintext password). */
    {
        char diag_msg[192];
        snprintf(diag_msg,
                 sizeof(diag_msg),
                 "wifi config: mode=%s sta_ssid_set=%d password_set=%d static_ip_set=%d udp_port=%lu",
                 params->wifi_mode,
                 params->sta_ssid[0] != '\0' ? 1 : 0,
                 params->sta_password[0] != '\0' ? 1 : 0,
                 params->sta_static_ip[0] != '\0' ? 1 : 0,
                 (unsigned long)params->wifi_udp_port);
        console_send_event_text(diag_msg);
    }

    if (mode == WIFI_DEBUG_MODE_SOFTAP && !params->wifi_ap_enable) {
        wifi_ap_event_log("softap disabled by wifi_ap_enable=0");
        return ESP_OK;
    }

    if (start_sta && params->sta_ssid[0] == '\0') {
        if (mode == WIFI_DEBUG_MODE_STA) {
            wifi_ap_event_log("sta_ssid empty; falling back to softap");
            start_ap = true;
            start_sta = false;
            mode = WIFI_DEBUG_MODE_SOFTAP;
        } else {
            wifi_ap_event_log("apsta sta side disabled because sta_ssid is empty");
            start_sta = false;
        }
    }

    esp_err_t err = esp_netif_init();
    if ((err = wifi_log_and_return("esp_netif_init", err)) != ESP_OK) {
        return err;
    }

    err = esp_event_loop_create_default();
    if ((err = wifi_log_and_return("esp_event_loop_create_default", err)) != ESP_OK) {
        return err;
    }

    if ((start_ap || mode == WIFI_DEBUG_MODE_STA) && s_ap_netif == NULL) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        if (s_ap_netif == NULL) {
            wifi_ap_event_log("softap netif create failed");
            return ESP_FAIL;
        }
    }
    if (start_sta && s_sta_netif == NULL) {
        s_sta_netif = esp_netif_create_default_wifi_sta();
        if (s_sta_netif == NULL) {
            wifi_ap_event_log("sta netif create failed; falling back to softap");
            start_sta = false;
            start_ap = true;
            mode = WIFI_DEBUG_MODE_SOFTAP;
        }
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if ((err = wifi_log_and_return("esp_wifi_init", err)) != ESP_OK) {
        return err;
    }

    esp_event_handler_instance_t wifi_handler = NULL;
    err = esp_event_handler_instance_register(WIFI_EVENT,
                                              ESP_EVENT_ANY_ID,
                                              &wifi_ap_event_handler,
                                              NULL,
                                              &wifi_handler);
    if ((err = wifi_log_and_return("esp_event_handler_instance_register_wifi", err)) != ESP_OK) {
        return err;
    }

    esp_event_handler_instance_t ip_handler = NULL;
    err = esp_event_handler_instance_register(IP_EVENT,
                                              IP_EVENT_STA_GOT_IP,
                                              &wifi_ap_event_handler,
                                              NULL,
                                              &ip_handler);
    if ((err = wifi_log_and_return("esp_event_handler_instance_register_ip", err)) != ESP_OK) {
        return err;
    }

    wifi_prepare_ap_config(params, &s_ap_wifi_config);
    wifi_config_t sta_wifi_config = {0};
    if (start_sta) {
        err = wifi_configure_sta_ip(s_sta_netif, params);
        if ((err = wifi_log_and_return("sta_static_ip", err)) != ESP_OK) {
            return err;
        }
        wifi_prepare_sta_config(params, &sta_wifi_config);
    }

    const wifi_mode_t esp_mode =
        (start_ap && start_sta) ? WIFI_MODE_APSTA : (start_sta ? WIFI_MODE_STA : WIFI_MODE_AP);
    err = esp_wifi_set_mode(esp_mode);
    if ((err = wifi_log_and_return("esp_wifi_set_mode", err)) != ESP_OK) {
        return err;
    }

    if (start_ap) {
        err = esp_wifi_set_config(WIFI_IF_AP, &s_ap_wifi_config);
        if ((err = wifi_log_and_return("esp_wifi_set_config_ap", err)) != ESP_OK) {
            return err;
        }
    }
    if (start_sta) {
        err = esp_wifi_set_config(WIFI_IF_STA, &sta_wifi_config);
        if ((err = wifi_log_and_return("esp_wifi_set_config_sta", err)) != ESP_OK) {
            return err;
        }
    }

    err = esp_wifi_start();
    if ((err = wifi_log_and_return("esp_wifi_start", err)) != ESP_OK) {
        return err;
    }

    s_wifi_started = true;
    s_softap_active = start_ap;
    s_sta_active = start_sta;
    s_configured_mode = mode;
    s_sta_retry_count = 0;

    if (start_ap) {
        wifi_log_softap_ready(mode == WIFI_DEBUG_MODE_APSTA ? "apsta softap started" : "softap started");
    }
    if (start_sta) {
        char message[192];
        snprintf(message,
                 sizeof(message),
                 "%s connecting ssid=%s udp_port=%lu",
                 wifi_debug_mode_text(mode),
                 params->sta_ssid,
                 (unsigned long)params->wifi_udp_port);
        wifi_ap_event_log(message);
        err = esp_wifi_connect();
        if (err != ESP_OK) {
            snprintf(message, sizeof(message), "sta connect start failed: %s", esp_err_to_name(err));
            wifi_enter_safe_state(message);
            if (mode == WIFI_DEBUG_MODE_STA) {
                wifi_schedule_softap_fallback();
            }
        }
    }

    return ESP_OK;
}
