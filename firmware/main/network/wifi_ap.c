#include "wifi_ap.h"

#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "console.h"
#include "params.h"

#define WIFI_AP_MAX_CONNECTIONS 2

static const char *TAG = "wifi_ap";
static bool s_wifi_started;

static void wifi_ap_event_log(const char *message)
{
    ESP_LOGI(TAG, "%s", message);
    console_send_event_text(message);
}

static void wifi_ap_event_handler(void *arg,
                                  esp_event_base_t event_base,
                                  int32_t event_id,
                                  void *event_data)
{
    (void)arg;

    if (event_base != WIFI_EVENT) {
        return;
    }

    char message[128];
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
    }
}

static esp_err_t wifi_ap_log_and_return(const char *step, esp_err_t err)
{
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
        return ESP_OK;
    }

    char message[128];
    snprintf(message, sizeof(message), "softap start failed at %s: %s", step, esp_err_to_name(err));
    ESP_LOGE(TAG, "%s", message);
    console_send_event_text(message);
    return err;
}

esp_err_t wifi_ap_start(void)
{
    if (s_wifi_started) {
        return ESP_OK;
    }

    const params_store_t *params = params_get();
    if (!params->wifi_ap_enable) {
        wifi_ap_event_log("softap disabled by wifi_ap_enable=0");
        return ESP_OK;
    }

    esp_err_t err = esp_netif_init();
    if ((err = wifi_ap_log_and_return("esp_netif_init", err)) != ESP_OK) {
        return err;
    }

    err = esp_event_loop_create_default();
    if ((err = wifi_ap_log_and_return("esp_event_loop_create_default", err)) != ESP_OK) {
        return err;
    }

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    if (ap_netif == NULL) {
        const char *message = "softap start failed: esp_netif_create_default_wifi_ap returned NULL";
        ESP_LOGE(TAG, "%s", message);
        console_send_event_text(message);
        return ESP_FAIL;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if ((err = wifi_ap_log_and_return("esp_wifi_init", err)) != ESP_OK) {
        return err;
    }

    esp_event_handler_instance_t any_id = NULL;
    err = esp_event_handler_instance_register(WIFI_EVENT,
                                              ESP_EVENT_ANY_ID,
                                              &wifi_ap_event_handler,
                                              NULL,
                                              &any_id);
    if ((err = wifi_ap_log_and_return("esp_event_handler_instance_register", err)) != ESP_OK) {
        return err;
    }

    wifi_config_t wifi_config = {0};
    snprintf((char *)wifi_config.ap.ssid, sizeof(wifi_config.ap.ssid), "%s", WIFI_AP_DEFAULT_SSID);
    snprintf((char *)wifi_config.ap.password, sizeof(wifi_config.ap.password), "%s", WIFI_AP_DEFAULT_PASSWORD);
    wifi_config.ap.ssid_len = strlen(WIFI_AP_DEFAULT_SSID);
    wifi_config.ap.channel = params->wifi_ap_channel;
    wifi_config.ap.max_connection = WIFI_AP_MAX_CONNECTIONS;
    wifi_config.ap.authmode = strlen(WIFI_AP_DEFAULT_PASSWORD) >= 8 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    wifi_config.ap.pmf_cfg.required = false;

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if ((err = wifi_ap_log_and_return("esp_wifi_set_mode", err)) != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    if ((err = wifi_ap_log_and_return("esp_wifi_set_config", err)) != ESP_OK) {
        return err;
    }

    err = esp_wifi_start();
    if ((err = wifi_ap_log_and_return("esp_wifi_start", err)) != ESP_OK) {
        return err;
    }

    esp_netif_ip_info_t ip_info = {0};
    const char *ip = WIFI_AP_DEFAULT_IP;
    if (esp_netif_get_ip_info(ap_netif, &ip_info) == ESP_OK) {
        static char ip_text[16];
        snprintf(ip_text,
                 sizeof(ip_text),
                 IPSTR,
                 IP2STR(&ip_info.ip));
        ip = ip_text;
    }

    char message[160];
    snprintf(message,
             sizeof(message),
             "softap started ssid=%s channel=%u ip=%s udp_port=%lu",
             WIFI_AP_DEFAULT_SSID,
             (unsigned)wifi_config.ap.channel,
             ip,
             (unsigned long)params->wifi_udp_port);
    wifi_ap_event_log(message);
    s_wifi_started = true;
    return ESP_OK;
}
