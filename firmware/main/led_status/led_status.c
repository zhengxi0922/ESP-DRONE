#include "led_status.h"

#include "driver/gpio.h"

#include "board_config.h"

typedef struct {
    bool g;
    bool r;
    bool y;
} led_levels_t;

static led_state_t s_state = LED_STATE_INIT_WAIT_IMU;
static bool s_initialized;

static void led_status_write(led_levels_t levels)
{
    const board_config_t *board = board_config_get();
    gpio_set_level(board->led_g, levels.g ? 1 : 0);
    gpio_set_level(board->led_r, levels.r ? 1 : 0);
    gpio_set_level(board->led_y, levels.y ? 1 : 0);
}

esp_err_t led_status_init(void)
{
    const board_config_t *board = board_config_get();
    const gpio_config_t io_cfg = {
        .pin_bit_mask =
            (1ULL << board->led_g) |
            (1ULL << board->led_r) |
            (1ULL << board->led_y),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io_cfg);
    if (err != ESP_OK) {
        return err;
    }

    s_initialized = true;
    led_status_write((led_levels_t){0});
    return ESP_OK;
}

void led_status_set_state(led_state_t state)
{
    s_state = state;
}

led_state_t led_status_get_state(void)
{
    return s_state;
}

void led_status_service(uint32_t now_ms)
{
    if (!s_initialized) {
        return;
    }

    const bool slow_blink = ((now_ms / 500U) % 2U) == 0U;
    const bool fast_blink = ((now_ms / 125U) % 2U) == 0U;
    const bool alternate = ((now_ms / 250U) % 2U) == 0U;

    switch (s_state) {
    case LED_STATE_INIT_WAIT_IMU:
        led_status_write((led_levels_t){.y = slow_blink});
        break;
    case LED_STATE_DISARMED_READY:
        led_status_write((led_levels_t){.y = true});
        break;
    case LED_STATE_ARMED_HEALTHY:
        led_status_write((led_levels_t){.g = true});
        break;
    case LED_STATE_LOW_BAT:
        led_status_write((led_levels_t){.r = slow_blink});
        break;
    case LED_STATE_FAILSAFE:
    case LED_STATE_IMU_ERROR:
    case LED_STATE_RC_LOSS:
        led_status_write((led_levels_t){.r = fast_blink});
        break;
    case LED_STATE_FAULT_LOCK:
        led_status_write((led_levels_t){.r = true});
        break;
    case LED_STATE_CALIBRATING:
    case LED_STATE_PARAM_SAVE:
        led_status_write((led_levels_t){.g = alternate, .y = !alternate});
        break;
    }
}
