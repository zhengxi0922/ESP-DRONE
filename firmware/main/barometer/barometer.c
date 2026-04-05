/**
 * @file barometer.c
 * @brief 气压计状态缓存实现。
 */

#include "barometer.h"

#include <math.h>
#include <stdatomic.h>

#include "esp_timer.h"

#define BARO_STALE_TIMEOUT_US 500000u
#define BARO_MIN_PRESSURE_PA 30000.0f
#define BARO_MAX_PRESSURE_PA 120000.0f
#define BARO_MIN_TEMP_C -40.0f
#define BARO_MAX_TEMP_C 125.0f

static barometer_state_t s_baro_samples[2];
static altitude_hold_reserved_state_t s_altitude_reserved_samples[2];
static _Atomic uint32_t s_sample_index;
static _Atomic uint32_t s_sample_seq;
static bool s_initialized;

static bool barometer_is_value_valid(float pressure_pa, float temperature_c)
{
    return isfinite(pressure_pa) && isfinite(temperature_c) &&
           pressure_pa >= BARO_MIN_PRESSURE_PA &&
           pressure_pa <= BARO_MAX_PRESSURE_PA &&
           temperature_c >= BARO_MIN_TEMP_C &&
           temperature_c <= BARO_MAX_TEMP_C;
}

static void barometer_publish(const barometer_state_t *baro_state,
                              const altitude_hold_reserved_state_t *reserved_state)
{
    const uint32_t next_index = atomic_load(&s_sample_index) ^ 1u;
    s_baro_samples[next_index] = *baro_state;
    s_altitude_reserved_samples[next_index] = *reserved_state;
    atomic_store(&s_sample_index, next_index);
    atomic_fetch_add(&s_sample_seq, 1u);
}

esp_err_t barometer_init(void)
{
    const barometer_state_t initial_baro = {
        .timestamp_us = 0,
        .pressure_pa = 0.0f,
        .temperature_c = 0.0f,
        .altitude_m = 0.0f,
        .vertical_speed_mps = 0.0f,
        .has_baro = false,
        .valid = false,
        .health = BARO_HEALTH_INIT,
        .update_age_us = 0,
    };
    const altitude_hold_reserved_state_t initial_reserved = {
        .target_altitude_m = 0.0f,
        .estimated_altitude_m = 0.0f,
        .estimated_vz_mps = 0.0f,
        .altitude_hold_reserved_enabled = false,
    };

    s_baro_samples[0] = initial_baro;
    s_baro_samples[1] = initial_baro;
    s_altitude_reserved_samples[0] = initial_reserved;
    s_altitude_reserved_samples[1] = initial_reserved;
    atomic_store(&s_sample_index, 0u);
    atomic_store(&s_sample_seq, 0u);
    s_initialized = true;
    return ESP_OK;
}

void barometer_update_from_module_frame(int32_t pressure_pa,
                                        int32_t altitude_cm,
                                        float temperature_c,
                                        uint64_t timestamp_us)
{
    if (!s_initialized) {
        return;
    }

    barometer_state_t previous = {0};
    (void)barometer_get_latest(&previous);

    const float pressure_pa_f = (float)pressure_pa;
    const float altitude_m = (float)altitude_cm * 0.01f;
    const bool valid = barometer_is_value_valid(pressure_pa_f, temperature_c);

    float vertical_speed_mps = 0.0f;
    if (previous.has_baro && previous.valid && valid && previous.timestamp_us != 0 &&
        timestamp_us > previous.timestamp_us) {
        const float dt_s = (float)(timestamp_us - previous.timestamp_us) / 1000000.0f;
        if (dt_s >= 0.005f && dt_s <= 1.0f) {
            const float raw_vspeed = (altitude_m - previous.altitude_m) / dt_s;
            vertical_speed_mps = 0.7f * previous.vertical_speed_mps + 0.3f * raw_vspeed;
        }
    }

    const barometer_state_t baro_state = {
        .timestamp_us = timestamp_us,
        .pressure_pa = pressure_pa_f,
        .temperature_c = temperature_c,
        .altitude_m = altitude_m,
        .vertical_speed_mps = vertical_speed_mps,
        .has_baro = true,
        .valid = valid,
        .health = valid ? BARO_HEALTH_OK : BARO_HEALTH_INVALID,
        .update_age_us = 0,
    };
    const altitude_hold_reserved_state_t reserved_state = {
        .target_altitude_m = 0.0f,
        .estimated_altitude_m = altitude_m,
        .estimated_vz_mps = vertical_speed_mps,
        .altitude_hold_reserved_enabled = false,
    };

    barometer_publish(&baro_state, &reserved_state);
}

bool barometer_get_latest(barometer_state_t *out_state)
{
    if (!s_initialized || out_state == NULL) {
        return false;
    }

    uint32_t seq_before = 0;
    uint32_t seq_after = 0;
    do {
        seq_before = atomic_load(&s_sample_seq);
        const uint32_t index = atomic_load(&s_sample_index);
        *out_state = s_baro_samples[index];
        seq_after = atomic_load(&s_sample_seq);
    } while (seq_before != seq_after);

    if (!out_state->has_baro || out_state->timestamp_us == 0) {
        out_state->health = BARO_HEALTH_INIT;
        out_state->update_age_us = 0;
        return true;
    }

    const uint64_t now_us = (uint64_t)esp_timer_get_time();
    const uint32_t age_us = (uint32_t)(now_us - out_state->timestamp_us);
    out_state->update_age_us = age_us;
    if (out_state->valid) {
        out_state->health = (age_us > BARO_STALE_TIMEOUT_US) ? BARO_HEALTH_STALE : BARO_HEALTH_OK;
    } else {
        out_state->health = BARO_HEALTH_INVALID;
    }
    return true;
}

bool barometer_get_altitude_hold_reserved_state(altitude_hold_reserved_state_t *out_state)
{
    if (!s_initialized || out_state == NULL) {
        return false;
    }

    uint32_t seq_before = 0;
    uint32_t seq_after = 0;
    do {
        seq_before = atomic_load(&s_sample_seq);
        const uint32_t index = atomic_load(&s_sample_index);
        *out_state = s_altitude_reserved_samples[index];
        seq_after = atomic_load(&s_sample_seq);
    } while (seq_before != seq_after);
    return true;
}

const char *barometer_health_to_string(baro_health_t health)
{
    switch (health) {
    case BARO_HEALTH_INIT:
        return "INIT";
    case BARO_HEALTH_OK:
        return "OK";
    case BARO_HEALTH_STALE:
        return "STALE";
    case BARO_HEALTH_INVALID:
        return "INVALID";
    default:
        return "UNKNOWN";
    }
}
