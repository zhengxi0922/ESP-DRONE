#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "esp_drone_types.h"

typedef struct {
    uint32_t published_seq;
    uint32_t good_frames;
    uint32_t parse_errors;
    uint32_t consecutive_parse_errors;
    uint64_t last_frame_us;
} imu_stats_t;

esp_err_t imu_init(void);
esp_err_t imu_reconfigure_from_params(void);
void imu_service_rx(void);

bool imu_get_latest(imu_sample_t *out_sample, uint32_t *out_seq);
imu_stats_t imu_get_stats(void);
const char *imu_health_to_string(imu_health_t health);
