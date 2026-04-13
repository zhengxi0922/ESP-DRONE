#pragma once

#include "barometer.h"
#include "esp_drone_types.h"

void udp_protocol_task(void *arg);
void udp_protocol_send_telemetry(const imu_sample_t *imu_sample,
                                 uint32_t sample_seq,
                                 const barometer_state_t *baro_state,
                                 float battery_voltage,
                                 int battery_raw);
