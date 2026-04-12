#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "console_protocol.h"
#include "esp_drone_types.h"

typedef struct {
    float throttle;
    axis3f_t axis;
    bool timed_out;
    bool should_disarm;
} udp_manual_control_t;

void udp_manual_init(void);
void udp_manual_reset(void);

console_cmd_status_t udp_manual_enable(void);
console_cmd_status_t udp_manual_disable(void);
console_cmd_status_t udp_manual_stop(void);
console_cmd_status_t udp_manual_takeoff(void);
console_cmd_status_t udp_manual_land(void);
console_cmd_status_t udp_manual_setpoint(float throttle, float pitch, float roll, float yaw);

bool udp_manual_get_control(uint64_t now_us, uint32_t loop_dt_us, udp_manual_control_t *out_control);
