#pragma once

#include <stdbool.h>

typedef enum {
    PREFLIGHT_LINK_USB = 0,
    PREFLIGHT_LINK_UDP = 1,
} preflight_link_t;

typedef void (*preflight_report_fn_t)(const char *line, void *ctx);

bool preflight_check_stabilize_min(preflight_link_t link,
                                   preflight_report_fn_t report_fn,
                                   void *report_ctx);
