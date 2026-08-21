// gnss/include/gnss_callbacks.h
// سیستم Callback (اختیاری و بدون overhead وقتی غیرفعال باشد)

#ifndef GNSS_CALLBACKS_H
#define GNSS_CALLBACKS_H

#include "gnss_types.h"
#include "gnss_config.h"

#if GNSS_ENABLE_CALLBACKS

typedef enum {
    GNSS_EVENT_SOLUTION = 0,
    GNSS_EVENT_POSITION,
    GNSS_EVENT_SATELLITES,
    GNSS_EVENT_ERROR
} gnss_event_type_t;

typedef struct {
    gnss_event_type_t type;
    union {
        const gnss_solution_t* solution;
        const gnss_satellite_t* satellites;
        gnss_error_t error;
    } data;
    uint8_t satellite_count;
} gnss_event_t;

typedef void (*gnss_callback_t)(const gnss_event_t* event);

void gnss_register_callback(gnss_callback_t callback);

#endif // GNSS_ENABLE_CALLBACKS

#endif // GNSS_CALLBACKS_H
