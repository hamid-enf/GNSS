// gnss/src/core/gnss_callbacks.c
// پیاده‌سازی Callback (فقط وقتی GNSS_ENABLE_CALLBACKS=1 کامپایل می‌شود)

#include "gnss.h"
#include "gnss_config.h"
#include "gnss_callbacks.h"

#if GNSS_ENABLE_CALLBACKS

static gnss_callback_t g_callback = NULL;

void gnss_register_callback(gnss_callback_t callback)
{
    g_callback = callback;
}

void gnss_internal_fire_callback(gnss_event_type_t type, const void* data, uint8_t count)
{
    if (g_callback) {
        gnss_event_t event;
        event.type = type;
        event.satellite_count = count;

        if (type == GNSS_EVENT_SOLUTION || type == GNSS_EVENT_POSITION) {
            event.data.solution = (const gnss_solution_t*)data;
        } else if (type == GNSS_EVENT_SATELLITES) {
            event.data.satellites = (const gnss_satellite_t*)data;
        } else if (type == GNSS_EVENT_ERROR) {
            event.data.error = *(const gnss_error_t*)data;
        } else {
            // generic fallback to solution
            event.data.solution = (const gnss_solution_t*)data;
        }

        g_callback(&event);
    }
}

#endif // GNSS_ENABLE_CALLBACKS
