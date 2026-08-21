// examples/simple/callback_example.c
// مثال Callback - مناسب برای سیستم‌های event-driven

#include "gnss.h"
#include <stdio.h>
#include <string.h>

void my_gnss_callback(const gnss_event_t* event)
{
    if (!event) return;
    switch (event->type) {
        case GNSS_EVENT_SOLUTION:
            printf("[CALLBACK] Solution: lat=%.6f lon=%.6f alt=%.1f sats=%d fix=%d\n",
                event->data.solution->latitude_deg,
                event->data.solution->longitude_deg,
                event->data.solution->altitude_m,
                event->data.solution->satellites,
                event->data.solution->fix_type);
            break;
        case GNSS_EVENT_SATELLITES:
            printf("[CALLBACK] Satellites updated: %d sats\n", event->satellite_count);
            for (int i=0;i<event->satellite_count && i<3;i++) {
                printf("  PRN %d SNR %d %s\n",
                    event->data.satellites[i].prn,
                    event->data.satellites[i].snr,
                    event->data.satellites[i].used_in_solution ? "USED" : "");
            }
            break;
        case GNSS_EVENT_ERROR:
            printf("[CALLBACK] Error: %d\n", event->data.error);
            break;
        default:
            break;
    }
}

int main(void)
{
    printf("=== Callback Example ===\n");
    gnss_init();
    gnss_register_callback(my_gnss_callback);

    const char* data =
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n"
        "$GPGSV,2,1,08,02,10,090,35,04,20,120,40,05,30,200,45,07,40,250,42*78\r\n"
        "$GPGSV,2,2,08,09,50,300,44,12,60,320,46,15,10,040,30,18,20,180,33*78\r\n"
        "$GPGSA,A,3,02,04,05,09,,,,,,,,2.5,1.3,2.1*12\r\n";

    gnss_parse((const uint8_t*)data, strlen(data));

    return 0;
}
