// examples/simple/simple_buffer.c
// ساده‌ترین مثال: یک بافر ورودی، نمایش خروجی

#include "gnss.h"
#include <stdio.h>
#include <string.h>

int main(void)
{
    gnss_init();

    // فقط یک جمله GGA
    const char* my_buffer = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";

    printf("Input: %s", my_buffer);

    gnss_error_t err = gnss_parse((const uint8_t*)my_buffer, strlen(my_buffer));
    if (err != GNSS_OK) {
        printf("Parse failed: %d\n", err);
        return 1;
    }

    if (gnss_has_new_solution()) {
        gnss_solution_t sol;
        gnss_get_solution(&sol);
        printf("Output:\n");
        printf("  Lat: %.6f\n", sol.latitude_deg);
        printf("  Lon: %.6f\n", sol.longitude_deg);
        printf("  Alt: %.1f m\n", sol.altitude_m);
        printf("  Sats: %d\n", sol.satellites);
    }

    return 0;
}
