// examples/baremetal/main.c
// مثال Bare-Metal ساده - بدون OS، بدون malloc، فقط یک بافر ورودی
// این مثال همان چیزی است که در README وعده داده شده بود

#include "gnss.h"
#include <stdio.h>
#include <string.h>

// شبیه‌سازی بافر دریافتی از UART/DMA
static const char* sample_nmea_stream =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n"
    "$GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39\r\n"
    "$GPGSV,2,1,08,02,10,090,35,04,20,120,40,05,30,200,45,07,40,250,42*78\r\n"
    "$GPGSV,2,2,08,09,50,300,44,12,60,320,46,15,10,040,30,18,20,180,33*78\r\n"
    "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n"
    "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48\r\n";

int main(void)
{
    printf("=== GNSS Bare-Metal Example ===\n");
    printf("کتابخانه gnss-core - مثال ساده بدون OS\n\n");

    // 1. مقداردهی اولیه
    gnss_init();
    printf("[1] gnss_init() done\n");

    // 2. تغذیه بافر کامل (روش 1)
    printf("[2] Parsing full buffer (%zu bytes)...\n", strlen(sample_nmea_stream));
    gnss_error_t err = gnss_parse((const uint8_t*)sample_nmea_stream, strlen(sample_nmea_stream));
    printf("    gnss_parse() = %d (%s)\n", err, err==GNSS_OK ? "OK" : "ERR");

    // 3. بررسی و دریافت solution
    if (gnss_has_new_solution()) {
        gnss_solution_t sol;
        if (gnss_get_solution(&sol) == GNSS_OK) {
            printf("\n[3] === Solution ===\n");
            printf("    Latitude : %.7f %s\n", sol.latitude_deg, sol.valid_position ? "VALID" : "INVALID");
            printf("    Longitude: %.7f\n", sol.longitude_deg);
            printf("    Altitude : %.2f m\n", sol.altitude_m);
            printf("    Speed    : %.2f m/s (%.1f km/h)\n", sol.speed_mps, sol.speed_mps*3.6);
            printf("    Course   : %.1f deg\n", sol.course_deg);
            printf("    Fix Type : %d (%s)\n", sol.fix_type, sol.fix_type>=2 ? "2D/3D" : "No Fix");
            printf("    Sats     : %d\n", sol.satellites);
            printf("    HDOP/VDOP/PDOP: %.1f / %.1f / %.1f\n", sol.hdop, sol.vdop, sol.pdop);
            if (sol.valid_time) {
                printf("    Time     : %04d-%02d-%02d %02d:%02d:%02d\n",
                    sol.year, sol.month, sol.day, sol.hour, sol.minute, sol.second);
            }
        }
    }

    // 4. نمایش ماهواره‌ها (GSV + GSA used flag)
    uint8_t sat_count=0;
    if (gnss_get_satellite_count(&sat_count)==GNSS_OK) {
        printf("\n[4] === Satellites (%d) ===\n", sat_count);
        for (uint8_t i=0;i<sat_count;i++) {
            gnss_satellite_t sat;
            if (gnss_get_satellite(i, &sat)==GNSS_OK) {
                printf("    PRN %02d: Elev %2d Az %3d SNR %2d %s %s\n",
                    sat.prn, sat.elevation, sat.azimuth, sat.snr,
                    sat.used_in_solution ? "[USED]" : "      ",
                    sat.gnss_id==0 ? "GPS" : sat.gnss_id==1 ? "GLONASS" : sat.gnss_id==2 ? "Galileo" : "Other");
            }
        }
    }

    // 5. API سطح 2 - get_field
    printf("\n[5] === Field API (Level 2) ===\n");
    gnss_float_t lat=0; size_t sz=sizeof(lat);
    if (gnss_get_field(GNSS_FIELD_LATITUDE, &lat, &sz)==GNSS_OK) {
        printf("    GNSS_FIELD_LATITUDE = %.7f\n", lat);
    }

    // 6. آمار
#if GNSS_ENABLE_STATS
    gnss_stats_t stats;
    if (gnss_get_stats(&stats)==GNSS_OK) {
        printf("\n[6] === Stats ===\n");
        printf("    Received: %u, Parsed: %u\n", stats.messages_received, stats.messages_parsed);
        printf("    NMEA: %u, UBX: %u, Vendor: %u\n", stats.nmea_messages, stats.ubx_messages, stats.vendor_messages);
        printf("    Checksum Err: %u, Frame Err: %u, Overflow: %u\n",
            stats.checksum_errors, stats.frame_errors, stats.buffer_overflows);
    }
#endif

    // 7. روش بایت‌به‌بایت (مناسب ISR)
    printf("\n[7] === Byte-by-Byte Feeding (UART ISR simulation) ===\n");
    gnss_init();
    const char* gga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    for (size_t i=0;i<strlen(gga);i++) {
        gnss_feed_byte((uint8_t)gga[i]);
    }
    if (gnss_has_new_solution()) {
        gnss_solution_t sol;
        gnss_get_solution(&sol);
        printf("    Got solution via feed_byte: lat=%.6f lon=%.6f\n", sol.latitude_deg, sol.longitude_deg);
    }

    printf("\n=== Example Finished ===\n");
    return 0;
}
