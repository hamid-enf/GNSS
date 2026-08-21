// tests/test_nmea.c - تست واحد NMEA
#include "gnss.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define ASSERT(cond, msg) do { if (!(cond)) { printf("[FAIL] %s\n", msg); return 1; } else { printf("[PASS] %s\n", msg); } } while(0)
#define ASSERT_FLOAT_NEAR(a,b,eps,msg) do { if (fabs((a)-(b)) > (eps)) { printf("[FAIL] %s: %.6f vs %.6f\n", msg, (double)(a), (double)(b)); return 1; } else { printf("[PASS] %s\n", msg); } } while(0)

int main(void)
{
    printf("=== GNSS NMEA Test ===\n");
    gnss_init();

    // تست 1: GGA
    const char* gga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    gnss_error_t err = gnss_parse((const uint8_t*)gga, strlen(gga));
    printf("GGA parse err=%d\n", err);
    ASSERT(err == GNSS_OK, "GGA parse OK");

    ASSERT(gnss_has_new_solution(), "has new solution after GGA");
    gnss_solution_t sol;
    memset(&sol, 0, sizeof(sol));
    err = gnss_get_solution(&sol);
    ASSERT(err == GNSS_OK, "get solution after GGA");
    ASSERT(sol.valid_position, "valid_position after GGA");
    ASSERT_FLOAT_NEAR(sol.latitude_deg, 48.1173, 0.001, "latitude GGA");
    ASSERT_FLOAT_NEAR(sol.longitude_deg, 11.516666, 0.001, "longitude GGA");
    ASSERT(sol.satellites == 8, "satellites GGA");
    printf("  -> lat=%.6f lon=%.6f alt=%.1f sats=%d hdop=%.1f\n", sol.latitude_deg, sol.longitude_deg, sol.altitude_m, sol.satellites, sol.hdop);

    // تست 2: RMC
    gnss_init();
    const char* rmc = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
    err = gnss_parse((const uint8_t*)rmc, strlen(rmc));
    printf("RMC parse err=%d\n", err);
    ASSERT(err == GNSS_OK, "RMC parse OK");
    ASSERT(gnss_has_new_solution(), "has solution after RMC");
    memset(&sol, 0, sizeof(sol));
    err = gnss_get_solution(&sol);
    ASSERT(err == GNSS_OK, "get solution RMC");
    ASSERT(sol.valid_position && sol.valid_velocity, "valid pos+vel after RMC");
    printf("  -> lat=%.6f lon=%.6f speed=%.2f m/s course=%.1f\n", sol.latitude_deg, sol.longitude_deg, sol.speed_mps, sol.course_deg);
    // 22.4 knots = ~11.52 m/s
    ASSERT_FLOAT_NEAR(sol.speed_mps, 11.52, 0.05, "speed RMC");

    // تست 3: GSV Aggregation
    gnss_init();
    const char* gsv1 = "$GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74\r\n";
    const char* gsv2 = "$GPGSV,3,2,11,14,25,170,00,16,57,208,39,18,67,296,40,19,40,246,00*74\r\n";
    const char* gsv3 = "$GPGSV,3,3,11,22,42,067,42,24,14,311,43,27,05,244,00,31,25,066,00*78\r\n";
    
    err = gnss_parse((const uint8_t*)gsv1, strlen(gsv1));
    ASSERT(err == GNSS_OK, "GSV1 OK");
    err = gnss_parse((const uint8_t*)gsv2, strlen(gsv2));
    ASSERT(err == GNSS_OK, "GSV2 OK");
    err = gnss_parse((const uint8_t*)gsv3, strlen(gsv3));
    ASSERT(err == GNSS_OK, "GSV3 OK");

    uint8_t count = 0;
    err = gnss_get_satellite_count(&count);
    ASSERT(err == GNSS_OK, "get sat count OK");
    printf("  GSV satellite count=%d\n", count);
    // دیتای واقعی 12 ماهواره دارد (header 11 ولی payload 12)
    ASSERT(count == 12, "GSV aggregation count 12");

    gnss_satellite_t sat;
    err = gnss_get_satellite(0, &sat);
    ASSERT(err == GNSS_OK, "get sat 0");
    printf("  sat0 prn=%d elev=%d az=%d snr=%d\n", sat.prn, sat.elevation, sat.azimuth, sat.snr);

    // تست 4: Byte-by-byte feeding (UART ISR simulation)
    gnss_init();
    const char* stream = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    for (size_t i = 0; i < strlen(stream); i++) {
        err = gnss_feed_byte((uint8_t)stream[i]);
    }
    ASSERT(gnss_has_new_solution(), "byte-by-byte GGA");
    ASSERT(gnss_get_solution(&sol) == GNSS_OK, "get solution byte-by-byte");

    // تست 5: Bad checksum should be rejected
    gnss_init();
    const char* bad = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00\r\n";
    err = gnss_parse((const uint8_t*)bad, strlen(bad));
    ASSERT(err == GNSS_ERR_BAD_CHECKSUM, "bad checksum rejected");
    ASSERT(!gnss_has_new_solution(), "no solution after bad checksum");

    // تست 6: GSA + VTG
    gnss_init();
    const char* gsa = "$GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39\r\n";
    err = gnss_parse((const uint8_t*)gsa, strlen(gsa));
    ASSERT(err == GNSS_OK, "GSA parse OK");
    ASSERT(gnss_has_new_solution(), "has solution after GSA");
    gnss_get_solution(&sol);
    printf("  GSA pdop=%.1f hdop=%.1f vdop=%.1f fix=%d\n", sol.pdop, sol.hdop, sol.vdop, sol.fix_type);

    const char* vtg = "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48\r\n";
    err = gnss_parse((const uint8_t*)vtg, strlen(vtg));
    ASSERT(err == GNSS_OK, "VTG parse OK");

    // تست 7: Mixed GN talker ID
    gnss_init();
    const char* gn_gga = "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*59\r\n";
    err = gnss_parse((const uint8_t*)gn_gga, strlen(gn_gga));
    ASSERT(err == GNSS_OK, "GNGGA OK (multi-constellation)");

    printf("\n=== ALL NMEA TESTS PASSED ===\n");
    return 0;
}
