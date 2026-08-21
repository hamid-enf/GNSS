// tests/test_gsa_used.c - تست GSA used_in_solution flag
#include "gnss.h"
#include <stdio.h>
#include <string.h>

int main(void)
{
    printf("=== GSA used_in_solution Test ===\n");
    gnss_init();

    // GSV با 8 ماهواره - checksum درست
    const char* gsv1 = "$GPGSV,2,1,08,02,10,090,35,04,20,120,40,05,30,200,45,07,40,250,42*78\r\n";
    const char* gsv2 = "$GPGSV,2,2,08,09,50,300,44,12,60,320,46,15,10,040,30,18,20,180,33*78\r\n";
    // GSA با 4 ماهواره استفاده شده: 02,04,05,09 - checksum *12
    const char* gsa = "$GPGSA,A,3,02,04,05,09,,,,,,,,2.5,1.3,2.1*12\r\n";

    gnss_parse((const uint8_t*)gsv1, strlen(gsv1));
    gnss_parse((const uint8_t*)gsv2, strlen(gsv2));

    uint8_t cnt=0;
    gnss_get_satellite_count(&cnt);
    printf("After GSV count=%d\n", cnt);
    if (cnt!=8) { printf("[FAIL] expected 8 sats after GSV, got %d\n", cnt); return 1; }

    // قبل از GSA هیچکدام used نیست
    gnss_satellite_t sat;
    gnss_get_satellite(0, &sat);
    printf("Before GSA sat0 prn=%d used=%d\n", sat.prn, sat.used_in_solution);
    if (sat.used_in_solution) { printf("[FAIL] should not be used before GSA\n"); return 1; }

    // حالا GSA
    gnss_parse((const uint8_t*)gsa, strlen(gsa));

    // بعد از GSA باید 02,04,05,09 used باشند
    gnss_get_satellite_count(&cnt);
    int used=0;
    for (uint8_t i=0;i<cnt;i++) {
        gnss_get_satellite(i, &sat);
        printf("  sat[%d] prn=%d used=%d snr=%d\n", i, sat.prn, sat.used_in_solution, sat.snr);
        if (sat.used_in_solution) used++;
    }
    printf("Used count after GSA=%d\n", used);
    if (used!=4) { printf("[FAIL] expected 4 used sats, got %d\n", used); return 1; }

    // تست معکوس: GSA اول، بعد GSV
    gnss_init();
    gnss_parse((const uint8_t*)gsa, strlen(gsa));
    gnss_parse((const uint8_t*)gsv1, strlen(gsv1));
    gnss_parse((const uint8_t*)gsv2, strlen(gsv2));
    gnss_get_satellite_count(&cnt);
    used=0;
    for (uint8_t i=0;i<cnt;i++) {
        gnss_get_satellite(i, &sat);
        if (sat.used_in_solution) used++;
    }
    printf("Reverse order used count=%d\n", used);
    if (used!=4) { printf("[FAIL] reverse order should also have 4 used\n"); return 1; }

    printf("\n=== GSA USED FLAG TEST PASSED ===\n");
    return 0;
}
