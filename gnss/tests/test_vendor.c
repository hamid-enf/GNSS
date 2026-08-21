// tests/test_vendor.c - تست Vendor Extensions Quectel + Trimble
#include "gnss.h"
#include <stdio.h>
#include <string.h>

int main(void)
{
    printf("=== Vendor Extensions Test ===\n");
    gnss_init();

    // Quectel GSV (با PQ) - checksum *61
    const char* pqgsv = "$PQGSV,1,1,04,02,10,090,35,04,20,120,40,05,30,200,45,07,40,250,42*61\r\n";
    gnss_error_t err = gnss_parse((const uint8_t*)pqgsv, strlen(pqgsv));
    printf("PQGSV parse err=%d\n", err);
    if (err!=GNSS_OK) { printf("[FAIL] PQGSV\n"); return 1; }

    uint8_t cnt=0;
    gnss_get_satellite_count(&cnt);
    printf("After PQGSV count=%d\n", cnt);

    // Quectel QGNS - checksum *64
    const char* pqgns = "$PQGNS,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,46.9,,*64\r\n";
    err = gnss_parse((const uint8_t*)pqgns, strlen(pqgns));
    printf("PQGNS parse err=%d has_sol=%d\n", err, gnss_has_new_solution());
    if (err!=GNSS_OK) { printf("[FAIL] PQGNS should be OK\n"); return 1; }

    // Trimble PTNL,AVR - checksum *76
    const char* avr = "$PTNL,AVR,123519.00,45.0,Yaw,1.2,Tilt,0.5,Roll,10.0,Range,2,1,1.5,1*76\r\n";
    err = gnss_parse((const uint8_t*)avr, strlen(avr));
    printf("PTNL,AVR parse err=%d\n", err);
    if (err!=GNSS_OK) { printf("[FAIL] AVR\n"); return 1; }

    // Trimble PTNL,GGK - RTK - checksum *23
    const char* ggk = "$PTNL,GGK,123519.00,071219,4807.038,N,01131.000,E,4,08,0.9,545.4,M*23\r\n";
    err = gnss_parse((const uint8_t*)ggk, strlen(ggk));
    printf("PTNL,GGK parse err=%d has_sol=%d\n", err, gnss_has_new_solution());
    if (err!=GNSS_OK) { printf("[FAIL] GGK\n"); return 1; }

    gnss_solution_t sol;
    if (gnss_get_solution(&sol)==GNSS_OK) {
        printf("  GGK sol lat=%.6f lon=%.6f alt=%.1f fix=%d sats=%d vendor=%d rtk=%d\n",
            sol.latitude_deg, sol.longitude_deg, sol.altitude_m, sol.fix_type, sol.satellites, sol.last_vendor, sol.rtk_fix_type);
    }

    // PQTXT - checksum *35
    const char* txt = "$PQTXT,W,01,01,01,NMEA unknown msg*35\r\n";
    err = gnss_parse((const uint8_t*)txt, strlen(txt));
    printf("PQTXT parse err=%d (should be OK)\n", err);

    // PMTK - MTK checksum *2E
    const char* pmtk = "$PMTK010,001*2E\r\n";
    err = gnss_parse((const uint8_t*)pmtk, strlen(pmtk));
    printf("PMTK010 parse err=%d\n", err);

    // بررسی last vendor msg
    gnss_vendor_msg_t vm;
    gnss_get_last_vendor_msg(&vm);
    printf("Last vendor: id=%d talker=%s type=%s subtype=%s raw=%.40s\n", vm.vendor, vm.talker, vm.type, vm.subtype, vm.raw);

    printf("\n=== VENDOR TEST PASSED ===\n");
    return 0;
}
