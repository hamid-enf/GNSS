// examples/vendor/vendor_example.c
// مثال Vendor Extensions - Quectel + Trimble

#include "gnss.h"
#include <stdio.h>
#include <string.h>

static gnss_error_t custom_parser(char* fields[], int count, const char* talker, const char* full_id, const char* raw)
{
    (void)fields; (void)count; (void)talker;
    if (strcmp(full_id, "PXYZ") == 0) {
        printf("  Custom parser caught PXYZ: %s\n", raw);
        return GNSS_OK;
    }
    return GNSS_ERR_UNSUPPORTED_MESSAGE;
}

int main(void)
{
    printf("=== Vendor Extensions Example ===\n");
    printf("پشتیبانی از Quectel و Trimble\n\n");
    gnss_init();

    // --- Quectel ---
    printf("[Quectel] Parsing PQGSV + PQGNS + PQTXT\n");
    const char* quectel_stream =
        "$PQGSV,1,1,04,02,10,090,35,04,20,120,40,05,30,200,45,07,40,250,42*61\r\n"
        "$PQGNS,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,46.9,,*64\r\n"
        "$PQTXT,W,01,01,01,NMEA unknown msg*35\r\n";

    gnss_parse((const uint8_t*)quectel_stream, strlen(quectel_stream));

    if (gnss_has_new_solution()) {
        gnss_solution_t sol;
        gnss_get_solution(&sol);
        printf("  Quectel Solution: lat=%.6f lon=%.6f vendor=%d\n",
            sol.latitude_deg, sol.longitude_deg, sol.last_vendor);
    }

    gnss_vendor_msg_t vm;
    if (gnss_get_last_vendor_msg(&vm)==GNSS_OK) {
        printf("  Last Vendor Msg: %s %s raw=%.50s\n", vm.talker, vm.type, vm.raw);
    }

    // --- Trimble ---
    printf("\n[Trimble] Parsing PTNL,AVR + PTNL,GGK (RTK)\n");
    const char* trimble_stream =
        "$PTNL,AVR,123519.00,45.0,Yaw,1.2,Tilt,0.5,Roll,10.0,Range,2,1,1.5,1*76\r\n"
        "$PTNL,GGK,123519.00,071219,4807.038,N,01131.000,E,4,08,0.9,545.4,M*23\r\n";

    gnss_parse((const uint8_t*)trimble_stream, strlen(trimble_stream));

    if (gnss_has_new_solution()) {
        gnss_solution_t sol;
        gnss_get_solution(&sol);
        printf("  Trimble Solution: lat=%.6f lon=%.6f fix=%d rtk=%d vendor=%d\n",
            sol.latitude_deg, sol.longitude_deg, sol.fix_type, sol.rtk_fix_type, sol.last_vendor);
    }

    // --- Custom Vendor Parser ---
    printf("\n[Custom] Registering custom parser for $PXYZ\n");
    gnss_vendor_register_parser(custom_parser);

    const char* custom = "$PXYZ,1,2,3*4D\r\n";
    gnss_parse((const uint8_t*)custom, strlen(custom));

    printf("\n=== Vendor Example Finished ===\n");
    return 0;
}
