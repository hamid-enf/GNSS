// examples/live_demo.c
// دمو زنده و فوق ساده - دقیقاً همونی که خواستی: بافر UART → ترجمه → چاپ خروجی
// بدون پیچیدگی، فقط یک بافر و نمایش کیفیت‌ها

#include "gnss.h"
#include <stdio.h>
#include <string.h>

int main(void)
{
    printf("========================================\n");
    printf("  GNSS Live Demo - ساده‌ترین حالت ممکن\n");
    printf("========================================\n\n");

    // ---------------------------------------------------------
    // مثال 1: یک بافر ساده از UART (فقط یک جمله GGA)
    // ---------------------------------------------------------
    printf("[مثال 1] یک جمله GGA از UART اومده:\n");
    printf("------------------------------------------------\n");

    // فرض کن این داده از UART با DMA اومده
    const char* uart_buffer = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";

    printf("Input Buffer (از UART):\n  %s", uart_buffer);

    gnss_init(); // اول init
    gnss_parse((const uint8_t*)uart_buffer, strlen(uart_buffer)); // ترجمه

    if (gnss_has_new_solution()) {
        gnss_solution_t sol;
        gnss_get_solution(&sol);

        printf("\nOutput (ترجمه شده):\n");
        printf("  ✅ موقعیت معتبر: %s\n", sol.valid_position ? "بله" : "خیر");
        printf("  📍 عرض جغرافیایی (Lat): %.6f درجه\n", sol.latitude_deg);
        printf("  📍 طول جغرافیایی (Lon): %.6f درجه\n", sol.longitude_deg);
        printf("  ⛰️  ارتفاع: %.1f متر\n", sol.altitude_m);
        printf("  🛰️  تعداد ماهواره: %d\n", sol.satellites);
        printf("  📶 HDOP (کیفیت): %.1f (کمتر = بهتر)\n", sol.hdop);
        printf("  🔧 نوع فیکس: %d (1=GPS, 2=2D, 3=3D)\n", sol.fix_type);
    }

    // ---------------------------------------------------------
    // مثال 2: استریم واقعی با GGA + GSA + GSV + RMC (مثل گیرنده واقعی)
    // ---------------------------------------------------------
    printf("\n\n[مثال 2] استریم کامل از گیرنده (مثل u-blox):\n");
    printf("------------------------------------------------\n");

    const char* full_stream =
        "$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*76\r\n"
        "$GPGSA,A,3,10,07,05,02,29,04,08,13,,,,,1.72,1.03,1.38*0A\r\n"
        "$GPGSV,3,1,11,10,63,137,17,07,61,098,15,05,59,290,20,08,54,157,30*70\r\n"
        "$GPGSV,3,2,11,02,39,223,19,13,28,070,17,26,23,252,,04,14,186,14*79\r\n"
        "$GPGSV,3,3,11,29,09,301,24,16,09,020,,36,,,*76\r\n"
        "$GPRMC,092750.000,A,5321.6802,N,00630.3372,W,0.02,31.66,280511,,,A*43\r\n";

    printf("Input: 6 جمله (GGA+GSA+GSV*3+RMC) از UART\n");

    gnss_init();
    gnss_parse((const uint8_t*)full_stream, strlen(full_stream));

    if (gnss_has_new_solution()) {
        gnss_solution_t sol;
        gnss_get_solution(&sol);
        printf("\nOutput نهایی (Merge شده):\n");
        printf("  📍 Lat: %.6f, Lon: %.6f\n", sol.latitude_deg, sol.longitude_deg);
        printf("  ⛰️  Alt: %.1f m\n", sol.altitude_m);
        printf("  🚀 Speed: %.2f m/s (%.1f km/h)\n", sol.speed_mps, sol.speed_mps*3.6);
        printf("  🧭 Course: %.1f°\n", sol.course_deg);
        printf("  📊 DOP: HDOP=%.2f VDOP=%.2f PDOP=%.2f\n", sol.hdop, sol.vdop, sol.pdop);
        printf("  🛰️  Sats Used: %d, Fix: %d\n", sol.satellites, sol.fix_type);
    }

    uint8_t sat_cnt=0;
    gnss_get_satellite_count(&sat_cnt);
    printf("\n  لیست ماهواره‌ها (%d تا) با used flag از GSA:\n", sat_cnt);
    for (int i=0;i<sat_cnt && i<11;i++) {
        gnss_satellite_t sat;
        gnss_get_satellite(i, &sat);
        printf("    PRN %02d | Elev %2d° | Az %3d° | SNR %2d dB | %s\n",
            sat.prn, sat.elevation, sat.azimuth, sat.snr,
            sat.used_in_solution ? "✅ USED" : "   not used");
    }

    // ---------------------------------------------------------
    // مثال 3: بایت به بایت (شبیه‌سازی UART ISR)
    // ---------------------------------------------------------
    printf("\n\n[مثال 3] بایت به بایت - شبیه‌سازی وقفه UART:\n");
    printf("------------------------------------------------\n");

    const char* isr_data = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    printf("Input: داده بایت به بایت از ISR میاد...\n");

    gnss_init();
    for (size_t i=0;i<strlen(isr_data);i++) {
        gnss_feed_byte((uint8_t)isr_data[i]); // هر بایت مثل ISR

        if (gnss_has_new_solution()) {
            gnss_solution_t sol;
            gnss_get_solution(&sol);
            printf("  → بعد از %zu بایت، فیکس گرفتم: Lat=%.6f Lon=%.6f\n", i+1, sol.latitude_deg, sol.longitude_deg);
        }
    }

    // ---------------------------------------------------------
    // مثال 4: API سطح 2 - گرفتن فیلد تکی
    // ---------------------------------------------------------
    printf("\n\n[مثال 4] API سطح 2 - گرفتن فقط یک فیلد:\n");
    printf("------------------------------------------------\n");

    gnss_init();
    gnss_parse((const uint8_t*)"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n", 67);

    gnss_float_t lat=0; size_t sz=sizeof(lat);
    if (gnss_get_field(GNSS_FIELD_LATITUDE, &lat, &sz)==GNSS_OK) {
        printf("  GNSS_FIELD_LATITUDE = %.6f (فقط همین فیلد)\n", lat);
    }

    uint8_t sats=0; sz=sizeof(sats);
    gnss_get_field(GNSS_FIELD_SATELLITES, &sats, &sz);
    printf("  GNSS_FIELD_SATELLITES = %d\n", sats);

    // ---------------------------------------------------------
    // مثال 5: کیفیت‌ها به زبان ساده
    // ---------------------------------------------------------
    printf("\n\n[مثال 5] کیفیت سیگنال به زبان ساده:\n");
    printf("------------------------------------------------\n");

    gnss_init();
    const char* quality_test =
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n"
        "$GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39\r\n";

    gnss_parse((const uint8_t*)quality_test, strlen(quality_test));
    gnss_solution_t sol;
    gnss_get_solution(&sol);

    printf("  Fix Type %d = ", sol.fix_type);
    if (sol.fix_type==0) printf("❌ No Fix\n");
    else if (sol.fix_type==1) printf("⚠️ Dead Reckoning\n");
    else if (sol.fix_type==2) printf("✅ 2D Fix\n");
    else if (sol.fix_type==3) printf("✅✅ 3D Fix (بهترین)\n");

    printf("  HDOP %.1f = ", sol.hdop);
    if (sol.hdop < 1.0) printf("عالی (<1)\n");
    else if (sol.hdop < 2.0) printf("خوب (1-2)\n");
    else if (sol.hdop < 5.0) printf("متوسط (2-5)\n");
    else printf("ضعیف (>5)\n");

    printf("  Sats %d = ", sol.satellites);
    if (sol.satellites >= 8) printf("عالی (>=8)\n");
    else if (sol.satellites >= 5) printf("خوب (5-7)\n");
    else printf("کم (<5)\n");

    printf("\n========================================\n");
    printf("  دمو تمام شد - همینقدر ساده!\n");
    printf("========================================\n");

    return 0;
}
