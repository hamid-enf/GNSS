// tests/test_frame.c - تست پارسر فریم (مخلوط NMEA+UBX)
#include "gnss.h"
#include <stdio.h>
#include <string.h>

int main(void)
{
    printf("=== Frame Parser Mixed Test ===\n");
    gnss_init();

    const char* stream = 
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n"
        "some garbage $$$$ \r\n"
        "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";

    gnss_error_t err = gnss_parse((const uint8_t*)stream, strlen(stream));
    printf("mixed stream err=%d has_solution=%d\n", err, gnss_has_new_solution());

    gnss_solution_t sol;
    if (gnss_get_solution(&sol) == GNSS_OK) {
        printf("[PASS] mixed parsing got solution lat=%.6f\n", sol.latitude_deg);
    } else {
        printf("[FAIL] no solution from mixed\n");
        return 1;
    }

    // تست تغذیه بایت‌به‌بایت با نویز بین پیام‌ها
    gnss_init();
    const char* noisy = "xxx$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n---$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
    for (size_t i=0;i<strlen(noisy);i++) gnss_feed_byte(noisy[i]);
    if (!gnss_has_new_solution()) { printf("[FAIL] noisy feeding\n"); return 1; }
    printf("[PASS] noisy feeding\n");

    // تست UBX در وسط NMEA
    gnss_init();
    // ساخت UBX ساده
    uint8_t ubx[100];
    memset(ubx,0,sizeof(ubx));
    ubx[0]=0xB5; ubx[1]=0x62; ubx[2]=0x01; ubx[3]=0x07;
    ubx[4]=92; ubx[5]=0;
    // payload پر نشده = فیکس 0
    uint8_t ck_a=0,ck_b=0;
    for(size_t i=2;i<6+92;i++){ck_a+=ubx[i]; ck_b+=ck_a;}
    ubx[6+92]=ck_a; ubx[6+92+1]=ck_b;
    gnss_feed(ubx, 6+92+2);
    // بعد NMEA
    const char* after = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    gnss_parse((const uint8_t*)after, strlen(after));
    if (!gnss_has_new_solution()) { printf("[FAIL] UBX+NMEA mixed\n"); return 1; }
    printf("[PASS] UBX+NMEA mixed\n");

    printf("\n=== FRAME TESTS PASSED ===\n");
    return 0;
}
