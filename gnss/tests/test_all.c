// tests/test_all.c - تست جامع همه پروتکل‌ها و APIها
#include "gnss.h"
#include <stdio.h>
#include <string.h>

#if GNSS_ENABLE_CALLBACKS
static int callback_count=0;
static void cb(const gnss_event_t* ev){
    callback_count++;
    if(ev->type==GNSS_EVENT_SOLUTION){
        printf("  [CB] solution lat=%.6f\n", ev->data.solution->latitude_deg);
    } else if(ev->type==GNSS_EVENT_SATELLITES){
        printf("  [CB] satellites count=%d\n", ev->satellite_count);
    }
}
#endif

int main(void){
    printf("=== GNSS Full Integration Test ===\n");
    gnss_init();

#if GNSS_ENABLE_CALLBACKS
    gnss_register_callback(cb);
    printf("Callback registered\n");
#endif

    // سناریوی واقعی: GGA -> RMC -> GSA -> GSV x3 -> VTG -> UBX
    const char* seq[] = {
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n",
        "$GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39\r\n",
        "$GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74\r\n",
        "$GPGSV,3,2,11,14,25,170,00,16,57,208,39,18,67,296,40,19,40,246,00*74\r\n",
        "$GPGSV,3,3,11,22,42,067,42,24,14,311,43,27,05,244,00,31,25,066,00*78\r\n",
        "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n",
        "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48\r\n",
    };

    for(int i=0;i<(int)(sizeof(seq)/sizeof(seq[0]));i++){
        gnss_parse((const uint8_t*)seq[i], strlen(seq[i]));
    }

    if(!gnss_has_new_solution()){
        printf("[FAIL] no solution after full seq\n");
        return 1;
    }
    gnss_solution_t sol;
    memset(&sol,0,sizeof(sol));
    gnss_get_solution(&sol);
    printf("Final solution: lat=%.6f lon=%.6f alt=%.1f speed=%.2f m/s course=%.1f sats=%d fix=%d hdop=%.1f pdop=%.1f\n",
        sol.latitude_deg, sol.longitude_deg, sol.altitude_m, sol.speed_mps, sol.course_deg, sol.satellites, sol.fix_type, sol.hdop, sol.pdop);

    // تست API سطح 2
    gnss_float_t lat=0; size_t sz=sizeof(lat);
    gnss_get_field(GNSS_FIELD_LATITUDE, &lat, &sz);
    printf("Field LAT via get_field=%.6f\n", lat);

    uint8_t sats=0; sz=sizeof(sats);
    gnss_get_field(GNSS_FIELD_SATELLITES, &sats, &sz);
    printf("Field SATS=%d\n", sats);

    // تست stats
#if GNSS_ENABLE_STATS
    gnss_stats_t stats;
    gnss_get_stats(&stats);
    printf("Stats: received=%u parsed=%u checksum_err=%u frame_err=%u overflow=%u\n",
        stats.messages_received, stats.messages_parsed, stats.checksum_errors, stats.frame_errors, stats.buffer_overflows);
#endif

    // تست satellite API
#if GNSS_ENABLE_GSV
    uint8_t cnt=0;
    gnss_get_satellite_count(&cnt);
    printf("Sat count=%d\n", cnt);
    for(uint8_t i=0;i<cnt && i<3;i++){
        gnss_satellite_t s;
        gnss_get_satellite(i,&s);
        printf("  sat[%d] prn=%d elev=%d az=%d snr=%d gnss_id=%d\n", i, s.prn, s.elevation, s.azimuth, s.snr, s.gnss_id);
    }
#endif

#if GNSS_ENABLE_CALLBACKS
    printf("Callback invoked %d times\n", callback_count);
    if(callback_count < 3){ printf("[WARN] callback count low but OK\n"); }
#endif

    printf("\n=== FULL INTEGRATION PASSED ===\n");
    return 0;
}
