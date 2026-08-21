// examples/ubx/ubx_example.c
// مثال UBX NAV-PVT + NAV-SAT + NAV-DOP

#include "gnss.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

// Helper برای ساخت UBX NAV-PVT
static void build_nav_pvt(uint8_t* buf, size_t* len, double lat, double lon, int32_t alt_mm)
{
    buf[0]=0xB5; buf[1]=0x62; buf[2]=0x01; buf[3]=0x07;
    uint16_t l=92; buf[4]=l&0xFF; buf[5]=l>>8;
    uint8_t* p=buf+6; memset(p,0,92);
    p[4]=0xE8; p[5]=0x07; p[6]=8; p[7]=20; p[8]=12; p[9]=34; p[10]=56; p[11]=0x07;
    p[20]=3; p[23]=12;
    int32_t ilon = (int32_t)(lon*1e7);
    int32_t ilat = (int32_t)(lat*1e7);
    p[24]=ilon&0xFF; p[25]=(ilon>>8)&0xFF; p[26]=(ilon>>16)&0xFF; p[27]=(ilon>>24)&0xFF;
    p[28]=ilat&0xFF; p[29]=(ilat>>8)&0xFF; p[30]=(ilat>>16)&0xFF; p[31]=(ilat>>24)&0xFF;
    p[36]=alt_mm&0xFF; p[37]=(alt_mm>>8)&0xFF; p[38]=(alt_mm>>16)&0xFF; p[39]=(alt_mm>>24)&0xFF;
    int32_t gSpeed=5000; p[60]=gSpeed&0xFF; p[61]=(gSpeed>>8)&0xFF; p[62]=(gSpeed>>16)&0xFF; p[63]=(gSpeed>>24)&0xFF;
    int32_t head=9000000; p[64]=head&0xFF; p[65]=(head>>8)&0xFF; p[66]=(head>>16)&0xFF; p[67]=(head>>24)&0xFF;
    p[76]=150&0xFF; p[77]=150>>8;
    uint8_t ca=0,cb=0; for(size_t i=2;i<6+92;i++){ca+=buf[i]; cb+=ca;}
    buf[6+92]=ca; buf[6+92+1]=cb; *len=6+92+2;
}

static void build_nav_sat(uint8_t* buf, size_t* len)
{
    // NAV-SAT with 4 sats
    uint8_t numSvs=4;
    uint16_t payload_len = 8 + numSvs*12;
    buf[0]=0xB5; buf[1]=0x62; buf[2]=0x01; buf[3]=0x35;
    buf[4]=payload_len&0xFF; buf[5]=payload_len>>8;
    uint8_t* p=buf+6; memset(p,0,payload_len);
    p[5]=numSvs;
    // sat0: GPS 02
    p[8]=0; p[9]=2; p[10]=35; p[11]=10; p[12]=90&0xFF; p[13]=90>>8; p[16]=0x08; // used
    // sat1: GPS 04
    p[20]=0; p[21]=4; p[22]=40; p[23]=20; p[24]=120&0xFF; p[25]=120>>8; p[28]=0x08;
    // sat2: Galileo 05
    p[32]=2; p[33]=5; p[34]=45; p[35]=30; p[36]=200&0xFF; p[37]=200>>8; p[40]=0x00; // not used
    // sat3: GLONASS 09
    p[44]=6; p[45]=9; p[46]=44; p[47]=50; p[48]=44; p[49]=1; p[52]=0x08;

    uint8_t ca=0,cb=0; for(size_t i=2;i<6+payload_len;i++){ca+=buf[i]; cb+=ca;}
    buf[6+payload_len]=ca; buf[6+payload_len+1]=cb; *len=6+payload_len+2;
}

static void build_nav_dop(uint8_t* buf, size_t* len)
{
    uint16_t payload_len=18;
    buf[0]=0xB5; buf[1]=0x62; buf[2]=0x01; buf[3]=0x04;
    buf[4]=payload_len&0xFF; buf[5]=payload_len>>8;
    uint8_t* p=buf+6; memset(p,0,payload_len);
    // gDOP 180, pDOP 150, tDOP 120, vDOP 130, hDOP 90, nDOP 80, eDOP 70 (scale 0.01)
    p[4]=180&0xFF; p[5]=180>>8;
    p[6]=150&0xFF; p[7]=150>>8;
    p[8]=120&0xFF; p[9]=120>>8;
    p[10]=130&0xFF; p[11]=130>>8;
    p[12]=90&0xFF; p[13]=90>>8;
    p[14]=80&0xFF; p[15]=80>>8;
    p[16]=70&0xFF; p[17]=70>>8;
    uint8_t ca=0,cb=0; for(size_t i=2;i<6+payload_len;i++){ca+=buf[i]; cb+=ca;}
    buf[6+payload_len]=ca; buf[6+payload_len+1]=cb; *len=6+payload_len+2;
}

int main(void)
{
    printf("=== UBX Extended Example ===\n");
    gnss_init();

    uint8_t buf[512];
    size_t len;

    printf("\n[1] NAV-PVT\n");
    build_nav_pvt(buf, &len, 48.1173, 11.5166, 545400);
    gnss_parse(buf, len);
    if (gnss_has_new_solution()) {
        gnss_solution_t sol; gnss_get_solution(&sol);
        printf("  PVT: lat=%.6f lon=%.6f alt=%.1f sats=%d pdop=%.2f\n",
            sol.latitude_deg, sol.longitude_deg, sol.altitude_m, sol.satellites, sol.pdop);
    }

    printf("\n[2] NAV-SAT (with used flag)\n");
    build_nav_sat(buf, &len);
    gnss_parse(buf, len);
    uint8_t cnt=0; gnss_get_satellite_count(&cnt);
    printf("  SAT count=%d\n", cnt);
    for (uint8_t i=0;i<cnt;i++) {
        gnss_satellite_t s; gnss_get_satellite(i,&s);
        printf("    gnssId=%d svId=%d cno=%d elev=%d az=%d used=%d\n",
            s.gnss_id, s.prn, s.snr, s.elevation, s.azimuth, s.used_in_solution);
    }

    printf("\n[3] NAV-DOP\n");
    build_nav_dop(buf, &len);
    gnss_parse(buf, len);
    if (gnss_has_new_solution()) {
        gnss_solution_t sol; gnss_get_solution(&sol);
        printf("  DOP: g=%.2f p=%.2f t=%.2f v=%.2f h=%.2f n=%.2f e=%.2f\n",
            sol.gdop, sol.pdop, sol.tdop, sol.vdop, sol.hdop, sol.ndop, sol.edop);
    }

    printf("\n[4] Mixed NMEA + UBX stream\n");
    gnss_init();
    const char* nmea = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    gnss_parse((const uint8_t*)nmea, strlen(nmea));
    build_nav_dop(buf, &len);
    gnss_feed(buf, len);
    build_nav_sat(buf, &len);
    gnss_feed(buf, len);
    gnss_solution_t sol; 
    if (gnss_get_solution(&sol)==GNSS_OK) {
        printf("  Mixed final: lat=%.6f dop=%.1f sats=%d\n", sol.latitude_deg, sol.pdop, sol.satellites);
    }
    gnss_get_satellite_count(&cnt);
    printf("  Final sat count=%d\n", cnt);

    printf("\n=== UBX Example Finished ===\n");
    return 0;
}
