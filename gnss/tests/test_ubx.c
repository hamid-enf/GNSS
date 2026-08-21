// tests/test_ubx.c - تست UBX NAV-PVT
#include "gnss.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

static uint16_t read_u16_le(const uint8_t* p) { return p[0] | (p[1]<<8); }
static uint32_t read_u32_le(const uint8_t* p) { return p[0] | (p[1]<<8) | (p[2]<<16) | (p[3]<<24); }

static void build_ubx_nav_pvt(uint8_t* buf, size_t* out_len, double lat, double lon, int32_t alt_mm, uint8_t fixType, uint8_t numSV)
{
    // Header 6 bytes + payload 92 bytes + 2 checksum = 100 bytes
    buf[0]=0xB5; buf[1]=0x62; buf[2]=0x01; buf[3]=0x07;
    uint16_t len=92;
    buf[4]=len & 0xFF; buf[5]=len>>8;
    uint8_t* p = buf+6;
    memset(p, 0, 92);
    // iTOW
    p[0]=0; p[1]=0; p[2]=0; p[3]=0;
    // year 2024
    p[4]=0xE8; p[5]=0x07;
    p[6]=8; p[7]=20; p[8]=12; p[9]=34; p[10]=56;
    p[11]=0x07; // valid date & time
    p[20]=fixType;
    p[23]=numSV;
    int32_t ilon = (int32_t)(lon * 10000000);
    int32_t ilat = (int32_t)(lat * 10000000);
    // lon at 24
    p[24]=ilon & 0xFF; p[25]=(ilon>>8)&0xFF; p[26]=(ilon>>16)&0xFF; p[27]=(ilon>>24)&0xFF;
    p[28]=ilat & 0xFF; p[29]=(ilat>>8)&0xFF; p[30]=(ilat>>16)&0xFF; p[31]=(ilat>>24)&0xFF;
    // height ellipsoid 0, hMSL
    p[36]=alt_mm & 0xFF; p[37]=(alt_mm>>8)&0xFF; p[38]=(alt_mm>>16)&0xFF; p[39]=(alt_mm>>24)&0xFF;
    // gSpeed 5 m/s = 5000 mm/s
    int32_t gSpeed=5000;
    p[60]=gSpeed &0xFF; p[61]=(gSpeed>>8)&0xFF; p[62]=(gSpeed>>16)&0xFF; p[63]=(gSpeed>>24)&0xFF;
    // headMot 90 deg = 9000000 *1e-5
    int32_t head=9000000;
    p[64]=head &0xFF; p[65]=(head>>8)&0xFF; p[66]=(head>>16)&0xFF; p[67]=(head>>24)&0xFF;
    // pDOP 150 = 1.5
    p[76]=150 &0xFF; p[77]=150>>8;

    // calc checksum over class, id, len, payload
    uint8_t ck_a=0, ck_b=0;
    for(size_t i=2;i<6+92;i++) { ck_a+=buf[i]; ck_b+=ck_a; }
    buf[6+92]=ck_a;
    buf[6+92+1]=ck_b;
    *out_len = 6+92+2;
}

int main(void)
{
    printf("=== UBX NAV-PVT Test ===\n");
    gnss_init();

    uint8_t ubx[120];
    size_t ubx_len=0;
    build_ubx_nav_pvt(ubx, &ubx_len, 48.1173, 11.5166, 545400, 3, 12);

    gnss_error_t err = gnss_parse(ubx, ubx_len);
    printf("UBX parse err=%d\n", err);
    if (err != GNSS_OK) {
        printf("[FAIL] UBX parse\n");
        return 1;
    }
    if (!gnss_has_new_solution()) {
        printf("[FAIL] no solution after UBX\n");
        return 1;
    }
    gnss_solution_t sol;
    memset(&sol, 0, sizeof(sol));
    err = gnss_get_solution(&sol);
    if (err != GNSS_OK) {
        printf("[FAIL] get solution UBX %d\n", err);
        return 1;
    }
    printf("[PASS] UBX solution lat=%.6f lon=%.6f alt=%.1f sats=%d fix=%d speed=%.1f course=%.1f pdop=%.2f\n",
           sol.latitude_deg, sol.longitude_deg, sol.altitude_m, sol.satellites, sol.fix_type, sol.speed_mps, sol.course_deg, sol.pdop);

    if (fabs(sol.latitude_deg - 48.1173) > 0.0001) { printf("[FAIL] lat\n"); return 1; }
    if (fabs(sol.longitude_deg - 11.5166) > 0.0001) { printf("[FAIL] lon\n"); return 1; }
    if (sol.satellites != 12) { printf("[FAIL] sats\n"); return 1; }
    if (sol.fix_type != 3) { printf("[FAIL] fix\n"); return 1; }

    // Test bad checksum
    gnss_init();
    ubx[ubx_len-1] ^= 0xFF; // corrupt checksum
    err = gnss_parse(ubx, ubx_len);
    if (err != GNSS_ERR_BAD_CHECKSUM) {
        printf("[FAIL] bad checksum should be rejected, got %d\n", err);
        return 1;
    }
    printf("[PASS] bad checksum rejected\n");

    // Test streaming byte-by-byte
    gnss_init();
    build_ubx_nav_pvt(ubx, &ubx_len, 35.6895, 139.6917, 40000, 3, 10);
    for (size_t i=0;i<ubx_len;i++) {
        gnss_feed_byte(ubx[i]);
    }
    if (!gnss_has_new_solution()) { printf("[FAIL] byte-by-byte UBX no solution\n"); return 1; }
    gnss_get_solution(&sol);
    printf("[PASS] byte-by-byte UBX lat=%.6f lon=%.6f\n", sol.latitude_deg, sol.longitude_deg);

    printf("\n=== ALL UBX TESTS PASSED ===\n");
    return 0;
}
