// gnss/src/parser/ubx/ubx_parser.c
// پارسر UBX - NAV-PVT + NAV-SAT + NAV-DOP + NAV-TIME + پشتیبانی کامل

#include "ubx_parser.h"
#include "gnss_config.h"
#include "gnss.h"
#include <string.h>

#if GNSS_ENABLE_UBX

// ======================== helpers LE ========================
static uint16_t read_u16_le(const uint8_t* p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static uint32_t read_u32_le(const uint8_t* p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static int32_t read_i32_le(const uint8_t* p) { return (int32_t)read_u32_le(p); }
static int16_t read_i16_le(const uint8_t* p) { return (int16_t)read_u16_le(p); }

gnss_error_t ubx_parse(const uint8_t* buffer, size_t length, uint8_t ck_a, uint8_t ck_b)
{
    if (!buffer || length < 6) return GNSS_ERR_INVALID_FRAME;
    if (buffer[0] != 0xB5 || buffer[1] != 0x62) return GNSS_ERR_INVALID_FRAME;

    uint8_t class_id = buffer[2];
    uint8_t msg_id   = buffer[3];
    uint16_t payload_len = (uint16_t)buffer[4] | ((uint16_t)buffer[5] << 8);

    if (payload_len != (length - 6)) {
        if (payload_len > (length - 6)) return GNSS_ERR_INVALID_FRAME;
    }
    if (payload_len > GNSS_RX_BUFFER_SIZE) return GNSS_ERR_BUFFER_OVERFLOW;

    uint8_t calc_a = 0, calc_b = 0;
    for (size_t i = 2; i < length; i++) { calc_a += buffer[i]; calc_b += calc_a; }
    if (calc_a != ck_a || calc_b != ck_b) return GNSS_ERR_BAD_CHECKSUM;

    const uint8_t* payload = buffer + 6;

    // ======================== NAV-PVT (0x01 0x07) ========================
#if GNSS_ENABLE_UBX_NAV_PVT
    if (class_id == 0x01 && msg_id == 0x07) {
        if (payload_len < 92) return GNSS_ERR_INVALID_FRAME;
        uint8_t fixType = payload[20];
        uint8_t numSV   = payload[23];
        int32_t lon = read_i32_le(payload + 24);
        int32_t lat = read_i32_le(payload + 28);
        int32_t hMSL = read_i32_le(payload + 36);
        int32_t height = read_i32_le(payload + 32);
        int32_t gSpeed = read_i32_le(payload + 60);
        int32_t headMot = read_i32_le(payload + 64);
        uint16_t pDOP = read_u16_le(payload + 76);

        gnss_solution_t sol; memset(&sol, 0, sizeof(sol));
        sol.longitude_deg = lon / 10000000.0;
        sol.latitude_deg  = lat / 10000000.0;
        sol.altitude_m    = hMSL / 1000.0;
        sol.altitude_ellipsoid_m = height / 1000.0;
        sol.speed_mps = gSpeed / 1000.0f;
        if (sol.speed_mps < 0) sol.speed_mps = -sol.speed_mps;
        sol.course_deg = headMot / 100000.0f;
        sol.satellites = numSV;
        sol.fix_type = fixType;
        sol.valid_position = (fixType >= 2);
        sol.valid_velocity = (fixType >= 2);
        sol.pdop = pDOP / 100.0f;
        sol.valid_dop = true;

        uint16_t year = read_u16_le(payload + 4);
        uint8_t month = payload[6];
        uint8_t day = payload[7];
        uint8_t hour = payload[8];
        uint8_t min = payload[9];
        uint8_t sec = payload[10];
        uint8_t valid = payload[11];
        if (valid & 0x01) { sol.year=year; sol.month=month; sol.day=day; sol.valid_time=true; }
        if (valid & 0x02) { sol.hour=hour; sol.minute=min; sol.second=sec; sol.valid_time=true; }

        gnss_internal_update_solution(&sol);
        return GNSS_OK;
    }
#endif

    // ======================== NAV-SAT (0x01 0x35) ========================
#if GNSS_ENABLE_UBX_NAV_SAT
    if (class_id == 0x01 && msg_id == 0x35) {
        if (payload_len < 8) return GNSS_ERR_INVALID_FRAME;
        uint8_t numSvs = payload[5];
        // header 8 bytes
        if (payload_len < (size_t)(8 + numSvs*12)) return GNSS_ERR_INVALID_FRAME;
        if (numSvs > GNSS_MAX_SATELLITES) numSvs = GNSS_MAX_SATELLITES;

        gnss_satellite_t sats[GNSS_MAX_SATELLITES];
        memset(sats, 0, sizeof(sats));

        for (uint8_t i=0; i<numSvs; i++) {
            const uint8_t* sv = payload + 8 + i*12;
            uint8_t gnssId = sv[0];
            uint8_t svId   = sv[1];
            uint8_t cno    = sv[2]; // SNR
            int8_t elev    = (int8_t)sv[3];
            int16_t azim   = read_i16_le(sv+4);
            // int16_t prRes = read_i16_le(sv+6);
            uint32_t flags = read_u32_le(sv+8);
            bool svUsed = (flags & 0x08) != 0;
            bool healthy = (flags & 0x10) ? false : true; // health bit? simplified

            sats[i].prn = svId;
            sats[i].gnss_id = gnssId;
            sats[i].snr = cno;
            sats[i].elevation = (elev < 0) ? 0 : (uint8_t)elev;
            sats[i].elev_raw = elev;
            sats[i].azimuth = (azim < 0) ? (uint16_t)(azim+360) : (uint16_t)azim;
            sats[i].used_in_solution = svUsed;
            sats[i].healthy = healthy;
            sats[i].quality = (flags & 0x07); // quality indicator
        }

        gnss_internal_set_satellites(sats, numSvs);
        return GNSS_OK;
    }
#endif

    // ======================== NAV-DOP (0x01 0x04) ========================
#if GNSS_ENABLE_UBX_NAV_DOP
    if (class_id == 0x01 && msg_id == 0x04) {
        if (payload_len < 18) return GNSS_ERR_INVALID_FRAME;
        uint16_t gDOP = read_u16_le(payload + 4);
        uint16_t pDOP = read_u16_le(payload + 6);
        uint16_t tDOP = read_u16_le(payload + 8);
        uint16_t vDOP = read_u16_le(payload + 10);
        uint16_t hDOP = read_u16_le(payload + 12);
        uint16_t nDOP = read_u16_le(payload + 14);
        uint16_t eDOP = read_u16_le(payload + 16);

        gnss_solution_t sol; memset(&sol, 0, sizeof(sol));
        sol.gdop = gDOP / 100.0f;
        sol.pdop = pDOP / 100.0f;
        sol.tdop = tDOP / 100.0f;
        sol.vdop = vDOP / 100.0f;
        sol.hdop = hDOP / 100.0f;
        sol.ndop = nDOP / 100.0f;
        sol.edop = eDOP / 100.0f;
        sol.valid_dop = true;
        // fix_type نداریم، فقط DOP
        gnss_internal_update_solution(&sol);
        return GNSS_OK;
    }
#endif

    // ======================== NAV-TIMEUTC (0x01 0x21) - اختیاری ========================
#if GNSS_ENABLE_UBX_NAV_TIME
    if (class_id == 0x01 && msg_id == 0x21) {
        if (payload_len < 20) return GNSS_ERR_INVALID_FRAME;
        uint16_t year = read_u16_le(payload + 12);
        uint8_t month = payload[14];
        uint8_t day = payload[15];
        uint8_t hour = payload[16];
        uint8_t min = payload[17];
        uint8_t sec = payload[18];
        uint8_t valid = payload[19];
        gnss_solution_t sol; memset(&sol,0,sizeof(sol));
        if (valid & 0x04) {
            sol.year=year; sol.month=month; sol.day=day;
            sol.hour=hour; sol.minute=min; sol.second=sec;
            sol.valid_time=true;
            gnss_internal_update_solution(&sol);
        }
        return GNSS_OK;
    }
#endif

    return GNSS_ERR_UNSUPPORTED_MESSAGE;
}

#endif // GNSS_ENABLE_UBX
