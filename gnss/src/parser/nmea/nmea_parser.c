// gnss/src/parser/nmea/nmea_parser.c
// پارسر NMEA 0183 - نسخه کامل با GSA used flag + Vendor
// پشتیبانی: GGA, RMC, GSV با Aggregation, GSA با used_in_solution, VTG, GLL, ZDA, TXT + Vendor

#include "nmea_parser.h"
#include "gnss_config.h"
#include "gnss.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#if GNSS_ENABLE_NMEA

#if GNSS_ENABLE_VENDOR_EXT
#include "../vendor/vendor_parser.h"
#endif

// ======================== وضعیت GSV aggregation ========================
#if GNSS_ENABLE_GSV || GNSS_ENABLE_UBX_NAV_SAT
static gnss_satellite_t g_gsv_buffer[GNSS_MAX_SATELLITES];
static uint8_t g_gsv_count = 0;
static uint8_t g_gsv_expected_msgs = 0;
#endif

// ======================== وضعیت GSA used PRNs ========================
#if GNSS_ENABLE_GSA
static uint8_t g_gsa_used_prns[GNSS_MAX_SATELLITES];
static uint8_t g_gsa_used_count = 0;
static bool g_gsa_has_used = false;

static void apply_gsa_used_flags(void)
{
    if (!g_gsa_has_used) return;
    // روی بافر GSV اعمال کن
    for (uint8_t i=0; i<g_gsv_count; i++) {
        g_gsv_buffer[i].used_in_solution = false;
        for (uint8_t j=0; j<g_gsa_used_count; j++) {
            if (g_gsv_buffer[i].prn == g_gsa_used_prns[j]) {
                g_gsv_buffer[i].used_in_solution = true;
                break;
            }
        }
    }
}

static void apply_gsa_to_existing_satellites(void)
{
    if (!g_gsa_has_used) return;
    if (g_gsv_count>0) {
        apply_gsa_used_flags();
        gnss_internal_set_satellites(g_gsv_buffer, g_gsv_count);
    }
}

void gnss_internal_set_used_prns(const uint8_t* prns, uint8_t count)
{
    if (!prns || count==0) return;
    if (count > GNSS_MAX_SATELLITES) count = GNSS_MAX_SATELLITES;
    memcpy(g_gsa_used_prns, prns, count);
    g_gsa_used_count = count;
    g_gsa_has_used = true;
    apply_gsa_to_existing_satellites();
}
#endif // GSA

// ======================== توابع کمکی HEX ========================
static int hex_char_to_val(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static uint8_t nmea_checksum_calc(const uint8_t* data, size_t len, size_t star_pos)
{
    uint8_t cs = 0;
    for (size_t i = 1; i < star_pos && i < len; i++) cs ^= data[i];
    return cs;
}

static bool nmea_validate_checksum(const uint8_t* buffer, size_t len)
{
    if (len < 6) return false;
    size_t star_pos = 0;
    bool found = false;
    for (size_t i = 0; i < len; i++) {
        if (buffer[i] == '*') { star_pos = i; found = true; break; }
    }
    if (!found) return false;
    if (star_pos + 3 > len) return false;
    uint8_t calc = nmea_checksum_calc(buffer, len, star_pos);
    int hi = hex_char_to_val((char)buffer[star_pos + 1]);
    int lo = hex_char_to_val((char)buffer[star_pos + 2]);
    if (hi < 0 || lo < 0) return false;
    uint8_t recv = (uint8_t)((hi << 4) | lo);
    return calc == recv;
}

// ======================== تبدیل مختصات ========================
static gnss_float_t nmea_ddmm_to_deg(const char* field, char hemisphere)
{
    if (!field || field[0] == '\0') return 0.0;
    double val = atof(field);
    if (val == 0.0) return 0.0;
    int degrees = (int)(val / 100.0);
    double minutes = val - (degrees * 100.0);
    double result = degrees + (minutes / 60.0);
    if (hemisphere == 'S' || hemisphere == 'W' || hemisphere == 's' || hemisphere == 'w') result = -result;
    return (gnss_float_t)result;
}

// export for vendor
gnss_float_t nmea_ddmm_to_deg_export(const char* field, char hemi) { return nmea_ddmm_to_deg(field, hemi); }

static uint8_t gnss_id_from_talker(const char* talker)
{
    if (!talker) return 0;
    if (strncmp(talker, "GP", 2) == 0) return GNSS_ID_GPS;
    if (strncmp(talker, "GL", 2) == 0) return GNSS_ID_GLONASS;
    if (strncmp(talker, "GA", 2) == 0) return GNSS_ID_GALILEO;
    if (strncmp(talker, "GB", 2) == 0 || strncmp(talker, "BD", 2) == 0) return GNSS_ID_BEIDOU;
    if (strncmp(talker, "GQ", 2) == 0) return GNSS_ID_QZSS;
    if (strncmp(talker, "GI", 2) == 0) return GNSS_ID_NAVIC;
    if (strncmp(talker, "GN", 2) == 0) return GNSS_ID_COMBINED;
    if (strncmp(talker, "PQ", 2) == 0) return GNSS_ID_GPS; // Quectel often GPS
    if (strncmp(talker, "PT", 2) == 0) return GNSS_ID_GPS; // Trimble
    return GNSS_ID_GPS;
}

// ======================== پارسرهای پیام ========================
#if GNSS_ENABLE_GGA
static gnss_error_t parse_gga(char* fields[], int field_count)
{
    if (field_count < 10) return GNSS_ERR_INVALID_FRAME;
    int fix = 0;
    if (fields[6] && fields[6][0] != '\0') fix = atoi(fields[6]);
    gnss_solution_t sol; memset(&sol, 0, sizeof(sol));
    if (fields[2] && fields[3] && fields[4] && fields[5]) {
        sol.latitude_deg  = nmea_ddmm_to_deg(fields[2], fields[3][0]);
        sol.longitude_deg = nmea_ddmm_to_deg(fields[4], fields[5][0]);
    }
    sol.fix_type = (uint8_t)fix;
    sol.valid_position = (fix > 0);
    if (fields[7] && fields[7][0] != '\0') sol.satellites = (uint8_t)atoi(fields[7]);
    if (fields[8] && fields[8][0] != '\0') sol.hdop = (gnss_float_t)atof(fields[8]);
    if (fields[9] && fields[9][0] != '\0') sol.altitude_m = (gnss_float_t)atof(fields[9]);
    if (fields[1] && strlen(fields[1]) >= 6) {
        char tmp[3] = {0};
        tmp[0]=fields[1][0]; tmp[1]=fields[1][1]; sol.hour = (uint8_t)atoi(tmp);
        tmp[0]=fields[1][2]; tmp[1]=fields[1][3]; sol.minute = (uint8_t)atoi(tmp);
        tmp[0]=fields[1][4]; tmp[1]=fields[1][5]; sol.second = (uint8_t)atoi(tmp);
        sol.valid_time = true;
    }
    gnss_internal_update_solution(&sol);
    return GNSS_OK;
}
#endif

#if GNSS_ENABLE_RMC
static gnss_error_t parse_rmc(char* fields[], int field_count)
{
    if (field_count < 8) return GNSS_ERR_INVALID_FRAME;
    char status = (fields[2] && fields[2][0]) ? fields[2][0] : 'V';
    gnss_solution_t sol; memset(&sol, 0, sizeof(sol));
    sol.valid_position = (status == 'A');
    sol.valid_velocity = (status == 'A');
    if (fields[3] && fields[4] && fields[5] && fields[6]) {
        sol.latitude_deg  = nmea_ddmm_to_deg(fields[3], fields[4][0]);
        sol.longitude_deg = nmea_ddmm_to_deg(fields[5], fields[6][0]);
    }
    if (fields[7] && fields[7][0] != '\0') {
        double knots = atof(fields[7]);
        sol.speed_mps = (gnss_float_t)(knots * 0.514444);
    }
    if (field_count > 8 && fields[8] && fields[8][0] != '\0') sol.course_deg = (gnss_float_t)atof(fields[8]);
    if (fields[1] && strlen(fields[1]) >= 6) {
        char tmp[3]={0};
        tmp[0]=fields[1][0]; tmp[1]=fields[1][1]; sol.hour = (uint8_t)atoi(tmp);
        tmp[0]=fields[1][2]; tmp[1]=fields[1][3]; sol.minute = (uint8_t)atoi(tmp);
        tmp[0]=fields[1][4]; tmp[1]=fields[1][5]; sol.second = (uint8_t)atoi(tmp);
        sol.valid_time = true;
    }
    if (field_count > 9 && fields[9] && strlen(fields[9]) == 6) {
        char tmp[3]={0};
        tmp[0]=fields[9][0]; tmp[1]=fields[9][1]; sol.day = (uint8_t)atoi(tmp);
        tmp[0]=fields[9][2]; tmp[1]=fields[9][3]; sol.month = (uint8_t)atoi(tmp);
        tmp[0]=fields[9][4]; tmp[1]=fields[9][5]; sol.year = (uint16_t)(2000 + atoi(tmp));
        sol.valid_time = true;
    }
    gnss_internal_update_solution(&sol);
    return GNSS_OK;
}
#endif

#if GNSS_ENABLE_GSV
static gnss_error_t parse_gsv(char* fields[], int field_count, const char* talker)
{
    if (field_count < 4) return GNSS_ERR_INVALID_FRAME;
    uint8_t total_msgs = (fields[1] && fields[1][0]) ? (uint8_t)atoi(fields[1]) : 0;
    uint8_t msg_num    = (fields[2] && fields[2][0]) ? (uint8_t)atoi(fields[2]) : 0;
    if (msg_num == 1) { g_gsv_count = 0; g_gsv_expected_msgs = total_msgs; }
    uint8_t gnss_id = gnss_id_from_talker(talker);
    int idx = 4;
    while (idx + 3 < field_count && g_gsv_count < GNSS_MAX_SATELLITES) {
        if (!fields[idx] || fields[idx][0] == '\0') { idx+=4; continue; }
        gnss_satellite_t sat; memset(&sat,0,sizeof(sat));
        sat.prn = (fields[idx]) ? (uint8_t)atoi(fields[idx]) : 0;
        sat.elevation = (fields[idx+1] && fields[idx+1][0]) ? (uint8_t)atoi(fields[idx+1]) : 0;
        sat.azimuth   = (fields[idx+2] && fields[idx+2][0]) ? (uint16_t)atoi(fields[idx+2]) : 0;
        sat.snr       = (fields[idx+3] && fields[idx+3][0]) ? (uint8_t)atoi(fields[idx+3]) : 0;
        sat.gnss_id   = gnss_id;
        sat.used_in_solution = false;
        sat.healthy = (sat.snr > 0);
        g_gsv_buffer[g_gsv_count++] = sat;
        idx+=4;
    }
    if (msg_num == g_gsv_expected_msgs && g_gsv_expected_msgs != 0) {
#if GNSS_ENABLE_GSA
        apply_gsa_used_flags();
#endif
        gnss_internal_set_satellites(g_gsv_buffer, g_gsv_count);
    }
    return GNSS_OK;
}
#endif

#if GNSS_ENABLE_GSA
static gnss_error_t parse_gsa(char* fields[], int field_count)
{
    if (field_count < 4) return GNSS_ERR_INVALID_FRAME;
    gnss_solution_t sol; memset(&sol,0,sizeof(sol));
    int fix = 0;
    if (fields[2] && fields[2][0]) fix = atoi(fields[2]);
    sol.fix_type = (fix >= 2) ? (uint8_t)fix : 0;
    sol.valid_position = (fix >= 2);

    // استخراج PRN های استفاده شده (فیلد 3 تا 14)
    uint8_t used_prns[12];
    uint8_t used_count=0;
    for (int i=3; i<=14 && i<field_count; i++) {
        if (fields[i] && fields[i][0]!='\0') {
            int prn = atoi(fields[i]);
            if (prn>0 && used_count<12) used_prns[used_count++] = (uint8_t)prn;
        }
    }
    if (used_count>0) {
        gnss_internal_set_used_prns(used_prns, used_count);
    }

    // DOPs
    if (field_count >= 18) {
        if (fields[15] && fields[15][0]) sol.pdop = (gnss_float_t)atof(fields[15]);
        if (fields[16] && fields[16][0]) sol.hdop = (gnss_float_t)atof(fields[16]);
        if (fields[17] && fields[17][0]) sol.vdop = (gnss_float_t)atof(fields[17]);
    } else {
        if (field_count>15 && fields[field_count-3][0]) sol.pdop = (gnss_float_t)atof(fields[field_count-3]);
        if (field_count>15 && fields[field_count-2][0]) sol.hdop = (gnss_float_t)atof(fields[field_count-2]);
        if (field_count>15 && fields[field_count-1][0]) sol.vdop = (gnss_float_t)atof(fields[field_count-1]);
    }
    sol.valid_dop = (sol.pdop!=0 || sol.hdop!=0 || sol.vdop!=0);
    gnss_internal_update_solution(&sol);
    return GNSS_OK;
}
#endif

#if GNSS_ENABLE_VTG
static gnss_error_t parse_vtg(char* fields[], int field_count)
{
    if (field_count < 6) return GNSS_ERR_INVALID_FRAME;
    gnss_solution_t sol; memset(&sol,0,sizeof(sol));
    if (fields[1] && fields[1][0]) sol.course_deg = (gnss_float_t)atof(fields[1]);
    if (field_count>5 && fields[5] && fields[5][0]) {
        double knots = atof(fields[5]);
        sol.speed_mps = (gnss_float_t)(knots * 0.514444);
    } else if (field_count>7 && fields[7] && fields[7][0]) {
        double kmh = atof(fields[7]);
        sol.speed_mps = (gnss_float_t)(kmh / 3.6);
    }
    sol.valid_velocity = true;
    gnss_internal_update_solution(&sol);
    return GNSS_OK;
}
#endif

#if GNSS_ENABLE_GLL
static gnss_error_t parse_gll(char* fields[], int field_count)
{
    if (field_count < 5) return GNSS_ERR_INVALID_FRAME;
    gnss_solution_t sol; memset(&sol,0,sizeof(sol));
    if (fields[1] && fields[2] && fields[3] && fields[4]) {
        sol.latitude_deg = nmea_ddmm_to_deg(fields[1], fields[2][0]);
        sol.longitude_deg = nmea_ddmm_to_deg(fields[3], fields[4][0]);
        sol.valid_position = true;
    }
    if (field_count>6 && fields[6] && fields[6][0]=='A') sol.valid_position = true;
    gnss_internal_update_solution(&sol);
    return GNSS_OK;
}
#endif

#if GNSS_ENABLE_ZDA
static gnss_error_t parse_zda(char* fields[], int field_count)
{
    if (field_count < 5) return GNSS_ERR_INVALID_FRAME;
    gnss_solution_t sol; memset(&sol,0,sizeof(sol));
    if (fields[1] && strlen(fields[1])>=6) {
        char tmp[3]={0};
        tmp[0]=fields[1][0]; tmp[1]=fields[1][1]; sol.hour=(uint8_t)atoi(tmp);
        tmp[0]=fields[1][2]; tmp[1]=fields[1][3]; sol.minute=(uint8_t)atoi(tmp);
        tmp[0]=fields[1][4]; tmp[1]=fields[1][5]; sol.second=(uint8_t)atoi(tmp);
        sol.valid_time=true;
    }
    if (fields[2]) sol.day=(uint8_t)atoi(fields[2]);
    if (fields[3]) sol.month=(uint8_t)atoi(fields[3]);
    if (fields[4]) sol.year=(uint16_t)atoi(fields[4]);
    gnss_internal_update_solution(&sol);
    return GNSS_OK;
}
#endif

#if GNSS_ENABLE_TXT
static gnss_error_t parse_txt(char* fields[], int field_count)
{
    // $GPTXT,01,01,02,ANTSTATUS=OK*3B
    // فقط log، خطا نیست
    (void)fields; (void)field_count;
    return GNSS_OK;
}
#endif

// ======================== تابع اصلی ========================
gnss_error_t nmea_parse(const uint8_t* buffer, size_t length)
{
    if (!buffer || length < 6) return GNSS_ERR_INVALID_FRAME;
    if (buffer[0] != '$') return GNSS_ERR_INVALID_FRAME;

    if (!nmea_validate_checksum(buffer, length)) return GNSS_ERR_BAD_CHECKSUM;

    char tmp[GNSS_RX_BUFFER_SIZE + 1];
    size_t copy_len = (length < GNSS_RX_BUFFER_SIZE) ? length : GNSS_RX_BUFFER_SIZE;
    memcpy(tmp, buffer, copy_len);
    tmp[copy_len] = '\0';
    for (size_t i=0;i<copy_len;i++) {
        if (tmp[i]=='\r' || tmp[i]=='\n') { tmp[i]='\0'; break; }
    }

    // نگهداری raw برای vendor
    char raw_copy[GNSS_RX_BUFFER_SIZE+1];
    strncpy(raw_copy, tmp, GNSS_RX_BUFFER_SIZE);
    raw_copy[GNSS_RX_BUFFER_SIZE]='\0';

    char* fields[32];
    int field_count=0;
    char* start = tmp+1;
    for (size_t i=1;i<copy_len;i++) {
        if (tmp[i]==',' || tmp[i]=='*' || tmp[i]=='\0') {
            char orig=tmp[i];
            tmp[i]='\0';
            if (field_count < (int)(sizeof(fields)/sizeof(fields[0]))) fields[field_count++]=start;
            start=tmp+i+1;
            if (orig=='*' || orig=='\0') break;
        }
    }
    if (field_count<1) return GNSS_ERR_INVALID_FRAME;
    const char* full_id = fields[0];
    if (!full_id || strlen(full_id)<2) return GNSS_ERR_INVALID_FRAME;

    // Talker: دو حرف اول، ولی برای $PTNL سه حرف؟ ما 2 می‌گیریم ولی vendor جدا هندل می‌کند
    char talker[4]={0};
    talker[0]=full_id[0];
    talker[1]=full_id[1];
    if (full_id[0]=='P' && strlen(full_id)>=3) {
        // برای $PTNL, $PQTM etc talker ممکن است PT, PQ, PM باشد - همان 2 حرف اول کافی است
        talker[2]='\0';
    }

    const char* msg_id = full_id + 2;
    // برای PTNL, msg_id = "NL" است، ولی ما full_id را هم پاس می‌دهیم به vendor

    // تشخیص پیام استاندارد
#if GNSS_ENABLE_GGA
    if (strcmp(msg_id, "GGA")==0) return parse_gga(fields, field_count);
#endif
#if GNSS_ENABLE_RMC
    if (strcmp(msg_id, "RMC")==0) return parse_rmc(fields, field_count);
#endif
#if GNSS_ENABLE_GSV
    if (strcmp(msg_id, "GSV")==0) return parse_gsv(fields, field_count, talker);
    // Quectel $PQGSV هم GSV است - msg_id=GSV
    if (strcmp(full_id, "PQGSV")==0 || strcmp(full_id, "QGSV")==0) return parse_gsv(fields, field_count, talker);
#endif
#if GNSS_ENABLE_GSA
    if (strcmp(msg_id, "GSA")==0) return parse_gsa(fields, field_count);
    if (strcmp(full_id, "PQGSA")==0) return parse_gsa(fields, field_count);
#endif
#if GNSS_ENABLE_VTG
    if (strcmp(msg_id, "VTG")==0) return parse_vtg(fields, field_count);
#endif
#if GNSS_ENABLE_GLL
    if (strcmp(msg_id, "GLL")==0) return parse_gll(fields, field_count);
#endif
#if GNSS_ENABLE_ZDA
    if (strcmp(msg_id, "ZDA")==0) return parse_zda(fields, field_count);
#endif
#if GNSS_ENABLE_TXT
    if (strcmp(msg_id, "TXT")==0) return parse_txt(fields, field_count);
#endif

    // اگر استاندارد نبود، سعی کن Vendor
#if GNSS_ENABLE_VENDOR_EXT
    {
        gnss_error_t verr = vendor_parse_dispatch(fields, field_count, talker, msg_id, full_id, raw_copy);
        if (verr == GNSS_OK) return GNSS_OK;
    }
#endif

    // برای $PTNL و $PQ... اگر vendor فعال نبود ولی $P بود، به عنوان unsupported برنگرد، OK بده تا آمار خراب نشود
    if (full_id[0]=='P') {
#if GNSS_ENABLE_VENDOR_EXT
        return GNSS_ERR_UNSUPPORTED_MESSAGE;
#else
        // اگر vendor غیرفعال است، $P را نادیده بگیر
        return GNSS_ERR_UNSUPPORTED_MESSAGE;
#endif
    }

    return GNSS_ERR_UNSUPPORTED_MESSAGE;
}

#endif // GNSS_ENABLE_NMEA
