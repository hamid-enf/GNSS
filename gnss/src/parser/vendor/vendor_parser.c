// src/parser/vendor/vendor_parser.c
// پیاده‌سازی Vendor Extension Framework + Quectel + Trimble

#include "vendor_parser.h"
#include "gnss.h"
#include "gnss_vendor.h"
#include "gnss_config.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#if GNSS_ENABLE_VENDOR_EXT

// ======================== ذخیره آخرین پیام Vendor ========================
static gnss_vendor_msg_t g_last_vendor_msg = {0};
static gnss_vendor_parser_t g_custom_parsers[GNSS_VENDOR_MAX_PARSERS];
static uint8_t g_custom_parser_count = 0;

gnss_vendor_id_t gnss_vendor_detect(const char* talker, const char* full_id)
{
    if (!talker || !full_id) return GNSS_VENDOR_UNKNOWN;

    // Quectel: $PQ... or $PQT...
    if (talker[0] == 'P' && talker[1] == 'Q') return GNSS_VENDOR_QUECTEL;
    if (strncmp(full_id, "PQ", 2) == 0) return GNSS_VENDOR_QUECTEL;
    if (strncmp(full_id, "QTM", 3) == 0) return GNSS_VENDOR_QUECTEL;

    // Trimble: $PTNL
    if (strcmp(full_id, "PTNL") == 0 || strncmp(full_id, "TNL", 3) == 0) return GNSS_VENDOR_TRIMBLE;
    if (talker[0] == 'P' && talker[1] == 'T') return GNSS_VENDOR_TRIMBLE;

    // MTK: $PMTK
    if (strcmp(full_id, "PMTK") == 0) return GNSS_VENDOR_MTK;
    if (talker[0] == 'P' && talker[1] == 'M') return GNSS_VENDOR_MTK;

    // SiRF: $PSRF
    if (strcmp(full_id, "PSRF") == 0) return GNSS_VENDOR_SIRF;

    // u-blox proprietary $PUBX
    if (strcmp(full_id, "PUBX") == 0) return GNSS_VENDOR_UBLOX;

    return GNSS_VENDOR_UNKNOWN;
}

void gnss_vendor_set_last_msg(const gnss_vendor_msg_t* msg)
{
    if (msg) memcpy(&g_last_vendor_msg, msg, sizeof(gnss_vendor_msg_t));
}

gnss_error_t gnss_get_last_vendor_msg(gnss_vendor_msg_t* msg)
{
    if (!msg) return GNSS_ERR_INVALID_ARG;
    memcpy(msg, &g_last_vendor_msg, sizeof(gnss_vendor_msg_t));
    return GNSS_OK;
}

gnss_error_t gnss_vendor_register_parser(gnss_vendor_parser_t parser)
{
    if (!parser) return GNSS_ERR_INVALID_ARG;
    if (g_custom_parser_count >= GNSS_VENDOR_MAX_PARSERS) return GNSS_ERR_BUFFER_OVERFLOW;
    g_custom_parsers[g_custom_parser_count++] = parser;
    return GNSS_OK;
}

// ======================== Quectel Parser ========================
#if GNSS_ENABLE_VENDOR_QUECTEL
gnss_error_t gnss_vendor_quectel_parse(char* fields[], int field_count, const char* talker, const char* full_id, const char* raw)
{
    // ذخیره آخرین پیام
    gnss_vendor_msg_t vm;
    memset(&vm, 0, sizeof(vm));
    vm.vendor = GNSS_VENDOR_QUECTEL;
    strncpy(vm.talker, talker ? talker : "PQ", 3);
    strncpy(vm.type, full_id ? full_id : "UNKNOWN", 15);
    if (raw) strncpy(vm.raw, raw, GNSS_VENDOR_MAX_SENTENCE_LEN-1);
    gnss_vendor_set_last_msg(&vm);

    // Quectel بعضی جملات استاندارد را با پیشوند PQ می‌فرستد که قبلاً توسط GSV/GSA هندل شده
    // اینجا جملات خاص Quectel را هندل می‌کنیم

    // $PQTXT - متن اطلاعاتی
    if (strcmp(full_id, "QTXT") == 0 || strcmp(full_id, "PQTXT") == 0) {
        // fields[1]=msg, fields[2]=type, fields[3]=text
        // فقط log می‌کنیم، خطا نیست
        return GNSS_OK;
    }

    // $PQTMVER - نسخه
    if (strstr(full_id, "QTMVER") != NULL) {
        return GNSS_OK;
    }

    // $PQTMGNSS - اطلاعات GNSS
    if (strstr(full_id, "QTMGNSS") != NULL) {
        return GNSS_OK;
    }

    // $PQTMCFGSVIN - Survey-in config (برای RTK base)
    if (strstr(full_id, "QTMCFG") != NULL) {
        return GNSS_OK;
    }

    // $PQGNS - GNSS fix data (ترکیب GGA+RMC)
    // فرمت: $PQGNS,time,lat,N/S,lon,E/W,fix,sats,hdop,alt,geoid,age,ref*cs
    if (strcmp(full_id, "QGNS") == 0 || strcmp(full_id, "PQGNS") == 0) {
        if (field_count < 10) return GNSS_ERR_INVALID_FRAME;
        gnss_solution_t sol;
        memset(&sol, 0, sizeof(sol));
        // مشابه GGA
        extern gnss_float_t nmea_ddmm_to_deg_export(const char* field, char hemi); // forward, we'll reimplement locally
        // برای سادگی از atof استفاده می‌کنیم
        // lat
        if (fields[2] && fields[3] && fields[4] && fields[5]) {
            double lat_val = atof(fields[2]);
            int deg = (int)(lat_val/100);
            double min = lat_val - deg*100;
            sol.latitude_deg = deg + min/60.0;
            if (fields[3][0]=='S') sol.latitude_deg = -sol.latitude_deg;
            double lon_val = atof(fields[4]);
            deg = (int)(lon_val/100);
            min = lon_val - deg*100;
            sol.longitude_deg = deg + min/60.0;
            if (fields[5][0]=='W') sol.longitude_deg = -sol.longitude_deg;
            sol.valid_position = true;
        }
        if (fields[6]) sol.fix_type = (uint8_t)atoi(fields[6]);
        if (fields[7]) sol.satellites = (uint8_t)atoi(fields[7]);
        if (fields[8]) sol.hdop = (gnss_float_t)atof(fields[8]);
        if (fields[9]) sol.altitude_m = (gnss_float_t)atof(fields[9]);
        sol.last_vendor = GNSS_VENDOR_QUECTEL;
        gnss_internal_update_solution(&sol);
        return GNSS_OK;
    }

    return GNSS_ERR_UNSUPPORTED_MESSAGE;
}
#endif // QUECTEL

// ======================== Trimble Parser ========================
#if GNSS_ENABLE_VENDOR_TRIMBLE
static gnss_float_t trimble_ddmm_to_deg(const char* field, char hemi)
{
    if (!field || field[0]=='\0') return 0;
    double v = atof(field);
    int d = (int)(v/100);
    double m = v - d*100;
    double r = d + m/60.0;
    if (hemi=='S' || hemi=='W') r = -r;
    return (gnss_float_t)r;
}

gnss_error_t gnss_vendor_trimble_parse(char* fields[], int field_count, const char* talker, const char* full_id, const char* raw)
{
    gnss_vendor_msg_t vm;
    memset(&vm, 0, sizeof(vm));
    vm.vendor = GNSS_VENDOR_TRIMBLE;
    strncpy(vm.talker, talker ? talker : "PT", 3);
    strncpy(vm.type, full_id ? full_id : "TNL", 15);
    if (field_count > 1) strncpy(vm.subtype, fields[1] ? fields[1] : "", 15);
    if (raw) strncpy(vm.raw, raw, GNSS_VENDOR_MAX_SENTENCE_LEN-1);
    gnss_vendor_set_last_msg(&vm);

    // PTNL sentences: $PTNL,XXX,...
    // fields[0]=PTNL, fields[1]=subtype
    if (field_count < 2) return GNSS_ERR_INVALID_FRAME;

    const char* subtype = fields[1];

    // PTNL,AVR - Attitude
    // $PTNL,AVR,time,yaw,yaw_txt,tilt,tilt_txt,roll,roll_txt,range,2D_quality,1D_quality*cs
    if (strcmp(subtype, "AVR") == 0) {
        if (field_count < 6) return GNSS_ERR_INVALID_FRAME;
        gnss_solution_t sol;
        memset(&sol, 0, sizeof(sol));
        // yaw = heading
        if (fields[3] && fields[3][0]) sol.course_deg = (gnss_float_t)atof(fields[3]);
        sol.valid_velocity = true;
        sol.last_vendor = GNSS_VENDOR_TRIMBLE;
        gnss_internal_update_solution(&sol);
        return GNSS_OK;
    }

    // PTNL,GGK - RTK position (مشابه GGA ولی با RTK)
    // فرمت‌ها:
    // $PTNL,GGK,time,lat,N,lon,E,fix,num_sats,dop,alt,M*cs
    // $PTNL,GGK,time,date,lat,N,lon,E,fix,num_sats,dop,alt,M*cs (با تاریخ)
    // برای robustness، lat/lon را با جستجو پیدا می‌کنیم: الگوی lat,N,lon,E
    if (strcmp(subtype, "GGK") == 0) {
        if (field_count < 8) return GNSS_ERR_INVALID_FRAME;
        gnss_solution_t sol;
        memset(&sol, 0, sizeof(sol));

        // جستجوی lat,N,lon,E
        int lat_idx = -1;
        for (int i=2; i<field_count-3; i++) {
            if (!fields[i] || !fields[i+1] || !fields[i+2] || !fields[i+3]) continue;
            // fields[i] باید lat باشد (دارای '.' و عددی < 9000)
            // fields[i+1] N/S
            // fields[i+2] lon
            // fields[i+3] E/W
            char ns = fields[i+1][0];
            char ew = fields[i+3][0];
            if ((ns=='N' || ns=='S') && (ew=='E' || ew=='W')) {
                // چک lat/lon دارای '.'
                if (strchr(fields[i], '.') && strchr(fields[i+2], '.')) {
                    double lat_val = atof(fields[i]);
                    double lon_val = atof(fields[i+2]);
                    // lat ddmm.mmmm < 9000, lon < 18000
                    if (lat_val < 9000 && lon_val < 18000 && lat_val > 0 && lon_val > 0) {
                        lat_idx = i;
                        break;
                    }
                }
            }
        }

        if (lat_idx >= 0) {
            sol.latitude_deg = trimble_ddmm_to_deg(fields[lat_idx], fields[lat_idx+1][0]);
            sol.longitude_deg = trimble_ddmm_to_deg(fields[lat_idx+2], fields[lat_idx+3][0]);
            sol.valid_position = true;
            // بعد از lon,E: fix, sats, dop, alt
            if (lat_idx+4 < field_count && fields[lat_idx+4][0]) sol.fix_type = (uint8_t)atoi(fields[lat_idx+4]);
            if (lat_idx+5 < field_count && fields[lat_idx+5][0]) sol.satellites = (uint8_t)atoi(fields[lat_idx+5]);
            if (lat_idx+6 < field_count && fields[lat_idx+6][0]) sol.hdop = (gnss_float_t)atof(fields[lat_idx+6]);
            if (lat_idx+7 < field_count && fields[lat_idx+7][0]) sol.altitude_m = (gnss_float_t)atof(fields[lat_idx+7]);
        } else {
            // fallback قدیمی
            if (fields[3] && fields[4] && fields[5] && fields[6]) {
                sol.latitude_deg = trimble_ddmm_to_deg(fields[3], fields[4][0]);
                sol.longitude_deg = trimble_ddmm_to_deg(fields[5], fields[6][0]);
                sol.valid_position = true;
            }
            if (field_count > 7 && fields[7]) sol.fix_type = (uint8_t)atoi(fields[7]);
            if (field_count > 8 && fields[8]) sol.satellites = (uint8_t)atoi(fields[8]);
            if (field_count > 9 && fields[9]) sol.hdop = (gnss_float_t)atof(fields[9]);
            if (field_count > 10 && fields[10]) sol.altitude_m = (gnss_float_t)atof(fields[10]);
        }

        sol.rtk_fix_type = (sol.fix_type >= 4) ? 2 : (sol.fix_type==2 ? 1 : 0);
        sol.last_vendor = GNSS_VENDOR_TRIMBLE;
        gnss_internal_update_solution(&sol);
        return GNSS_OK;
    }

    // PTNL,BPQ - Base Position Quality
    if (strcmp(subtype, "BPQ") == 0) {
        return GNSS_OK; // اطلاعات کیفیت، فعلاً فقط log
    }

    // PTNL,VGK - Vector
    if (strcmp(subtype, "VGK") == 0) {
        return GNSS_OK;
    }

    // PTNL,GGA - Trimble GGA
    if (strcmp(subtype, "GGA") == 0) {
        // مشابه GGA استاندارد ولی با PTNL
        // fields[3]=lat, 4=N, 5=lon,6=E,7=quality,8=sats,9=dop,10=alt
        if (field_count < 11) return GNSS_ERR_INVALID_FRAME;
        gnss_solution_t sol;
        memset(&sol, 0, sizeof(sol));
        if (fields[3] && fields[4] && fields[5] && fields[6]) {
            sol.latitude_deg = trimble_ddmm_to_deg(fields[3], fields[4][0]);
            sol.longitude_deg = trimble_ddmm_to_deg(fields[5], fields[6][0]);
            sol.valid_position = true;
        }
        if (fields[7]) sol.fix_type = (uint8_t)atoi(fields[7]);
        if (fields[8]) sol.satellites = (uint8_t)atoi(fields[8]);
        if (fields[9]) sol.hdop = (gnss_float_t)atof(fields[9]);
        if (fields[10]) sol.altitude_m = (gnss_float_t)atof(fields[10]);
        sol.last_vendor = GNSS_VENDOR_TRIMBLE;
        gnss_internal_update_solution(&sol);
        return GNSS_OK;
    }

    return GNSS_ERR_UNSUPPORTED_MESSAGE;
}
#endif // TRIMBLE

// ======================== MTK Parser ========================
#if GNSS_ENABLE_VENDOR_MTK
gnss_error_t gnss_vendor_mtk_parse(char* fields[], int field_count, const char* talker, const char* full_id, const char* raw)
{
    gnss_vendor_msg_t vm;
    memset(&vm, 0, sizeof(vm));
    vm.vendor = GNSS_VENDOR_MTK;
    strncpy(vm.talker, talker ? talker : "MT", 3);
    strncpy(vm.type, full_id ? full_id : "MTK", 15);
    if (raw) strncpy(vm.raw, raw, GNSS_VENDOR_MAX_SENTENCE_LEN-1);
    gnss_vendor_set_last_msg(&vm);

    // $PMTK010,001*2E - system message
    // $PMTK011,MTKGPS*08 - text
    // فعلاً فقط ACK می‌کنیم
    return GNSS_OK;
}
#endif

// ======================== Dispatcher ========================
gnss_error_t vendor_parser_init(void)
{
    memset(g_custom_parsers, 0, sizeof(g_custom_parsers));
    g_custom_parser_count = 0;
    memset(&g_last_vendor_msg, 0, sizeof(g_last_vendor_msg));
    return GNSS_OK;
}

gnss_error_t vendor_parse_dispatch(char* fields[], int field_count, const char* talker, const char* msg_id, const char* full_id, const char* raw)
{
    if (!fields || field_count < 1) return GNSS_ERR_INVALID_ARG;

    // اول custom parsers کاربر
    for (uint8_t i=0;i<g_custom_parser_count;i++) {
        if (g_custom_parsers[i]) {
            gnss_error_t err = g_custom_parsers[i](fields, field_count, talker, full_id, raw);
            if (err == GNSS_OK) return GNSS_OK;
        }
    }

    gnss_vendor_id_t vendor = gnss_vendor_detect(talker, full_id);

    switch (vendor) {
#if GNSS_ENABLE_VENDOR_QUECTEL
        case GNSS_VENDOR_QUECTEL:
            return gnss_vendor_quectel_parse(fields, field_count, talker, full_id, raw);
#endif
#if GNSS_ENABLE_VENDOR_TRIMBLE
        case GNSS_VENDOR_TRIMBLE:
            return gnss_vendor_trimble_parse(fields, field_count, talker, full_id, raw);
#endif
#if GNSS_ENABLE_VENDOR_MTK
        case GNSS_VENDOR_MTK:
            return gnss_vendor_mtk_parse(fields, field_count, talker, full_id, raw);
#endif
        default:
            break;
    }

    // اگر vendor ناشناخته بود ولی $P داشت، به عنوان vendor عمومی قبول کن
    if (talker && talker[0]=='P') {
        gnss_vendor_msg_t vm;
        memset(&vm, 0, sizeof(vm));
        vm.vendor = GNSS_VENDOR_UNKNOWN;
        strncpy(vm.talker, talker, 3);
        strncpy(vm.type, full_id ? full_id : "", 15);
        if (raw) strncpy(vm.raw, raw, GNSS_VENDOR_MAX_SENTENCE_LEN-1);
        gnss_vendor_set_last_msg(&vm);
        return GNSS_OK; // نادیده بگیر ولی خطا نده
    }

    return GNSS_ERR_UNSUPPORTED_MESSAGE;
}

gnss_error_t gnss_vendor_parse(char* fields[], int field_count, const char* talker, const char* msg_id, const char* full_id, const char* raw)
{
    return vendor_parse_dispatch(fields, field_count, talker, msg_id, full_id, raw);
}

#endif // GNSS_ENABLE_VENDOR_EXT
