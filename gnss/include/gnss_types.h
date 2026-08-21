// gnss/include/gnss_types.h
// انواع پایه کتابخانه GNSS - نسخه توسعه‌یافته با Vendor + UBX DOP/SAT
#ifndef GNSS_TYPES_H
#define GNSS_TYPES_H

#include "gnss_config.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ======================== دقت عددی ========================
#if GNSS_USE_DOUBLE
typedef double gnss_float_t;
#else
typedef float gnss_float_t;
#endif

// ======================== کدهای خطا ========================
typedef enum {
    GNSS_OK = 0,
    GNSS_ERR_INVALID_ARG          = -1,
    GNSS_ERR_INVALID_FRAME        = -2,
    GNSS_ERR_BAD_CHECKSUM         = -3,
    GNSS_ERR_BUFFER_OVERFLOW      = -4,
    GNSS_ERR_UNSUPPORTED_MESSAGE  = -5,
    GNSS_ERR_NO_FIX               = -6,
    GNSS_ERR_INVALID_FIELD        = -7,
    GNSS_ERR_VENDOR_NOT_SUPPORTED = -8
} gnss_error_t;

// ======================== شناسه فیلدها (API سطح ۲) ========================
typedef enum {
    GNSS_FIELD_LATITUDE = 0,
    GNSS_FIELD_LONGITUDE,
    GNSS_FIELD_ALTITUDE,
    GNSS_FIELD_SPEED,
    GNSS_FIELD_COURSE,
    GNSS_FIELD_HDOP,
    GNSS_FIELD_VDOP,
    GNSS_FIELD_PDOP,
    GNSS_FIELD_GDOP,
    GNSS_FIELD_TDOP,
    GNSS_FIELD_SATELLITES,
    GNSS_FIELD_FIX_TYPE,
    GNSS_FIELD_COUNT
} gnss_field_id_t;

// ======================== شناسه GNSS (بر اساس NMEA + UBX) ========================
typedef enum {
    GNSS_ID_GPS = 0,
    GNSS_ID_GLONASS = 1,
    GNSS_ID_GALILEO = 2,
    GNSS_ID_BEIDOU = 3,
    GNSS_ID_QZSS = 4,
    GNSS_ID_NAVIC = 5,
    GNSS_ID_SBAS = 6,
    GNSS_ID_COMBINED = 7,
    GNSS_ID_UNKNOWN = 255
} gnss_constellation_t;

// ======================== Vendor IDs ========================
typedef enum {
    GNSS_VENDOR_UNKNOWN = 0,
    GNSS_VENDOR_QUECTEL,
    GNSS_VENDOR_TRIMBLE,
    GNSS_VENDOR_MTK,
    GNSS_VENDOR_SIRF,
    GNSS_VENDOR_UBLOX
} gnss_vendor_id_t;

// ======================== ساختار جواب نهایی ========================
typedef struct {
    gnss_float_t latitude_deg;
    gnss_float_t longitude_deg;
    gnss_float_t altitude_m;     // hMSL
    gnss_float_t altitude_ellipsoid_m; // height above ellipsoid (از UBX)
    gnss_float_t speed_mps;
    gnss_float_t course_deg;
    gnss_float_t hdop;
    gnss_float_t vdop;
    gnss_float_t pdop;
    gnss_float_t gdop;
    gnss_float_t tdop;
    gnss_float_t ndop; // northing
    gnss_float_t edop; // easting
    uint8_t      satellites;
    uint8_t      fix_type; // 0=No fix,1=DR,2=2D,3=3D,4=GNSS+DR,5=Time only
    bool         valid_position;
    bool         valid_velocity;
    bool         valid_time;
    bool         valid_dop;
    // زمان UTC
    uint16_t     year;
    uint8_t      month;
    uint8_t      day;
    uint8_t      hour;
    uint8_t      minute;
    uint8_t      second;
    uint16_t     millisecond;
    // اطلاعات اضافی برای RTK/Trimble
    uint8_t      rtk_fix_type; // 0=No RTK,1=Float,2=Fixed
    gnss_float_t rtk_age_sec;
    gnss_vendor_id_t last_vendor;
} gnss_solution_t;

// ======================== ماهواره ========================
typedef struct {
    uint8_t  prn;
    uint8_t  elevation;         // 0..90
    uint16_t azimuth;           // 0..359
    uint8_t  snr;               // C/No dBHz
    uint8_t  gnss_id;           // gnss_constellation_t
    bool     used_in_solution;
    bool     healthy;
    // فیلدهای اضافی UBX NAV-SAT
    uint8_t  quality;           // signal quality indicator
    int8_t   elev_raw;          // raw elevation signed
    uint16_t pr_res;            // pseudorange residual
} gnss_satellite_t;

// ======================== آمار ========================
typedef struct {
    uint32_t messages_received;
    uint32_t messages_parsed;
    uint32_t checksum_errors;
    uint32_t frame_errors;
    uint32_t buffer_overflows;
    uint32_t nmea_messages;
    uint32_t ubx_messages;
    uint32_t vendor_messages;
    uint32_t gsv_messages;
    uint32_t gsa_messages;
} gnss_stats_t;

// ======================== Vendor Message ========================
typedef struct {
    gnss_vendor_id_t vendor;
    char talker[4]; // e.g., "PQ", "PT", "PM"
    char type[16];  // e.g., "GSV", "AVR", "GGK"
    char subtype[16]; // e.g., for PTNL, AVR
    char raw[GNSS_VENDOR_MAX_SENTENCE_LEN];
} gnss_vendor_msg_t;

#ifdef __cplusplus
}
#endif

#endif // GNSS_TYPES_H
