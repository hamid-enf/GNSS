// gnss/src/core/gnss.c
// هسته اصلی - نسخه توسعه‌یافته با GSA used, UBX SAT/DOP, Vendor, DOP کامل

#include "gnss.h"
#include "gnss_config.h"
#include "gnss_callbacks.h"
#include "parser/frame.h"
#if GNSS_ENABLE_VENDOR_EXT
#include "parser/vendor/vendor_parser.h"
#endif
#include <string.h>
#include <stdbool.h>

#if GNSS_ENABLE_STATS
static gnss_stats_t g_stats = {0};
#endif

static gnss_solution_t g_solution = {0};
static bool g_has_new_solution = false;

#if GNSS_ENABLE_GSV || GNSS_ENABLE_UBX_NAV_SAT
static gnss_satellite_t g_satellites[GNSS_MAX_SATELLITES];
static uint8_t g_sat_count = 0;
#endif

#if GNSS_ENABLE_VENDOR_EXT
static gnss_vendor_msg_t g_last_vendor_msg_cache = {0};
#endif

// ======================== مقداردهی اولیه ========================
void gnss_init(void)
{
    memset(&g_solution, 0, sizeof(g_solution));
    g_has_new_solution = false;

#if GNSS_ENABLE_GSV || GNSS_ENABLE_UBX_NAV_SAT
    memset(g_satellites, 0, sizeof(g_satellites));
    g_sat_count = 0;
#endif

#if GNSS_ENABLE_STATS
    gnss_reset_stats();
#endif

#if GNSS_ENABLE_VENDOR_EXT
    vendor_parser_init();
    memset(&g_last_vendor_msg_cache, 0, sizeof(g_last_vendor_msg_cache));
#endif

    gnss_frame_init();
}

// ======================== merge هوشمند ========================
static void gnss_update_solution(const gnss_solution_t* new_sol)
{
    if (!new_sol) return;

    if (new_sol->valid_position) {
        g_solution.latitude_deg  = new_sol->latitude_deg;
        g_solution.longitude_deg = new_sol->longitude_deg;
        g_solution.valid_position = true;
        if (new_sol->fix_type != 0) g_solution.fix_type = new_sol->fix_type;
        if (new_sol->satellites != 0) g_solution.satellites = new_sol->satellites;
        if (new_sol->altitude_m != 0.0 || new_sol->fix_type > 0) g_solution.altitude_m = new_sol->altitude_m;
        if (new_sol->altitude_ellipsoid_m != 0.0) g_solution.altitude_ellipsoid_m = new_sol->altitude_ellipsoid_m;
        if (new_sol->hdop != 0.0) g_solution.hdop = new_sol->hdop;
        if (new_sol->valid_time) {
            g_solution.valid_time = true;
            g_solution.year = new_sol->year;
            g_solution.month = new_sol->month;
            g_solution.day = new_sol->day;
            g_solution.hour = new_sol->hour;
            g_solution.minute = new_sol->minute;
            g_solution.second = new_sol->second;
            g_solution.millisecond = new_sol->millisecond;
        }
        if (new_sol->last_vendor != GNSS_VENDOR_UNKNOWN) g_solution.last_vendor = new_sol->last_vendor;
        if (new_sol->rtk_fix_type != 0) {
            g_solution.rtk_fix_type = new_sol->rtk_fix_type;
            g_solution.rtk_age_sec = new_sol->rtk_age_sec;
        }
    }

    if (new_sol->valid_velocity) {
        g_solution.speed_mps = new_sol->speed_mps;
        g_solution.course_deg = new_sol->course_deg;
        g_solution.valid_velocity = true;
        if (new_sol->last_vendor != GNSS_VENDOR_UNKNOWN) g_solution.last_vendor = new_sol->last_vendor;
    }

    // DOPs - همیشه حتی بدون position
    if (new_sol->hdop != 0.0) { g_solution.hdop = new_sol->hdop; g_solution.valid_dop = true; }
    if (new_sol->vdop != 0.0) { g_solution.vdop = new_sol->vdop; g_solution.valid_dop = true; }
    if (new_sol->pdop != 0.0) { g_solution.pdop = new_sol->pdop; g_solution.valid_dop = true; }
    if (new_sol->gdop != 0.0) { g_solution.gdop = new_sol->gdop; g_solution.valid_dop = true; }
    if (new_sol->tdop != 0.0) { g_solution.tdop = new_sol->tdop; g_solution.valid_dop = true; }
    if (new_sol->ndop != 0.0) g_solution.ndop = new_sol->ndop;
    if (new_sol->edop != 0.0) g_solution.edop = new_sol->edop;

    // GSA فقط fix_type
    if (new_sol->fix_type != 0 && !new_sol->valid_position && !new_sol->valid_velocity && !new_sol->valid_dop) {
        g_solution.fix_type = new_sol->fix_type;
    } else if (new_sol->fix_type != 0 && new_sol->valid_dop) {
        // GSA با DOP
        g_solution.fix_type = new_sol->fix_type;
    }

    if (new_sol->altitude_m != 0.0 && g_solution.altitude_m == 0.0) g_solution.altitude_m = new_sol->altitude_m;
    if (new_sol->altitude_ellipsoid_m != 0.0 && g_solution.altitude_ellipsoid_m == 0.0) g_solution.altitude_ellipsoid_m = new_sol->altitude_ellipsoid_m;

    if (!g_solution.valid_position && !g_solution.valid_velocity && g_solution.fix_type == 0) {
        if (new_sol->fix_type != 0) g_solution.fix_type = new_sol->fix_type;
    }

    // valid_time مستقل
    if (new_sol->valid_time && !g_solution.valid_position && !g_solution.valid_velocity) {
        g_solution.valid_time = true;
        g_solution.year = new_sol->year;
        g_solution.month = new_sol->month;
        g_solution.day = new_sol->day;
        g_solution.hour = new_sol->hour;
        g_solution.minute = new_sol->minute;
        g_solution.second = new_sol->second;
    }

    g_has_new_solution = true;

#if GNSS_ENABLE_STATS
    g_stats.messages_parsed++;
#endif

#if GNSS_ENABLE_CALLBACKS
    gnss_internal_fire_callback(GNSS_EVENT_SOLUTION, &g_solution, 0);
#endif
}

// ======================== API سطح ۱ ========================
gnss_error_t gnss_parse(const uint8_t* buffer, size_t length)
{
    if (!buffer || length == 0) return GNSS_ERR_INVALID_ARG;
#if GNSS_ENABLE_STATS
    g_stats.messages_received++;
#endif
    return gnss_frame_parse_buffer(buffer, length);
}

gnss_error_t gnss_feed(const uint8_t* data, size_t length)
{
    if (!data || length == 0) return GNSS_ERR_INVALID_ARG;
#if GNSS_ENABLE_STATS
    g_stats.messages_received += 1;
#endif
    gnss_error_t last_ok = GNSS_OK;
    for (size_t i = 0; i < length; i++) {
        gnss_error_t err = gnss_feed_byte(data[i]);
        if (err != GNSS_OK && err != GNSS_ERR_UNSUPPORTED_MESSAGE) last_ok = err;
    }
    return last_ok;
}

gnss_error_t gnss_feed_byte(uint8_t byte) { return gnss_frame_feed_byte(byte); }

bool gnss_has_new_solution(void) { return g_has_new_solution; }

gnss_error_t gnss_get_solution(gnss_solution_t* solution)
{
    if (!solution) return GNSS_ERR_INVALID_ARG;
    if (!g_has_new_solution) return GNSS_ERR_NO_FIX;
    memcpy(solution, &g_solution, sizeof(gnss_solution_t));
    g_has_new_solution = false;
    return GNSS_OK;
}

// ======================== API سطح ۲ ========================
gnss_error_t gnss_get_field(gnss_field_id_t field, void* value, size_t* size)
{
    if (!value || !size) return GNSS_ERR_INVALID_ARG;
    switch (field) {
        case GNSS_FIELD_LATITUDE:
            if (*size < sizeof(gnss_float_t)) return GNSS_ERR_INVALID_ARG;
            *(gnss_float_t*)value = g_solution.latitude_deg;
            *size = sizeof(gnss_float_t);
            return g_solution.valid_position ? GNSS_OK : GNSS_ERR_NO_FIX;
        case GNSS_FIELD_LONGITUDE:
            if (*size < sizeof(gnss_float_t)) return GNSS_ERR_INVALID_ARG;
            *(gnss_float_t*)value = g_solution.longitude_deg;
            *size = sizeof(gnss_float_t);
            return g_solution.valid_position ? GNSS_OK : GNSS_ERR_NO_FIX;
        case GNSS_FIELD_ALTITUDE:
            if (*size < sizeof(gnss_float_t)) return GNSS_ERR_INVALID_ARG;
            *(gnss_float_t*)value = g_solution.altitude_m;
            *size = sizeof(gnss_float_t);
            return g_solution.valid_position ? GNSS_OK : GNSS_ERR_NO_FIX;
        case GNSS_FIELD_SPEED:
            if (*size < sizeof(gnss_float_t)) return GNSS_ERR_INVALID_ARG;
            *(gnss_float_t*)value = g_solution.speed_mps;
            *size = sizeof(gnss_float_t);
            return g_solution.valid_velocity ? GNSS_OK : GNSS_ERR_NO_FIX;
        case GNSS_FIELD_COURSE:
            if (*size < sizeof(gnss_float_t)) return GNSS_ERR_INVALID_ARG;
            *(gnss_float_t*)value = g_solution.course_deg;
            *size = sizeof(gnss_float_t);
            return g_solution.valid_velocity ? GNSS_OK : GNSS_ERR_NO_FIX;
        case GNSS_FIELD_SATELLITES:
            if (*size < sizeof(uint8_t)) return GNSS_ERR_INVALID_ARG;
            *(uint8_t*)value = g_solution.satellites;
            *size = sizeof(uint8_t);
            return GNSS_OK;
        case GNSS_FIELD_HDOP:
            if (*size < sizeof(gnss_float_t)) return GNSS_ERR_INVALID_ARG;
            *(gnss_float_t*)value = g_solution.hdop;
            *size = sizeof(gnss_float_t);
            return GNSS_OK;
        case GNSS_FIELD_VDOP:
            if (*size < sizeof(gnss_float_t)) return GNSS_ERR_INVALID_ARG;
            *(gnss_float_t*)value = g_solution.vdop;
            *size = sizeof(gnss_float_t);
            return GNSS_OK;
        case GNSS_FIELD_PDOP:
            if (*size < sizeof(gnss_float_t)) return GNSS_ERR_INVALID_ARG;
            *(gnss_float_t*)value = g_solution.pdop;
            *size = sizeof(gnss_float_t);
            return GNSS_OK;
        case GNSS_FIELD_GDOP:
            if (*size < sizeof(gnss_float_t)) return GNSS_ERR_INVALID_ARG;
            *(gnss_float_t*)value = g_solution.gdop;
            *size = sizeof(gnss_float_t);
            return GNSS_OK;
        case GNSS_FIELD_TDOP:
            if (*size < sizeof(gnss_float_t)) return GNSS_ERR_INVALID_ARG;
            *(gnss_float_t*)value = g_solution.tdop;
            *size = sizeof(gnss_float_t);
            return GNSS_OK;
        case GNSS_FIELD_FIX_TYPE:
            if (*size < sizeof(uint8_t)) return GNSS_ERR_INVALID_ARG;
            *(uint8_t*)value = g_solution.fix_type;
            *size = sizeof(uint8_t);
            return GNSS_OK;
        default: return GNSS_ERR_INVALID_FIELD;
    }
}

bool gnss_is_valid(gnss_field_id_t field)
{
    switch (field) {
        case GNSS_FIELD_LATITUDE:
        case GNSS_FIELD_LONGITUDE:
        case GNSS_FIELD_ALTITUDE: return g_solution.valid_position;
        case GNSS_FIELD_SPEED:
        case GNSS_FIELD_COURSE: return g_solution.valid_velocity;
        case GNSS_FIELD_SATELLITES:
        case GNSS_FIELD_HDOP:
        case GNSS_FIELD_VDOP:
        case GNSS_FIELD_PDOP:
        case GNSS_FIELD_GDOP:
        case GNSS_FIELD_TDOP:
        case GNSS_FIELD_FIX_TYPE: return true;
        default: return false;
    }
}

// ======================== ماهواره‌ها ========================
#if GNSS_ENABLE_GSV || GNSS_ENABLE_UBX_NAV_SAT
gnss_error_t gnss_get_satellite_count(uint8_t* count)
{
    if (!count) return GNSS_ERR_INVALID_ARG;
    *count = g_sat_count;
    return GNSS_OK;
}

gnss_error_t gnss_get_satellite(uint8_t index, gnss_satellite_t* sat)
{
    if (!sat || index >= g_sat_count) return GNSS_ERR_INVALID_ARG;
    memcpy(sat, &g_satellites[index], sizeof(gnss_satellite_t));
    return GNSS_OK;
}

gnss_error_t gnss_get_satellites(gnss_satellite_t* sats, uint8_t max_count, uint8_t* out_count)
{
    if (!sats || !out_count) return GNSS_ERR_INVALID_ARG;
    uint8_t cnt = (g_sat_count < max_count) ? g_sat_count : max_count;
    memcpy(sats, g_satellites, cnt * sizeof(gnss_satellite_t));
    *out_count = cnt;
    return GNSS_OK;
}
#endif

// ======================== آمار ========================
#if GNSS_ENABLE_STATS
gnss_error_t gnss_get_stats(gnss_stats_t* stats)
{
    if (!stats) return GNSS_ERR_INVALID_ARG;
    memcpy(stats, &g_stats, sizeof(gnss_stats_t));
    return GNSS_OK;
}
void gnss_reset_stats(void) { memset(&g_stats, 0, sizeof(g_stats)); }
#endif

// ======================== داخلی ========================
void gnss_internal_update_solution(const gnss_solution_t* sol) { gnss_update_solution(sol); }

#if GNSS_ENABLE_GSV || GNSS_ENABLE_UBX_NAV_SAT
void gnss_internal_set_satellites(const gnss_satellite_t* sats, uint8_t count)
{
    if (!sats) return;
    if (count > GNSS_MAX_SATELLITES) count = GNSS_MAX_SATELLITES;
    memcpy(g_satellites, sats, count * sizeof(gnss_satellite_t));
    g_sat_count = count;
#if GNSS_ENABLE_STATS
    g_stats.gsv_messages++;
#endif
#if GNSS_ENABLE_CALLBACKS
    gnss_internal_fire_callback(GNSS_EVENT_SATELLITES, g_satellites, count);
#endif
}
#endif

void gnss_internal_report_error(gnss_error_t err)
{
#if GNSS_ENABLE_STATS
    if (err == GNSS_ERR_BAD_CHECKSUM) g_stats.checksum_errors++;
    else if (err == GNSS_ERR_INVALID_FRAME) g_stats.frame_errors++;
    else if (err == GNSS_ERR_BUFFER_OVERFLOW) g_stats.buffer_overflows++;
#endif
#if GNSS_ENABLE_CALLBACKS
    if (err != GNSS_OK && err != GNSS_ERR_UNSUPPORTED_MESSAGE) {
        gnss_internal_fire_callback(GNSS_EVENT_ERROR, &err, 0);
    }
#endif
}

#if GNSS_ENABLE_VENDOR_EXT
// wrapper برای vendor_parser.c که last_msg را هم cache کند
void gnss_internal_vendor_set_last(const gnss_vendor_msg_t* msg)
{
    if (msg) memcpy(&g_last_vendor_msg_cache, msg, sizeof(gnss_vendor_msg_t));
}
#endif
