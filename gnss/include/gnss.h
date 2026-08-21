// gnss/include/gnss.h
// API عمومی کتابخانه gnss-core - سه سطحی + Vendor + UBX Extended
#ifndef GNSS_H
#define GNSS_H

#include "gnss_config.h"
#include "gnss_types.h"
#include "gnss_callbacks.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ======================== مقداردهی ========================
void gnss_init(void);

// ======================== API سطح ۱ - ساده ========================
gnss_error_t gnss_parse(const uint8_t* buffer, size_t length);
gnss_error_t gnss_feed(const uint8_t* data, size_t length);
gnss_error_t gnss_feed_byte(uint8_t byte);

bool         gnss_has_new_solution(void);
gnss_error_t gnss_get_solution(gnss_solution_t* solution);

// ======================== API سطح ۲ - فیلد به فیلد ========================
gnss_error_t gnss_get_field(gnss_field_id_t field, void* value, size_t* size);
bool         gnss_is_valid(gnss_field_id_t field);

// ======================== ماهواره‌ها (GSV + NAV-SAT) ========================
#if GNSS_ENABLE_GSV || GNSS_ENABLE_UBX_NAV_SAT
gnss_error_t gnss_get_satellite_count(uint8_t* count);
gnss_error_t gnss_get_satellite(uint8_t index, gnss_satellite_t* sat);
gnss_error_t gnss_get_satellites(gnss_satellite_t* sats, uint8_t max_count, uint8_t* out_count);
#endif

// ======================== آمار ========================
#if GNSS_ENABLE_STATS
gnss_error_t gnss_get_stats(gnss_stats_t* stats);
void         gnss_reset_stats(void);
#endif

// ======================== Vendor Extension API ========================
#if GNSS_ENABLE_VENDOR_EXT
#include "gnss_vendor.h"
gnss_error_t gnss_vendor_register_parser(gnss_vendor_parser_t parser);
gnss_error_t gnss_get_last_vendor_msg(gnss_vendor_msg_t* msg);
#endif

// ======================== توابع داخلی (برای پارسرها) ========================
void gnss_internal_update_solution(const gnss_solution_t* sol);
#if GNSS_ENABLE_GSV || GNSS_ENABLE_UBX_NAV_SAT
void gnss_internal_set_satellites(const gnss_satellite_t* sats, uint8_t count);
#endif
void gnss_internal_report_error(gnss_error_t err);

#if GNSS_ENABLE_CALLBACKS
void gnss_internal_fire_callback(gnss_event_type_t type, const void* data, uint8_t count);
#endif

// برای GSA used flag
#if GNSS_ENABLE_GSA
void gnss_internal_set_used_prns(const uint8_t* prns, uint8_t count);
#endif

#ifdef __cplusplus
}
#endif

#endif // GNSS_H
