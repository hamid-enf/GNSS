// gnss/include/gnss_vendor.h
// سیستم Vendor Extension - پشتیبانی Quectel, Trimble, MTK, SiRF
#ifndef GNSS_VENDOR_H
#define GNSS_VENDOR_H

#include "gnss_config.h"
#include "gnss_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if GNSS_ENABLE_VENDOR_EXT

// ======================== نوع Parser ========================
typedef gnss_error_t (*gnss_vendor_parser_t)(char* fields[], int field_count, const char* talker, const char* full_id, const char* raw_sentence);

// ======================== API ثبت Parser سفارشی ========================
gnss_error_t gnss_vendor_register_parser(gnss_vendor_parser_t parser);
gnss_error_t gnss_vendor_parse(char* fields[], int field_count, const char* talker, const char* msg_id, const char* full_id, const char* raw);

// ======================== Parsers داخلی ========================
#if GNSS_ENABLE_VENDOR_QUECTEL
gnss_error_t gnss_vendor_quectel_parse(char* fields[], int field_count, const char* talker, const char* full_id, const char* raw);
#endif

#if GNSS_ENABLE_VENDOR_TRIMBLE
gnss_error_t gnss_vendor_trimble_parse(char* fields[], int field_count, const char* talker, const char* full_id, const char* raw);
#endif

#if GNSS_ENABLE_VENDOR_MTK
gnss_error_t gnss_vendor_mtk_parse(char* fields[], int field_count, const char* talker, const char* full_id, const char* raw);
#endif

// ======================== Helpers ========================
gnss_vendor_id_t gnss_vendor_detect(const char* talker, const char* full_id);
void gnss_vendor_set_last_msg(const gnss_vendor_msg_t* msg);
gnss_error_t gnss_get_last_vendor_msg(gnss_vendor_msg_t* msg);

#endif // GNSS_ENABLE_VENDOR_EXT

#ifdef __cplusplus
}
#endif

#endif // GNSS_VENDOR_H
