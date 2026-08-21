// src/parser/vendor/vendor_parser.h
#ifndef VENDOR_PARSER_H
#define VENDOR_PARSER_H

#include "gnss_types.h"
#include "gnss_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if GNSS_ENABLE_VENDOR_EXT
#include "gnss_vendor.h"

gnss_error_t vendor_parser_init(void);
gnss_error_t vendor_parse_dispatch(char* fields[], int field_count, const char* talker, const char* msg_id, const char* full_id, const char* raw);

#endif

#ifdef __cplusplus
}
#endif

#endif // VENDOR_PARSER_H
