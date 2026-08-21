// gnss/src/parser/nmea/nmea_parser.h
#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <stdint.h>
#include <stddef.h>
#include "gnss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

gnss_error_t nmea_parse(const uint8_t* buffer, size_t length);

#ifdef __cplusplus
}
#endif

#endif // NMEA_PARSER_H
