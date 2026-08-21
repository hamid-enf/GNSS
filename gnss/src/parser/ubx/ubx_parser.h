// gnss/src/parser/ubx/ubx_parser.h
#ifndef UBX_PARSER_H
#define UBX_PARSER_H

#include <stdint.h>
#include <stddef.h>
#include "gnss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

gnss_error_t ubx_parse(const uint8_t* buffer, size_t length, uint8_t ck_a, uint8_t ck_b);

#ifdef __cplusplus
}
#endif

#endif // UBX_PARSER_H
