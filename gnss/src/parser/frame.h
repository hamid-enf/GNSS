// gnss/src/parser/frame.h
// پارسر فریم و تشخیص پروتکل (Byte Stream Parser)

#ifndef GNSS_FRAME_H
#define GNSS_FRAME_H

#include <stdint.h>
#include <stddef.h>
#include "gnss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void         gnss_frame_init(void);
gnss_error_t gnss_frame_feed_byte(uint8_t byte);
gnss_error_t gnss_frame_parse_buffer(const uint8_t* buffer, size_t length);

#ifdef __cplusplus
}
#endif

#endif // GNSS_FRAME_H
