// gnss/src/parser/frame.c
// پارسر فریم سبک و بدون تخصیص پویا - بازنویسی کامل
// تشخیص پروتکل: NMEA ($) و UBX (0xB5 0x62)

#include "frame.h"
#include "gnss.h"
#include "gnss_config.h"
#include <string.h>

#if GNSS_ENABLE_NMEA
#include "nmea/nmea_parser.h"
#endif
#if GNSS_ENABLE_UBX
#include "ubx/ubx_parser.h"
#endif

// ======================== وضعیت پارسر ========================
typedef enum {
    FRAME_STATE_IDLE = 0,
    FRAME_STATE_NMEA,
    FRAME_STATE_UBX_SYNC2,
    FRAME_STATE_UBX_CLASS,
    FRAME_STATE_UBX_ID,
    FRAME_STATE_UBX_LEN1,
    FRAME_STATE_UBX_LEN2,
    FRAME_STATE_UBX_PAYLOAD,
    FRAME_STATE_UBX_CK_A,
    FRAME_STATE_UBX_CK_B
} frame_state_t;

static frame_state_t g_state = FRAME_STATE_IDLE;
static uint8_t  g_rx_buffer[GNSS_RX_BUFFER_SIZE];
static size_t   g_rx_index = 0;
static uint16_t g_ubx_length = 0;
static uint8_t  g_ubx_ck_a = 0;
static uint8_t  g_ubx_ck_b = 0;

// ======================== مقداردهی ========================
void gnss_frame_init(void)
{
    g_state = FRAME_STATE_IDLE;
    g_rx_index = 0;
    g_ubx_length = 0;
}

// ======================== پردازش بایت ========================
gnss_error_t gnss_frame_feed_byte(uint8_t byte)
{
    gnss_error_t err = GNSS_OK;

    // محافظت سرریز سراسری
    if (g_rx_index >= GNSS_RX_BUFFER_SIZE) {
        g_state = FRAME_STATE_IDLE;
        g_rx_index = 0;
        gnss_internal_report_error(GNSS_ERR_BUFFER_OVERFLOW);
        return GNSS_ERR_BUFFER_OVERFLOW;
    }

    switch (g_state) {
        case FRAME_STATE_IDLE:
            if (byte == '$') {
                g_state = FRAME_STATE_NMEA;
                g_rx_index = 0;
                g_rx_buffer[g_rx_index++] = byte;
            } else if (byte == 0xB5) {
                g_state = FRAME_STATE_UBX_SYNC2;
                g_rx_index = 0;
                g_rx_buffer[g_rx_index++] = byte;
            }
            break;

        case FRAME_STATE_NMEA:
#if GNSS_ENABLE_NMEA
            // ذخیره بایت
            g_rx_buffer[g_rx_index++] = byte;

            // پایان فریم NMEA با \n
            if (byte == '\n') {
                err = nmea_parse(g_rx_buffer, g_rx_index);
                if (err != GNSS_OK) {
                    gnss_internal_report_error(err);
                }
                g_state = FRAME_STATE_IDLE;
                g_rx_index = 0;
            } else if (g_rx_index >= GNSS_RX_BUFFER_SIZE - 1) {
                // بافر پر شد بدون \n
                gnss_internal_report_error(GNSS_ERR_BUFFER_OVERFLOW);
                err = GNSS_ERR_BUFFER_OVERFLOW;
                g_state = FRAME_STATE_IDLE;
                g_rx_index = 0;
            }
#else
            // NMEA غیرفعال، بازگشت به IDLE
            g_state = FRAME_STATE_IDLE;
            g_rx_index = 0;
            err = GNSS_ERR_UNSUPPORTED_MESSAGE;
#endif
            break;

        case FRAME_STATE_UBX_SYNC2:
            if (byte == 0x62) {
                // sync2 صحیح
                if (g_rx_index < GNSS_RX_BUFFER_SIZE) {
                    g_rx_buffer[g_rx_index++] = byte;
                    g_state = FRAME_STATE_UBX_CLASS;
                } else {
                    g_state = FRAME_STATE_IDLE;
                    g_rx_index = 0;
                    err = GNSS_ERR_BUFFER_OVERFLOW;
                }
            } else {
                // همگام‌سازی شکست خورد - آیا بایت جدید شروع NMEA است؟
                g_state = FRAME_STATE_IDLE;
                g_rx_index = 0;
                if (byte == '$') {
                    g_state = FRAME_STATE_NMEA;
                    g_rx_buffer[g_rx_index++] = byte;
                } else if (byte == 0xB5) {
                    // دوباره sync1
                    g_state = FRAME_STATE_UBX_SYNC2;
                    g_rx_buffer[g_rx_index++] = byte;
                }
            }
            break;

        case FRAME_STATE_UBX_CLASS:
            g_rx_buffer[g_rx_index++] = byte;
            g_state = FRAME_STATE_UBX_ID;
            break;

        case FRAME_STATE_UBX_ID:
            g_rx_buffer[g_rx_index++] = byte;
            g_state = FRAME_STATE_UBX_LEN1;
            break;

        case FRAME_STATE_UBX_LEN1:
            g_ubx_length = byte;
            g_rx_buffer[g_rx_index++] = byte;
            g_state = FRAME_STATE_UBX_LEN2;
            break;

        case FRAME_STATE_UBX_LEN2:
            g_ubx_length |= ((uint16_t)byte << 8);
            g_rx_buffer[g_rx_index++] = byte;

            // بررسی طول
            if (g_ubx_length > (GNSS_RX_BUFFER_SIZE - 8)) {
                // -8 برای هدر 6 + چک‌سام 2
                gnss_internal_report_error(GNSS_ERR_BUFFER_OVERFLOW);
                g_state = FRAME_STATE_IDLE;
                g_rx_index = 0;
                g_ubx_length = 0;
                return GNSS_ERR_BUFFER_OVERFLOW;
            }

            if (g_ubx_length == 0) {
                g_state = FRAME_STATE_UBX_CK_A;
            } else {
                g_state = FRAME_STATE_UBX_PAYLOAD;
            }
            break;

        case FRAME_STATE_UBX_PAYLOAD:
            // ذخیره payload از ایندکس 6 به بعد (هدر 6 بایت قبلاً ذخیره شده)
            if (g_rx_index < GNSS_RX_BUFFER_SIZE) {
                g_rx_buffer[g_rx_index++] = byte;
                // g_rx_index - 6 تعداد payload دریافتی
                if ((g_rx_index - 6) >= g_ubx_length) {
                    g_state = FRAME_STATE_UBX_CK_A;
                }
            } else {
                gnss_internal_report_error(GNSS_ERR_BUFFER_OVERFLOW);
                g_state = FRAME_STATE_IDLE;
                g_rx_index = 0;
                g_ubx_length = 0;
                return GNSS_ERR_BUFFER_OVERFLOW;
            }
            break;

        case FRAME_STATE_UBX_CK_A:
            g_ubx_ck_a = byte;
            g_state = FRAME_STATE_UBX_CK_B;
            break;

        case FRAME_STATE_UBX_CK_B:
            g_ubx_ck_b = byte;
#if GNSS_ENABLE_UBX
            {
                // g_rx_index در اینجا = 6 + payload_len
                // بافر شامل sync + class + id + len + payload است
                err = ubx_parse(g_rx_buffer, g_rx_index, g_ubx_ck_a, g_ubx_ck_b);
                if (err != GNSS_OK) {
                    gnss_internal_report_error(err);
                }
            }
#else
            err = GNSS_ERR_UNSUPPORTED_MESSAGE;
#endif
            g_state = FRAME_STATE_IDLE;
            g_rx_index = 0;
            g_ubx_length = 0;
            break;

        default:
            g_state = FRAME_STATE_IDLE;
            g_rx_index = 0;
            g_ubx_length = 0;
            break;
    }

    return err;
}

// ======================== پردازش بافر کامل ========================
gnss_error_t gnss_frame_parse_buffer(const uint8_t* buffer, size_t length)
{
    if (!buffer || length == 0) return GNSS_ERR_INVALID_ARG;

    gnss_error_t last_err = GNSS_OK;
    for (size_t i = 0; i < length; i++) {
        gnss_error_t err = gnss_frame_feed_byte(buffer[i]);
        // فقط خطاهای مهم را نگه دار، unsupported را نادیده بگیر
        if (err != GNSS_OK && err != GNSS_ERR_UNSUPPORTED_MESSAGE) {
            last_err = err;
        }
    }
    return last_err;
}
