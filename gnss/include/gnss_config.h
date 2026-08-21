// gnss/include/gnss_config.h
// پیکربندی کامل کتابخانه GNSS - نسخه توسعه‌یافته
#ifndef GNSS_CONFIG_H
#define GNSS_CONFIG_H

// ======================== سطح فعال‌سازی پروتکل‌ها ========================
#define GNSS_ENABLE_NMEA          1
#define GNSS_ENABLE_UBX           1
#define GNSS_ENABLE_VENDOR_EXT    1

// ======================== پیام‌های NMEA ========================
#define GNSS_ENABLE_GGA           1
#define GNSS_ENABLE_RMC           1
#define GNSS_ENABLE_GSA           1
#define GNSS_ENABLE_GSV           1
#define GNSS_ENABLE_VTG           1
#define GNSS_ENABLE_GLL           1
#define GNSS_ENABLE_ZDA           1
#define GNSS_ENABLE_TXT           1

// ======================== پیام‌های UBX ========================
#define GNSS_ENABLE_UBX_NAV_PVT   1
#define GNSS_ENABLE_UBX_NAV_SAT   1
#define GNSS_ENABLE_UBX_NAV_DOP   1
#define GNSS_ENABLE_UBX_NAV_TIME  0

// ======================== Vendor Extensions ========================
#define GNSS_ENABLE_VENDOR_QUECTEL  1
#define GNSS_ENABLE_VENDOR_TRIMBLE  1
#define GNSS_ENABLE_VENDOR_MTK      1
#define GNSS_ENABLE_VENDOR_SIRF     0

// ======================== تنظیمات حافظه ========================
// RX buffer باید برای NAV-SAT (حداکثر 64 ماهواره *12 +8 = 776) کافی باشد
// برای embedded کوچک می‌توان 256 گذاشت ولی برای Full 512 توصیه می‌شود
#define GNSS_MAX_SATELLITES       64
#define GNSS_RX_BUFFER_SIZE       512

// ======================== دقت عددی ========================
#define GNSS_USE_DOUBLE           1
#define GNSS_USE_FLOAT            0

// ======================== قابلیت‌های اضافی ========================
#define GNSS_ENABLE_CALLBACKS     1
#define GNSS_ENABLE_STATS         1
#define GNSS_ENABLE_STRING_API    0
#define GNSS_ENABLE_RAW_MESSAGES  0

// ======================== اندازه‌گیری عملکرد ========================
#define GNSS_ENABLE_BENCHMARK     0

// ======================== Vendor Buffer ========================
#define GNSS_VENDOR_MAX_PARSERS   8
#define GNSS_VENDOR_MAX_SENTENCE_LEN 128

#endif // GNSS_CONFIG_H
