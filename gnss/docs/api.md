# GNSS-Core API Documentation - نسخه کامل

## فهرست

1. [مقدمه](#مقدمه)
2. [پیکربندی](#پیکربندی)
3. [API سطح ۱ - ساده](#api-سطح-۱)
4. [API سطح ۲ - فیلد](#api-سطح-۲)
5. [ماهواره‌ها](#ماهواره‌ها)
6. [UBX Extended](#ubx-extended)
7. [Vendor Extensions](#vendor-extensions)
8. [Callback System](#callback)
9. [Stats](#stats)
10. [مثال‌ها](#مثال‌ها)

---

## مقدمه

`gnss-core` کتابخانه‌ای Zero-Allocation برای parsing GNSS است. دو پروتکل اصلی NMEA و UBX + افزونه‌های Quectel/Trimble را پشتیبانی می‌کند.

**واحدهای استاندارد:**
- مختصات: درجه اعشاری (WGS84)
- ارتفاع: متر (hMSL و ellipsoid)
- سرعت: m/s
- جهت: درجه

---

## پیکربندی

فایل `include/gnss_config.h`:

```c
#define GNSS_ENABLE_NMEA 1
#define GNSS_ENABLE_UBX 1
#define GNSS_ENABLE_VENDOR_EXT 1
#define GNSS_ENABLE_GGA 1
#define GNSS_ENABLE_RMC 1
#define GNSS_ENABLE_GSA 1 // شامل used_in_solution
#define GNSS_ENABLE_GSV 1
#define GNSS_ENABLE_VTG 1
#define GNSS_ENABLE_GLL 1
#define GNSS_ENABLE_ZDA 1
#define GNSS_ENABLE_UBX_NAV_PVT 1
#define GNSS_ENABLE_UBX_NAV_SAT 1 // جدید
#define GNSS_ENABLE_UBX_NAV_DOP 1 // جدید
#define GNSS_ENABLE_VENDOR_QUECTEL 1
#define GNSS_ENABLE_VENDOR_TRIMBLE 1
#define GNSS_MAX_SATELLITES 64
#define GNSS_RX_BUFFER_SIZE 512 // برای NAV-SAT بزرگ
```

---

## API سطح ۱

### `void gnss_init(void)`
مقداردهی اولیه. باید یکبار در شروع صدا زده شود. بافرها و آمار را ریست می‌کند.

### `gnss_error_t gnss_parse(const uint8_t* buffer, size_t length)`
بافر کامل را parse می‌کند. مناسب برای فایل یا تست.
```c
gnss_parse((uint8_t*)"$GPGGA,...*47\r\n", len);
```

### `gnss_error_t gnss_feed(const uint8_t* data, size_t length)`
مشابه parse ولی برای DMA. بافر را بایت‌به‌بایت feed می‌کند.

### `gnss_error_t gnss_feed_byte(uint8_t byte)`
مناسب برای UART ISR:
```c
void USART2_IRQHandler(void) {
    uint8_t b = USART2->DR;
    gnss_feed_byte(b);
}
```

### `bool gnss_has_new_solution(void)`
آیا solution جدید داریم؟

### `gnss_error_t gnss_get_solution(gnss_solution_t* sol)`
solution را کپی می‌کند و flag را پاک می‌کند. اگر solution جدید نباشد `GNSS_ERR_NO_FIX` برمی‌گرداند.

**ساختار `gnss_solution_t`:**
```c
typedef struct {
    double latitude_deg, longitude_deg;
    double altitude_m; // hMSL
    double altitude_ellipsoid_m;
    double speed_mps, course_deg;
    double hdop, vdop, pdop, gdop, tdop;
    uint8_t satellites, fix_type;
    bool valid_position, valid_velocity, valid_time, valid_dop;
    uint16_t year; uint8_t month,day,hour,minute,second;
    uint8_t rtk_fix_type; // برای Trimble RTK
    gnss_vendor_id_t last_vendor;
} gnss_solution_t;
```

---

## API سطح ۲

### `gnss_get_field(field_id, value, size)`
```c
double lat; size_t sz=sizeof(lat);
gnss_get_field(GNSS_FIELD_LATITUDE, &lat, &sz);
```

Field IDs:
- `GNSS_FIELD_LATITUDE`, `LONGITUDE`, `ALTITUDE`
- `GNSS_FIELD_SPEED`, `COURSE`
- `GNSS_FIELD_HDOP`, `VDOP`, `PDOP`, `GDOP`, `TDOP`
- `GNSS_FIELD_SATELLITES`, `FIX_TYPE`

### `bool gnss_is_valid(field_id)`
بررسی اعتبار فیلد.

---

## ماهواره‌ها

### GSA + GSV Aggregation با used flag (جدید)

**قابلیت جدید:** GSA لیست PRN های استفاده‌شده را می‌دهد. کتابخانه آن را با GSV ترکیب می‌کند و `used_in_solution` را پر می‌کند.

```c
// ترتیب مهم نیست: GSA اول یا GSV اول، هر دو کار می‌کند
gnss_parse(gsv1);
gnss_parse(gsv2);
gnss_parse(gsa); // PRN های 02,04,05,09 used می‌شوند

uint8_t cnt;
gnss_get_satellite_count(&cnt);
for (int i=0;i<cnt;i++) {
    gnss_satellite_t sat;
    gnss_get_satellite(i, &sat);
    printf("PRN %d used=%d\n", sat.prn, sat.used_in_solution);
}
```

### `gnss_get_satellite_count`, `gnss_get_satellite`, `gnss_get_satellites`

---

## UBX Extended

### NAV-PVT (0x01 0x07)
قبلاً بود، الان با `altitude_ellipsoid_m` و `valid_dop` و زمان کامل.

### NAV-SAT (0x01 0x35) - جدید
```c
// Payload: iTOW(4) + version(1) + numSvs(1) + reserved(2) + 12*numSvs
// هر ماهواره 12 بایت: gnssId, svId, cno, elev, azim, prRes, flags
// flags bit 3 = svUsed
```
کتابخانه `gnss_internal_set_satellites` را صدا می‌زند و `used_in_solution` را از flags پر می‌کند.

### NAV-DOP (0x01 0x04) - جدید
```c
// gDOP, pDOP, tDOP, vDOP, hDOP, nDOP, eDOP - هر کدام U2 scale 0.01
```
همه DOP ها به solution اضافه می‌شوند.

---

## Vendor Extensions

### معماری

```c
// include/gnss_vendor.h
typedef gnss_error_t (*gnss_vendor_parser_t)(char* fields[], int count, const char* talker, const char* full_id, const char* raw);

gnss_error_t gnss_vendor_register_parser(gnss_vendor_parser_t parser);
```

کتابخانه داخلی دو vendor را دارد:

#### Quectel
- `$PQGSV`, `$PQGSA` → به صورت GSV/GSA استاندارد هندل می‌شود
- `$PQGNS` → مشابه GGA+RMC
- `$PQTXT` → متن اطلاعاتی
- `$PQTMVER`, `$PQTMGNSS` → نسخه و اطلاعات

#### Trimble
- `$PTNL,AVR` → Attitude (yaw, tilt, roll)
  ```
  $PTNL,AVR,time,yaw,Yaw,tilt,Tilt,roll,Roll,range,2D,1D,1.5,1*cs
  ```
  yaw به course_deg تبدیل می‌شود.

- `$PTNL,GGK` → RTK position (مشابه GGA با RTK fix)
  ```
  $PTNL,GGK,time,date,lat,N,lon,E,fix,num_sats,dop,alt,M*cs
  ```
  fix_type و rtk_fix_type پر می‌شود.

- `$PTNL,BPQ`, `VGK`, `GGA` → پشتیبانی

#### MTK
- `$PMTK010,001*2E` → system messages

### استفاده

```c
gnss_parse((uint8_t*)"$PTNL,GGK,123519.00,071219,4807.038,N,01131.000,E,4,08,0.9,545.4,M*23\r\n", len);

gnss_vendor_msg_t vm;
gnss_get_last_vendor_msg(&vm);
printf("Vendor %d Type %s\n", vm.vendor, vm.type);
```

### Custom Parser

```c
gnss_error_t my_parser(char* fields[], int cnt, const char* talker, const char* full_id, const char* raw) {
    if (strcmp(full_id, "PXYZ")==0) { printf("Got PXYZ\n"); return GNSS_OK; }
    return GNSS_ERR_UNSUPPORTED_MESSAGE;
}
gnss_vendor_register_parser(my_parser);
```

---

## Callback

```c
void my_cb(const gnss_event_t* ev) {
    if (ev->type == GNSS_EVENT_SOLUTION) { ... }
    else if (ev->type == GNSS_EVENT_SATELLITES) { ... }
    else if (ev->type == GNSS_EVENT_ERROR) { ... }
}
gnss_init();
gnss_register_callback(my_cb);
```

Event types:
- `GNSS_EVENT_SOLUTION`, `POSITION`, `SATELLITES`, `ERROR`

---

## Stats

```c
gnss_stats_t stats;
gnss_get_stats(&stats);
printf("Parsed %u, NMEA %u, UBX %u, Vendor %u\n",
    stats.messages_parsed, stats.nmea_messages, stats.ubx_messages, stats.vendor_messages);
```

---

## مثال‌ها

### ساده‌ترین (یک بافر)

```c
#include "gnss.h"
gnss_init();
gnss_parse((uint8_t*)"$GPGGA,...*47\r\n", len);
if (gnss_has_new_solution()) {
    gnss_solution_t sol;
    gnss_get_solution(&sol);
    printf("Lat %.6f Lon %.6f\n", sol.latitude_deg, sol.longitude_deg);
}
```

### Bare-Metal (examples/baremetal/main.c)

فایل کامل با GGA+GSA+GSV+RMC+VTG، نمایش ماهواره با used flag، field API، stats و byte-by-byte.

### STM32 UART + DMA (examples/stm32/stm32_uart_example.c)

```c
uint8_t dma_rx_buffer[128];
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    gnss_feed(dma_rx_buffer, Size);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_buffer, sizeof(dma_rx_buffer));
}
```

### UBX (examples/ubx/ubx_example.c)

ساخت دستی NAV-PVT, NAV-SAT, NAV-DOP و parse.

### Vendor (examples/vendor/vendor_example.c)

Quectel + Trimble + custom parser.

---

## خطاها

- `GNSS_OK` = 0
- `GNSS_ERR_BAD_CHECKSUM` = -3
- `GNSS_ERR_UNSUPPORTED_MESSAGE` = -5 (نادیده گرفته می‌شود، خطا نیست)
- `GNSS_ERR_NO_FIX` = -6
- ...

---

## نکات Embedded

- **Zero Allocation:** هیچ malloc ندارد.
- **Buffer:** برای UBX NAV-SAT با 64 ماهواره، RX buffer باید 512 باشد.
- **Double vs Float:** با `GNSS_USE_DOUBLE=0` و `FLOAT=1` می‌توان RAM/Flash را کم کرد.
- **Compile-time:** هر قابلیت غیرفعال، از باینری حذف می‌شود.

---

**نسخه:** 2.0 - با GSA used, UBX SAT/DOP, Vendor Quectel/Trimble
