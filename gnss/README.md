# gnss-core v2.0

کتابخانه حرفه‌ای، سبک و production-grade برای **Parsing و استخراج داده GNSS/GPS** در سیستم‌های Embedded - نسخه توسعه‌یافته.

## ویژگی‌های کلیدی v2.0

- **Zero Dynamic Allocation** - بدون malloc، مناسب Bare-Metal و RTOS
- **دو حالت ورودی**: بافر کامل + بایت به بایت (UART/DMA ISR)
- **تشخیص خودکار پروتکل** (NMEA / UBX / Vendor)
- **NMEA کامل**: GGA,RMC,GSV (Aggregation), **GSA با used_in_solution flag**, VTG, GLL, ZDA, TXT
- **UBX Extended**: NAV-PVT, **NAV-SAT (ماهواره با used flag)**, **NAV-DOP (gDOP,pDOP,tDOP,vDOP,hDOP,nDOP,eDOP)**
- **Vendor Extensions**: **Quectel (PQGSV,PQGNS,PQTXT)**, **Trimble (PTNL,AVR,GGK RTK,BPQ,VGK)**, MTK, Custom Framework
- **API سه سطحی**: Beginner (parse/get_solution) + Normal (get_field/is_valid) + Advanced (callback, stats, vendor)
- **قابل تنظیم کامل** در زمان کامپایل
- **مناسب STM32** و سایر MCUها (C99)

## ساختار پروژه

```
gnss/
├── include/
│   ├── gnss.h                ← API اصلی
│   ├── gnss_types.h          ← انواع (solution, satellite, vendor, stats)
│   ├── gnss_config.h         ← پیکربندی
│   ├── gnss_callbacks.h      ← Callback
│   └── gnss_vendor.h         ← Vendor Framework
├── src/
│   ├── core/
│   │   ├── gnss.c            ← Dispatcher + Merge Logic + Stats
│   │   └── gnss_callbacks.c
│   └── parser/
│       ├── frame.c/.h        ← Byte Stream + Resync
│       ├── nmea/nmea_parser.c ← GGA,RMC,GSV,GSA(used),VTG,GLL,ZDA,TXT
│       ├── ubx/ubx_parser.c  ← NAV-PVT,SAT,DOP
│       └── vendor/vendor_parser.c ← Quectel,Trimble,MTK,Custom
├── examples/
│   ├── baremetal/main.c      ← مثال کامل Bare-Metal (وعده README)
│   ├── simple/
│   │   ├── simple_buffer.c   ← یک بافر ورودی، نمایش خروجی
│   │   ├── simple_stream.c   ← UART stream + DMA
│   │   └── callback_example.c
│   ├── vendor/vendor_example.c
│   ├── ubx/ubx_example.c
│   └── stm32/stm32_uart_example.c
├── tests/
│   ├── test_nmea.c
│   ├── test_ubx.c
│   ├── test_frame.c
│   ├── test_all.c
│   ├── test_gsa_used.c       ← جدید: تست GSA used flag
│   └── test_vendor.c         ← جدید: Quectel+Trimble
├── docs/
│   ├── architecture.md       ← معماری v2
│   ├── protocol_matrix.md    ← ماتریس پروتکل v2
│   └── api.md                ← مستندات API کامل (جدید)
├── CMakeLists.txt
└── Makefile
```

## استفاده سریع (سطح ۱)

```c
#include "gnss.h"

uint8_t rx_buf[256];

int main(void)
{
    gnss_init();

    // روش ۱: بافر کامل
    gnss_parse(rx_buf, sizeof(rx_buf));

    // روش ۲: بایت به بایت (مناسب ISR/DMA)
    for(int i=0; i<received; i++)
        gnss_feed_byte(rx_buf[i]);

    // روش ۳: DMA buffer
    gnss_feed(dma_buffer, dma_size);

    if (gnss_has_new_solution()) {
        gnss_solution_t sol;
        if (gnss_get_solution(&sol) == GNSS_OK) {
            printf("Lat: %.7f Lon: %.7f Alt: %.1f Sats: %d\n",
                sol.latitude_deg, sol.longitude_deg, sol.altitude_m, sol.satellites);
        }
    }
}
```

## مثال ملموس - یک بافر ورودی و نمایش خروجی

```c
// examples/simple/simple_buffer.c
#include "gnss.h"
#include <stdio.h>
#include <string.h>

int main(void) {
    gnss_init();
    const char* buf = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    gnss_parse((uint8_t*)buf, strlen(buf));
    if (gnss_has_new_solution()) {
        gnss_solution_t sol;
        gnss_get_solution(&sol);
        printf("Lat %.6f Lon %.6f Alt %.1f\n", sol.latitude_deg, sol.longitude_deg, sol.altitude_m);
    }
}
```

## GSA used_in_solution (جدید)

```c
// GSV + GSA = ماهواره‌های استفاده‌شده مشخص می‌شود
gnss_parse(gsv1); // 4 sats
gnss_parse(gsv2); // 4 sats -> total 8
gnss_parse(gsa);  // PRN 02,04,05,09 used

uint8_t cnt; gnss_get_satellite_count(&cnt);
for (int i=0;i<cnt;i++) {
    gnss_satellite_t sat;
    gnss_get_satellite(i, &sat);
    printf("PRN %d used=%d\n", sat.prn, sat.used_in_solution);
}
```

## UBX NAV-SAT + NAV-DOP (جدید)

```c
// NAV-SAT با used flag
// NAV-DOP با gDOP,pDOP,tDOP,vDOP,hDOP,nDOP,eDOP
gnss_parse(ubx_nav_sat_buffer, len);
gnss_parse(ubx_nav_dop_buffer, len);

gnss_solution_t sol;
gnss_get_solution(&sol);
printf("DOP g=%.2f p=%.2f h=%.2f v=%.2f\n", sol.gdop, sol.pdop, sol.hdop, sol.vdop);
```

## Vendor Extensions (جدید)

```c
// Quectel
gnss_parse((uint8_t*)"$PQGNS,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,46.9,,*64\r\n", len);

// Trimble RTK
gnss_parse((uint8_t*)"$PTNL,GGK,123519.00,071219,4807.038,N,01131.000,E,4,08,0.9,545.4,M*23\r\n", len);

// Custom parser
gnss_error_t my_parser(char* fields[], int cnt, const char* talker, const char* full_id, const char* raw) {
    if (strcmp(full_id, "PXYZ")==0) { printf("Custom PXYZ\n"); return GNSS_OK; }
    return GNSS_ERR_UNSUPPORTED_MESSAGE;
}
gnss_vendor_register_parser(my_parser);
```

## Callback (سطح پیشرفته)

```c
void my_cb(const gnss_event_t* ev) {
    if (ev->type == GNSS_EVENT_SOLUTION)
        printf("Solution lat=%.6f\n", ev->data.solution->latitude_deg);
}
gnss_register_callback(my_cb);
```

## پیکربندی (gnss_config.h)

```c
#define GNSS_ENABLE_NMEA 1
#define GNSS_ENABLE_UBX 1
#define GNSS_ENABLE_VENDOR_EXT 1
#define GNSS_ENABLE_GSA 1 // با used flag
#define GNSS_ENABLE_GSV 1
#define GNSS_ENABLE_UBX_NAV_SAT 1 // جدید
#define GNSS_ENABLE_UBX_NAV_DOP 1 // جدید
#define GNSS_ENABLE_VENDOR_QUECTEL 1
#define GNSS_ENABLE_VENDOR_TRIMBLE 1
#define GNSS_MAX_SATELLITES 64
#define GNSS_RX_BUFFER_SIZE 512
```

## تست‌ها

```bash
make test_nmea test_ubx test_frame test_all test_gsa_used test_vendor
./test_nmea       # GGA,RMC,GSV,GSA,VTG,GLL,GNSS
./test_ubx        # NAV-PVT
./test_gsa_used   # GSA used flag
./test_vendor     # Quectel + Trimble
./test_all        # Full integration
```

همه تست‌ها PASS.

## مثال‌ها

```bash
make simple_buffer && ./simple_buffer
make baremetal_example && ./baremetal_example
make vendor_example && ./vendor_example
make ubx_example && ./ubx_example
```

## وضعیت پیاده‌سازی v2.0

- [x] هسته اصلی + Dispatcher + Merge Logic
- [x] پارسر فریم (Byte Stream + Resync)
- [x] NMEA (GGA,RMC,GSV Aggregation, GSA با used flag, VTG, GLL, ZDA, TXT)
- [x] UBX (NAV-PVT, NAV-SAT با used, NAV-DOP کامل)
- [x] Vendor Extensions (Quectel, Trimble, MTK, Custom Framework)
- [x] تست کامل (6 تست)
- [x] مثال Bare-Metal + Simple + Vendor + UBX + STM32
- [x] مستندات API کامل (docs/api.md)

## مجوز

MIT License

---
**gnss-core v2.0** — GNSS Parsing Engine برای Embedded Systems با پشتیبانی کامل NMEA/UBX/Vendor
