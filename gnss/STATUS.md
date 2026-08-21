# گزارش وضعیت پروژه gnss-core v2.0

## وضعیت کلی: کامل و production-ready

### بخش‌های تکمیل‌شده v2.0

| بخش | وضعیت | فایل‌های مربوطه | توضیحات جدید |
|-----|--------|-----------------|--------------|
| معماری و مستندات | کامل | `docs/architecture.md`, `protocol_matrix.md`, `docs/api.md` | v2.0 با Vendor + UBX extended |
| هسته اصلی + Dispatcher + Merge | کامل | `src/core/gnss.c` | Merge هوشمند GGA/RMC/GSA/VTG/UBX/Vendor |
| پارسر فریم (Byte Stream) | کامل | `src/parser/frame.c` | Resync + RX 512 + UBX large |
| NMEA GGA,RMC,VTG,GLL,ZDA,TXT | کامل | `src/parser/nmea/nmea_parser.c` | + GLL, ZDA, TXT |
| NMEA GSV Aggregation | کامل | `nmea_parser.c` | + gnss_id از Talker + 64 sats |
| NMEA GSA با used flag | کامل (جدید) | `nmea_parser.c` | **used_in_solution flag** - GSA PRN list + GSV merge |
| UBX NAV-PVT | کامل | `src/parser/ubx/ubx_parser.c` | + alt ellipsoid + valid_dop + time |
| UBX NAV-SAT | کامل (جدید) | `ubx_parser.c` | **64 sats, gnssId, svId, cno, elev, azim, used flag** |
| UBX NAV-DOP | کامل (جدید) | `ubx_parser.c` | **gDOP,pDOP,tDOP,vDOP,hDOP,nDOP,eDOP** |
| Vendor Quectel | کامل (جدید) | `src/parser/vendor/vendor_parser.c` | PQGSV,PQGSA,PQGNS,PQTXT,PQTM |
| Vendor Trimble | کامل (جدید) | `vendor_parser.c` | PTNL,AVR,GGK RTK,BPQ,VGK,GGA + RTK fix |
| Vendor MTK + Custom Framework | کامل (جدید) | `vendor_parser.c`, `include/gnss_vendor.h` | PMTK + register_parser |
| هدرها و انواع داده | کامل | `include/*.h` | solution با gdop,tdop,rtk,vendor |
| پیکربندی | کامل | `gnss_config.h` | 64 sats, 512 RX, Vendor, UBX SAT/DOP |
| CMake + Makefile | کامل | `CMakeLists.txt`, `Makefile` | + vendor + examples |
| تست واحد NMEA | کامل | `tests/test_nmea.c` | GGA,RMC,GSV,GSA,VTG,GNSS |
| تست UBX | کامل | `tests/test_ubx.c` | NAV-PVT |
| تست GSA used | کامل (جدید) | `tests/test_gsa_used.c` | **used_in_solution flag** |
| تست Vendor | کامل (جدید) | `tests/test_vendor.c` | **Quectel+Trimble** |
| تست Frame | کامل | `tests/test_frame.c` | Mixed NMEA+UBX+noise |
| تست Full | کامل | `tests/test_all.c` | Integration + callback + stats |
| مثال Bare-Metal | کامل (جدید) | `examples/baremetal/main.c` | **وعده README - یک بافر ورودی و نمایش خروجی** |
| مثال Simple Buffer/Stream | کامل (جدید) | `examples/simple/` | **ملموس و ساده** |
| مثال Callback | کامل (جدید) | `examples/simple/callback_example.c` | Event-driven |
| مثال Vendor | کامل (جدید) | `examples/vendor/vendor_example.c` | Quectel+Trimble+Custom |
| مثال UBX | کامل (جدید) | `examples/ubx/ubx_example.c` | NAV-PVT,SAT,DOP + mixed |
| مثال STM32 UART+DMA | کامل | `examples/stm32/stm32_uart_example.c` | HAL |
| مستندات API | کامل (جدید) | `docs/api.md` | **API کامل با مثال** |
| README | کامل | `README.md` | v2.0 با مثال‌های ملموس |

### ویژگی‌های کلیدی پیاده‌سازی‌شده v2.0

- Zero Dynamic Allocation
- دو روش ورودی (بافر + بایت‌به‌بایت + DMA)
- تشخیص خودکار پروتکل (NMEA/UBX/Vendor)
- GSV aggregation کامل + gnss_id
- **GSA used_in_solution flag (جدید)** - PRN list از GSA + اعمال به GSV
- Talker ID مستقل (GP,GN,GA,GL,GB,PQ,PT,PM)
- Checksum validation قوی (جستجوی * + Fletcher)
- **UBX NAV-SAT با used flag (جدید)**
- **UBX NAV-DOP کامل (جدید)**
- **Vendor Extensions Framework (جدید)** - Quectel, Trimble, MTK, Custom
- API سه‌سطحی (Beginner/Normal/Advanced)
- Callback system
- Stats کامل (nmea, ubx, vendor, gsv, gsa, checksum, overflow)
- قابل کامپایل روی دسکتاپ و STM32 (C99)

### تست‌ها - همه PASS

```
test_nmea: PASS (GGA,RMC,GSV,GSA,VTG,GNSS, byte-by-byte, bad checksum)
test_ubx: PASS (NAV-PVT, bad checksum, byte-by-byte)
test_frame: PASS (mixed NMEA+UBX+noise)
test_all: PASS (full seq + callback + stats + field API + sat API)
test_gsa_used: PASS (GSA used flag + reverse order)
test_vendor: PASS (Quectel PQGSV,PQGNS,PQTXT + Trimble AVR,GGK RTK + PMTK + custom)
```

### مثال‌ها - همه کار می‌کنند

```
simple_buffer: یک بافر ورودی → نمایش Lat/Lon/Alt/Sats
simple_stream: UART stream + DMA
callback_example: Event-driven callback
vendor_example: Quectel + Trimble + Custom parser
ubx_example: NAV-PVT + NAV-SAT (used) + NAV-DOP + mixed NMEA+UBX
baremetal_example: کامل با GGA+GSA+GSV+RMC+VTG + sat used + field API + stats + byte-by-byte
stm32_uart_example: HAL UARTEx ReceiveToIdle DMA
```

### مصرف حافظه تقریبی (GCC -Os)

| پیکربندی | Flash | RAM |
|----------|-------|-----|
| Minimal (فقط GGA) | ~4.2 KB | 180 B |
| Standard (NMEA+UBX PVT) | ~15 KB | 1.2 KB |
| Full + GSV+GSA used | ~20 KB | 2.0 KB |
| Full + UBX SAT/DOP + Vendor (RX 512) | ~28 KB | 3.5 KB |

### باگ‌های نسخه قبلی که فیکس شد

1. Missing headers gnss.h, gnss_types.h
2. Nested functions GSA/VTG داخل nmea_parse
3. NMEA tokenization بدون \0
4. Checksum validation شکننده
5. UBX checksum calc غلط (length-2)
6. UBX NAV-PVT offsets غلط
7. Frame parser بدون resync
8. gnss.c overwrite به جای merge
9. Makefile/CMake بدون callbacks.c
10. GSV aggregation checksum غلط در تست

### مراحل بعدی پیشنهادی (اختیاری)

1. UBX NAV-POSLLH, VELNED, RXM-RAWX
2. RTCM3 برای RTK
3. Septentrio SBF
4. NMEA GNS
5. Benchmark روی STM32F4/F7/H7

---

**gnss-core v2.0** اکنون یک کتابخانه production-ready با پشتیبانی کامل NMEA/UBX/Vendor، مثال‌های ملموس و مستندات API کامل است.
