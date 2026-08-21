# معماری gnss-core v2.0

## نمای کلی

```
Application
    ↓
High-Level API (gnss.h) - Level 1/2 + Field API + Vendor API
    ↓
Normalized Data Model (gnss_solution_t + Satellite Registry + Vendor Msg)
    ↓
Message Dispatcher (NMEA / UBX / Vendor)
    ↓
Protocol Parsers
    ├── NMEA Parser (GGA,RMC,GSV+GSA used flag,VTG,GLL,ZDA,TXT)
    ├── UBX Parser (NAV-PVT, NAV-SAT, NAV-DOP, TIMEUTC)
    └── Vendor Parser (Quectel, Trimble, MTK, Custom)
    ↓
Frame Parser (Byte Stream + Protocol Detection + Resync)
    ↓
User Buffer / UART / DMA / SPI / File
```

## اصول طراحی

1. **Zero Dynamic Allocation** — تمام بافرها static یا caller-owned. هیچ malloc.
2. **Protocol-Centric** — یک پارسر NMEA و UBX برای اکثر گیرنده‌ها کافی است.
3. **Compile-Time Configuration** — هر قابلیت استفاده‌نشده از firmware حذف می‌شود.
4. **Deterministic** — بدون recursion، بدون heap، قابل پیش‌بینی برای RTOS.
5. **Merge Logic** — GGA موقعیت، RMC سرعت، GSA DOP+used flag، VTG course، UBX DOP/SAT همه در یک solution ادغام می‌شوند.

## مدل داده

- **Canonical Units**: درجه اعشاری، متر، m/s، درجه
- **gnss_solution_t**: موقعیت، سرعت، DOP کامل (hdop,vdop,pdop,gdop,tdop,ndop,edop)، زمان، RTK info، vendor id
- **gnss_satellite_t**: prn, elevation, azimuth, snr, gnss_id, used_in_solution (از GSA و UBX NAV-SAT), healthy, quality
- **gnss_vendor_msg_t**: آخرین پیام vendor (talker, type, subtype, raw)

## پارسر فریم (Frame Parser)

- State Machine سبک با 10 حالت
- تشخیص پروتکل:
  - `$` → NMEA
  - `0xB5 0x62` → UBX
- Resync هوشمند: اگر sync2 شکست خورد، `$` جدید را چک می‌کند
- مناسب UART Interrupt و DMA Idle
- RX buffer 512 برای NAV-SAT بزرگ (64 sat *12 +8)

## NMEA Parser v2

- Talker ID مستقل: GP, GN, GA, GL, GB, PQ, PT, PM...
- Checksum validation با جستجوی `*`
- Tokenization امن با کپی محلی و `\0`
- تبدیل `ddmm.mmmm` به درجه اعشاری
- **GSV Aggregation**: چندبخشی با expected_msgs
- **GSA used flag (جدید)**: 
  - GSA لیست PRN های used (12 عدد) را می‌دهد
  - `g_gsa_used_prns[]` ذخیره می‌شود
  - هنگام GSV complete و هنگام GSA هر دو، `apply_gsa_used_flags()` صدا زده می‌شود
  - ترتیب GSA/GSV مهم نیست
- **GLL, ZDA, TXT**: اضافه شد

## UBX Parser v2

- Fletcher checksum درست (از class تا end payload)
- **NAV-PVT (0x01 0x07)**: lon/lat/hMSL/height/gSpeed/headMot/pDOP + زمان + fixType + numSV
- **NAV-SAT (0x01 0x35) - جدید**:
  - Header 8 بایت + 12*numSvs
  - هر ماهواره: gnssId, svId, cno, elev, azim, prRes, flags
  - flags bit 3 = svUsed → used_in_solution
  - مستقیم به `gnss_internal_set_satellites`
- **NAV-DOP (0x01 0x04) - جدید**:
  - gDOP,pDOP,tDOP,vDOP,hDOP,nDOP,eDOP (U2 scale 0.01)
  - به solution merge می‌شود
- **NAV-TIMEUTC (0x01 0x21)**: زمان UTC

## Vendor Parser (جدید)

- Framework: `gnss_vendor_register_parser()` برای custom parser
- Built-in:
  - **Quectel**: PQGSV,PQGSA (به عنوان GSV/GSA), PQGNS (fix), PQTXT, PQTMVER, MTK
  - **Trimble**: PTNL,AVR (yaw→course), PTNL,GGK (RTK position با rtk_fix_type), PTNL,BPQ,VGK,GGA
  - **MTK**: PMTK010 etc
- Detection: بر اساس talker (`PQ`, `PT`, `PM`) و full_id
- Last vendor message cache: `gnss_get_last_vendor_msg()`
- Stats: vendor_messages شمارش

## هسته (gnss.c) v2

- **Merge هوشمند**:
  - valid_position → lat/lon/alt/fix/sats/hdop/time
  - valid_velocity → speed/course
  - valid_dop → hdop/vdop/pdop/gdop/tdop/ndop/edop
  - GSA → fix_type + DOP + used PRNs
  - Vendor → last_vendor + rtk
- **Satellite Registry**: 64 ماهواره، GSV و NAV-SAT هر دو استفاده می‌کنند
- **Callback**: SOLUTION, SATELLITES, ERROR
- **Stats**: messages_received, parsed, checksum_errors, frame_errors, overflow, nmea, ubx, vendor, gsv, gsa

## گسترش‌پذیری

- `gnss_vendor_register_parser()` - کاربر می‌تواند پروتکل اختصاصی اضافه کند
- `GNSS_VENDOR_MAX_PARSERS=8`
- مثال custom parser در `examples/vendor/vendor_example.c`

## مصرف حافظه (تقریبی با GCC -Os)

| پیکربندی | Flash | RAM |
|----------|-------|-----|
| Minimal (فقط GGA) | ~4.2 KB | ~180 B |
| Standard (NMEA+UBX PVT) | ~15 KB | ~1.2 KB |
| Full + GSV+GSA used | ~20 KB | ~2.0 KB |
| Full + UBX SAT/DOP + Vendor | ~28 KB | ~3.5 KB (RX 512) |

## دیاگرام جریان GSA+GSV

```
GSV msg1 -> g_gsv_buffer[0..3], count=4
GSV msg2 -> g_gsv_buffer[4..7], count=8
GSA -> g_gsa_used_prns=[02,04,05,09], count=4, gsa_has_used=true
       -> apply_gsa_used_flags() روی g_gsv_buffer
       -> used_in_solution برای PRN های مطابق true
GSV msg3 (آخر) -> g_gsv_count=12, expected=3
       -> apply_gsa_used_flags() دوباره
       -> gnss_internal_set_satellites(buffer, 12)
       -> callback SATELLITES
```

## نتیجه‌گیری

معماری v2 تعادل بین سادگی، عملکرد و قابلیت گسترش را با اضافه کردن Vendor Framework و UBX Extended و GSA used flag حفظ کرده است.
