# ماتریس پشتیبانی پروتکل‌ها v2.0

| پروتکل | پیام | وضعیت | توضیحات |
|--------|------|--------|---------|
| NMEA | GGA | کامل | موقعیت + ارتفاع + کیفیت فیکس + hdop + sats |
| NMEA | RMC | کامل | موقعیت + سرعت + جهت + تاریخ/زمان + status |
| NMEA | GSV | کامل + Aggregation | اطلاعات ماهواره (چندبخشی) + gnss_id از Talker |
| NMEA | GSA | کامل + used flag | DOP + ماهواره‌های فعال + **used_in_solution flag** (جدید) |
| NMEA | VTG | کامل | سرعت و جهت زمینی (knots و km/h) |
| NMEA | GLL | کامل | موقعیت جغرافیایی (lat/lon) |
| NMEA | ZDA | کامل | زمان و تاریخ UTC |
| NMEA | TXT | کامل | متن اطلاعاتی (ANTSTATUS etc) |
| UBX | NAV-PVT | کامل | موقعیت، سرعت، زمان، فیکس، pDOP، hMSL+ellipsoid |
| UBX | NAV-SAT | کامل (جدید) | اطلاعات ماهواره کامل (gnssId, svId, cno, elev, azim, used flag) |
| UBX | NAV-DOP | کامل (جدید) | DOP کامل (gDOP,pDOP,tDOP,vDOP,hDOP,nDOP,eDOP) |
| UBX | NAV-TIMEUTC | پایه | زمان UTC |
| Vendor | Quectel PQGSV/PQGSA | کامل | به عنوان GSV/GSA استاندارد هندل می‌شود |
| Vendor | Quectel PQGNS | کامل (جدید) | GNSS fix data (ترکیب GGA) |
| Vendor | Quectel PQTXT/PQTM | کامل | متن و نسخه |
| Vendor | Trimble PTNL,AVR | کامل (جدید) | Attitude (yaw, tilt, roll) → course |
| Vendor | Trimble PTNL,GGK | کامل (جدید) | RTK position با rtk_fix_type |
| Vendor | Trimble PTNL,BPQ/VGK/GGA | کامل | Base quality, Vector, GGA |
| Vendor | MTK PMTK | کامل (جدید) | System messages |
| Vendor | Custom | Framework | کاربر می‌تواند parser سفارشی ثبت کند |

## جزئیات GSA used flag (جدید)

- **ورودی:** GSA فیلد 3-14: لیست 12 PRN استفاده‌شده در solution
- **خروجی:** `gnss_satellite_t.used_in_solution` در GSV و NAV-SAT
- **منطق:** 
  - GSA می‌آید → `g_gsa_used_prns[]` ذخیره می‌شود
  - اگر GSV buffer موجود باشد → `apply_gsa_used_flags()` و آپدیت هسته
  - GSV complete می‌شود → `apply_gsa_used_flags()` قبل از `set_satellites`
  - ترتیب GSA/GSV مهم نیست

## جزئیات UBX NAV-SAT (جدید)

- **Class/ID:** 0x01 0x35
- **Payload:** 8 + 12*numSvs
- **هر ماهواره 12 بایت:**
  - 0: gnssId (0=GPS,1=SBAS,2=Galileo,3=BeiDou,4=IMES,5=QZSS,6=GLONASS)
  - 1: svId (PRN)
  - 2: cno (SNR dBHz)
  - 3: elev (I1, -90..90)
  - 4-5: azim (I2, 0..360)
  - 6-7: prRes (I2)
  - 8-11: flags (U4) - bit 3 = svUsed
- **خروجی:** `gnss_satellite_t` با used flag

## جزئیات UBX NAV-DOP (جدید)

- **Class/ID:** 0x01 0x04
- **Payload:** 18 بایت
- **فیلدها:** gDOP,pDOP,tDOP,vDOP,hDOP,nDOP,eDOP (U2, scale 0.01)
- **خروجی:** merge به `gnss_solution_t` (gdop,pdop,tdop,vdop,hdop,ndop,edop)

## جزئیات Vendor Extensions (جدید)

### Quectel
| جمله | فرمت | خروجی |
|------|-------|-------|
| PQGSV | مشابه GSV | satellite |
| PQGSA | مشابه GSA | DOP + used |
| PQGNS | time,lat,N,lon,E,fix,sats,hdop,alt,... | solution |
| PQTXT | W,01,01,01,text | log |
| PQTMVER | version | log |

### Trimble
| جمله | فرمت | خروجی |
|------|-------|-------|
| PTNL,AVR | time,yaw,Yaw,tilt,Tilt,roll,Roll,range,... | course_deg |
| PTNL,GGK | time,date,lat,N,lon,E,fix,sats,dop,alt,M | solution + rtk_fix_type |
| PTNL,BPQ | ... | log |
| PTNL,VGK | ... | log |

### MTK
| جمله | خروجی |
|------|-------|
| PMTK010,001 | system startup |
| PMTK011,text | text |

## وضعیت فعلی

- **NMEA:** GGA,RMC,GSV,GSA (با used),VTG,GLL,ZDA,TXT کامل
- **UBX:** NAV-PVT, NAV-SAT, NAV-DOP کامل
- **Vendor:** Quectel + Trimble + MTK + Custom Framework کامل
- **مثال‌ها:** baremetal, simple_buffer, simple_stream, callback, vendor, ubx, stm32

## مراحل بعدی پیشنهادی (اختیاری)

1. UBX NAV-POSLLH, NAV-VELNED
2. UBX RXM-RAWX برای raw measurements
3. Septentrio SBF
4. RTCM3 parsing برای RTK
5. NMEA GNS (GNSS fix data)
