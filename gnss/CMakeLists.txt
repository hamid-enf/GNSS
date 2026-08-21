# gnss/CMakeLists.txt - نسخه توسعه‌یافته با Vendor + UBX extended
cmake_minimum_required(VERSION 3.16)
project(gnss-core C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ======================== گزینه‌ها ========================
option(GNSS_ENABLE_NMEA "Enable NMEA parser" ON)
option(GNSS_ENABLE_UBX  "Enable UBX parser"  ON)
option(GNSS_ENABLE_GSV  "Enable GSV aggregation" ON)
option(GNSS_ENABLE_GSA  "Enable GSA" ON)
option(GNSS_ENABLE_VTG  "Enable VTG" ON)
option(GNSS_ENABLE_GGA  "Enable GGA" ON)
option(GNSS_ENABLE_RMC  "Enable RMC" ON)
option(GNSS_ENABLE_VENDOR "Enable Vendor Extensions" ON)

# ======================== منابع ========================
set(SOURCES
    src/core/gnss.c
    src/core/gnss_callbacks.c
    src/parser/frame.c
    src/parser/nmea/nmea_parser.c
    src/parser/ubx/ubx_parser.c
)

if(GNSS_ENABLE_VENDOR)
    list(APPEND SOURCES src/parser/vendor/vendor_parser.c)
endif()

add_library(gnss STATIC ${SOURCES})

target_include_directories(gnss PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_options(gnss PRIVATE -Wall -Wextra -Wpedantic)

if(GNSS_ENABLE_NMEA)
    target_compile_definitions(gnss PUBLIC GNSS_ENABLE_NMEA=1)
endif()
if(GNSS_ENABLE_UBX)
    target_compile_definitions(gnss PUBLIC GNSS_ENABLE_UBX=1 GNSS_ENABLE_UBX_NAV_PVT=1 GNSS_ENABLE_UBX_NAV_SAT=1 GNSS_ENABLE_UBX_NAV_DOP=1)
endif()
if(GNSS_ENABLE_GSV)
    target_compile_definitions(gnss PUBLIC GNSS_ENABLE_GSV=1)
endif()
if(GNSS_ENABLE_GSA)
    target_compile_definitions(gnss PUBLIC GNSS_ENABLE_GSA=1)
endif()
if(GNSS_ENABLE_VENDOR)
    target_compile_definitions(gnss PUBLIC GNSS_ENABLE_VENDOR_EXT=1 GNSS_ENABLE_VENDOR_QUECTEL=1 GNSS_ENABLE_VENDOR_TRIMBLE=1 GNSS_ENABLE_VENDOR_MTK=1)
endif()

# ======================== تست‌ها ========================
set(TESTS test_nmea test_ubx test_frame test_all test_gsa_used test_vendor)

foreach(TEST_NAME ${TESTS})
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/${TEST_NAME}.c)
        add_executable(${TEST_NAME} tests/${TEST_NAME}.c)
        target_link_libraries(${TEST_NAME} gnss m)
        target_include_directories(${TEST_NAME} PRIVATE include src)
    endif()
endforeach()

# Examples
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples/simple/simple_buffer.c)
    add_executable(simple_buffer examples/simple/simple_buffer.c)
    target_link_libraries(simple_buffer gnss m)
    target_include_directories(simple_buffer PRIVATE include src)
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples/baremetal/main.c)
    add_executable(baremetal_example examples/baremetal/main.c)
    target_link_libraries(baremetal_example gnss m)
    target_include_directories(baremetal_example PRIVATE include src)
endif()
