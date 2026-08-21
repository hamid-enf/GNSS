# gnss/Makefile - نسخه توسعه‌یافته
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -Iinclude -Isrc -O2 -g
LDFLAGS = -lm

SRC = src/core/gnss.c src/core/gnss_callbacks.c src/parser/frame.c src/parser/nmea/nmea_parser.c src/parser/ubx/ubx_parser.c src/parser/vendor/vendor_parser.c

CFLAGS += -DGNSS_ENABLE_NMEA=1 -DGNSS_ENABLE_UBX=1 -DGNSS_ENABLE_GSV=1 -DGNSS_ENABLE_GSA=1 -DGNSS_ENABLE_VTG=1 -DGNSS_ENABLE_GGA=1 -DGNSS_ENABLE_RMC=1 -DGNSS_ENABLE_GLL=1 -DGNSS_ENABLE_ZDA=1 -DGNSS_ENABLE_UBX_NAV_PVT=1 -DGNSS_ENABLE_UBX_NAV_SAT=1 -DGNSS_ENABLE_UBX_NAV_DOP=1 -DGNSS_ENABLE_VENDOR_EXT=1 -DGNSS_ENABLE_VENDOR_QUECTEL=1 -DGNSS_ENABLE_VENDOR_TRIMBLE=1 -DGNSS_ENABLE_VENDOR_MTK=1

TESTS = test_nmea test_ubx test_frame test_all test_gsa_used test_vendor
EXAMPLES = simple_buffer simple_stream callback_example vendor_example ubx_example baremetal_example live_demo

all: $(TESTS) $(EXAMPLES)

test_nmea: tests/test_nmea.c $(SRC)
	$(CC) $(CFLAGS) -o $@ tests/test_nmea.c $(SRC) $(LDFLAGS)

test_ubx: tests/test_ubx.c $(SRC)
	$(CC) $(CFLAGS) -o $@ tests/test_ubx.c $(SRC) $(LDFLAGS)

test_frame: tests/test_frame.c $(SRC)
	$(CC) $(CFLAGS) -o $@ tests/test_frame.c $(SRC) $(LDFLAGS)

test_all: tests/test_all.c $(SRC)
	$(CC) $(CFLAGS) -o $@ tests/test_all.c $(SRC) $(LDFLAGS)

test_gsa_used: tests/test_gsa_used.c $(SRC)
	$(CC) $(CFLAGS) -o $@ tests/test_gsa_used.c $(SRC) $(LDFLAGS)

test_vendor: tests/test_vendor.c $(SRC)
	$(CC) $(CFLAGS) -o $@ tests/test_vendor.c $(SRC) $(LDFLAGS)

simple_buffer: examples/simple/simple_buffer.c $(SRC)
	$(CC) $(CFLAGS) -o $@ examples/simple/simple_buffer.c $(SRC) $(LDFLAGS)

simple_stream: examples/simple/simple_stream.c $(SRC)
	$(CC) $(CFLAGS) -o $@ examples/simple/simple_stream.c $(SRC) $(LDFLAGS)

callback_example: examples/simple/callback_example.c $(SRC)
	$(CC) $(CFLAGS) -o $@ examples/simple/callback_example.c $(SRC) $(LDFLAGS)

vendor_example: examples/vendor/vendor_example.c $(SRC)
	$(CC) $(CFLAGS) -o $@ examples/vendor/vendor_example.c $(SRC) $(LDFLAGS)

ubx_example: examples/ubx/ubx_example.c $(SRC)
	$(CC) $(CFLAGS) -o $@ examples/ubx/ubx_example.c $(SRC) $(LDFLAGS)

baremetal_example: examples/baremetal/main.c $(SRC)
	$(CC) $(CFLAGS) -o $@ examples/baremetal/main.c $(SRC) $(LDFLAGS)

live_demo: examples/live_demo.c $(SRC)
	$(CC) $(CFLAGS) -o $@ examples/live_demo.c $(SRC) $(LDFLAGS)

clean:
	rm -f $(TESTS) $(EXAMPLES) *.o
	rm -rf build/

.PHONY: all clean
