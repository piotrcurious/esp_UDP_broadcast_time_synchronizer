CC=g++
CFLAGS=-I./mock_arduino -Wall -DTEST_V8
LDFLAGS=-lm

all: test_v8

test_v8: mock_arduino/test_v8.cpp esp_udp_sync_v8.cpp mock_arduino/Arduino.cpp
	$(CC) $(CFLAGS) mock_arduino/test_v8.cpp -o test_v8 $(LDFLAGS)

clean:
	rm -f test_v8
