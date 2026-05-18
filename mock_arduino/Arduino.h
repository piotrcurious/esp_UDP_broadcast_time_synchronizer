#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <string>
#include <vector>
#include <algorithm>

typedef uint8_t byte;

#define WIFI_STA 1
#define WL_CONNECTED 3
#define WIFI_NONE_SLEEP 0

struct IPAddress {
    uint8_t bytes[4];
    IPAddress() { bytes[0]=bytes[1]=bytes[2]=bytes[3]=0; }
    IPAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
        bytes[0]=a; bytes[1]=b; bytes[2]=c; bytes[3]=d;
    }
    IPAddress(const char* str) {
        sscanf(str, "%hhu.%hhu.%hhu.%hhu", &bytes[0], &bytes[1], &bytes[2], &bytes[3]);
    }
    bool operator==(const IPAddress& other) const {
        return memcmp(bytes, other.bytes, 4) == 0;
    }
};

class MockSerial {
public:
    void begin(int baud) {}
    void print(const char* s) { printf("%s", s); }
    void print(double d) { printf("%f", d); }
    void print(int i) { printf("%d", i); }
    void println(const char* s = "") { printf("%s\n", s); }
    template<typename... Args>
    void printf(const char* format, Args... args) {
        ::printf(format, args...);
    }
};

extern MockSerial Serial;

uint32_t millis();
uint32_t micros();
void delay(uint32_t ms);

// Mock WiFi
class MockWiFi {
public:
    void mode(int m) {}
    void begin(const char* ssid, const char* pass) {}
    int status() { return WL_CONNECTED; }
    IPAddress localIP() { return IPAddress(192, 168, 1, 100 + nodeId); }
    void macAddress(uint8_t* mac) {
        mac[0]=0x00; mac[1]=0x11; mac[2]=0x22; mac[3]=0x33; mac[4]=0x44; mac[5]=nodeId;
    }
    void setSleepMode(int m) {}
    uint8_t nodeId;
};

extern MockWiFi WiFi;

class WiFiUDP {
public:
    std::vector<uint8_t> currentPacket;
    IPAddress currentRemoteIP;
    int beginMulticast(IPAddress local, IPAddress multi, uint16_t port) { return 1; }
    int beginPacket(IPAddress addr, uint16_t port) {
        currentRemoteIP = addr;
        currentPacket.clear();
        return 1;
    }
    size_t write(const uint8_t* buffer, size_t size) {
        currentPacket.insert(currentPacket.end(), buffer, buffer + size);
        return size;
    }
    virtual int endPacket();
    virtual int parsePacket();
    virtual int read(uint8_t* buffer, size_t size);
    virtual IPAddress remoteIP();
};

#endif
