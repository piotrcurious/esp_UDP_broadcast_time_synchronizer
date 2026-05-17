#include "Arduino.h"
#include <queue>
#include <map>

// Implementations moved to test_v8.cpp or kept here if not redefined
#ifndef TEST_V8
MockSerial Serial;
MockWiFi WiFi;

static uint64_t g_currentRealTimeUs = 0;
static double g_nodeDrifts[256];
static double g_nodePhaseOffsets[256];

uint32_t millis() { return (uint32_t)(g_currentRealTimeUs / 1000); }
uint32_t micros() {
    uint8_t id = WiFi.nodeId;
    return (uint32_t)(g_currentRealTimeUs * g_nodeDrifts[id] + g_nodePhaseOffsets[id]);
}
void delay(uint32_t ms) { g_currentRealTimeUs += (uint64_t)ms * 1000; }

int WiFiUDP::endPacket() { return 0; }
int WiFiUDP::parsePacket() { return 0; }
int WiFiUDP::read(uint8_t* buffer, size_t size) { return 0; }
IPAddress WiFiUDP::remoteIP() { return IPAddress(); }
#endif
