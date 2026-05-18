#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <vector>

#define TEST_V11
#include "../esp_udp_sync_v8.cpp"

float g_packetLossRate = 0.10f;
static uint64_t g_realTimeUs = 0;
static double g_drifts[256];
static double g_offsets[256];

uint32_t millis() { return (uint32_t)(g_realTimeUs / 1000); }
uint32_t micros() {
    uint8_t id = WiFi.nodeId;
    double val = (double)g_realTimeUs * g_drifts[id] + g_offsets[id];
    return (uint32_t)fmod(val, 4294967296.0);
}
void delay(uint32_t ms) { g_realTimeUs += (uint64_t)ms * 1000; }

MockSerial Serial;
MockWiFi WiFi;

struct Packet {
    uint64_t deliveryTime;
    int srcId, dstId;
    std::vector<uint8_t> data;
    IPAddress remoteIP;
};

std::vector<Packet> g_packetWire;
std::vector<Packet> g_nodeQueues[256];

int WiFiUDP::endPacket() {
    if (((float)rand() / (float)RAND_MAX) < g_packetLossRate) return 1;
    Packet p; p.srcId = WiFi.nodeId;
    if (currentRemoteIP == IPAddress(224, 0, 0, 1)) p.dstId = -1;
    else p.dstId = currentRemoteIP.bytes[3] - 100;
    p.data = currentPacket; p.remoteIP = IPAddress(192, 168, 1, 100 + WiFi.nodeId);
    uint64_t latency = 2000 + (rand() % 10000);
    if (p.dstId == -1) {
        uint64_t BI = 102400;
        latency = ((g_realTimeUs + latency) / BI + 1) * BI - g_realTimeUs;
    }
    p.deliveryTime = g_realTimeUs + latency;
    g_packetWire.push_back(p);
    return 1;
}

int WiFiUDP::parsePacket() {
    if (g_nodeQueues[WiFi.nodeId].empty()) return 0;
    return g_nodeQueues[WiFi.nodeId].front().data.size();
}
int WiFiUDP::read(uint8_t* buffer, size_t size) {
    Packet p = g_nodeQueues[WiFi.nodeId].front();
    g_nodeQueues[WiFi.nodeId].erase(g_nodeQueues[WiFi.nodeId].begin());
    size_t toRead = std::min(size, p.data.size());
    memcpy(buffer, p.data.data(), toRead);
    return toRead;
}
IPAddress WiFiUDP::remoteIP() { return IPAddress(192, 168, 1, 100); }

void run_simulation(int numNodes, uint64_t durationUs, bool collisionTest = false) {
    g_packetWire.clear();
    for (int i=0; i<256; i++) g_nodeQueues[i].clear();
    g_realTimeUs = 0;

    ClockSynchronizer* nodes[MAX_NODES];
    for (int i = 0; i < numNodes; i++) {
        nodes[i] = new ClockSynchronizer();
        g_drifts[i] = 1.0 + ((rand() % 1000) - 500) / 1000000.0;
        g_offsets[i] = (double)(rand() % 1000000);
        WiFi.nodeId = i;
        nodes[i]->begin(i);
        if (collisionTest) nodes[i]->preferredSlot = 0;
        nodes[i]->localPhase = 0; nodes[i]->localDrift = 0; nodes[i]->localQuality = 0;
    }

    uint64_t targetUs = durationUs;
    while (g_realTimeUs < targetUs) {
        g_realTimeUs += 1000;
        uint64_t t = g_realTimeUs;
        for (auto it = g_packetWire.begin(); it != g_packetWire.end(); ) {
            if (t >= it->deliveryTime) {
                if (it->dstId == -1) { for (int j=0; j<numNodes; j++) if (j != it->srcId) g_nodeQueues[j].push_back(*it); }
                else if (it->dstId >= 0 && it->dstId < numNodes) g_nodeQueues[it->dstId].push_back(*it);
                it = g_packetWire.erase(it);
            } else ++it;
        }
        for (int i = 0; i < numNodes; i++) {
            WiFi.nodeId = i;
            nodes[i]->update();
        }
    }

    uint64_t refTime = nodes[0]->getNetworkMicros();
    printf("\nResults (%d nodes, %0.1fs, Collisions=%d):\n", numNodes, (double)durationUs/1000000.0, collisionTest);
    for (int i = 0; i < numNodes; i++) {
        int64_t diff = (int64_t)nodes[i]->getNetworkMicros() - (int64_t)refTime;
        while (diff > 2000000000LL) diff -= 4294967296LL;
        while (diff < -2000000000LL) diff += 4294967296LL;

        // Final sanity: check if slots converged
        bool slotUnique = true;
        for (int j=0; j<numNodes; j++) if (i != j && nodes[i]->preferredSlot == nodes[j]->preferredSlot) slotUnique = false;

        printf("Node %2d: Offset=%10lld us, Q=%.2f, Slot=%2d %s, Anchor=%d\n",
               i, (long long)diff, (double)nodes[i]->localQuality, (int)nodes[i]->preferredSlot, slotUnique ? "OK" : "DUP", (int)nodes[i]->netAnchorId);
        delete nodes[i];
    }
}

int main() {
    srand(42);
    run_simulation(16, 600000000, true); // 600s for 16 nodes
    return 0;
}
