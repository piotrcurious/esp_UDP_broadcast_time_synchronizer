#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <vector>
#include <map>

uint32_t lastMicrosLow = 0;
uint32_t microsHigh = 0;

float g_packetLossRate = 0.10f; // Stress: 10% packet loss

#define private public
#include "../esp_udp_sync_v8.cpp"
#undef private

struct NodeState {
    uint8_t myId;
    uint8_t netAnchorId;
    std::vector<Peer> peers;
    uint32_t txSeq;
    uint32_t lastPollFrame;
    double localPhase;
    double localDrift;
    float localQuality;
    uint64_t lastSlewUs;
    uint32_t lastKfTime;
    uint32_t lastDiag;
    uint32_t lastMicrosLow;
    uint32_t microsHigh;
    WiFiUDP udp;
};

static NodeState* g_nodes[256];
static int g_activeNode = 0;

void swapToNode(int id) {
    if (g_nodes[g_activeNode]) {
        g_nodes[g_activeNode]->peers = ::peers;
        g_nodes[g_activeNode]->txSeq = ::txSeq;
        g_nodes[g_activeNode]->lastPollFrame = ::lastPollFrame;
        g_nodes[g_activeNode]->localPhase = ::localPhase;
        g_nodes[g_activeNode]->localDrift = ::localDrift;
        g_nodes[g_activeNode]->localQuality = ::localQuality;
        g_nodes[g_activeNode]->lastSlewUs = ::lastSlewUs;
        g_nodes[g_activeNode]->lastKfTime = ::lastKfTime;
        g_nodes[g_activeNode]->lastDiag = ::lastDiag;
        g_nodes[g_activeNode]->udp = ::udp;
        g_nodes[g_activeNode]->myId = ::myId;
        g_nodes[g_activeNode]->netAnchorId = ::netAnchorId;
        g_nodes[g_activeNode]->lastMicrosLow = ::lastMicrosLow;
        g_nodes[g_activeNode]->microsHigh = ::microsHigh;
    }

    g_activeNode = id;
    WiFi.nodeId = id;

    ::peers = g_nodes[id]->peers;
    ::txSeq = g_nodes[id]->txSeq;
    ::lastPollFrame = g_nodes[id]->lastPollFrame;
    ::localPhase = g_nodes[id]->localPhase;
    ::localDrift = g_nodes[id]->localDrift;
    ::localQuality = g_nodes[id]->localQuality;
    ::lastSlewUs = g_nodes[id]->lastSlewUs;
    ::lastKfTime = g_nodes[id]->lastKfTime;
    ::lastDiag = g_nodes[id]->lastDiag;
    ::udp = g_nodes[id]->udp;
    ::myId = g_nodes[id]->myId;
    ::netAnchorId = g_nodes[id]->netAnchorId;
    ::lastMicrosLow = g_nodes[id]->lastMicrosLow;
    ::microsHigh = g_nodes[id]->microsHigh;
}

static uint64_t g_realTimeUs = 0;
static double g_drifts[256];
static double g_offsets[256];

uint32_t millis() { return (uint32_t)(g_realTimeUs / 1000); }
uint32_t micros() {
    uint8_t id = WiFi.nodeId;
    return (uint32_t)(g_realTimeUs * g_drifts[id] + g_offsets[id]);
}
void delay(uint32_t ms) { g_realTimeUs += (uint64_t)ms * 1000; }

MockSerial Serial;
MockWiFi WiFi;

struct Packet {
    uint64_t deliveryTime;
    int srcId;
    int dstId;
    std::vector<uint8_t> data;
    IPAddress remoteIP;
};

std::vector<Packet> g_packetWire;

int WiFiUDP::endPacket() {
    if (((float)rand() / (float)RAND_MAX) < g_packetLossRate) return 1;
    Packet p;
    p.srcId = g_activeNode;
    if (currentRemoteIP == IPAddress(224, 0, 0, 1)) p.dstId = -1;
    else p.dstId = currentRemoteIP.bytes[3] - 100;
    p.data = currentPacket;
    p.remoteIP = IPAddress(192, 168, 1, 100 + g_activeNode);
    uint64_t latency = 2000 + (rand() % 3000);
    if (p.dstId == -1) {
        uint64_t BI = 102400;
        uint64_t arrival = g_realTimeUs + latency;
        uint64_t nextBeacon = ((arrival / BI) + 1) * BI;
        latency = nextBeacon - g_realTimeUs;
    }
    p.deliveryTime = g_realTimeUs + latency;
    g_packetWire.push_back(p);
    return 1;
}

std::vector<Packet> g_nodeQueues[256];
Packet g_currentReadPkt;
int WiFiUDP::parsePacket() {
    if (g_nodeQueues[g_activeNode].empty()) return 0;
    g_currentReadPkt = g_nodeQueues[g_activeNode].front();
    g_nodeQueues[g_activeNode].erase(g_nodeQueues[g_activeNode].begin());
    return g_currentReadPkt.data.size();
}
int WiFiUDP::read(uint8_t* buffer, size_t size) {
    size_t toRead = std::min(size, g_currentReadPkt.data.size());
    memcpy(buffer, g_currentReadPkt.data.data(), toRead);
    return toRead;
}
IPAddress WiFiUDP::remoteIP() { return g_currentReadPkt.remoteIP; }

void run_sim(int seconds) {
    uint64_t targetTime = g_realTimeUs + (uint64_t)seconds * 1000000;
    while (g_realTimeUs < targetTime) {
        g_realTimeUs += 1000;
        uint64_t now = g_realTimeUs;
        for (auto it = g_packetWire.begin(); it != g_packetWire.end(); ) {
            if (now >= it->deliveryTime) {
                if (it->dstId == -1) {
                    for (int i = 0; i < 4; i++) {
                        if (i != it->srcId) g_nodeQueues[i].push_back(*it);
                    }
                } else if (it->dstId >= 0 && it->dstId < 4) {
                    g_nodeQueues[it->dstId].push_back(*it);
                }
                it = g_packetWire.erase(it);
            } else ++it;
        }
        for (int i = 0; i < 4; i++) {
            swapToNode(i);
            loop();
        }
    }
}

int main() {
    srand(42);
    int numNodes = 4;
    for (int i = 0; i < numNodes; i++) {
        g_nodes[i] = new NodeState();
        g_nodes[i]->netAnchorId = 0xFF;
        g_drifts[i] = 1.0 + ((rand() % 200) - 100) / 1000000.0;
        g_offsets[i] = (double)(rand() % 10000000);
        swapToNode(i);
        setup();
        localPhase = 0; localDrift = 0; localQuality = 0;
    }
    printf("--- Phase 1: Convergence (300s, 10%% Loss) ---\n");
    run_sim(300);
    swapToNode(0);
    uint64_t refTime = getNetworkMicros();
    printf("After 300s:\n");
    for (int i = 0; i < numNodes; i++) {
        swapToNode(i);
        int64_t diff = (int64_t)getNetworkMicros() - (int64_t)refTime;
        printf("Node %d: Offset=%lld us, Quality=%.2f, Drift=%.6f, Anchor=%d\n",
               i, (long long)diff, (double)localQuality, localDrift, (int)netAnchorId);
    }
    return 0;
}
