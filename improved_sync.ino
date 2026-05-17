/*
 * ============================================================
 * ESP-UDP-SYNC: High-Performance Distributed Clock Synchronization
 * ============================================================
 * Version 6.0 - Precision & Consensus Stability
 *
 * Key Features:
 * - NTP-style 4-timestamp exchange for RTT & Offset estimation.
 * - Per-peer 2D Kalman Filters for Phase & Drift tracking.
 * - Quality-weighted Consensus: Nodes trust well-synced peers more.
 * - Precision Clock Discipline: PI-controller for stable convergence.
 * - Collision Avoidance: TDMA with randomized slot jitter.
 * ============================================================
 */

#include <Arduino.h>
#if defined(ESP8266)
  #include <ESP8266WiFi.h>
#elif defined(ESP32)
  #include <WiFi.h>
#endif
#include <WiFiUdp.h>
#include <vector>
#include <algorithm>

// ── Configuration ─────────────────────────────────────────────
#ifndef WIFI_SSID
#define WIFI_SSID "YourSSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "YourPassword"
#endif

#define UDP_PORT 7777
#define MULTICAST_IP "224.0.0.1"
#define MAX_NODES 16

#define TDMA_SLOT_MS 60
#define SUPERFRAME_MS (TDMA_SLOT_MS * MAX_NODES)
#define TX_WINDOW_MS 5

// Kalman Filter constants - Stability-centric
#define KF_Q_PHASE 1.0
#define KF_Q_DRIFT 0.001
#define KF_R_BASE 2000.0

// ── Types ──────────────────────────────────────────────────────
enum PacketType : uint8_t { PKT_POLL = 1, PKT_REPLY = 2 };

#pragma pack(push, 1)
struct SyncPacket {
  uint8_t type;
  uint8_t srcId;
  uint8_t dstId;
  uint32_t seq;
  uint64_t t1;
  uint64_t t2;
  uint64_t t3;
  double advPhase;
  double advDrift;
  float advQuality;
};
#pragma pack(pop)

struct KalmanFilter {
  double phase;
  double drift;
  double P[2][2];
  uint64_t lastUpdateUs;

  KalmanFilter() {
    phase = 0; drift = 0; lastUpdateUs = 0;
    P[0][0] = 1e6; P[0][1] = 0;
    P[1][0] = 0; P[1][1] = 1e4;
  }

  void predict(uint64_t nowUs) {
    if (lastUpdateUs == 0) { lastUpdateUs = nowUs; return; }
    double dt = (double)(nowUs - lastUpdateUs) / 1000000.0;
    if (dt <= 0) return;

    phase += drift * dt;
    P[0][0] += dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + KF_Q_PHASE;
    P[0][1] += dt * P[1][1];
    P[1][0] += dt * P[1][1];
    P[1][1] += KF_Q_DRIFT;
    lastUpdateUs = nowUs;
  }

  void update(double z, double R) {
    double y = z - phase;
    double S = P[0][0] + R;
    double K0 = P[0][0] / S;
    double K1 = P[1][0] / S;
    phase += K0 * y;
    drift += K1 * y;
    double P00 = P[0][0]; double P01 = P[0][1];
    P[0][0] -= K0 * P00; P[0][1] -= K0 * P01;
    P[1][0] -= K1 * P00; P[1][1] -= K1 * P01;
  }

  float quality() const {
    double uncertainty = sqrt(abs(P[0][0])) + sqrt(abs(P[1][1])) * 50.0;
    return (float)(1.0 / (1.0 + uncertainty / 1000.0));
  }
};

struct Peer {
  uint8_t id;
  bool active;
  uint32_t lastHeard;
  KalmanFilter filter;
  double avgRtt;
  double advPhase;
  double advDrift;
  float advQuality;

  Peer(uint8_t _id) : id(_id), active(false), lastHeard(0), filter(), avgRtt(0), advPhase(0), advDrift(0), advQuality(0) {}
};

// ── Global State ──────────────────────────────────────────────
WiFiUDP udp;
std::vector<Peer> peers;
uint8_t myId = 0;
uint32_t txSeq = 0;
uint32_t lastPollFrame = 0xFFFFFFFF;
double localPhase = 0;
double localDrift = 0;
float localQuality = 0;
uint32_t lastKfTime = 0;

// ── Helpers ───────────────────────────────────────────────────
uint64_t getRawMicros() {
#if defined(ESP32)
  return (uint64_t)esp_timer_get_time();
#else
  return (uint64_t)micros();
#endif
}

uint64_t getNetworkMicros() {
  return (uint64_t)((int64_t)getRawMicros() - (int64_t)localPhase);
}

void sendPacket(SyncPacket& pkt, IPAddress addr) {
  udp.beginPacket(addr, UDP_PORT);
  udp.write((uint8_t*)&pkt, sizeof(SyncPacket));
  udp.endPacket();
}

// ── Implementation ────────────────────────────────────────────
void handlePoll(SyncPacket& pkt, uint64_t rxTime) {
  if (pkt.srcId == myId) return;
  auto it = std::find_if(peers.begin(), peers.end(), [&](const Peer& p){ return p.id == pkt.srcId; });
  if (it == peers.end()) {
    if (peers.size() < MAX_NODES) { peers.emplace_back(pkt.srcId); it = peers.end() - 1; }
    else return;
  }
  Peer& p = *it;
  p.active = true; p.lastHeard = millis(); p.advPhase = pkt.advPhase; p.advDrift = pkt.advDrift; p.advQuality = pkt.advQuality;

  SyncPacket reply;
  reply.type = PKT_REPLY; reply.srcId = myId; reply.dstId = pkt.srcId; reply.seq = pkt.seq; reply.t1 = pkt.t1; reply.t2 = rxTime; reply.t3 = getRawMicros();
  reply.advPhase = localPhase; reply.advDrift = localDrift; reply.advQuality = localQuality;
  sendPacket(reply, udp.remoteIP());
}

void handleReply(SyncPacket& pkt, uint64_t rxTime) {
  auto it = std::find_if(peers.begin(), peers.end(), [&](const Peer& p){ return p.id == pkt.srcId; });
  if (it == peers.end() || pkt.dstId != myId) return;
  Peer& p = *it;
  uint64_t nowUs = getRawMicros();
  int64_t T1 = (int64_t)pkt.t1; int64_t T2 = (int64_t)pkt.t2; int64_t T3 = (int64_t)pkt.t3; int64_t T4 = (int64_t)rxTime;
  double theta = (double)((T2 - T1) + (T3 - T4)) * 0.5; double rtt = (double)((T4 - T1) - (T3 - T2));
  if (rtt < 0 || rtt > 100000) return;
  p.filter.predict(nowUs); p.filter.update(p.advPhase - theta, KF_R_BASE + rtt);
  if (p.avgRtt == 0) p.avgRtt = rtt; else p.avgRtt = p.avgRtt * 0.9 + rtt * 0.1;
}

void setup() {
  Serial.begin(115200); WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  uint8_t mac[6]; WiFi.macAddress(mac); myId = mac[5];
#if defined(ESP8266)
  udp.beginMulticast(WiFi.localIP(), IPAddress(224, 0, 0, 1), UDP_PORT);
#else
  udp.beginMulticast(IPAddress(224, 0, 0, 1), UDP_PORT);
#endif
  lastKfTime = millis();
}

void loop() {
  uint32_t now = millis(); uint64_t netUs = getNetworkMicros();
  uint32_t frame = (uint32_t)(netUs / (SUPERFRAME_MS * 1000ULL));
  uint32_t myJitter = (myId * 13) % (TDMA_SLOT_MS / 2);
  uint64_t mySlotStart = (uint64_t)frame * SUPERFRAME_MS * 1000ULL + (uint64_t)(myId % MAX_NODES) * TDMA_SLOT_MS * 1000ULL + (uint64_t)myJitter * 1000ULL;
  if (frame != lastPollFrame && netUs >= mySlotStart && (netUs - mySlotStart) < TX_WINDOW_MS * 1000ULL) {
    SyncPacket pkt; pkt.type = PKT_POLL; pkt.srcId = myId; pkt.dstId = 0xFF; pkt.seq = txSeq++; pkt.t1 = getRawMicros();
    pkt.advPhase = localPhase; pkt.advDrift = localDrift; pkt.advQuality = localQuality;
    sendPacket(pkt, IPAddress(224,0,0,1)); lastPollFrame = frame;
  }
  int sz; int packetsRead = 0;
  while ((sz = udp.parsePacket()) >= (int)sizeof(SyncPacket) && packetsRead < 10) {
    SyncPacket pkt; udp.read((uint8_t*)&pkt, sizeof(SyncPacket));
    uint64_t rxTime = getRawMicros();
    if (pkt.type == PKT_POLL) handlePoll(pkt, rxTime); else if (pkt.type == PKT_REPLY) handleReply(pkt, rxTime);
    packetsRead++;
  }
  if (now - lastKfTime >= 100) {
    double dt = (double)(now - lastKfTime) / 1000.0; lastKfTime = now;
    double weightedPhase = 0, weightedDrift = 0, totalWeight = 0;
    for (auto& p : peers) {
      if (!p.active || (now - p.lastHeard > 20000)) { p.active = false; continue; }
      double w = p.filter.quality() * (p.advQuality + 0.1) / (1.0 + p.avgRtt / 1000.0);
      weightedPhase += p.filter.phase * w; weightedDrift += p.filter.drift * w; totalWeight += w;
    }
    if (totalWeight > 1e-6) {
      weightedPhase /= totalWeight; weightedDrift /= totalWeight;
      double diff = weightedPhase - localPhase;
      if (abs(diff) > 200000) { localPhase = weightedPhase; localDrift = weightedDrift; }
      else { localPhase += 0.05 * diff; localDrift += 0.02 * (weightedDrift - localDrift); }
      localQuality = (float)std::min(1.0, totalWeight);
    } else { localQuality = 0; }
    localPhase += localDrift * dt;
  }
  static uint32_t lastDiag = 0;
  if (now - lastDiag >= 5000) { lastDiag = now; Serial.printf("NetTime: %llu, Phase: %.1f, Drift: %.2f, Quality: %.2f\n", getNetworkMicros(), localPhase, localDrift, localQuality); }
}
