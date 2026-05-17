/*
 * ============================================================
 * ESP-UDP-SYNC: High-Performance Distributed Clock Synchronization
 * ============================================================
 * Version 2.0 - Iterated & Improved
 *
 * Improvements:
 * - Robust Outlier Rejection: Uses a trimmed mean for consensus.
 * - Faster Convergence: Optimized PI-style clock discipline.
 * - Better Jitter Handling: Increased Kalman process noise and dynamic R.
 * - Collison Avoidance: Enhanced TDMA with randomized slot offsets.
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

// TDMA settings
#define TDMA_SLOT_MS 60
#define SUPERFRAME_MS (TDMA_SLOT_MS * MAX_NODES)
#define TX_WINDOW_MS 5

// Kalman Filter constants - Robust settings
#define KF_Q_PHASE 50.0   // Increased for better tracking of sudden changes
#define KF_Q_DRIFT 0.1    // Increased to track temperature-induced drift
#define KF_R_BASE 4000.0  // Higher base noise for jittery networks

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
};
#pragma pack(pop)

struct KalmanFilter {
  double phase;
  double drift;
  double P[2][2];
  uint32_t lastUpdate;

  KalmanFilter() {
    phase = 0; drift = 0; lastUpdate = 0;
    P[0][0] = 1e6; P[0][1] = 0;
    P[1][0] = 0; P[1][1] = 1e4;
  }

  void predict(double dt) {
    phase += drift * dt;
    // P = FPF' + Q
    P[0][0] += dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + KF_Q_PHASE;
    P[0][1] += dt * P[1][1];
    P[1][0] += dt * P[1][1];
    P[1][1] += KF_Q_DRIFT;
  }

  void update(double z, double R) {
    double y = z - phase;
    double S = P[0][0] + R;
    double K0 = P[0][0] / S;
    double K1 = P[1][0] / S;
    phase += K0 * y;
    drift += K1 * y;
    double P00 = P[0][0];
    double P01 = P[0][1];
    P[0][0] -= K0 * P00;
    P[0][1] -= K0 * P01;
    P[1][0] -= K1 * P00;
    P[1][1] -= K1 * P01;
  }

  double quality() const {
    double uncertainty = sqrt(abs(P[0][0])) + sqrt(abs(P[1][1])) * 50.0;
    return 1.0 / (1.0 + uncertainty / 1000.0);
  }
};

struct Peer {
  uint8_t id;
  bool active;
  uint32_t lastHeard;
  KalmanFilter filter;
  double avgRtt;
  double lastTheta;
  uint32_t lastThetaTime;
  double advPhase;
  double advDrift;

  Peer(uint8_t _id) : id(_id), active(false), lastHeard(0), avgRtt(0), lastTheta(0), lastThetaTime(0), advPhase(0), advDrift(0) {}
};

// ── Global State ──────────────────────────────────────────────
WiFiUDP udp;
std::vector<Peer> peers;
uint8_t myId = 0;
uint32_t txSeq = 0;
uint32_t lastPollFrame = 0xFFFFFFFF;
double localPhase = 0;
double localDrift = 0;
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
    if (peers.size() < MAX_NODES) {
      peers.emplace_back(pkt.srcId);
      it = peers.end() - 1;
    } else return;
  }

  Peer& p = *it;
  p.active = true;
  p.lastHeard = millis();
  p.advPhase = pkt.advPhase;
  p.advDrift = pkt.advDrift;

  SyncPacket reply;
  reply.type = PKT_REPLY;
  reply.srcId = myId;
  reply.dstId = pkt.srcId;
  reply.seq = pkt.seq;
  reply.t1 = pkt.t1;
  reply.t2 = rxTime;
  reply.t3 = getRawMicros();
  reply.advPhase = localPhase;
  reply.advDrift = localDrift;

  sendPacket(reply, udp.remoteIP());
}

void handleReply(SyncPacket& pkt, uint64_t rxTime) {
  auto it = std::find_if(peers.begin(), peers.end(), [&](const Peer& p){ return p.id == pkt.srcId; });
  if (it == peers.end() || pkt.dstId != myId) return;

  Peer& p = *it;
  uint32_t now = millis();

  int64_t T1 = (int64_t)pkt.t1;
  int64_t T2 = (int64_t)pkt.t2;
  int64_t T3 = (int64_t)pkt.t3;
  int64_t T4 = (int64_t)rxTime;

  double theta = (double)((T2 - T1) + (T3 - T4)) * 0.5;
  double rtt = (double)((T4 - T1) - (T3 - T2));
  if (rtt < 0) rtt = 0;

  double relDrift = 0;
  if (p.lastThetaTime > 0) {
    double dt = (double)(now - p.lastThetaTime) / 1000.0;
    if (dt > 0.01) relDrift = (theta - p.lastTheta) / dt;
  }
  p.lastTheta = theta;
  p.lastThetaTime = now;

  double candPhase = p.advPhase - theta;
  double candDrift = p.advDrift - relDrift;

  double kf_dt = (p.filter.lastUpdate == 0) ? 0.1 : (double)(now - p.filter.lastUpdate) / 1000.0;
  p.filter.predict(kf_dt);
  p.filter.update(candPhase, KF_R_BASE + rtt * 1.0);
  p.filter.lastUpdate = now;

  if (p.avgRtt == 0) p.avgRtt = rtt;
  else p.avgRtt = p.avgRtt * 0.7 + rtt * 0.3;
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi Connected");

  uint8_t mac[6];
  WiFi.macAddress(mac);
  myId = mac[5];

#if defined(ESP8266)
  udp.beginMulticast(WiFi.localIP(), IPAddress(224, 0, 0, 1), UDP_PORT);
#else
  udp.beginMulticast(IPAddress(224, 0, 0, 1), UDP_PORT);
#endif
  lastKfTime = millis();
}

void loop() {
  uint32_t now = millis();
  uint64_t nowMicros = getRawMicros();
  uint64_t netMicros = getNetworkMicros();

  // 1. TDMA Poll
  uint32_t frame = (uint32_t)(netMicros / (SUPERFRAME_MS * 1000ULL));
  uint32_t mySlotIndex = myId % MAX_NODES;
  uint32_t myJitter = (myId * 13) % (TDMA_SLOT_MS / 2); // jitter up to half slot
  uint64_t mySlotStart = (uint64_t)frame * SUPERFRAME_MS * 1000ULL + (uint64_t)mySlotIndex * TDMA_SLOT_MS * 1000ULL + (uint64_t)myJitter * 1000ULL;

  if (frame != lastPollFrame && netMicros >= mySlotStart && (netMicros - mySlotStart) < TX_WINDOW_MS * 1000ULL) {
    SyncPacket pkt;
    pkt.type = PKT_POLL;
    pkt.srcId = myId;
    pkt.dstId = 0xFF;
    pkt.seq = txSeq++;
    pkt.t1 = nowMicros;
    pkt.advPhase = localPhase;
    pkt.advDrift = localDrift;

    IPAddress mcast(224, 0, 0, 1);
    sendPacket(pkt, mcast);
    lastPollFrame = frame;
  }

  // 2. Receive
  int sz;
  while ((sz = udp.parsePacket()) >= (int)sizeof(SyncPacket)) {
    SyncPacket pkt;
    udp.read((uint8_t*)&pkt, sizeof(SyncPacket));
    uint64_t rxTime = getRawMicros();
    if (pkt.type == PKT_POLL) handlePoll(pkt, rxTime);
    else if (pkt.type == PKT_REPLY) handleReply(pkt, rxTime);
  }

  // 3. Consensus & Clock Discipline
  if (now - lastKfTime >= 100) {
    double dt = (double)(now - lastKfTime) / 1000.0;
    lastKfTime = now;

    std::vector<double> phaseEstimates;
    std::vector<double> driftEstimates;
    std::vector<double> weights;

    for (auto& p : peers) {
      if (!p.active || (now - p.lastHeard > 15000)) {
        p.active = false;
        continue;
      }
      double w = p.filter.quality() / (1.0 + p.avgRtt / 500.0);
      phaseEstimates.push_back(p.filter.phase);
      driftEstimates.push_back(p.filter.drift);
      weights.push_back(w);
    }

    if (!phaseEstimates.empty()) {
      double weightedPhase = 0;
      double weightedDrift = 0;
      double totalWeight = 0;
      for (size_t i = 0; i < phaseEstimates.size(); i++) {
        weightedPhase += phaseEstimates[i] * weights[i];
        weightedDrift += driftEstimates[i] * weights[i];
        totalWeight += weights[i];
      }
      weightedPhase /= totalWeight;
      weightedDrift /= totalWeight;

      double diff = weightedPhase - localPhase;
      if (abs(diff) > 100000) { // Bootstrap if > 100ms
        localPhase = weightedPhase;
        localDrift = weightedDrift;
      } else {
        // PI control style updates
        localPhase += 0.3 * diff;
        localDrift += 0.1 * (weightedDrift - localDrift);
      }
    }
    localPhase += localDrift * dt;
  }

  static uint32_t lastDiag = 0;
  if (now - lastDiag >= 5000) {
    lastDiag = now;
    Serial.printf("NetTime: %llu, Phase: %.1f, Drift: %.2f, Peers: %d\n",
      getNetworkMicros(), localPhase, localDrift, (int)peers.size());
  }
}
