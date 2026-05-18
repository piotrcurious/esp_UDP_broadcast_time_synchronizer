/*
 * ============================================================
 * ESP-UDP-SYNC v11.2 — Adaptive Lord-Slave Architecture
 * ============================================================
 */

#include <Arduino.h>
#if defined(ESP8266)
  #include <ESP8266WiFi.h>
#elif defined(ESP32)
  #include <WiFi.h>
  #include "esp_wifi.h"
#endif
#include <WiFiUdp.h>
#include <vector>
#include <algorithm>
#include <cmath>

#ifndef WIFI_SSID
#define WIFI_SSID "YourSSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "YourPassword"
#endif

// ── Constants ──────────────────────────────────────────────────
#define UDP_PORT              7777
#define MULTICAST_IP          "224.0.0.1"
#define MAX_NODES             16
#define TDMA_SLOT_MS          60
#define SUPERFRAME_MS         (TDMA_SLOT_MS * MAX_NODES)
#define TX_WINDOW_MS          5

#define KF_Q_PHASE            1.0
#define KF_Q_DRIFT            0.001
#define KF_R_BASE             500.0

#define HUBER_BASE_US         2000.0
#define ANCHOR_QUALITY_BONUS  5.0
#define RTT_WEIGHT_TAU        20000.0
#define MAX_SLEW_DRIFT        10000.0

#define PI_PH_GAIN            0.20
#define PI_DR_GAIN            0.05

enum PacketType : uint8_t { PKT_POLL = 1, PKT_REPLY = 2 };

#pragma pack(push, 1)
struct SyncPacket {
  uint8_t  type, srcId, dstId, preferredSlot;
  uint32_t seq;
  uint64_t t1, t2, t3;
  double   advPhase, advDrift;
  float    advQuality;
};
#pragma pack(pop)

class KalmanFilter {
public:
  double phase, drift, P[2][2];
  uint64_t lastUpdateUs;
  KalmanFilter() : phase(0.0), drift(0.0), lastUpdateUs(0) {
    P[0][0] = 1e7; P[0][1] = 0.0; P[1][0] = 0.0; P[1][1] = 1e5;
  }
  void predict(uint64_t nowUs) {
    if (!lastUpdateUs) { lastUpdateUs = nowUs; return; }
    double dt = (double)(nowUs - lastUpdateUs) * 1e-6;
    if (dt <= 0 || dt > 10.0) { lastUpdateUs = nowUs; return; }
    phase += drift * dt;
    double p00=P[0][0]+dt*(P[1][0]+P[0][1])+dt*dt*P[1][1]+KF_Q_PHASE, p01=P[0][1]+dt*P[1][1], p11=P[1][1]+KF_Q_DRIFT;
    P[0][0]=p00; P[0][1]=P[1][0]=p01; P[1][1]=p11; lastUpdateUs = nowUs;
  }
  void update(double z, double R) {
    if (std::isnan(z) || std::isinf(z)) return;
    double y = z - phase;
    if (fabs(y) > 200000.0) { phase = z; P[0][0]=1e7; P[1][1]=1e5; P[0][1]=P[1][0]=0; return; }
    double S = P[0][0] + R; if (S < 1e-9) return;
    double K0 = P[0][0]/S, K1 = P[1][0]/S;
    phase += K0*y; drift += K1*y;
    double a=1.0-K0, c=-K1, p00=P[0][0], p01=P[0][1], p11=P[1][1];
    P[0][0]=a*a*p00 + K0*K0*R; P[0][1]=a*(c*p00+p01) + K0*K1*R; P[1][0]=P[0][1];
    P[1][1]=c*c*p00 + 2.0*c*p01 + p11 + K1*K1*R;
  }
  float quality() const { return (float)(1.0 / (1.0 + (sqrt(fabs(P[0][0])) + sqrt(fabs(P[1][1]))*50.0)/1000.0)); }
};

struct Peer {
  uint8_t id; bool active; uint32_t lastHeard, lastSeq;
  KalmanFilter filter; double avgRtt, varRtt, minRtt, bestTheta;
  double advPhase, advDrift; float advQuality;
  explicit Peer(uint8_t _id) : id(_id), active(false), lastHeard(0), lastSeq(0xFFFFFFFF), filter(), avgRtt(0), varRtt(0), minRtt(1e9), bestTheta(0), advPhase(0), advDrift(0), advQuality(0) {}
};

class ClockSynchronizer {
public:
  uint8_t  myId = 0, netAnchorId = 0xFF;
  uint32_t txSeq = 0, lastPollFrame = 0xFFFFFFFF, lastProcessedFrame = 0xFFFFFFFF;
  double   localPhase = 0, localDrift = 0; float localQuality = 0;
  uint64_t lastSlewUs = 0; uint32_t lastKfTime = 0;
  std::vector<Peer> peers;
  WiFiUDP udp;

  uint32_t lastMicrosLow = 0;
  uint32_t microsHigh = 0;

  uint8_t  preferredSlot = 0xFF;
  uint32_t slotOccupancy[MAX_NODES];
  uint8_t  slotOwner[MAX_NODES];

  void begin(uint8_t id) {
    myId = id; preferredSlot = id % MAX_NODES;
    lastSlewUs = getRawMicros(); lastKfTime = millis();
    for (int i=0; i<MAX_NODES; i++) { slotOccupancy[i] = 0; slotOwner[i] = 0xFF; }
  }

  uint64_t getRawMicros() {
#if defined(ESP32)
    return (uint64_t)esp_timer_get_time();
#else
    uint32_t m = micros();
    if (m < lastMicrosLow) microsHigh++;
    lastMicrosLow = m;
    return ((uint64_t)microsHigh << 32) | m;
#endif
  }

  uint64_t getNetworkMicros() { return (uint64_t)((int64_t)getRawMicros() - (int64_t)localPhase); }

  void update() {
    uint64_t rawUs = getRawMicros(); uint32_t now = millis();
    uint64_t slewDt = rawUs - lastSlewUs;
    if (slewDt > 0) { localPhase += localDrift * (slewDt * 1e-6); lastSlewUs = rawUs; }

    uint64_t netUs = getNetworkMicros();
    uint64_t frameLenUs = (uint64_t)SUPERFRAME_MS * 1000ULL;
    uint32_t frame = (uint32_t)(netUs / frameLenUs);
    uint32_t pollInterval = (localQuality > 0.9f) ? 4 : 1;
    uint64_t slotStart = (uint64_t)frame * frameLenUs + (uint64_t)preferredSlot * TDMA_SLOT_MS * 1000ULL + (uint64_t)((myId * 13u) % (TDMA_SLOT_MS / 2)) * 1000ULL;

    if (frame != lastProcessedFrame) {
      lastProcessedFrame = frame;
      // Automatic slot discovery: check for collisions every frame
      if (slotOwner[preferredSlot] != 0xFF && slotOwner[preferredSlot] != myId) {
          if (slotOccupancy[preferredSlot] + 4 >= frame) { // Seen recently
              if (myId > slotOwner[preferredSlot]) { // Higher ID yields
                  preferredSlot = (preferredSlot + 1 + (uint8_t)((getRawMicros() + myId) % (MAX_NODES-1))) % MAX_NODES;
                  // Clear new slot so we don't immediately jump again if it happens to be "owned" by someone else in memory
                  slotOwner[preferredSlot] = myId; slotOccupancy[preferredSlot] = frame;
              }
          }
      }
    }

    if (frame % pollInterval == 0 && frame != lastPollFrame && netUs >= slotStart && (netUs - slotStart) < (uint64_t)(TX_WINDOW_MS * 1000)) {
      SyncPacket pkt = { PKT_POLL, myId, 0xFF, preferredSlot, txSeq++, getRawMicros(), 0, 0, localPhase, localDrift, localQuality };
      sendPacket(pkt, IPAddress(224, 0, 0, 1)); lastPollFrame = frame;
    }

    int sz, pkts = 0;
    while ((sz = udp.parsePacket()) >= (int)sizeof(SyncPacket) && pkts++ < 10) {
      SyncPacket pkt; udp.read((uint8_t*)&pkt, sizeof(SyncPacket));
      uint64_t rx = getRawMicros();
      uint64_t rxNetUs = rx - (uint64_t)localPhase;
      uint32_t rxFrame = (uint32_t)(rxNetUs / frameLenUs);
      uint32_t slot = pkt.preferredSlot;
      if (slot < MAX_NODES && pkt.srcId != myId) {
          if (rxFrame >= slotOccupancy[slot] || slotOwner[slot] == 0xFF) {
              if (rxFrame > slotOccupancy[slot] || pkt.srcId < slotOwner[slot]) {
                  slotOccupancy[slot] = rxFrame;
                  slotOwner[slot] = pkt.srcId;
              }
          }
      }
      if (pkt.type == PKT_POLL) handlePoll(pkt, rx); else if (pkt.type == PKT_REPLY) handleReply(pkt, rx);
    }
    if (now - lastKfTime >= 100) { lastKfTime = now; runConsensus(getRawMicros(), now); }
  }

private:
  void sendPacket(SyncPacket& pkt, IPAddress addr) { udp.beginPacket(addr, UDP_PORT); udp.write((uint8_t*)&pkt, sizeof(SyncPacket)); udp.endPacket(); }
  Peer* findOrCreatePeer(uint8_t id) { for (auto& p : peers) if (p.id == id) return &p; if (peers.size() < MAX_NODES) { peers.emplace_back(id); return &peers.back(); } return nullptr; }

  void handlePoll(SyncPacket& pkt, uint64_t rxTime) {
    if (pkt.srcId == myId) return;
    Peer* p = findOrCreatePeer(pkt.srcId); if (!p) return;
    p->active = true; p->lastHeard = millis(); p->advPhase = pkt.advPhase; p->advDrift = pkt.advDrift; p->advQuality = pkt.advQuality;
    if (localQuality < 0.05f) { localPhase = pkt.advPhase - (double)((int64_t)pkt.t1 - (int64_t)rxTime + 10000); localQuality = 0.1f; }
    SyncPacket reply = { PKT_REPLY, myId, pkt.srcId, preferredSlot, pkt.seq, pkt.t1, rxTime, getRawMicros(), localPhase, localDrift, localQuality };
    sendPacket(reply, udp.remoteIP());
  }

  void handleReply(SyncPacket& pkt, uint64_t rxTime) {
    if (pkt.dstId != myId) return;
    Peer* p = findOrCreatePeer(pkt.srcId); if (!p || pkt.seq == p->lastSeq) return;
    p->lastSeq = pkt.seq; uint64_t now = getRawMicros();
    double fwd = (double)((int64_t)pkt.t2 - (int64_t)pkt.t1), rev = (double)((int64_t)rxTime - (int64_t)pkt.t3);
    double rtt = fwd + rev, theta = (fwd - rev) * 0.5;
    if (rtt < 0 || rtt > 300000.0) return;
    if (rtt < p->minRtt) { p->minRtt = rtt; p->bestTheta = theta; } else { p->minRtt = 0.999*p->minRtt + 0.001*rtt; }
    if (p->avgRtt == 0) { p->avgRtt = rtt; p->varRtt = rtt*0.1; } else { double d = rtt-p->avgRtt; p->avgRtt += 0.1*d; p->varRtt = 0.9*p->varRtt + 0.1*d*d; }
    double targetPh = pkt.advPhase - p->bestTheta;
    if (localQuality < 0.1f || (pkt.srcId == netAnchorId && fabs(targetPh - localPhase) > 50000.0)) {
        localPhase = targetPh; localDrift = pkt.advDrift; localQuality = std::max(0.1f, pkt.advQuality * 0.8f);
        p->filter.phase = localPhase; p->filter.drift = localDrift;
    }
    p->filter.predict(now); p->filter.update(targetPh, KF_R_BASE + (rtt - p->minRtt)*5.0);
    p->active = true; p->lastHeard = millis(); p->advPhase = pkt.advPhase; p->advDrift = pkt.advDrift; p->advQuality = pkt.advQuality;
  }

  void runConsensus(uint64_t nowUs, uint32_t nowMs) {
    for (auto& p : peers) if (p.active && (nowMs - p.lastHeard > 20000)) p.active = false;
    netAnchorId = myId;
    for (auto& p : peers) if (p.active && p.id < netAnchorId) netAnchorId = p.id;
    if (netAnchorId == myId) {
        localDrift = 0;
        // We are the anchor. We keep our current phase to avoid jumps, but stop drifting.
        localQuality = 1.0f;
        return;
    }
    auto getW = [&](const Peer& pr) { return (double)(pr.filter.quality()+0.05f)*(pr.advQuality+0.05f)/(1.0 + (pr.avgRtt-pr.minRtt)/RTT_WEIGHT_TAU) * (pr.id == netAnchorId ? ANCHOR_QUALITY_BONUS : 1.0); };
    double wPh1=0, wDr1=0, wT1=0;
    for (auto& p : peers) if (p.active) { p.filter.predict(nowUs); double w = getW(p); wPh1 += p.filter.phase*w; wDr1 += p.filter.drift*w; wT1 += w; }
    double selfW = (double)localQuality * 0.2 + 0.01; wPh1 += localPhase*selfW; wDr1 += localDrift*selfW; wT1 += selfW;
    if (wT1 < 1e-9) return;
    double mu0 = wPh1/wT1, wPh2=0, wDr2=0, wT2=0, hT = (localQuality > 0.8) ? HUBER_BASE_US : 1000000.0;
    for (auto& p : peers) if (p.active) { double w = getW(p), diff_r = fabs(p.filter.phase - mu0); if (diff_r > hT) w *= hT/diff_r; wPh2 += p.filter.phase*w; wDr2 += p.filter.drift*w; wT2 += w; }
    double sW2 = selfW, rS = fabs(localPhase-mu0); if (rS > hT) sW2 *= hT/rS;
    if (wT2 < 1e-9) { wPh2 = localPhase; wDr2 = localDrift; wT2 = 1.0; } else { wPh2 += localPhase*sW2; wDr2 += localDrift*sW2; wT2 += sW2; }
    double rPh = wPh2/wT2, rDr = wDr2/wT2, diff = rPh - localPhase;
    if (fabs(diff) > 200000.0) { localPhase = rPh; localDrift = rDr; }
    else { localPhase += diff * PI_PH_GAIN; localDrift += PI_DR_GAIN * (rDr - localDrift) + 0.05 * diff; localDrift = std::max(-MAX_SLEW_DRIFT, std::min(MAX_SLEW_DRIFT, localDrift)); }
    localQuality = (float)std::min(1.0, wT2);
  }
};

#ifndef TEST_V11
ClockSynchronizer sync;
void setup() {
  Serial.begin(115200); WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print('.'); }
#if defined(ESP8266)
  WiFi.setSleepMode(WIFI_NONE_SLEEP); sync.udp.beginMulticast(WiFi.localIP(), IPAddress(224, 0, 0, 1), UDP_PORT);
#elif defined(ESP32)
  esp_wifi_set_ps(WIFI_PS_NONE); sync.udp.beginMulticast(IPAddress(224, 0, 0, 1), UDP_PORT);
#endif
  uint8_t mac[6]; WiFi.macAddress(mac); sync.begin(mac[5]);
}
void loop() {
  sync.update();
  static uint32_t lastDiag = 0;
  if (millis() - lastDiag >= 5000) {
    lastDiag = millis();
    Serial.printf("NetTime:%llu Ph:%.1f Dr:%.4f Q:%.2f Peers:%u\n", (unsigned long long)sync.getNetworkMicros(), sync.localPhase, sync.localDrift, (double)sync.localQuality, (unsigned)sync.peers.size());
  }
}
#endif
