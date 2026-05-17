#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include <stdint.h>

// ============================================================
// Configuration
// ============================================================

static const char* WIFI_SSID     = "YourSSID";
static const char* WIFI_PASSWORD = "YourPassword";

static constexpr uint8_t NUM_NODES       = 4;
static constexpr uint8_t MASTER_NODE_ID   = 0;
static constexpr uint8_t LED_PIN         = 2;

// Build-time node identity: set this per board.
// Example:
//   NODE_ID=0 -> master
//   NODE_ID=1 -> follower
#ifndef NODE_ID
#define NODE_ID 0
#endif

static constexpr uint8_t THIS_NODE_ID = NODE_ID;

// UDP multicast
static const IPAddress MULTICAST_IP(224, 0, 0, 1);
static constexpr uint16_t MULTICAST_PORT = 7777;

// TDMA timing
static constexpr uint64_t TDMA_SLOT_US       = 50000ULL;   // 50 ms per node slot
static constexpr uint64_t SUPERFRAME_US      = TDMA_SLOT_US * NUM_NODES;
static constexpr uint64_t TX_WINDOW_US       = 4000ULL;     // send near slot start
static constexpr uint64_t NODE_TIMEOUT_US     = 5000000ULL;  // 5 s
static constexpr uint64_t DIAGNOSTIC_PERIOD_US = 5000000ULL; // 5 s

// NTP / sync tuning
static constexpr double INITIAL_PROCESS_NOISE_OFFSET = 0.5;   // us^2-ish
static constexpr double INITIAL_PROCESS_NOISE_DRIFT  = 0.01;  // (us/s)^2-ish

// ============================================================
// Packet format
// ============================================================

enum PacketType : uint8_t {
  PKT_BEACON    = 1,   // Master beacon in its TDMA slot
  PKT_SYNC_POLL = 2,   // Follower poll to master
  PKT_SYNC_REPLY= 3    // Master reply with T2/T3
};

static constexpr uint8_t FLAG_NONE        = 0x00;
static constexpr uint8_t FLAG_TDMA        = 0x01;
static constexpr uint8_t FLAG_SYNC_VALID   = 0x02;
static constexpr uint8_t FLAG_REPLY       = 0x04;
static constexpr uint8_t FLAG_BEACON      = 0x08;

#pragma pack(push, 1)
struct NetworkPacket {
  uint8_t  version;
  uint8_t  type;
  uint8_t  flags;

  uint8_t  srcNodeId;
  uint8_t  dstNodeId;

  uint32_t sequence;
  uint32_t superframeId;

  // NTP-style timestamps
  // Poll:  t1Us is valid, t2Us/t3Us are zero
  // Reply: t1Us echoed from poll; t2Us and t3Us filled by master
  uint64_t t1Us;
  uint64_t t2Us;
  uint64_t t3Us;

  // Link / diagnostics
  int16_t  rssi;
  uint16_t reserved;

  float    syncQuality;
  float    offsetUs;
  float    delayUs;
  float    driftUsPerSec;
};
#pragma pack(pop)

// ============================================================
// Small utilities
// ============================================================

static inline double clampd(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi ? hi : v);
}

static inline uint64_t clampu64(uint64_t v, uint64_t lo, uint64_t hi) {
  return (v < lo) ? lo : (v > hi ? hi : v);
}

static inline double fabsd(double v) {
  return v < 0.0 ? -v : v;
}

// ============================================================
// 2-state clock filter: offset + drift
// Units:
//   offsetUs     = microseconds
//   driftUsPerSec= microseconds per second  (numerically same scale as ppm)
// ============================================================

struct ClockFilter {
  double offsetUs;
  double driftUsPerSec;

  double P[2][2];
  double Q[2][2];
  double lastUpdateUs;

  ClockFilter()
    : offsetUs(0.0),
      driftUsPerSec(0.0),
      lastUpdateUs(0.0) {
    P[0][0] = 1000.0; P[0][1] = 0.0;
    P[1][0] = 0.0;    P[1][1] = 1000.0;

    Q[0][0] = INITIAL_PROCESS_NOISE_OFFSET; Q[0][1] = 0.0;
    Q[1][0] = 0.0;                         Q[1][1] = INITIAL_PROCESS_NOISE_DRIFT;
  }

  double predictedOffsetUs(uint64_t nowUs) const {
    if (lastUpdateUs <= 0.0) {
      return offsetUs;
    }
    double dtSec = ((double)nowUs - lastUpdateUs) / 1000000.0;
    if (dtSec < 0.0) dtSec = 0.0;
    return offsetUs + dtSec * driftUsPerSec;
  }

  void update(double measurementUs, double measurementNoiseUs2, uint64_t nowUs) {
    double dtSec = 0.0;
    if (lastUpdateUs > 0.0) {
      dtSec = ((double)nowUs - lastUpdateUs) / 1000000.0;
      if (dtSec < 0.0) dtSec = 0.0;
    } else {
      dtSec = 0.05; // small default
    }

    // Predict
    offsetUs += dtSec * driftUsPerSec;

    const double P00 = P[0][0];
    const double P01 = P[0][1];
    const double P10 = P[1][0];
    const double P11 = P[1][1];

    const double dt2 = dtSec * dtSec;

    P[0][0] = P00 + dtSec * (P10 + P01) + dt2 * P11 + Q[0][0];
    P[0][1] = P01 + dtSec * P11 + Q[0][1];
    P[1][0] = P10 + dtSec * P11 + Q[1][0];
    P[1][1] = P11 + Q[1][1];

    // Update
    const double R = clampd(measurementNoiseUs2, 1.0, 1e12);
    const double S = P[0][0] + R;
    if (S <= 1e-12) {
      lastUpdateUs = (double)nowUs;
      return;
    }

    const double K0 = P[0][0] / S;
    const double K1 = P[1][0] / S;
    const double innovation = measurementUs - offsetUs;

    offsetUs      += K0 * innovation;
    driftUsPerSec += K1 * innovation;

    const double newP00 = (1.0 - K0) * P[0][0];
    const double newP01 = (1.0 - K0) * P[0][1];
    const double newP10 = P[1][0] - K1 * P[0][0];
    const double newP11 = P[1][1] - K1 * P[0][1];

    P[0][0] = (newP00 < 1e-9) ? 1e-9 : newP00;
    P[0][1] = newP01;
    P[1][0] = newP10;
    P[1][1] = (newP11 < 1e-9) ? 1e-9 : newP11;

    // Keep symmetry reasonable
    const double avg = 0.5 * (P[0][1] + P[1][0]);
    P[0][1] = avg;
    P[1][0] = avg;

    lastUpdateUs = (double)nowUs;
  }

  double quality() const {
    double p = P[0][0];
    if (p < 1e-9) p = 1e-9;
    return 1.0 / (1.0 + sqrt(p));
  }
};

// ============================================================
// Node state
// ============================================================

struct NodeState {
  bool active;
  IPAddress ip;
  uint64_t lastHeardUs;
  int signalStrength;
  double packetLoss;
  uint32_t lastSequence;
  uint64_t lastRTTUs;
  double avgRTTUs;
  double lastOffsetUs;
  ClockFilter filter;

  NodeState()
    : active(false),
      ip(0, 0, 0, 0),
      lastHeardUs(0),
      signalStrength(0),
      packetLoss(0.0),
      lastSequence(0),
      lastRTTUs(0),
      avgRTTUs(0.0),
      lastOffsetUs(0.0),
      filter() {}
};

struct PendingRequest {
  bool active;
  uint32_t sequence;
  uint64_t t1Us;

  PendingRequest() : active(false), sequence(0), t1Us(0) {}
};

// ============================================================
// Main synchronizer
// ============================================================

class NetworkTimeSync {
private:
  WiFiUDP udp;

  NodeState nodes[NUM_NODES];
  PendingRequest pending[8];

  uint8_t nodeId;
  uint32_t txSequence;
  uint32_t lastBeaconFrameSent;
  uint32_t lastPollFrameSent;
  uint64_t lastDiagnosticsUs;

  bool isMaster() const {
    return nodeId == MASTER_NODE_ID;
  }

  uint64_t nowUs() const {
    return (uint64_t)esp_timer_get_time();
  }

  uint64_t getDisciplinedTimeUs() const {
    uint64_t base = nowUs();
    if (isMaster()) {
      return base;
    }
    double predictedOffset = nodes[nodeId].filter.predictedOffsetUs(base);
    double corrected = (double)base + predictedOffset;
    if (corrected < 0.0) corrected = 0.0;
    return (uint64_t)llround(corrected);
  }

  uint32_t currentSuperframeId(uint64_t tUs) const {
    return (uint32_t)(tUs / SUPERFRAME_US);
  }

  uint64_t superframeStartUs(uint32_t frameId) const {
    return (uint64_t)frameId * SUPERFRAME_US;
  }

  uint64_t slotStartUs(uint32_t frameId, uint8_t slot) const {
    return superframeStartUs(frameId) + (uint64_t)slot * TDMA_SLOT_US;
  }

  bool withinWindow(uint64_t tUs, uint64_t startUs, uint64_t windowUs) const {
    return (tUs >= startUs) && ((tUs - startUs) < windowUs);
  }

  double calculateMeasurementNoiseUs2(int rssi, double packetLoss, double approxDelayUs) const {
    // Larger noise for weaker signal, more packet loss, and larger/erratic delay.
    double rssiClamped = clampd((double)rssi, -90.0, -30.0);
    double rssiBad = (-30.0 - rssiClamped) / 60.0; // 0 good -> 1 bad
    double lossBad = clampd(packetLoss, 0.0, 0.5) / 0.5;
    double delayBad = clampd(approxDelayUs / 2000.0, 0.0, 1.0);

    double stddevUs = 80.0 * (1.0 + 4.0 * rssiBad + 6.0 * lossBad + 2.0 * delayBad);
    if (stddevUs < 10.0) stddevUs = 10.0;
    return stddevUs * stddevUs;
  }

  double packetLossFromSeq(uint32_t currentSeq, uint32_t lastSeq) const {
    if (lastSeq == 0) return 0.0;
    uint32_t delta = currentSeq - lastSeq; // wrap-safe
    if (delta <= 1) return 0.0;
    return (double)(delta - 1) / (double)delta;
  }

  void registerPending(uint32_t sequence, uint64_t t1Us) {
    uint8_t idx = (uint8_t)(sequence & 0x07);
    pending[idx].active = true;
    pending[idx].sequence = sequence;
    pending[idx].t1Us = t1Us;
  }

  bool resolvePending(uint32_t sequence, uint64_t &t1UsOut) {
    uint8_t idx = (uint8_t)(sequence & 0x07);
    if (pending[idx].active && pending[idx].sequence == sequence) {
      t1UsOut = pending[idx].t1Us;
      pending[idx].active = false;
      return true;
    }
    return false;
  }

  void sendPacket(const NetworkPacket &packet, const IPAddress &targetIP) {
    udp.beginPacket(targetIP, MULTICAST_PORT);
    udp.write((const uint8_t*)&packet, sizeof(packet));
    udp.endPacket();
  }

  void sendBeacon(uint64_t tUs, uint32_t frameId) {
    NetworkPacket pkt{};
    pkt.version = 1;
    pkt.type = PKT_BEACON;
    pkt.flags = FLAG_BEACON | FLAG_TDMA | FLAG_SYNC_VALID;
    pkt.srcNodeId = nodeId;
    pkt.dstNodeId = 0xFF;
    pkt.sequence = ++txSequence;
    pkt.superframeId = frameId;
    pkt.t1Us = tUs;
    pkt.t2Us = 0;
    pkt.t3Us = 0;
    pkt.rssi = (int16_t)WiFi.RSSI();
    pkt.reserved = 0;
    pkt.syncQuality = isMaster() ? 1.0f : (float)nodes[nodeId].filter.quality();
    pkt.offsetUs = isMaster() ? 0.0f : (float)nodes[nodeId].filter.predictedOffsetUs(tUs);
    pkt.delayUs = 0.0f;
    pkt.driftUsPerSec = isMaster() ? 0.0f : (float)nodes[nodeId].filter.driftUsPerSec;

    sendPacket(pkt, MULTICAST_IP);
    lastBeaconFrameSent = frameId;

    if (isMaster()) {
      digitalWrite(LED_PIN, HIGH);
    }
  }

  void sendSyncPoll(uint64_t tUs, uint32_t frameId) {
    NetworkPacket pkt{};
    pkt.version = 1;
    pkt.type = PKT_SYNC_POLL;
    pkt.flags = FLAG_TDMA | FLAG_SYNC_VALID;
    pkt.srcNodeId = nodeId;
    pkt.dstNodeId = MASTER_NODE_ID;
    pkt.sequence = ++txSequence;
    pkt.superframeId = frameId;
    pkt.t1Us = tUs;
    pkt.t2Us = 0;
    pkt.t3Us = 0;
    pkt.rssi = (int16_t)WiFi.RSSI();
    pkt.reserved = 0;
    pkt.syncQuality = (float)nodes[nodeId].filter.quality();
    pkt.offsetUs = (float)nodes[nodeId].filter.predictedOffsetUs(tUs);
    pkt.delayUs = 0.0f;
    pkt.driftUsPerSec = (float)nodes[nodeId].filter.driftUsPerSec;

    registerPending(pkt.sequence, tUs);
    sendPacket(pkt, MULTICAST_IP);
    lastPollFrameSent = frameId;

    digitalWrite(LED_PIN, HIGH);
  }

  void sendSyncReply(const IPAddress &targetIP, const NetworkPacket &pollPkt, uint64_t rxUs) {
    NetworkPacket reply{};
    reply.version = 1;
    reply.type = PKT_SYNC_REPLY;
    reply.flags = FLAG_REPLY | FLAG_SYNC_VALID;
    reply.srcNodeId = MASTER_NODE_ID;
    reply.dstNodeId = pollPkt.srcNodeId;
    reply.sequence = pollPkt.sequence;   // echo poll sequence for matching
    reply.superframeId = pollPkt.superframeId;
    reply.t1Us = pollPkt.t1Us;
    reply.t2Us = rxUs;
    reply.t3Us = nowUs();                // transmit timestamp approximation
    reply.rssi = (int16_t)WiFi.RSSI();
    reply.reserved = 0;
    reply.syncQuality = 1.0f;
    reply.offsetUs = 0.0f;
    reply.delayUs = 0.0f;
    reply.driftUsPerSec = 0.0f;

    sendPacket(reply, targetIP);
  }

  void handleSyncReply(const NetworkPacket &pkt, uint64_t nowBaseUs, uint64_t nowDiscUs) {
    if (isMaster()) return;
    if (pkt.dstNodeId != nodeId) return;

    uint64_t t1Us = 0;
    if (!resolvePending(pkt.sequence, t1Us)) {
      return;
    }

    const uint64_t t4Us = nowDiscUs;

    // NTP equations:
    // offset = ((T2 - T1) + (T3 - T4)) / 2
    // delay  = (T4 - T1) - (T3 - T2)
    double T1 = (double)t1Us;
    double T2 = (double)pkt.t2Us;
    double T3 = (double)pkt.t3Us;
    double T4 = (double)t4Us;

    double offsetUs = ((T2 - T1) + (T3 - T4)) * 0.5;
    double delayUs  = (T4 - T1) - (T3 - T2);

    if (!isfinite(offsetUs) || !isfinite(delayUs)) {
      return;
    }

    if (delayUs < 0.0) delayUs = 0.0;
    if (fabsd(offsetUs) > 2000000.0) {
      // Reject wild outliers > 2 seconds.
      return;
    }

    NodeState &master = nodes[MASTER_NODE_ID];
    master.active = true;
    master.lastHeardUs = nowBaseUs;
    master.signalStrength = pkt.rssi;
    master.lastOffsetUs = offsetUs;
    master.lastRTTUs = (uint64_t)delayUs;
    if (master.avgRTTUs <= 0.0) master.avgRTTUs = delayUs;
    else master.avgRTTUs = master.avgRTTUs * 0.8 + delayUs * 0.2;

    // Use delay and link quality to scale measurement noise.
    double loss = master.packetLoss;
    double noiseUs2 = calculateMeasurementNoiseUs2(pkt.rssi, loss, delayUs);

    nodes[nodeId].filter.update(offsetUs, noiseUs2, nowBaseUs);

    // Record current estimate for diagnostics.
    master.filter.offsetUs = nodes[nodeId].filter.offsetUs;
    master.filter.driftUsPerSec = nodes[nodeId].filter.driftUsPerSec;
    master.filter.P[0][0] = nodes[nodeId].filter.P[0][0];
    master.filter.P[0][1] = nodes[nodeId].filter.P[0][1];
    master.filter.P[1][0] = nodes[nodeId].filter.P[1][0];
    master.filter.P[1][1] = nodes[nodeId].filter.P[1][1];
    master.filter.lastUpdateUs = nodes[nodeId].filter.lastUpdateUs;
  }

  void processIncomingPackets() {
    int packetSize = 0;

    while ((packetSize = udp.parsePacket()) > 0) {
      if (packetSize != (int)sizeof(NetworkPacket)) {
        while (udp.available()) {
          udp.read();
        }
        continue;
      }

      NetworkPacket pkt{};
      int readBytes = udp.read((uint8_t*)&pkt, sizeof(pkt));
      if (readBytes != (int)sizeof(pkt)) {
        continue;
      }

      const uint64_t nowBaseUs = nowUs();
      const uint64_t nowDiscUs = getDisciplinedTimeUs();

      // Ignore our own multicast echoes.
      if (pkt.srcNodeId == nodeId) {
        continue;
      }

      if (pkt.srcNodeId < NUM_NODES) {
        NodeState &peer = nodes[pkt.srcNodeId];
        peer.active = true;
        peer.ip = udp.remoteIP();
        peer.lastHeardUs = nowBaseUs;
        peer.signalStrength = pkt.rssi;

        if (pkt.type != PKT_SYNC_REPLY) {
          peer.packetLoss = packetLossFromSeq(pkt.sequence, peer.lastSequence);
          peer.lastSequence = pkt.sequence;
        }
      }

      if (pkt.type == PKT_BEACON) {
        if (pkt.srcNodeId == MASTER_NODE_ID) {
          // Beacon gives everyone a clean frame boundary signal.
          nodes[MASTER_NODE_ID].active = true;
          nodes[MASTER_NODE_ID].lastHeardUs = nowBaseUs;
          nodes[MASTER_NODE_ID].signalStrength = pkt.rssi;
        }
      } else if (pkt.type == PKT_SYNC_POLL) {
        // Only master replies to polls.
        if (isMaster() && pkt.dstNodeId == MASTER_NODE_ID) {
          sendSyncReply(udp.remoteIP(), pkt, nowBaseUs);
        }
      } else if (pkt.type == PKT_SYNC_REPLY) {
        handleSyncReply(pkt, nowBaseUs, nowDiscUs);
      }
    }
  }

  void maintainTdmaSchedule() {
    const uint64_t tUs = getDisciplinedTimeUs();
    const uint32_t frameId = currentSuperframeId(tUs);
    const uint64_t frameStart = superframeStartUs(frameId);

    if (isMaster()) {
      // Master beacon in its slot.
      const uint64_t beaconSlotStart = slotStartUs(frameId, MASTER_NODE_ID);
      if (frameId != lastBeaconFrameSent &&
          withinWindow(tUs, beaconSlotStart, TX_WINDOW_US)) {
        sendBeacon(tUs, frameId);
      } else if ((tUs - frameStart) > TX_WINDOW_US) {
        digitalWrite(LED_PIN, LOW);
      }
    } else {
      // Each follower transmits its poll in its own TDMA slot.
      const uint64_t mySlotStart = slotStartUs(frameId, nodeId);
      if (frameId != lastPollFrameSent &&
          withinWindow(tUs, mySlotStart, TX_WINDOW_US)) {
        sendSyncPoll(tUs, frameId);
      } else if ((tUs - frameStart) > TX_WINDOW_US) {
        digitalWrite(LED_PIN, LOW);
      }
    }
  }

  void updatePeerTimeouts(uint64_t nowUsVal) {
    for (uint8_t i = 0; i < NUM_NODES; ++i) {
      if (!nodes[i].active) continue;
      if ((nowUsVal - nodes[i].lastHeardUs) > NODE_TIMEOUT_US) {
        nodes[i].active = false;
      }
    }
  }

  void printDiagnostics(uint64_t nowUsVal) {
    Serial.println();
    Serial.println(F("Network Diagnostics"));
    Serial.printf("Role: %s | Node ID: %u\n", isMaster() ? "MASTER" : "FOLLOWER", (unsigned)nodeId);
    Serial.printf("Local IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("WiFi RSSI: %d dBm\n", WiFi.RSSI());
    Serial.printf("Disciplined time: %llu us\n", (unsigned long long)getDisciplinedTimeUs());
    Serial.printf("Clock offset: %.2f us\n", nodes[nodeId].filter.predictedOffsetUs(nowUsVal));
    Serial.printf("Clock drift: %.6f us/s\n", nodes[nodeId].filter.driftUsPerSec);
    Serial.printf("Clock quality: %.4f\n", nodes[nodeId].filter.quality());

    for (uint8_t i = 0; i < NUM_NODES; ++i) {
      if (!nodes[i].active) continue;

      Serial.printf(
        "Node %u: RSSI=%d dBm | Loss=%.2f%% | RTT=%.2f us | Offset=%.2f us | SyncQ=%.4f\n",
        (unsigned)(i + 1),
        nodes[i].signalStrength,
        nodes[i].packetLoss * 100.0,
        nodes[i].avgRTTUs,
        nodes[i].lastOffsetUs,
        nodes[i].filter.quality()
      );
    }

    Serial.println();
    lastDiagnosticsUs = nowUsVal;
  }

  void reconnectWiFiIfNeeded() {
    if (WiFi.status() == WL_CONNECTED) return;

    static uint64_t lastAttemptUs = 0;
    const uint64_t tUs = nowUs();

    if ((tUs - lastAttemptUs) < 5000000ULL) return;
    lastAttemptUs = tUs;

    Serial.println(F("WiFi reconnecting..."));
    WiFi.disconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

public:
  NetworkTimeSync()
    : nodeId(THIS_NODE_ID),
      txSequence(0),
      lastBeaconFrameSent(0xFFFFFFFFUL),
      lastPollFrameSent(0xFFFFFFFFUL),
      lastDiagnosticsUs(0) {}

  void begin() {
    Serial.begin(115200);
    delay(200);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    if (nodeId >= NUM_NODES) {
      nodeId = MASTER_NODE_ID;
    }

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.persistent(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print(F("Connecting to WiFi"));
    uint64_t startUs = nowUs();
    while (WiFi.status() != WL_CONNECTED && (nowUs() - startUs) < 15000000ULL) {
      delay(250);
      Serial.print(F("."));
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.print(F("Connected. IP: "));
      Serial.println(WiFi.localIP());
    } else {
      Serial.println(F("WiFi not connected yet; continuing with reconnect attempts."));
    }

    // Join multicast group. If your core needs a different beginMulticast signature,
    // adjust only this line.
    if (!udp.beginMulticast(MULTICAST_IP, MULTICAST_PORT)) {
      Serial.println(F("Failed to join multicast group."));
    } else {
      Serial.println(F("Joined multicast group."));
    }

    nodes[nodeId].active = true;
    nodes[nodeId].lastHeardUs = nowUs();

    lastDiagnosticsUs = nowUs();
  }

  void update() {
    reconnectWiFiIfNeeded();

    processIncomingPackets();
    maintainTdmaSchedule();

    const uint64_t tUs = nowUs();
    updatePeerTimeouts(tUs);

    if ((tUs - lastDiagnosticsUs) >= DIAGNOSTIC_PERIOD_US) {
      printDiagnostics(tUs);
    }

    delay(1); // yield to WiFi stack
  }

  int64_t getNetworkTimeUs() const {
    uint64_t base = (uint64_t)esp_timer_get_time();
    if (isMaster()) {
      return (int64_t)base;
    }
    double predictedOffset = nodes[nodeId].filter.predictedOffsetUs(base);
    double corrected = (double)base + predictedOffset;
    if (corrected < 0.0) corrected = 0.0;
    return (int64_t)llround(corrected);
  }
};

// ============================================================
// Arduino entry points
// ============================================================

NetworkTimeSync timeSync;

void setup() {
  timeSync.begin();
}

void loop() {
  timeSync.update();
}
