#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_timer.h>
#include <math.h>
#include <stdint.h>

// ============================================================
// Configuration
// ============================================================

static const char* WIFI_SSID     = "YourSSID";
static const char* WIFI_PASSWORD = "YourPassword";

static constexpr uint8_t NUM_NODES = 4;
#ifndef NODE_ID
#define NODE_ID 0
#endif
static constexpr uint8_t THIS_NODE_ID = NODE_ID;

static constexpr uint8_t LED_PIN = 2;

// UDP multicast for discovery / polls
static const IPAddress MULTICAST_IP(224, 0, 0, 1);
static constexpr uint16_t MULTICAST_PORT = 7777;

// Soft TDMA
static constexpr uint64_t TDMA_SLOT_US        = 60000ULL;  // 60 ms per node
static constexpr uint64_t SUPERFRAME_US       = TDMA_SLOT_US * NUM_NODES;
static constexpr uint64_t TX_WINDOW_US        = 5000ULL;    // transmit window at slot start
static constexpr uint64_t DIAGNOSTIC_PERIOD_US = 5000000ULL; // 5 s
static constexpr uint64_t PEER_TIMEOUT_US      = 8000000ULL; // 8 s
static constexpr uint64_t PENDING_TIMEOUT_US   = 3000000ULL; // 3 s

static constexpr uint32_t MAX_PENDING_POLLS   = 8;
static constexpr uint32_t MAX_PENDING_REPLIES = 16;

// ============================================================
// Helpers
// ============================================================

static inline double clampd(double v, double lo, double hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline uint64_t clampu64(uint64_t v, uint64_t lo, uint64_t hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline double fabsd(double v) {
  return (v < 0.0) ? -v : v;
}

// Deterministic small delay for replies to reduce collisions.
static uint64_t deterministicReplyDelayUs(uint8_t src, uint8_t dst, uint32_t seq) {
  uint32_t x = seq ^ (uint32_t(src) * 0x9E3779B9u) ^ (uint32_t(dst) * 0x85EBCA6Bu);
  x ^= (x >> 16);
  x *= 0x7FEB352Du;
  x ^= (x >> 15);
  x *= 0x846CA68Bu;
  x ^= (x >> 16);
  return 600ULL + (uint64_t)(x % 2500u); // 0.6 ms .. 3.1 ms
}

// ============================================================
// Packet format
// ============================================================

enum PacketType : uint8_t {
  PKT_POLL  = 1,
  PKT_REPLY = 2
};

static constexpr uint8_t FLAG_NONE       = 0x00;
static constexpr uint8_t FLAG_TDMA       = 0x01;
static constexpr uint8_t FLAG_SYNC_VALID = 0x02;
static constexpr uint8_t FLAG_POLL       = 0x04;
static constexpr uint8_t FLAG_REPLY      = 0x08;

#pragma pack(push, 1)
struct NetworkPacket {
  uint8_t  version;
  uint8_t  type;
  uint8_t  flags;

  uint8_t  srcNodeId;
  uint8_t  dstNodeId;
  uint8_t  reserved0;
  uint16_t reserved1;

  uint32_t sequence;      // sequence of this transmission
  uint32_t echoSequence;   // poll sequence echoed back by reply
  uint32_t superframeId;

  // NTP-style timestamps
  // Poll:  t1Us is valid
  // Reply: t1Us echoed, t2Us = receive time at responder, t3Us = send time of reply
  uint64_t t1Us;
  uint64_t t2Us;
  uint64_t t3Us;

  // Advertised local clock estimate of the sender:
  // phaseUs = local_clock - network_clock
  // driftUsPerSec = d(phaseUs)/dt
  float phaseUs;
  float driftUsPerSec;
  float quality;

  int16_t rssi;
  uint16_t reserved2;
};
#pragma pack(pop)

// ============================================================
// Decentralized clock discipline
// phaseUs: local clock error relative to network time
//          positive means local clock is ahead of network time
// networkTime ~= rawTime - phaseUs - drift*dt
// ============================================================

struct ClockDiscipline {
  double phaseUs;
  double driftUsPerSec;
  double quality;
  uint64_t lastPredictUs;

  ClockDiscipline()
    : phaseUs(0.0),
      driftUsPerSec(0.0),
      quality(0.0),
      lastPredictUs(0) {}

  void predictTo(uint64_t nowUs) {
    if (lastPredictUs == 0) {
      lastPredictUs = nowUs;
      return;
    }
    double dtSec = double(nowUs - lastPredictUs) / 1000000.0;
    if (dtSec < 0.0) dtSec = 0.0;

    phaseUs += dtSec * driftUsPerSec;
    phaseUs = clampd(phaseUs, -1000000.0, 1000000.0);
    driftUsPerSec = clampd(driftUsPerSec, -2000.0, 2000.0); // +/- 2000 us/s = 2000 ppm-ish
    lastPredictUs = nowUs;
  }

  double predictedPhaseUs(uint64_t nowUs) const {
    if (lastPredictUs == 0) return phaseUs;
    double dtSec = double(nowUs - lastPredictUs) / 1000000.0;
    if (dtSec < 0.0) dtSec = 0.0;
    return phaseUs + dtSec * driftUsPerSec;
  }

  void mergeCandidate(double candidatePhaseUs, double candidateDriftUsPerSec, double weight, uint64_t nowUs) {
    predictTo(nowUs);

    // Conservative update to avoid oscillation.
    const double alpha = clampd(0.02 * weight, 0.01, 0.20);
    const double beta  = clampd(0.002 * weight, 0.0005, 0.05);

    phaseUs += alpha * (candidatePhaseUs - phaseUs);
    driftUsPerSec += beta * (candidateDriftUsPerSec - driftUsPerSec);

    phaseUs = clampd(phaseUs, -1000000.0, 1000000.0);
    driftUsPerSec = clampd(driftUsPerSec, -2000.0, 2000.0);

    quality = 1.0 / (1.0 + fabsd(phaseUs) / 5000.0 + fabsd(driftUsPerSec) / 50.0);
    lastPredictUs = nowUs;
  }
};

// ============================================================
// Peer state
// ============================================================

struct PeerState {
  bool active;
  IPAddress ip;
  uint64_t lastHeardUs;
  uint32_t lastSequence;
  double packetLoss;
  int rssi;

  double avgRttUs;
  double avgThetaUs;

  double lastThetaUs;
  uint64_t lastThetaTimeUs;

  double advertisedPhaseUs;
  double advertisedDriftUsPerSec;
  double advertisedQuality;

  PeerState()
    : active(false),
      ip(0, 0, 0, 0),
      lastHeardUs(0),
      lastSequence(0),
      packetLoss(0.0),
      rssi(0),
      avgRttUs(0.0),
      avgThetaUs(0.0),
      lastThetaUs(0.0),
      lastThetaTimeUs(0),
      advertisedPhaseUs(0.0),
      advertisedDriftUsPerSec(0.0),
      advertisedQuality(0.0) {}
};

// ============================================================
// Pending poll/reply tracking
// ============================================================

struct PendingPoll {
  bool active;
  uint32_t sequence;
  uint64_t t1Us;

  PendingPoll()
    : active(false), sequence(0), t1Us(0) {}
};

struct PendingReply {
  bool active;
  IPAddress targetIP;
  NetworkPacket poll;
  uint64_t rxUs;
  uint64_t sendAtUs;

  PendingReply()
    : active(false), targetIP(0, 0, 0, 0), rxUs(0), sendAtUs(0) {}
};

// ============================================================
// Main decentralized synchronizer
// ============================================================

class DecentralizedTimeSync {
private:
  WiFiUDP udp;

  PeerState peers[NUM_NODES];
  ClockDiscipline localClock;

  PendingPoll pendingPolls[MAX_PENDING_POLLS];
  PendingReply pendingReplies[MAX_PENDING_REPLIES];

  uint8_t nodeId;
  uint32_t txSequence;
  uint32_t lastPollFrameSent;
  uint64_t lastDiagnosticsUs;

  uint64_t rawNowUs() const {
    return (uint64_t)esp_timer_get_time();
  }

  uint64_t networkTimeUs() const {
    const uint64_t now = rawNowUs();
    const double phase = localClock.predictedPhaseUs(now);
    double corrected = (double)now - phase;
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

  bool withinWindow(uint64_t nowUs, uint64_t startUs, uint64_t windowUs) const {
    return (nowUs >= startUs) && ((nowUs - startUs) < windowUs);
  }

  double measurementNoiseUs2(int rssi, double delayUs, double packetLoss) const {
    double r = clampd((double)rssi, -90.0, -30.0);
    double rssiBad = (-30.0 - r) / 60.0; // 0 good -> 1 bad
    double delayBad = clampd(delayUs / 3000.0, 0.0, 1.0);
    double lossBad  = clampd(packetLoss, 0.0, 0.5) / 0.5;

    double stddevUs = 80.0 * (1.0 + 3.0 * rssiBad + 5.0 * lossBad + 2.0 * delayBad);
    if (stddevUs < 10.0) stddevUs = 10.0;
    return stddevUs * stddevUs;
  }

  double packetLossFromSeq(uint32_t currentSeq, uint32_t lastSeq) const {
    if (lastSeq == 0) return 0.0;
    uint32_t delta = currentSeq - lastSeq; // wrap-safe
    if (delta <= 1) return 0.0;
    return (double)(delta - 1) / (double)delta;
  }

  double peerWeight(const PeerState& p, double delayUs) const {
    double w = 1.0;

    // Lower weight for bigger delay and worse loss.
    w /= (1.0 + delayUs / 2000.0);
    w /= (1.0 + p.packetLoss * 10.0);

    // Weight slightly by advertised quality.
    w *= clampd(p.advertisedQuality, 0.05, 1.0);

    return clampd(w, 0.01, 1.0);
  }

  int findPendingPoll(uint32_t sequence) {
    for (uint32_t i = 0; i < MAX_PENDING_POLLS; ++i) {
      if (pendingPolls[i].active && pendingPolls[i].sequence == sequence) {
        return (int)i;
      }
    }
    return -1;
  }

  void registerPendingPoll(uint32_t sequence, uint64_t t1Us) {
    for (uint32_t i = 0; i < MAX_PENDING_POLLS; ++i) {
      if (!pendingPolls[i].active) {
        pendingPolls[i].active = true;
        pendingPolls[i].sequence = sequence;
        pendingPolls[i].t1Us = t1Us;
        return;
      }
    }
    // If full, overwrite the first slot.
    pendingPolls[0].active = true;
    pendingPolls[0].sequence = sequence;
    pendingPolls[0].t1Us = t1Us;
  }

  bool resolvePendingPoll(uint32_t echoSequence, uint64_t &t1UsOut) {
    int idx = findPendingPoll(echoSequence);
    if (idx < 0) return false;
    t1UsOut = pendingPolls[idx].t1Us;
    pendingPolls[idx].active = false;
    return true;
  }

  int allocatePendingReply() {
    for (uint32_t i = 0; i < MAX_PENDING_REPLIES; ++i) {
      if (!pendingReplies[i].active) return (int)i;
    }
    return 0; // overwrite the first if full
  }

  void cleanupExpiredPending(uint64_t nowUs) {
    for (uint32_t i = 0; i < MAX_PENDING_POLLS; ++i) {
      if (pendingPolls[i].active && (nowUs - pendingPolls[i].t1Us) > PENDING_TIMEOUT_US) {
        pendingPolls[i].active = false;
      }
    }

    for (uint32_t i = 0; i < MAX_PENDING_REPLIES; ++i) {
      if (pendingReplies[i].active && (nowUs - pendingReplies[i].rxUs) > PENDING_TIMEOUT_US) {
        pendingReplies[i].active = false;
      }
    }
  }

  void sendPacket(const NetworkPacket& pkt, const IPAddress& targetIP) {
    udp.beginPacket(targetIP, MULTICAST_PORT);
    udp.write((const uint8_t*)&pkt, sizeof(pkt));
    udp.endPacket();
  }

  void sendPoll(uint64_t nowUs, uint32_t frameId) {
    NetworkPacket pkt{};
    pkt.version = 1;
    pkt.type = PKT_POLL;
    pkt.flags = FLAG_TDMA | FLAG_POLL | FLAG_SYNC_VALID;
    pkt.srcNodeId = nodeId;
    pkt.dstNodeId = 0xFF;
    pkt.reserved0 = 0;
    pkt.reserved1 = 0;
    pkt.sequence = ++txSequence;
    pkt.echoSequence = 0;
    pkt.superframeId = frameId;
    pkt.t1Us = nowUs;
    pkt.t2Us = 0;
    pkt.t3Us = 0;
    pkt.phaseUs = (float)localClock.predictedPhaseUs(nowUs);
    pkt.driftUsPerSec = (float)localClock.driftUsPerSec;
    pkt.quality = (float)localClock.quality;
    pkt.rssi = (int16_t)WiFi.RSSI();
    pkt.reserved2 = 0;

    registerPendingPoll(pkt.sequence, nowUs);
    sendPacket(pkt, MULTICAST_IP);

    lastPollFrameSent = frameId;
    digitalWrite(LED_PIN, HIGH);
  }

  void queueReply(const IPAddress& targetIP, const NetworkPacket& pollPkt, uint64_t rxUs) {
    int idx = allocatePendingReply();
    pendingReplies[idx].active = true;
    pendingReplies[idx].targetIP = targetIP;
    pendingReplies[idx].poll = pollPkt;
    pendingReplies[idx].rxUs = rxUs;
    pendingReplies[idx].sendAtUs = rxUs + deterministicReplyDelayUs(pollPkt.srcNodeId, nodeId, pollPkt.sequence);
  }

  void flushReplies(uint64_t nowUs) {
    for (uint32_t i = 0; i < MAX_PENDING_REPLIES; ++i) {
      if (!pendingReplies[i].active) continue;
      if (nowUs < pendingReplies[i].sendAtUs) continue;

      const NetworkPacket pollPkt = pendingReplies[i].poll;

      NetworkPacket reply{};
      reply.version = 1;
      reply.type = PKT_REPLY;
      reply.flags = FLAG_REPLY | FLAG_SYNC_VALID;
      reply.srcNodeId = nodeId;
      reply.dstNodeId = pollPkt.srcNodeId;
      reply.reserved0 = 0;
      reply.reserved1 = 0;
      reply.sequence = ++txSequence;
      reply.echoSequence = pollPkt.sequence;
      reply.superframeId = pollPkt.superframeId;

      reply.t1Us = pollPkt.t1Us;
      reply.t2Us = pendingReplies[i].rxUs;
      reply.t3Us = nowUs;

      // Advertise this node's current estimate.
      reply.phaseUs = (float)localClock.predictedPhaseUs(nowUs);
      reply.driftUsPerSec = (float)localClock.driftUsPerSec;
      reply.quality = (float)localClock.quality;
      reply.rssi = (int16_t)WiFi.RSSI();
      reply.reserved2 = 0;

      sendPacket(reply, pendingReplies[i].targetIP);
      pendingReplies[i].active = false;
    }
  }

  void handleReply(const NetworkPacket& pkt, uint64_t nowUs) {
    uint64_t t1Us = 0;
    if (!resolvePendingPoll(pkt.echoSequence, t1Us)) {
      return;
    }

    const double T1 = (double)t1Us;
    const double T2 = (double)pkt.t2Us;
    const double T3 = (double)pkt.t3Us;
    const double T4 = (double)nowUs;

    // NTP-style estimates.
    // theta = responder phase - our phase
    // delay = total round-trip - responder processing time
    double thetaUs = ((T2 - T1) + (T3 - T4)) * 0.5;
    double delayUs = (T4 - T1) - (T3 - T2);

    if (!isfinite(thetaUs) || !isfinite(delayUs)) return;
    if (delayUs < 0.0) delayUs = 0.0;
    if (fabsd(thetaUs) > 2000000.0) return; // reject wild outliers

    if (pkt.srcNodeId >= NUM_NODES) return;
    PeerState& peer = peers[pkt.srcNodeId];

    // Update peer link stats.
    peer.active = true;
    peer.ip = udp.remoteIP();
    peer.lastHeardUs = nowUs;
    peer.rssi = pkt.rssi;
    peer.packetLoss = packetLossFromSeq(pkt.sequence, peer.lastSequence);
    peer.lastSequence = pkt.sequence;
    peer.avgRttUs = (peer.avgRttUs <= 0.0) ? delayUs : (peer.avgRttUs * 0.8 + delayUs * 0.2);
    peer.avgThetaUs = (peer.avgThetaUs == 0.0) ? thetaUs : (peer.avgThetaUs * 0.8 + thetaUs * 0.2);
    peer.advertisedPhaseUs = pkt.phaseUs;
    peer.advertisedDriftUsPerSec = pkt.driftUsPerSec;
    peer.advertisedQuality = pkt.quality;

    // Relative drift estimate from change in theta over time:
    // d(theta)/dt = peerDrift - ourDrift
    double relDriftUsPerSec = 0.0;
    if (peer.lastThetaTimeUs != 0 && nowUs > peer.lastThetaTimeUs) {
      double dtSec = double(nowUs - peer.lastThetaTimeUs) / 1000000.0;
      if (dtSec > 1e-6) {
        relDriftUsPerSec = (thetaUs - peer.lastThetaUs) / dtSec;
      }
    }

    // Our estimate from this peer:
    // ourPhase = peerPhase - theta
    // ourDrift = peerDrift - d(theta)/dt
    double candidatePhaseUs = (double)pkt.phaseUs - thetaUs;
    double candidateDriftUsPerSec = (double)pkt.driftUsPerSec - relDriftUsPerSec;

    // Weight based on link quality and latency.
    double w = peerWeight(peer, delayUs);

    // Merge into local clock.
    localClock.mergeCandidate(candidatePhaseUs, candidateDriftUsPerSec, w, nowUs);

    // Save for next drift comparison.
    peer.lastThetaUs = thetaUs;
    peer.lastThetaTimeUs = nowUs;

    // Keep a simple quality hint.
    peer.advertisedQuality = (float)(1.0 / (1.0 + delayUs / 2000.0 + peer.packetLoss * 10.0));
  }

  void handlePoll(const NetworkPacket& pkt, uint64_t nowUs) {
    if (pkt.srcNodeId >= NUM_NODES) return;
    if (pkt.srcNodeId == nodeId) return;

    PeerState& peer = peers[pkt.srcNodeId];
    peer.active = true;
    peer.ip = udp.remoteIP();
    peer.lastHeardUs = nowUs;
    peer.rssi = pkt.rssi;
    peer.packetLoss = packetLossFromSeq(pkt.sequence, peer.lastSequence);
    peer.lastSequence = pkt.sequence;
    peer.advertisedPhaseUs = pkt.phaseUs;
    peer.advertisedDriftUsPerSec = pkt.driftUsPerSec;
    peer.advertisedQuality = pkt.quality;

    // Queue a reply rather than sending immediately to reduce collisions.
    queueReply(udp.remoteIP(), pkt, nowUs);
  }

  void processIncomingPackets() {
    int packetSize = 0;
    while ((packetSize = udp.parsePacket()) > 0) {
      if (packetSize != (int)sizeof(NetworkPacket)) {
        while (udp.available()) udp.read();
        continue;
      }

      NetworkPacket pkt{};
      const int readBytes = udp.read((uint8_t*)&pkt, sizeof(pkt));
      if (readBytes != (int)sizeof(pkt)) continue;

      const uint64_t nowUs = rawNowUs();

      if (pkt.srcNodeId == nodeId) {
        continue; // ignore our own multicast echo
      }

      if (pkt.type == PKT_POLL) {
        handlePoll(pkt, nowUs);
      } else if (pkt.type == PKT_REPLY) {
        if (pkt.dstNodeId == nodeId) {
          handleReply(pkt, nowUs);
        }
      }
    }
  }

  void maintainTdmaSchedule() {
    const uint64_t nowUs = rawNowUs();
    const uint64_t netUs = networkTimeUs();
    const uint32_t frameId = currentSuperframeId(netUs);

    const uint64_t mySlotStart = slotStartUs(frameId, nodeId);

    // Send exactly once per superframe in the node's own slot.
    if (frameId != lastPollFrameSent &&
        withinWindow(netUs, mySlotStart, TX_WINDOW_US)) {
      sendPoll(nowUs, frameId);
    } else if ((netUs % SUPERFRAME_US) > TX_WINDOW_US) {
      digitalWrite(LED_PIN, LOW);
    }
  }

  void updatePeerTimeouts(uint64_t nowUs) {
    for (uint8_t i = 0; i < NUM_NODES; ++i) {
      if (!peers[i].active) continue;
      if ((nowUs - peers[i].lastHeardUs) > PEER_TIMEOUT_US) {
        peers[i].active = false;
      }
    }
  }

  void printDiagnostics(uint64_t nowUs) {
    Serial.println();
    Serial.println(F("Decentralized Time Sync Diagnostics"));
    Serial.printf("Node ID: %u\n", (unsigned)nodeId);
    Serial.printf("WiFi RSSI: %d dBm\n", WiFi.RSSI());
    Serial.printf("Raw time: %llu us\n", (unsigned long long)nowUs);
    Serial.printf("Disciplined network time: %llu us\n", (unsigned long long)networkTimeUs());
    Serial.printf("Local phase error: %.2f us\n", localClock.predictedPhaseUs(nowUs));
    Serial.printf("Local drift: %.6f us/s\n", localClock.driftUsPerSec);
    Serial.printf("Local quality: %.4f\n", localClock.quality);
    Serial.printf("Free heap: %u bytes\n", (unsigned)ESP.getFreeHeap());

    for (uint8_t i = 0; i < NUM_NODES; ++i) {
      if (!peers[i].active) continue;
      Serial.printf(
        "Peer %u: RSSI=%d dBm Loss=%.2f%% RTT=%.2f us Theta=%.2f us PeerPhase=%.2f us PeerDrift=%.6f\n",
        (unsigned)(i + 1),
        peers[i].rssi,
        peers[i].packetLoss * 100.0,
        peers[i].avgRttUs,
        peers[i].avgThetaUs,
        peers[i].advertisedPhaseUs,
        peers[i].advertisedDriftUsPerSec
      );
    }
    Serial.println();

    lastDiagnosticsUs = nowUs;
  }

  void reconnectWiFiIfNeeded() {
    if (WiFi.status() == WL_CONNECTED) return;

    static uint64_t lastAttemptUs = 0;
    uint64_t nowUs = rawNowUs();

    if ((nowUs - lastAttemptUs) < 5000000ULL) return;
    lastAttemptUs = nowUs;

    Serial.println(F("WiFi reconnecting..."));
    WiFi.disconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

public:
  DecentralizedTimeSync()
    : nodeId(THIS_NODE_ID),
      txSequence(0),
      lastPollFrameSent(0xFFFFFFFFUL),
      lastDiagnosticsUs(0) {}

  void begin() {
    Serial.begin(115200);
    delay(200);

    if (nodeId >= NUM_NODES) {
      nodeId = 0;
    }

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.persistent(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print(F("Connecting to WiFi"));
    uint64_t startUs = rawNowUs();
    while (WiFi.status() != WL_CONNECTED && (rawNowUs() - startUs) < 15000000ULL) {
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

    if (!udp.beginMulticast(MULTICAST_IP, MULTICAST_PORT)) {
      Serial.println(F("Failed to join multicast group."));
    } else {
      Serial.println(F("Joined multicast group."));
    }

    localClock.lastPredictUs = rawNowUs();
    peers[nodeId].active = true;
    peers[nodeId].lastHeardUs = rawNowUs();
    lastDiagnosticsUs = rawNowUs();
  }

  void update() {
    reconnectWiFiIfNeeded();

    const uint64_t nowUs = rawNowUs();
    localClock.predictTo(nowUs);

    processIncomingPackets();
    flushReplies(nowUs);
    maintainTdmaSchedule();

    updatePeerTimeouts(nowUs);
    cleanupExpiredPending(nowUs);

    if ((nowUs - lastDiagnosticsUs) >= DIAGNOSTIC_PERIOD_US) {
      printDiagnostics(nowUs);
    }

    delay(1); // yield to WiFi stack
  }

  int64_t getNetworkTimeUs() const {
    const uint64_t nowUs = rawNowUs();
    const double phase = localClock.predictedPhaseUs(nowUs);
    double corrected = (double)nowUs - phase;
    if (corrected < 0.0) corrected = 0.0;
    return (int64_t)llround(corrected);
  }
};

// ============================================================
// Arduino entry points
// ============================================================

DecentralizedTimeSync timeSync;

void setup() {
  timeSync.begin();
}

void loop() {
  timeSync.update();
}
