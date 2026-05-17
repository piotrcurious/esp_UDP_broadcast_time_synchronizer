#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

// ============================================================
// Configuration
// ============================================================

static const char* WIFI_SSID     = "YourSSID";
static const char* WIFI_PASSWORD = "YourPassword";

static constexpr uint8_t  NUM_NODES = 4;
static constexpr uint8_t  LED_PIN    = 2;     // Built-in LED on many ESP32 boards

// Set this per node at compile time if possible:
// 0..NUM_NODES-1
#ifndef NODE_ID
#define NODE_ID 0
#endif

// Multicast configuration
static const IPAddress MULTICAST_IP(224, 0, 0, 1);
static constexpr uint16_t MULTICAST_PORT = 7777;

// Timing
static constexpr uint32_t INITIAL_PACKET_INTERVAL = 1000;  // ms
static constexpr uint32_t MIN_PACKET_INTERVAL     = 100;   // ms
static constexpr uint32_t MAX_PACKET_INTERVAL     = 5000;  // ms
static constexpr uint32_t NODE_TIMEOUT            = 5000;  // ms
static constexpr uint32_t DIAGNOSTIC_PERIOD       = 5000;  // ms

// Packet flags
static constexpr uint8_t FLAG_ACTIVE      = 0x01;
static constexpr uint8_t FLAG_SYNC_VALID   = 0x02;
static constexpr uint8_t FLAG_DIAGNOSTIC   = 0x04;

// ============================================================
// Helpers
// ============================================================

static inline double clampd(double v, double lo, double hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline uint32_t clampu32(uint32_t v, uint32_t lo, uint32_t hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// ============================================================
// Packet format
// ============================================================

#pragma pack(push, 1)
struct NetworkPacket {
  uint32_t timestampMs;   // Sender millis() at transmit time
  uint32_t sequence;      // Sender sequence number
  int16_t  rssi;          // Sender RSSI
  uint16_t reservedRtt;   // Reserved (not a true RTT in multicast)
  float    syncQuality;   // 0..1-ish quality indicator
  float    timeOffsetMs;  // Estimated time offset vs network
  float    driftRate;     // Estimated drift rate
  uint8_t  nodeId;       // Sender ID
  uint8_t  flags;        // Status flags
};
#pragma pack(pop)

static_assert(sizeof(NetworkPacket) == 26, "Unexpected NetworkPacket size");

// ============================================================
// Kalman filter
// State: x = [timeOffset, driftRate]
// Model: x_k = F x_{k-1} + w,  z_k = H x_k + v
// F = [1 dt; 0 1], H = [1 0]
// ============================================================

struct KalmanFilter {
  double timeOffsetMs;
  double driftRate;
  double syncQuality;

  double P[2][2];
  double Q[2][2];
  double R;

  uint32_t lastUpdateMs;

  KalmanFilter()
    : timeOffsetMs(0.0),
      driftRate(0.0),
      syncQuality(0.0),
      R(25.0),
      lastUpdateMs(0) {
    P[0][0] = 1000.0; P[0][1] = 0.0;
    P[1][0] = 0.0;    P[1][1] = 1000.0;

    Q[0][0] = 0.5;    Q[0][1] = 0.0;
    Q[1][0] = 0.0;    Q[1][1] = 0.01;
  }
};

// ============================================================
// Node state
// ============================================================

struct NodeState {
  bool     active;
  IPAddress ip;
  uint32_t lastPacketTime;
  int      signalStrength;
  double   packetLoss;
  uint32_t lastSequence;
  KalmanFilter kf;

  NodeState()
    : active(false),
      ip(0, 0, 0, 0),
      lastPacketTime(0),
      signalStrength(0),
      packetLoss(0.0),
      lastSequence(0),
      kf() {}
};

// ============================================================
// Main synchronizer
// ============================================================

class NetworkTimeSync {
private:
  WiFiUDP udp;
  NodeState nodes[NUM_NODES];

  int8_t   nodeIndex;
  uint32_t lastSendTime;
  uint32_t lastDiagnosticsTime;
  uint32_t packetInterval;
  uint32_t sequenceCounter;

  uint32_t calculateTimeSlot() const {
    return (uint32_t(nodeIndex) * packetInterval) / NUM_NODES;
  }

  double calculateMeasurementNoise(int rssi, double packetLoss) const {
    // More noise when RSSI is weak and packet loss is high.
    const double rssiClamped = clampd((double)rssi, -90.0, -30.0);
    const double rssiBadness  = (-30.0 - rssiClamped) / 60.0; // 0 good -> 1 bad
    const double lossClamped   = clampd(packetLoss, 0.0, 0.5); // cap at 50%
    const double lossBadness   = lossClamped / 0.5;             // 0 good -> 1 bad

    const double factor = 1.0 + 4.0 * rssiBadness + 6.0 * lossBadness;
    return clampd(25.0 * factor, 1.0, 5000.0);
  }

  static double packetLossFromSeq(uint32_t currentSeq, uint32_t lastSeq) {
    if (lastSeq == 0) return 0.0;

    // Works correctly even when the uint32_t sequence number wraps.
    uint32_t delta = currentSeq - lastSeq;
    if (delta <= 1) return 0.0;

    return double(delta - 1) / double(delta);
  }

  void kalmanPredict(KalmanFilter& kf, double dtSec) {
    if (dtSec < 0.0) dtSec = 0.0;

    const double x0 = kf.timeOffsetMs;
    const double x1 = kf.driftRate;

    // Predict state
    kf.timeOffsetMs = x0 + dtSec * x1;
    // driftRate stays the same

    // Predict covariance: P = F P F^T + Q
    const double P00 = kf.P[0][0];
    const double P01 = kf.P[0][1];
    const double P10 = kf.P[1][0];
    const double P11 = kf.P[1][1];

    const double dt  = dtSec;
    const double dt2 = dt * dt;

    kf.P[0][0] = P00 + dt * (P10 + P01) + dt2 * P11 + kf.Q[0][0];
    kf.P[0][1] = P01 + dt * P11 + kf.Q[0][1];
    kf.P[1][0] = P10 + dt * P11 + kf.Q[1][0];
    kf.P[1][1] = P11 + kf.Q[1][1];
  }

  void kalmanUpdate(NodeState& node, double measurementMs, double measurementNoise) {
    KalmanFilter& kf = node.kf;

    // Use local elapsed time to predict between updates.
    const uint32_t now = millis();
    double dtSec = 0.0;
    if (kf.lastUpdateMs != 0) {
      dtSec = double(uint32_t(now - kf.lastUpdateMs)) / 1000.0;
    } else {
      dtSec = double(packetInterval) / 1000.0;
    }

    kalmanPredict(kf, dtSec);

    // Measurement update with H = [1, 0]
    const double P00 = kf.P[0][0];
    const double P01 = kf.P[0][1];
    const double P10 = kf.P[1][0];
    const double P11 = kf.P[1][1];

    const double innovation = measurementMs - kf.timeOffsetMs;
    const double S = P00 + measurementNoise;

    if (S > 1e-9) {
      const double K0 = P00 / S;
      const double K1 = P10 / S;

      kf.timeOffsetMs += K0 * innovation;
      kf.driftRate    += K1 * innovation;

      // P = (I - K H) P, H=[1 0]
      kf.P[0][0] = (1.0 - K0) * P00;
      kf.P[0][1] = (1.0 - K0) * P01;
      kf.P[1][0] = P10 - K1 * P00;
      kf.P[1][1] = P11 - K1 * P01;

      // Keep covariance symmetric.
      const double avg = 0.5 * (kf.P[0][1] + kf.P[1][0]);
      kf.P[0][1] = avg;
      kf.P[1][0] = avg;
    }

    // Simple quality metric from uncertainty.
    kf.syncQuality = 1.0 / (1.0 + sqrt(fabs(kf.P[0][0])));
    kf.lastUpdateMs = now;
  }

  void updatePacketInterval(double avgPacketLoss) {
    if (avgPacketLoss > 0.10) {
      // Increase interval when the network is struggling.
      uint32_t next = (packetInterval * 11UL + 5UL) / 10UL;
      packetInterval = clampu32(next, MIN_PACKET_INTERVAL, MAX_PACKET_INTERVAL);
    } else if (avgPacketLoss < 0.05) {
      // Decrease interval when the network is healthy.
      uint32_t next = (packetInterval * 9UL + 5UL) / 10UL;
      packetInterval = clampu32(next, MIN_PACKET_INTERVAL, MAX_PACKET_INTERVAL);
    }
  }

  void updateStaleNodes(uint32_t now) {
    for (uint8_t i = 0; i < NUM_NODES; ++i) {
      if (nodes[i].active && uint32_t(now - nodes[i].lastPacketTime) > NODE_TIMEOUT) {
        nodes[i].active = false;
      }
    }
  }

  void printDiagnostics(uint32_t now) {
    Serial.println();
    Serial.println(F("Network Diagnostics"));
    Serial.printf("Node ID: %d\n", nodeIndex);
    Serial.printf("Local IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
    Serial.printf("Packet interval: %lu ms\n", (unsigned long)packetInterval);
    Serial.printf("Free heap: %u bytes\n", (unsigned)ESP.getFreeHeap());

    for (uint8_t i = 0; i < NUM_NODES; ++i) {
      if (!nodes[i].active) continue;

      Serial.printf(
        "Node %u: RSSI=%d dBm, Loss=%.2f%%, Sync=%.3f, Offset=%.2f ms, Drift=%.6f\n",
        unsigned(i + 1),
        nodes[i].signalStrength,
        nodes[i].packetLoss * 100.0,
        nodes[i].kf.syncQuality,
        nodes[i].kf.timeOffsetMs,
        nodes[i].kf.driftRate
      );
    }

    Serial.println();
    lastDiagnosticsTime = now;
  }

  void ensureWiFi() {
    if (WiFi.status() == WL_CONNECTED) return;

    static uint32_t lastAttempt = 0;
    const uint32_t now = millis();

    if (uint32_t(now - lastAttempt) < 5000) return;
    lastAttempt = now;

    Serial.println(F("WiFi reconnecting..."));
    WiFi.disconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  void sendStatus(uint32_t now) {
    if (WiFi.status() != WL_CONNECTED) return;

    NetworkPacket packet{};
    packet.timestampMs = now;
    packet.sequence    = ++sequenceCounter;
    packet.rssi        = (int16_t)WiFi.RSSI();
    packet.reservedRtt = 0;
    packet.syncQuality = (float)nodes[nodeIndex].kf.syncQuality;
    packet.timeOffsetMs = (float)nodes[nodeIndex].kf.timeOffsetMs;
    packet.driftRate   = (float)nodes[nodeIndex].kf.driftRate;
    packet.nodeId      = (uint8_t)nodeIndex;
    packet.flags       = FLAG_ACTIVE | FLAG_SYNC_VALID;

    udp.beginPacket(MULTICAST_IP, MULTICAST_PORT);
    udp.write((const uint8_t*)&packet, sizeof(packet));
    udp.endPacket();

    lastSendTime = now;

    // Make this node visible as active locally too.
    nodes[nodeIndex].active = true;
    nodes[nodeIndex].lastPacketTime = now;
    nodes[nodeIndex].signalStrength = packet.rssi;
  }

public:
  NetworkTimeSync()
    : nodeIndex(0),
      lastSendTime(0),
      lastDiagnosticsTime(0),
      packetInterval(INITIAL_PACKET_INTERVAL),
      sequenceCounter(0) {}

  void begin() {
    Serial.println();
    Serial.println(F("Starting NetworkTimeSync..."));

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.persistent(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print(F("Connecting to WiFi"));
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && uint32_t(millis() - start) < 15000) {
      delay(250);
      Serial.print(F("."));
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.print(F("Connected. IP: "));
      Serial.println(WiFi.localIP());
    } else {
      Serial.println(F("WiFi not connected yet; will keep retrying in loop()."));
    }

    nodeIndex = clampu32(NODE_ID, 0, NUM_NODES - 1);

    // Start UDP multicast listener.
    if (!udp.beginMulticast(MULTICAST_IP, MULTICAST_PORT)) {
      Serial.println(F("Failed to join multicast group."));
    } else {
      Serial.println(F("Joined multicast group."));
    }

    // Stagger first send using the slot offset.
    lastSendTime = millis() - calculateTimeSlot();
    lastDiagnosticsTime = millis();
  }

  void update() {
    ensureWiFi();

    const uint32_t now = millis();
    updateStaleNodes(now);

    // Drain all queued packets.
    int packetSize = 0;
    while ((packetSize = udp.parsePacket()) > 0) {
      if (packetSize != (int)sizeof(NetworkPacket)) {
        while (udp.available()) udp.read();
        continue;
      }

      NetworkPacket packet{};
      const int readBytes = udp.read((uint8_t*)&packet, sizeof(packet));
      if (readBytes != (int)sizeof(packet)) continue;

      if (packet.nodeId >= NUM_NODES) continue;
      if (packet.nodeId == (uint8_t)nodeIndex) continue; // Ignore our own multicast echo

      NodeState& node = nodes[packet.nodeId];

      node.active = true;
      node.lastPacketTime = now;
      node.signalStrength = packet.rssi;
      node.ip = udp.remoteIP();

      node.packetLoss = packetLossFromSeq(packet.sequence, node.lastSequence);
      node.lastSequence = packet.sequence;

      const double measurementNoise = calculateMeasurementNoise(
        node.signalStrength,
        node.packetLoss
      );

      // Use the sender's reported offset as a measurement.
      kalmanUpdate(node, packet.timeOffsetMs, measurementNoise);
    }

    // Periodic transmission, staggered per node.
    if (WiFi.status() == WL_CONNECTED &&
        uint32_t(now - lastSendTime) >= packetInterval) {
      sendStatus(now);

      // Adapt interval based on average loss among active peers.
      double avgPacketLoss = 0.0;
      uint8_t activeCount = 0;

      for (uint8_t i = 0; i < NUM_NODES; ++i) {
        if (!nodes[i].active) continue;
        avgPacketLoss += nodes[i].packetLoss;
        ++activeCount;
      }

      if (activeCount > 0) {
        avgPacketLoss /= (double)activeCount;
        updatePacketInterval(avgPacketLoss);
      }
    }

    // LED pulse for the diagnostic node.
    if (nodeIndex == NUM_NODES - 1) {
      const bool ledOn = (uint32_t(now - lastSendTime) % packetInterval) < 100;
      digitalWrite(LED_PIN, ledOn ? HIGH : LOW);

      if (uint32_t(now - lastDiagnosticsTime) >= DIAGNOSTIC_PERIOD) {
        printDiagnostics(now);
      }
    }
  }

  int64_t getNetworkTime() const {
    return (int64_t)millis() + (int64_t)nodes[nodeIndex].kf.timeOffsetMs;
  }

  double getSyncQuality() const {
    return nodes[nodeIndex].kf.syncQuality;
  }
};

// ============================================================
// Arduino entry points
// ============================================================

NetworkTimeSync timeSync;

void setup() {
  Serial.begin(115200);
  delay(200);
  timeSync.begin();
}

void loop() {
  timeSync.update();
  delay(1); // Small yield to WiFi stack
}
