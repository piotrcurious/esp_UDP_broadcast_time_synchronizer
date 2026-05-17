#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

constexpr uint8_t  MAX_NODES           = 4;
constexpr uint16_t BROADCAST_PORT      = 1234;
constexpr uint32_t DEFAULT_GLOBAL_FREQ = 1000;   // ms
constexpr uint32_t NODE_TIMEOUT        = 5000;   // ms
constexpr float    RSSI_WEIGHT         = 0.10f;
constexpr float    PACKET_LOSS_WEIGHT   = 0.10f;
constexpr uint32_t DIAGNOSTICS_INTERVAL = 5000;  // ms

// Set this uniquely on each device: 1..MAX_NODES
constexpr uint8_t NODE_ID = 1;

const char *ssid     = "YourSSID";
const char *password = "YourPassword";

WiFiUDP udp;

struct NodeState {
  uint32_t lastSeen = 0;
  uint32_t lastPacketID = 0;
  float offset = 0.0f;
  float rssi = 0.0f;
  float packetLoss = 0.0f;
  bool active = false;
  bool hasPacket = false;
};

struct KalmanState {
  float offset = 0.0f;
  float variance = 1.0f;
  float processNoise = 0.1f;
};

NodeState nodes[MAX_NODES];
KalmanState kalman;

// Global state
uint32_t globalFrequency = DEFAULT_GLOBAL_FREQ;
uint32_t nextBroadcastAt = 0;
uint32_t diagnosticsTimer = 0;
uint32_t packetsSent = 0;
uint32_t packetsReceived = 0;

static inline bool timeReached(uint32_t now, uint32_t target) {
  return (int32_t)(now - target) >= 0;
}

static inline float smoothValue(float oldValue, float newValue, float weight) {
  if (weight < 0.0f) weight = 0.0f;
  if (weight > 1.0f) weight = 1.0f;
  return oldValue + weight * (newValue - oldValue);
}

uint8_t activeNodeCount() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < MAX_NODES; ++i) {
    if (nodes[i].active) count++;
  }
  return count;
}

uint32_t calculateSlotTime(uint8_t nodeID) {
  uint8_t activeNodes = activeNodeCount();
  if (activeNodes == 0) activeNodes = 1;

  uint32_t slot = globalFrequency / activeNodes;
  if (slot == 0) slot = 1;

  if (nodeID < 1) nodeID = 1;
  if (nodeID > MAX_NODES) nodeID = MAX_NODES;

  return ((uint32_t)(nodeID - 1) * slot) % globalFrequency;
}

void updateKalmanFilter(NodeState &node) {
  const float predictedOffset = kalman.offset;

  // Stronger penalty for weak RSSI (more negative RSSI => noisier measurement)
  float signalPenalty = constrain((float)(-node.rssi), 0.0f, 100.0f);
  float measurementNoise = 1.0f + signalPenalty / 50.0f;

  float K = kalman.variance / (kalman.variance + measurementNoise);

  kalman.offset += K * (node.offset - predictedOffset);
  kalman.variance = (1.0f - K) * kalman.variance;

  kalman.processNoise = smoothValue(kalman.processNoise, node.packetLoss * 10.0f, 0.10f);
  kalman.variance += kalman.processNoise;
}

void broadcastTimingPacket(uint32_t now) {
  packetsSent++;

  char packet[96];
  snprintf(packet, sizeof(packet), "%u,%lu,%d,%lu,%lu",
           (unsigned)NODE_ID,
           (unsigned long)now,
           (int)WiFi.RSSI(),
           (unsigned long)packetsSent,
           (unsigned long)globalFrequency);

  udp.beginPacket(IPAddress(255, 255, 255, 255), BROADCAST_PORT);
  udp.write((const uint8_t *)packet, strlen(packet));
  udp.endPacket();
}

void processIncomingPacket(const char *packet) {
  int nodeID = 0;
  uint32_t timestamp = 0;
  int rssi = 0;
  uint32_t packetID = 0;
  uint32_t frequency = 0;

  // Expected format:
  // nodeID,timestamp,rssi,packetID,frequency
  int fields = sscanf(packet, "%d,%lu,%d,%lu,%lu",
                      &nodeID, &timestamp, &rssi, &packetID, &frequency);

  if (fields != 5) return;
  if (nodeID < 1 || nodeID > MAX_NODES) return;
  if (nodeID == NODE_ID) return; // ignore our own broadcast if it loops back

  NodeState &node = nodes[nodeID - 1];
  uint32_t now = millis();

  node.offset = (float)((int32_t)(timestamp - now));
  node.rssi = smoothValue(node.rssi, (float)rssi, RSSI_WEIGHT);

  if (!node.hasPacket) {
    node.packetLoss = 0.0f;
  } else if (packetID > node.lastPacketID) {
    uint32_t missing = packetID - node.lastPacketID - 1;
    uint32_t span = packetID - node.lastPacketID;
    float lossSample = (span > 0) ? (float)missing / (float)span : 0.0f;
    node.packetLoss = smoothValue(node.packetLoss, lossSample, PACKET_LOSS_WEIGHT);
  }

  node.lastPacketID = packetID;
  node.lastSeen = now;
  node.active = true;
  node.hasPacket = true;

  globalFrequency = (uint32_t)smoothValue((float)globalFrequency, (float)frequency, 0.05f);
  packetsReceived++;

  updateKalmanFilter(node);
}

void deactivateInactiveNodes(uint32_t now) {
  for (uint8_t i = 0; i < MAX_NODES; ++i) {
    if (nodes[i].active && (uint32_t)(now - nodes[i].lastSeen) > NODE_TIMEOUT) {
      nodes[i].active = false;
      Serial.printf("Node %u deactivated due to inactivity\n", (unsigned)(i + 1));
    }
  }
}

void logDiagnostics() {
  Serial.printf("\n--- Diagnostics ---\n");
  Serial.printf("Node ID: %u\n", (unsigned)NODE_ID);
  Serial.printf("Global Frequency: %lu ms\n", (unsigned long)globalFrequency);
  Serial.printf("Packets Sent: %lu, Received: %lu\n",
                (unsigned long)packetsSent, (unsigned long)packetsReceived);
  Serial.printf("Active Nodes: %u\n", (unsigned)activeNodeCount());

  for (uint8_t i = 0; i < MAX_NODES; ++i) {
    if (nodes[i].active) {
      Serial.printf("Node %u - Offset: %.2f ms, RSSI: %.2f, Loss: %.2f%%, LastSeen: %lu\n",
                    (unsigned)(i + 1),
                    nodes[i].offset,
                    nodes[i].rssi,
                    nodes[i].packetLoss * 100.0f,
                    (unsigned long)nodes[i].lastSeen);
    }
  }
}

void initializeWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  WiFi.begin(ssid, password);
  Serial.printf("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Connected. IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  initializeWiFi();

  if (!udp.begin(BROADCAST_PORT)) {
    Serial.println("UDP begin failed");
  } else {
    Serial.printf("UDP listening on port %u\n", BROADCAST_PORT);
  }

  for (uint8_t i = 0; i < MAX_NODES; ++i) {
    nodes[i] = NodeState();
  }

  uint32_t now = millis();
  nextBroadcastAt = now + calculateSlotTime(NODE_ID);
  diagnosticsTimer = now;
}

void loop() {
  uint32_t now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
  }

  // Broadcast on schedule
  if (timeReached(now, nextBroadcastAt)) {
    broadcastTimingPacket(now);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    // Next transmission after one full global period
    nextBroadcastAt = now + globalFrequency;
  }

  // Receive packets
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char packet[128];
    int len = udp.read(packet, sizeof(packet) - 1);
    if (len > 0) {
      packet[len] = '\0';
      processIncomingPacket(packet);
    }
  }

  deactivateInactiveNodes(now);

  if (timeReached(now, diagnosticsTimer + DIAGNOSTICS_INTERVAL)) {
    logDiagnostics();
    diagnosticsTimer = now;
  }
}
