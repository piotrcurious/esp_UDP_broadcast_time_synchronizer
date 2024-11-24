#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#define MAX_NODES 4                // Maximum number of nodes
#define BROADCAST_PORT 1234        // UDP port for communication
#define DEFAULT_GLOBAL_FREQ 1000   // Default global frequency in ms
#define NODE_TIMEOUT 5000          // Timeout for inactive nodes in ms
#define RSSI_WEIGHT 0.1            // Weight for RSSI smoothing
#define PACKET_LOSS_WEIGHT 0.1     // Weight for packet loss smoothing
#define DIAGNOSTICS_INTERVAL 5000  // Interval for diagnostics in ms

const char *ssid = "YourSSID";
const char *password = "YourPassword";
WiFiUDP udp;

// Global variables
unsigned long globalFrequency = DEFAULT_GLOBAL_FREQ;
unsigned long lastBroadcast = 0;
unsigned long diagnosticsTimer = 0;
unsigned long millisOverflowCounter = 0;
unsigned long lastMillis = 0;
unsigned int localIP;

// Node state
struct NodeState {
  unsigned long lastSeen = 0;
  unsigned long lastPacketID = 0;
  float offset = 0;              // Timer offset
  float rssi = 0;                // RSSI value
  float packetLoss = 0;          // Packet loss estimate
  bool active = false;           // Node activity status
};
NodeState nodes[MAX_NODES];

// Kalman filter state
struct KalmanState {
  float offset = 0;       // Estimated time offset
  float variance = 1;     // Variance of the offset
  float processNoise = 0.1;
};
KalmanState kalman;

// Packet counters
unsigned long packetsSent = 0;
unsigned long packetsReceived = 0;

// Function prototypes
void initializeWiFi();
void broadcastTimingPacket();
void processIncomingPacket(char *packet, int length);
void updateKalmanFilter(NodeState &node);
unsigned long calculateSlotTime(unsigned int nodeID);
void handleMillisOverflow();
void logDiagnostics();
float smoothValue(float oldValue, float newValue, float weight);
void deactivateInactiveNodes(unsigned long now);

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  initializeWiFi();
  udp.begin(BROADCAST_PORT);

  localIP = WiFi.localIP()[3];
  Serial.printf("Node IP: %s\n", WiFi.localIP().toString().c_str());

  // Initialize node states
  for (int i = 0; i < MAX_NODES; i++) {
    nodes[i] = NodeState();
  }
}

void loop() {
  unsigned long now = millis();

  // Handle millis overflow
  handleMillisOverflow();

  // Broadcast timing information
  unsigned long slotTime = calculateSlotTime(localIP);
  if ((now % globalFrequency >= slotTime) && (now - lastBroadcast >= globalFrequency / MAX_NODES)) {
    broadcastTimingPacket();
    lastBroadcast = now;

    // Pulse LED for diagnostics
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // Receive and process incoming packets
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char packet[255];
    udp.read(packet, sizeof(packet));
    processIncomingPacket(packet, packetSize);
  }

  // Deactivate inactive nodes
  deactivateInactiveNodes(now);

  // Log diagnostics periodically
  if (now - diagnosticsTimer >= DIAGNOSTICS_INTERVAL) {
    logDiagnostics();
    diagnosticsTimer = now;
  }
}

void initializeWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
}

void broadcastTimingPacket() {
  packetsSent++;
  String packet = String(localIP) + "," +
                  String(millis() + millisOverflowCounter) + "," +
                  String(WiFi.RSSI()) + "," +
                  String(packetsSent) + "," +
                  String(globalFrequency);
  udp.beginPacket("255.255.255.255", BROADCAST_PORT);
  udp.write(packet.c_str());
  udp.endPacket();
}

void processIncomingPacket(char *packet, int length) {
  int nodeID;
  unsigned long timestamp, packetID;
  int rssi;
  unsigned long frequency;

  // Parse incoming packet
  int fields = sscanf(packet, "%d,%lu,%d,%lu,%lu", &nodeID, &timestamp, &rssi, &packetID, &frequency);
  if (fields != 5 || nodeID <= 0 || nodeID > MAX_NODES) {
    Serial.println("Invalid packet received");
    return;
  }

  NodeState &node = nodes[nodeID - 1];
  unsigned long now = millis() + millisOverflowCounter;

  // Handle millis overflow in timestamp
  if (timestamp > now) timestamp -= 0xFFFFFFFF;

  // Update node state
  node.offset = (float)(timestamp - now);
  node.rssi = smoothValue(node.rssi, (float)rssi, RSSI_WEIGHT);
  node.packetLoss = smoothValue(node.packetLoss, (float)(packetID - node.lastPacketID - 1) / packetID, PACKET_LOSS_WEIGHT);
  node.lastPacketID = packetID;
  node.lastSeen = now;
  node.active = true;

  // Update global frequency
  globalFrequency = smoothValue(globalFrequency, frequency, 0.05);
  packetsReceived++;
  updateKalmanFilter(node);
}

void updateKalmanFilter(NodeState &node) {
  // Prediction
  float predictedOffset = kalman.offset;

  // Measurement update
  float measurementOffset = node.offset;
  float measurementNoise = 1.0f / (abs(node.rssi) + 1);

  // Kalman gain
  float K = kalman.variance / (kalman.variance + measurementNoise);

  // Update state
  kalman.offset += K * (measurementOffset - predictedOffset);
  kalman.variance = (1 - K) * kalman.variance;

  // Adjust process noise dynamically
  kalman.processNoise = smoothValue(kalman.processNoise, node.packetLoss * 10, 0.1);
  kalman.variance += kalman.processNoise;
}

unsigned long calculateSlotTime(unsigned int nodeID) {
  unsigned int activeNodes = 0;
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].active) activeNodes++;
  }
  return ((nodeID - 1) * (globalFrequency / activeNodes)) % globalFrequency;
}

void handleMillisOverflow() {
  unsigned long currentMillis = millis();
  if (currentMillis < lastMillis) {
    millisOverflowCounter += 0xFFFFFFFF;
  }
  lastMillis = currentMillis;
}

void deactivateInactiveNodes(unsigned long now) {
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].active && now - nodes[i].lastSeen > NODE_TIMEOUT) {
      nodes[i].active = false;
      Serial.printf("Node %d deactivated due to inactivity\n", i + 1);
    }
  }
}

void logDiagnostics() {
  Serial.printf("Global Frequency: %lu ms\n", globalFrequency);
  Serial.printf("Packets Sent: %lu, Received: %lu\n", packetsSent, packetsReceived);

  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].active) {
      Serial.printf("Node %d - Offset: %.2f ms, RSSI: %.2f, Loss: %.2f%%\n",
                    i + 1, nodes[i].offset, nodes[i].rssi, nodes[i].packetLoss * 100);
    }
  }
}
