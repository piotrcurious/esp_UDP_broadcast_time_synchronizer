#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#define MAX_NODES 10              // Maximum number of nodes
#define BROADCAST_PORT 1234       // UDP port for communication
#define DEFAULT_SLOT_DURATION 100 // Default slot duration in ms
#define INIT_GLOBAL_FREQ 1000     // Initial global frequency in ms
#define EWMA_ALPHA 0.1            // Weight for packet loss smoothing
#define MIN_RSSI -90              // Minimum RSSI for valid communication
#define MAX_RSSI -30              // Maximum RSSI for scaling noise

const char *ssid = "YourSSID";
const char *password = "YourPassword";
WiFiUDP udp;

// Global variables
unsigned long globalFrequency = INIT_GLOBAL_FREQ; // Global frequency in ms
unsigned long lastBroadcast = 0;
unsigned long millisOverflowCounter = 0;
unsigned int localIP;

// Kalman filter state
struct KalmanState {
  float offset;      // Timer offset estimate
  float rssi;        // Signal strength estimate
  float packetLoss;  // Packet loss estimate
  float P[3][3];     // Covariance matrix
};
KalmanState kalman;

// Node state tracking
struct NodeState {
  unsigned long lastSeen;   // Last update timestamp
  float offset;             // Timer offset
  float rssi;               // RSSI value
  float packetLoss;         // Smoothed packet loss
  bool active;              // Node activity status
};
NodeState nodes[MAX_NODES];

// Packet statistics
unsigned long packetsSent = 0;
unsigned long packetsReceived = 0;

// Function prototypes
void initializeWiFi();
void broadcastTimingPacket();
void processPacket(char *packet, int length);
void updateKalmanFilter(NodeState &node, unsigned long now);
float calculateSlotTime(unsigned int nodeID);
void handleMillisOverflow();
void logDiagnostics();
float normalizeRSSI(int rssi);
float computeEWMA(float oldValue, float newValue, float alpha);

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  initializeWiFi();
  udp.begin(BROADCAST_PORT);

  localIP = WiFi.localIP()[3];
  Serial.printf("Node IP: %s\n", WiFi.localIP().toString().c_str());

  // Initialize Kalman filter state
  kalman.offset = 0;
  kalman.rssi = 0;
  kalman.packetLoss = 0;
  memset(kalman.P, 0, sizeof(kalman.P));
}

void loop() {
  unsigned long now = millis();

  // Handle millis() overflow
  handleMillisOverflow();

  // Broadcast timing information in the assigned slot
  unsigned long slotTime = calculateSlotTime(localIP);
  if (now % globalFrequency >= slotTime && now - lastBroadcast >= DEFAULT_SLOT_DURATION) {
    broadcastTimingPacket();
    lastBroadcast = now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // Pulse LED
  }

  // Listen for incoming packets
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char packet[255];
    udp.read(packet, 255);
    processPacket(packet, packetSize);
  }

  // Log diagnostics periodically
  static unsigned long lastLog = 0;
  if (now - lastLog >= 5000) {
    logDiagnostics();
    lastLog = now;
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
                  String(kalman.packetLoss) + "," +
                  String(globalFrequency);
  udp.beginPacket("255.255.255.255", BROADCAST_PORT);
  udp.write(packet.c_str());
  udp.endPacket();
}

void processPacket(char *packet, int length) {
  int nodeID;
  unsigned long timestamp;
  int rssi;
  float loss;
  unsigned long frequency;

  // Parse packet data
  sscanf(packet, "%d,%lu,%d,%f,%lu", &nodeID, &timestamp, &rssi, &loss, &frequency);

  if (nodeID > 0 && nodeID <= MAX_NODES) {
    NodeState &node = nodes[nodeID - 1];

    // Update node state
    unsigned long now = millis() + millisOverflowCounter;
    if (timestamp > now) timestamp -= 0xFFFFFFFF;
    node.offset = (float)(timestamp - now);
    node.rssi = normalizeRSSI(rssi);
    node.packetLoss = computeEWMA(node.packetLoss, loss, EWMA_ALPHA);
    node.lastSeen = now;
    node.active = true;

    // Update global frequency and Kalman filter
    globalFrequency = frequency;
    packetsReceived++;
    updateKalmanFilter(node, now);
  }
}

void updateKalmanFilter(NodeState &node, unsigned long now) {
  // Prediction step (simple constant offset model)
  float predictedOffset = kalman.offset;

  // Measurement update
  float measurementOffset = node.offset;
  float measurementRSSI = node.rssi;
  float measurementLoss = node.packetLoss;

  // Adaptive measurement noise
  float noiseRSSI = max(1.0f, (MAX_RSSI - MIN_RSSI) / max(1.0f, abs(measurementRSSI)));
  float noiseLoss = 10.0f * measurementLoss;

  // Kalman gain
  float K_offset = 0.5;  // Adjust for more responsive or stable behavior
  kalman.offset += K_offset * (measurementOffset - predictedOffset);

  // Update process noise estimates
  kalman.rssi = computeEWMA(kalman.rssi, measurementRSSI, EWMA_ALPHA);
  kalman.packetLoss = computeEWMA(kalman.packetLoss, measurementLoss, EWMA_ALPHA);
}

float calculateSlotTime(unsigned int nodeID) {
  return ((nodeID - 1) * DEFAULT_SLOT_DURATION) % globalFrequency;
}

void handleMillisOverflow() {
  static unsigned long lastMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis < lastMillis) {
    millisOverflowCounter += 0xFFFFFFFF;
  }
  lastMillis = currentMillis;
}

float normalizeRSSI(int rssi) {
  return (float)(rssi - MIN_RSSI) / (MAX_RSSI - MIN_RSSI);
}

float computeEWMA(float oldValue, float newValue, float alpha) {
  return alpha * newValue + (1 - alpha) * oldValue;
}

void logDiagnostics() {
  Serial.printf("Global Frequency: %lu ms\n", globalFrequency);
  Serial.printf("Packets Sent: %lu, Received: %lu, Loss: %.2f%%\n",
                packetsSent, packetsReceived, (1.0f - (float)packetsReceived / packetsSent) * 100);

  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].active) {
      Serial.printf("Node %d - Offset: %.2f ms, RSSI: %.2f, Packet Loss: %.2f%%\n",
                    i + 1, nodes[i].offset, nodes[i].rssi, nodes[i].packetLoss * 100);
    }
  }
}
