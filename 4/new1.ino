#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#define NUM_NODES 4               // Maximum number of nodes
#define BROADCAST_PORT 1234       // UDP port for communication
#define SLOT_DURATION_MS 100      // Default slot duration (ms)
#define GLOBAL_FREQ_MS 1000       // Initial global frequency (ms)
#define KALMAN_GAIN_INIT 0.1      // Initial Kalman gain factor
#define PACKET_LOSS_WINDOW 50     // Packet loss calculation window size

const char *ssid = "YourSSID";
const char *password = "YourPassword";
WiFiUDP udp;

// Global variables
unsigned long globalFrequencyMs = GLOBAL_FREQ_MS;
unsigned long lastBroadcast = 0;
unsigned long millisOverflowCounter = 0; // Tracks `millis()` overflow

struct NodeState {
  float offset;         // Timer offset
  float quality;        // Synchronization quality
  float rtt;            // Round-trip time
  int rssi;             // Signal strength
  float packetLoss;     // Packet loss (rolling average)
  unsigned long lastSeen; // Last update time
};

NodeState nodes[NUM_NODES];
unsigned int localIP;

// Packet statistics
unsigned long packetsSent = 0;
unsigned long packetsReceived = 0;

// Kalman filter state
struct KalmanState {
  float x;    // Timer offset estimate
  float q;    // Synchronization quality
  float rtt;  // Round-trip time
  float P[3][3];  // Covariance matrix
};

KalmanState kalman = {0, 1, 0, {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

void initializeWiFi();
void broadcastTimingPacket();
void processPacket(char *packet, int length);
float calculatePacketLoss();
void updateKalmanFilter(unsigned long now);
void handleMillisOverflow();
unsigned long calculateSlotTime(unsigned int nodeID);
void logDiagnostics();

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  initializeWiFi();

  udp.begin(BROADCAST_PORT);
  localIP = WiFi.localIP()[3];
  Serial.printf("Node IP: %s\n", WiFi.localIP().toString().c_str());
}

void loop() {
  unsigned long now = millis();

  // Handle millis() overflow
  handleMillisOverflow();

  // Broadcast timing information in assigned slot
  unsigned long slotTime = calculateSlotTime(localIP);
  if (now % globalFrequencyMs >= slotTime && now - lastBroadcast >= SLOT_DURATION_MS) {
    broadcastTimingPacket();
    lastBroadcast = now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // Pulse LED
  }

  // Listen for packets
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char packet[255];
    udp.read(packet, 255);
    processPacket(packet, packetSize);
  }

  // Update Kalman filter
  updateKalmanFilter(now);

  // Log diagnostics
  logDiagnostics();
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
                  String(calculatePacketLoss()) + "," +
                  String(globalFrequencyMs);
  udp.beginPacket("255.255.255.255", BROADCAST_PORT);
  udp.write(packet.c_str());
  udp.endPacket();
}

void processPacket(char *packet, int length) {
  // Parse packet: IP, millis, RSSI, packet loss, frequency
  int ip, rssi;
  unsigned long timestamp, frequency;
  float loss;
  sscanf(packet, "%d,%lu,%d,%f,%lu", &ip, &timestamp, &rssi, &loss, &frequency);

  if (ip > 0 && ip <= NUM_NODES) {
    NodeState &node = nodes[ip - 1];
    unsigned long receivedMillis = millis() + millisOverflowCounter;

    // Handle timestamp overflow safely
    if (timestamp > receivedMillis) {
      timestamp -= 0xFFFFFFFF;
    }

    node.offset = (float)(timestamp - receivedMillis);
    node.rssi = rssi;
    node.packetLoss = loss;
    node.lastSeen = millis();
    globalFrequencyMs = frequency;
    packetsReceived++;
  }
}

float calculatePacketLoss() {
  return packetsSent > 0 ? 1.0f - (float)packetsReceived / packetsSent : 0.0f;
}

void updateKalmanFilter(unsigned long now) {
  NodeState &node = nodes[localIP - 1];

  // Predict step
  float predicted_x = kalman.x;
  float predicted_q = kalman.q;
  float predicted_rtt = kalman.rtt;

  // Update step with measurements
  float y_timer = node.offset;  // Timer offset measurement
  float y_rssi = node.rssi;     // RSSI measurement
  float y_loss = node.packetLoss;  // Packet loss measurement

  // Adaptive measurement noise
  float measurementNoise = 10.0f / max(1, abs(y_rssi));
  if (now < lastBroadcast) measurementNoise *= 10.0f;

  // Kalman gains
  float K[3] = {0.1, 0.05, 0.05};
  kalman.x = predicted_x + K[0] * (y_timer - predicted_x);
  kalman.q = predicted_q + K[1] * (y_rssi - predicted_q);
  kalman.rtt = predicted_rtt + K[2] * (y_loss - predicted_rtt);
}

void handleMillisOverflow() {
  static unsigned long lastMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis < lastMillis) {
    millisOverflowCounter += 0xFFFFFFFF;
  }
  lastMillis = currentMillis;
}

unsigned long calculateSlotTime(unsigned int nodeID) {
  return ((nodeID - 1) * SLOT_DURATION_MS) % globalFrequencyMs;
}

void logDiagnostics() {
  Serial.printf("Kalman: x=%.2f, q=%.2f, rtt=%.2f\n", kalman.x, kalman.q, kalman.rtt);
  Serial.printf("Packets Sent: %lu, Received: %lu, Loss: %.2f%%\n",
                packetsSent, packetsReceived, calculatePacketLoss() * 100);
}
