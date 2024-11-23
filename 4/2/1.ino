#include <WiFi.h>
#include <WiFiUdp.h>

// Network configuration
const char* WIFI_SSID = "YourSSID";
const char* WIFI_PASSWORD = "YourPassword";
const int NUM_NODES = 4;  // Default number of nodes
const int LED_PIN = 2;    // Built-in LED

// UDP Multicast configuration
const IPAddress MULTICAST_IP(224, 0, 0, 1);
const uint16_t MULTICAST_PORT = 7777;

// System timing configuration
const unsigned long INITIAL_PACKET_INTERVAL = 1000;  // Start with 1 second between packets
const unsigned long MIN_PACKET_INTERVAL = 100;       // Minimum 100ms between packets
const unsigned long MAX_PACKET_INTERVAL = 5000;      // Maximum 5s between packets

// Packed network packet structure
#pragma pack(push, 1)
struct NetworkPacket {
    uint32_t timestamp;      // Local time when packet was sent
    uint32_t sequence;       // Packet sequence number
    int16_t rssi;           // Signal strength
    uint16_t roundTrip;     // Round trip time
    float syncQuality;      // Synchronization quality metric
    float timeOffset;       // Current time offset estimate
    float driftRate;        // Current drift rate estimate
    uint8_t nodeId;         // Node identifier
    uint8_t flags;          // Status flags
    
    // Constructor for easy initialization
    NetworkPacket() : 
        timestamp(0), sequence(0), rssi(0), roundTrip(0),
        syncQuality(0), timeOffset(0), driftRate(0),
        nodeId(0), flags(0) {}
};
#pragma pack(pop)

// Kalman filter parameters structure
struct KalmanFilter {
    // State variables
    double timeOffset;     // Difference between local and network time
    double driftRate;      // Rate of clock drift
    double syncQuality;    // Overall synchronization quality

    // Covariance matrix (2x2 for time offset and drift rate)
    double P[2][2];
    
    // Process noise covariance
    double Q[2][2];
    
    // Measurement noise
    double R;
    
    KalmanFilter() {
        timeOffset = 0;
        driftRate = 0;
        syncQuality = 0;
        
        // Initialize covariance matrix
        P[0][0] = 1000;  // High initial uncertainty
        P[0][1] = 0;
        P[1][0] = 0;
        P[1][1] = 1000;
        
        // Initialize process noise
        Q[0][0] = 0.1;   // Time offset process noise
        Q[0][1] = 0;
        Q[1][0] = 0;
        Q[1][1] = 0.01;  // Drift rate process noise
        
        R = 100;         // Initial measurement noise
    }
};

// Node state structure
struct NodeState {
    IPAddress ip;
    unsigned long lastPacketTime;
    int signalStrength;
    double packetLoss;
    unsigned long roundTripTime;
    bool active;
    KalmanFilter kf;
    uint32_t lastSequence;  // Track sequence numbers for packet loss calculation
    
    NodeState() : lastPacketTime(0), signalStrength(0), packetLoss(0),
                 roundTripTime(0), active(false), lastSequence(0) {}
};

class NetworkTimeSync {
private:
    WiFiUDP udp;
    NodeState nodes[NUM_NODES];
    int nodeIndex;
    unsigned long lastSendTime;
    unsigned long packetInterval;
    unsigned long networkEpoch;
    uint32_t sequenceCounter;

    // Calculate measurement noise based on signal strength and packet loss
    double calculateMeasurementNoise(int rssi, double packetLoss) {
        double baseNoise = 100;
        double signalFactor = map(constrain(rssi, -90, -30), -90, -30, 100, 1);
        double lossFactor = map(constrain(packetLoss * 100, 0, 50), 0, 50, 1, 100);
        return baseNoise * (signalFactor / 100.0) * (lossFactor / 100.0);
    }

    // Update Kalman filter with new measurement
    void updateKalmanFilter(NodeState& node, double measurement, double measurementNoise) {
        KalmanFilter& kf = node.kf;
        
        // Predict step
        double predicted_time = kf.timeOffset + kf.driftRate;
        
        // Update process noise based on millis() overflow proximity
        unsigned long currentTime = millis();
        if (currentTime > 0xFFFFF000) {  // Close to overflow
            kf.Q[0][0] *= 2;  // Increase process noise
        }
        
        // Update covariance
        kf.P[0][0] = kf.P[0][0] + kf.P[1][0] + kf.P[0][1] + kf.P[1][1] + kf.Q[0][0];
        kf.P[0][1] = kf.P[0][1] + kf.P[1][1] + kf.Q[0][1];
        kf.P[1][0] = kf.P[1][0] + kf.P[1][1] + kf.Q[1][0];
        kf.P[1][1] = kf.P[1][1] + kf.Q[1][1];
        
        // Kalman gain
        double K[2];
        double S = kf.P[0][0] + measurementNoise;
        K[0] = kf.P[0][0] / S;
        K[1] = kf.P[1][0] / S;
        
        // Update state
        double innovation = measurement - predicted_time;
        kf.timeOffset = predicted_time + K[0] * innovation;
        kf.driftRate = kf.driftRate + K[1] * innovation;
        
        // Update covariance
        double P00_temp = kf.P[0][0];
        double P01_temp = kf.P[0][1];
        
        kf.P[0][0] = kf.P[0][0] - K[0] * P00_temp;
        kf.P[0][1] = kf.P[0][1] - K[0] * P01_temp;
        kf.P[1][0] = kf.P[1][0] - K[1] * P00_temp;
        kf.P[1][1] = kf.P[1][1] - K[1] * P01_temp;
        
        // Update sync quality metric
        kf.syncQuality = 1.0 / (1.0 + sqrt(kf.P[0][0]));
    }

    // Calculate optimal time slot
    unsigned long calculateTimeSlot() {
        return (nodeIndex * packetInterval) / NUM_NODES;
    }

    // Calculate packet loss based on sequence numbers
    double calculatePacketLoss(uint32_t currentSeq, uint32_t lastSeq) {
        if (lastSeq == 0) return 0.0;
        uint32_t expected = currentSeq - lastSeq;
        return expected > 1 ? (double)(expected - 1) / expected : 0.0;
    }

public:
    NetworkTimeSync() : nodeIndex(-1), lastSendTime(0), 
                       packetInterval(INITIAL_PACKET_INTERVAL), 
                       networkEpoch(0), sequenceCounter(0) {}

    void begin() {
        // Connect to WiFi
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.println("\nConnected to WiFi");

        // Determine node index from IP address
        IPAddress localIP = WiFi.localIP();
        nodeIndex = localIP[3] - 1;  // Assuming IPs start from x.x.x.1
        
        // Initialize UDP
        udp.beginMulticast(MULTICAST_IP, MULTICAST_PORT);
        
        // Initialize LED pin
        pinMode(LED_PIN, OUTPUT);
    }

    void update() {
        unsigned long currentTime = millis();
        
        // Check for incoming packets
        int packetSize = udp.parsePacket();
        if (packetSize == sizeof(NetworkPacket)) {
            NetworkPacket packet;
            udp.read((uint8_t*)&packet, sizeof(NetworkPacket));
            
            if (packet.nodeId < NUM_NODES) {
                NodeState& node = nodes[packet.nodeId];
                
                // Update node state
                node.active = true;
                node.lastPacketTime = currentTime;
                node.signalStrength = packet.rssi;
                node.roundTripTime = currentTime - packet.timestamp;
                
                // Calculate packet loss
                node.packetLoss = calculatePacketLoss(packet.sequence, node.lastSequence);
                node.lastSequence = packet.sequence;
                
                // Update Kalman filter
                double measurementNoise = calculateMeasurementNoise(
                    node.signalStrength,
                    node.packetLoss
                );
                
                updateKalmanFilter(
                    node,
                    packet.timeOffset,
                    measurementNoise
                );
            }
        }
        
        // Send our status if it's our time slot
        unsigned long timeSlot = calculateTimeSlot();
        if (currentTime - lastSendTime >= packetInterval) {
            NetworkPacket packet;
            packet.timestamp = currentTime;
            packet.sequence = ++sequenceCounter;
            packet.rssi = WiFi.RSSI();
            packet.roundTrip = 0;  // Will be calculated by receiver
            packet.syncQuality = nodes[nodeIndex].kf.syncQuality;
            packet.timeOffset = nodes[nodeIndex].kf.timeOffset;
            packet.driftRate = nodes[nodeIndex].kf.driftRate;
            packet.nodeId = nodeIndex;
            packet.flags = 0;  // Reserved for future use
            
            // Send packet
            udp.beginPacket(MULTICAST_IP, MULTICAST_PORT);
            udp.write((uint8_t*)&packet, sizeof(NetworkPacket));
            udp.endPacket();
            
            lastSendTime = currentTime;
            
            // Update packet interval based on network conditions
            double avgPacketLoss = 0;
            int activeNodes = 0;
            for (int i = 0; i < NUM_NODES; i++) {
                if (nodes[i].active) {
                    avgPacketLoss += nodes[i].packetLoss;
                    activeNodes++;
                }
            }
            
            if (activeNodes > 0) {
                avgPacketLoss /= activeNodes;
                // Adjust packet interval using gradient descent
                if (avgPacketLoss > 0.1) {  // Too much loss
                    packetInterval = min(packetInterval * 1.1, MAX_PACKET_INTERVAL);
                } else if (avgPacketLoss < 0.05) {  // Low loss
                    packetInterval = max(packetInterval * 0.9, MIN_PACKET_INTERVAL);
                }
            }
        }
        
        // Update LED if we're the diagnostic node (last node)
        if (nodeIndex == NUM_NODES - 1) {
            // Pulse LED at network frequency
            digitalWrite(LED_PIN, (currentTime % packetInterval) < 100);
            
            // Print diagnostics
            if (currentTime % 5000 == 0) {  // Every 5 seconds
                Serial.println("\nNetwork Diagnostics:");
                Serial.printf("Packet Interval: %lu ms\n", packetInterval);
                for (int i = 0; i < NUM_NODES; i++) {
                    if (nodes[i].active) {
                        Serial.printf("Node %d: RSSI: %d dBm, Loss: %.2f%%, Sync: %.2f\n",
                            i + 1,
                            nodes[i].signalStrength,
                            nodes[i].packetLoss * 100,
                            nodes[i].kf.syncQuality
                        );
                    }
                }
            }
        }
    }
    
    // Get synchronized network time
    unsigned long getNetworkTime() {
        return millis() + (long)nodes[nodeIndex].kf.timeOffset;
    }
};

NetworkTimeSync timeSync;

void setup() {
    Serial.begin(115200);
    timeSync.begin();
}

void loop() {
    timeSync.update();
}
