#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include <CircularBuffer.h>

// Configuration constants
struct Config {
    static constexpr char WIFI_SSID[] = "YourSSID";
    static constexpr char WIFI_PASSWORD[] = "YourPassword";
    static constexpr uint8_t NUM_NODES = 4;
    static constexpr uint8_t LED_PIN = 2;
    static constexpr IPAddress MULTICAST_IP{224, 0, 0, 1};
    static constexpr uint16_t MULTICAST_PORT = 7777;
    
    // Timing parameters
    static constexpr uint32_t INITIAL_PACKET_INTERVAL = 1000;
    static constexpr uint32_t MIN_PACKET_INTERVAL = 100;
    static constexpr uint32_t MAX_PACKET_INTERVAL = 5000;
    static constexpr uint32_t STATS_INTERVAL = 5000;
    static constexpr uint32_t NODE_TIMEOUT = 10000;
    
    // Kalman filter parameters
    static constexpr double INITIAL_P = 1000.0;
    static constexpr double PROCESS_NOISE_TIME = 0.1;
    static constexpr double PROCESS_NOISE_DRIFT = 0.01;
    static constexpr double BASE_MEASUREMENT_NOISE = 100.0;
    
    // Network parameters
    static constexpr uint8_t MAX_WIFI_RETRIES = 20;
    static constexpr uint8_t HISTORY_SIZE = 32;
    static constexpr uint8_t MIN_SAMPLES_FOR_SYNC = 5;
};

// Packet flags
enum PacketFlags : uint8_t {
    FLAG_SYNC_MASTER = 0x01,
    FLAG_OVERFLOW_SOON = 0x02,
    FLAG_LOW_BATTERY = 0x04,
    FLAG_POOR_WIFI = 0x08
};

// Network statistics structure
struct NetworkStats {
    uint32_t packetsReceived;
    uint32_t packetsSent;
    uint32_t packetsLost;
    uint32_t badPackets;
    int16_t minRssi;
    int16_t maxRssi;
    float avgRoundTrip;
    uint32_t lastStatsReset;
    
    void reset() {
        packetsReceived = 0;
        packetsSent = 0;
        packetsLost = 0;
        badPackets = 0;
        minRssi = 0;
        maxRssi = -100;
        avgRoundTrip = 0;
        lastStatsReset = millis();
    }
};

// Packet structure (packed for network transmission)
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
    uint16_t checksum;      // CRC16 checksum
    
    // Calculate checksum
    uint16_t calculateChecksum() const {
        uint16_t crc = 0xFFFF;
        const uint8_t* data = reinterpret_cast<const uint8_t*>(this);
        for(size_t i = 0; i < sizeof(NetworkPacket) - sizeof(checksum); i++) {
            crc ^= data[i];
            for(uint8_t bit = 0; bit < 8; bit++) {
                if(crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc = crc >> 1;
                }
            }
        }
        return crc;
    }
    
    // Validate packet
    bool isValid() const {
        return calculateChecksum() == checksum;
    }
    
    // Update checksum before sending
    void updateChecksum() {
        checksum = calculateChecksum();
    }
};
#pragma pack(pop)

// Kalman filter implementation
class KalmanFilter {
private:
    double timeOffset;     // State variable: time offset
    double driftRate;      // State variable: drift rate
    double P[2][2];       // Covariance matrix
    double Q[2][2];       // Process noise
    double R;             // Measurement noise
    double syncQuality;   // Synchronization quality metric
    bool initialized;     // Filter initialization flag

public:
    KalmanFilter() : initialized(false) {
        reset();
    }
    
    void reset() {
        timeOffset = 0;
        driftRate = 0;
        syncQuality = 0;
        initialized = false;
        
        // Initialize covariance matrix
        P[0][0] = Config::INITIAL_P;
        P[0][1] = P[1][0] = 0;
        P[1][1] = Config::INITIAL_P;
        
        // Initialize process noise
        Q[0][0] = Config::PROCESS_NOISE_TIME;
        Q[0][1] = Q[1][0] = 0;
        Q[1][1] = Config::PROCESS_NOISE_DRIFT;
        
        R = Config::BASE_MEASUREMENT_NOISE;
    }
    
    void update(double measurement, double deltaT, double measurementNoise) {
        if (!initialized) {
            timeOffset = measurement;
            initialized = true;
            return;
        }
        
        // Predict step
        double predicted = timeOffset + driftRate * deltaT;
        
        // Update covariance with process noise
        P[0][0] += (Q[0][0] * deltaT * deltaT);
        P[0][1] += (Q[0][1] * deltaT);
        P[1][0] += (Q[1][0] * deltaT);
        P[1][1] += Q[1][1];
        
        // Kalman gain calculation
        double S = P[0][0] + measurementNoise;
        double K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;
        
        // Update state
        double innovation = measurement - predicted;
        timeOffset = predicted + K[0] * innovation;
        driftRate += K[1] * innovation;
        
        // Update covariance
        double P00_temp = P[0][0];
        P[0][0] = (1 - K[0]) * P00_temp;
        P[0][1] = (1 - K[0]) * P[0][1];
        P[1][0] = P[1][0] - K[1] * P00_temp;
        P[1][1] = P[1][1] - K[1] * P[0][1];
        
        // Update sync quality metric
        syncQuality = 1.0 / (1.0 + sqrt(P[0][0]));
    }
    
    // Getters
    double getTimeOffset() const { return timeOffset; }
    double getDriftRate() const { return driftRate; }
    double getSyncQuality() const { return syncQuality; }
    bool isInitialized() const { return initialized; }
};

// Node state tracking
class NodeState {
private:
    CircularBuffer<NetworkPacket, Config::HISTORY_SIZE> packetHistory;
    KalmanFilter kalman;
    IPAddress ip;
    uint32_t lastPacketTime;
    int16_t signalStrength;
    float packetLoss;
    bool active;
    uint32_t lastSequence;
    NetworkStats stats;

public:
    NodeState() : lastPacketTime(0), signalStrength(0), 
                 packetLoss(0), active(false), lastSequence(0) {
        stats.reset();
    }
    
    void updateFromPacket(const NetworkPacket& packet, uint32_t receivedTime) {
        active = true;
        lastPacketTime = receivedTime;
        signalStrength = packet.rssi;
        
        // Calculate packet loss
        if (lastSequence > 0) {
            uint32_t missed = packet.sequence - lastSequence - 1;
            if (missed > 0) {
                stats.packetsLost += missed;
            }
        }
        lastSequence = packet.sequence;
        
        // Update packet history
        packetHistory.push(packet);
        
        // Calculate packet loss ratio
        packetLoss = static_cast<float>(stats.packetsLost) / 
                    (stats.packetsReceived + stats.packetsLost);
        
        // Update statistics
        stats.packetsReceived++;
        stats.minRssi = min(stats.minRssi, packet.rssi);
        stats.maxRssi = max(stats.maxRssi, packet.rssi);
        stats.avgRoundTrip = (stats.avgRoundTrip * (stats.packetsReceived - 1) + 
                             (receivedTime - packet.timestamp)) / stats.packetsReceived;
    }
    
    void updateKalman(double measurement, double deltaT) {
        double measurementNoise = calculateMeasurementNoise();
        kalman.update(measurement, deltaT, measurementNoise);
    }
    
    double calculateMeasurementNoise() const {
        double signalFactor = map(constrain(signalStrength, -90, -30), -90, -30, 100, 1);
        double lossFactor = map(constrain(packetLoss * 100, 0, 50), 0, 50, 1, 100);
        return Config::BASE_MEASUREMENT_NOISE * (signalFactor / 100.0) * (lossFactor / 100.0);
    }
    
    bool isActive(uint32_t currentTime) const {
        return active && (currentTime - lastPacketTime < Config::NODE_TIMEOUT);
    }
    
    // Getters
    const KalmanFilter& getKalman() const { return kalman; }
    const NetworkStats& getStats() const { return stats; }
    int16_t getSignalStrength() const { return signalStrength; }
    float getPacketLoss() const { return packetLoss; }
    uint32_t getLastPacketTime() const { return lastPacketTime; }
};

class NetworkTimeSync {
private:
    WiFiUdP udp;
    NodeState nodes[Config::NUM_NODES];
    int nodeIndex;
    uint32_t lastSendTime;
    uint32_t packetInterval;
    uint32_t sequenceCounter;
    NetworkStats globalStats;
    
    bool initializeWiFi() {
        WiFi.mode(WIFI_STA);
        WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);
        
        uint8_t retries = 0;
        while (WiFi.status() != WL_CONNECTED && retries < Config::MAX_WIFI_RETRIES) {
            delay(500);
            Serial.print(".");
            retries++;
        }
        
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("\nWiFi connection failed!");
            return false;
        }
        
        // Configure WiFi power and performance
        esp_wifi_set_ps(WIFI_PS_NONE);  // Disable power saving
        
        Serial.println("\nConnected to WiFi");
        Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
        return true;
    }
    
    void updatePacketInterval() {
        double avgPacketLoss = 0;
        int activeNodes = 0;
        
        for (int i = 0; i < Config::NUM_NODES; i++) {
            if (nodes[i].isActive(millis())) {
                avgPacketLoss += nodes[i].getPacketLoss();
                activeNodes++;
            }
        }
        
        if (activeNodes > 0) {
            avgPacketLoss /= activeNodes;
            
            // Gradient descent for packet interval
            if (avgPacketLoss > 0.1) {
                packetInterval = min(packetInterval * 1.1, Config::MAX_PACKET_INTERVAL);
            } else if (avgPacketLoss < 0.05 && activeNodes * packetInterval < Config::MAX_PACKET_INTERVAL) {
                packetInterval = max(packetInterval * 0.9, Config::MIN_PACKET_INTERVAL);
            }
        }
    }
    
    void sendNetworkPacket() {
        NetworkPacket packet;
        packet.timestamp = millis();
        packet.sequence = ++sequenceCounter;
        packet.rssi = WiFi.RSSI();
        packet.nodeId = nodeIndex;
        packet.syncQuality = nodes[nodeIndex].getKalman().getSyncQuality();
        packet.timeOffset = nodes[nodeIndex].getKalman().getTimeOffset();
        packet.driftRate = nodes[nodeIndex].getKalman().getDriftRate();
        
        // Set flags
        packet.flags = 0;
        if (packet.timestamp > 0xFFFFF000) packet.flags |= FLAG_OVERFLOW_SOON;
        if (packet.rssi < -75) packet.flags |= FLAG_POOR_WIFI;
        
        packet.updateChecksum();
        
        udp.beginPacket(Config::MULTICAST_IP, Config::MULTICAST_PORT);
        udp.write(reinterpret_cast<uint8_t*>(&packet), sizeof(NetworkPacket));
        udp.endPacket();
        
        globalStats.packetsSent++;
        lastSendTime = packet.timestamp;
    }
    
    void processIncomingPacket() {
        NetworkPacket packet;
        if (udp.read(reinterpret_cast<uint8_t*>(&packet), sizeof(NetworkPacket)) != sizeof(NetworkPacket)) {
            globalStats.badPackets++;
            return;
        }
        
        if (!packet.isValid()) {
            globalStats.badPackets++;
            return;
        }
        
        if (packet.nodeId >= Config::NUM_NODES) {
            globalStats.badPackets++;
            return;
        }
        
        uint32_t receiveTime = millis();
        nodes[packet.nodeId].updateFromPacket(packet, receiveTime);
        
        // Update time synchronization
        double measurement = static_cast<double>(packet.timestamp) + packet.timeOffset;
        double deltaT = static_cast<double>(receiveTime - nodes[packet.nodeId].getLastPacketTime()) / 1000.0;
        nodes[packet.nodeId].updateKalman(measurement, deltaT);
    }
    
    void printDiagnostics() {
        uint32_t currentTime = millis();
        if (currentTime - globalStats.lastStatsReset >= Config::STATS_INTERVAL) {
            Serial.printf("\n=== Network Diagnostics at %lu ms ===\n", currentTime);
            Serial.printf("Packet Interval: %lu ms\n", packetInterval);
            Serial.printf("Global Stats - Sent: %lu, Received: %lu</antArtifact>
