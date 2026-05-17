/*
 * ============================================================
 * DECENTRALIZED DISTRIBUTED CLOCK SYNCHRONIZATION
 * WITH PER-PEER KALMAN FILTERS
 * ============================================================
 *
 * Features
 * --------
 * - Fully decentralized
 * - No master node
 * - NTP-style 4 timestamp exchange
 * - TDMA collision reduction
 * - Per-peer Kalman filters
 * - Drift estimation
 * - Hardware timer based (esp_timer_get_time)
 * - Consensus clock discipline
 * - Packet-loss-aware weighting
 * - Latency-aware weighting
 * - Robust against peer instability
 *
 * ============================================================
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_timer.h>
#include <math.h>

// ============================================================
// CONFIGURATION
// ============================================================

static const char* WIFI_SSID     = "YourSSID";
static const char* WIFI_PASSWORD = "YourPassword";

static constexpr uint8_t NUM_NODES = 4;

#ifndef NODE_ID
#define NODE_ID 0
#endif

static constexpr uint8_t THIS_NODE_ID = NODE_ID;
static constexpr uint8_t LED_PIN = 2;

static const IPAddress MULTICAST_IP(224,0,0,1);
static constexpr uint16_t MULTICAST_PORT = 7777;

// TDMA
static constexpr uint64_t TDMA_SLOT_US   = 60000ULL;
static constexpr uint64_t SUPERFRAME_US  = TDMA_SLOT_US * NUM_NODES;
static constexpr uint64_t TX_WINDOW_US   = 5000ULL;

// Timeouts
static constexpr uint64_t PEER_TIMEOUT_US    = 8000000ULL;
static constexpr uint64_t PENDING_TIMEOUT_US = 3000000ULL;

// Diagnostics
static constexpr uint64_t DIAGNOSTIC_PERIOD_US = 5000000ULL;

// Buffers
static constexpr uint32_t MAX_PENDING_POLLS   = 8;
static constexpr uint32_t MAX_PENDING_REPLIES = 16;

// ============================================================
// HELPERS
// ============================================================

static inline double clampd(double v, double lo, double hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline double fabsd(double v) {
    return (v < 0.0) ? -v : v;
}

// ============================================================
// PACKETS
// ============================================================

enum PacketType : uint8_t {
    PKT_POLL  = 1,
    PKT_REPLY = 2
};

#pragma pack(push,1)
struct NetworkPacket {

    uint8_t version;
    uint8_t type;

    uint8_t srcNodeId;
    uint8_t dstNodeId;

    uint32_t sequence;
    uint32_t echoSequence;

    uint32_t superframeId;

    // NTP timestamps
    uint64_t t1Us;
    uint64_t t2Us;
    uint64_t t3Us;

    // Advertised state
    float advertisedPhaseUs;
    float advertisedDriftUsPerSec;
    float advertisedQuality;

    int16_t rssi;

};
#pragma pack(pop)

// ============================================================
// KALMAN FILTER
// State:
//   x0 = phase offset
//   x1 = drift
//
// Units:
//   phase -> microseconds
//   drift -> microseconds / second
// ============================================================

struct KalmanPeerFilter {

    double phaseUs;
    double driftUsPerSec;

    // Covariance
    double P[2][2];

    // Process noise
    double Q[2][2];

    // Measurement noise
    double R;

    uint64_t lastUpdateUs;

    KalmanPeerFilter() {

        phaseUs = 0.0;
        driftUsPerSec = 0.0;

        P[0][0] = 100000.0;
        P[0][1] = 0.0;
        P[1][0] = 0.0;
        P[1][1] = 1000.0;

        Q[0][0] = 1.0;
        Q[0][1] = 0.0;
        Q[1][0] = 0.0;
        Q[1][1] = 0.01;

        R = 1000.0;

        lastUpdateUs = 0;
    }

    void predict(uint64_t nowUs) {

        if(lastUpdateUs == 0) {
            lastUpdateUs = nowUs;
            return;
        }

        double dt = double(nowUs - lastUpdateUs) / 1000000.0;

        if(dt < 0.0) dt = 0.0;

        // State prediction
        phaseUs += driftUsPerSec * dt;

        // Covariance prediction
        double P00 = P[0][0];
        double P01 = P[0][1];
        double P10 = P[1][0];
        double P11 = P[1][1];

        P[0][0] =
            P00 +
            dt * (P10 + P01) +
            dt * dt * P11 +
            Q[0][0];

        P[0][1] =
            P01 +
            dt * P11 +
            Q[0][1];

        P[1][0] =
            P10 +
            dt * P11 +
            Q[1][0];

        P[1][1] =
            P11 +
            Q[1][1];

        lastUpdateUs = nowUs;
    }

    void update(
        double measuredPhaseUs,
        double measurementNoise
    ) {

        R = measurementNoise;

        // Innovation
        double y = measuredPhaseUs - phaseUs;

        // Innovation covariance
        double S = P[0][0] + R;

        if(S < 1e-9) return;

        // Kalman gain
        double K0 = P[0][0] / S;
        double K1 = P[1][0] / S;

        // State update
        phaseUs += K0 * y;
        driftUsPerSec += K1 * y;

        // Covariance update
        double P00 = P[0][0];
        double P01 = P[0][1];

        P[0][0] -= K0 * P00;
        P[0][1] -= K0 * P01;
        P[1][0] -= K1 * P00;
        P[1][1] -= K1 * P01;
    }

    double quality() const {

        double uncertainty =
            sqrt(fabsd(P[0][0])) +
            sqrt(fabsd(P[1][1])) * 10.0;

        return 1.0 / (1.0 + uncertainty / 1000.0);
    }
};

// ============================================================
// PEER STATE
// ============================================================

struct PeerState {

    bool active;

    IPAddress ip;

    uint64_t lastHeardUs;

    uint32_t lastSequence;

    double packetLoss;

    int rssi;

    double avgRTTUs;

    double lastThetaUs;

    uint64_t lastThetaTimeUs;

    double advertisedPhaseUs;
    double advertisedDriftUsPerSec;
    double advertisedQuality;

    KalmanPeerFilter filter;

    PeerState() {

        active = false;

        ip = IPAddress(0,0,0,0);

        lastHeardUs = 0;

        lastSequence = 0;

        packetLoss = 0.0;

        rssi = 0;

        avgRTTUs = 0.0;

        lastThetaUs = 0.0;
        lastThetaTimeUs = 0;

        advertisedPhaseUs = 0.0;
        advertisedDriftUsPerSec = 0.0;
        advertisedQuality = 0.0;
    }
};

// ============================================================
// PENDING POLLS
// ============================================================

struct PendingPoll {

    bool active;

    uint32_t sequence;

    uint64_t t1Us;

    PendingPoll() {
        active = false;
        sequence = 0;
        t1Us = 0;
    }
};

struct PendingReply {

    bool active;

    IPAddress targetIP;

    NetworkPacket pollPacket;

    uint64_t rxUs;
    uint64_t sendAtUs;

    PendingReply() {
        active = false;
        targetIP = IPAddress(0,0,0,0);
        rxUs = 0;
        sendAtUs = 0;
    }
};

// ============================================================
// MAIN CLASS
// ============================================================

class DistributedClock {

private:

    WiFiUDP udp;

    PeerState peers[NUM_NODES];

    PendingPoll pendingPolls[MAX_PENDING_POLLS];
    PendingReply pendingReplies[MAX_PENDING_REPLIES];

    uint32_t txSequence;
    uint32_t lastPollFrame;

    uint64_t lastDiagnosticsUs;

    // ========================================================
    // LOCAL CONSENSUS CLOCK
    // ========================================================

    double localPhaseUs;
    double localDriftUsPerSec;

    // ========================================================

    uint64_t rawTimeUs() const {
        return (uint64_t)esp_timer_get_time();
    }

    uint64_t networkTimeUs() const {

        uint64_t raw = rawTimeUs();

        double corrected =
            double(raw)
            - localPhaseUs;

        if(corrected < 0.0)
            corrected = 0.0;

        return (uint64_t)llround(corrected);
    }

    // ========================================================
    // TDMA
    // ========================================================

    uint32_t currentSuperframe(uint64_t tUs) const {
        return (uint32_t)(tUs / SUPERFRAME_US);
    }

    uint64_t slotStartUs(
        uint32_t frame,
        uint8_t slot
    ) const {

        return
            uint64_t(frame) * SUPERFRAME_US +
            uint64_t(slot) * TDMA_SLOT_US;
    }

    bool withinWindow(
        uint64_t nowUs,
        uint64_t startUs
    ) const {

        return
            nowUs >= startUs &&
            (nowUs - startUs) < TX_WINDOW_US;
    }

    // ========================================================
    // LOSS
    // ========================================================

    double packetLoss(
        uint32_t current,
        uint32_t previous
    ) {

        if(previous == 0)
            return 0.0;

        uint32_t delta = current - previous;

        if(delta <= 1)
            return 0.0;

        return double(delta - 1) / double(delta);
    }

    // ========================================================
    // NOISE MODEL
    // ========================================================

    double measurementNoise(
        int rssi,
        double delayUs,
        double loss
    ) {

        double r =
            clampd((double)rssi, -90.0, -30.0);

        double rssiBad =
            (-30.0 - r) / 60.0;

        double delayBad =
            clampd(delayUs / 3000.0, 0.0, 1.0);

        double lossBad =
            clampd(loss, 0.0, 0.5) / 0.5;

        double stddev =
            40.0 *
            (1.0 +
            4.0 * rssiBad +
            3.0 * delayBad +
            5.0 * lossBad);

        return stddev * stddev;
    }

    // ========================================================
    // PENDING POLLS
    // ========================================================

    void registerPendingPoll(
        uint32_t seq,
        uint64_t t1Us
    ) {

        for(uint32_t i=0;i<MAX_PENDING_POLLS;i++) {

            if(!pendingPolls[i].active) {

                pendingPolls[i].active = true;
                pendingPolls[i].sequence = seq;
                pendingPolls[i].t1Us = t1Us;

                return;
            }
        }

        pendingPolls[0].active = true;
        pendingPolls[0].sequence = seq;
        pendingPolls[0].t1Us = t1Us;
    }

    bool resolvePendingPoll(
        uint32_t echoSeq,
        uint64_t& t1UsOut
    ) {

        for(uint32_t i=0;i<MAX_PENDING_POLLS;i++) {

            if(
                pendingPolls[i].active &&
                pendingPolls[i].sequence == echoSeq
            ) {

                t1UsOut = pendingPolls[i].t1Us;

                pendingPolls[i].active = false;

                return true;
            }
        }

        return false;
    }

    // ========================================================
    // PACKETS
    // ========================================================

    void sendPacket(
        const NetworkPacket& pkt,
        const IPAddress& target
    ) {

        udp.beginPacket(target, MULTICAST_PORT);
        udp.write((const uint8_t*)&pkt, sizeof(pkt));
        udp.endPacket();
    }

    // ========================================================
    // SEND POLL
    // ========================================================

    void sendPoll(
        uint64_t nowUs,
        uint32_t frame
    ) {

        NetworkPacket pkt{};

        pkt.version = 1;
        pkt.type = PKT_POLL;

        pkt.srcNodeId = THIS_NODE_ID;
        pkt.dstNodeId = 255;

        pkt.sequence = ++txSequence;
        pkt.echoSequence = 0;

        pkt.superframeId = frame;

        pkt.t1Us = nowUs;
        pkt.t2Us = 0;
        pkt.t3Us = 0;

        pkt.advertisedPhaseUs =
            (float)localPhaseUs;

        pkt.advertisedDriftUsPerSec =
            (float)localDriftUsPerSec;

        pkt.advertisedQuality = 1.0f;

        pkt.rssi = WiFi.RSSI();

        registerPendingPoll(
            pkt.sequence,
            nowUs
        );

        sendPacket(pkt, MULTICAST_IP);

        digitalWrite(LED_PIN, HIGH);
    }

    // ========================================================
    // REPLY
    // ========================================================

    void queueReply(
        const IPAddress& targetIP,
        const NetworkPacket& poll,
        uint64_t rxUs
    ) {

        for(uint32_t i=0;i<MAX_PENDING_REPLIES;i++) {

            if(!pendingReplies[i].active) {

                pendingReplies[i].active = true;

                pendingReplies[i].targetIP =
                    targetIP;

                pendingReplies[i].pollPacket =
                    poll;

                pendingReplies[i].rxUs = rxUs;

                pendingReplies[i].sendAtUs =
                    rxUs + 1500 + random(2000);

                return;
            }
        }
    }

    void flushReplies(uint64_t nowUs) {

        for(uint32_t i=0;i<MAX_PENDING_REPLIES;i++) {

            if(!pendingReplies[i].active)
                continue;

            if(nowUs < pendingReplies[i].sendAtUs)
                continue;

            NetworkPacket reply{};

            reply.version = 1;
            reply.type = PKT_REPLY;

            reply.srcNodeId = THIS_NODE_ID;
            reply.dstNodeId =
                pendingReplies[i]
                    .pollPacket
                    .srcNodeId;

            reply.sequence = ++txSequence;

            reply.echoSequence =
                pendingReplies[i]
                    .pollPacket
                    .sequence;

            reply.superframeId =
                pendingReplies[i]
                    .pollPacket
                    .superframeId;

            reply.t1Us =
                pendingReplies[i]
                    .pollPacket
                    .t1Us;

            reply.t2Us =
                pendingReplies[i]
                    .rxUs;

            reply.t3Us =
                nowUs;

            reply.advertisedPhaseUs =
                (float)localPhaseUs;

            reply.advertisedDriftUsPerSec =
                (float)localDriftUsPerSec;

            reply.advertisedQuality = 1.0f;

            reply.rssi = WiFi.RSSI();

            sendPacket(
                reply,
                pendingReplies[i].targetIP
            );

            pendingReplies[i].active = false;
        }
    }

    // ========================================================
    // HANDLE POLL
    // ========================================================

    void handlePoll(
        const NetworkPacket& pkt,
        uint64_t nowUs
    ) {

        if(pkt.srcNodeId >= NUM_NODES)
            return;

        PeerState& peer =
            peers[pkt.srcNodeId];

        peer.active = true;

        peer.ip = udp.remoteIP();

        peer.lastHeardUs = nowUs;

        peer.rssi = pkt.rssi;

        peer.packetLoss =
            packetLoss(
                pkt.sequence,
                peer.lastSequence
            );

        peer.lastSequence =
            pkt.sequence;

        peer.advertisedPhaseUs =
            pkt.advertisedPhaseUs;

        peer.advertisedDriftUsPerSec =
            pkt.advertisedDriftUsPerSec;

        peer.advertisedQuality =
            pkt.advertisedQuality;

        queueReply(
            udp.remoteIP(),
            pkt,
            nowUs
        );
    }

    // ========================================================
    // HANDLE REPLY
    // ========================================================

    void handleReply(
        const NetworkPacket& pkt,
        uint64_t nowUs
    ) {

        uint64_t t1Us;

        if(!resolvePendingPoll(
            pkt.echoSequence,
            t1Us
        )) return;

        if(pkt.srcNodeId >= NUM_NODES)
            return;

        PeerState& peer =
            peers[pkt.srcNodeId];

        // ====================================================
        // NTP ESTIMATION
        // ====================================================

        double T1 = (double)t1Us;
        double T2 = (double)pkt.t2Us;
        double T3 = (double)pkt.t3Us;
        double T4 = (double)nowUs;

        double thetaUs =
            ((T2 - T1) +
            (T3 - T4)) * 0.5;

        double delayUs =
            (T4 - T1) -
            (T3 - T2);

        if(delayUs < 0.0)
            delayUs = 0.0;

        // ====================================================
        // RELATIVE DRIFT
        // ====================================================

        double relativeDrift = 0.0;

        if(
            peer.lastThetaTimeUs != 0 &&
            nowUs > peer.lastThetaTimeUs
        ) {

            double dt =
                double(nowUs -
                peer.lastThetaTimeUs)
                / 1000000.0;

            if(dt > 1e-6) {

                relativeDrift =
                    (thetaUs -
                    peer.lastThetaUs)
                    / dt;
            }
        }

        peer.lastThetaUs =
            thetaUs;

        peer.lastThetaTimeUs =
            nowUs;

        // ====================================================
        // CANDIDATE ESTIMATE
        // ====================================================

        double candidatePhase =
            pkt.advertisedPhaseUs
            - thetaUs;

        double candidateDrift =
            pkt.advertisedDriftUsPerSec
            - relativeDrift;

        // ====================================================
        // KALMAN UPDATE
        // ====================================================

        peer.filter.predict(nowUs);

        double noise =
            measurementNoise(
                pkt.rssi,
                delayUs,
                peer.packetLoss
            );

        peer.filter.update(
            candidatePhase,
            noise
        );

        // ====================================================
        // RTT
        // ====================================================

        if(peer.avgRTTUs == 0.0)
            peer.avgRTTUs = delayUs;
        else
            peer.avgRTTUs =
                peer.avgRTTUs * 0.9 +
                delayUs * 0.1;
    }

    // ========================================================
    // CONSENSUS CLOCK
    // ========================================================

    void updateConsensusClock(uint64_t nowUs) {

        double weightedPhase = 0.0;
        double weightedDrift = 0.0;

        double totalWeight = 0.0;

        for(uint8_t i=0;i<NUM_NODES;i++) {

            if(i == THIS_NODE_ID)
                continue;

            PeerState& p = peers[i];

            if(!p.active)
                continue;

            // Timeout
            if(
                (nowUs - p.lastHeardUs)
                > PEER_TIMEOUT_US
            ) continue;

            double q =
                p.filter.quality();

            double latencyWeight =
                1.0 /
                (1.0 +
                p.avgRTTUs / 2000.0);

            double lossWeight =
                1.0 /
                (1.0 +
                p.packetLoss * 10.0);

            double weight =
                q *
                latencyWeight *
                lossWeight;

            weightedPhase +=
                p.filter.phaseUs * weight;

            weightedDrift +=
                p.filter.driftUsPerSec * weight;

            totalWeight += weight;
        }

        if(totalWeight > 1e-6) {

            weightedPhase /= totalWeight;
            weightedDrift /= totalWeight;

            // Slow consensus convergence
            localPhaseUs +=
                0.02 *
                (weightedPhase - localPhaseUs);

            localDriftUsPerSec +=
                0.005 *
                (weightedDrift -
                localDriftUsPerSec);
        }
    }

    // ========================================================
    // PROCESS
    // ========================================================

    void processIncomingPackets() {

        int packetSize;

        while(
            (packetSize = udp.parsePacket()) > 0
        ) {

            if(
                packetSize !=
                sizeof(NetworkPacket)
            ) {

                while(udp.available())
                    udp.read();

                continue;
            }

            NetworkPacket pkt{};

            int bytes =
                udp.read(
                    (uint8_t*)&pkt,
                    sizeof(pkt)
                );

            if(
                bytes !=
                sizeof(pkt)
            ) continue;

            if(
                pkt.srcNodeId ==
                THIS_NODE_ID
            ) continue;

            uint64_t nowUs =
                rawTimeUs();

            if(pkt.type == PKT_POLL) {

                handlePoll(
                    pkt,
                    nowUs
                );

            } else
            if(pkt.type == PKT_REPLY) {

                if(
                    pkt.dstNodeId ==
                    THIS_NODE_ID
                ) {

                    handleReply(
                        pkt,
                        nowUs
                    );
                }
            }
        }
    }

    // ========================================================
    // TDMA
    // ========================================================

    void maintainTDMA() {

        uint64_t netUs =
            networkTimeUs();

        uint32_t frame =
            currentSuperframe(netUs);

        uint64_t mySlot =
            slotStartUs(
                frame,
                THIS_NODE_ID
            );

        if(
            frame != lastPollFrame &&
            withinWindow(
                netUs,
                mySlot
            )
        ) {

            sendPoll(
                rawTimeUs(),
                frame
            );

            lastPollFrame = frame;
        }
    }

    // ========================================================
    // DIAGNOSTICS
    // ========================================================

    void diagnostics(uint64_t nowUs) {

        Serial.println();
        Serial.println(
            "================================="
        );

        Serial.printf(
            "Node %u\n",
            THIS_NODE_ID
        );

        Serial.printf(
            "Raw Time: %llu us\n",
            (unsigned long long)rawTimeUs()
        );

        Serial.printf(
            "Network Time: %llu us\n",
            (unsigned long long)networkTimeUs()
        );

        Serial.printf(
            "Local Phase: %.3f us\n",
            localPhaseUs
        );

        Serial.printf(
            "Local Drift: %.6f us/s\n",
            localDriftUsPerSec
        );

        Serial.printf(
            "Heap: %u\n",
            ESP.getFreeHeap()
        );

        for(uint8_t i=0;i<NUM_NODES;i++) {

            if(i == THIS_NODE_ID)
                continue;

            PeerState& p =
                peers[i];

            if(!p.active)
                continue;

            Serial.printf(
                "Peer %u | "
                "Phase %.2f us | "
                "Drift %.4f | "
                "RTT %.2f us | "
                "Loss %.2f%% | "
                "Q %.3f\n",

                i,

                p.filter.phaseUs,

                p.filter.driftUsPerSec,

                p.avgRTTUs,

                p.packetLoss * 100.0,

                p.filter.quality()
            );
        }

        Serial.println(
            "================================="
        );

        lastDiagnosticsUs =
            nowUs;
    }

    // ========================================================
    // WIFI
    // ========================================================

    void reconnectWiFi() {

        if(
            WiFi.status() ==
            WL_CONNECTED
        ) return;

        static uint64_t lastTry = 0;

        uint64_t nowUs =
            rawTimeUs();

        if(
            (nowUs - lastTry)
            < 5000000ULL
        ) return;

        lastTry = nowUs;

        Serial.println(
            "WiFi reconnect..."
        );

        WiFi.disconnect(true);

        WiFi.begin(
            WIFI_SSID,
            WIFI_PASSWORD
        );
    }

public:

    DistributedClock() {

        txSequence = 0;

        lastPollFrame = 0xFFFFFFFFUL;

        lastDiagnosticsUs = 0;

        localPhaseUs = 0.0;
        localDriftUsPerSec = 0.0;
    }

    // ========================================================
    // BEGIN
    // ========================================================

    void begin() {

        Serial.begin(115200);

        delay(200);

        pinMode(
            LED_PIN,
            OUTPUT
        );

        digitalWrite(
            LED_PIN,
            LOW
        );

        WiFi.mode(WIFI_STA);

        WiFi.setSleep(false);

        WiFi.begin(
            WIFI_SSID,
            WIFI_PASSWORD
        );

        Serial.print(
            "Connecting"
        );

        uint64_t start =
            rawTimeUs();

        while(
            WiFi.status() != WL_CONNECTED &&
            (rawTimeUs() - start)
            < 15000000ULL
        ) {

            delay(250);

            Serial.print(".");
        }

        Serial.println();

        if(
            WiFi.status() ==
            WL_CONNECTED
        ) {

            Serial.print(
                "IP: "
            );

            Serial.println(
                WiFi.localIP()
            );
        }

        if(
            !udp.beginMulticast(
                MULTICAST_IP,
                MULTICAST_PORT
            )
        ) {

            Serial.println(
                "Multicast failed"
            );

        } else {

            Serial.println(
                "Multicast OK"
            );
        }

        lastDiagnosticsUs =
            rawTimeUs();
    }

    // ========================================================
    // UPDATE
    // ========================================================

    void update() {

        reconnectWiFi();

        uint64_t nowUs =
            rawTimeUs();

        processIncomingPackets();

        flushReplies(nowUs);

        maintainTDMA();

        updateConsensusClock(nowUs);

        if(
            (nowUs - lastDiagnosticsUs)
            >= DIAGNOSTIC_PERIOD_US
        ) {

            diagnostics(nowUs);
        }

        delay(1);
    }
};

// ============================================================
// GLOBAL
// ============================================================

DistributedClock distributedClock;

// ============================================================
// ARDUINO
// ============================================================

void setup() {

    distributedClock.begin();
}

void loop() {

    distributedClock.update();
}
