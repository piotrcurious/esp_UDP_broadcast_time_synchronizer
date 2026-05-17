/*
 * ============================================================
 * ESP-UDP-SYNC v8.0 — Beacon-Aware Precision Sync
 * ============================================================
 *
 * [A] AP Beacon Interval Compensation
 *
 *   A1. WiFi power-save disabled at startup — eliminates the
 *       dominant DTIM-buffering source.  With power-save on,
 *       multicast POLL packets are held by the AP until the
 *       next DTIM beacon (up to 102.4 ms × DTIM_period), so
 *       T2−T1 contains a random sawtooth term Uniform[0,BI].
 *       Disabling sleep collapses this to CSMA/CA contention
 *       jitter (≈ hundreds of µs, not tens of ms).
 *
 *   A2. BeaconTracker — even with power-save off, packets that
 *       transit the AP arrive in mild cadence with the 802.11
 *       beacon (AP processing queues drain after TIM/DTIM, and
 *       CSMA back-off correlates with the beacon slot). The
 *       tracker runs a circular-mean EWMA of rxTime % BI and
 *       measures the residual variance σ²_bcn.  This is fed
 *       directly into the Kalman R so the filter gain correctly
 *       reflects the noise floor that can't be eliminated.
 *
 *   A3. Asymmetric-path detector — POLL is multicast (may be
 *       AP-buffered), REPLY is unicast (delivered promptly).
 *       Per-peer EWMAs of forward delay (T2−T1) and reverse
 *       delay (T4−T3) are maintained.  When fwdEwma exceeds
 *       revEwma by > ASYMMETRY_THRESHOLD_US (a likely DTIM
 *       event made it through despite A1 — e.g. an AP that
 *       ignores the No-Power-Save bit for multicast), θ is
 *       de-biased by half the excess and R is inflated by the
 *       squared asymmetry so the Kalman gain is suppressed.
 *
 *   A4. Beacon-informed R = KF_R_BASE + RTT_variance_term
 *       + 0.5 × σ²_bcn.  The 0.5 factor reflects that beacon
 *       jitter affects both legs additively but the NTP
 *       averaging halves the variance.
 *
 * [B] Enhanced Inter-Node Inference
 *
 *   B1. Reference-node election — each consensus cycle the
 *       peer with the highest advQuality is elected anchor.
 *       The anchor receives ANCHOR_QUALITY_BONUS weight.
 *       This implements a soft spanning-tree: all nodes
 *       transitively inherit the anchor's network-time
 *       through their local Kalman filters, suppressing the
 *       random-walk that flat averaging allows.
 *
 *   B2. Transitive quality weighting — each peer's weight
 *       includes advQuality² so that a well-synced peer
 *       (advQuality≈1) propagates its synchronisation
 *       knowledge with full weight, while a peer that hasn't
 *       converged yet (advQuality≈0.2) contributes only 4%
 *       of a well-synced peer at the same RTT, rather than
 *       20%.  The square makes the discount steeper, which
 *       matters when a partially-synced node would otherwise
 *       pull consensus toward its stale offset.
 *
 *   B3. One-pass Huber-robust consensus — after an initial
 *       weighted mean (pass 1), each peer's residual |φᵢ − μ̂|
 *       is checked against HUBER_THRESHOLD_US.  Peers that
 *       have jumped or badly drifted are down-weighted by
 *       δ/|rᵢ|, and a final mean is computed (pass 2).
 *       This is one step of IRLS with the Huber ρ-function;
 *       it converges fast enough in one iteration given the
 *       small number of peers.
 *
 *   B4. Sequence-number guard — stale/duplicate replies
 *       (same seq as the last accepted reply from that peer)
 *       are discarded before any filter update, preventing
 *       ghost observations from queued UDP packets.
 *
 * [C] Correctness Fixes
 *
 *   C1. Joseph-form covariance update: P = (I−KH)P(I−KH)ᵀ
 *       + KRKᵀ.  The simplified form P = (I−KH)P does not
 *       preserve symmetry; over thousands of updates the off-
 *       diagonal elements diverge, eventually making Kalman
 *       gains nonsensical.  The Joseph form is O(1) extra
 *       work for a 2×2 filter.
 *
 *   C2. µs-precision slew — the v7.3 slew loop accumulated
 *       1 ms quantisation error (millis() resolution) every
 *       integration step.  The new loop uses getRawMicros()
 *       so the phase integrates at full hardware resolution.
 *
 *   C3. predict() dt guard — if no update has been received
 *       for > 10 s (peer went silent), the accumulated dt
 *       would produce an enormous, numerically unstable P
 *       blow-up on re-connection.  The guard resets the
 *       timestamp without advancing the state.
 * ============================================================
 */

#include <Arduino.h>
#if defined(ESP8266)
  #include <ESP8266WiFi.h>
#elif defined(ESP32)
  #include <WiFi.h>
  #include "esp_wifi.h"
#endif
#include <WiFiUdp.h>
#include <vector>
#include <algorithm>
#include <cmath>

// ── Configuration ──────────────────────────────────────────────
#ifndef WIFI_SSID
#define WIFI_SSID "YourSSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "YourPassword"
#endif

#define UDP_PORT              7777
#define MULTICAST_IP          "224.0.0.1"
#define MAX_NODES             16

#define TDMA_SLOT_MS          60
#define SUPERFRAME_MS         (TDMA_SLOT_MS * MAX_NODES)
#define TX_WINDOW_MS          5

// Kalman constants
#define KF_Q_PHASE            10.0      // µs²/s process noise, phase
#define KF_Q_DRIFT            0.001     // (µs/s)²/s process noise, drift
#define KF_R_BASE             1000.0    // µs² base measurement noise

// [A] Beacon model
// 802.11 beacon interval = 100 TU, 1 TU = 1024 µs
#define BEACON_INTERVAL_US    102400ULL
#define BEACON_EWMA_ALPHA     0.08      // slow: ~12 samples time constant
// Forward/reverse asymmetry threshold: suspect DTIM buffering above this
#define ASYMMETRY_THRESHOLD_US 8000.0  // 8 ms

// [B] Consensus tuning
#define HUBER_THRESHOLD_US    5000.0    // µs: Huber clip point
#define ANCHOR_QUALITY_BONUS  2.0       // weight multiplier for reference node
#define RTT_WEIGHT_TAU        500.0     // µs: RTT penalty half-power point
#define MAX_SLEW_DRIFT        5000.0    // µs/s: maximum frequency correction

// ── Packet ─────────────────────────────────────────────────────
enum PacketType : uint8_t { PKT_POLL = 1, PKT_REPLY = 2 };

#pragma pack(push, 1)
struct SyncPacket {
  uint8_t  type;
  uint8_t  srcId;
  uint8_t  dstId;
  uint32_t seq;
  uint64_t t1;
  uint64_t t2;
  uint64_t t3;
  double   advPhase;
  double   advDrift;
  float    advQuality;
};
#pragma pack(pop)

// ── [A2] Beacon Phase Tracker ───────────────────────────────────
//
// Each packet transiting the AP arrives with a phase that is
// loosely correlated to the beacon boundary.  We maintain an
// EWMA circular mean of (rxUs % BEACON_INTERVAL_US).
//
// Residual variance σ²_bcn starts at the uniform-distribution
// prior (BI²/12 ≈ 8.7×10⁸ µs²) and converges to the measured
// circular variance as samples accumulate.  This gives a
// conservative (large R) estimate early on, shrinking as the
// tracker gains confidence.
struct BeaconTracker {
  double   circMean;  // circular EWMA of rxUs % BI
  double   circVar;   // circular variance EWMA
  uint16_t n;         // sample count, capped at 200

  BeaconTracker()
    : circMean(0.0),
      circVar((double)BEACON_INTERVAL_US * (double)BEACON_INTERVAL_US / 12.0),
      n(0) {}

  void update(uint64_t rxUs) {
    double sample = (double)(rxUs % BEACON_INTERVAL_US);
    // Circular delta wrapped to (−BI/2, +BI/2]
    double delta = sample - circMean;
    const double BI = (double)BEACON_INTERVAL_US;
    if (delta >  BI * 0.5) delta -= BI;
    if (delta < -BI * 0.5) delta += BI;
    circMean += BEACON_EWMA_ALPHA * delta;
    if (circMean <  0.0) circMean += BI;
    if (circMean >= BI ) circMean -= BI;
    circVar = (1.0 - BEACON_EWMA_ALPHA) * circVar
            + BEACON_EWMA_ALPHA * delta * delta;
    if (n < 200) n++;
  }

  // σ²_bcn: residual variance after beacon phase is known (µs²).
  // Blends from prior down to measured variance as n grows.
  double residualVariance() const {
    double conv  = (double)n / 100.0;
    if (conv > 1.0) conv = 1.0;
    double prior = (double)BEACON_INTERVAL_US * (double)BEACON_INTERVAL_US / 12.0;
    return prior * (1.0 - conv) + circVar * conv;
  }
};

// ── 2-State Kalman Filter (phase, drift) ───────────────────────
// State:   x = [phase, drift]ᵀ   (µs, µs/s)
// Model:   x[k] = F·x[k-1] + w,  F = [[1,dt],[0,1]]
// Measure: z    = H·x + v,        H = [1, 0]
struct KalmanFilter {
  double   phase;
  double   drift;
  double   P[2][2];
  uint64_t lastUpdateUs;

  KalmanFilter() : phase(0.0), drift(0.0), lastUpdateUs(0) {
    P[0][0] = 1e7; P[0][1] = 0.0;
    P[1][0] = 0.0; P[1][1] = 1e5;
  }

  void predict(uint64_t nowUs) {
    if (lastUpdateUs == 0) { lastUpdateUs = nowUs; return; }
    double dt = (double)(nowUs - lastUpdateUs) * 1e-6;
    // [C3] Guard against stale dt: don't blow up P if peer was silent
    if (dt <= 0.0 || dt > 10.0) { lastUpdateUs = nowUs; return; }

    phase += drift * dt;

    // P = F·P·Fᵀ + Q,  F=[[1,dt],[0,1]],  Q=diag(Q_phase,Q_drift)
    double p00 = P[0][0] + dt*(P[1][0] + P[0][1]) + dt*dt*P[1][1] + KF_Q_PHASE;
    double p01 = P[0][1] + dt*P[1][1];
    double p10 = P[1][0] + dt*P[1][1];
    double p11 = P[1][1] + KF_Q_DRIFT;
    P[0][0]=p00; P[0][1]=p01; P[1][0]=p10; P[1][1]=p11;
    lastUpdateUs = nowUs;
  }

  // [C1] Joseph-form update: P = (I−KH)·P·(I−KH)ᵀ + K·R·Kᵀ
  // This preserves P symmetry; the simplified form P=(I−KH)P does not.
  void update(double z, double R) {
    double y = z - phase;
    double S = P[0][0] + R;
    if (S < 1e-9) return;          // numerical guard
    double K0 = P[0][0] / S;
    double K1 = P[1][0] / S;

    // Slew integration guard: if update is huge, maybe we should just jump
  if (fabs(y) > 100000.0) {
        phase = z;
        // Reset covariance on big jump
        P[0][0] = 1e7; P[1][1] = 1e5;
        P[0][1] = P[1][0] = 0;
        return;
    }

    phase += K0 * y;
    drift += K1 * y;

    // I−KH = [[a, 0], [c, 1]],  a=1−K0,  c=−K1
    double a   = 1.0 - K0;
    double c   = -K1;
    double p00 = P[0][0], p01 = P[0][1], p11 = P[1][1];
    P[0][0] = a*a*p00                     + K0*K0*R;
    P[0][1] = a*(c*p00 + p01)             + K0*K1*R;
    P[1][0] = P[0][1];                    // enforce symmetry
    P[1][1] = c*c*p00 + 2.0*c*p01 + p11  + K1*K1*R;
  }

  // Quality ∈ (0,1]: inverse of combined phase+drift uncertainty.
  float quality() const {
    double unc = sqrt(fabs(P[0][0])) + sqrt(fabs(P[1][1])) * 50.0;
    return (float)(1.0 / (1.0 + unc / 1000.0));
  }
};

// ── Peer ────────────────────────────────────────────────────────
struct Peer {
  uint8_t  id;
  bool     active;
  uint32_t lastHeard;
  KalmanFilter filter;

  double   avgRtt;     // µs EWMA
  double   varRtt;     // µs² variance EWMA

  // [A3] Per-direction delay EWMAs (µs)
  double   fwdEwma;   // forward  T2−T1: multicast POLL path
  double   revEwma;   // reverse  T4−T3: unicast   REPLY path
  bool     dirInit;   // first-sample guard

  // Advertised peer network state
  double   advPhase;
  double   advDrift;
  float    advQuality;

  // [B4] Last accepted sequence number (init to impossible value)
  uint32_t lastSeq;

  explicit Peer(uint8_t _id)
    : id(_id), active(false), lastHeard(0), filter(),
      avgRtt(0.0), varRtt(0.0),
      fwdEwma(0.0), revEwma(0.0), dirInit(false),
      advPhase(0.0), advDrift(0.0), advQuality(0.0f),
      lastSeq(0xFFFFFFFF) {}
};

// ── Global State ─────────────────────────────────────────────────
WiFiUDP          udp;
std::vector<Peer> peers;
uint8_t  myId          = 0;
uint32_t txSeq         = 0;
uint32_t lastPollFrame = 0xFFFFFFFF;

double   localPhase    = 0.0;
double   localDrift    = 0.0;
float    localQuality  = 0.0f;

uint64_t lastSlewUs    = 0;   // [C2] µs-precision slew reference
uint32_t lastKfTime    = 0;
uint32_t lastDiag      = 0;

BeaconTracker beaconTracker;  // [A2] global beacon phase model

#if defined(ESP8266)
uint32_t lastMicrosLow = 0;
uint32_t microsHigh    = 0;
#endif

// ── Hardware Microsecond Counter ─────────────────────────────────
uint64_t getRawMicros() {
#if defined(ESP32)
  return (uint64_t)esp_timer_get_time();
#else
  uint32_t m = micros();
  if (m < lastMicrosLow) microsHigh++;
  lastMicrosLow = m;
  return ((uint64_t)microsHigh << 32) | m;
#endif
}

uint64_t getNetworkMicros() {
  return (uint64_t)((int64_t)getRawMicros() - (int64_t)localPhase);
}

void sendPacket(SyncPacket& pkt, IPAddress addr) {
  udp.beginPacket(addr, UDP_PORT);
  udp.write((uint8_t*)&pkt, sizeof(SyncPacket));
  udp.endPacket();
}

// ── Peer lookup / lazy create ─────────────────────────────────────
Peer* findOrCreatePeer(uint8_t id) {
  for (auto& p : peers) if (p.id == id) return &p;
  if (peers.size() < MAX_NODES) { peers.emplace_back(id); return &peers.back(); }
  return nullptr;
}

// ── Poll Handler ──────────────────────────────────────────────────
void handlePoll(SyncPacket& pkt, uint64_t rxTime) {
  if (pkt.srcId == myId) return;

  // [A2] Every arriving packet informs beacon phase
  beaconTracker.update(rxTime);

  Peer* pp = findOrCreatePeer(pkt.srcId);
  if (!pp) return;
  Peer& p = *pp;

  p.active     = true;
  p.lastHeard  = millis();
  p.advPhase   = pkt.advPhase;
  p.advDrift   = pkt.advDrift;
  p.advQuality = pkt.advQuality;

  SyncPacket reply;
  reply.type       = PKT_REPLY;
  reply.srcId      = myId;
  reply.dstId      = pkt.srcId;
  reply.seq        = pkt.seq;
  reply.t1         = pkt.t1;
  reply.t2         = rxTime;
  reply.t3         = getRawMicros();   // capture AFTER read, minimise skew
  reply.advPhase   = localPhase;
  reply.advDrift   = localDrift;
  reply.advQuality = localQuality;
  sendPacket(reply, udp.remoteIP());
}

// ── Reply Handler ─────────────────────────────────────────────────
void handleReply(SyncPacket& pkt, uint64_t rxTime) {
  if (pkt.dstId != myId) return;

  Peer* pp = findOrCreatePeer(pkt.srcId);
  if (!pp) return;
  Peer& p = *pp;

  // [B4] Sequence-number guard: discard stale/duplicate replies
  if (pkt.seq == p.lastSeq) return;
  p.lastSeq = pkt.seq;

  // [A2] REPLY arrivals also carry beacon cadence signal
  beaconTracker.update(rxTime);

  uint64_t nowUs = getRawMicros();

  int64_t T1 = (int64_t)pkt.t1;
  int64_t T2 = (int64_t)pkt.t2;
  int64_t T3 = (int64_t)pkt.t3;
  int64_t T4 = (int64_t)rxTime;

  double fwdDelay = (double)(T2 - T1);   // POLL path (multicast, AP-relayed)
  double revDelay = (double)(T4 - T3);   // REPLY path (unicast)
  double rtt      = fwdDelay + revDelay;
  double theta    = (fwdDelay - revDelay) * 0.5;  // NTP offset estimator

  // Sanity: ignore improbable round-trips.
  // Note: fwdDelay and revDelay can be negative due to clock offset,
  // but RTT should always be positive and reasonable.
  if (rtt < 0 || rtt > 100000.0) return;

  // [A3] Maintain per-direction EWMAs
  if (!p.dirInit) {
    p.fwdEwma = fwdDelay;
    p.revEwma = revDelay;
    p.dirInit = true;
  } else {
    p.fwdEwma = 0.9*p.fwdEwma + 0.1*fwdDelay;
    p.revEwma = 0.9*p.revEwma + 0.1*revDelay;
  }

  // [A3] Asymmetry check: if fwd >> rev, suspect DTIM buffering on
  //      the multicast POLL path despite power-save being disabled
  //      (some APs buffer multicast regardless; rogue clients, etc.).
  //      De-bias theta and inflate R proportionally to the asymmetry.
  double asymmetry = p.fwdEwma - p.revEwma;
  double rInflate  = 0.0;
  if (asymmetry > ASYMMETRY_THRESHOLD_US) {
    // Forward path delayed by ~asymmetry µs → θ inflated by ~asymmetry/2.
    // Subtract the estimated bias.
    theta   -= asymmetry * 0.5;
    // Add asymmetry² to R: large asymmetry = high uncertainty in correction.
    rInflate = asymmetry * asymmetry;
  }

  // RTT statistics (for R calculation and weight)
  if (p.avgRtt == 0.0) { p.avgRtt = rtt; p.varRtt = rtt * 0.1; }
  else {
    double diff = rtt - p.avgRtt;
    p.avgRtt += 0.1 * diff;
    p.varRtt  = 0.9*p.varRtt + 0.1*diff*diff;
  }

  // [A4] Measurement noise R
  //  = base noise
  //  + RTT jitter term (excess RTT spread → measurement noise)
  //  + 0.5 × σ²_bcn (beacon residual affects both legs; NTP halves it)
  //  + asymmetry inflation term
  double R = KF_R_BASE
           + sqrt(fabs(p.varRtt)) * 5.0
           + beaconTracker.residualVariance() * 0.5
           + rInflate;

  // Bootstrap: if this is the first update from any peer and we are not synced,
  // we can snap our local phase to it.
  if (localQuality < 0.05 && p.advQuality > 0.05) {
      localPhase = pkt.advPhase - theta;
      localQuality = 0.5; // High initial boost to trust the first peer
      localDrift = p.advDrift; // Also inherit drift for faster lock
  }

  // Kalman update with z = what localPhase should be, from this peer:
  //   peer's network time = peer_raw − peer.advPhase
  //   our raw − theta = peer_raw  ⟹  peer_raw = ourRaw − theta
  //   ⟹  network_time = ourRaw − theta − peer.advPhase
  //   ⟹  localPhase   = peer.advPhase − theta   (so ourRaw − localPhase = networkTime)
  p.filter.predict(nowUs);
  p.filter.update(pkt.advPhase - theta, R);

  // Refresh peer state from reply
  p.active     = true;
  p.lastHeard  = millis();
  p.advPhase   = pkt.advPhase;
  p.advDrift   = pkt.advDrift;
  p.advQuality = pkt.advQuality;
}

// ── Consensus Update ──────────────────────────────────────────────
// Called every 100 ms.  Combines per-peer Kalman estimates into
// a single local phase/drift using Huber-robust IRLS with
// reference-node anchoring.
void runConsensus(uint64_t nowUs, uint32_t nowMs) {
  // Expire silent peers
  for (auto& p : peers)
    if (p.active && (nowMs - p.lastHeard > 20000)) p.active = false;

  // [B1] Elect reference anchor: peer with highest advQuality
  uint8_t anchorId      = 0xFF;
  float   anchorQuality = -1.0f; // Start with negative to pick even 0-quality peers
  for (auto& p : peers) {
    if (!p.active) continue;
    if (p.advQuality > anchorQuality) {
      anchorQuality = p.advQuality;
      anchorId      = p.id;
    } else if (p.advQuality == anchorQuality && p.id < anchorId) {
      // Tie-breaker: lowest ID
      anchorId = p.id;
    }
  }

  // Tie-break with ourselves. If we have better or equal quality than any peer,
  // we can potentially be the anchor for the network (stability anchor).
  // Use a slight bias to prefer the network anchor to maintain global consensus.
  if (localQuality > anchorQuality + 0.05) {
      anchorId = myId;
      anchorQuality = localQuality;
  }

  // If no peers are active, we might be the first node or isolated.
  // In a decentralized network, the node with the lowest ID (including us)
  // could act as a virtual anchor to prevent drift of the whole network.
  // For now, we only anchor to peers.

  // Helper: base weight for peer p
  // [B2] w = filter_quality × (advQuality + 0.1)² / (1 + avgRtt/τ)
  //      Adding 0.1 ensures bootstrap works when everyone has 0 quality.
  //      The square penalises poorly-synced peers quadratically:
  //      a peer at advQuality=0.5 gets ~25% weight vs a fully synced one.
  auto baseWeight = [&](const Peer& p) -> double {
    double qAdv = (double)p.advQuality + 0.1;
    double q = (double)p.filter.quality() * qAdv * qAdv;
    double w = q / (1.0 + p.avgRtt / RTT_WEIGHT_TAU);
    if (p.id == anchorId) w *= ANCHOR_QUALITY_BONUS;
    return w;
  };

  // ── Pass 1: initial weighted mean ────────────────────────────
  double wPhase1 = 0.0, wDrift1 = 0.0, wTotal1 = 0.0;
  for (auto& p : peers) {
    if (!p.active) continue;
    p.filter.predict(nowUs);
    double w = baseWeight(p);
    wPhase1 += p.filter.phase * w;
    wDrift1 += p.filter.drift * w;
    wTotal1 += w;
  }

  // Self-contribution to prevent drift if we are already high quality
  // and have no better peers.
  if (localQuality > 0.05) {
      double selfW = (double)localQuality * 1.0; // Weight our own opinion
      wPhase1 += localPhase * selfW;
      wDrift1 += localDrift * selfW;
      wTotal1 += selfW;
  }

  if (wTotal1 < 1e-9) { localQuality = 0.0f; return; }
  double mu0  = wPhase1 / wTotal1;   // initial phase estimate

  // ── [B3] Pass 2: Huber-robust re-weighting ───────────────────
  // Peers whose phase estimate deviates by more than δ from mu0
  // are down-weighted by δ/|residual|, implementing the Huber
  // M-estimator influence function with a hard clip at δ.
  double wPhase2 = 0.0, wDrift2 = 0.0, wTotal2 = 0.0;
  for (auto& p : peers) {
    if (!p.active) continue;
    double w = baseWeight(p);
    double r = fabs(p.filter.phase - mu0);
    if (r > HUBER_THRESHOLD_US) w *= HUBER_THRESHOLD_US / r;
    wPhase2 += p.filter.phase * w;
    wDrift2 += p.filter.drift * w;
    wTotal2 += w;
  }

  // Self-contribution to Pass 2
  if (localQuality > 0.05) {
      double selfW = (double)localQuality * 1.0;
      double r = fabs(localPhase - mu0);
      if (r > HUBER_THRESHOLD_US) selfW *= HUBER_THRESHOLD_US / r;
      wPhase2 += localPhase * selfW;
      wDrift2 += localDrift * selfW;
      wTotal2 += selfW;
  }

  if (wTotal2 < 1e-9) {
      // No consensus possible
      return;
  }
  double robustPhase = wPhase2 / wTotal2;
  double robustDrift = wDrift2 / wTotal2;

  // ── Fuse into local state ────────────────────────────────────
  double diff = robustPhase - localPhase;
  if (fabs(diff) > 10000.0) {
    // Large discontinuity: snap (e.g. cold start or topology change)
    localPhase = robustPhase;
    localDrift = robustDrift;
  } else {
    // Smooth slew: drift integrates phase error + tracks consensus drift
    // Increased gains for faster convergence
    localDrift += 0.1 * (robustDrift - localDrift) + 0.1 * diff;
    // Frequency cap to prevent runaway
    if (localDrift >  MAX_SLEW_DRIFT) localDrift =  MAX_SLEW_DRIFT;
    if (localDrift < -MAX_SLEW_DRIFT) localDrift = -MAX_SLEW_DRIFT;
  }
  localQuality = (float)std::min(1.0, wTotal2);
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print('.'); }
  Serial.println();

  // [A1] Disable 802.11 power-save. This prevents the AP from
  //      buffering multicast frames until the next DTIM beacon,
  //      which would add a Uniform[0, BI×DTIM_period] bias to T2−T1.
  //      With WIFI_NONE_SLEEP the radio stays awake; CSMA/CA
  //      contention jitter (< 1 ms) is all that remains.
#if defined(ESP8266)
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  udp.beginMulticast(WiFi.localIP(), IPAddress(224,0,0,1), UDP_PORT);
  lastMicrosLow = micros();
#elif defined(ESP32)
  esp_wifi_set_ps(WIFI_PS_NONE);
  udp.beginMulticast(IPAddress(224,0,0,1), UDP_PORT);
#endif

  uint8_t mac[6];
  WiFi.macAddress(mac);
  myId = mac[5];

  uint64_t now = getRawMicros();
  lastSlewUs = now;          // [C2] start µs-precision slew reference
  lastKfTime = millis();
  lastDiag   = millis();
}

// ── Main Loop ─────────────────────────────────────────────────────
void loop() {
  uint64_t rawUs = getRawMicros();
  uint32_t now   = millis();

  // [C2] µs-precision phase slew (was millis-precision in v7.3)
  uint64_t slewDtUs = rawUs - lastSlewUs;
  if (slewDtUs > 0) {
    localPhase += localDrift * ((double)slewDtUs * 1e-6);
    lastSlewUs  = rawUs;
  }

  // ── TDMA slot scheduling ──────────────────────────────────────
  uint64_t netUs       = getNetworkMicros();
  uint32_t frame       = (uint32_t)(netUs / ((uint64_t)SUPERFRAME_MS * 1000ULL));
  uint32_t mySlotIdx   = myId % MAX_NODES;
  uint32_t myJitter    = (uint32_t)((myId * 13u) % (TDMA_SLOT_MS / 2));
  uint64_t mySlotStart = (uint64_t)frame * (uint64_t)SUPERFRAME_MS * 1000ULL
                       + (uint64_t)mySlotIdx * (uint64_t)TDMA_SLOT_MS * 1000ULL
                       + (uint64_t)myJitter  * 1000ULL;

  if (frame != lastPollFrame &&
      netUs >= mySlotStart &&
      (netUs - mySlotStart) < (uint64_t)(TX_WINDOW_MS * 1000)) {
    SyncPacket pkt;
    pkt.type       = PKT_POLL;
    pkt.srcId      = myId;
    pkt.dstId      = 0xFF;
    pkt.seq        = txSeq++;
    pkt.t1         = getRawMicros();  // timestamp as late as possible
    pkt.advPhase   = localPhase;
    pkt.advDrift   = localDrift;
    pkt.advQuality = localQuality;
    sendPacket(pkt, IPAddress(224,0,0,1));
    lastPollFrame = frame;
  }

  // ── Receive ───────────────────────────────────────────────────
  int sz;
  int packetsRead = 0;
  while ((sz = udp.parsePacket()) >= (int)sizeof(SyncPacket) && packetsRead < 10) {
    SyncPacket pkt;
    udp.read((uint8_t*)&pkt, sizeof(SyncPacket));
    uint64_t rxTime = getRawMicros();
    if      (pkt.type == PKT_POLL)  handlePoll(pkt, rxTime);
    else if (pkt.type == PKT_REPLY) handleReply(pkt, rxTime);
    packetsRead++;
  }

  // ── Consensus (100 ms cadence) ────────────────────────────────
  if (now - lastKfTime >= 100) {
    lastKfTime = now;
    runConsensus(getRawMicros(), now);
  }

  // ── Diagnostics ───────────────────────────────────────────────
  if (now - lastDiag >= 5000) {
    lastDiag = now;
    Serial.printf(
      "NetTime:%llu  Ph:%.1f  Dr:%.4f  Q:%.2f  Peers:%u  BcnVar:%.0f\n",
      (unsigned long long)getNetworkMicros(),
      localPhase, localDrift, (double)localQuality,
      (unsigned)peers.size(),
      beaconTracker.residualVariance()
    );
  }
}
