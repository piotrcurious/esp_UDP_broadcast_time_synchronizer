// ============================================================
//  ESP8266 · Distributed TDMA Time-Slot Synchronisation
//  Kalman-filtered OTS adjustment via UDP broadcast
//
//  Each node:
//    1. Broadcasts its current OTS + timestamp at every sendTimer tick.
//    2. Listens for peers and stores one-way delay, RSSI, packet-loss.
//    3. Runs a scalar Kalman filter to estimate its clock offset (timerError).
//    4. Corrects its OTS so that all nodes converge to non-overlapping slots.
//    5. The node whose slot maps to LED_SLOT pulses the built-in LED and
//       prints a full diagnostics report.
// ============================================================

// ── Configuration ─────────────────────────────────────────────
#define GNPSF        1000          // global network packet-send frequency (ms)
#define NODES           4          // total nodes in the mesh
#define UDP_PORT     8888
#define LED_SLOT        0          // slot index that pulses the LED (0..NODES-1)
#define KF_Q         0.01f         // Kalman process noise  (Q)
#define KF_R_BASE    1.0f          // Kalman base measurement noise (R)
#define TX_BUF_SZ      64
#define RX_BUF_SZ      64

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// ── Per-peer state ─────────────────────────────────────────────
struct NodeData {
  String        ip;
  float         ots;              // peer's last reported OTS [0,1)
  unsigned long peerTimer;        // millis value peer embedded in last packet
  int           wsl;              // peer's WiFi RSSI (dBm, negative)
  unsigned long pat;              // packet arrival time on our clock (ms)
  long          pdt;              // pat − peerTimer  (one-way delay estimate, ms)
  unsigned long rxCount;          // packets successfully received from this peer
};

NodeData nodeData[NODES];

WiFiUDP  udp;

// ── Kalman filter state ────────────────────────────────────────
float timerError = 0.0f;          // x : estimated clock offset (ms)
float stateCov   = 1.0f;          // P : state covariance
float measNoise  = KF_R_BASE;     // R : measurement noise (inflated on rollover)

// ── OTS : fractional phase in [0,1) relative to GNPSF ─────────
// timer(ms) = ots * GNPSF gives the phase offset within each cycle.
float         ots;
unsigned long sendTimer;           // absolute millis of next TX

// ── Packet counters ────────────────────────────────────────────
unsigned long txCount = 0;         // packets we have transmitted
unsigned long rxTotal = 0;         // packets received from all peers (excl. own)

// ── millis() guard ─────────────────────────────────────────────
unsigned long prevMillis = 0;

// ── Own last one-way delay – used in the RTT estimate ─────────
// Updated each time we receive any peer packet.
long lastOwnPdt = 0;

// ── New-data flag – Kalman update only fires on fresh data ─────
bool newDataReceived = false;

// ─────────────────────────────────────────────────────────────
// parseRxPacket()
//   Tokenises a "IP,OTS,TIMER,RSSI" buffer in-place.
//   Returns false if the format is invalid.
// ─────────────────────────────────────────────────────────────
bool parseRxPacket(char *buf,
                   String        &outIp,
                   float         &outOts,
                   unsigned long &outTimer,
                   int           &outWsl)
{
  char *tok;
  if (!(tok = strtok(buf,  ","))) return false; outIp    = String(tok);
  if (!(tok = strtok(NULL, ","))) return false; outOts   = atof(tok);
  if (!(tok = strtok(NULL, ","))) return false; outTimer = strtoul(tok, NULL, 10);
  if (!(tok = strtok(NULL, ","))) return false; outWsl   = atoi(tok);
  return true;
}

// ─────────────────────────────────────────────────────────────
// findOrAllocPeer()
//   Returns index into nodeData[] for the given IP.
//   Allocates a new slot if this peer has never been seen.
//   Returns -1 if the array is full.
// ─────────────────────────────────────────────────────────────
int findOrAllocPeer(const String &ip)
{
  for (int i = 0; i < NODES; i++)
    if (nodeData[i].ip == ip) return i;
  for (int i = 0; i < NODES; i++)
    if (nodeData[i].ip == "") {
      nodeData[i].ip      = ip;
      nodeData[i].rxCount = 0;
      nodeData[i].pat     = 0;   // marks "no data yet"
      return i;
    }
  return -1;  // table full
}

// ─────────────────────────────────────────────────────────────
void setup()
{
  Serial.begin(115200);

  // FIX: pinMode was missing in original code
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Seed RNG from floating ADC pin for a unique initial phase
  randomSeed(analogRead(A0));

  // Random initial OTS so nodes don't collide immediately
  ots       = random(0, 1000) / 1000.0f;
  sendTimer = (unsigned long)(ots * GNPSF);

  WiFi.begin("SSID", "PASSWORD");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print('.'); }
  Serial.println();
  Serial.print(F("IP        : ")); Serial.println(WiFi.localIP());
  Serial.print(F("OTS       : ")); Serial.println(ots, 4);
  Serial.print(F("First TX  : ")); Serial.println(sendTimer);

  udp.begin(UDP_PORT);
}

// ─────────────────────────────────────────────────────────────
void loop()
{
  unsigned long currMillis = millis();

  // ── millis() rollover (~49.7 days) ────────────────────────
  // FIX: original code only bumped measNoise but left sendTimer broken.
  if (currMillis < prevMillis) {
    measNoise += 50.0f;                            // relax filter temporarily
    sendTimer  = currMillis + (unsigned long)(ots * GNPSF);
  }
  prevMillis = currMillis;

  // ── Transmit ──────────────────────────────────────────────
  if (currMillis >= sendTimer) {
    char buf[TX_BUF_SZ];
    char otsStr[12];

    // FIX: use dtostrf – sprintf %f is unreliable on some ESP8266 builds
    dtostrf(ots, 1, 6, otsStr);
    snprintf(buf, sizeof(buf), "%s,%s,%lu,%d",
             WiFi.localIP().toString().c_str(),
             otsStr, sendTimer, WiFi.RSSI());

    udp.beginPacket(IPAddress(255, 255, 255, 255), UDP_PORT);
    udp.write(buf);
    udp.endPacket();

    Serial.print(F("TX: ")); Serial.println(buf);
    txCount++;
    sendTimer += GNPSF;
  }

  // ── Receive ───────────────────────────────────────────────
  int pktSz = udp.parsePacket();
  if (pktSz > 0) {
    // FIX: original buffer was 32 bytes – too small; increased to RX_BUF_SZ
    char buf[RX_BUF_SZ] = {0};
    int  len = udp.read(buf, sizeof(buf) - 1);
    buf[len] = '\0';

    Serial.print(F("RX: ")); Serial.println(buf);

    String        rxIp;
    float         rxOts;
    unsigned long rxTimer;
    int           rxWsl;

    if (!parseRxPacket(buf, rxIp, rxOts, rxTimer, rxWsl)) {
      Serial.println(F("RX: parse error, discarding"));
      goto done_rx;
    }

    // FIX: discard own broadcast echo (node receives its own UDP)
    if (rxIp == WiFi.localIP().toString()) goto done_rx;

    {
      unsigned long pat = millis();
      // FIX: cast both sides to signed long before subtracting to avoid
      //      unsigned underflow when peer's clock is slightly ahead of ours
      long pdt = (long)pat - (long)rxTimer;
      lastOwnPdt = pdt;
      rxTotal++;

      int idx = findOrAllocPeer(rxIp);
      if (idx != -1) {
        nodeData[idx].ots       = rxOts;
        nodeData[idx].peerTimer = rxTimer;
        nodeData[idx].wsl       = rxWsl;
        nodeData[idx].pat       = pat;
        nodeData[idx].pdt       = pdt;
        nodeData[idx].rxCount++;
        newDataReceived = true;
      }
    }
    done_rx:;
  }

  // ── Kalman measurement + update ───────────────────────────
  // FIX: original code ran the filter every loop tick even without new data,
  //      which caused the same stale measurement to be applied repeatedly.
  if (!newDataReceived) goto done_kf;
  newDataReceived = false;

  {
    float z[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float H[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    int   peers = 0;
    String localIp = WiFi.localIP().toString();

    for (int i = 0; i < NODES; i++) {
      if (nodeData[i].ip == "" || nodeData[i].ip == localIp) continue;
      if (nodeData[i].pat == 0) continue;       // no packet received yet

      // RTT estimate = peer's one-way delay + our last one-way delay
      // FIX: original referenced undefined 'pdt' from the RX block's inner scope
      float rtt = (float)(nodeData[i].pdt + lastOwnPdt);

      // DPA: deviation of peer's arrival from its ideal slot start
      float dpa = (float)nodeData[i].pat - nodeData[i].ots * GNPSF;

      // Use absolute value of RSSI (it is negative dBm; stronger = larger abs)
      float wsl = fabsf((float)nodeData[i].wsl);

      // Per-peer packet loss: how many of our TX frames did this peer miss?
      // FIX: original used global (expectedPackets - receivedPackets) which
      //      mixed up self-TX counts with per-peer RX counts.
      float pl = (txCount > nodeData[i].rxCount)
                 ? (float)(txCount - nodeData[i].rxCount) : 0.0f;

      // Guard against divide-by-zero
      if (rtt  <  1.0f)  rtt  =  1.0f;
      if (fabsf(dpa) < 1.0f) dpa = (dpa >= 0.0f) ?  1.0f : -1.0f;
      if (wsl  <  1.0f)  wsl  =  1.0f;

      z[0] += rtt;
      z[1] += dpa;
      z[2] += wsl;
      z[3] += pl;

      // Observation matrix coefficients (heuristic):
      //   RTT  is inversely proportional to offset quality
      //   DPA  is directly  proportional to offset error
      //   WSL  stronger signal → less expected error
      //   PL   more loss → larger positive offset contribution
      H[0] -= 1.0f / rtt;
      H[1] += 1.0f / dpa;
      H[2] -= 1.0f / wsl;
      if (pl > 0.5f) H[3] += 1.0f / pl;

      peers++;
    }

    if (peers == 0) goto done_kf;

    // Normalise by number of contributing peers
    float inv = 1.0f / (float)peers;
    for (int i = 0; i < 4; i++) { z[i] *= inv; H[i] *= inv; }

    // ── Kalman prediction step ──────────────────────────────
    // x unchanged (no process input u)
    stateCov += KF_Q;

    // ── Kalman update step ──────────────────────────────────
    // Innovation:  y = z − H·x
    float y[4];
    for (int i = 0; i < 4; i++) y[i] = z[i] - H[i] * timerError;

    // FIX: original code declared and computed 'denom' twice (copy-paste bug)
    // Denominator: S = H·P·Hᵀ + R
    float S = measNoise;
    for (int i = 0; i < 4; i++) S += H[i] * stateCov * H[i];

    // Kalman gain:  K = P·Hᵀ / S
    float K[4];
    for (int i = 0; i < 4; i++) K[i] = stateCov * H[i] / S;

    // State update: x ← x + K·y
    for (int i = 0; i < 4; i++) timerError += K[i] * y[i];

    // Covariance update: P ← P − K·H·P
    for (int i = 0; i < 4; i++) stateCov -= K[i] * H[i] * stateCov;
    if (stateCov < 1e-6f) stateCov = 1e-6f;     // prevent numerical collapse

    // Slowly decay inflated R back toward the baseline
    if (measNoise > KF_R_BASE)
      measNoise = measNoise * 0.95f + KF_R_BASE * 0.05f;

    // ── OTS correction ──────────────────────────────────────
    ots -= timerError / (float)GNPSF;

    // FIX: original wrapped with ±NODES instead of ±1.0 – OTS is in [0,1)
    while (ots <  0.0f) ots += 1.0f;
    while (ots >= 1.0f) ots -= 1.0f;

    Serial.print(F("err=")); Serial.print(timerError, 3);
    Serial.print(F(" P="));  Serial.print(stateCov, 5);
    Serial.print(F(" OTS=")); Serial.println(ots, 4);
  }
  done_kf:;

  // ── LED pulse + diagnostics (only for the LED_SLOT node) ──
  if ((int)(ots * NODES) == LED_SLOT) {
    // Brief pulse – visible on scope / naked eye
    digitalWrite(LED_BUILTIN, HIGH); delay(10); digitalWrite(LED_BUILTIN, LOW);

    String localIp = WiFi.localIP().toString();
    Serial.println(F("────────── Diagnostics ──────────────"));
    Serial.print(F("GNPSF    : ")); Serial.print(GNPSF);   Serial.println(F(" ms"));
    Serial.print(F("OTS slot : ")); Serial.println((int)(ots * NODES));
    Serial.print(F("P        : ")); Serial.println(stateCov, 6);
    Serial.print(F("R        : ")); Serial.println(measNoise, 4);
    Serial.print(F("TX count : ")); Serial.println(txCount);
    Serial.print(F("RX total : ")); Serial.println(rxTotal);
    Serial.println(F("IP             OTS      PDT    RSSI  RX   PktLoss"));
    for (int i = 0; i < NODES; i++) {
      if (nodeData[i].ip == "" || nodeData[i].ip == localIp) continue;
      unsigned long loss = (txCount > nodeData[i].rxCount)
                           ? (txCount - nodeData[i].rxCount) : 0;
      Serial.print(nodeData[i].ip);       Serial.print(F("   "));
      Serial.print(nodeData[i].ots, 4);   Serial.print(F("  "));
      Serial.print(nodeData[i].pdt);      Serial.print(F("ms  "));
      Serial.print(nodeData[i].wsl);      Serial.print(F("dBm  "));
      Serial.print(nodeData[i].rxCount);  Serial.print(F("  "));
      Serial.println(loss);
    }
    Serial.println(F("─────────────────────────────────────"));
  }
}
