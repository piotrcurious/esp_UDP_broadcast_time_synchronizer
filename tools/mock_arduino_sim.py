import random
import math

class Network:
    def __init__(self, loss_rate=0.05, latency_min=5, latency_max=15, jitter=5):
        self.loss_rate = loss_rate
        self.latency_min = latency_min
        self.latency_max = latency_max
        self.jitter = jitter
        self.packets = []
        self.num_nodes = 0
        self.burst_loss_timer = 0

    def broadcast(self, src_id, data, current_real_time):
        for i in range(self.num_nodes):
            if i != src_id:
                self.send(src_id, i, data, current_real_time)

    def send(self, src_id, dst_id, data, current_real_time):
        current_loss_rate = self.loss_rate
        if self.burst_loss_timer > 0:
            current_loss_rate = 0.5
            self.burst_loss_timer -= 1
        elif random.random() < 0.001:
            self.burst_loss_timer = random.randint(10, 50)

        if random.random() < current_loss_rate:
            return

        base_latency = random.uniform(self.latency_min, self.latency_max)
        jitter = random.uniform(-self.jitter, self.jitter)
        latency = max(1, base_latency + jitter)
        self.packets.append((current_real_time + latency, dst_id, data))

    def get_packets(self, node_id, current_real_time):
        received = []
        remaining = []
        for delivery_time, dst_id, data in self.packets:
            if dst_id == node_id and delivery_time <= current_real_time:
                received.append(data)
            else:
                remaining.append((delivery_time, dst_id, data))
        self.packets = remaining
        return received

class KalmanPeerFilter:
    def __init__(self):
        self.phaseUs = 0.0
        self.driftUsPerSec = 0.0
        self.P = [[1e6, 0.0], [0.0, 1e4]]
        self.Q = [[10.0, 0.0], [0.0, 0.01]]
        self.R = 2000.0
        self.lastUpdateUs = 0

    def predict(self, nowUs):
        if self.lastUpdateUs == 0:
            self.lastUpdateUs = nowUs
            return
        dt = (nowUs - self.lastUpdateUs) / 1_000_000.0
        if dt < 0: dt = 0

        self.phaseUs += self.driftUsPerSec * dt

        P00 = self.P[0][0]
        P01 = self.P[0][1]
        P10 = self.P[1][0]
        P11 = self.P[1][1]

        self.P[0][0] = P00 + dt * (P10 + P01) + dt * dt * P11 + self.Q[0][0]
        self.P[0][1] = P01 + dt * P11 + self.Q[0][1]
        self.P[1][0] = P10 + dt * P11 + self.Q[1][0]
        self.P[1][1] = P11 + self.Q[1][1]

        self.lastUpdateUs = nowUs

    def update(self, measuredPhaseUs, measurementNoise):
        self.R = measurementNoise
        y = measuredPhaseUs - self.phaseUs
        S = self.P[0][0] + self.R
        if S < 1e-9: return

        K0 = self.P[0][0] / S
        K1 = self.P[1][0] / S

        self.phaseUs += K0 * y
        self.driftUsPerSec += K1 * y

        P00 = self.P[0][0]
        P01 = self.P[0][1]

        self.P[0][0] -= K0 * P00
        self.P[0][1] -= K0 * P01
        self.P[1][0] -= K1 * P00
        self.P[1][1] -= K1 * P01

    def quality(self):
        uncertainty = math.sqrt(abs(self.P[0][0])) + math.sqrt(abs(self.P[1][1])) * 20.0
        return 1.0 / (1.0 + uncertainty / 1000.0)

class NTPNode:
    def __init__(self, node_id, num_nodes, drift_ppm=0):
        self.node_id = node_id
        self.num_nodes = num_nodes
        self.drift = 1.0 + (drift_ppm / 1_000_000.0)
        self.phase_offset = random.uniform(-10_000_000, 10_000_000)

        self.TDMA_SLOT_US = 50000
        self.MAX_NODES = 16
        self.SUPERFRAME_US = self.TDMA_SLOT_US * self.MAX_NODES
        self.TX_WINDOW_US = 4000

        self.localPhaseUs = 0.0
        self.localDriftUsPerSec = 0.0
        self.peers = {}

        self.txSequence = 0
        self.lastPollFrame = -1
        self.pendingPolls = {}
        self.pendingReplies = []
        self.lastKfTimeMs = 0

    def rawTimeUs(self, real_time_ms):
        return real_time_ms * 1000 * self.drift + self.phase_offset

    def networkTimeUs(self, real_time_ms):
        return self.rawTimeUs(real_time_ms) - self.localPhaseUs

    def step(self, real_time_ms, network):
        nowUs = self.rawTimeUs(real_time_ms)
        netUs = self.networkTimeUs(real_time_ms)

        frame = int(netUs // self.SUPERFRAME_US)
        myJitter = (self.node_id * 7) % 10
        mySlotStart = frame * self.SUPERFRAME_US + (self.node_id % self.MAX_NODES) * self.TDMA_SLOT_US + myJitter * 1000
        if frame != self.lastPollFrame and netUs >= mySlotStart and (netUs - mySlotStart) < self.TX_WINDOW_US:
            self.txSequence += 1
            pkt = {
                'type': 'POLL',
                'src': self.node_id,
                'seq': self.txSequence,
                't1': nowUs,
                'advPhase': self.localPhaseUs,
                'advDrift': self.localDriftUsPerSec
            }
            network.broadcast(self.node_id, pkt, real_time_ms)
            self.pendingPolls[pkt['seq']] = nowUs
            self.lastPollFrame = frame

        rx_packets = network.get_packets(self.node_id, real_time_ms)
        for pkt in rx_packets:
            src = pkt['src']
            if pkt['type'] == 'POLL':
                if src not in self.peers:
                    self.peers[src] = {
                        'active': True,
                        'filter': KalmanPeerFilter(),
                        'lastHeardMs': real_time_ms,
                        'avgRTTUs': 0.0,
                        'lastThetaUs': 0.0,
                        'lastThetaTimeUs': 0,
                        'advPhase': pkt['advPhase'],
                        'advDrift': pkt['advDrift']
                    }
                p = self.peers[src]
                p['active'] = True
                p['lastHeardMs'] = real_time_ms
                p['advPhase'] = pkt['advPhase']
                p['advDrift'] = pkt['advDrift']

                self.pendingReplies.append({
                    'target': src,
                    'poll': pkt,
                    'rxUs': nowUs,
                    'sendAtUs': nowUs + 1500 + random.uniform(0, 2000)
                })
            elif pkt['type'] == 'REPLY' and pkt['dst'] == self.node_id:
                if pkt['echoSeq'] in self.pendingPolls:
                    t1Us = self.pendingPolls.pop(pkt['echoSeq'])
                    if src in self.peers:
                        p = self.peers[src]
                        T1, T2, T3, T4 = t1Us, pkt['t2'], pkt['t3'], nowUs
                        thetaUs = ((T2 - T1) + (T3 - T4)) * 0.5
                        delayUs = max(0, (T4 - T1) - (T3 - T2))

                        relDrift = 0.0
                        if p['lastThetaTimeUs'] != 0 and nowUs > p['lastThetaTimeUs']:
                            dt = (nowUs - p['lastThetaTimeUs']) / 1_000_000.0
                            if dt > 0.01:
                                relDrift = (thetaUs - p['lastThetaUs']) / dt
                        p['lastThetaUs'] = thetaUs
                        p['lastThetaTimeUs'] = nowUs

                        candPhase = p['advPhase'] - thetaUs
                        candDrift = p['advDrift'] - relDrift

                        p['filter'].predict(nowUs)
                        noise = 2000.0 + delayUs * 0.5
                        p['filter'].update(candPhase, noise)

                        if p['avgRTTUs'] == 0: p['avgRTTUs'] = delayUs
                        else: p['avgRTTUs'] = p['avgRTTUs'] * 0.8 + delayUs * 0.2

        rem_replies = []
        for r in self.pendingReplies:
            if nowUs >= r['sendAtUs']:
                reply = {
                    'type': 'REPLY',
                    'src': self.node_id,
                    'dst': r['target'],
                    'echoSeq': r['poll']['seq'],
                    't1': r['poll']['t1'],
                    't2': r['rxUs'],
                    't3': nowUs,
                    'advPhase': self.localPhaseUs,
                    'advDrift': self.localDriftUsPerSec
                }
                network.send(self.node_id, r['target'], reply, real_time_ms)
            else:
                rem_replies.append(r)
        self.pendingReplies = rem_replies

        if real_time_ms - self.lastKfTimeMs >= 50:
            dt = (real_time_ms - self.lastKfTimeMs) / 1000.0
            self.lastKfTimeMs = real_time_ms

            weightedPhase = 0.0
            weightedDrift = 0.0
            totalWeight = 0.0
            for pid, p in self.peers.items():
                if not p['active'] or (real_time_ms - p['lastHeardMs'] > 10000):
                    p['active'] = False
                    continue

                weight = p['filter'].quality() / (1.0 + p['avgRTTUs'] / 1000.0)
                weightedPhase += p['filter'].phaseUs * weight
                weightedDrift += p['filter'].driftUsPerSec * weight
                totalWeight += weight

            if totalWeight > 1e-6:
                weightedPhase /= totalWeight
                weightedDrift /= totalWeight

                diff = weightedPhase - self.localPhaseUs
                if abs(diff) > 200000:
                    self.localPhaseUs = weightedPhase
                    self.localDriftUsPerSec = weightedDrift
                else:
                    self.localPhaseUs += 0.2 * diff
                    self.localDriftUsPerSec += 0.1 * (weightedDrift - self.localDriftUsPerSec)

            self.localPhaseUs += self.localDriftUsPerSec * dt

        return netUs / 1000.0

def run_simulation(NodeClass, num_nodes=4, duration_s=120, drift_ppm_max=200):
    network = Network(loss_rate=0.1, jitter=10)
    network.num_nodes = num_nodes
    nodes = [NodeClass(i, num_nodes, drift_ppm=random.uniform(-drift_ppm_max, drift_ppm_max)) for i in range(num_nodes)]
    history = []
    for t_ms in range(0, duration_s * 1000):
        step_data = []
        for node in nodes:
            curr_net_millis = node.step(t_ms, network)
            step_data.append(curr_net_millis)
        history.append(step_data)
    return history

if __name__ == "__main__":
    print("Running Iterated Final Simulation...")
    history = run_simulation(NTPNode, num_nodes=8)
    num_nodes = len(history[0])
    steady_state = history[-20000:]
    for i in range(1, num_nodes):
        diffs = [h[i] - h[0] for h in steady_state]
        mean_diff = sum(diffs) / len(diffs)
        std_diff = math.sqrt(sum((d - mean_diff)**2 for d in diffs) / len(diffs))
        max_abs_diff = max(abs(d) for d in diffs)
        print(f"Node {i} vs Node 0: Mean diff = {mean_diff:.2f}ms, Std = {std_diff:.2f}ms, Max abs = {max_abs_diff:.2f}ms")
