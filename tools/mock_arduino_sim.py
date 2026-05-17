import random
import math

class Network:
    def __init__(self, loss_rate=0.05, latency_min=5, latency_max=15):
        self.loss_rate = loss_rate
        self.latency_min = latency_min
        self.latency_max = latency_max
        self.packets = [] # List of (delivery_time, dst_id, data)
        self.num_nodes = 0

    def broadcast(self, src_id, data, current_real_time):
        for i in range(self.num_nodes):
            if i != src_id:
                self.send(src_id, i, data, current_real_time)

    def send(self, src_id, dst_id, data, current_real_time):
        if random.random() < self.loss_rate:
            return
        latency = random.uniform(self.latency_min, self.latency_max)
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
        self.P = [[100000.0, 0.0], [0.0, 1000.0]]
        self.Q = [[1.0, 0.0], [0.0, 0.01]]
        self.R = 1000.0
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
        uncertainty = math.sqrt(abs(self.P[0][0])) + math.sqrt(abs(self.P[1][1])) * 10.0
        return 1.0 / (1.0 + uncertainty / 1000.0)

class NTPNode:
    def __init__(self, node_id, num_nodes, drift_ppm=0):
        self.node_id = node_id
        self.num_nodes = num_nodes
        self.drift = 1.0 + (drift_ppm / 1_000_000.0)
        self.phase_offset = random.uniform(0, 1_000_000) # microseconds

        self.TDMA_SLOT_US = 60000
        self.SUPERFRAME_US = self.TDMA_SLOT_US * num_nodes
        self.TX_WINDOW_US = 5000

        self.localPhaseUs = 0.0
        self.localDriftUsPerSec = 0.0
        self.peers = [None] * num_nodes
        for i in range(num_nodes):
            if i != node_id:
                self.peers[i] = {
                    'active': False,
                    'filter': KalmanPeerFilter(),
                    'lastHeardUs': 0,
                    'avgRTTUs': 0.0,
                    'packetLoss': 0.0,
                    'lastSequence': 0,
                    'lastThetaUs': 0.0,
                    'lastThetaTimeUs': 0,
                    'advertisedPhaseUs': 0.0,
                    'advertisedDriftUsPerSec': 0.0
                }

        self.txSequence = 0
        self.lastPollFrame = -1
        self.pendingPolls = {} # seq -> t1Us
        self.pendingReplies = [] # List of {'target': id, 'poll': pkt, 'rxUs': us, 'sendAtUs': us}

    def rawTimeUs(self, real_time_ms):
        return real_time_ms * 1000 * self.drift + self.phase_offset

    def networkTimeUs(self, real_time_ms):
        return self.rawTimeUs(real_time_ms) - self.localPhaseUs

    def step(self, real_time_ms, network):
        nowUs = self.rawTimeUs(real_time_ms)
        netUs = self.networkTimeUs(real_time_ms)

        # TDMA
        frame = int(netUs // self.SUPERFRAME_US)
        mySlotStart = frame * self.SUPERFRAME_US + self.node_id * self.TDMA_SLOT_US
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

        # Process Incoming
        rx_packets = network.get_packets(self.node_id, real_time_ms)
        for pkt in rx_packets:
            src = pkt['src']
            if pkt['type'] == 'POLL':
                p = self.peers[src]
                p['active'] = True
                p['lastHeardUs'] = nowUs
                p['advertisedPhaseUs'] = pkt['advPhase']
                p['advertisedDriftUsPerSec'] = pkt['advDrift']

                # Queue Reply
                self.pendingReplies.append({
                    'target': src,
                    'poll': pkt,
                    'rxUs': nowUs,
                    'sendAtUs': nowUs + 1500 + random.uniform(0, 2000)
                })
            elif pkt['type'] == 'REPLY' and pkt['dst'] == self.node_id:
                if pkt['echoSeq'] in self.pendingPolls:
                    t1Us = self.pendingPolls.pop(pkt['echoSeq'])
                    p = self.peers[src]

                    T1, T2, T3, T4 = t1Us, pkt['t2'], pkt['t3'], nowUs
                    thetaUs = ((T2 - T1) + (T3 - T4)) * 0.5
                    delayUs = max(0, (T4 - T1) - (T3 - T2))

                    relDrift = 0.0
                    if p['lastThetaTimeUs'] != 0 and nowUs > p['lastThetaTimeUs']:
                        dt = (nowUs - p['lastThetaTimeUs']) / 1_000_000.0
                        if dt > 1e-6:
                            relDrift = (thetaUs - p['lastThetaUs']) / dt

                    p['lastThetaUs'] = thetaUs
                    p['lastThetaTimeUs'] = nowUs

                    candPhase = p['advertisedPhaseUs'] - thetaUs
                    candDrift = p['advertisedDriftUsPerSec'] - relDrift

                    p['filter'].predict(nowUs)
                    noise = 40.0 * 40.0 # Simplified
                    p['filter'].update(candPhase, noise)

                    if p['avgRTTUs'] == 0: p['avgRTTUs'] = delayUs
                    else: p['avgRTTUs'] = p['avgRTTUs'] * 0.9 + delayUs * 0.1

        # Flush Replies
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

        # Consensus
        weightedPhase = 0.0
        weightedDrift = 0.0
        totalWeight = 0.0
        for i in range(self.num_nodes):
            if i == self.node_id: continue
            p = self.peers[i]
            if not p['active']: continue
            if (nowUs - p['lastHeardUs']) > 8_000_000: continue

            weight = p['filter'].quality() / (1.0 + p['avgRTTUs'] / 2000.0)
            weightedPhase += p['filter'].phaseUs * weight
            weightedDrift += p['filter'].driftUsPerSec * weight
            totalWeight += weight

        if totalWeight > 1e-6:
            weightedPhase /= totalWeight
            weightedDrift /= totalWeight
            self.localPhaseUs += 0.02 * (weightedPhase - self.localPhaseUs)
            self.localDriftUsPerSec += 0.005 * (weightedDrift - self.localDriftUsPerSec)

        return netUs / 1000.0

def run_simulation(NodeClass, num_nodes=4, duration_s=60, drift_ppm_max=100):
    network = Network()
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
    print("Running NTP Simulation...")
    history = run_simulation(NTPNode)
    num_nodes = len(history[0])
    # Analyze only last 10 seconds for steady state
    steady_state = history[-10000:]
    for i in range(1, num_nodes):
        diffs = [h[i] - h[0] for h in steady_state]
        mean_diff = sum(diffs) / len(diffs)
        std_diff = math.sqrt(sum((d - mean_diff)**2 for d in diffs) / len(diffs))
        max_abs_diff = max(abs(d) for d in diffs)
        print(f"Node {i} vs Node 0: Mean diff = {mean_diff:.2f}ms, Std = {std_diff:.2f}ms, Max abs = {max_abs_diff:.2f}ms")
