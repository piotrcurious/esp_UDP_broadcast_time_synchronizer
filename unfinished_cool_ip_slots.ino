
// Define the global network packet sending frequency (GNPSF) in milliseconds
#define GNPSF 1000

// Define the number of nodes in the network
#define NODES 4

// Define the IP address of each node (assuming last field starts from 1)
#define IP1 "192.168.1.1"
#define IP2 "192.168.1.2"
#define IP3 "192.168.1.3"
#define IP4 "192.168.1.4"

// Define the slot for pulsing LEDs and reporting diagnostics (0 to NODES - 1)
#define LED_SLOT 0

// Include the libraries for ESP8266 and UDP
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// Create a UDP object
WiFiUDP udp;

// Create a structure for storing the data from other nodes
struct NodeData {
  String ip; // sender IP
  float ots; // sender OTS
  unsigned long timer; // sender timer value
  int wsl; // sender WiFi signal level
  unsigned long pat; // packet arrival time
  long pdt; // packet delay time
  int pl; // packet loss
};

// Create an array of NodeData objects for storing the data from other nodes
NodeData nodeData[NODES];

// Create a variable for storing the timer error estimate (x)
float timerError = 0;

// Create a variable for storing the state covariance matrix (P)
float stateCov = 1;

// Create a variable for storing the measurement noise matrix (R)
float measNoise = 1;

// Create a variable for storing the optimal time slot (OTS)
float ots = random(0, NODES) / (float)NODES;

// Create a variable for storing the timer value
unsigned long timer = ots * GNPSF;

// Create a variable for storing the expected number of packets
int expectedPackets = 0;

// Create a variable for storing the received number of packets
int receivedPackets = 0;

// Create a variable for storing the previous millis value
unsigned long prevMillis = 0;

// Setup function
void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Connect to WiFi network
  WiFi.begin("SSID", "PASSWORD");

  // Wait until connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Print local IP address
  Serial.println();
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Start UDP listening on port 8888
  udp.begin(8888);

  // Print OTS and timer value
  Serial.print("OTS: ");
  Serial.println(ots);
  Serial.print("Timer: ");
  Serial.println(timer);
}

// Loop function
void loop() {
  // Get the current millis value
  unsigned long currMillis = millis();

  // Check if the millis value has overflowed
  if (currMillis < prevMillis) {
    // Increase the measurement noise to account for the uncertainty
    measNoise += 10;
  }

  // Update the previous millis value
  prevMillis = currMillis;

  // Check if it is time to send a UDP packet
  if (currMillis >= timer) {
    // Create a buffer for storing the packet data
    char buffer[32];

    // Format the packet data as a string: IP,OTS,TIMER,WSL
    sprintf(buffer, "%s,%f,%lu,%d", WiFi.localIP().toString().c_str(), ots, timer, WiFi.RSSI());

    // Broadcast the packet to all nodes on port 8888
    udp.beginPacket(IPAddress(255, 255, 255, 255), 8888);
    udp.write(buffer);
    udp.endPacket();

    // Print the packet data to serial console
    Serial.print("Sent: ");
    Serial.println(buffer);

    // Increment the expected number of packets
    expectedPackets++;

    // Update the timer value by adding the GNPSF
    timer += GNPSF;
  }

  // Check if there is any UDP packet available
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Create a buffer for storing the packet data
    char buffer[32];

    // Read the packet data into the buffer
    udp.read(buffer, packetSize);

    // Print the packet data to serial console
    Serial.print("Received: ");
    Serial.println(buffer);

    // Increment the received number of packets
    receivedPackets++;

    // Parse the packet data as a string: IP,OTS,TIMER,WSL
    String ip = strtok(buffer, ",");
    float ots = atof(strtok(NULL, ","));
    unsigned long timer = atol(strtok(NULL, ","));
    int wsl = atoi(strtok(NULL, ","));

    // Get the packet arrival time
    unsigned long pat = millis();

    // Calculate the packet delay time
    long pdt = pat - timer;

    // Calculate the packet loss
    int pl = expectedPackets - receivedPackets;

    // Find the index of the sender node in the nodeData array (based on IP)
    int index = -1;
    for (int i = 0; i < NODES; i++) {
      if (nodeData[i].ip == ip) {
        index = i;
        break;
      }
    }

    // If the sender node is not found in the nodeData array, find an empty slot and assign its IP
    if (index == -1) {
      for (int i = 0; i < NODES; i++) {
        if (nodeData[i].ip == "") {
          index = i;
          nodeData[i].ip = ip;
          break;
        }
      }
    }

    // If the sender node is found or assigned in the nodeData array, update its data
    if (index != -1) {
      nodeData[index].ots = ots;
      nodeData[index].timer = timer;
      nodeData[index].wsl = wsl;
      nodeData[index].pat = pat;
      nodeData[index].pdt = pdt;
      nodeData[index].pl = pl;
    }
  }

  // Calculate the measurement vector (z) and matrix (H) using the data from other nodes
  float z[4] = {0}; // RTT, DPA, WSL, PL
  float H[4] = {0}; // coefficients for x in z
  for (int i = 0; i < NODES; i++) {
    // Skip the empty slots or the local node in the nodeData array
    if (nodeData[i].ip == "" || nodeData[i].ip == WiFi.localIP().toString()) {
      continue;
    }

    // Calculate the round trip time (RTT) as the sum of the packet delay times of both nodes
    float rtt = nodeData[i].pdt + pdt;

    // Calculate the difference of packet arrival time from OTS (DPA) as the difference between the arrival time and the OTS multiplied by GNPSF
    float dpa = nodeData[i].pat - nodeData[i].ots * GNPSF;

    // Get the WiFi signal level (WSL) of the other node
    float wsl = nodeData[i].wsl;

    // Get the packet loss (PL) of the other node
    float pl = nodeData[i].pl;

    // Add the values to the measurement vector
    z[0] += rtt;
    z[1] += dpa;
    z[2] += wsl;
    z[3] += pl;

    // Calculate the coefficients for x in z using some heuristics
    H[0] -= 1 / rtt; // RTT is inversely proportional to x
    H[1] += 1 / dpa; // DPA is directly proportional to x
    H[2] -= 1 / wsl; // WSL is inversely proportional to v
    H[3] += 1 / pl; // PL is directly proportional to v
  }

  // Normalize the measurement vector and matrix by dividing by NODES - 1
  for (int i = 0; i < 4; i++) {
    z[i] /= NODES - 1;
    H[i] /= NODES - 1;
  }

  // Update the timer error (x) and OTS using the Kalman filter algorithm

  // Prediction step
  timerError += 0; // u is zero in this case
  stateCov += 0.01; // Q is a small constant in this case

  // Update step
  float y[4]; // innovation vector
  float K[4]; // Kalman gain

  // Calculate y as z - H * x
  for (int i = 0; i < 4; i++) {
    y[i] = z[i] - H[i] * timerError;
  }

  // Calculate K as P * H' / (H * P * H' + R)
  float denom = 0; // denominator of K
  for (int i = 0; i < 4; i++) {
    denom += H[i] * stateCov * H[i];
  }
  denom += measNoise;
  
  // Calculate K as P * H' / (H * P * H' + R)
  float denom = 0; // denominator of K
  for (int i = 0; i < 4; i++) {
    denom += H[i] * stateCov * H[i];
  }
  denom += measNoise;
  for (int i = 0; i < 4; i++) {
    K[i] = stateCov * H[i] / denom;
  }

  // Calculate x as x + K * y
  for (int i = 0; i < 4; i++) {
    timerError += K[i] * y[i];
  }

  // Calculate P as P - K * H * P
  for (int i = 0; i < 4; i++) {
    stateCov -= K[i] * H[i] * stateCov;
  }

  // Update OTS as OTS - x / GNPSF
  ots -= timerError / GNPSF;

  // Constrain OTS to be between 0 and NODES - 1
  if (ots < 0) {
    ots += NODES;
  }
  if (ots >= NODES) {
    ots -= NODES;
  }

  // Print x and OTS to serial console
  Serial.print("Timer error: ");
  Serial.println(timerError);
  Serial.print("OTS: ");
  Serial.println(ots);

  // Check if the current node is assigned to the LED slot
  if ((int)(ots * NODES) == LED_SLOT) {
    // Pulse the built-in LED at the GNPSF frequency
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);

    // Report the diagnostics to serial console
    Serial.println("Diagnostics:");
    Serial.print("GNPSF: ");
    Serial.println(GNPSF);
    Serial.print("State covariance: ");
    Serial.println(stateCov);
    Serial.print("Measurement noise: ");
    Serial.println(measNoise);
    Serial.print("Expected packets: ");
    Serial.println(expectedPackets);
    Serial.print("Received packets: ");
    Serial.println(receivedPackets);
    Serial.print("Data from other nodes: ");
    Serial.println();
    for (int i = 0; i < NODES; i++) {
      if (nodeData[i].ip != "" && nodeData[i].ip != WiFi.localIP().toString()) {
        Serial.print(nodeData[i].ip);
        Serial.print(",");
        Serial.print(nodeData[i].ots);
        Serial.print(",");
        Serial.print(nodeData[i].timer);
        Serial.print(",");
        Serial.print(nodeData[i].wsl);
        Serial.print(",");
        Serial.print(nodeData[i].pat);
        Serial.print(",");
        Serial.print(nodeData[i].pdt);
        Serial.print(",");
        Serial.println(nodeData[i].pl);
      }
    }
    

