
// Define constants and variables

#define GNPSF 1000 // Global network packet sending frequency in milliseconds
#define NODES 4 // Number of nodes in the network
#define PORT 8888 // UDP port number

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp; // UDP socket object

// Variables for optimal time slot (OTS), timer value (TV), wifi signal level (WSL), wifi noise level (WNL),
// calculated packet loss (CPL), estimated packet loss (EPL), estimated timer error (ETE),
// round trip time (RTT), packet arrival time (PAT), measurement noise (MN)

float OTS; // Fraction of GNPSF that indicates when to send packet
unsigned long TV; // Local counter that tracks elapsed time in milliseconds
int WSL; // Wifi signal level in dBm
int WNL; // Wifi noise level in dBm
int CPL; // Number of packets lost in current cycle
float EPL; // Estimated packet loss rate based on WSL and WNL
float ETE; // Estimated timer error in milliseconds
float RTT[NODES]; // Array of round trip times for each node in milliseconds
float PAT[NODES]; // Array of packet arrival times for each node in milliseconds
float MN[NODES]; // Array of measurement noises for each node in milliseconds

// Variables for kalman filter model (KFM)

float A; // State transition matrix element that relates TV to ETE
float B; // Control input matrix element that relates OTS to E
float C; // Observation matrix element that relates ETE to PAT
float Q; // Process noise covariance matrix element that represents the uncertainty in ETE
float R; // Measurement noise covariance matrix element that represents the uncertainty in PAT
float P; // Error covariance matrix element that represents the estimation error in ETE
float K; // Kalman gain matrix element that balances the prediction and measurement

// Variables for gradient descent

float alpha; // Learning rate parameter that controls the step size
float J; // Cost function value that measures the deviation from the optimal solution
float dJ_dOTS; // Partial derivative of the cost function with respect to OTS
float dJ_dTV; // Partial derivative of the cost function with respect to TV

// Variables for UDP packet

byte packetBuffer[64]; // Buffer to hold incoming and outgoing packets
byte packetData[8]; // Data to be sent in each packet
byte packetIP[4]; // IP address of the sender node
byte packetOTS[2]; // OTS of the sender node
byte packetTV[2]; // TV of the sender node

// Initialize the device

void setup() {
  Serial.begin(115200); // Start serial communication
  WiFi.mode(WIFI_STA); // Set wifi mode to station
  WiFi.begin("SSID", "PASSWORD"); // Connect to wifi network
  while (WiFi.status() != WL_CONNECTED) { // Wait for connection
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  udp.begin(PORT); // Start UDP socket
  randomSeed(analogRead(0)); // Seed random number generator
  OTS = random(0, GNPSF) / (float) GNPSF; // Initialize OTS with a random value between 0 and GNPSF
  TV = millis(); // Initialize TV with current time
}

// Main loop

void loop() {
  WSL = WiFi.RSSI(); // Get wifi signal level
  WNL = WiFi.noise(); // Get wifi noise level
  EPL = estimatePacketLoss(WSL, WNL); // Estimate packet loss rate based on WSL and WNL
  
  if (millis() - TV >= OTS * GNPSF) { // Check if it is time to send a packet
    sendPacket(); // Send a packet with data useful for the task
    TV = millis(); // Update TV with current time
    CPL = 0; // Reset CPL for next cycle
  }

  if (udp.parsePacket()) { // Check if a packet is available
    receivePacket(); // Receive and parse the packet data
    CPL++; // Increment CPL for current cycle
    updateVariables(); // Update variables based on packet data
    updateKalmanFilter(); // Update kalman filter model based on packet data
    updateGradientDescent(); // Update gradient descent algorithm based on packet data
    pulseLED(); // Pulse the built-in LED at the GNPSF
    reportDiagnostics(); // Report diagnostics to serial console
  }
}

// Define helper functions

void sendPacket() {
  IPAddress broadcastIP = WiFi.localIP();
  broadcastIP[3] = 255; // Set last byte of IP address to 255 for broadcast
  packetIP[0] = WiFi.localIP()[0]; // Get first byte of IP address
  packetIP[1] = WiFi.localIP()[1]; // Get second byte of IP address
  packetIP[2] = WiFi.localIP()[2]; // Get third byte of IP address
  packetIP[3] = WiFi.localIP()[3]; // Get fourth byte of IP address
  packetOTS[0] = highByte((int) (OTS * 1000)); // Get high byte of OTS
  packetOTS[1] = lowByte((int) (OTS * 1000)); // Get low byte of OTS
  packetTV[0] = highByte((int) TV); // Get high byte of TV
  packetTV[1] = lowByte((int) TV); // Get low byte of TV
  for (int i = 0; i < 4; i++) { // Copy IP bytes to packet data
    packetData[i] = packetIP[i];
  }
  for (int i = 4; i < 6; i++) { // Copy OTS bytes to packet data
    packetData[i] = packetOTS[i - 4];
  }
  for (int i = 6; i < 8; i++) { // Copy TV bytes to packet data
    packetData[i] = packetTV[i - 6];
  }
  udp.beginPacket(broadcastIP, PORT); // Start UDP packet
  udp.write(packetData, sizeof(packetData)); // Write packet data
  udp.endPacket(); // End UDP packet
}

void receivePacket() {
  udp.read(packetBuffer, sizeof(packetBuffer)); // Read UDP packet
  for (int i = 0; i < sizeof(packetData); i++) { // Copy packet buffer to packet data
    packetData[i] = packetBuffer[i];
  }
}

void updateVariables() {
  int nodeIndex; // Index of the sender node in the arrays
  float nodeOTS; // OTS of the sender node
  unsigned long nodeTV; // TV of the sender node
  
  for (int i = 0; i < NODES; i++) { // Loop through all nodes
    if (packetIP[3] == WiFi.localIP()[3]) { // Check if the sender node is the same as the receiver node
      nodeIndex = i; // Set node index to current index
      break; // Exit the loop
    }
  }
  nodeOTS = (packetOTS[0] * 256 + packetOTS[1]) / (float) 1000; // Convert OTS bytes to float
  nodeTV = packetTV[0] * 256 + packetTV[1]; // Convert TV bytes to unsigned long
  RTT[nodeIndex] = millis() - nodeTV; // Calculate RTT for the sender node
  PAT[nodeIndex] = millis() % GNPSF - nodeOTS * GNPSF; // Calculate PAT for the sender node
  MN[nodeIndex] = estimateMeasurementNoise(WSL, WNL, CPL, EPL, ETE, RTT[nodeIndex], PAT[nodeIndex]); // Estimate MN for the sender node
}

void updateKalmanFilter() {
  // TODO: Define the kalman filter model and update ETE and MN based on KFM data
}

void updateGradientDescent() {
  // TODO: Define the cost function and update OTS and TV based on gradient descent data
}

void pulseLED() {
  if (millis() % GNPSF == 0) { // Check if it is time to pulse the LED
    digitalWrite(LED_BUILTIN, HIGH); // Turn on the LED
    delay(10); // Wait for 10 milliseconds
    digitalWrite(LED_BUILTIN, LOW); // Turn off the LED
  }
}

void reportDiagnostics() {
  Serial.print("OTS: ");
  Serial.println(OTS);
  Serial.print("TV: ");
  Serial.println(TV);
  Serial.print("WSL: ");
  Serial.println(WSL);
  Serial.print("WNL: ");
  Serial.println(WNL);
  Serial.print("CPL: ");
  Serial.println(CPL);
  Serial.print("EPL: ");
  Serial.println(EPL);
  Serial.print("ETE: ");
  Serial.println(ETE);
  for (int i = 0; i < NODES; i++) { // Loop through all nodes
    Serial.print("RTT[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(RTT[i]);
    Serial.print("PAT[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(PAT[i]);
    Serial.print("MN[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(MN[i]);
  }
  Serial.println();
}

float estimatePacketLoss(int WSL, int WNL) {
  // A simple function that estimates the packet loss rate based on WSL and WNL
  // Assuming that the packet loss rate is proportional to the difference between WSL and WNL
  // And that the packet loss rate is bounded between 0 and 1
  float PLR = (WNL - WSL) / 100.0; // Calculate the packet loss rate
  if (PLR < 0) PLR = 0; // Set the lower bound to 0
  if (PLR > 1) PLR = 1; // Set the upper bound to 1
  return PLR; // Return the packet loss rate
}

float estimateMeasurementNoise(int WSL, int WNL, int CPL, float EPL, float ETE, float RTT, float PAT) {
  // A simple function that estimates the measurement noise based on WSL, WNL, CPL, EPL, ETE, RTT, and PAT
  // Assuming that the measurement noise is a weighted sum of the variances of each variable
  // And that the weights are arbitrary constants that can be tuned
  float wWSL = 0.1; // Weight for WSL variance
  float wWNL = 0.1; // Weight for WNL variance
  float wCPL = 0.2; // Weight for CPL variance
  float wEPL = 0.2; // Weight for EPL variance
  float wETE = 0.2; // Weight for ETE variance
  float wRTT = 0.1; // Weight for RTT variance
  float wPAT = 0.1; // Weight for PAT variance
  float MN = 0; // Initialize the measurement noise
  MN += wWSL * sq(WSL); // Add the WSL variance
  MN += wWNL * sq(WNL); // Add the WNL variance
  MN += wCPL * sq(CPL); // Add the CPL variance
  MN += wEPL * sq(EPL); // Add the EPL variance
  MN += wETE * sq(ETE); // Add the ETE variance
  MN += wRTT * sq(RTT); // Add the RTT variance
  MN += wPAT * sq(PAT); // Add the PAT variance
  return MN; // Return the measurement noise
}
