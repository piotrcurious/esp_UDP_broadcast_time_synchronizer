
// Include libraries
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <MatrixMath.h>

// Define constants
#define NUM_NODES 4 // Number of nodes in the network
#define BROADCAST_PORT 8888 // UDP broadcast port
#define PACKET_SIZE 28 // UDP packet size in bytes
#define LED_PIN LED_BUILTIN // LED pin
#define SERIAL_BAUDRATE 115200 // Serial baud rate

// Define variables
float systemState[2 * NUM_NODES]; // System state vector
float systemCovariance[2 * NUM_NODES][2 * NUM_NODES]; // System covariance matrix
float measurementState[2 * NUM_NODES]; // Measurement state vector
float measurementCovariance[2 * NUM_NODES][2 * NUM_NODES]; // Measurement covariance matrix
float systemMatrix[2 * NUM_NODES][2 * NUM_NODES]; // System matrix
float measurementMatrix[2 * NUM_NODES][2 * NUM_NODES]; // Measurement matrix
float transitionMatrix[2 * NUM_NODES][2 * NUM_NODES]; // Transition matrix
float controlMatrix[2 * NUM_NODES][1]; // Control matrix
float processNoiseCovariance[2 * NUM_NODES][2 * NUM_NODES]; // Process noise covariance matrix
float measurementNoiseCovariance[2 * NUM_NODES][2 * NUM_NODES]; // Measurement noise covariance matrix
float kalmanGain[2 * NUM_NODES][2 * NUM_NODES]; // Kalman gain matrix

WiFiUDP udp; // UDP object
IPAddress broadcastIP; // Broadcast IP address

byte packetBuffer[PACKET_SIZE]; // Packet buffer

unsigned long timerValue; // Timer value in milliseconds
long wifiSignalLevel; // WiFi signal level in dBm
int calculatedPacketLoss; // Calculated packet loss in percentage
int estimatedPacketLoss; // Estimated packet loss in percentage
long estimatedTimerError; // Estimated timer error in milliseconds
unsigned long roundTripTime; // Round trip time in milliseconds
long packetArrivalTime; // Packet arrival time in milliseconds

// Define functions

// Initialize the system state vector with random OTS and default GNPSF values for each node
void initializeSystemState() {
  for (int i = 0; i < NUM_NODES; i++) {
    systemState[i] = random(1000); // Random OTS between 0 and 1000 ms
    systemState[i + NUM_NODES] = 1000; // Default GNPSF of 1000 ms for each node
  }
}

// Initialize the system covariance matrix with large diagonal values and zero off-diagonal values
void initializeSystemCovariance() {
  for (int i = 0; i < 2 * NUM_NODES; i++) {
    for (int j = 0; j < 2 * NUM_NODES; j++) {
      if (i == j) {
        systemCovariance[i][j] = 1000000.0; // Large diagonal value to indicate high uncertainty
      } else {
        systemCovariance[i][j] = 0.0; // Zero off-diagonal value to indicate no correlation
      }
    }
  }
}

// Initialize the measurement state vector with zero PAT and WSL values for each node
void initializeMeasurementState() {
  for (int i = 0; i < 2 * NUM_NODES; i++) {
    measurementState[i] = 0.0; // Zero PAT and WSL values for each node
  }
}

// Initialize the measurement covariance matrix with large diagonal values and zero off-diagonal values
void initializeMeasurementCovariance() {
  for (int i = 0; i < 2 * NUM_NODES; i++) {
    for (int j = 0; j < 2 * NUM_NODES; j++) {
      if (i == j) {
        measurementCovariance[i][j] = 1000000.0; // Large diagonal value to indicate high uncertainty
      } else {
        measurementCovariance[i][j] = 0.0; // Zero off-diagonal value to indicate no correlation
      }
    }
  }
}

// Initialize the system matrix with identity values 
void initializeSystemMatrix() {
  for (int i = 0; i < 2 * NUM_NODES; i++) {
    for (int j = 0; j < 2 * NUM_NODES; j++) {
      if (i == j) {
        systemMatrix[i][j] = 1.0; // Identity value to indicate no change in the system state
      } else {
        systemMatrix[i][j] = 0.0; // Zero value to indicate no influence from other system state variables
      }
    }
  }
}

// Initialize the measurement matrix with identity values
void initializeMeasurementMatrix() {
  for (int i = 0; i < 2 * NUM_NODES; i++) {
    for (int j = 0; j < 2 * NUM_NODES; j++) {
      if (i == j) {
        measurementMatrix[i][j] = 1.0; // Identity value to indicate no change in the measurement state
      } else {
        measurementMatrix[i][j] = 0.0; // Zero value to indicate no influence from other measurement state variables
      }
    }
  }
}

// Initialize the transition matrix with identity values and kalman gains as off-diagonal values
void initializeTransitionMatrix() {
  for (int i = 0; i < 2 * NUM_NODES; i++) {
    for (int j = 0; j < 2 * NUM_NODES; j++) {
      if (i == j) {
        transitionMatrix[i][j] = 1.0; // Identity value to indicate no change in the system state
      } else if (i < NUM_NODES && j >= NUM_NODES) {
        transitionMatrix[i][j] = -kalmanGain[i][j - NUM_NODES]; // Negative kalman gain value to indicate how the system state is updated based on the kalman filter
      } else if (i >= NUM_NODES && j < NUM_NODES) {
        transitionMatrix[i][j] = kalmanGain[i - NUM_NODES][j]; // Positive kalman gain value to indicate how the system state is updated based on the kalman filter
      } else if (i == NUM_NODES + j) {
        transitionMatrix[i][j] = 1.0 - kalmanGain[i - NUM_NODES][j - NUM_NODES]; // Identity value minus kalman gain value to indicate how the system state is updated based on the kalman filter
      } else {
        transitionMatrix[i][j] = 0.0; // Zero value to indicate no influence from other system state variables
      }
    }
  }
}

// Initialize the control matrix with zero values
void initializeControlMatrix() {
  for (int i = 0; i < 2 * NUM_NODES; i++) {
    controlMatrix[i][0] = 0.0; // Zero value to indicate no external input to the system state
  }
}

// Initialize the process noise covariance matrix with small diagonal values and zero off-diagonal values
void initializeProcessNoiseCovariance() {
  for (int i = 0; i < 2 * NUM_NODES; i++) {
    for (int j = 0; j < 2 * NUM_NODES; j++) {
      if (i == j) {
        processNoiseCovariance[i][j] = 0.001; // Small diagonal value to indicate low process noise
      } else {
        processNoiseCovariance[i][j] = 0.0; // Zero off-diagonal value to indicate no correlation
      }
    }
  }
}

// Initialize the measurement noise covariance matrix with small diagonal values and zero off-diagonal values
void initializeMeasurementNoiseCovariance() {
  for (int i = 0; i < 2 * NUM_NODES; i++) {
    for (int j = 0; j < 2 * NUM_NODES; j++) {
      if (i == j) {
        measurementNoiseCovariance[i][j] = 0.001; // Small diagonal value to indicate low measurement noise
      } else {
        measurementNoiseCovariance[i][j] = 0.0; // Zero off-diagonal value to indicate no correlation
      }
    }
  }
}

// Initialize the kalman gain matrix with zero values
void initializeKalmanGain() {
  for (int i = 0; i < 2 * NUM_NODES; i++) {
    for (int j = 0; j < 2 * NUM_NODES; j++) {
      kalmanGain[i][j] = 0.0; // Zero value to indicate no initial kalman gain
    }
  }
}

// Initialize the UDP object and the broadcast IP address
void initializeUDP() {
  udp.begin(BROADCAST_PORT); // Start listening on the broadcast port
  broadcastIP = WiFi.localIP(); // Get the local IP address
  broadcastIP[3] = 255; // Set the last field to 255 to make it a broadcast address
}

// Initialize the timer value with millis()
void initializeTimerValue() {
  timerValue = millis(); // Get the current time in milliseconds
}

// Initialize the wifi signal level with WiFi.RSSI()
void initializeWifiSignalLevel() {
  wifiSignalLevel = WiFi.RSSI(); // Get the current wifi signal level in dBm
}

// Initialize the calculated packet loss with zero
void initializeCalculatedPacketLoss() {
  calculatedPacketLoss = 0; // Set the calculated packet loss to zero
}

// Initialize the estimated packet loss with zero
void initializeEstimatedPacketLoss() {
  estimatedPacketLoss = 0; // Set the estimated packet loss to zero
}

// Initialize the estimated timer error with zero
void initializeEstimatedTimerError() {
  estimatedTimerError = 0; // Set the estimated timer error to zero
}

// Initialize the round trip time with zero
void initializeRoundTripTime() {
  roundTripTime = 0; // Set the round trip time to zero
}

// Initialize the packet arrival time with zero
void initializePacketArrivalTime() {
  packetArrivalTime = 0; // Set the packet arrival time to zero
}

// Send a UDP broadcast packet containing the system state vector
void sendUDPPacket() {
  // Copy the system state vector to the packet buffer
  for (int i = 0; i < 2 * NUM_NODES; i++) {
    memcpy(packetBuffer + i * sizeof(float), &systemState[i], sizeof(float));
  }
  // Send the packet buffer to the broadcast IP address and port
  udp.beginPacket(broadcastIP, BROADCAST_PORT);
  udp.write(packetBuffer, PACKET_SIZE);
  udp.endPacket();
}

// Receive a UDP broadcast packet and parse the data into the measurement state vector
void receiveUDPPacket() {
  // Check if there is a packet available
  int packetSize = udp.parsePacket();
  if (packetSize == PACKET_SIZE) {
    // Read the packet into the packet buffer
    udp.read(packetBuffer, PACKET_SIZE);
    // Parse the packet buffer into the measurement state vector
    for (int i = 0; i < NUM_NODES; i++) {
      memcpy(&measurementState[i], packetBuffer + i * sizeof(float), sizeof(float)); // Parse the OTS value for each node
      measurementState[i + NUM_NODES] = WiFi.RSSI(); // Get the WSL value for each node
    }
  }
}

// Update the timer value using millis() and handle overflow by increasing the measurement noise covariance
void updateTimerValue() {
  // Get the current time in milliseconds
  unsigned long currentTime = millis();
  // Check if there is an overflow
  if (currentTime < timerValue) {
    // Increase the measurement noise covariance for PAT by a large factor
    for (int i = 0; i < NUM_NODES; i++) {
      measurementNoiseCovariance[i][i] *= 1000.0;
    }
  }
  // Update the timer value with the current time
  timerValue = currentTime;
}

// Update the wifi signal level using WiFi.RSSI()
void updateWifiSignalLevel() {
  // Get the current wifi signal level in dBm
  wifiSignalLevel = WiFi.RSSI();
}

// Update the calculated packet loss by counting the number of packets sent and received in a given time interval
void updateCalculatedPacketLoss() {
  // Define a static variable to store the previous time
  static unsigned long previousTime = 0;
  // Define a static variable to store the number of packets sent
  static int packetsSent = 0;
  // Define a static variable to store the number of packets received
  static int packetsReceived = 0;
  // Define a constant for the time interval in milliseconds
  const unsigned long timeInterval = 10000;
  // Increment the number of packets sent
  packetsSent++;
  // Check if there is a packet available
  int packetSize = udp.parsePacket();
  if (packetSize == PACKET_SIZE) {
    // Increment the number of packets received
    packetsReceived++;
  }
  // Check if the time interval has elapsed
  if (timerValue - previousTime >= timeInterval) {
    // Calculate the packet loss in percentage
    calculatedPacketLoss = (packetsSent - packetsReceived) * 100 / packetsSent;
    // Reset the previous time
    previousTime = timerValue;
    // Reset the number of packets sent
    packetsSent = 0;
    // Reset the number of packets received
    packetsReceived = 0;
  }
}

// Update the estimated packet loss by comparing the calculated packet loss with an expected packet loss based on the wifi signal level
void updateEstimatedPacketLoss() {
  // Define a constant for the expected packet loss in percentage for a good wifi signal level of -50 dBm or higher
  const int expectedPacketLossGood = 1;
  // Define a constant for the expected packet loss in percentage for a fair wifi signal level of -60 dBm or lower
  const int expectedPacketLossFair = 5;
  // Define a constant for the expected packet loss in percentage for a poor wifi signal level of -70 dBm or lower
  const int expectedPacketLossPoor = 10;
  // Check the wifi signal level and compare it with the expected packet loss
  if (wifiSignalLevel >= -50) {
    // Good wifi signal level
    estimatedPacketLoss = calculatedPacketLoss - expectedPacketLossGood;
  } else if (wifiSignalLevel >= -60) {
    // Fair wifi signal level
    estimatedPacketLoss = calculatedPacketLoss - expectedPacketLossFair;
  } else {
    // Poor wifi signal level
    estimatedPacketLoss = calculatedPacketLoss - expectedPacketLossPoor;
  }
}

// Update the estimated timer error by comparing the timer value with the average timer value of other nodes in the network
void updateEstimatedTimerError() {
  // Define a variable to store the sum of timer values of other nodes
  long timerSum = 0;
  // Define a variable to store the number of timer values received
  int timerCount = 0;
  // Loop through the measurement state vector and get the OTS values of other nodes
  for (int i = 0; i < NUM_NODES; i++) {
    // Check if the OTS value is not zero
    if (measurementState[i] != 0.0) {
      // Add the OTS value to the timer sum
      timerSum += measurementState[i];
      // Increment the timer count
      timerCount++;
    }
  }
  // Check if the timer count is not zero
  if (timerCount != 0) {
    // Calculate the average timer value of other nodes
    long timerAverage = timerSum / timerCount;
    // Calculate the estimated timer error by subtracting the timer value from the average timer value
    estimatedTimerError = timerAverage - timerValue;
  }
}

// Update the round trip time by measuring the time difference between sending and receiving a packet from another node
void updateRoundTripTime() {
  // Define a static variable to store the previous time
  static unsigned long previousTime = 0;
  // Check if there is a packet available
  int packetSize = udp.parsePacket();
  if (packetSize == PACKET_SIZE) {
    // Calculate the round trip time by subtracting the previous time from the current time
    roundTripTime = timerValue - previousTime;
    // Update the previous time with the current time
    previousTime = timerValue;
  }
}

// Update the packet arrival time by measuring the time difference between receiving a packet and its optimal time slot
void updatePacketArrivalTime() {
  // Check if there is a packet available
  int packetSize = udp.parsePacket();
  if (packetSize == PACKET_SIZE) {
    // Get the sender IP address
    IPAddress senderIP = udp.remoteIP();
    // Get the sender index by subtracting 1 from the last field of the sender IP address
    int senderIndex = senderIP[3] - 1;
    // Check if the sender index is valid
    if (senderIndex >= 0 && senderIndex < NUM_NODES) {
      // Get the OTS value of the sender from the measurement state vector
      long senderOTS = measurementState[senderIndex];
      // Calculate the packet arrival time by subtracting the sender OTS from the current time
      packetArrivalTime = timerValue - senderOTS;
    }
  }
}

// Update the measurement noise covariance matrix by using a formula that depends on wifi signal level, calculated packet loss, estimated packet loss, estimated timer error, round trip time, and packet arrival time
void updateMeasurementNoiseCovariance() {
  // Define a constant for the base measurement noise
  const float baseMeasurementNoise = 0.001;
  // Define a variable to store the total measurement noise
  float totalMeasurementNoise = 0.0;
  // Calculate the total measurement noise by adding the base measurement noise and the scaled values of wifi signal level, calculated packet loss, estimated packet loss, estimated timer error, round trip time, and packet arrival time
  totalMeasurementNoise = baseMeasurementNoise + abs(wifiSignalLevel) * 0.0001 + abs(calculatedPacketLoss) * 0.0001 + abs(estimatedPacketLoss) * 0.0001 + abs(estimatedTimerError) * 0.0001 + abs(roundTripTime) * 0.0001 + abs(packetArrivalTime) * 0.0001;
  // Update the measurement noise covariance matrix with the total measurement noise as diagonal values and zero as off-diagonal values
  for (int i = 0; i < 2 * NUM_NODES; i++) {
    for (int j = 0; j < 2 * NUM_NODES; j++) {
      if (i == j) {
        measurementNoiseCovariance[i][j] = totalMeasurementNoise; // Total measurement noise as diagonal value
      } else {
        measurementNoiseCovariance[i][j] = 0.0; // Zero as off-diagonal value
      }
    }
  }
}

// Update the process noise covariance matrix by using a formula that depends on global network packet sending frequency and optimal time slot
void updateProcessNoiseCovariance() {
  // Define a constant for the base process noise
  const float baseProcessNoise = 0.001;
  // Define a variable to store the total process noise
  float totalProcessNoise = 0.0;
  // Calculate the total process noise by adding the base process noise and the scaled values of global network packet sending frequency and optimal time slot
  totalProcessNoise = baseProcessNoise + abs(systemState[NUM_NODES]) * 0.0001 + abs(systemState[0]) * 0.0001;
  // Update the process noise covariance matrix with the total process noise as diagonal values and zero as off-diagonal values
  for (int i = 0; i < 2 * NUM_NODES; i++) {
    for (int j = 0; j < 2 * NUM_NODES; j++) {
      if (i == j) {
        processNoiseCovariance[i][j] = totalProcessNoise; // Total process noise as diagonal value
      } else {
        processNoiseCovariance[i][j] = 0.0; // Zero as off-diagonal value
      }
    }
  }
}

// Update the kalman gain matrix by using the standard kalman filter formula: KG = PN * MT / (MT * PN * MT + MN)
void updateKalmanGain() {
  // Define a temporary matrix to store the intermediate results
  float tempMatrix[2 * NUM_NODES][2 * NUM_NODES];
  // Calculate MT * PN and store it in tempMatrix
  Matrix.Multiply((float*)measurementMatrix, (float*)processNoiseCovariance, 2 * NUM_NODES, 2 * NUM_NODES, 2 * NUM_NODES, (float*)tempMatrix);
  // Calculate MT * PN * MT and store it in tempMatrix
  Matrix.Multiply((float*)tempMatrix, (float*)measurementMatrix, 2 * NUM_NODES, 2 * NUM_NODES, 2 * NUM_NODES, (float*)tempMatrix);
  // Calculate MT * PN * MT + MN and store it in tempMatrix
  Matrix.Add((float*)tempMatrix, (float*)measurementNoiseCovariance, 2 * NUM_NODES, 2 * NUM_NODES, (float*)tempMatrix);
  // Calculate the inverse of MT * PN * MT + MN and store it in tempMatrix
  Matrix.Invert((float*)tempMatrix, 2 * NUM_NODES);
  // Calculate PN * MT and store it in tempMatrix
  Matrix.Multiply((float*)processNoiseCovariance, (float*)measurementMatrix, 2 * NUM_NODES, 2 * NUM_NODES, 2 * NUM_NODES, (float*)tempMatrix);
  // Calculate KG = PN * MT / (MT * PN * MT + MN) and store it in kalmanGain
  Matrix.Multiply((float*)tempMatrix, (float*)tempMatrix, 2 * NUM_NODES, 2 * NUM_NODES, 2 * NUM_NODES, (float*)kalmanGain);
}

// Update the system state vector by using the standard kalman filter formula: SS = SS + KG * (MS - SS)
void updateSystemState() {
  // Define a temporary vector to store the intermediate results
  float tempVector[2 * NUM_NODES];
  // Calculate MS - SS and store it in tempVector
  Matrix.Subtract((float*)measurementState, (float*)systemState, 2 * NUM_NODES, 1, (float*)tempVector);
  // Calculate KG * (MS - SS) and store it in tempVector
  Matrix.Multiply((float*)kalmanGain, (float*)tempVector, 2 * NUM_NODES, 2 * NUM_NODES, 1, (float*)tempVector);
  // Calculate SS = SS + KG * (MS - SS) and store it in systemState
  Matrix.Add((float*)systemState, (float*)tempVector, 2 * NUM_NODES, 1, (float*)systemState);
}

// Update the system covariance matrix by using the standard kalman filter formula: SC = (I - KG * MT) * SC
void updateSystemCovariance() {
