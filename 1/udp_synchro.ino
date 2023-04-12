
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// Define the number of nodes in the network
#define NUM_NODES 4

// Define the port for UDP communication
#define UDP_PORT 8888

// Define the struct for storing data
struct Data {
  float globalFreq; // Global frequency in Hz
  int slotNum; // Slot number in global frequency
  unsigned long timerVal; // Timer value in microseconds
  int signalLevel; // Signal level in dBm
  int noiseLevel; // Noise level in dBm
  float packetLoss; // Packet loss rate in percentage
  float timerError; // Timer error in microseconds
};

// Declare global variables
WiFiUDP udp; // UDP object
IPAddress ips[NUM_NODES]; // IP addresses of nodes
Data data[NUM_NODES]; // Data of nodes
int nodeIndex; // Index of this node
unsigned long lastSendTime; // Last time a packet was sent
unsigned long lastRecvTime; // Last time a packet was received

// Declare functions
void setupWiFi(); // Set up WiFi connection
void setupData(); // Set up initial data values
void sendData(); // Send data to other nodes
void recvData(); // Receive data from other nodes
void calcDelay(); // Calculate packet delay time
void adjustFreq(); // Adjust global frequency and slot number
void kalmanFilter(); // Apply Kalman filter to estimate timer error and noise level
void correctTimer(); // Correct timer value based on timer error and noise level

// Set up WiFi connection
void setupWiFi() {
  WiFi.mode(WIFI_STA); // Set WiFi mode to station
  WiFi.begin("SSID", "PASSWORD"); // Connect to WiFi network with SSID and PASSWORD
  while (WiFi.status() != WL_CONNECTED) { // Wait until connected
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP()); // Print local IP address

  udp.begin(UDP_PORT); // Start UDP communication on port 8888

  // Assign IP addresses to nodes based on their order in the network (can be changed)
  ips[0] = IPAddress(192,168,1,100);
  ips[1] = IPAddress(192,168,1,101);
  ips[2] = IPAddress(192,168,1,102);
  ips[3] = IPAddress(192,168,1,103);

  // Find the index of this node based on its IP address
  for (int i = 0; i < NUM_NODES; i++) {
    if (WiFi.localIP() == ips[i]) {
      nodeIndex = i;
      break;
    }
  }
}

// Set up initial data values
void setupData() {
  // Initialize global frequency to 10 Hz
  for (int i = 0; i < NUM_NODES; i++) {
    data[i].globalFreq = 10.0;
  }

  // Assign slot numbers to nodes based on their index
  for (int i = 0; i < NUM_NODES; i++) {
    data[i].slotNum = i;
  }

  // Initialize timer value to 0
  for (int i = 0; i < NUM_NODES; i++) {
    data[i].timerVal = 0;
  }

  // Initialize signal level, noise level, packet loss, timer error to random values (can be changed)
  for (int i = 0; i < NUM_NODES; i++) {
    data[i].signalLevel = random(-90, -30);
    data[i].noiseLevel = random(-90, -30);
    data[i].packetLoss = random(0, 100) / 100.0;
    data[i].timerError = random(-10000, 10000);
  }
}

// Send data to other nodes
void sendData() {
  // Check if it is time to send a packet based on the global frequency and slot number
  unsigned long currentTime = micros(); // Get current time in microseconds
  unsigned long interval = (unsigned long) (1000000.0 / data[nodeIndex].globalFreq); // Calculate interval between packets in microseconds
  unsigned long offset = (unsigned long) (interval * data[nodeIndex].slotNum / NUM_NODES); // Calculate offset for this node's slot in microseconds
  if (currentTime - lastSendTime >= interval + offset) { // If enough time has passed since last packet
    lastSendTime = currentTime; // Update last send time

    // Update timer value with current time
    data[nodeIndex].timerVal = currentTime;

    // Update signal level and noise level with WiFi RSSI and noise floor values
    data[nodeIndex].signalLevel = WiFi.RSSI();
    data[nodeIndex].noiseLevel = WiFi.noiseFloor();

    // Update packet loss with a random value (can be changed)
    data[nodeIndex].packetLoss = random(0, 100) / 100.0;

    // Update timer error with a random value (can be changed)
    data[nodeIndex].timerError = random(-10000, 10000);

    // Serialize the data into a byte array
    byte buffer[sizeof(Data)];
    memcpy(buffer, &data[nodeIndex], sizeof(Data));

    // Send the byte array as a broadcast packet to all nodes on port 8888
    udp.beginPacket(IPAddress(255,255,255,255), UDP_PORT);
    udp.write(buffer, sizeof(Data));
    udp.endPacket();

    Serial.println("Sent packet");
  }
}

// Receive data from other nodes
void recvData() {
  // Check if there is a packet available to read
  int packetSize = udp.parsePacket(); // Get the size of the packet in bytes
  if (packetSize) { // If there is a packet
    lastRecvTime = micros(); // Update last receive time

    // Read the packet into a byte array
    byte buffer[packetSize];
    udp.read(buffer, packetSize);

    // Deserialize the byte array into a data struct
    Data receivedData;
    memcpy(&receivedData, buffer, sizeof(Data));

    // Find the index of the sender node based on its IP address
    int senderIndex = -1;
    for (int i = 0; i < NUM_NODES; i++) {
      if (udp.remoteIP() == ips[i]) {
        senderIndex = i;
        break;
      }
    }

    // Update the data of the sender node with the received data
    if (senderIndex != -1) {
      data[senderIndex] = receivedData;
    }

    Serial.println("Received packet");
  }
}

// Calculate packet delay time
void calcDelay() {
  // Calculate the average delay time between sending and receiving packets for each node
  float delayTimes[NUM_NODES];
  for (int i = 0; i < NUM_NODES; i++) {
    delayTimes[i] = (lastRecvTime - data[i].timerVal) / 2.0; // Half of the round-trip time
  }

  // Print the delay times for debugging purposes
  Serial.print("Delay times: ");
  for (int i = 0; i < NUM_NODES; i++) {
    Serial.print(delayTimes[i]);
    Serial.print(" ");
  }
  Serial.println();
}

// Adjust global frequency and slot number
void adjustFreq() {
  // Calculate the average global frequency and slot number for the network
  float avgGlobalFreq = 0;
  int avgSlotNum = 0;
  for (int i = 0; i < NUM_NODES; i++) {
    avgGlobalFreq += data[i].globalFreq;
    avgSlotNum += data[i].slotNum;
  }
  avgGlobalFreq /= NUM_NODES;
  avgSlotNum /= NUM_NODES;

  // Calculate the deviation of this node's global frequency and slot number from the average
  float freqDev = data[nodeIndex].globalFreq - avgGlobalFreq;
  int slotDev = data[nodeIndex].slotNum - avgSlotNum;

  // Adjust this node's global frequency and slot number based on the deviation and the packet loss
  data[nodeIndex].globalFreq -= freqDev * data[nodeIndex].packetLoss;
  data[nodeIndex].slotNum -= slotDev * data[nodeIndex].packetLoss;

  // Print the global frequency and slot number for debugging purposes
  Serial.print("Global frequency: ");
  Serial.println(data[nodeIndex].globalFreq);
  Serial.print("Slot number: ");
  Serial.println(data[nodeIndex].slotNum);
}

// Apply Kalman filter to estimate timer error and noise level
void kalmanFilter() {
  // Define the state vector and the state transition matrix
  // The state vector consists of two elements: timer error and noise level
  // The state transition matrix is a 2x2 identity matrix
  float x[2] = {data[nodeIndex].timerError, data[nodeIndex].noiseLevel}; // State vector
  float F[2][2] = {{1,0},{0,1}}; // State transition matrix

  // Define the measurement vector and the measurement matrix
  // The measurement vector consists of the timer values from all the nodes
  // The measurement matrix is a NUM_NODES x 2 matrix with 1s in the first column and signal levels in the second column
  float z[NUM_NODES]; // Measurement vector
  float H[NUM_NODES][2]; // Measurement matrix
  for (int i = 0; i < NUM_NODES; i++) {
    z[i] = data[i].timerVal;
    H[i][0] = 1;
    H[i][1] = data[i].signalLevel;
  }

  // Define the state covariance matrix and the process noise covariance matrix
  // The state covariance matrix is a 2x2 matrix that represents the uncertainty of the state vector
  // The process noise covariance matrix is a 2x2 matrix that represents the uncertainty of the state transition
  float P[2][2] = {{10000,0},{0,10000}}; // State covariance matrix (initialized with large values)
  float Q[2][2] = {{0.01,0},{0,0.01}}; // Process noise covariance matrix (can be changed)

  // Define the measurement noise covariance matrix
  // The measurement noise covariance matrix is a NUM_NODES x NUM_NODES diagonal matrix with noise levels as the diagonal elements
  float R[NUM_NODES][NUM_NODES]; // Measurement noise covariance matrix
  for (int i = 0; i < NUM_NODES; i++) {
    for (int j = 0; j < NUM_NODES; j++) {
      if (i == j) {
        R[i][j] = data[i].noiseLevel;
      } else {
        R[i][j] = 0;
      }
    }
  }

  // Predict the next state vector and state covariance matrix using the state transition matrix and the process noise covariance matrix
  float x_pred[2]; // Predicted state vector
  float P_pred[2][2]; // Predicted state covariance matrix
  for (int i = 0; i < 2; i++) {
    x_pred[i] = 0;
    for (int j = 0; j < 2; j++) {
      x_pred[i] += F[i][j] * x[j]; // Apply state transition matrix to state vector
      P_pred[i][j] = Q[i][j]; // Copy process noise covariance matrix to predicted state covariance matrix
      for (int k = 0; k < 2; k++) {
        P_pred[i][j] += F[i][k] * P[k][j] * F[j][k]; // Add state transition matrix and state covariance matrix product to predicted state covariance matrix
      }
    }
  }

  // Update the state vector and state covariance matrix using the measurement vector, the measurement matrix, and the measurement noise covariance matrix
  float y[NUM_NODES]; // Innovation vector
  float S[NUM_NODES][NUM_NODES]; // Innovation covariance matrix
  float K[2][NUM_NODES]; // Kalman gain matrix
  for (int i = 0; i < NUM_NODES; i++) {
    y[i] = z[i]; // Copy measurement vector to innovation vector
    for (int j = 0; j < 2; j++) {
      y[i] -= H[i][j] * x_pred[j]; // Subtract measurement matrix and predicted state vector product from innovation vector
      K[j][i] = 0;
      for (int k = 0; k < NUM_NODES; k++) {
        S[i][k] = R[i][k]; // Copy measurement noise covariance matrix to innovation covariance matrix
        for (int l = 0; l < 2; l++) {
          S[i][k] += H[i][l] * P_pred[l][k] * H[k][l]; // Add measurement matrix and predicted state covariance matrix product to innovation covariance matrix
          K[j][i] += P_pred[j][l] * H[k][l]; // Add predicted state covariance matrix and measurement matrix product to Kalman gain matrix
        }
      }
    }
  }

  // Invert the innovation covariance matrix using Gauss-Jordan elimination
  float S_inv[NUM_NODES][NUM_NODES]; // Inverse of innovation covariance matrix
  for (int i = 0; i < NUM_NODES; i++) {
    for (int j = 0; j < NUM_NODES; j++) {
      if (i == j) {
        S_inv[i][j] = 1; // Initialize inverse matrix with identity matrix
      } else {
        S_inv[i][j] = 0;
      }
    }
  }
  for (int i = 0; i < NUM_NODES; i++) {
    float pivot = S[i][i]; // Find pivot element
    if (pivot == 0) { // If pivot element is zero, swap rows with a nonzero element
      for (int j = i + 1; j < NUM_NODES; j++) {
        if (S[j][i] != 0) {
          for (int k = 0; k < NUM_NODES; k++) {
            float temp = S[i][k];
            S[i][k] = S[j][k];
            S[j][k] = temp;
            temp = S_inv[i][k];
            S_inv[i][k] = S_inv[j][k];
            S_inv[j][k] = temp;
          }
          pivot = S[i][i];
          break;
        }
      }
    }
    if (pivot != 0) { // If pivot element is nonzero, divide row by pivot element
      for (int j = 0; j < NUM_NODES; j++) {
        S[i][j] /= pivot;
        S_inv[i][j] /= pivot;
      }
      for (int j = 0; j < NUM_NODES; j++) { // Subtract multiples of pivot row from other rows
        if (j != i) {
          float factor = S[j][i];
          for (int k = 0; k < NUM_NODES; k++) {
            S[j][k] -= factor * S[i][k];
            S_inv[j][k] -= factor * S_inv[i][k];
          }
        }
      }
    }
  }

  // Multiply the Kalman gain matrix and the inverse of the innovation covariance matrix
  float K_S_inv[2][NUM_NODES]; // Product of Kalman gain matrix and inverse of innovation covariance matrix
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < NUM_NODES; j++) {
      K_S_inv[i][j] = 0;
      for (int k = 0; k < NUM_NODES; k++) {
        K_S_inv[i][j] += K[i][k] * S_inv[k][j];
      }
    }
  }

  // Multiply the product of Kalman gain matrix and inverse of innovation covariance matrix and the innovation vector
  float x_update[2]; // Update for state vector
  for (int i = 0; i < 2; i++) {
    x_update[i] = 0;
    for (int j = 0; j < NUM_NODES; j++) {
      x_update[i] += K_S_inv[i][j] * y[j];
    }
  }

  // Add the update to the predicted state vector to get the updated state vector
  float x_new[2]; // Updated state vector
  for (int i = 0; i < 2; i++) {
    x_new[i] = x_pred[i] + x_update[i];
  }

  // Multiply the product of Kalman gain matrix and inverse of innovation covariance matrix and the measurement matrix
  float K_S_inv_H[2][2]; // Product of Kalman gain matrix, inverse of innovation covariance matrix, and measurement matrix
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      K_S_inv_H[i][j] = 0;
      for (int k = 0; k < NUM_NODES; k++) {
        K_S_inv_H[i][j] += K_S_inv[i][k] * H[k][j];
      }
    }
  }

  // Subtract the product of Kalman gain matrix, inverse of innovation covariance matrix, and measurement matrix from the identity matrix
  float I_K_S_inv_H[2][2]; // Difference of identity matrix and product of Kalman gain matrix, inverse of innovation covariance matrix, and measurement matrix
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      if (i == j) {
        I_K_S_inv_H[i][j] = 1 - K_S_inv_H[i][j];
      } else {
        I_K_S_inv_H[i][j] = -K_S_inv_H[i][j];
      }
    }
  }

  // Multiply the difference of identity matrix and product of Kalman gain matrix, inverse of innovation covariance matrix, and measurement matrix and the predicted state covariance matrix
  float P_new[2][2]; // Updated state covariance matrix
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      P_new[i][j] = 0;
      for (int k = 0; k < 2; k++) {
        P_new[i][j] += I_K_S_inv_H[i][k] * P_pred[k][j];
      }
    }
  }

  // Update the state vector and state covariance matrix with the new values
  for (int i = 0; i < 2; i++) {
    x[i] = x_new[i];
    for (int j = 0; j < 2; j++) {
      P[i][j] = P_new[i][j];
    }
  }

  // Update the timer error and noise level with the new state vector values
  data[nodeIndex].timerError = x[0];
  data[nodeIndex].noiseLevel = x[1];

  // Print the timer error and noise level for debugging purposes
  Serial.print("Timer error: ");
  Serial.println(data[nodeIndex].timerError);
  Serial.print("Noise level: ");
  Serial.println(data[nodeIndex].noiseLevel);
}

// Correct timer value based on timer error and noise level
void correctTimer() {
  // Subtract the timer error from the timer value
  data[nodeIndex].timerVal -= data[nodeIndex].timerError;

  // Add a random noise to the timer value based on the noise level
  data[nodeIndex].timerVal += random(-data[nodeIndex].noiseLevel, data[nodeIndex].noiseLevel);

  // Print the corrected timer value for debugging purposes
  Serial.print("Corrected timer value: ");
  Serial.println(data[nodeIndex].timerVal);
}

// Setup function
void setup() {
  Serial.begin(115200); // Start serial communication at 115200 baud rate
  setupWiFi(); // Set up WiFi connection
  setupData(); // Set up initial data values
}

// Loop function
void loop() {
  // Use a non-blocking loop to send and receive data, calculate delay, adjust frequency, apply Kalman filter, and correct timer
  sendData(); // Send data to other nodes
  recvData(); // Receive data from other nodes
  calcDelay(); // Calculate packet delay time
  adjustFreq(); // Adjust global frequency and slot number
  kalmanFilter(); // Apply Kalman filter to estimate timer error and noise level
  correctTimer(); // Correct timer value based on timer error and noise level
}
