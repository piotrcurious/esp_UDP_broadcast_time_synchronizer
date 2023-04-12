
// Define constants
#define NUM_NODES 4 // Number of nodes in
the network
#define MAX_GNPSF 1000 // Maximum value of GNPSF in milliseconds
#define MIN_GNPSF 100 // Minimum value of GNPSF in milliseconds
#define MAX_OTS 1000 // Maximum value of OTS in milliseconds
#define MIN_OTS 0 // Minimum value of OTS in milliseconds
#define LED_PIN 2 // Pin number for LED

// Define UDP packet structure
struct Packet {
  uint32_t senderIP; // Sender IP address
  uint32_t gnpsf; // Global network packet sending frequency
  uint32_t ots; // Optimal time slot
  uint32_t millis; // Millis() value
  uint8_t wifiSignal; // WiFi signal level
  uint8_t wifiNoise; // WiFi noise level
  uint8_t estPacketLoss; // Estimated packet loss
  uint8_t estTimerError; // Estimated timer error
};

// Define Kalman filter model
struct KalmanFilter {
  float x[4]; // State vector: [gnpsf, ots, timer offset, timer drift]
  float P[4][4]; // State covariance matrix
  float F[4][4]; // State transition matrix
  float Q[4][4]; // State transition noise matrix
  float H[4][4]; // Measurement matrix
  float R[4][4]; // Measurement noise matrix
};

// Define global variables
uint32_t localIP; // Local IP address
uint32_t gnpsf; // Local GNPSF parameter
uint32_t ots; // Local OTS parameter
KalmanFilter kf; // Local Kalman filter object
Packet packetBuffer[NUM_NODES]; // Buffer for storing received packets

// Define functions

// Function to initialize the Kalman filter with default values
void initKalmanFilter() {
  // Set state vector to zero
  for (int i = 0; i < 4; i++) {
    kf.x[i] = 0;
  }
  // Set state covariance matrix to large values
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      if (i == j) {
        kf.P[i][j] = 1000;
      } else {
        kf.P[i][j] = 0;
      }
    }
  }
  // Set state transition matrix to identity
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      if (i == j) {
        kf.F[i][j] = 1;
      } else {
        kf.F[i][j] = 0;
      }
    }
  }
  // Set state transition noise matrix to small values
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      if (i == j) {
        kf.Q[i][j] = 0.01;
      } else {
        kf.Q[i][j] = 0;
      }
    }
  }
  // Set measurement matrix to map state variables to packet fields
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      if (i == j) {
        kf.H[i][j] = 1;
      } else {
        kf.H[i][j] = 0;
      }
    }
  }
}

// Function to update the Kalman filter with a new packet
void updateKalmanFilter(Packet p) {
  // Predict the next state and covariance using the state transition matrix and noise
  float x_pred[4]; // Predicted state vector
  float P_pred[4][4]; // Predicted state covariance matrix
  for (int i = 0; i < 4; i++) {
    x_pred[i] = 0;
    for (int j = 0; j < 4; j++) {
      x_pred[i] += kf.F[i][j] * kf.x[j]; // Matrix multiplication
      P_pred[i][j] = 0;
      for (int k = 0; k < 4; k++) {
        P_pred[i][j] += kf.F[i][k] * kf.P[k][j]; // Matrix multiplication
      }
      P_pred[i][j] += kf.Q[i][j]; // Matrix addition
    }
  }
  // Calculate the measurement residual and residual covariance using the measurement matrix and noise
  float z[4]; // Measurement vector from packet fields
  z[0] = p.gnpsf;
  z[1] = p.ots;
  z[2] = p.millis - x_pred[1]; // Timer offset is measured as packet millis minus local OTS
  z[3] = p.gnpsf - x_pred[0]; // Timer drift is measured as packet GNPSF minus local GNPSF
  float y[4]; // Measurement residual vector
  float S[4][4]; // Residual covariance matrix
  for (int i = 0; i < 4; i++) {
    y[i] = z[i] - x_pred[i]; // Vector subtraction
    for (int j = 0; j < 4; j++) {
      S[i][j] = 0;
      for (int k = 0; k < 4; k++) {
        S[i][j] += kf.H[i][k] * P_pred[k][j]; // Matrix multiplication
      }
      S[i][j] += kf.R[i][j]; // Matrix addition
    }
  }
  // Calculate the Kalman gain using the residual covariance and the state transition matrix
  float K[4][4]; // Kalman gain matrix
  float S_inv[4][4]; // Inverse of residual covariance matrix
  invertMatrix(S, S_inv); // Function to invert a matrix using Gaussian elimination or other methods
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      K[i][j] = 0;
      for (int k = 0; k < 4; k++) {
        K[i][j] += P_pred[i][k] * kf.H[k][j]; // Matrix multiplication
      }
      K[i][j] *= S_inv[j][i]; // Matrix multiplication by inverse transpose
    }
  }
  
// Correct the state estimate and covariance using the measurement residual and the Kalman gain
float x_corr[4]; // Corrected state vector
float P_corr[4][4]; // Corrected state covariance matrix
for (int i = 0; i < 4; i++) {
x_corr[i] = x_pred[i];
for (int j = 0; j < 4; j++) {
x_corr[i] += K[i][j] * y[j]; // Matrix multiplication and addition
P_corr[i][j] = P_pred[i][j];
for (int k = 0; k < 4; k++) {
P_corr[i][j] -= K[i][k] * S[k][j]; // Matrix multiplication and subtraction
}
}
}
// Update the local Kalman filter object with the corrected values
for (int i = 0; i < 4; i++) {
kf.x[i] = x_corr[i];
for (int j = 0; j < 4; j++) {
kf.P[i][j] = P_corr[i][j];
}
}
}

// Function to adjust the GNPSF and OTS parameters based on the state estimate
void adjustParameters() {
  // Calculate the desired GNPSF as the average GNPSF of the network plus a correction term proportional to the timer drift
  float avgGNPSF = kf.x[0]; // Average GNPSF of the network from state estimate
  float timerDrift = kf.x[3]; // Timer drift from state estimate
  float k1 = 0.1; // Proportionality constant for GNPSF correction
  float desGNPSF = avgGNPSF + k1 * timerDrift; // Desired GNPSF
  // Constrain the desired GNPSF to be within reasonable limits
  if (desGNPSF > MAX_GNPSF) {
    desGNPSF = MAX_GNPSF;
  } else if (desGNPSF < MIN_GNPSF) {
    desGNPSF = MIN_GNPSF;
  }
  // Set the local GNPSF to be equal to the desired value
  gnpsf = (uint32_t) desGNPSF;
  
  // Calculate the desired OTS as the average OTS of the network plus a correction term proportional to the timer offset
  float avgOTS = kf.x[1]; // Average OTS of the network from state estimate
  float timerOffset = kf.x[2]; // Timer offset from state estimate
  float k2 = 0.1; // Proportionality constant for OTS correction
  float desOTS = avgOTS + k2 * timerOffset; // Desired OTS
  // Constrain the desired OTS to be within reasonable limits
  if (desOTS > MAX_OTS) {
    desOTS = MAX_OTS;
  } else if (desOTS < MIN_OTS) {
    desOTS = MIN_OTS;
  }
  // Set the local OTS to be equal to the desired value
  ots = (uint32_t) desOTS;
}

// Function to pulse the LED and report diagnostics based on the state estimate
void pulseLED() {
  // Check if millis() is equal to or close to OTS modulo GNPSF
  uint32_t currentTime = millis(); // Current millis() value
  uint32_t timeDiff = currentTime - ots; // Difference between current time and OTS
  uint32_t timeMod = timeDiff % gnpsf; // Modulo operation by GNPSF
  uint32_t timeTol = gnpsf / 10; // Tolerance for timing error in milliseconds
  if (timeMod < timeTol || timeMod > gnpsf - timeTol) {
    // If yes, turn on the LED for a short duration
    digitalWrite(LED_PIN, HIGH); // Turn on LED
    delay(10); // Wait for 10 milliseconds
    digitalWrite(LED_PIN, LOW); // Turn off LED
  } else {
    // Otherwise, turn off the LED
    digitalWrite(LED_PIN, LOW); // Turn off LED
  }
  
// Print out the current values of millis(), GNPSF, OTS, timer offset, timer drift, WiFi signal level, WiFi noise level,
// estimated packet loss, estimated timer error to the serial console
Serial.print("Millis: ");
Serial.println(currentTime);
Serial.print("GNPSF: ");
Serial.println(gnpsf);
Serial.print("OTS: ");
Serial.println(ots);
Serial.print("Timer offset: ");
Serial.println(kf.x[2]);
Serial.print("Timer drift: ");
Serial.println(kf.x[3]);
Serial.print("WiFi signal level: ");
Serial.println(getWiFiSignalLevel()); // Function to get WiFi signal level
Serial.print("WiFi noise level: ");
Serial.println(getWiFiNoiseLevel()); // Function to get WiFi noise level
Serial.print("Estimated packet loss: ");
Serial.println(getEstimatedPacketLoss()); // Function to get estimated packet loss
Serial.print("Estimated timer error: ");
Serial.println(getEstimatedTimerError()); // Function to get estimated timer error

}

// Main setup function
void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  // Initialize WiFi connection
  WiFi.begin("SSID", "PASSWORD"); // Replace with your WiFi credentials
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  // Get local IP address
  localIP = WiFi.localIP();
  Serial.print("IP address: ");
  Serial.println(localIP);
  // Initialize UDP socket
  UDP.begin(1234); // Use port 1234 for UDP communication
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  // Initialize Kalman filter
  initKalmanFilter();
  // Initialize GNPSF and OTS parameters
  gnpsf = MIN_GNPSF; // Start with minimum GNPSF
  ots = localIP % gnpsf; // Start with OTS based on IP address modulo GNPSF
}

// Main loop function
void loop() {
  // Check if there is a new UDP packet available
  int packetSize = UDP.parsePacket();
  if (packetSize) {
    // If yes, read the packet and store it in the buffer
    Packet p;
    UDP.read((char*)&p, sizeof(p)); // Read the packet as a byte array and cast it to a Packet structure
    packetBuffer[p.senderIP % NUM_NODES] = p; // Store the packet in the buffer using the sender IP modulo NUM_NODES as the index
    Serial.print("Received packet from ");
    Serial.println(p.senderIP);
    // Update the Kalman filter with the new packet
    updateKalmanFilter(p);
    // Adjust the GNPSF and OTS parameters based on the state estimate
    adjustParameters();
  }
  
// Pulse the LED and report diagnostics based on the state estimate
pulseLED();

// Send a UDP broadcast packet with the local parameters and values
Packet q;
q.senderIP = localIP;
q.gnpsf = gnpsf;
q.ots = ots;
q.millis = millis();
q.wifiSignal = getWiFiSignalLevel();
q.wifiNoise = getWiFiNoiseLevel();
q.estPacketLoss = getEstimatedPacketLoss();
q.estTimerError = getEstimatedTimerError();
UDP.beginPacket(IPAddress(255,255,255,255), 1234); // Use broadcast address and port for UDP communication
UDP.write((char*)&q, sizeof(q)); // Write the packet as a byte array and send it
UDP.endPacket();
Serial.print("Sent packet to ");
Serial.println(IPAddress(255,255,255,255));
}
