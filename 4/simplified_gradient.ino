
// Arduino code for esp8266 synchronizing millis() timer with other devices in the subnet
// by exchanging udp broadcast packets containing data useful for the task

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// WiFi network settings
const char* ssid = "your-ssid"; // your WiFi network name
const char* password = "your-password"; // your WiFi network password

// UDP settings
const int localPort = 8888; // local port to listen for UDP packets
const int broadcastPort = 9999; // broadcast port to send UDP packets
const int packetSize = 32; // size of UDP packet in bytes
const int maxDevices = 10; // maximum number of devices in the subnet

// Timer synchronization settings
const unsigned long syncInterval = 1000; // interval in milliseconds to send and receive UDP packets
const unsigned long timeSlot = syncInterval / maxDevices; // time slot in milliseconds for each device to send UDP packet
const float alpha = 0.1; // learning rate for gradient descent algorithm
const float epsilon = 0.01; // threshold for convergence of gradient descent algorithm

// Global variables
WiFiUDP udp; // UDP object
byte packetBuffer[packetSize]; // buffer to hold incoming and outgoing packets
unsigned long lastSyncTime = 0; // last time a UDP packet was sent or received
unsigned long timerOffset = 0; // offset to adjust the local timer
int deviceID = 0; // unique ID of this device based on its MAC address
int signalLevel = 0; // WiFi signal level in percentage
int packetLoss = 0; // number of packets lost in the last sync interval
int estimatedLoss = 0; // estimated number of packets lost based on signal level and packet size
float timerError = 0; // estimated error of the local timer based on packet loss and timer offset

void setup() {
  Serial.begin(115200); // initialize serial communication
  WiFi.begin(ssid, password); // connect to WiFi network
  while (WiFi.status() != WL_CONNECTED) { // wait until WiFi is connected
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP()); // print local IP address

  udp.begin(localPort); // start listening for UDP packets on local port
  Serial.print("Listening on UDP port ");
  Serial.println(localPort);

  deviceID = getDeviceID(); // get unique device ID based on MAC address
  Serial.print("Device ID: ");
  Serial.println(deviceID);
}

void loop() {
  unsigned long currentTime = millis(); // get current time in milliseconds

  if (currentTime - lastSyncTime >= syncInterval) { // check if it is time to sync timers
    lastSyncTime = currentTime; // update last sync time

    signalLevel = WiFi.RSSI(); // get WiFi signal level in dBm
    signalLevel = map(signalLevel, -100, -50, 0, 100); // map signal level from dBm to percentage
    signalLevel = constrain(signalLevel, 0, 100); // constrain signal level between 0 and 100

    estimatedLoss = estimatePacketLoss(signalLevel, packetSize); // estimate number of packets lost based on signal level and packet size

    timerError = calculateTimerError(packetLoss, estimatedLoss, timerOffset); // calculate timer error based on packet loss and timer offset

    timerOffset = adjustTimerOffset(timerError, alpha); // adjust timer offset using gradient descent algorithm

    sendPacket(); // send UDP packet with data useful for the task

    packetLoss = 0; // reset packet loss counter

    printData(); // print data to serial monitor for debugging purposes
  }

  if (udp.parsePacket()) { // check if a UDP packet is available
    receivePacket(); // receive UDP packet and store it in buffer

    unsigned long receivedTime = currentTime + timerOffset; // get received time adjusted by timer offset

    int senderID = packetBuffer[0]; // get sender ID from packet buffer

    if (senderID != deviceID) { // check if sender ID is different from device ID
      unsigned long senderTime = readLong(1); // get sender time from packet buffer
      
      int senderSignalLevel = packetBuffer[5]; // get sender signal level from packet buffer
      
      int senderPacketLoss = packetBuffer[6]; // get sender packet loss from packet buffer
      
      int senderEstimatedLoss = packetBuffer[7]; // get sender estimated loss from packet buffer
      
      float senderTimerError = readFloat(8); // get sender timer error from packet buffer

      if (abs(senderTime - receivedTime) > timeSlot / 2) { // check if sender time and received time are out of sync by more than half of the time slot
        packetLoss++; // increment packet loss counter
      }

      Serial.print("Received packet from device ");
      Serial.println(senderID);
      Serial.print("Sender time: ");
      Serial.println(senderTime);
      Serial.print("Received time: ");
      Serial.println(receivedTime);
      Serial.print("Sender signal level: ");
      Serial.println(senderSignalLevel);
      Serial.print("Sender packet loss: ");
      Serial.println(senderPacketLoss);
      Serial.print("Sender estimated loss: ");
      Serial.println(senderEstimatedLoss);
      Serial.print("Sender timer error: ");
      Serial.println(senderTimerError);
    }
  }
}

// Function to get unique device ID based on MAC address
int getDeviceID() {
  byte mac[6]; // array to store MAC address
  WiFi.macAddress(mac); // get MAC address
  int id = 0; // variable to store device ID
  for (int i = 0; i < 6; i++) { // loop through MAC address bytes
    id += mac[i]; // add byte value to device ID
  }
  id = id % maxDevices; // modulo device ID by maximum number of devices
  return id; // return device ID
}

// Function to estimate number of packets lost based on signal level and packet size
int estimatePacketLoss(int signalLevel, int packetSize) {
  float ber = pow(10, -signalLevel / 10); // calculate bit error rate based on signal level
  float per = 1 - pow(1 - ber, packetSize * 8); // calculate packet error rate based on bit error rate and packet size
  int loss = round(per * syncInterval / timeSlot); // calculate number of packets lost based on packet error rate and sync interval
  return loss; // return number of packets lost
}

// Function to calculate timer error based on packet loss and timer offset
float calculateTimerError(int packetLoss, int estimatedLoss, unsigned long timerOffset) {
  float error = (packetLoss - estimatedLoss) * timeSlot + timerOffset; // calculate timer error based on packet loss, estimated loss and timer offset
  return error; // return timer error
}

// Function to adjust timer offset using gradient descent algorithm
unsigned long adjustTimerOffset(float timerError, float alpha) {
  unsigned long offset = timerOffset - alpha * timerError; // update timer offset using gradient descent formula
  return offset; // return timer offset
}

// Function to send UDP packet with data useful for the task
void sendPacket() {
  unsigned long sendTime = millis() + timerOffset; // get send time adjusted by timer offset

  int sendSlot = deviceID * timeSlot; // get send slot based on device ID and time slot

  while (millis() < sendTime + sendSlot) { // wait until send slot is reached
    delay(1);
  }

  packetBuffer[0] = deviceID; // write device ID to packet buffer

  writeLong(1, sendTime); // write send time to packet buffer

  packetBuffer[5] = signalLevel; // write signal level to packet buffer

  packetBuffer[6] = packetLoss; // write packet loss to packet buffer

  packetBuffer[7] = estimatedLoss; // write estimated loss to packet buffer

  writeFloat(8, timerError); // write timer error to packet buffer

  udp.beginPacket(IPAddress(255, 255, 255, 255), broadcastPort); // start UDP packet to broadcast address and port
  udp.write(packetBuffer, packetSize); // write packet buffer to UDP packet
  udp.endPacket(); // end UDP packet

  Serial.print("Sent packet at ");
  Serial.println(sendTime);
}

// Function to receive UDP packet and store it in buffer
void receivePacket() {
  udp.read(packetBuffer, packetSize); // read UDP packet and store it in packet buffer
}

// Function to write a long value to packet buffer at a given index
void writeLong(int index, unsigned long value) {
  byte* bytes = (byte*) &value; // get pointer to bytes of value
  for (int i = 0; i < 4; i++) { // loop through bytes
    packetBuffer[index + i] = bytes[i]; // write byte to packet buffer
  }
}

// Function to read a long value from packet buffer at a given index
unsigned long readLong(int index) {
  unsigned long value = 0; // variable to store value
  byte* bytes = (byte*) &value; // get pointer to bytes of value
  for (int i = 0; i < 4; i++) { // loop through bytes
    bytes[i] = packetBuffer[index + i]; // read byte from packet buffer
  }
  return value; // return value
}

// Function to write a float value to packet buffer at a given index
void writeFloat(int index, float value) {
  byte* bytes = (byte*) &value; // get pointer to bytes of value
  for (int i = 0; i < 4; i++) { // loop through bytes
    packetBuffer[index + i] = bytes[i]; // write byte to packet buffer
  }
}

// Function to read a float value from packet buffer at a given index
float readFloat(int index) {
  float value = 0; // variable to store value
  byte* bytes = (byte*) &value; // get pointer to bytes of value
  for (int i = 0; i < 4; i++) { // loop through bytes
    bytes[i] = packetBuffer[index + i]; // read byte from packet buffer
  }
  return value; // return value
}

// Function to print data to serial monitor for debugging purposes
void printData() {
  Serial.print("Signal level: ");
  Serial.println(signalLevel);
  Serial.print("Packet loss: ");
  Serial.println(packetLoss);
  Serial.print("Estimated loss: ");
  Serial.println(estimatedLoss);
  Serial.print("Timer error: ");
  Serial.println(timerError);
  Serial.print("Timer offset: ");
  Serial.println(timerOffset);
  Serial.println();
}
