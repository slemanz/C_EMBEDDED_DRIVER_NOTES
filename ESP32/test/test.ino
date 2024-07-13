#include <Wire.h>

#define I2C_SLAVE_ADDRESS 0x68  // Set your I2C slave address here

String dataToSend = "Hello!";
bool dataAvailable = false;

void setup() {
  Wire.begin(I2C_SLAVE_ADDRESS);  // Initialize I2C with the desired slave address
  Wire.onRequest(onRequest);  // Set up callback function for when data is requested by the master
  Wire.onReceive(onReceive);  // Set up callback function for when data is received from the master

  Serial.begin(9600);  // Initialize Serial communication for debugging
  Serial.println("I2C Slave initialized.");
}

void loop() {
  // Main loop can be used for other tasks
}

void onRequest() {
  if (dataAvailable) {
    Wire.write(dataToSend.c_str());  // Send the string data to the master
    dataAvailable = false;           // Reset the flag after sending data
    Serial.println("Data sent to master: " + dataToSend);
  }
}

void onReceive(int numBytes) {
  while (Wire.available()) {
    char c = Wire.read();  // Read the received data
    if (c == 'R') {  // If the data is 'R', prepare to send data
      dataAvailable = true;
      Serial.println("Data request received from master.");
    }
  }
}
