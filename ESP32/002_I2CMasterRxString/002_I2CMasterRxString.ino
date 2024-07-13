// Wire Slave Transmitter and receiver
#include <Wire.h>

int LED = 2;
uint8_t active_command = 0xff, led_status = 0;
char name_msg[32] = "Welcome to FastBit EBA\n";

uint16_t device_id = 0xFF45;

#define SLAVE_ADDR 0x68
#define SDA_PIN 21 // Pino padrão SDA para ESP32
#define SCL_PIN 22 // Pino padrão SCL para ESP32

uint8_t get_len_of_data(void) {
  return (uint8_t)strlen(name_msg);
}

void setup() {
  // Define the LED pin as Output
  pinMode(LED, OUTPUT);
  
  // Start the I2C Bus as Slave on specified address
  Wire.begin(SLAVE_ADDR);
  Serial.begin(9600);
  
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);
  //Wire.onRequest(requestEvent);
}

// write
void receiveEvent(int bytes) {
  while (Wire.available()) {
    active_command = Wire.read();
  }
  //active_command = Wire.read(); // read one character from the I2C 

  Serial.println("Received!");
}

// read
/*
void requestEvent() {
  if (active_command == 0x51) {
    uint8_t len = get_len_of_data();
    len = 5;
    Wire.write(&len, 1);
    active_command = 0xff;
  }
  
  if (active_command == 0x52) {
    //Wire.write(reinterpret_cast<const uint8_t*>(name_msg), get_len_of_data());
    uint8_t buffer[3] = {'B', 'o', 'm'};
    Wire.write(buffer, 3);

    active_command = 0xff;
  }
}*/

void loop() {
  // Empty loop
}
