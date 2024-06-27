// Wire Slave Receiver
// ESP32 I2C Pins: SDA (default is GPIO21), SCL (default is GPIO22)
#include <Wire.h>

#define MY_ADDR   0x68

const int LED = 2;  // A maioria dos ESP32 utiliza o GPIO2 para o LED onboard
char rx_buffer[32];
uint32_t cnt = 0;
char message[50];

void setup() {
  Serial.begin(9600);
  // Define o pino do LED como saída
  pinMode(LED, OUTPUT);

  // Inicializa o barramento I2C como Slave com endereço MY_ADDR
  Wire.begin(MY_ADDR);

  // Anexa a função receiveEvent para ser chamada quando algo é recebido
  Wire.onReceive(receiveEvent);

  sprintf(message, "Slave is ready: Address 0x%x", MY_ADDR);
  Serial.println(message);
  Serial.println("Waiting for data from master");
}

void loop(void) {
  // Código no loop principal
}

void receiveEvent(int bytes) {
  while (Wire.available()) {
    rx_buffer[cnt++] = Wire.read();
  }
  rx_buffer[cnt] = '\0';
  cnt = 0;
  Serial.print("Received:");
  Serial.println((char*)rx_buffer);
}
