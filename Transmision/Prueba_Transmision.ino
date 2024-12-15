#include <SPI.h> // Incluye la biblioteca SPI para la comunicación SPI


// Definición de pines para la comunicacion de datos
#define RECIEVE_PIN 16 // Pin para recibir datos
#define TRANSMIT_PIN 17 // Pin para transmitir datos
#define CLK_OUT_PIN 21 // Pin para el reloj de salida

// Definición de pines para la configuración del ADF7021
#define CE_PIN 5 // Pin para la habilitación del chip
#define SREAD_PIN 19 // Pin para la lectura de datos en serie
#define SDATA_PIN 23 // Pin para la escritura de datos en serie
#define SCLK_PIN 18 // Pin para el reloj de entrada
#define SLE_PIN 32 // Pin para la carga de datos en registros

// Definición de pines para la comunicación con el FlatSat
#define FS_TX_PIN 1 // Pin para la transmisión de datos en el FlatSat
#define FS_RX_PIN 3 // Pin para la recepción de datos en el FlatSat

// Definición de los valores de los registros del ADF7021
const uint32_t REG0 = 0x11616A20; // Valor del registro 0
const uint32_t REG1 = 0x00475011; // Valor del registro 1
const uint32_t REG2 = 0x003DD082; // Valor del registro 2
const uint32_t REG3 = 0x2B149923; // Valor del registro 3
const uint32_t REG15 = 0x000E000F; // Valor del registro 15

// Definición de constantes para el tamaño de los paquetes de telemetría y el buffer
#define MAX_PACKET_SIZE 256 // Define el tamaño máximo de los paquetes de telemetría
#define BUFFER_SIZE 10 // Define el número de paquetes que se pueden almacenar

// Definición de buffers para almacenar los paquetes de telemetría
uint8_t telemetryBuffer[BUFFER_SIZE][MAX_PACKET_SIZE]; // Buffer para almacenar los paquetes de telemetría
uint8_t packetLengths[BUFFER_SIZE]; // Array para almacenar las longitudes de los paquetes
uint8_t bufferIndex = 0; // Índice para llevar un seguimiento del paquete actual

// Función para configurar los pines
void setupPins(){
  pinMode(SLE_PIN, OUTPUT); // Configura el pin SLE como salida
  pinMode(TRANSMIT_PIN, OUTPUT); // Configura el pin TRANSMIT como salida
  pinMode(RECIEVE_PIN, INPUT); // Configura el pin RECIEVE como entrada
  pinMode(CLK_OUT_PIN, INPUT); // Configura el pin CLK_OUT como entrada
  pinMode(SCLK_PIN, OUTPUT); // Configura el pin SCLK como salida

  pinMode(FS_RX_PIN, INPUT); // Configura el pin FS_RX como entrada
  pinMode(FS_TX_PIN, OUTPUT); // Configura el pin FS_TX como salida

  digitalWrite(SLE_PIN, LOW); // Establece el pin SLE en bajo
  digitalWrite(TRANSMIT_PIN, LOW); // Establece el pin TRANSMIT en bajo
}

void writeRegister(uint32_t data, uint8_t length) {
  // Calculate number of bytes to send
  int numBytes = (length + 7) / 8; 
  
  // Send each byte over SPI
  for (int i = numBytes - 1; i >= 0; i--) {
    SPI.transfer((uint8_t)(data >> (i * 8))); // Send each byte, MSB first
  }
}

void loadRegister(){
  digitalWrite(SLE_PIN, HIGH); 
  delay(1);
  digitalWrite(SLE_PIN, LOW); 
}

void setupTransmitter(){
  // Configure SPI for setup
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  
  digitalWrite(CE_PIN, HIGH); // Enable chip
  delay(1); // Wait for XTAL

  // Set registers
  writeRegister(REG1, 26);
  loadRegister();
  delay(1);

  writeRegister(REG3, 32);
  loadRegister();
  
  writeRegister(REG0, 32);
  loadRegister();
  delay(1);

  writeRegister(REG2, 31);
  loadRegister();
  
  writeRegister(REG15, 32);
  loadRegister();
  
  Serial.println("Transmission setup");
}

void transmitData(uint32_t data) {
  // Ensure that the data is clocked in on the positive edge of CLKOUT
  for (int i = 31; i >= 0; i--) {
    // Wait for the positive edge of CLKOUT
    while (digitalRead(CLK_OUT_PIN) == LOW);
    while (digitalRead(CLK_OUT_PIN) == HIGH);

    digitalWrite(TRANSMIT_PIN, (data >> i) & 0x01);
  }
  Serial.println("Data sent");
}

void receiveTelemetry() {
  if (Serial1.available() > 0) {
    uint8_t packet[MAX_PACKET_SIZE];
    uint8_t length = 0;
    
    // Read telemetry packet from UART
    while (Serial1.available() > 0 && length < MAX_PACKET_SIZE) {
      packet[length++] = Serial1.read();
    }

    // Store packet in the buffer
    if (bufferIndex < BUFFER_SIZE) {
      memcpy(telemetryBuffer[bufferIndex], packet, length);
      packetLengths[bufferIndex] = length;
      bufferIndex++;
    } else {
      // Buffer is full, handle overflow
      Serial.println("Buffer overflow");
    }
  }
}

void printTelemetry() {
  for (uint8_t i = 0; i < bufferIndex; i++) {
    Serial.print("Packet ");
    Serial.print(i);
    Serial.print(": ");
    for (uint8_t j = 0; j < packetLengths[i]; j++) {
      Serial.print(telemetryBuffer[i][j], BIN);
      Serial.print(" ");
    }
    Serial.println();
  }
  bufferIndex = 0; // clear buffer
}

void transmitTelemetryBuffer() {
  for (uint8_t i = 0; i < bufferIndex; i++) {
    Serial.print("Transmitting Packet ");
    Serial.print(i);
    Serial.println(":");
    for (uint8_t j = 0; j < packetLengths[i]; j++) {
      // Assuming each byte of data is sent in sequence.
      transmitData(telemetryBuffer[i][j]);
    }
    Serial.println("Packet transmitted");
  }
  bufferIndex = 0; // Clear buffer after transmission
}


void setup() {
  setupPins();
  Serial.begin(9600);
  Serial1.begin(9600); // FS Serial initialization
  setupTransmitter();  
}

void loop() {
  // transmit dummy data
  receiveTelemetry();
  printTelemetry();
}
