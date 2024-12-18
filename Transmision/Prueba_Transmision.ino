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

// Función para escribir en un registro del ADF7021
void writeRegister(uint32_t data, uint8_t length) {
  // Calcula el número de bytes a enviar
  int numBytes = (length + 7) / 8; 
  
  // Envía cada byte a través de SPI
  for (int i = numBytes - 1; i >= 0; i--) {
    SPI.transfer((uint8_t)(data >> (i * 8))); // Envía cada byte, comenzando por el byte más significativo (MSB)
  }
}

// Función para cargar los registros en el ADF7021
void loadRegister() {
  digitalWrite(SLE_PIN, HIGH); // Activa la carga de los registros
  delay(1); // Espera 1 ms
  digitalWrite(SLE_PIN, LOW); // Desactiva la carga de los registros
}

// Función para configurar el transmisor
void setupTransmitter() {
  // Configura SPI para la configuración
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  
  digitalWrite(CE_PIN, HIGH); // Habilita el chip
  delay(1); // Espera a que el cristal oscilador (XTAL) se estabilice

  // Configura los registros del ADF7021
  writeRegister(REG1, 26); // Escribe en el registro 1
  loadRegister(); // Carga el registro
  delay(1); // Espera 1 ms

  writeRegister(REG3, 32); // Escribe en el registro 3
  loadRegister(); // Carga el registro
  
  writeRegister(REG0, 32); // Escribe en el registro 0
  loadRegister(); // Carga el registro
  delay(1); // Espera 1 ms

  writeRegister(REG2, 31); // Escribe en el registro 2
  loadRegister(); // Carga el registro
  
  writeRegister(REG15, 32); // Escribe en el registro 15
  loadRegister(); // Carga el registro
  
  Serial.println("Transmission setup"); // Imprime un mensaje indicando que la configuración de transmisión está completa
}

// Función para transmitir datos
void transmitData(uint32_t data) {
  // Asegura que los datos se registren en el flanco positivo de CLKOUT
  for (int i = 31; i >= 0; i--) {
    // Espera el flanco positivo de CLKOUT
    while (digitalRead(CLK_OUT_PIN) == LOW);
    while (digitalRead(CLK_OUT_PIN) == HIGH);

    // Escribe el bit correspondiente en el pin de transmisión
    digitalWrite(TRANSMIT_PIN, (data >> i) & 0x01);
  }
}

// Función para recibir telemetría
void receiveTelemetry() {
  if (Serial1.available() > 0) { // Verifica si hay datos disponibles en el puerto serial 1
    uint8_t packet[MAX_PACKET_SIZE]; // Crea un buffer para almacenar el paquete recibido
    uint8_t length = 0; // Inicializa la longitud del paquete a 0
    
    // Leer el paquete de telemetría desde UART
    while (Serial1.available() > 0 && length < MAX_PACKET_SIZE) { // Mientras haya datos disponibles y no se exceda el tamaño máximo del paquete
      packet[length++] = Serial1.read(); // Lee un byte del puerto serial 1 y lo almacena en el buffer, incrementando la longitud
    }

    // Almacenar el paquete en el buffer
    if (bufferIndex < BUFFER_SIZE) { // Verifica si hay espacio en el buffer
      memcpy(telemetryBuffer[bufferIndex], packet, length); // Copia los datos leídos al buffer de telemetría
      packetLengths[bufferIndex] = length; // Almacena la longitud del paquete en el array de longitudes
      bufferIndex++; // Incrementa el índice del buffer
    } else {
      // El buffer está lleno, manejar el desbordamiento
      Serial.println("Buffer overflow"); // Imprime un mensaje de desbordamiento del buffer
    }
  }
}

// Función para imprimir telemetría
void printTelemetry() {
  for (uint8_t i = 0; i < bufferIndex; i++) { // Recorre cada paquete en el buffer
    Serial.print("Packet ");
    Serial.print(i);
    Serial.print(": ");
    for (uint8_t j = 0; j < packetLengths[i]; j++) { // Recorre cada byte en el paquete
      Serial.print(telemetryBuffer[i][j], BIN); // Imprime el byte en formato binario
      Serial.print(" ");
    }
    Serial.println();
  }
  bufferIndex = 0; // Limpia el buffer
}

// Función para transmitir el buffer de telemetría
void transmitTelemetryBuffer() {
  for (uint8_t i = 0; i < bufferIndex; i++) { // Recorre cada paquete en el buffer
    Serial.print("Transmitting Packet ");
    Serial.print(i);
    Serial.println(":");
    for (uint8_t j = 0; j < packetLengths[i]; j++) { // Recorre cada byte en el paquete
      // Asumiendo que cada byte de datos se envía en secuencia.
      transmitData(telemetryBuffer[i][j]); // Transmite cada byte
    }
    Serial.println("Packet transmitted");
  }
  bufferIndex = 0; // Limpia el buffer después de la transmisión
}

// Función de configuración inicial
void setup() {
  setupPins(); // Configura los pines
  Serial.begin(9600); // Inicializa la comunicación serial a 9600 baudios
  Serial1.begin(9600); // Inicializa la comunicación serial adicional a 9600 baudios
  setupTransmitter(); // Configura el transmisor
}

// Función principal de bucle
void loop() {
  receiveTelemetry(); // Recibe telemetría
  printTelemetry(); // Imprime telemetría
}