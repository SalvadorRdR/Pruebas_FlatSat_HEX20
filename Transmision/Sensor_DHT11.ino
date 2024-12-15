#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 4        // Pin GPIO para el DHT11
#define DHTTYPE DHT11   // Tipo de sensor DHT11

// Pines UART2
#define TXD2 17          // GPIO17 (TX del ESP32)
#define RXD2 16          // GPIO16 (RX del ESP32)

DHT dht(DHTPIN, DHTTYPE);

// Prototipo para la función CRC16-CCITT
uint16_t calculateCRC(const uint8_t *data, size_t length);

void setup() {
  Serial.begin(115200);    // UART principal
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2); // UART adicional: TX=16, RX=17 (ajustar pines según conexión)
  
  Serial.println("Inicializando DHT11...");
  dht.begin();
}

void loop() {
  delay(2000); // Espera entre lecturas

  // Leer humedad y temperatura
  float humedad = dht.readHumidity();
  float temperatura = dht.readTemperature();

  if (isnan(humedad) || isnan(temperatura)) {
    Serial.println("¡Error al leer el DHT11!");
    return;
  }

  // Crear el paquete de datos
  uint8_t packet[60] = { 
    0x48, 0x32, 0x30,       // Hex20 Header
    0x01,                   // Version
    0x51,                   // APID
    0x00, 0x01,             // Sequence Number (ejemplo)
    0x00, 0x2E,             // Packet Size: 46 bytes + CRC (2 bytes)
    0x00, 0x00, 0x00,       // OBC Time Hdr (placeholder)
    0x11, 0x00, 0x2A        // CRP Code y Data Size (42 bytes)
  };

  // Insertar User Data Field
  packet[26] = (uint8_t)temperatura; // Temperatura en Byte[26]
  packet[27] = (uint8_t)humedad;    // Humedad en Byte[27]
  
  // Llenar otros campos con 0x00
  for (int i = 28; i < 57; i++) {
    packet[i] = 0x00;
  }

  // Calcular CRC sobre los primeros 58 bytes
  uint16_t crc = calculateCRC(packet, 58);
  packet[58] = (crc >> 8) & 0xFF; // Byte alto del CRC
  packet[59] = crc & 0xFF;        // Byte bajo del CRC

  // Enviar paquete por UART1
  Serial1.write(packet, sizeof(packet));

  // Mostrar paquete en el monitor serial para verificación
  Serial.println("Paquete enviado por UART (en HEX):");
  for (int i = 0; i < 60; i++) {
    Serial.printf("%02X ", packet[i]);
  }
  Serial.println();
}

// Función para calcular el CRC16-CCITT
uint16_t calculateCRC(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF; // Valor inicial del CRC
  for (size_t i = 0; i < length; i++) {
    crc ^= (data[i] << 8);
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}
