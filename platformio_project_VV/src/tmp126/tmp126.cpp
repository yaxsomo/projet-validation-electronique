#include "tmp126.h"
#include "configuration.h"
#include "alarm_manager/alarm_manager.h"

SPISettings TMP126_SPI_SETTINGS(1000000, MSBFIRST, SPI_MODE0);
TMP126 tmp126(TMP126_CS);
TMP126::TMP126(uint8_t cs_pin) : _cs(cs_pin) {}

void TMP126::begin() {
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH); // Deselect

  SPI.begin(TMP126_SCK, TMP126_MISO, TMP126_MOSI, _cs); // SCK, MISO, MOSI, SS

  if (!tmp126.verifyDevice()) {
    Serial.println("TMP126 verification failed!");
    raiseAlarm(ALARM_SENSOR_MISSING);
    while (1); // Halt system
  }
}

float TMP126::readTemperature() {
  uint16_t tempRaw;
  float temperature;

  uint16_t command = (1 << 8) | TMP126_TEMP_RESULT_REG; // Read temp command

  SPI.beginTransaction(TMP126_SPI_SETTINGS);
  digitalWrite(_cs, LOW);

  SPI.transfer16(command);
  tempRaw = SPI.transfer16(0x0000); // Dummy write to receive

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  delayMicroseconds(1); // Inter-transaction delay

  int16_t tempSigned = (int16_t)(tempRaw & 0xFFFC);
  tempSigned >>= 2;
  temperature = tempSigned * 0.03125f;

  return temperature;
}

uint16_t TMP126::readRegister(uint8_t reg) {
  uint16_t command = (1 << 8) | reg;
  uint16_t value;

  SPI.beginTransaction(TMP126_SPI_SETTINGS);
  digitalWrite(_cs, LOW);

  SPI.transfer16(command);
  value = SPI.transfer16(0x0000);

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  delayMicroseconds(1);
  return value;
}

void TMP126::writeRegister(uint8_t reg, uint16_t value) {
  uint16_t command = (0 << 8) | reg;

  SPI.beginTransaction(TMP126_SPI_SETTINGS);
  digitalWrite(_cs, LOW);

  SPI.transfer16(command);
  SPI.transfer16(value);

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  delayMicroseconds(1);
}

bool TMP126::verifyDevice() {
  uint16_t id = readRegister(0x0C);  // Device ID register

  Serial.printf("TMP126 Device ID: 0x%04X\n", id);
  return (id == 0x2126);
}