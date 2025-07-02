#ifndef TMP126_H
#define TMP126_H

#include <Arduino.h>
#include <SPI.h>

#define TMP126_TEMP_RESULT_REG 0x00

class TMP126 {
public:
  TMP126(uint8_t cs_pin);

  void begin();
  float readTemperature();
  uint16_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint16_t value);
  bool verifyDevice();

private:
  uint8_t _cs;
};

#endif // TMP126_H
