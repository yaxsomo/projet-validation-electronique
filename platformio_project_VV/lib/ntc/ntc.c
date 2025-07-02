#include "ntc.h"
#include <Arduino.h>

#define TABLE_SIZE 16
static const float tempTable[TABLE_SIZE] = {-40, -30, -20, -10, 0, 10, 20, 25, 30, 40, 50, 60, 70, 80, 90, 100};
static const float resTable[TABLE_SIZE] = {
  336460, 182130, 101170, 58390, 34530, 20970,
  13100, 10000, 7670, 4710, 2940, 1890, 1230, 825, 565, 395
};

void ntc_init() {
  // Initialize Analog Pins
  analogReadResolution(12); // 12-bit resolution
  pinMode(NTC1_PIN, INPUT);
  pinMode(NTC2_PIN, INPUT);
}


float read_ntc_temperature(NTC_devices_t device) {
  const float R1 = 10000.0; // Pull-up resistor
  int adcValue = 0;

  switch (device) {
    case NTC1:
      adcValue = analogRead(NTC1_PIN);
      break;
    case NTC2:
      adcValue = analogRead(NTC2_PIN);
      break;
    default:
      return -1000; // Invalid reading
  }

  float Vout = adcValue * 3.3 / 4095.0;
  if (Vout <= 0.0 || Vout >= 3.3) return -1000; // Avoid divide by zero

  float Rntc = R1 * Vout / (3.3 - Vout);

  // Search for bounding indices
  for (int i = 0; i < TABLE_SIZE - 1; ++i) {
    if (Rntc <= resTable[i] && Rntc >= resTable[i + 1]) {
      // Linear interpolation
      float t1 = tempTable[i];
      float t2 = tempTable[i + 1];
      float r1 = resTable[i];
      float r2 = resTable[i + 1];
      float temp = t1 + (Rntc - r1) * (t2 - t1) / (r2 - r1);
      return temp;
    }
  }

  return -1000; // Out of range
}