#include <Wire.h>
#include <Arduino.h>

// Function to scan I2C devices on the bus
// This function will print the addresses of all devices found on the I2C bus
// It will loop through addresses 1 to 126 (0x01 to 0x7E) and attempt to communicate with each one.
void scan_i2c_port(){
      // I2C scan
    Serial.println("Scanning I2C...");
    for (byte address = 1; address < 127; ++address) {
      Wire.beginTransmission(address);
      if (Wire.endTransmission() == 0) {
        Serial.printf("Found I2C device at 0x%02X\n", address);
      }
    }
}