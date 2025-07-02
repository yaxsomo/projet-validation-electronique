// Native Libraries
#include <Arduino.h>
#include <Wire.h>

#include <SPI.h>
#include <BLEDevice.h>
#include <string.h>
#include <Adafruit_INA237.h>
#include "event_logger/event_logger.h"
#include "alarm_manager/alarm_manager.h"

// Custom Libraries
#include "configuration.h"
#include "tmp126/tmp126.h"
#include <ntc/ntc.h>
#include "ina237/ina237_manager.h"
#include "ble/ble_manager.h"
#include "buzzer/buzzer.h"
#include "rg_led/rg_led.h"
#include "i2c/i2c_scanner.h"



// Peripheral Objects
extern Adafruit_INA237 ina237;
TMP126 tmp126(TMP126_CS);
extern bool deviceConnected;
extern BLECharacteristic *pCharacteristic;
std::string lastValue = "";


void separator(){
  Serial.println("");
  Serial.println("--------------------------------------------------");
  Serial.println("");
}

void setup() {

  // 1. Initialize Serial Communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB 
  }

  delay(5000); // Wait for 5 secondS to be able to connect to the serial monitor
  Serial.println("Starting setup...");

  Serial.printf("Initializing BLE...");
  setup_ble();
  Serial.println("BLE initialized and advertising as ESP32-Yaxsomo");

  separator(); // -------------------------------------

  // 2. Initialize Buzzer PWM
  Serial.printf("Initializing Buzzer...");
  ledcSetup(0, 2000, 8);      // Channel 0, 2kHz frequency, 8-bit resolution
  ledcAttachPin(BUZZER_PIN, 0);
  ledcWrite(0, 0);            // Turn off the buzzer initially
  Serial.printf("Done.\n");

  // Initialize RG LED PWM
  Serial.printf("Initializing RG LED...");
  ledcSetup(LED_GREEN_PWM_CHANNEL, 5000, 8); // 5kHz, 8-bit
  ledcAttachPin(LED_GREEN_PIN, LED_GREEN_PWM_CHANNEL);
  ledcWrite(LED_GREEN_PWM_CHANNEL, 0); // Off

  ledcSetup(LED_RED_PWM_CHANNEL, 5000, 8);
  ledcAttachPin(LED_RED_PIN, LED_RED_PWM_CHANNEL);
  ledcWrite(LED_RED_PWM_CHANNEL, 0);
  Serial.printf("Done.\n");

  separator(); // -------------------------------------

  // Initialize I2C
  Serial.printf("Initializing I2C...");
  Wire.begin();
  Wire.setPins(21, 22);
  Serial.printf("Done.\n");

  separator(); // -------------------------------------

  //Initialize Analog Pins
  Serial.printf("Initializing Analog Pins...");
  ntc_init();
  Serial.printf("Done.\n");

  separator(); // -------------------------------------

  // Initialize TMP126
  Serial.printf("Initializing TMP126...");
  tmp126.begin();
  // if (!tmp126.verifyDevice()) {
  // Serial.println("❌ TMP126 verification failed!");
  // raiseAlarm(ALARM_SENSOR_MISSING);
  // while (1); // Halt system
  // }
  Serial.printf("Done.\n");

  separator(); // -------------------------------------

  // Scan I2C devices
  scan_i2c_port();

  separator(); // -------------------------------------

  //Setup current sensor
  Serial.printf("Setting up current sensor...");
  setup_current_sensor();
  Serial.printf("Done.\n");

  separator(); // -------------------------------------

  // End of setup
  Serial.println("Setup done!");

  initEventLogger();
  printAllEvents();
  // Example: raise alarm if a test fails
  // raiseAlarm(ALARM_SENSOR_MISSING);
}

void loop() {
  if (deviceConnected) {
    std::string value = pCharacteristic->getValue();
    pCharacteristic->setValue(""); // Clear BLE characteristic value after reading
    if (!value.empty()) {
      String received = String(value.c_str());
      lastValue = value;
      Serial.print("Received via BLE: ");
      Serial.println(received);
      CommandType cmdType = parseCommand(received);
      switch (cmdType) {
        case CMD_LED:
          test_rg_led();
          break;
        case CMD_SPI:
          Serial.println("SPI command received (not implemented).");
          break;
        case CMD_BUZZER:
          test_buzzer();
          break;
        case CMD_I2C:
          scan_i2c_port();
          break;
        case CMD_CTN:
          {
            float t1 = read_ntc_temperature(NTC1);
            float t2 = read_ntc_temperature(NTC2);
            Serial.print("NTC1: "); Serial.print(t1); Serial.print(" C, ");
            Serial.print("NTC2: "); Serial.print(t2); Serial.println(" C");
          }
          break;
        case CMD_INA:
          get_current_sensor_data();
          break;
        case CMD_NONE:
        default:
          Serial.println("Unknown command.");
          break;
      }
    }
  }

  // Monitor for alarm conditions
  static float lastTemperature = read_ntc_temperature(NTC1);
  float currentTemperature = read_ntc_temperature(NTC1);
  float deltaT = currentTemperature - lastTemperature;

  if (deltaT > 5.0) {
    raiseAlarm(ALARM_TEMPERATURE_RISE);
    Serial.println("Passer à 1 A");
  } else if (deltaT < -3.0) {
    raiseAlarm(ALARM_TEMPERATURE_SAFE);
    Serial.println("Passer à 2 A");
  }
  lastTemperature = currentTemperature;

  // Check for current spike
  float current = ina237.getCurrent_mA();
  if (current > 1500) {
    raiseAlarm(ALARM_CURRENT_SPIKE);
    Serial.println("⚠️  Current spike detected!");
  }

  delay(100); // Optional: short idle delay when not connected
}