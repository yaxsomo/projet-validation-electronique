// Native Libraries
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA237.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string.h>

// Custom Libraries
#include "configuration.h"
#include "tmp126.h"
#include "ntc.h"

// Helper function to parse command string to CommandType enum
CommandType parseCommand(String command) {
  command.trim();
  command.toUpperCase();
  if (command.startsWith("LED")) return CMD_LED;
  if (command.startsWith("SPI")) return CMD_SPI;
  if (command.startsWith("BUZZER")) return CMD_BUZZER;
  if (command.startsWith("I2C")) return CMD_I2C;
  if (command.startsWith("CTN")) return CMD_CTN;
  if (command.startsWith("INA")) return CMD_INA;
  return CMD_NONE;
}

// Peripheral Objects
Adafruit_INA237 ina237 = Adafruit_INA237();

// BLE Objects
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) { deviceConnected = true; }
  void onDisconnect(BLEServer* pServer) { deviceConnected = false; }
};

std::string lastValue = "";

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

TMP126 tmp126(TMP126_CS);

void setup_current_sensor(){
  if (!ina237.begin()) {
    Serial.println("Couldn't find INA237 chip");
    while (1)
      ;
  }
  Serial.println("Found INA237 chip");
  // set shunt resistance and max current
  ina237.setShunt(0.0005, 2.0);

  ina237.setAveragingCount(INA2XX_COUNT_16);
  uint16_t counts[] = {1, 4, 16, 64, 128, 256, 512, 1024};
  Serial.print("Averaging counts: ");
  Serial.println(counts[ina237.getAveragingCount()]);

  // set the time over which to measure the current and bus voltage
  ina237.setVoltageConversionTime(INA2XX_TIME_150_us);
  Serial.print("Voltage conversion time: ");
  switch (ina237.getVoltageConversionTime()) {
  case INA2XX_TIME_50_us:
    Serial.print("50");
    break;
  case INA2XX_TIME_84_us:
    Serial.print("84");
    break;
  case INA2XX_TIME_150_us:
    Serial.print("150");
    break;
  case INA2XX_TIME_280_us:
    Serial.print("280");
    break;
  case INA2XX_TIME_540_us:
    Serial.print("540");
    break;
  case INA2XX_TIME_1052_us:
    Serial.print("1052");
    break;
  case INA2XX_TIME_2074_us:
    Serial.print("2074");
    break;
  case INA2XX_TIME_4120_us:
    Serial.print("4120");
    break;
  }
  Serial.println(" uS");

  ina237.setCurrentConversionTime(INA2XX_TIME_280_us);
  Serial.print("Current conversion time: ");
  switch (ina237.getCurrentConversionTime()) {
  case INA2XX_TIME_50_us:
    Serial.print("50");
    break;
  case INA2XX_TIME_84_us:
    Serial.print("84");
    break;
  case INA2XX_TIME_150_us:
    Serial.print("150");
    break;
  case INA2XX_TIME_280_us:
    Serial.print("280");
    break;
  case INA2XX_TIME_540_us:
    Serial.print("540");
    break;
  case INA2XX_TIME_1052_us:
    Serial.print("1052");
    break;
  case INA2XX_TIME_2074_us:
    Serial.print("2074");
    break;
  case INA2XX_TIME_4120_us:
    Serial.print("4120");
    break;
  }
  Serial.println(" uS");

  // default polarity for the alert is low on ready, but
  // it can be inverted!
  //ina237.setAlertPolarity(INA2XX_ALERT_POLARITY_INVERTED);
}


void get_current_sensor_data(){
    // by default the sensor does continuous reading, but
  // we can set to triggered mode. to do that, we have to set
  // the mode to trigger a new reading, then wait for a conversion
  // either by checking the ALERT pin or reading the ready register
  // ina237.setMode(INA2XX_MODE_TRIGGERED);
  // while (!ina237.conversionReady())
  //  delay(1);

  Serial.print("Current: ");
  Serial.print(ina237.getCurrent_mA());
  Serial.println(" mA");

  Serial.print("Bus Voltage: ");
  Serial.print(ina237.getBusVoltage_V());
  Serial.println(" V");

  Serial.print("Shunt Voltage: ");
  Serial.print(ina237.getShuntVoltage_mV() * 1000.0); // Convert from mV to μV
  Serial.println(" uV");

  Serial.print("Power: ");
  Serial.print(ina237.getPower_mW());
  Serial.println(" mW");

  Serial.print("Temperature: ");
  Serial.print(ina237.readDieTemp());
  Serial.println(" *C");


  
}

void test_buzzer() {
  Serial.println("Testing buzzer...");
  ledcWrite(0, 128); // 50% duty cycle for audible tone
  delay(500);        // Buzz for 0.5 seconds
  ledcWrite(0, 0);   // Turn off the buzzer
  Serial.println("Buzzer test complete.");
}

void test_rg_led() {
  Serial.println("Testing RG LED...");
  for (int i = 0; i <= 255; i += 5) {
    ledcWrite(LED_GREEN_PWM_CHANNEL, i);
    delay(10);
  }
  for (int i = 255; i >= 0; i -= 5) {
    ledcWrite(LED_GREEN_PWM_CHANNEL, i);
    delay(10);
  }

  for (int i = 0; i <= 255; i += 5) {
    ledcWrite(LED_RED_PWM_CHANNEL, i);
    delay(10);
  }
  for (int i = 255; i >= 0; i -= 5) {
    ledcWrite(LED_RED_PWM_CHANNEL, i);
    delay(10);
  }

  ledcWrite(LED_GREEN_PWM_CHANNEL, 0);
  ledcWrite(LED_RED_PWM_CHANNEL, 0);
  Serial.println("RG LED test complete.");
}

void test_bluetooth() {
  Serial.println("Testing BLE (notify)...");
  pCharacteristic->setValue("Hello from ESP32 via BLE!");
  pCharacteristic->notify();
  Serial.println("BLE notification sent.");
}

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
  BLEDevice::init("ESP32-Yaxsomo");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(BLE_SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      BLE_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->start();

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
}

void loop() {
  if (deviceConnected) {
    std::string value = pCharacteristic->getValue();
    if (!value.empty() && value != lastValue) {
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

  delay(100); // Optional: short idle delay when not connected
}