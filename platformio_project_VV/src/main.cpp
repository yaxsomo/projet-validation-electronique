// Native Libraries
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA237.h>
#include <SPI.h>
#include <ArduinoBLE.h>

// Custom Libraries
#include "tmp126.h"
#include "configuration.h"


// Peripheral Objects
Adafruit_INA237 ina237 = Adafruit_INA237();

// BLE Objects
BLEService customService("180C"); // Custom Service
BLECharacteristic customChar("2A56", BLERead | BLENotify, 20); // Custom Characteristic

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

float read_ntc_temperature(NTC_devices_t device) {
  const float R1 = 10000.0; // 10k pull-up
  const float T0 = 298.15;  // 25°C in Kelvin
  const float B = 3435.0;   // Beta value
  const float R0 = 10000.0; // 10k reference resistance
  int adcValue = 0; // Initialize adcValue to avoid compilation error

  switch (device)
  {
  case  NTC1:
    adcValue = analogRead(NTC1_PIN);
    break;
  case NTC2:
    adcValue = analogRead(NTC2_PIN);
    break;
  default:
    break;
  }
  
  float Vout = adcValue * 3.3 / 4095.0;
  float Rntc = R1 * Vout / (3.3 - Vout);
  float tempK = 1.0 / (1.0 / T0 + log(Rntc / R0) / B);
  return tempK - 273.15;
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
  customChar.writeValue("Hello from ESP32 via BLE!", true);
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
  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1);
  }
  BLE.setLocalName("ESP32-Yaxsomo");
  BLE.setAdvertisedServiceUuid("180C");
  BLE.setAdvertisedService(customService); // Advertise the custom service
  customService.addCharacteristic(customChar); // Add characteristic to the service
  BLE.addService(customService); // Add service to BLE stack
  BLE.advertise();
  Serial.println("BLE initialized and advertising as ESP32-BLE-Test");

  separator();

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

  separator();

  // Initialize I2C
  Serial.printf("Initializing I2C...");
  Wire.begin();
  Wire.setPins(21, 22);
  Serial.printf("Done.\n");

  separator();

  // Initialize Analog Pins
  Serial.printf("Initializing Analog Pins...");
  analogReadResolution(12); // 12-bit resolution
  pinMode(NTC1_PIN, INPUT);
  pinMode(NTC2_PIN, INPUT);
  Serial.printf("Done.\n");

  separator();

  // Initialize TMP126
  Serial.printf("Initializing TMP126...");
  tmp126.begin();
  Serial.printf("Done.\n");

  separator();

  // Scan I2C devices
  scan_i2c_port();

  separator();

  //Setup current sensor
  Serial.printf("Setting up current sensor...");
  setup_current_sensor();
  Serial.printf("Done.\n");

  separator();

  // End of setup
  Serial.println("Setup done!");
}

void loop() {
  
  test_rg_led();

  test_buzzer();

  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      if (customChar.subscribed()) {
        Serial.println("Central subscribed. Sending BLE notification...");
        customChar.writeValue("Hello from ESP32 via BLE!", true);
      } else {
        Serial.println("Central connected but not subscribed.");
      }
      delay(1000); // Wait between notifications
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }

  //Get current sensor data
  get_current_sensor_data();

  float tmpTemp = tmp126.readTemperature();
  Serial.print("TMP126 Temperature: ");
  Serial.print(tmpTemp);
  Serial.println(" °C");


  Serial.print("NTC1 Temperature: ");
  Serial.print(read_ntc_temperature(NTC1));
  Serial.println(" °C");

  Serial.print("NTC2 Temperature: ");
  Serial.print(read_ntc_temperature(NTC2));
  Serial.println(" °C");

  separator();

  // Delay between loops
  delay(1000);
}