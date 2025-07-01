
#ifndef CONFIGURATION_H
#define CONFIGURATION_H


// BLE Configuration
#define BLE_CHARACTERISTIC_UUID "abcd1234-5678-1234-5678-123456789abc" // Custom Characteristic UUID
#define BLE_SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"

// TMP126 SPI Pin Definitions
#define TMP126_CS   5
#define TMP126_SCK  18
#define TMP126_MISO 19
#define TMP126_MOSI 23
#define NTC1_PIN 26
#define NTC2_PIN 25


// Pin Definitions
#define BUZZER_PIN 13 // GPIO13

#define LED_GREEN_PIN 14
#define LED_RED_PIN 15
#define LED_GREEN_PWM_CHANNEL 1
#define LED_RED_PWM_CHANNEL 2


typedef enum {
  NTC1 = 1,
  NTC2 = 2
} NTC_devices_t;

#endif // CONFIGURATION_H