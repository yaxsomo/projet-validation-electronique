
#ifndef CONFIGURATION_H
#define CONFIGURATION_H

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


// Command type enum for BLE commands
enum CommandType {
  CMD_NONE,
  CMD_LED,
  CMD_SPI,
  CMD_BUZZER,
  CMD_I2C,
  CMD_CTN,
  CMD_INA
};

#endif // CONFIGURATION_H