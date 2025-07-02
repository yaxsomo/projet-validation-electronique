#include "Arduino.h"
#include "configuration.h"

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