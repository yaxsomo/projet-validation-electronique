#include "buzzer.h"
#include <Arduino.h>

void test_buzzer() {
  Serial.println("Testing buzzer...");
  ledcWrite(0, 128); // 50% duty cycle for audible tone
  delay(500);        // Buzz for 0.5 seconds
  ledcWrite(0, 0);   // Turn off the buzzer
  Serial.println("Buzzer test complete.");
}