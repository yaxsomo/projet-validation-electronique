#include "event_logger.h"

static LoggedEvent eventBuffer[MAX_EVENTS];
static uint8_t eventCount = 0;

void initEventLogger() {
    // Load stored events from flash here (TODO: replace with real flash logic)
    Serial.println("Event logger initialized.");
    eventCount = 0; // Simulate cold start
}

void logEvent(AlarmType type) {
    if (eventCount < MAX_EVENTS) {
        eventBuffer[eventCount++] = {
            .type = type,
            .timestamp = millis() // Simple uptime-based timestamp
        };
        Serial.printf("Event logged: %s at %lu ms\n", getAlarmDescription(type), millis());
    } else {
        Serial.println("Event log is full!");
    }
}

void printAllEvents() {
    if (eventCount == 0) {
        Serial.println("No events logged.");
        return;
    }

    Serial.println("Logged Events:");
    for (uint8_t i = 0; i < eventCount; i++) {
        Serial.printf("  [%lu ms] %s\n", eventBuffer[i].timestamp, getAlarmDescription(eventBuffer[i].type));
    }
}

void clearLoggedEvents() {
    eventCount = 0;
    Serial.println("Event log cleared.");
}