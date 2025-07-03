#ifndef EVENT_LOGGER_H
#define EVENT_LOGGER_H

#include <Arduino.h>
#include "../alarm_manager/alarm_manager.h"

#define MAX_EVENTS 50 // Limit for stored events

typedef struct {
    AlarmType type;
    uint32_t timestamp;
} LoggedEvent;

void initEventLogger();
void logEvent(AlarmType type);
void printAllEvents();
void clearLoggedEvents();

#endif