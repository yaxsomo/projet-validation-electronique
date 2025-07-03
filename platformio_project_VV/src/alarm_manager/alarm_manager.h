#ifndef ALARM_MANAGER_H
#define ALARM_MANAGER_H

#include <Arduino.h>

enum AlarmType {
  ALARM_NONE = 0,
  ALARM_TEMPERATURE_RISE,
  ALARM_TEMPERATURE_SAFE,
  ALARM_CURRENT_SPIKE,
  ALARM_SENSOR_MISSING,
  ALARM_COUNT // Total number of alarms
};

void raiseAlarm(AlarmType type);
void clearAlarm(AlarmType type);
bool isAlarmActive(AlarmType type);
void printActiveAlarms();
const char* getAlarmDescription(AlarmType type);

#endif // ALARM_MANAGER_H
