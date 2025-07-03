#include "alarm_manager.h"

#define MAX_ALARMS 10
static AlarmType activeAlarms[MAX_ALARMS];
static int alarmCount = 0;

void initAlarms() {
  alarmCount = 0;
  for (int i = 0; i < MAX_ALARMS; i++) {
    activeAlarms[i] = ALARM_NONE;
  }
}

void raiseAlarm(AlarmType type) {
  if (!isAlarmActive(type) && alarmCount < MAX_ALARMS) {
    activeAlarms[alarmCount++] = type;
  }
}

bool isAlarmActive(AlarmType type) {
  for (int i = 0; i < alarmCount; i++) {
    if (activeAlarms[i] == type) return true;
  }
  return false;
}

void clearAlarms() {
  alarmCount = 0;
}

const char* getAlarmDescription(AlarmType type) {
  switch (type) {
    case ALARM_TEMPERATURE_RISE: return "Temperature rising too fast";
    case ALARM_TEMPERATURE_SAFE: return "Temperature back to safe";
    case ALARM_CURRENT_SPIKE: return "Current spike detected";
    case ALARM_SENSOR_MISSING: return "Sensor check failed or not found";
    default: return "Unknown";
  }
}

void printAllAlarms() {
  Serial.println("== ALARMS ==");
  for (int i = 0; i < alarmCount; i++) {
    Serial.println(getAlarmDescription(activeAlarms[i]));
  }
}
