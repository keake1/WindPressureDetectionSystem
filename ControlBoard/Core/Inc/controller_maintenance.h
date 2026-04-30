#ifndef CONTROLLER_MAINTENANCE_H
#define CONTROLLER_MAINTENANCE_H

#include "FreeRTOS.h"
#include "global_status.h"
#include <stdint.h>

#define CONTROLLER_MAINTENANCE_TASK_INTERVAL_MS 20U

typedef struct
{
  uint8_t lastControllerAlarm;
  uint8_t lastDuplicateAddress;
  uint8_t lastZeroAddress;
  uint8_t lastIconSensorOnline[GLOBAL_STATUS_SENSOR_COUNT];
  uint8_t lastIconSensorAlarm[GLOBAL_STATUS_SENSOR_COUNT];
  uint8_t lastValueSensorOnline[GLOBAL_STATUS_SENSOR_COUNT];
  uint32_t lastValueSensorResponseTime[GLOBAL_STATUS_SENSOR_COUNT];
  uint8_t lastControllerAddress;
  uint32_t lastErrorResponseCount;
  uint8_t baselineReady;
  TickType_t lastIconTick;
  TickType_t lastValueTick;
  TickType_t lastAlarmTick;
  TickType_t lastStatusTick;
} ControllerMaintenanceState_t;

void ControllerMaintenance_Init(ControllerMaintenanceState_t *state);
void ControllerMaintenance_Process(ControllerMaintenanceState_t *state);

#endif /* CONTROLLER_MAINTENANCE_H */
