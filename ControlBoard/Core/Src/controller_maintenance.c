#include "controller_maintenance.h"

#include "FreeRTOS.h"
#include "dwin.h"
#include "global_status.h"
#include "gpio.h"
#include "modbus_sensor_host.h"
#include "task.h"
#include <string.h>

#define CONTROLLER_ALARM_UPDATE_INTERVAL_MS 500U
#define ALARM_SET_CO2_THRESHOLD             1000U
#define ALARM_SET_CH2O_THRESHOLD            80U
#define ALARM_SET_TVOC_THRESHOLD            600U
#define ALARM_SET_PM25_THRESHOLD            35U
#define ALARM_SET_PM10_THRESHOLD            50U
#define ALARM_SET_CO_X100_THRESHOLD         2500U
#define ALARM_CLEAR_CO2_THRESHOLD           800U
#define ALARM_CLEAR_CH2O_THRESHOLD          70U
#define ALARM_CLEAR_TVOC_THRESHOLD          500U
#define ALARM_CLEAR_PM25_THRESHOLD          25U
#define ALARM_CLEAR_PM10_THRESHOLD          40U
#define ALARM_CLEAR_CO_X100_THRESHOLD       2000U
#define DWIN_ICON_UPDATE_INTERVAL_MS        100U
#define DWIN_VALUE_UPDATE_INTERVAL_MS       500U

static uint8_t ControllerMaintenance_CalcSensorAlarm(uint8_t sensorIndex)
{
  uint8_t nowAlarm;
  SensorModel_t model;

  if (SENSOR_ONLINE(sensorIndex) == 0U)
  {
    return 0U;
  }

  model = (SensorModel_t)SENSOR_MODEL(sensorIndex);
  if ((model != SENSOR_MODEL_CO) && (model != SENSOR_MODEL_SEVEN_IN_ONE))
  {
    return 0U;
  }

  nowAlarm = SENSOR_ALARM(sensorIndex);
  if (nowAlarm == 0U)
  {
    /* 当前正常：CO 数据按实际 ppm * 100 保存，阈值也使用 x100 单位。 */
    if (model == SENSOR_MODEL_CO)
    {
      if (SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_CO) > ALARM_SET_CO_X100_THRESHOLD)
      {
        nowAlarm = 1U;
      }
    }
    else /* SENSOR_MODEL_SEVEN_IN_ONE */
    {
      if ((SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_CO2) > ALARM_SET_CO2_THRESHOLD) ||
          (SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_CH2O) > ALARM_SET_CH2O_THRESHOLD) ||
          (SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_TVOC) > ALARM_SET_TVOC_THRESHOLD) ||
          (SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_PM25) > ALARM_SET_PM25_THRESHOLD) ||
          (SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_PM10) > ALARM_SET_PM10_THRESHOLD))
      {
        nowAlarm = 1U;
      }
    }
  }
  else
  {
    /* 当前报警：CO 传感器按 x100 单位恢复，七合一传感器需所有参与项均低于恢复下限。 */
    if (model == SENSOR_MODEL_CO)
    {
      if (SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_CO) < ALARM_CLEAR_CO_X100_THRESHOLD)
      {
        nowAlarm = 0U;
      }
    }
    else /* SENSOR_MODEL_SEVEN_IN_ONE */
    {
      if ((SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_CO2) < ALARM_CLEAR_CO2_THRESHOLD) &&
          (SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_CH2O) < ALARM_CLEAR_CH2O_THRESHOLD) &&
          (SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_TVOC) < ALARM_CLEAR_TVOC_THRESHOLD) &&
          (SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_PM25) < ALARM_CLEAR_PM25_THRESHOLD) &&
          (SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_PM10) < ALARM_CLEAR_PM10_THRESHOLD))
      {
        nowAlarm = 0U;
      }
    }
  }

  return nowAlarm;
}

static void ControllerMaintenance_UpdateIcons(ControllerMaintenanceState_t *state)
{
  uint8_t i;

  if (g_controllerInfo.startupScanDone == 0U)
  {
    return;
  }

  if (g_controllerInfo.alarm != state->lastControllerAlarm)
  {
    uint8_t value = (g_controllerInfo.alarm != 0U) ? 1U : 0U;
    DwinIcon_WriteValue(DWIN_GLOBAL_ALARM_ICON_ADDR, value);
    DwinIcon_WriteValue(DWIN_WINDOWS_STATUS_ICON_ADDR, value);
    state->lastControllerAlarm = g_controllerInfo.alarm;
  }

  if (g_controllerInfo.hasDuplicateAddressSensor != state->lastDuplicateAddress)
  {
    DwinIcon_WriteValue(DWIN_DUP_ADDR_ICON_ADDR,
                        (g_controllerInfo.hasDuplicateAddressSensor != 0U) ? 1U : 0U);
    state->lastDuplicateAddress = g_controllerInfo.hasDuplicateAddressSensor;
  }

  if (g_controllerInfo.hasZeroAddressSensor != state->lastZeroAddress)
  {
    DwinIcon_WriteValue(DWIN_ZERO_ADDR_ICON_ADDR,
                        (g_controllerInfo.hasZeroAddressSensor != 0U) ? 1U : 0U);
    state->lastZeroAddress = g_controllerInfo.hasZeroAddressSensor;
  }

  for (i = 0U; i < GLOBAL_STATUS_SENSOR_COUNT; ++i)
  {
    uint8_t online = SENSOR_ONLINE(i);
    uint8_t alarm = SENSOR_ALARM(i);

    if ((online != state->lastIconSensorOnline[i]) ||
        (alarm != state->lastIconSensorAlarm[i]))
    {
      uint16_t addr = (uint16_t)(DWIN_SENSOR_STATUS_ICON_ADDR + i);
      DwinIcon_WriteValue(addr, DwinIcon_GetSensorStatusValue(online, alarm));
      state->lastIconSensorOnline[i] = online;
      state->lastIconSensorAlarm[i] = alarm;
    }
  }
}

static void ControllerMaintenance_UpdateValues(ControllerMaintenanceState_t *state)
{
  uint8_t controllerAddress = GPIO_ReadDeviceAddr();
  uint8_t i;

  if (controllerAddress != state->lastControllerAddress)
  {
    g_controllerInfo.controllerAddress = controllerAddress;
    DwinValue_WriteU16(DWIN_CONTROL_BORAD_ADDR_VAR, controllerAddress);
    state->lastControllerAddress = controllerAddress;
  }

  if (g_controllerInfo.startupScanDone == 0U)
  {
    return;
  }

  for (i = 0U; i < GLOBAL_STATUS_SENSOR_COUNT; ++i)
  {
    uint8_t online = SENSOR_ONLINE(i);
    uint32_t responseTime = g_sensorInfos[i].lastResponseTime;

    /* 仅在线状态下且数据有刷新（lastResponseTime 变化）才推屏，避免重复刷写。 */
    if ((online != 0U) &&
        ((state->lastValueSensorOnline[i] == 0U) ||
         (responseTime != state->lastValueSensorResponseTime[i])))
    {
      DwinValue_UpdateSensor(i);
      state->lastValueSensorResponseTime[i] = responseTime;
    }

    state->lastValueSensorOnline[i] = online;
  }
}

static void ControllerMaintenance_UpdateAlarm(void)
{
  uint8_t i;
  uint8_t controllerAlarm = 0U;

  if (g_controllerInfo.startupScanDone == 0U)
  {
    return;
  }

  for (i = 0U; i < GLOBAL_STATUS_SENSOR_COUNT; ++i)
  {
    uint8_t sensorAlarm = ControllerMaintenance_CalcSensorAlarm(i);
    SENSOR_ALARM(i) = sensorAlarm;
    if (sensorAlarm != 0U)
    {
      controllerAlarm = 1U;
    }
  }

  g_controllerInfo.alarm = controllerAlarm;
}

static void ControllerMaintenance_UpdateStatus(ControllerMaintenanceState_t *state)
{
  if (g_controllerInfo.startupScanDone == 0U)
  {
    return;
  }

  if (state->baselineReady == 0U)
  {
    state->lastErrorResponseCount = g_controllerInfo.errorResponseCount;
    state->baselineReady = 1U;
  }

  g_controllerInfo.hasZeroAddressSensor =
    ModbusSensorHost_ProbeZeroAddress(MODBUS_SENSOR_ZERO_ADDR_RESPONSE_TIMEOUT_MS);

  {
    uint32_t currentErrorResponseCount = g_controllerInfo.errorResponseCount;
    uint32_t errorIncrease = currentErrorResponseCount - state->lastErrorResponseCount;
    g_controllerInfo.hasDuplicateAddressSensor =
      (errorIncrease > GLOBAL_STATUS_DUPLICATE_ADDR_ERROR_THRESHOLD) ? 1U : 0U;
    state->lastErrorResponseCount = currentErrorResponseCount;
  }
}

void ControllerMaintenance_Init(ControllerMaintenanceState_t *state)
{
  if (state == NULL)
  {
    return;
  }

  memset(state, 0, sizeof(*state));
}

void ControllerMaintenance_Process(ControllerMaintenanceState_t *state)
{
  TickType_t now = xTaskGetTickCount();

  if (state == NULL)
  {
    return;
  }

  if ((now - state->lastIconTick) >= pdMS_TO_TICKS(DWIN_ICON_UPDATE_INTERVAL_MS))
  {
    ControllerMaintenance_UpdateIcons(state);
    state->lastIconTick = now;
  }

  if ((now - state->lastValueTick) >= pdMS_TO_TICKS(DWIN_VALUE_UPDATE_INTERVAL_MS))
  {
    ControllerMaintenance_UpdateValues(state);
    state->lastValueTick = now;
  }

  if ((now - state->lastAlarmTick) >= pdMS_TO_TICKS(CONTROLLER_ALARM_UPDATE_INTERVAL_MS))
  {
    ControllerMaintenance_UpdateAlarm();
    state->lastAlarmTick = now;
  }

  if ((now - state->lastStatusTick) >= pdMS_TO_TICKS(GLOBAL_STATUS_UPDATE_INTERVAL_MS))
  {
    ControllerMaintenance_UpdateStatus(state);
    state->lastStatusTick = now;
  }

  /* 每个调度周期顺手把诊断字段刷到输入寄存器；操作非常轻量（8 个 16 位写入）。 */
  GlobalStatus_RefreshDiagnostics();
}
