#include "global_status.h"

#include <string.h>

/* Modbus 线圈后备区：前 64 位为在线标志，后 64 位为报警标志。 */
uint8_t g_modbusCoils[GLOBAL_STATUS_COIL_COUNT];
/* Modbus 输入寄存器后备区：保存 64 个传感器的数据和型号。 */
uint16_t g_modbusInputRegs[GLOBAL_STATUS_INPUT_REG_COUNT];
/* 传感器信息数组：每一项通过指针映射到 Modbus 后备区。 */
SensorInfo_t g_sensorInfos[GLOBAL_STATUS_SENSOR_COUNT];
/* 控制器状态：保存通信错误、总报警和地址异常等全局状态。 */
ControllerInfo_t g_controllerInfo;

void GlobalStatus_Init(void)
{
  uint8_t i;

  /* 统一清空对外可读的 Modbus 状态区，避免上电后暴露旧数据。 */
  memset(g_modbusCoils, 0, sizeof(g_modbusCoils));
  memset(g_modbusInputRegs, 0, sizeof(g_modbusInputRegs));
  memset(g_sensorInfos, 0, sizeof(g_sensorInfos));
  memset(&g_controllerInfo, 0, sizeof(g_controllerInfo));

  g_controllerInfo.controllerAddress = GLOBAL_STATUS_CONTROLLER_ADDR_MIN;

  /* SensorInfo_t 只保存指针，实际数据直接落在 Modbus 线圈/输入寄存器数组中。 */
  for (i = 0U; i < GLOBAL_STATUS_SENSOR_COUNT; ++i)
  {
    g_sensorInfos[i].model = &g_modbusInputRegs[GLOBAL_STATUS_SENSOR_MODEL_REG_BASE + i];
    g_sensorInfos[i].online = &g_modbusCoils[GLOBAL_STATUS_SENSOR_ONLINE_COIL_BASE + i];
    g_sensorInfos[i].alarm = &g_modbusCoils[GLOBAL_STATUS_SENSOR_ALARM_COIL_BASE + i];
    g_sensorInfos[i].data = &g_modbusInputRegs[GLOBAL_STATUS_SENSOR_DATA_REG_BASE + ((uint16_t)i * GLOBAL_STATUS_SENSOR_DATA_COUNT)];
  }
}
