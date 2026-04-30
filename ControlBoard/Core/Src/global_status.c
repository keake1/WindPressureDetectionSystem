#include "global_status.h"

#include "dwin.h"
#include <string.h>

/* Modbus 线圈后备区：前 64 位为在线标志，后 64 位为报警标志。 */
uint8_t g_modbusCoils[GLOBAL_STATUS_COIL_COUNT];
/* Modbus 输入寄存器后备区：保存 64 个传感器的数据、型号和诊断字段。 */
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

void GlobalStatus_RefreshDiagnostics(void)
{
  uint16_t *diag = &g_modbusInputRegs[GLOBAL_STATUS_DIAG_REG_BASE];
  uint32_t errCount = g_controllerInfo.errorResponseCount;
  uint32_t dropCount = DwinSend_GetDropCount();

  /* 把不便用 16 位寄存器表达的字段拆成低/高字，便于上位机一次读完整段。 */
  diag[GLOBAL_STATUS_DIAG_ERR_COUNT_LO] = (uint16_t)(errCount & 0xFFFFU);
  diag[GLOBAL_STATUS_DIAG_ERR_COUNT_HI] = (uint16_t)(errCount >> 16);
  diag[GLOBAL_STATUS_DIAG_ZERO_ADDR]    = (uint16_t)g_controllerInfo.hasZeroAddressSensor;
  diag[GLOBAL_STATUS_DIAG_DUP_ADDR]     = (uint16_t)g_controllerInfo.hasDuplicateAddressSensor;
  diag[GLOBAL_STATUS_DIAG_STARTUP_DONE] = (uint16_t)g_controllerInfo.startupScanDone;
  diag[GLOBAL_STATUS_DIAG_CTRL_ADDR]    = (uint16_t)g_controllerInfo.controllerAddress;
  diag[GLOBAL_STATUS_DIAG_DWIN_DROP_LO] = (uint16_t)(dropCount & 0xFFFFU);
  diag[GLOBAL_STATUS_DIAG_DWIN_DROP_HI] = (uint16_t)(dropCount >> 16);
}
