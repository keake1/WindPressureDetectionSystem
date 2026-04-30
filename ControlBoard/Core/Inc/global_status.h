#ifndef GLOBAL_STATUS_H
#define GLOBAL_STATUS_H

#include <stdint.h>

/* 输入寄存器映射:
 *   0..639   : 64 个传感器的数据区，每个传感器占 10 个寄存器
 *   640..703 : 64 个传感器的型号区，每个传感器占 1 个寄存器
 *
 * 线圈映射:
 *   0..63    : 64 个传感器在线标志
 *   64..127  : 64 个传感器报警标志
 */
#define GLOBAL_STATUS_SENSOR_COUNT      64U
#define GLOBAL_STATUS_SENSOR_DATA_COUNT 10U
#define GLOBAL_STATUS_SENSOR_MODEL_COUNT 4U
#define GLOBAL_STATUS_DATA_WIND_PRESSURE 0U /* 风压原始值。 */
#define GLOBAL_STATUS_DATA_CO           1U  /* CO 浓度，保存值 = 实际 ppm * 100。 */
#define GLOBAL_STATUS_DATA_RESIDUAL_PRESSURE 2U /* 余压原始值。 */
#define GLOBAL_STATUS_DATA_CO2          3U  /* CO2 浓度原始值。 */
#define GLOBAL_STATUS_DATA_CH2O         4U  /* CH2O 浓度原始值。 */
#define GLOBAL_STATUS_DATA_TVOC         5U  /* TVOC 原始值。 */
#define GLOBAL_STATUS_DATA_PM25         6U  /* PM2.5 原始值。 */
#define GLOBAL_STATUS_DATA_PM10         7U  /* PM10 原始值。 */
#define GLOBAL_STATUS_DATA_TEMPERATURE  8U  /* 温度，保存值 = 实际温度 * 10。 */
#define GLOBAL_STATUS_DATA_HUMIDITY     9U  /* 湿度，保存值 = 实际湿度 * 10。 */
#define GLOBAL_STATUS_SENSOR_DATA_REG_BASE 0U
#define GLOBAL_STATUS_SENSOR_MODEL_REG_BASE (GLOBAL_STATUS_SENSOR_DATA_REG_BASE + (GLOBAL_STATUS_SENSOR_COUNT * GLOBAL_STATUS_SENSOR_DATA_COUNT))
#define GLOBAL_STATUS_INPUT_REG_COUNT   (GLOBAL_STATUS_SENSOR_MODEL_REG_BASE + GLOBAL_STATUS_SENSOR_COUNT)
#define GLOBAL_STATUS_SENSOR_ONLINE_COIL_BASE 0U
#define GLOBAL_STATUS_SENSOR_ALARM_COIL_BASE  (GLOBAL_STATUS_SENSOR_ONLINE_COIL_BASE + GLOBAL_STATUS_SENSOR_COUNT)
#define GLOBAL_STATUS_COIL_COUNT        (GLOBAL_STATUS_SENSOR_ALARM_COIL_BASE + GLOBAL_STATUS_SENSOR_COUNT)
#define GLOBAL_STATUS_CONTROLLER_ADDR_MIN 1U
#define GLOBAL_STATUS_CONTROLLER_ADDR_MAX 128U
#define GLOBAL_STATUS_DUPLICATE_ADDR_ERROR_THRESHOLD 2U
#define GLOBAL_STATUS_UPDATE_INTERVAL_MS 5000U

typedef enum
{
  SENSOR_MODEL_NONE = 0,
  SENSOR_MODEL_WIND_PRESSURE,
  SENSOR_MODEL_CO,
  SENSOR_MODEL_RESIDUAL_PRESSURE,
  SENSOR_MODEL_SEVEN_IN_ONE
} SensorModel_t;

typedef struct
{
  uint16_t *model; /* 指向型号输入寄存器，保存 SENSOR_MODEL_* 枚举值。 */
  uint8_t *online; /* 指向在线状态线圈。 */
  uint8_t *alarm;  /* 指向报警状态线圈。 */
  uint32_t lastResponseTime;
  uint16_t *data;  /* 指向该传感器的 10 个数据输入寄存器。 */
} SensorInfo_t;

typedef struct
{
  uint32_t errorResponseCount;
  uint8_t alarm;
  uint8_t hasZeroAddressSensor;
  uint8_t hasDuplicateAddressSensor;
  uint8_t controllerAddress;
  uint8_t startupScanDone;
} ControllerInfo_t;

/* Modbus 线圈后备区：前 64 位为在线标志，后 64 位为报警标志。 */
extern uint8_t g_modbusCoils[GLOBAL_STATUS_COIL_COUNT];
/* Modbus 输入寄存器后备区：保存 64 个传感器的数据和型号。 */
extern uint16_t g_modbusInputRegs[GLOBAL_STATUS_INPUT_REG_COUNT];
/* 传感器信息数组：每一项通过指针映射到 Modbus 后备区。 */
extern SensorInfo_t g_sensorInfos[GLOBAL_STATUS_SENSOR_COUNT];
/* 控制器状态：保存通信错误、总报警和地址异常等全局状态。 */
extern ControllerInfo_t g_controllerInfo;

void GlobalStatus_Init(void);

#endif /* GLOBAL_STATUS_H */
