#include "modbus_sensor_host.h"

#include "FreeRTOS.h"
#include "modbus_rtu.h"
#include "semphr.h"
#include "task.h"

#define MB_FC_READ_HOLDING_REGS 0x03U
#define MB_READ_REQ_SIZE        8U
#define MB_READ_START_REG       0x0000U
#define MB_READ_REG_COUNT       0x0001U

#define SENSOR_PAYLOAD_SHORT_SIZE        3U
#define SENSOR_PAYLOAD_SEVEN_IN_ONE_SIZE 15U
#define SENSOR_RESPONSE_MIN_SIZE         5U

/* 下行传感器总线使用的 UART，当前为 USART1。 */
static UART_HandleTypeDef *g_sensorUart;
/* 保护 USART1 上完整请求-响应事务的互斥锁。 */
static SemaphoreHandle_t g_requestMutex;
/* 每个传感器最近一次轮询结果，用于调试和后续状态判断。 */
static ModbusSensor_Status_t g_sensorStatus[MODBUS_SENSOR_COUNT];

static uint16_t Mb_Crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  uint16_t pos;
  uint8_t bit;

  for (pos = 0U; pos < length; ++pos)
  {
    crc ^= (uint16_t)data[pos];
    for (bit = 0U; bit < 8U; ++bit)
    {
      uint16_t carry = crc & 0x0001U;
      crc >>= 1U;
      if (carry != 0U)
      {
        crc ^= 0xA001U;
      }
    }
  }

  return crc;
}

static void Mb_WriteU16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value >> 8);
  data[1] = (uint8_t)value;
}

static uint16_t Mb_ReadU16(const uint8_t *data)
{
  return ((uint16_t)data[0] << 8) | (uint16_t)data[1];
}

static uint16_t Mb_BuildFixedReadReq(uint8_t slaveAddr, uint8_t *request, uint16_t requestMax)
{
  uint16_t crc;

  if ((request == NULL) || (requestMax < MB_READ_REQ_SIZE))
  {
    return 0U;
  }

  request[0] = slaveAddr;
  request[1] = MB_FC_READ_HOLDING_REGS;
  Mb_WriteU16(&request[2], MB_READ_START_REG);
  Mb_WriteU16(&request[4], MB_READ_REG_COUNT);
  crc = Mb_Crc16(request, 6U);
  request[6] = (uint8_t)crc;
  request[7] = (uint8_t)(crc >> 8);
  return MB_READ_REQ_SIZE;
}

static void SensorHost_ClearData(uint8_t sensorIndex)
{
  uint8_t i;

  for (i = 0U; i < GLOBAL_STATUS_SENSOR_DATA_COUNT; ++i)
  {
    g_sensorInfos[sensorIndex].data[i] = 0U;
  }
}

static uint8_t SensorHost_ModelAndLengthValid(SensorModel_t model, uint8_t payloadLen)
{
  switch (model)
  {
    case SENSOR_MODEL_WIND_PRESSURE:
    case SENSOR_MODEL_CO:
    case SENSOR_MODEL_RESIDUAL_PRESSURE:
      return (payloadLen == SENSOR_PAYLOAD_SHORT_SIZE) ? 1U : 0U;
    case SENSOR_MODEL_SEVEN_IN_ONE:
      return (payloadLen == SENSOR_PAYLOAD_SEVEN_IN_ONE_SIZE) ? 1U : 0U;
    default:
      return 0U;
  }
}

static ModbusSensor_Status_t SensorHost_ParsePayload(uint8_t sensorIndex,
                                                     const uint8_t *payload,
                                                     uint8_t payloadLen)
{
  SensorModel_t model;

  if ((sensorIndex >= MODBUS_SENSOR_COUNT) || (payload == NULL))
  {
    return MODBUS_SENSOR_STATUS_ARG_ERROR;
  }

  model = (SensorModel_t)payload[0];
  if (SensorHost_ModelAndLengthValid(model, payloadLen) == 0U)
  {
    return MODBUS_SENSOR_STATUS_FRAME_ERROR;
  }

  /* 同一个传感器只填自己有关的数据项，其余统一清 0，避免保留旧型号的残留数据。 */
  SensorHost_ClearData(sensorIndex);
  *g_sensorInfos[sensorIndex].model = (uint16_t)model;

  switch (model)
  {
    case SENSOR_MODEL_WIND_PRESSURE:
      g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_WIND_PRESSURE] = Mb_ReadU16(&payload[1]);
      break;
    case SENSOR_MODEL_CO:
      g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_CO] = Mb_ReadU16(&payload[1]);
      break;
    case SENSOR_MODEL_RESIDUAL_PRESSURE:
      g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_RESIDUAL_PRESSURE] = Mb_ReadU16(&payload[1]);
      break;
    case SENSOR_MODEL_SEVEN_IN_ONE:
      g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_CO2] = Mb_ReadU16(&payload[1]);
      g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_CH2O] = Mb_ReadU16(&payload[3]);
      g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_TVOC] = Mb_ReadU16(&payload[5]);
      g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_PM25] = Mb_ReadU16(&payload[7]);
      g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_PM10] = Mb_ReadU16(&payload[9]);
      g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_TEMPERATURE] = Mb_ReadU16(&payload[11]);
      g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_HUMIDITY] = Mb_ReadU16(&payload[13]);
      break;
    default:
      return MODBUS_SENSOR_STATUS_FRAME_ERROR;
  }

  return MODBUS_SENSOR_STATUS_OK;
}

static ModbusSensor_Status_t SensorHost_ParseResponse(uint8_t slaveAddr,
                                                      uint8_t sensorIndex,
                                                      const uint8_t *response,
                                                      uint16_t responseLen)
{
  uint16_t crcRx;
  uint16_t crcCalc;
  uint8_t payloadLen;

  /* 传感器响应仍保留 RTU 外壳：地址、功能码、数据长度、自定义数据、CRC。 */
  if (response == NULL)
  {
    return MODBUS_SENSOR_STATUS_ARG_ERROR;
  }

  if (responseLen < SENSOR_RESPONSE_MIN_SIZE)
  {
    return MODBUS_SENSOR_STATUS_FRAME_ERROR;
  }

  crcRx = (uint16_t)response[responseLen - 2U] | ((uint16_t)response[responseLen - 1U] << 8);
  crcCalc = Mb_Crc16(response, (uint16_t)(responseLen - 2U));
  if (crcRx != crcCalc)
  {
    return MODBUS_SENSOR_STATUS_CRC_ERROR;
  }

  if ((response[0] != slaveAddr) || (response[1] != MB_FC_READ_HOLDING_REGS))
  {
    return MODBUS_SENSOR_STATUS_FRAME_ERROR;
  }

  payloadLen = response[2];
  if ((responseLen != (uint16_t)(payloadLen + 5U)) ||
      ((payloadLen != SENSOR_PAYLOAD_SHORT_SIZE) && (payloadLen != SENSOR_PAYLOAD_SEVEN_IN_ONE_SIZE)))
  {
    return MODBUS_SENSOR_STATUS_FRAME_ERROR;
  }

  return SensorHost_ParsePayload(sensorIndex, &response[3], payloadLen);
}

static ModbusSensor_Status_t SensorHost_ReadOne(uint8_t slaveAddr, uint8_t sensorIndex, uint32_t responseTimeoutMs)
{
  uint8_t request[MB_READ_REQ_SIZE];
  uint8_t response[MODBUS_RTU_MAX_FRAME_SIZE];
  uint16_t requestLen;
  uint16_t responseLen;
  TickType_t startTick;
  ModbusSensor_Status_t status = MODBUS_SENSOR_STATUS_TIMEOUT;

  if ((g_sensorUart == NULL) ||
      (g_requestMutex == NULL) ||
      (slaveAddr == 0U) ||
      (sensorIndex >= MODBUS_SENSOR_COUNT) ||
      (responseTimeoutMs == 0U))
  {
    return MODBUS_SENSOR_STATUS_ARG_ERROR;
  }

  /* 同一条 USART1 总线一次只允许一个请求-响应事务，避免多个任务抢占串口导致帧交错。 */
  (void)xSemaphoreTake(g_requestMutex, portMAX_DELAY);

  requestLen = Mb_BuildFixedReadReq(slaveAddr, request, sizeof(request));
  if (requestLen == 0U)
  {
    (void)xSemaphoreGive(g_requestMutex);
    return MODBUS_SENSOR_STATUS_ARG_ERROR;
  }

  ModbusRtu_ClearPort(g_sensorUart);
  if (ModbusRtu_SendTo(g_sensorUart, request, requestLen, responseTimeoutMs) != HAL_OK)
  {
    (void)xSemaphoreGive(g_requestMutex);
    return MODBUS_SENSOR_STATUS_TIMEOUT;
  }

  startTick = xTaskGetTickCount();
  /* 发送固定 0x03 请求后等待自定义响应帧。 */
  while ((xTaskGetTickCount() - startTick) < pdMS_TO_TICKS(responseTimeoutMs))
  {
    if (ModbusRtu_PollFrameFrom(g_sensorUart, response, sizeof(response), &responseLen) != 0U)
    {
      status = SensorHost_ParseResponse(slaveAddr, sensorIndex, response, responseLen);
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(1U));
  }

  (void)xSemaphoreGive(g_requestMutex);
  return status;
}

uint8_t ModbusSensorHost_Init(UART_HandleTypeDef *uart)
{
  uint8_t i;

  if (uart == NULL)
  {
    return 0U;
  }

  g_sensorUart = uart;
  if (g_requestMutex == NULL)
  {
    g_requestMutex = xSemaphoreCreateMutex();
  }

  if (g_requestMutex == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < MODBUS_SENSOR_COUNT; ++i)
  {
    g_sensorStatus[i] = MODBUS_SENSOR_STATUS_TIMEOUT;
  }

  return ModbusRtu_InitPort(g_sensorUart);
}

static void SensorHost_UpdateStatus(uint8_t sensorIndex, ModbusSensor_Status_t status)
{
  g_sensorStatus[sensorIndex] = status;

  if (status == MODBUS_SENSOR_STATUS_OK)
  {
    *g_sensorInfos[sensorIndex].online = 1U;
    g_sensorInfos[sensorIndex].lastResponseTime = HAL_GetTick();
  }
  else if (status == MODBUS_SENSOR_STATUS_TIMEOUT)
  {
    /* 超时可能是传感器离线或通信故障，在线标志清 0，等待下一轮扫描更新。 */
    *g_sensorInfos[sensorIndex].online = 0U;
  }
  else
  {
    /* 其他错误可能是通信干扰或协议不匹配，在线标志保持不变，等待下一轮扫描验证。 */
    g_controllerInfo.errorResponseCount++;
  }
}

void ModbusSensorHost_PollAll(uint16_t scanRounds, uint32_t requestIntervalMs, uint32_t responseTimeoutMs)
{
  uint16_t round;
  uint8_t i;

  if ((scanRounds == 0U) || (responseTimeoutMs == 0U))
  {
    return;
  }

  for (round = 0U; round < scanRounds; ++round)
  {
    for (i = 0U; i < MODBUS_SENSOR_COUNT; ++i)
    {
      uint8_t slaveAddr = (uint8_t)(MODBUS_SENSOR_FIRST_ADDR + i);
      ModbusSensor_Status_t status = SensorHost_ReadOne(slaveAddr, i, responseTimeoutMs);
      SensorHost_UpdateStatus(i, status);

      if ((requestIntervalMs > 0U) &&
          ((round < (uint16_t)(scanRounds - 1U)) || (i < (MODBUS_SENSOR_COUNT - 1U))))
      {
        /* 当前传感器收到回复或等待超时后，先停顿一段时间再请求下一个传感器。 */
        vTaskDelay(pdMS_TO_TICKS(requestIntervalMs));
      }
    }
  }
}

uint8_t ModbusSensorHost_PollOnline(uint32_t requestIntervalMs, uint32_t responseTimeoutMs)
{
  uint8_t i;
  uint8_t polledCount = 0U;

  if (responseTimeoutMs == 0U)
  {
    return 0U;
  }

  for (i = 0U; i < MODBUS_SENSOR_COUNT; ++i)
  {
    if (*g_sensorInfos[i].online == 0U)
    {
      continue;
    }

    {
      uint8_t slaveAddr = (uint8_t)(MODBUS_SENSOR_FIRST_ADDR + i);
      ModbusSensor_Status_t status = SensorHost_ReadOne(slaveAddr, i, responseTimeoutMs);
      SensorHost_UpdateStatus(i, status);
      polledCount++;
    }

    if (requestIntervalMs > 0U)
    {
      /* 在线传感器数量少时也保留间隔，避免一轮结束后立刻重新请求同一传感器。 */
      vTaskDelay(pdMS_TO_TICKS(requestIntervalMs));
    }
  }

  return polledCount;
}

uint8_t ModbusSensorHost_ProbeZeroAddress(uint32_t responseTimeoutMs)
{
  uint8_t request[MB_READ_REQ_SIZE];
  uint8_t response[MODBUS_RTU_MAX_FRAME_SIZE];
  uint16_t requestLen;
  uint16_t responseLen;
  TickType_t startTick;
  uint8_t hasResponse = 0U;

  if ((g_sensorUart == NULL) || (g_requestMutex == NULL) || (responseTimeoutMs == 0U))
  {
    return 0U;
  }

  /* 地址 0 正常只作为广播地址使用，不应有设备回复；收到任何回复都视为疑似 0 地址设备。 */
  (void)xSemaphoreTake(g_requestMutex, portMAX_DELAY);

  requestLen = Mb_BuildFixedReadReq(0U, request, sizeof(request));
  if (requestLen == 0U)
  {
    (void)xSemaphoreGive(g_requestMutex);
    return 0U;
  }

  ModbusRtu_ClearPort(g_sensorUart);
  if (ModbusRtu_SendTo(g_sensorUart, request, requestLen, responseTimeoutMs) != HAL_OK)
  {
    (void)xSemaphoreGive(g_requestMutex);
    return 0U;
  }

  startTick = xTaskGetTickCount();
  while ((xTaskGetTickCount() - startTick) < pdMS_TO_TICKS(responseTimeoutMs))
  {
    if (ModbusRtu_PollFrameFrom(g_sensorUart, response, sizeof(response), &responseLen) != 0U)
    {
      hasResponse = (responseLen > 0U) ? 1U : 0U;
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(1U));
  }

  (void)xSemaphoreGive(g_requestMutex);
  return hasResponse;
}
