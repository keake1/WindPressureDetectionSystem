#include "modbus_host.h"

#include "FreeRTOS.h"
#include "modbus_rtu.h"
#include "semphr.h"
#include "task.h"

#include <stddef.h>
#include <string.h>

#define MB_FC_READ_COILS              0x01U
#define MB_FC_READ_DISCRETE_INPUTS    0x02U
#define MB_FC_READ_HOLDING_REGS       0x03U
#define MB_FC_READ_INPUT_REGS         0x04U
#define MB_FC_WRITE_SINGLE_COIL       0x05U
#define MB_FC_WRITE_SINGLE_REG        0x06U
#define MB_FC_WRITE_MULTIPLE_COILS    0x0FU
#define MB_FC_WRITE_MULTIPLE_REGS     0x10U
#define MB_BASIC_REQ_SIZE             8U

static SemaphoreHandle_t g_requestMutex;

uint8_t ModbusHost_Init(void)
{
  if (g_requestMutex == NULL)
  {
    g_requestMutex = xSemaphoreCreateMutex();
  }

  return (g_requestMutex != NULL) ? 1U : 0U;
}

static uint16_t Mb_ReadU16(const uint8_t *data)
{
  return ((uint16_t)data[0] << 8) | (uint16_t)data[1];
}

static void Mb_WriteU16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value >> 8);
  data[1] = (uint8_t)value;
}

static void Mb_ClearResponseInfo(ModbusHost_ResponseInfo_t *info)
{
  if (info == NULL)
  {
    return;
  }

  info->slaveAddr = 0U;
  info->functionCode = 0U;
  info->payload = NULL;
  info->payloadLen = 0U;
}

uint16_t ModbusHost_Crc16(const uint8_t *data, uint16_t length)
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

static uint16_t Mb_AddCrc(uint8_t *frame, uint16_t payloadLen)
{
  uint16_t crc = ModbusHost_Crc16(frame, payloadLen);
  frame[payloadLen] = (uint8_t)crc;
  frame[payloadLen + 1U] = (uint8_t)(crc >> 8);
  return (uint16_t)(payloadLen + 2U);
}

static uint16_t Mb_BuildBasicReq(uint8_t slaveAddr,
                                 uint8_t function,
                                 uint16_t start,
                                 uint16_t value,
                                 uint8_t *request,
                                 uint16_t requestMax)
{
  if ((request == NULL) || (requestMax < MB_BASIC_REQ_SIZE))
  {
    return 0U;
  }

  request[0] = slaveAddr;
  request[1] = function;
  Mb_WriteU16(&request[2], start);
  Mb_WriteU16(&request[4], value);
  return Mb_AddCrc(request, 6U);
}

static uint8_t Mb_IsReadQuantityValid(uint8_t function, uint16_t quantity)
{
  if ((function == MB_FC_READ_COILS) || (function == MB_FC_READ_DISCRETE_INPUTS))
  {
    return ((quantity > 0U) && (quantity <= 2000U)) ? 1U : 0U;
  }

  return ((quantity > 0U) && (quantity <= 125U)) ? 1U : 0U;
}

static uint16_t Mb_BuildReadReq(uint8_t slaveAddr,
                                uint8_t function,
                                uint16_t start,
                                uint16_t quantity,
                                uint8_t *request,
                                uint16_t requestMax)
{
  if (Mb_IsReadQuantityValid(function, quantity) == 0U)
  {
    return 0U;
  }

  return Mb_BuildBasicReq(slaveAddr, function, start, quantity, request, requestMax);
}

static uint8_t Mb_IsWriteFunction(uint8_t function)
{
  return ((function == MB_FC_WRITE_SINGLE_COIL) ||
          (function == MB_FC_WRITE_SINGLE_REG) ||
          (function == MB_FC_WRITE_MULTIPLE_COILS) ||
          (function == MB_FC_WRITE_MULTIPLE_REGS)) ? 1U : 0U;
}

uint8_t ModbusHost_IsWriteRequest(const uint8_t *request, uint16_t requestLen)
{
  if ((request == NULL) || (requestLen < 2U))
  {
    return 0U;
  }

  return Mb_IsWriteFunction(request[1]);
}

uint16_t ModbusHost_BuildReadCoilsReq(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint8_t *request, uint16_t requestMax)
{
  return Mb_BuildReadReq(slaveAddr, MB_FC_READ_COILS, startAddr, quantity, request, requestMax);
}

uint16_t ModbusHost_BuildReadDiscreteInputsReq(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint8_t *request, uint16_t requestMax)
{
  return Mb_BuildReadReq(slaveAddr, MB_FC_READ_DISCRETE_INPUTS, startAddr, quantity, request, requestMax);
}

uint16_t ModbusHost_BuildReadHoldingReq(uint8_t slaveAddr, uint16_t startReg, uint16_t quantity, uint8_t *request, uint16_t requestMax)
{
  return Mb_BuildReadReq(slaveAddr, MB_FC_READ_HOLDING_REGS, startReg, quantity, request, requestMax);
}

uint16_t ModbusHost_BuildReadInputRegsReq(uint8_t slaveAddr, uint16_t startReg, uint16_t quantity, uint8_t *request, uint16_t requestMax)
{
  return Mb_BuildReadReq(slaveAddr, MB_FC_READ_INPUT_REGS, startReg, quantity, request, requestMax);
}

uint16_t ModbusHost_BuildWriteSingleCoilReq(uint8_t slaveAddr, uint16_t coilAddr, uint8_t coilValue, uint8_t *request, uint16_t requestMax)
{
  uint16_t value = (coilValue != 0U) ? 0xFF00U : 0x0000U;
  return Mb_BuildBasicReq(slaveAddr, MB_FC_WRITE_SINGLE_COIL, coilAddr, value, request, requestMax);
}

uint16_t ModbusHost_BuildWriteSingleRegReq(uint8_t slaveAddr, uint16_t regAddr, uint16_t regValue, uint8_t *request, uint16_t requestMax)
{
  return Mb_BuildBasicReq(slaveAddr, MB_FC_WRITE_SINGLE_REG, regAddr, regValue, request, requestMax);
}

uint16_t ModbusHost_BuildWriteMultipleCoilsReq(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, const uint8_t *values, uint8_t *request, uint16_t requestMax)
{
  uint8_t byteCount;
  uint16_t i;

  if ((values == NULL) || (quantity == 0U) || (quantity > 1968U))
  {
    return 0U;
  }

  byteCount = (uint8_t)((quantity + 7U) / 8U);
  if ((request == NULL) || (requestMax < (uint16_t)(byteCount + 9U)))
  {
    return 0U;
  }

  request[0] = slaveAddr;
  request[1] = MB_FC_WRITE_MULTIPLE_COILS;
  Mb_WriteU16(&request[2], startAddr);
  Mb_WriteU16(&request[4], quantity);
  request[6] = byteCount;
  memset(&request[7], 0, byteCount);

  for (i = 0U; i < quantity; ++i)
  {
    request[7U + (i / 8U)] |= (uint8_t)((values[i] ? 1U : 0U) << (i % 8U));
  }

  return Mb_AddCrc(request, (uint16_t)(byteCount + 7U));
}

uint16_t ModbusHost_BuildWriteMultipleRegsReq(uint8_t slaveAddr, uint16_t startReg, uint16_t quantity, const uint16_t *values, uint8_t *request, uint16_t requestMax)
{
  uint8_t byteCount;
  uint16_t i;

  if ((values == NULL) || (quantity == 0U) || (quantity > 123U))
  {
    return 0U;
  }

  byteCount = (uint8_t)(quantity * 2U);
  if ((request == NULL) || (requestMax < (uint16_t)(byteCount + 9U)))
  {
    return 0U;
  }

  request[0] = slaveAddr;
  request[1] = MB_FC_WRITE_MULTIPLE_REGS;
  Mb_WriteU16(&request[2], startReg);
  Mb_WriteU16(&request[4], quantity);
  request[6] = byteCount;

  for (i = 0U; i < quantity; ++i)
  {
    Mb_WriteU16(&request[7U + (2U * i)], values[i]);
  }

  return Mb_AddCrc(request, (uint16_t)(byteCount + 7U));
}

static ModbusHost_ResponseStatus_t Mb_CheckReadBits(const uint8_t *request,
                                                    const uint8_t *response,
                                                    uint16_t responseLen,
                                                    ModbusHost_ResponseInfo_t *info)
{
  uint16_t quantity = Mb_ReadU16(&request[4]);
  uint8_t expectedBytes = (uint8_t)((quantity + 7U) / 8U);

  if ((response[2] != expectedBytes) || (responseLen != (uint16_t)(expectedBytes + 5U)))
  {
    return MODBUS_HOST_RESP_FRAME_ERROR;
  }

  if (info != NULL)
  {
    info->payload = &response[3];
    info->payloadLen = expectedBytes;
  }

  return MODBUS_HOST_RESP_OK;
}

static ModbusHost_ResponseStatus_t Mb_CheckReadRegisters(const uint8_t *request,
                                                         const uint8_t *response,
                                                         uint16_t responseLen,
                                                         ModbusHost_ResponseInfo_t *info)
{
  uint16_t quantity = Mb_ReadU16(&request[4]);
  uint8_t expectedBytes = (uint8_t)(quantity * 2U);

  if ((response[2] != expectedBytes) || (responseLen != (uint16_t)(expectedBytes + 5U)))
  {
    return MODBUS_HOST_RESP_FRAME_ERROR;
  }

  if (info != NULL)
  {
    info->payload = &response[3];
    info->payloadLen = expectedBytes;
  }

  return MODBUS_HOST_RESP_OK;
}

static ModbusHost_ResponseStatus_t Mb_CheckReadResponse(const uint8_t *request,
                                                        const uint8_t *response,
                                                        uint16_t responseLen,
                                                        ModbusHost_ResponseInfo_t *info)
{
  switch (request[1])
  {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUTS:
      return Mb_CheckReadBits(request, response, responseLen, info);
    case MB_FC_READ_HOLDING_REGS:
    case MB_FC_READ_INPUT_REGS:
      return Mb_CheckReadRegisters(request, response, responseLen, info);
    default:
      return MODBUS_HOST_RESP_ARG_ERROR;
  }
}

static ModbusHost_ResponseStatus_t Mb_CheckWriteResponse(const uint8_t *request,
                                                         const uint8_t *response,
                                                         uint16_t responseLen)
{
  if (responseLen != MB_BASIC_REQ_SIZE)
  {
    return MODBUS_HOST_RESP_FRAME_ERROR;
  }

  if ((response[2] != request[2]) ||
      (response[3] != request[3]) ||
      (response[4] != request[4]) ||
      (response[5] != request[5]))
  {
    return MODBUS_HOST_RESP_FRAME_ERROR;
  }

  return MODBUS_HOST_RESP_OK;
}

static ModbusHost_ResponseStatus_t Mb_CheckResponseHeader(const uint8_t *request,
                                                          uint16_t requestLen,
                                                          const uint8_t *response,
                                                          uint16_t responseLen)
{
  uint16_t crcRx;
  uint16_t crcCalc;

  if ((request == NULL) || (response == NULL) || (requestLen < MB_BASIC_REQ_SIZE))
  {
    return MODBUS_HOST_RESP_ARG_ERROR;
  }

  if (responseLen < 5U)
  {
    return MODBUS_HOST_RESP_FRAME_ERROR;
  }

  crcRx = (uint16_t)response[responseLen - 2U] | ((uint16_t)response[responseLen - 1U] << 8);
  crcCalc = ModbusHost_Crc16(response, (uint16_t)(responseLen - 2U));
  if (crcRx != crcCalc)
  {
    return MODBUS_HOST_RESP_CRC_ERROR;
  }

  if ((response[0] != request[0]) || (response[1] != request[1]))
  {
    return MODBUS_HOST_RESP_FRAME_ERROR;
  }

  return MODBUS_HOST_RESP_OK;
}

ModbusHost_ResponseStatus_t ModbusHost_CheckResponseFrame(const uint8_t *request,
                                                          uint16_t requestLen,
                                                          const uint8_t *response,
                                                          uint16_t responseLen,
                                                          ModbusHost_ResponseInfo_t *info)
{
  ModbusHost_ResponseStatus_t status;

  Mb_ClearResponseInfo(info);
  status = Mb_CheckResponseHeader(request, requestLen, response, responseLen);
  if (status != MODBUS_HOST_RESP_OK)
  {
    return status;
  }

  if (info != NULL)
  {
    info->slaveAddr = response[0];
    info->functionCode = response[1];
  }

  if (ModbusHost_IsWriteRequest(request, requestLen) != 0U)
  {
    return Mb_CheckWriteResponse(request, response, responseLen);
  }

  return Mb_CheckReadResponse(request, response, responseLen, info);
}

ModbusHost_ResponseStatus_t ModbusHost_SendRequest(uint8_t *request,
                                                   uint16_t requestLen,
                                                   uint8_t *response,
                                                   uint16_t responseMax,
                                                   ModbusHost_ResponseInfo_t *info)
{
  uint16_t responseLen;
  TickType_t startTick;
  ModbusHost_ResponseStatus_t status = MODBUS_HOST_RESP_FRAME_ERROR;

  if ((g_requestMutex == NULL) || (response == NULL) || (responseMax == 0U))
  {
    return MODBUS_HOST_RESP_ARG_ERROR;
  }

  (void)xSemaphoreTake(g_requestMutex, portMAX_DELAY);

  if (ModbusRtu_Send(request, requestLen, 100U) != HAL_OK)
  {
    (void)xSemaphoreGive(g_requestMutex);
    return status;
  }

  startTick = xTaskGetTickCount();
  while ((xTaskGetTickCount() - startTick) < pdMS_TO_TICKS(100U))
  {
    if (ModbusRtu_PollFrame(response, responseMax, &responseLen) != 0U)
    {
      status = ModbusHost_CheckResponseFrame(request, requestLen, response, responseLen, info);
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(1U));
  }

  (void)xSemaphoreGive(g_requestMutex);
  return status;
}
