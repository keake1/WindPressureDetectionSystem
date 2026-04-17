#include <stddef.h>
#include "modbus_host.h"

static uint16_t ModbusHost_ReadU16BE(const uint8_t *data)
{
  return ((uint16_t)data[0] << 8) | (uint16_t)data[1];
}

static void ModbusHost_ClearResponseInfo(ModbusHost_ResponseInfo_t *info)
{
  if (info != NULL)
  {
    info->slaveAddr = 0U;
    info->functionCode = 0U;
    info->payload = NULL;
    info->payloadLen = 0U;
  }
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
      if ((crc & 0x0001U) != 0U)
      {
        crc >>= 1U;
        crc ^= 0xA001U;
      }
      else
      {
        crc >>= 1U;
      }
    }
  }

  return crc;
}

/* Function Code 01: Read Coils */
uint16_t ModbusHost_BuildReadCoilsReq(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint8_t *request, uint16_t requestMax)
{
  uint16_t crc;

  if ((request == NULL) || (requestMax < 8U) || (quantity == 0U) || (quantity > 2000U))
  {
    return 0U;
  }

  request[0] = slaveAddr;
  request[1] = 0x01U;  /* Function code */
  request[2] = (uint8_t)((startAddr >> 8) & 0xFFU);
  request[3] = (uint8_t)(startAddr & 0xFFU);
  request[4] = (uint8_t)((quantity >> 8) & 0xFFU);
  request[5] = (uint8_t)(quantity & 0xFFU);

  crc = ModbusHost_Crc16(request, 6U);
  request[6] = (uint8_t)(crc & 0xFFU);
  request[7] = (uint8_t)((crc >> 8) & 0xFFU);
  return 8U;
}

/* Function Code 02: Read Discrete Inputs */
uint16_t ModbusHost_BuildReadDiscreteInputsReq(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint8_t *request, uint16_t requestMax)
{
  uint16_t crc;

  if ((request == NULL) || (requestMax < 8U) || (quantity == 0U) || (quantity > 2000U))
  {
    return 0U;
  }

  request[0] = slaveAddr;
  request[1] = 0x02U;  /* Function code */
  request[2] = (uint8_t)((startAddr >> 8) & 0xFFU);
  request[3] = (uint8_t)(startAddr & 0xFFU);
  request[4] = (uint8_t)((quantity >> 8) & 0xFFU);
  request[5] = (uint8_t)(quantity & 0xFFU);

  crc = ModbusHost_Crc16(request, 6U);
  request[6] = (uint8_t)(crc & 0xFFU);
  request[7] = (uint8_t)((crc >> 8) & 0xFFU);
  return 8U;
}

/* Function Code 03: Read Holding Registers */
uint16_t ModbusHost_BuildReadHoldingReq(uint8_t slaveAddr, uint16_t startReg, uint16_t quantity, uint8_t *request, uint16_t requestMax)
{
  uint16_t crc;

  if ((request == NULL) || (requestMax < 8U) || (quantity == 0U) || (quantity > 125U))
  {
    return 0U;
  }

  request[0] = slaveAddr;
  request[1] = 0x03U;  /* Function code */
  request[2] = (uint8_t)((startReg >> 8) & 0xFFU);
  request[3] = (uint8_t)(startReg & 0xFFU);
  request[4] = (uint8_t)((quantity >> 8) & 0xFFU);
  request[5] = (uint8_t)(quantity & 0xFFU);

  crc = ModbusHost_Crc16(request, 6U);
  request[6] = (uint8_t)(crc & 0xFFU);
  request[7] = (uint8_t)((crc >> 8) & 0xFFU);
  return 8U;
}

/* Function Code 04: Read Input Registers */
uint16_t ModbusHost_BuildReadInputRegsReq(uint8_t slaveAddr, uint16_t startReg, uint16_t quantity, uint8_t *request, uint16_t requestMax)
{
  uint16_t crc;

  if ((request == NULL) || (requestMax < 8U) || (quantity == 0U) || (quantity > 125U))
  {
    return 0U;
  }

  request[0] = slaveAddr;
  request[1] = 0x04U;  /* Function code */
  request[2] = (uint8_t)((startReg >> 8) & 0xFFU);
  request[3] = (uint8_t)(startReg & 0xFFU);
  request[4] = (uint8_t)((quantity >> 8) & 0xFFU);
  request[5] = (uint8_t)(quantity & 0xFFU);

  crc = ModbusHost_Crc16(request, 6U);
  request[6] = (uint8_t)(crc & 0xFFU);
  request[7] = (uint8_t)((crc >> 8) & 0xFFU);
  return 8U;
}

/* Function Code 05: Write Single Coil */
uint16_t ModbusHost_BuildWriteSingleCoilReq(uint8_t slaveAddr, uint16_t coilAddr, uint8_t coilValue, uint8_t *request, uint16_t requestMax)
{
  uint16_t crc;

  if ((request == NULL) || (requestMax < 8U))
  {
    return 0U;
  }

  request[0] = slaveAddr;
  request[1] = 0x05U;  /* Function code */
  request[2] = (uint8_t)((coilAddr >> 8) & 0xFFU);
  request[3] = (uint8_t)(coilAddr & 0xFFU);
  request[4] = (coilValue != 0U) ? 0xFFU : 0x00U;  /* ON=0xFF00, OFF=0x0000 */
  request[5] = 0x00U;

  crc = ModbusHost_Crc16(request, 6U);
  request[6] = (uint8_t)(crc & 0xFFU);
  request[7] = (uint8_t)((crc >> 8) & 0xFFU);
  return 8U;
}

/* Function Code 06: Write Single Register */
uint16_t ModbusHost_BuildWriteSingleRegReq(uint8_t slaveAddr, uint16_t regAddr, uint16_t regValue, uint8_t *request, uint16_t requestMax)
{
  uint16_t crc;

  if ((request == NULL) || (requestMax < 8U))
  {
    return 0U;
  }

  request[0] = slaveAddr;
  request[1] = 0x06U;  /* Function code */
  request[2] = (uint8_t)((regAddr >> 8) & 0xFFU);
  request[3] = (uint8_t)(regAddr & 0xFFU);
  request[4] = (uint8_t)((regValue >> 8) & 0xFFU);
  request[5] = (uint8_t)(regValue & 0xFFU);

  crc = ModbusHost_Crc16(request, 6U);
  request[6] = (uint8_t)(crc & 0xFFU);
  request[7] = (uint8_t)((crc >> 8) & 0xFFU);
  return 8U;
}

ModbusHost_ResponseStatus_t ModbusHost_CheckResponseFrame(const uint8_t *request, uint16_t requestLen, const uint8_t *response, uint16_t responseLen, ModbusHost_ResponseInfo_t *info)
{
  uint8_t reqFunc;
  uint8_t rspFunc;
  uint16_t crcRx;
  uint16_t crcCalc;

  ModbusHost_ClearResponseInfo(info);

  /* 基本入参检查：指针有效性 */
  if ((request == NULL) || (response == NULL))
  {
    return MODBUS_HOST_RESP_ARG_ERROR;
  }

  /* 响应帧长度检查：最小 5 字节（addr+func+1data+crc_l+crc_h） */
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

  reqFunc = request[1];
  rspFunc = response[1];

  if (info != NULL)
  {
    info->slaveAddr = response[0];
    info->functionCode = rspFunc;
  }

  /* 从机异常直接不回复，无需处理异常响应 */

  /* 正常响应必须功能码匹配 */
  if (rspFunc != reqFunc)
  {
    return MODBUS_HOST_RESP_FRAME_ERROR;
  }

  switch (reqFunc)
  {
    case 0x01U:
    case 0x02U:
    {
      uint16_t quantity = ModbusHost_ReadU16BE(&request[4]);
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

    case 0x03U:
    case 0x04U:
    {
      uint16_t quantity = ModbusHost_ReadU16BE(&request[4]);
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

    case 0x05U:
    case 0x06U:
    {
      uint8_t i;
      if (responseLen != 8U)
      {
        return MODBUS_HOST_RESP_FRAME_ERROR;
      }

      for (i = 0U; i < 6U; ++i)
      {
        if (response[i] != request[i])
        {
          return MODBUS_HOST_RESP_FRAME_ERROR;
        }
      }

      if (info != NULL)
      {
        info->payload = &response[2];
        info->payloadLen = 4U;
      }
      return MODBUS_HOST_RESP_OK;
    }

    default:
      return MODBUS_HOST_RESP_ARG_ERROR;
  }
}

