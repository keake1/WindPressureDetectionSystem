#include "modbus_slave.h"

#include <string.h>

#define MB_SLAVE_ADDR             1U
#define MB_COIL_COUNT             64U
#define MB_DISCRETE_COUNT         64U
#define MB_INPUT_REG_COUNT        64U
#define MB_HOLDING_REG_COUNT      64U

static uint8_t g_mbCoils[MB_COIL_COUNT];
static uint8_t g_mbDiscreteInputs[MB_DISCRETE_COUNT];
static uint16_t g_mbInputRegs[MB_INPUT_REG_COUNT];
static uint16_t g_mbHoldingRegs[MB_HOLDING_REG_COUNT];

static uint8_t Modbus_GetBit(const uint8_t *array, uint16_t index, uint16_t maxCount)
{
  if ((array == NULL) || (index >= maxCount))
  {
    return 0U;
  }

  return array[index] ? 1U : 0U;
}

static void Modbus_SetBit(uint8_t *array, uint16_t index, uint16_t maxCount, uint8_t value)
{
  if ((array == NULL) || (index >= maxCount))
  {
    return;
  }

  array[index] = value ? 1U : 0U;
}

static uint16_t Modbus_Crc16(const uint8_t *data, uint16_t length)
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

void ModbusSlave_InitData(void)
{
  memset(g_mbCoils, 0, sizeof(g_mbCoils));
  memset(g_mbDiscreteInputs, 0, sizeof(g_mbDiscreteInputs));
  memset(g_mbInputRegs, 0, sizeof(g_mbInputRegs));
  memset(g_mbHoldingRegs, 0, sizeof(g_mbHoldingRegs));

  g_mbInputRegs[0] = 0x1234U;
  g_mbInputRegs[1] = 0x5678U;
  g_mbHoldingRegs[0] = 0x0001U;
}

uint16_t ModbusSlave_HandleRequest(const uint8_t *request, uint16_t requestLen, uint8_t *response, uint16_t responseMax)
{
  uint8_t address;
  uint8_t function;
  uint16_t crcRx;
  uint16_t crcCalc;
  uint16_t start;
  uint16_t quantity;
  uint16_t value;
  uint16_t i;
  uint8_t isBroadcast;

  if ((request == NULL) || (response == NULL) || (requestLen < 8U))
  {
    return 0U;
  }

  address = request[0];
  function = request[1];
  isBroadcast = (address == 0U) ? 1U : 0U;

  if ((address != MB_SLAVE_ADDR) && (isBroadcast == 0U))
  {
    return 0U;
  }

  crcRx = (uint16_t)request[requestLen - 2U] | ((uint16_t)request[requestLen - 1U] << 8);
  crcCalc = Modbus_Crc16(request, (uint16_t)(requestLen - 2U));
  if (crcRx != crcCalc)
  {
    return 0U;
  }

  start = ((uint16_t)request[2] << 8) | (uint16_t)request[3];
  quantity = ((uint16_t)request[4] << 8) | (uint16_t)request[5];

  switch (function)
  {
    case 0x01U:
    {
      uint8_t byteCount;
      uint16_t payloadLen;

      if ((quantity == 0U) || (quantity > 2000U))
      {
        return 0U;  /* 异常不回复 */
      }
      if ((start + quantity) > MB_COIL_COUNT)
      {
        return 0U;  /* 异常不回复 */
      }

      if (isBroadcast != 0U)
      {
        return 0U;
      }

      byteCount = (uint8_t)((quantity + 7U) / 8U);
      payloadLen = (uint16_t)(3U + byteCount);
      if (responseMax < (payloadLen + 2U))
      {
        return 0U;
      }

      response[0] = MB_SLAVE_ADDR;
      response[1] = function;
      response[2] = byteCount;
      memset(&response[3], 0, byteCount);
      for (i = 0U; i < quantity; ++i)
      {
        if (Modbus_GetBit(g_mbCoils, (uint16_t)(start + i), MB_COIL_COUNT) != 0U)
        {
          response[3U + (i / 8U)] |= (uint8_t)(1U << (i % 8U));
        }
      }

      crcCalc = Modbus_Crc16(response, payloadLen);
      response[payloadLen] = (uint8_t)(crcCalc & 0xFFU);
      response[payloadLen + 1U] = (uint8_t)((crcCalc >> 8) & 0xFFU);
      return (uint16_t)(payloadLen + 2U);
    }

    case 0x02U:
    {
      uint8_t byteCount;
      uint16_t payloadLen;

      if ((quantity == 0U) || (quantity > 2000U))
      {
        return 0U;  /* 异常不回复 */
      }
      if ((start + quantity) > MB_DISCRETE_COUNT)
      {
        return 0U;  /* 异常不回复 */
      }

      if (isBroadcast != 0U)
      {
        return 0U;
      }

      byteCount = (uint8_t)((quantity + 7U) / 8U);
      payloadLen = (uint16_t)(3U + byteCount);
      if (responseMax < (payloadLen + 2U))
      {
        return 0U;
      }

      response[0] = MB_SLAVE_ADDR;
      response[1] = function;
      response[2] = byteCount;
      memset(&response[3], 0, byteCount);
      for (i = 0U; i < quantity; ++i)
      {
        if (Modbus_GetBit(g_mbDiscreteInputs, (uint16_t)(start + i), MB_DISCRETE_COUNT) != 0U)
        {
          response[3U + (i / 8U)] |= (uint8_t)(1U << (i % 8U));
        }
      }

      crcCalc = Modbus_Crc16(response, payloadLen);
      response[payloadLen] = (uint8_t)(crcCalc & 0xFFU);
      response[payloadLen + 1U] = (uint8_t)((crcCalc >> 8) & 0xFFU);
      return (uint16_t)(payloadLen + 2U);
    }

    case 0x03U:
    {
      uint16_t payloadLen;

      if ((quantity == 0U) || (quantity > 125U))
      {
        return 0U;  /* 异常不回复 */
      }
      if ((start + quantity) > MB_HOLDING_REG_COUNT)
      {
        return 0U;  /* 异常不回复 */
      }

      if (isBroadcast != 0U)
      {
        return 0U;
      }

      payloadLen = (uint16_t)(3U + (2U * quantity));
      if (responseMax < (payloadLen + 2U))
      {
        return 0U;
      }

      response[0] = MB_SLAVE_ADDR;
      response[1] = function;
      response[2] = (uint8_t)(2U * quantity);
      for (i = 0U; i < quantity; ++i)
      {
        uint16_t reg = g_mbHoldingRegs[start + i];
        response[3U + (2U * i)] = (uint8_t)((reg >> 8) & 0xFFU);
        response[4U + (2U * i)] = (uint8_t)(reg & 0xFFU);
      }

      crcCalc = Modbus_Crc16(response, payloadLen);
      response[payloadLen] = (uint8_t)(crcCalc & 0xFFU);
      response[payloadLen + 1U] = (uint8_t)((crcCalc >> 8) & 0xFFU);
      return (uint16_t)(payloadLen + 2U);
    }

    case 0x04U:
    {
      uint16_t payloadLen;

      if ((quantity == 0U) || (quantity > 125U))
      {
        return 0U;  /* 异常不回复 */
      }
      if ((start + quantity) > MB_INPUT_REG_COUNT)
      {
        return 0U;  /* 异常不回复 */
      }

      if (isBroadcast != 0U)
      {
        return 0U;
      }

      payloadLen = (uint16_t)(3U + (2U * quantity));
      if (responseMax < (payloadLen + 2U))
      {
        return 0U;
      }

      response[0] = MB_SLAVE_ADDR;
      response[1] = function;
      response[2] = (uint8_t)(2U * quantity);
      for (i = 0U; i < quantity; ++i)
      {
        uint16_t reg = g_mbInputRegs[start + i];
        response[3U + (2U * i)] = (uint8_t)((reg >> 8) & 0xFFU);
        response[4U + (2U * i)] = (uint8_t)(reg & 0xFFU);
      }

      crcCalc = Modbus_Crc16(response, payloadLen);
      response[payloadLen] = (uint8_t)(crcCalc & 0xFFU);
      response[payloadLen + 1U] = (uint8_t)((crcCalc >> 8) & 0xFFU);
      return (uint16_t)(payloadLen + 2U);
    }

    case 0x05U:
    {
      if ((start + 1U) > MB_COIL_COUNT)
      {
        return 0U;  /* 异常不回复 */
      }

      value = ((uint16_t)request[4] << 8) | (uint16_t)request[5];
      if ((value != 0xFF00U) && (value != 0x0000U))
      {
        return 0U;  /* 异常不回复 */
      }

      Modbus_SetBit(g_mbCoils, start, MB_COIL_COUNT, (uint8_t)(value == 0xFF00U));
      if (isBroadcast != 0U)
      {
        return 0U;
      }

      if (responseMax < requestLen)
      {
        return 0U;
      }
      memcpy(response, request, requestLen);
      return requestLen;
    }

    case 0x06U:
    {
      if ((start + 1U) > MB_HOLDING_REG_COUNT)
      {
        return 0U;  /* 异常不回复 */
      }

      value = ((uint16_t)request[4] << 8) | (uint16_t)request[5];
      g_mbHoldingRegs[start] = value;
      if (isBroadcast != 0U)
      {
        return 0U;
      }

      if (responseMax < requestLen)
      {
        return 0U;
      }
      memcpy(response, request, requestLen);
      return requestLen;
    }

    default:
      return 0U;  /* 不支持的功能码：不回复 */
  }
}
