#include "modbus_slave.h"

#include "modbus_rtu.h"
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
#define MB_BROADCAST_ADDR             0U
#define MB_MIN_FRAME_SIZE             8U
#define MB_EXC_NONE                   0x00U
#define MB_EXC_ILLEGAL_FUNCTION       0x01U
#define MB_EXC_ILLEGAL_DATA_ADDRESS   0x02U
#define MB_EXC_ILLEGAL_DATA_VALUE     0x03U

typedef struct
{
  uint8_t address;
  uint8_t function;
  uint16_t start;
  uint16_t quantity;
  uint8_t broadcast;
} ModbusRequest_t;

/* 上行从机离散输入后备区，当前预留给后续开关量状态。 */
static uint8_t g_discreteInputs[MODBUS_SLAVE_DISCRETE_COUNT];
/* 上行从机保持寄存器后备区，当前用于保存控制类或配置类数据。 */
static uint16_t g_holdingRegs[MODBUS_SLAVE_HOLDING_REG_COUNT];

static uint8_t Mb_IsWriteFunction(uint8_t function)
{
  return ((function == MB_FC_WRITE_SINGLE_COIL) ||
          (function == MB_FC_WRITE_SINGLE_REG) ||
          (function == MB_FC_WRITE_MULTIPLE_COILS) ||
          (function == MB_FC_WRITE_MULTIPLE_REGS)) ? 1U : 0U;
}

static uint8_t Mb_GetBit(const uint8_t *array, uint16_t count, uint16_t address)
{
  if ((array == NULL) || (address >= count))
  {
    return 0U;
  }

  return array[address] ? 1U : 0U;
}

static void Mb_SetBit(uint8_t *array, uint16_t count, uint16_t address, uint8_t value)
{
  if ((array == NULL) || (address >= count))
  {
    return;
  }

  array[address] = value ? 1U : 0U;
}

static uint8_t Mb_RangeValid(uint16_t start, uint16_t quantity, uint16_t count)
{
  if (quantity == 0U)
  {
    return 0U;
  }

  return (start <= count) && (quantity <= (uint16_t)(count - start));
}

static uint8_t Mb_FrameValid(const uint8_t *frame, uint16_t length)
{
  uint16_t crcRx;
  uint16_t crcCalc;

  if ((frame == NULL) || (length < MB_MIN_FRAME_SIZE))
  {
    return 0U;
  }

  crcRx = (uint16_t)frame[length - 2U] | ((uint16_t)frame[length - 1U] << 8);
  crcCalc = ModbusRtu_Crc16(frame, (uint16_t)(length - 2U));
  return (crcRx == crcCalc) ? 1U : 0U;
}

static void Mb_ParseRequest(const uint8_t *request, ModbusRequest_t *parsed)
{
  parsed->address = request[0];
  parsed->function = request[1];
  parsed->start = ModbusRtu_ReadU16BE(&request[2]);
  parsed->quantity = ModbusRtu_ReadU16BE(&request[4]);
  parsed->broadcast = (request[0] == MB_BROADCAST_ADDR) ? 1U : 0U;
}

static uint8_t Mb_AddressMatches(const ModbusRequest_t *request)
{
  /* 广播地址 0 仅写功能码合法（5/6/0F/10），读功能码遇到广播地址直接丢弃。
   * 单播必须严格匹配本机地址才回复，否则保持沉默。 */
  if (request->broadcast != 0U)
  {
    return Mb_IsWriteFunction(request->function);
  }

  return (request->address == MODBUS_SLAVE_ADDR) ? 1U : 0U;
}

static uint16_t Mb_FinishResponse(uint8_t *response, uint16_t payloadLen)
{
  uint16_t crc = ModbusRtu_Crc16(response, payloadLen);
  response[payloadLen] = (uint8_t)crc;
  response[payloadLen + 1U] = (uint8_t)(crc >> 8);
  return (uint16_t)(payloadLen + 2U);
}

/* 构造异常响应帧：地址 + (功能码|0x80) + 异常码 + CRC。 */
static uint16_t Mb_BuildException(const ModbusRequest_t *request,
                                  uint8_t exceptionCode,
                                  uint8_t *response,
                                  uint16_t responseMax)
{
  if ((response == NULL) || (responseMax < 5U))
  {
    return 0U;
  }

  response[0] = MODBUS_SLAVE_ADDR;
  response[1] = (uint8_t)(request->function | 0x80U);
  response[2] = exceptionCode;
  return Mb_FinishResponse(response, 3U);
}

static uint16_t Mb_ReadBits(const ModbusRequest_t *request,
                            const uint8_t *bits,
                            uint16_t bitCount,
                            uint8_t *response,
                            uint16_t responseMax,
                            uint8_t *exception)
{
  uint8_t byteCount;
  uint16_t i;

  if (request->quantity > 2000U)
  {
    *exception = MB_EXC_ILLEGAL_DATA_VALUE;
    return 0U;
  }
  if (Mb_RangeValid(request->start, request->quantity, bitCount) == 0U)
  {
    *exception = MB_EXC_ILLEGAL_DATA_ADDRESS;
    return 0U;
  }

  byteCount = (uint8_t)((request->quantity + 7U) / 8U);
  if (responseMax < (uint16_t)(byteCount + 5U))
  {
    return 0U;
  }

  response[0] = MODBUS_SLAVE_ADDR;
  response[1] = request->function;
  response[2] = byteCount;
  memset(&response[3], 0, byteCount);

  /* Modbus 线圈响应按位打包，最低地址放在第一个数据字节的 bit0。 */
  for (i = 0U; i < request->quantity; ++i)
  {
    uint16_t address = (uint16_t)(request->start + i);
    response[3U + (i / 8U)] |= (uint8_t)(Mb_GetBit(bits, bitCount, address) << (i % 8U));
  }

  return Mb_FinishResponse(response, (uint16_t)(byteCount + 3U));
}

static uint16_t Mb_ReadRegisters(const ModbusRequest_t *request,
                                 const uint16_t *registers,
                                 uint16_t registerCount,
                                 uint8_t *response,
                                 uint16_t responseMax,
                                 uint8_t *exception)
{
  uint16_t i;
  uint16_t byteCount;

  if (request->quantity > 125U)
  {
    *exception = MB_EXC_ILLEGAL_DATA_VALUE;
    return 0U;
  }
  if (Mb_RangeValid(request->start, request->quantity, registerCount) == 0U)
  {
    *exception = MB_EXC_ILLEGAL_DATA_ADDRESS;
    return 0U;
  }

  byteCount = (uint16_t)(request->quantity * 2U);
  if (responseMax < (uint16_t)(byteCount + 5U))
  {
    return 0U;
  }

  response[0] = MODBUS_SLAVE_ADDR;
  response[1] = request->function;
  response[2] = (uint8_t)byteCount;

  for (i = 0U; i < request->quantity; ++i)
  {
    ModbusRtu_WriteU16BE(&response[3U + (2U * i)], registers[request->start + i]);
  }

  return Mb_FinishResponse(response, (uint16_t)(byteCount + 3U));
}

static uint8_t Mb_WriteSingleCoil(const ModbusRequest_t *request, uint8_t *exception)
{
  uint16_t value = request->quantity;

  if (request->start >= MODBUS_SLAVE_COIL_COUNT)
  {
    *exception = MB_EXC_ILLEGAL_DATA_ADDRESS;
    return 0U;
  }
  if ((value != 0xFF00U) && (value != 0x0000U))
  {
    *exception = MB_EXC_ILLEGAL_DATA_VALUE;
    return 0U;
  }

  Mb_SetBit(g_modbusCoils, MODBUS_SLAVE_COIL_COUNT, request->start, (uint8_t)(value == 0xFF00U));
  return 1U;
}

static uint8_t Mb_WriteSingleRegister(const ModbusRequest_t *request, uint8_t *exception)
{
  if (request->start >= MODBUS_SLAVE_HOLDING_REG_COUNT)
  {
    *exception = MB_EXC_ILLEGAL_DATA_ADDRESS;
    return 0U;
  }

  g_holdingRegs[request->start] = request->quantity;
  return 1U;
}

static uint8_t Mb_WriteMultipleCoils(const ModbusRequest_t *request,
                                     const uint8_t *frame,
                                     uint16_t frameLen,
                                     uint8_t *exception)
{
  uint8_t byteCount = frame[6];
  uint8_t expectedBytes = (uint8_t)((request->quantity + 7U) / 8U);
  uint16_t i;

  if ((frameLen != (uint16_t)(byteCount + 9U)) || (byteCount != expectedBytes))
  {
    *exception = MB_EXC_ILLEGAL_DATA_VALUE;
    return 0U;
  }
  if (request->quantity > 1968U)
  {
    *exception = MB_EXC_ILLEGAL_DATA_VALUE;
    return 0U;
  }
  if (Mb_RangeValid(request->start, request->quantity, MODBUS_SLAVE_COIL_COUNT) == 0U)
  {
    *exception = MB_EXC_ILLEGAL_DATA_ADDRESS;
    return 0U;
  }

  for (i = 0U; i < request->quantity; ++i)
  {
    uint8_t value = (frame[7U + (i / 8U)] >> (i % 8U)) & 0x01U;
    Mb_SetBit(g_modbusCoils, MODBUS_SLAVE_COIL_COUNT, (uint16_t)(request->start + i), value);
  }

  return 1U;
}

static uint8_t Mb_WriteMultipleRegisters(const ModbusRequest_t *request,
                                         const uint8_t *frame,
                                         uint16_t frameLen,
                                         uint8_t *exception)
{
  uint8_t byteCount = frame[6];
  uint16_t i;

  if ((frameLen != (uint16_t)(byteCount + 9U)) || (byteCount != (uint8_t)(request->quantity * 2U)))
  {
    *exception = MB_EXC_ILLEGAL_DATA_VALUE;
    return 0U;
  }
  if (request->quantity > 123U)
  {
    *exception = MB_EXC_ILLEGAL_DATA_VALUE;
    return 0U;
  }
  if (Mb_RangeValid(request->start, request->quantity, MODBUS_SLAVE_HOLDING_REG_COUNT) == 0U)
  {
    *exception = MB_EXC_ILLEGAL_DATA_ADDRESS;
    return 0U;
  }

  for (i = 0U; i < request->quantity; ++i)
  {
    g_holdingRegs[request->start + i] = ModbusRtu_ReadU16BE(&frame[7U + (2U * i)]);
  }

  return 1U;
}

static uint8_t Mb_ApplyWrite(const ModbusRequest_t *request,
                             const uint8_t *frame,
                             uint16_t frameLen,
                             uint8_t *exception)
{
  switch (request->function)
  {
    case MB_FC_WRITE_SINGLE_COIL:
      return Mb_WriteSingleCoil(request, exception);
    case MB_FC_WRITE_SINGLE_REG:
      return Mb_WriteSingleRegister(request, exception);
    case MB_FC_WRITE_MULTIPLE_COILS:
      return Mb_WriteMultipleCoils(request, frame, frameLen, exception);
    case MB_FC_WRITE_MULTIPLE_REGS:
      return Mb_WriteMultipleRegisters(request, frame, frameLen, exception);
    default:
      *exception = MB_EXC_ILLEGAL_FUNCTION;
      return 0U;
  }
}

static uint16_t Mb_BuildWriteResponse(const ModbusRequest_t *request,
                                      uint8_t *response,
                                      uint16_t responseMax)
{
  if (responseMax < MB_MIN_FRAME_SIZE)
  {
    return 0U;
  }

  response[0] = MODBUS_SLAVE_ADDR;
  response[1] = request->function;
  ModbusRtu_WriteU16BE(&response[2], request->start);
  ModbusRtu_WriteU16BE(&response[4], request->quantity);
  return Mb_FinishResponse(response, 6U);
}

static uint16_t Mb_BuildReadResponse(const ModbusRequest_t *request,
                                     uint8_t *response,
                                     uint16_t responseMax,
                                     uint8_t *exception)
{
  switch (request->function)
  {
    case MB_FC_READ_COILS:
      /* 线圈区公开传感器在线/报警状态。 */
      return Mb_ReadBits(request, g_modbusCoils, MODBUS_SLAVE_COIL_COUNT,
                         response, responseMax, exception);
    case MB_FC_READ_DISCRETE_INPUTS:
      return Mb_ReadBits(request, g_discreteInputs, MODBUS_SLAVE_DISCRETE_COUNT,
                         response, responseMax, exception);
    case MB_FC_READ_HOLDING_REGS:
      return Mb_ReadRegisters(request, g_holdingRegs, MODBUS_SLAVE_HOLDING_REG_COUNT,
                              response, responseMax, exception);
    case MB_FC_READ_INPUT_REGS:
      /* 输入寄存器区公开传感器数据、型号和诊断字段。 */
      return Mb_ReadRegisters(request, g_modbusInputRegs, MODBUS_SLAVE_INPUT_REG_COUNT,
                              response, responseMax, exception);
    default:
      *exception = MB_EXC_ILLEGAL_FUNCTION;
      return 0U;
  }
}

void ModbusSlave_InitData(void)
{
  /* 初始化上行从机数据区；全局状态区由传感器轮询任务持续刷新。 */
  GlobalStatus_Init();
  memset(g_discreteInputs, 0, sizeof(g_discreteInputs));
  memset(g_holdingRegs, 0, sizeof(g_holdingRegs));

  g_holdingRegs[0] = 0x0001U;
}

uint16_t ModbusSlave_HandleRequest(const uint8_t *request,
                                   uint16_t requestLen,
                                   uint8_t *response,
                                   uint16_t responseMax)
{
  ModbusRequest_t parsed;
  uint8_t exception = MB_EXC_NONE;
  uint16_t respLen;

  /* CRC 错误或长度不足：完全不响应（符合 Modbus 规范）。 */
  if (Mb_FrameValid(request, requestLen) == 0U)
  {
    return 0U;
  }

  Mb_ParseRequest(request, &parsed);
  if (Mb_AddressMatches(&parsed) == 0U)
  {
    return 0U;
  }

  if (response == NULL)
  {
    return 0U;
  }

  if (Mb_IsWriteFunction(parsed.function) != 0U)
  {
    if (Mb_ApplyWrite(&parsed, request, requestLen, &exception) == 0U)
    {
      /* 广播写入失败也保持沉默；单播写入失败回异常帧。 */
      if (parsed.broadcast != 0U)
      {
        return 0U;
      }
      return Mb_BuildException(&parsed, exception, response, responseMax);
    }

    /* 广播写请求只执行不回复，符合 Modbus RTU 约定。 */
    if (parsed.broadcast != 0U)
    {
      return 0U;
    }

    return Mb_BuildWriteResponse(&parsed, response, responseMax);
  }

  /* 读功能码遇到广播地址在 Mb_AddressMatches 中已被过滤。 */
  respLen = Mb_BuildReadResponse(&parsed, response, responseMax, &exception);
  if ((respLen == 0U) && (exception != MB_EXC_NONE))
  {
    return Mb_BuildException(&parsed, exception, response, responseMax);
  }
  return respLen;
}
