#ifndef MODBUS_HOST_H
#define MODBUS_HOST_H

#include <stdint.h>

#define MODBUS_HOST_MAX_ADU_SIZE       256U
#define MODBUS_HOST_DEFAULT_SLAVE_ADDR 1U

typedef enum
{
  MODBUS_HOST_RESP_OK = 0,
  MODBUS_HOST_RESP_ARG_ERROR,
  MODBUS_HOST_RESP_CRC_ERROR,
  MODBUS_HOST_RESP_FRAME_ERROR
} ModbusHost_ResponseStatus_t;

typedef struct
{
  uint8_t slaveAddr;
  uint8_t functionCode;
  const uint8_t *payload;
  uint16_t payloadLen;
} ModbusHost_ResponseInfo_t;

uint8_t ModbusHost_Init(void);
uint16_t ModbusHost_Crc16(const uint8_t *data, uint16_t length);
uint8_t ModbusHost_IsWriteRequest(const uint8_t *request, uint16_t requestLen);
uint16_t ModbusHost_BuildReadCoilsReq(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint8_t *request, uint16_t requestMax);
uint16_t ModbusHost_BuildReadDiscreteInputsReq(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint8_t *request, uint16_t requestMax);
uint16_t ModbusHost_BuildReadHoldingReq(uint8_t slaveAddr, uint16_t startReg, uint16_t quantity, uint8_t *request, uint16_t requestMax);
uint16_t ModbusHost_BuildReadInputRegsReq(uint8_t slaveAddr, uint16_t startReg, uint16_t quantity, uint8_t *request, uint16_t requestMax);
uint16_t ModbusHost_BuildWriteSingleCoilReq(uint8_t slaveAddr, uint16_t coilAddr, uint8_t coilValue, uint8_t *request, uint16_t requestMax);
uint16_t ModbusHost_BuildWriteSingleRegReq(uint8_t slaveAddr, uint16_t regAddr, uint16_t regValue, uint8_t *request, uint16_t requestMax);
uint16_t ModbusHost_BuildWriteMultipleCoilsReq(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, const uint8_t *values, uint8_t *request, uint16_t requestMax);
uint16_t ModbusHost_BuildWriteMultipleRegsReq(uint8_t slaveAddr, uint16_t startReg, uint16_t quantity, const uint16_t *values, uint8_t *request, uint16_t requestMax);
ModbusHost_ResponseStatus_t ModbusHost_SendRequest(uint8_t *request,
                                                   uint16_t requestLen,
                                                   uint8_t *response,
                                                   uint16_t responseMax,
                                                   ModbusHost_ResponseInfo_t *info);
ModbusHost_ResponseStatus_t ModbusHost_CheckResponseFrame(const uint8_t *request,
                                                          uint16_t requestLen,
                                                          const uint8_t *response,
                                                          uint16_t responseLen,
                                                          ModbusHost_ResponseInfo_t *info);

#endif /* MODBUS_HOST_H */
