#ifndef MODBUS_HOST_H
#define MODBUS_HOST_H

#include <stdint.h>

#define MODBUS_HOST_MAX_ADU_SIZE      256U
#define MODBUS_HOST_DEFAULT_SLAVE_ADDR 1U

typedef enum
{
	MODBUS_HOST_RESP_OK = 0,          /* 响应有效，所有验证通过 */
	MODBUS_HOST_RESP_ARG_ERROR,       /* 入参错误（NULL指针、缓冲区过短等） */
	MODBUS_HOST_RESP_CRC_ERROR,       /* CRC16校验失败 */
	MODBUS_HOST_RESP_FRAME_ERROR      /* 帧格式错误（长度、地址、功能码、回显任意一项不符） */
} ModbusHost_ResponseStatus_t;

typedef struct
{
	uint8_t slaveAddr;
	uint8_t functionCode;
	const uint8_t *payload;
	uint16_t payloadLen;
} ModbusHost_ResponseInfo_t;

/* CRC16 calculation */
uint16_t ModbusHost_Crc16(const uint8_t *data, uint16_t length);

/* Function Code 01: Read Coils */
uint16_t ModbusHost_BuildReadCoilsReq(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint8_t *request, uint16_t requestMax);

/* Function Code 02: Read Discrete Inputs */
uint16_t ModbusHost_BuildReadDiscreteInputsReq(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint8_t *request, uint16_t requestMax);

/* Function Code 03: Read Holding Registers */
uint16_t ModbusHost_BuildReadHoldingReq(uint8_t slaveAddr, uint16_t startReg, uint16_t quantity, uint8_t *request, uint16_t requestMax);

/* Function Code 04: Read Input Registers */
uint16_t ModbusHost_BuildReadInputRegsReq(uint8_t slaveAddr, uint16_t startReg, uint16_t quantity, uint8_t *request, uint16_t requestMax);

/* Function Code 05: Write Single Coil */
uint16_t ModbusHost_BuildWriteSingleCoilReq(uint8_t slaveAddr, uint16_t coilAddr, uint8_t coilValue, uint8_t *request, uint16_t requestMax);

/* Function Code 06: Write Single Register */
uint16_t ModbusHost_BuildWriteSingleRegReq(uint8_t slaveAddr, uint16_t regAddr, uint16_t regValue, uint8_t *request, uint16_t requestMax);

/* Check response frame and expose payload metadata for future parsing */
ModbusHost_ResponseStatus_t ModbusHost_CheckResponseFrame(const uint8_t *request, uint16_t requestLen, const uint8_t *response, uint16_t responseLen, ModbusHost_ResponseInfo_t *info);

#endif /* MODBUS_HOST_H */
