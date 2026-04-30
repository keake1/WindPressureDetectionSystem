#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include "main.h"
#include <stdint.h>

#define MODBUS_RTU_MAX_FRAME_SIZE 256U

void ModbusRtu_Init(UART_HandleTypeDef *uart);
uint8_t ModbusRtu_PollFrame(uint8_t *frame, uint16_t frameMax, uint16_t *frameLen);
HAL_StatusTypeDef ModbusRtu_Send(const uint8_t *frame, uint16_t frameLen, uint32_t timeoutMs);

#endif /* MODBUS_RTU_H */
