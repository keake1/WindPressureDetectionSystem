#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include "main.h"
#include <stdint.h>

#define MODBUS_RTU_MAX_FRAME_SIZE 256U

uint8_t ModbusRtu_InitPort(UART_HandleTypeDef *uart);
uint8_t ModbusRtu_PollFrameFrom(UART_HandleTypeDef *uart, uint8_t *frame, uint16_t frameMax, uint16_t *frameLen);
void ModbusRtu_ClearPort(UART_HandleTypeDef *uart);
HAL_StatusTypeDef ModbusRtu_SendTo(UART_HandleTypeDef *uart, const uint8_t *frame, uint16_t frameLen, uint32_t timeoutMs);

#endif /* MODBUS_RTU_H */
