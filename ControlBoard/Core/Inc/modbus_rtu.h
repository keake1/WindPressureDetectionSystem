#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include "main.h"
#include <stdint.h>

#define MODBUS_RTU_MAX_FRAME_SIZE 256U

uint8_t ModbusRtu_InitPort(UART_HandleTypeDef *uart);
uint8_t ModbusRtu_PollFrameFrom(UART_HandleTypeDef *uart, uint8_t *frame, uint16_t frameMax, uint16_t *frameLen);
void ModbusRtu_ClearPort(UART_HandleTypeDef *uart);
HAL_StatusTypeDef ModbusRtu_SendTo(UART_HandleTypeDef *uart, const uint8_t *frame, uint16_t frameLen, uint32_t timeoutMs);

/* Modbus RTU 公共工具：CRC-16 与大端 16 位读写。两个板的 modbus_rtu 实现刻意保持独立，
 * 但工具函数仅做无副作用计算，统一在此声明可避免在多处复制粘贴。 */
uint16_t ModbusRtu_Crc16(const uint8_t *data, uint16_t length);
uint16_t ModbusRtu_ReadU16BE(const uint8_t *data);
void ModbusRtu_WriteU16BE(uint8_t *data, uint16_t value);

#endif /* MODBUS_RTU_H */
