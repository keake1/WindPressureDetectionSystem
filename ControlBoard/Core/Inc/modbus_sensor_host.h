#ifndef MODBUS_SENSOR_HOST_H
#define MODBUS_SENSOR_HOST_H

#include "global_status.h"
#include "main.h"
#include <stdint.h>

#define MODBUS_SENSOR_COUNT             64U
#define MODBUS_SENSOR_FIRST_ADDR        1U
#define MODBUS_SENSOR_DISCOVERY_RESPONSE_TIMEOUT_MS 30U
#define MODBUS_SENSOR_ONLINE_RESPONSE_TIMEOUT_MS 50U
#define MODBUS_SENSOR_ZERO_ADDR_RESPONSE_TIMEOUT_MS 30U
#define MODBUS_SENSOR_STARTUP_SCAN_ROUNDS 5U
#define MODBUS_SENSOR_FAST_REQUEST_INTERVAL_MS 0U
#define MODBUS_SENSOR_NORMAL_REQUEST_INTERVAL_MS 50U
#define MODBUS_SENSOR_ONLINE_REQUEST_INTERVAL_MS 50U
#define MODBUS_SENSOR_ONLINE_IDLE_INTERVAL_MS 500U

typedef enum
{
  MODBUS_SENSOR_STATUS_OK = 0,
  MODBUS_SENSOR_STATUS_ARG_ERROR,
  MODBUS_SENSOR_STATUS_TIMEOUT,
  MODBUS_SENSOR_STATUS_CRC_ERROR,
  MODBUS_SENSOR_STATUS_FRAME_ERROR
} ModbusSensor_Status_t;

uint8_t ModbusSensorHost_Init(UART_HandleTypeDef *uart);
void ModbusSensorHost_PollAll(uint16_t scanRounds, uint32_t requestIntervalMs, uint32_t responseTimeoutMs);
uint8_t ModbusSensorHost_PollOnline(uint32_t requestIntervalMs, uint32_t responseTimeoutMs);
uint8_t ModbusSensorHost_ProbeZeroAddress(uint32_t responseTimeoutMs);

#endif /* MODBUS_SENSOR_HOST_H */
