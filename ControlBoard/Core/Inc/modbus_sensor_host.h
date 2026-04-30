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
#define MODBUS_SENSOR_NORMAL_REQUEST_INTERVAL_MS 50U
#define MODBUS_SENSOR_ONLINE_REQUEST_INTERVAL_MS 50U
#define MODBUS_SENSOR_FULL_SCAN_BATCH_COUNT 4U
#define MODBUS_SENSOR_POLL_CYCLE_INTERVAL_MS 10U
#define MODBUS_SENSOR_POLL_RESULT_NONE       0x00U
#define MODBUS_SENSOR_POLL_RESULT_POLLED     0x01U
#define MODBUS_SENSOR_POLL_RESULT_ROUND_DONE 0x02U

typedef enum
{
  MODBUS_SENSOR_STATUS_OK = 0,
  MODBUS_SENSOR_STATUS_ARG_ERROR,
  MODBUS_SENSOR_STATUS_TIMEOUT,
  MODBUS_SENSOR_STATUS_CRC_ERROR,
  MODBUS_SENSOR_STATUS_FRAME_ERROR
} ModbusSensor_Status_t;

typedef enum
{
  MODBUS_SENSOR_POLL_ALL = 0,
  MODBUS_SENSOR_POLL_ONLINE
} ModbusSensorPollMode_t;

uint8_t ModbusSensorHost_Init(UART_HandleTypeDef *uart);
uint8_t ModbusSensorHost_Poll(ModbusSensorPollMode_t mode,
                              uint8_t *nextSensorIndex,
                              uint8_t maxPollCount,
                              uint32_t requestIntervalMs,
                              uint32_t responseTimeoutMs);
uint8_t ModbusSensorHost_ProbeZeroAddress(uint32_t responseTimeoutMs);

#endif /* MODBUS_SENSOR_HOST_H */
