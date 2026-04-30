#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include "global_status.h"
#include <stdint.h>

#define MODBUS_SLAVE_ADDR              1U
#define MODBUS_SLAVE_MAX_ADU_SIZE      256U
#define MODBUS_SLAVE_COIL_COUNT        GLOBAL_STATUS_COIL_COUNT
#define MODBUS_SLAVE_DISCRETE_COUNT    64U
#define MODBUS_SLAVE_INPUT_REG_COUNT   GLOBAL_STATUS_INPUT_REG_COUNT
#define MODBUS_SLAVE_HOLDING_REG_COUNT 64U

void ModbusSlave_InitData(void);
uint16_t ModbusSlave_HandleRequest(const uint8_t *request,
                                   uint16_t requestLen,
                                   uint8_t *response,
                                   uint16_t responseMax);

#endif /* MODBUS_SLAVE_H */
