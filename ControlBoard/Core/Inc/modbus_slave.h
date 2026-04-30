#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <stdint.h>

#define MODBUS_SLAVE_ADDR              1U
#define MODBUS_SLAVE_MAX_ADU_SIZE      256U
#define MODBUS_SLAVE_COIL_COUNT        64U
#define MODBUS_SLAVE_DISCRETE_COUNT    64U
#define MODBUS_SLAVE_INPUT_REG_COUNT   64U
#define MODBUS_SLAVE_HOLDING_REG_COUNT 64U

void ModbusSlave_InitData(void);
void ModbusSlave_SetDiscreteInput(uint16_t address, uint8_t value);
void ModbusSlave_SetInputRegister(uint16_t address, uint16_t value);
uint8_t ModbusSlave_GetCoil(uint16_t address);
uint16_t ModbusSlave_GetHoldingRegister(uint16_t address);
uint16_t ModbusSlave_HandleRequest(const uint8_t *request,
                                   uint16_t requestLen,
                                   uint8_t *response,
                                   uint16_t responseMax);

#endif /* MODBUS_SLAVE_H */
