#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <stdint.h>

#define MODBUS_SLAVE_MAX_ADU_SIZE 256U

void ModbusSlave_InitData(void);
uint16_t ModbusSlave_HandleRequest(const uint8_t *request, uint16_t requestLen, uint8_t *response, uint16_t responseMax);

#endif /* MODBUS_SLAVE_H */
