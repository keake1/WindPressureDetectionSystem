#ifndef __MODEBUSRTU_H__
#define __MODEBUSRTU_H__

#include "STC8H.h"

#define uint8_t  unsigned char
#define uint16_t unsigned int

extern uint8_t address;
extern volatile uint16_t Reg[6];

/* CO2 module: P3.2 PWM measurement. */
void CO2_PWM_Init(void);
void CO2_PWM_Timer0Overflow(void);
void CO2_PWM_Tick1ms(void);
uint8_t CO2_GetLatest(uint16_t *ppm);
uint8_t CO2_PWM_IsTimedOut(void);

/* Controller: UART1 Modbus RTU slave protocol. */
void ModBus_ReceiveByte(uint8_t data_byte);
void ModBus_Tick1ms(void);
void ModBus_Process(void);
uint16_t ModBus_CRC(volatile uint8_t *buffer, uint8_t length);

#endif
