#ifndef _UART2_H
#define _UART2_H

#include "STC8H.h"

#define UART2_P10P11  0x00

void UART2_config(u8 UART_IOSEL, u32 FOSC, u32 BAUD2);

#endif
