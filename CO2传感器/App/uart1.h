#ifndef _UART1_H
#define _UART1_H

#include "STC8H.h"

#define UART1_P30P31  0x00
#define UART1_P36P37  0x40
#define UART1_P16P17  0x80

extern volatile bit B_TX1_Busy;

void UART1_config(u8 UART_IOSEL, u32 FOSC, u32 BAUD1);
void TX1_data(u8 tx1dat);
void UART1_sendArray(unsigned char count, unsigned char *buffer);

#endif
