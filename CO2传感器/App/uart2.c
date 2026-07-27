#include "uart2.h"

void UART2_config(u8 UART_IOSEL, u32 FOSC, u32 BAUD2)
{
    AUXR &= ~(1 << 4);  /* Stop Timer2. */
    AUXR &= ~(1 << 3);  /* Timer mode. */
    AUXR |= (1 << 2);   /* Timer2 1T mode. */
    T2H = (65536UL - (FOSC / 4) / BAUD2) / 256;
    T2L = (65536UL - (FOSC / 4) / BAUD2) % 256;

    IE2 &= ~(1 << 2);  /* Timer2 is only a baud-rate generator. */
    AUXR |= (1 << 4);   /* Start Timer2. */
    S2CON &= ~(1 << 7); /* UART2 8N1. */
    S2CON |= (1 << 4);  /* Enable receiver. */

    IP2 |= 1;
    IP2H |= 1;
    IE2 |= 1;
    P_SW2 = (P_SW2 & ~0x01) | UART_IOSEL;

    S2CON &= ~0x03;     /* Clear receive/transmit flags. */
}
