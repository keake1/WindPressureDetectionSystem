#include "uart1.h"

volatile bit B_TX1_Busy;

void TX1_data(u8 tx1dat)
{
    B_TX1_Busy = 1;
    SBUF = tx1dat;
    while (B_TX1_Busy);
}

void UART1_config(u8 UART_IOSEL, u32 FOSC, u32 BAUD1)
{
    TR1 = 0;
    AUXR &= ~0x01;      /* UART1 baud-rate generator: Timer1. */
    AUXR |= (1 << 6);   /* Timer1 1T mode. */
    TMOD &= ~(1 << 6);  /* Timer mode. */
    TMOD &= ~0x30;      /* Timer1 16-bit auto reload. */
    TH1 = (u8)((65536UL - (FOSC / 4) / BAUD1) / 256);
    TL1 = (u8)((65536UL - (FOSC / 4) / BAUD1) % 256);
    ET1 = 0;
    TR1 = 1;

    SCON = (SCON & 0x3F) | 0x40; /* UART mode 1, 8N1. */
    PS = 1;
    IPH |= (0x01 << 4);
    ES = 1;
    REN = 1;
    P_SW1 = (P_SW1 & 0x3F) | UART_IOSEL;

    RI = 0;
    TI = 0;
    B_TX1_Busy = 0;
}

void UART1_sendArray(unsigned char count, unsigned char *buffer)
{
    unsigned char i;

    for (i = 0; i < count; i++) {
        TX1_data(buffer[i]);
    }
}
