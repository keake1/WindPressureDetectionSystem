#include "uart.h"
#include "config.h"
#include "stc8h.h"

/*
 * UART setup is written for STC8H-style UART1/UART2 SFRs.
 * If the real board clock differs from FOSC_HZ, update config.h first.
 */

static uint16_t Uart_Reload(uint32_t baud)
{
    return (uint16_t)(65536UL - (FOSC_HZ / 4UL / baud));
}

void Uart_Init(void)
{
    uint16_t reload1 = Uart_Reload(PRESSURE_BAUD);
    uint16_t reload2 = Uart_Reload(CONTROLLER_BAUD);

    P_SW1 &= 0x3FU; /* UART1 on P30/RXD and P31/TXD. */
    SCON = SCON_MODE1 | SCON_REN;
    S2CON = S2CON_REN;

    /*
     * Timer2 drives UART1 baud on common STC8H configurations.
     * UART2 baud generator differs between STC subfamilies; the reload2
     * calculation is kept here to make the intended value explicit.
     */
    (void)reload2;
    T2H = (uint8_t)(reload1 >> 8);
    T2L = (uint8_t)reload1;
    AUXR = 0x15U; /* Official STC setup: T2 1T + T2 run + UART1 uses Timer2. */

    SCON &= (uint8_t)~(SCON_RI | SCON_TI);
    S2CON &= (uint8_t)~(S2CON_RI | S2CON_TI);
}

void Uart1_SendByte(uint8_t value)
{
    SBUF = value;
    while ((SCON & SCON_TI) == 0U) {
    }
    SCON &= (uint8_t)~SCON_TI;
}

uint8_t Uart1_ReadByte(uint8_t *value)
{
    if ((SCON & SCON_RI) == 0U) {
        return 0U;
    }

    *value = SBUF;
    SCON &= (uint8_t)~SCON_RI;
    return 1U;
}

void Uart2_SendByte(uint8_t value)
{
    S2BUF = value;
    while ((S2CON & S2CON_TI) == 0U) {
    }
    S2CON &= (uint8_t)~S2CON_TI;
}

uint8_t Uart2_ReadByte(uint8_t *value)
{
    if ((S2CON & S2CON_RI) == 0U) {
        return 0U;
    }

    S2CON &= (uint8_t)~S2CON_RI;
    *value = S2BUF;
    return 1U;
}

void Uart1_Send(const uint8_t *data, uint8_t len)
{
    uint8_t i;
    for (i = 0U; i < len; ++i) {
        Uart1_SendByte(data[i]);
    }
}

void Uart2_Send(const uint8_t *data, uint8_t len)
{
    uint8_t i;
    for (i = 0U; i < len; ++i) {
        Uart2_SendByte(data[i]);
    }
}
