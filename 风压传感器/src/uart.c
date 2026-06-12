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
    /*
     * 波特率发生器分配：
     *   UART1（气压模块，115200）→ Timer 1（8-bit auto-reload, SMOD=1）
     *   UART2（Modbus，9600）      → Timer 2（1T mode, 16-bit reload）
     *
     * STC8H1K28 的 UART2 固定使用 Timer 2 作为波特率发生器，
     * 因此 UART1 必须让出 Timer 2，改由 Timer 1 驱动。
     */
    uint16_t reload2 = Uart_Reload(CONTROLLER_BAUD);  // 9600 重装值

    P_SW1 &= 0x3FU; /* UART1 on P30/RXD and P31/TXD. */
    SCON = SCON_MODE1 | SCON_REN;
    S2CON = S2CON_REN;

    /* ---- Timer 1 → UART1 @ 115200 ---- */
    TMOD = (TMOD & 0x0FU) | 0x20U;            // Timer 1, 8-bit auto-reload (mode 2)
    TH1 = (uint8_t)(256U - (FOSC_HZ / 32U / PRESSURE_BAUD));
    PCON |= 0x80U;                             // SMOD = 1（倍频）
    TL1 = TH1;                                 // 初值与重装值一致
    TCON |= 0x40U;                             // TR1 = 1：启动 Timer 1

    /* ---- Timer 2 → UART2 @ 9600 ---- */
    T2H = (uint8_t)(reload2 >> 8);             // 0xFE
    T2L = (uint8_t)reload2;                     // 0xE0
    AUXR = (AUXR & (uint8_t)~0x01U) | 0x14U;   // S1ST2=0（UART1→T1）, T2R=1, T2x12=1

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
