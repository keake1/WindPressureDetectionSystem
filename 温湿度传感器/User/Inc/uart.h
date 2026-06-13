#ifndef __UART_H__
#define __UART_H__

#include <STC8H.H>

/* -------------------------------------------------------
 * 串口1 引脚
 *   RX -> P3.0
 *   TX -> P3.1
 * 波特率：9600 bps @ 24MHz
 *   使用定时器2作为波特率发生器（1T模式）
 *   重载值 = 65536 - 24000000 / (4 * 9600) = 0xFD8F
 * ------------------------------------------------------- */

/* 接收缓冲区大小 */
#define UART_RX_BUF_SIZE   64

extern unsigned char g_uart_rx_buf[UART_RX_BUF_SIZE];  /* 接收缓冲区           */
extern unsigned char g_uart_rx_len;                     /* 已接收字节数         */
extern unsigned char g_uart_rx_flag;                    /* 帧接收完成标志       */
extern unsigned char g_uart_rx_timeout;                 /* 帧超时计数器（ms）   */

/* 帧超时阈值：9600bps下3.5字符时间≈4ms，取5ms保险 */
#define UART_RX_TIMEOUT_MS   5

/* 函数声明 */
void Uart1_Init(void);
void Uart1_SendByte(unsigned char dat);
void Uart1_SendBuffer(unsigned char *buf, unsigned char len);
void Uart1_RxTimeoutTick(void);   /* 每1ms调用一次，处理帧超时 */

#endif /* __UART_H__ */
