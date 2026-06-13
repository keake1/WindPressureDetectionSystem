#include <STC8H.H>
#include "uart.h"

/*
 * 波特率发生器：定时器2，1T模式
 * 重载值 = 65536 - FOSC / (4 * 波特率)
 *        = 65536 - 24000000 / (4 * 9600)
 *        = 65536 - 625
 *        = 64911 = 0xFD8F
 */
#define UART1_BAUD_RELOAD   0xFD8F

/* 接收缓冲区及状态变量 */
unsigned char g_uart_rx_buf[UART_RX_BUF_SIZE] = {0};
unsigned char g_uart_rx_len     = 0;
unsigned char g_uart_rx_flag    = 0;
unsigned char g_uart_rx_timeout = 0;   /* 0=无数据/已处理，>0=倒计时中 */

/*
 * 串口1初始化
 * 模式1：8位UART，可变波特率
 * P3.0 -> RX，P3.1 -> TX（默认复用，无需切换）
 */
void Uart1_Init(void)
{
    /* --- 串口1 配置 --- */
    SCON = 0x50;            /* 模式1：8位UART，REN=1 允许接收 */

    /* --- 定时器2 作为波特率发生器（1T模式） --- */
    AUXR |= 0x04;           /* 定时器2 选择 1T 模式 */
    T2L   = UART1_BAUD_RELOAD & 0xFF;  /* 重载值低字节 */
    T2H   = UART1_BAUD_RELOAD >> 8;    /* 重载值高字节 */
    AUXR |= 0x15;           /* 串口1选择定时器2为波特率发生器，启动定时器2 */

    /* --- P3.0/P3.1 配置为准双向口（默认，串口使用） --- */
    P3M1 &= ~0x03;
    P3M0 &= ~0x03;

    /* --- 使能串口1中断 --- */
    ES = 1;
}

/* 发送单个字节（查询方式等待发送完成） */
void Uart1_SendByte(unsigned char dat)
{
    SBUF = dat;
    while(!TI);   /* 等待发送完成 */
    TI = 0;       /* 清除发送标志 */
}

/* 发送指定长度的数据缓冲区 */
void Uart1_SendBuffer(unsigned char *buf, unsigned char len)
{
    unsigned char i;
    for(i = 0; i < len; i++)
    {
        Uart1_SendByte(buf[i]);
    }
}

/* 串口1中断服务函数 */
void Uart1_ISR(void) interrupt 4
{
    if(RI)   /* 接收中断 */
    {
        RI = 0;
        if(g_uart_rx_len < UART_RX_BUF_SIZE)
        {
            g_uart_rx_buf[g_uart_rx_len++] = SBUF;  /* 存入缓冲区 */
        }
        g_uart_rx_timeout = UART_RX_TIMEOUT_MS;     /* 每收到一字节就重置超时计数器 */
    }
    if(TI)
    {
        TI = 0;
    }
}

/*
 * 帧超时判断（每 1ms 调用一次，在 Task_1ms 中调用）
 * 超时后认为一帧数据接收完毕，置 g_uart_rx_flag 通知协议层处理
 */
void Uart1_RxTimeoutTick(void)
{
    if(g_uart_rx_timeout == 0) return;   /* 无数据，不处理 */

    g_uart_rx_timeout--;

    if(g_uart_rx_timeout == 0 && g_uart_rx_len > 0)
    {
        g_uart_rx_flag = 1;   /* 超时，认为一帧接收完成 */
    }
}
