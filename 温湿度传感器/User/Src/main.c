#include <STC8H.H>
#include "gpio.h"
#include "timer.h"
#include "task.h"
#include "uart.h"
#include "iic.h"
#include "aht30.h"
#include "protocol.h"

void Delay1ms(void);
void DelayMs(unsigned int ms);


#define FOSC  24000000UL   /* 24 MHz，与 STC-ISP 中设置保持一致 */

void main()
{
    GPIO_Init();    /* 初始化 GPIO */
    Uart1_Init();   /* 初始化串口1，9600bps */
    IIC_Init();     /* 初始化软件模拟 I²C */
    DelayMs(100);   /* 等待 AHT30 上电稳定（手册要求 ≥100ms） */
    AHT30_Init();   /* 初始化 AHT30 温湿度传感器 */
    Timer0_Init();  /* 初始化定时器0，1ms 中断 */
    EA = 1;         /* 开总中断 */
    DelayMs(200);
    while(1)
    {
        /* 10ms 任务 */
        if(cnt_10ms >= 10)
        {
            cnt_10ms = 0;
            Task_10ms();
        }

        /* 50ms 任务 */
        if(cnt_50ms >= 50)
        {
            cnt_50ms = 0;
            Task_50ms();
        }

        /* 100ms 任务 */
        if(cnt_100ms >= 100)
        {
            cnt_100ms = 0;
            Task_100ms();
        }

        /* 500ms 任务 */
        if(cnt_500ms >= 500)
        {
            cnt_500ms = 0;
            Task_500ms();
        }

        /* 1000ms 任务 */
        if(cnt_1000ms >= 1000)
        {
            cnt_1000ms = 0;
            Task_1000ms();
        }
    }
}

void Delay1ms(void)	//@24.000MHz
{
	unsigned char data i, j;

	_nop_();
	i = 32;
	j = 40;
	do
	{
		while (--j);
	} while (--i);
}

/* 通用毫秒延时：调用 Delay1ms() ms 次，最大延时 65535ms */
void DelayMs(unsigned int ms)
{
    while (ms--)
    {
        Delay1ms();
    }
}

