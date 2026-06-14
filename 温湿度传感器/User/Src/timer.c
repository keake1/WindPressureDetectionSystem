#include <STC8H.H>
#include "timer.h"
#include "task.h"

/*
 * 定时器0 配置（@24MHz，1T模式，模式0 16位自动重载）
 * 重载值 = 65536 - 24000000 / 1000 = 65536 - 24000 = 41536 = 0xA200
 * 定时周期：1ms
 */
#define TIMER0_RELOAD  0xA200

/* 各任务计数器定义 */
unsigned int cnt_10ms   = 0;
unsigned int cnt_50ms   = 0;
unsigned int cnt_100ms  = 0;
unsigned int cnt_500ms  = 0;
unsigned int cnt_1000ms = 0;

/* 定时器0初始化 */
void Timer0_Init(void)
{
    AUXR |= 0x80;               /* 定时器0 选择 1T 模式（不分频） */
    TMOD &= 0xF0;               /* 定时器0 工作在模式0（16位自动重载） */
    TL0 = TIMER0_RELOAD & 0xFF; /* 加载重载值低字节 */
    TH0 = TIMER0_RELOAD >> 8;   /* 加载重载值高字节 */
    TF0 = 0;                    /* 清除溢出标志 */
    ET0 = 1;                    /* 使能定时器0中断 */
    TR0 = 1;                    /* 启动定时器0 */
}

/* 定时器0中断服务函数（每 1ms 触发一次） */
void Timer0_ISR(void) interrupt 1
{
    Task_1ms();     /* 1ms 任务：直接在中断中执行，响应最及时 */
    cnt_10ms++;
    cnt_50ms++;
    cnt_100ms++;
    cnt_500ms++;
    cnt_1000ms++;
}
