#ifndef __TIMER_H__
#define __TIMER_H__

#include <STC8H.H>

/* 各任务计数器（在 timer.c 中定义，其他文件通过 extern 访问） */
extern unsigned int cnt_10ms;
extern unsigned int cnt_50ms;
extern unsigned int cnt_100ms;
extern unsigned int cnt_500ms;
extern unsigned int cnt_1000ms;

/* 定时器0初始化声明 */
void Timer0_Init(void);

#endif /* __TIMER_H__ */
