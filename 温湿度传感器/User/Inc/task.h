#ifndef __TASK_H__
#define __TASK_H__

/* 各任务函数声明 */
void Task_1ms(void);     /*   1ms 任务（在定时器中断中直接调用） */
void Task_10ms(void);    /*  10ms 任务 */
void Task_50ms(void);    /*  50ms 任务 */
void Task_100ms(void);   /* 100ms 任务 */
void Task_500ms(void);   /* 500ms 任务 */
void Task_1000ms(void);  /*   1s  任务 */

#endif /* __TASK_H__ */
