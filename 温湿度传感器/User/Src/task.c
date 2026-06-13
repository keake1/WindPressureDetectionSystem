#include <STC8H.H>
#include "task.h"
#include "gpio.h"
#include "uart.h"
#include "aht30.h"
#include "protocol.h"

/* ---------- 1ms 任务：串口超时判断 + 协议处理（在定时器中断中调用） ---------- */
void Task_1ms(void)
{
    Uart1_RxTimeoutTick();   /* 超时计数，5ms无新数据则认为一帧完成 */
    Protocol_Process();      /* 帧完成后处理协议 */
}

/* ---------- 10ms 任务 ---------- */
void Task_10ms(void)
{
    /* TODO: 放需要每 10ms 执行一次的代码 */
}

/* ---------- 50ms 任务 ---------- */
void Task_50ms(void)
{
    /* TODO: 放需要每 50ms 执行一次的代码 */
}

/* ---------- 100ms 任务：读取拨码地址并通过串口上报 ---------- */
void Task_100ms(void)
{
    g_device_addr = Read_DeviceAddr();   /* 每 100ms 更新一次设备地址 */
}

/* ---------- 500ms 任务：翻转绿色 LED + 触发 AHT30 测量 ---------- */
void Task_500ms(void)
{
    LED_GREEN_Toggle();
    AHT30_Measure();    /* 触发测量，500ms 后由 Task_1000ms 读取结果 */
}

/* ---------- 1000ms 任务：读取 AHT30 数据 ---------- */
void Task_1000ms(void)
{
    AHT30_Read();          /* 读取并解析数据，更新全局变量 g_aht30 */
}
