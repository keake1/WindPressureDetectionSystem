#ifndef __AHT30_H__
#define __AHT30_H__

#include <STC8H.H>

/* AHT30 I²C 从机地址 */
#define AHT30_ADDR        0x38

/* 读写地址（7位地址左移1位 + 读写位） */
#define AHT30_ADDR_WRITE  (AHT30_ADDR << 1)        /* 0x70 */
#define AHT30_ADDR_READ   ((AHT30_ADDR << 1) | 1)  /* 0x71 */

/* 温湿度数据结构（放大10倍，保留1位小数） */
typedef struct
{
    int  temperature;   /* 温度 × 10，如 253 表示 25.3℃ */
    unsigned int humidity;     /* 湿度 × 10，如 456 表示 45.6% RH */
} AHT30_Data_t;

extern AHT30_Data_t g_aht30;   /* 全局温湿度数据 */

/* 函数声明 */
void          AHT30_Init(void);          /* 初始化 AHT30          */
unsigned char AHT30_Measure(void);       /* 触发测量，返回0=成功  */
unsigned char AHT30_Read(void);          /* 读取并解析数据        */

#endif /* __AHT30_H__ */
