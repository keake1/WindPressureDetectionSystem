#ifndef __GPIO_H__
#define __GPIO_H__

#include <STC8H.H>

/* -------------------------------------------------------
 * LED 引脚定义
 *   LED_GREEN -> P2.2  低电平点亮（绿色）
 *   LED_RED   -> P3.4  低电平点亮（红色）
 * ------------------------------------------------------- */
sbit LED_GREEN = P2^2;
sbit LED_RED   = P3^4;

/* 操作宏 */
#define LED_GREEN_ON()   (LED_GREEN = 0)
#define LED_GREEN_OFF()  (LED_GREEN = 1)
#define LED_RED_ON()     (LED_RED   = 0)
#define LED_RED_OFF()    (LED_RED   = 1)

/* -------------------------------------------------------
 * 地址拨码引脚定义（高阻输入模式）
 *   P0.2 -> bit5 最高位
 *   P0.3 -> bit4
 *   P1.2 -> bit3
 *   P1.3 -> bit2
 *   P1.4 -> bit1
 *   P1.5 -> bit0 最低位
 *   地址范围：0 ~ 63
 * ------------------------------------------------------- */
sbit ADDR_BIT5 = P0^2;
sbit ADDR_BIT4 = P0^3;
sbit ADDR_BIT3 = P1^2;
sbit ADDR_BIT2 = P1^3;
sbit ADDR_BIT1 = P1^4;
sbit ADDR_BIT0 = P1^5;

extern unsigned char g_device_addr;   /* 设备地址（0~63） */

/* 初始化函数声明 */
void GPIO_Init(void);

/* 电平翻转函数声明 */
void LED_GREEN_Toggle(void);   /* 翻转绿色 LED（P2.2） */

/* 地址读取函数声明 */
unsigned char Read_DeviceAddr(void);   /* 读取拨码地址，返回 0~63 */

#endif /* __GPIO_H__ */

