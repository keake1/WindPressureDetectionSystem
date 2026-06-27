#include <STC8H.H>
#include "gpio.h"

/*
 * STC8H GPIO 模式配置（PxM1 / PxM0）：
 *   M1=0  M0=0  ->  准双向口（默认）
 *   M1=0  M0=1  ->  推挽输出  ← LED 使用
 *   M1=1  M0=0  ->  高阻输入  ← 地址拨码使用
 *   M1=1  M0=1  ->  开漏输出
 *
 * P2.2 推挽输出：P2M1 &= ~0x04; P2M0 |=  0x04;
 * P3.4 推挽输出：P3M1 &= ~0x10; P3M0 |=  0x10;
 * P0.2 高阻输入：P0M1 |=  0x04; P0M0 &= ~0x04;
 * P0.3 高阻输入：P0M1 |=  0x08; P0M0 &= ~0x08;
 * P1.2 高阻输入：P1M1 |=  0x04; P1M0 &= ~0x04;
 * P1.3 高阻输入：P1M1 |=  0x08; P1M0 &= ~0x08;
 * P1.4 高阻输入：P1M1 |=  0x10; P1M0 &= ~0x10;
 * P1.5 高阻输入：P1M1 |=  0x20; P1M0 &= ~0x20;
 *
 * 地址位序（与其他传感器一致）：
 *   P1.5 = bit5(最高位), P1.4 = bit4, P1.3 = bit3,
 *   P1.2 = bit2, P0.3 = bit1, P0.2 = bit0(最低位)
 */

unsigned char g_device_addr = 0;   /* 设备地址全局变量 */

void GPIO_Init(void)
{
    /* ---------- P2.2：绿色 LED，推挽输出，初始熄灭（高电平） ---------- */
    P2M1 &= ~0x04;
    P2M0 |=  0x04;
    LED_GREEN_OFF();

    /* ---------- P3.4：红色 LED，推挽输出，初始熄灭（高电平） ---------- */
    P3M1 &= ~0x10;
    P3M0 |=  0x10;
    LED_RED_OFF();

    /* ---------- P0.2、P0.3：地址拨码 bit5、bit4，高阻输入 ---------- */
    P0M1 |=  0x0C;   /* P0M1[3:2] = 1 */
    P0M0 &= ~0x0C;   /* P0M0[3:2] = 0 */

    /* ---------- P1.2~P1.5：地址拨码 bit3~bit0，高阻输入 ---------- */
    P1M1 |=  0x3C;   /* P1M1[5:2] = 1 */
    P1M0 &= ~0x3C;   /* P1M0[5:2] = 0 */

    /* ---------- 使能内部上拉电阻（需先解锁扩展寄存器） ---------- */
    P_SW2 |= 0x80;   /* EAXFR=1，允许访问 XFR 扩展寄存器 */
    P0PU  |= 0x0C;   /* P0.2、P0.3 上拉使能 */
    P1PU  |= 0x3C;   /* P1.2、P1.3、P1.4、P1.5 上拉使能 */
    P_SW2 &= ~0x80;  /* EAXFR=0，关闭扩展寄存器访问 */
}

/* 翻转绿色 LED（P2.2）电平 */
void LED_GREEN_Toggle(void)
{
    P2 ^= 0x04;   /* 对 P2.2 位取反 */
}

/*
 * 读取拨码地址
 * 引脚低电平 -> 对应位为 1，高电平 -> 对应位为 0
 * P1.5 最高位，P0.2 最低位
 * 返回值：0 ~ 63
 */
unsigned char Read_DeviceAddr(void)
{
    unsigned char addr = 0;

    if(!ADDR_BIT5) addr |= 0x20;   /* P1.5 -> bit5 */
    if(!ADDR_BIT4) addr |= 0x10;   /* P1.4 -> bit4 */
    if(!ADDR_BIT3) addr |= 0x08;   /* P1.3 -> bit3 */
    if(!ADDR_BIT2) addr |= 0x04;   /* P1.2 -> bit2 */
    if(!ADDR_BIT1) addr |= 0x02;   /* P0.3 -> bit1 */
    if(!ADDR_BIT0) addr |= 0x01;   /* P0.2 -> bit0 */

    return addr;
}
