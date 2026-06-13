#ifndef __IIC_H__
#define __IIC_H__

#include <STC8H.H>
#include <intrins.h>

/* -------------------------------------------------------
 * 软件模拟 I²C
 *   SCL -> P3.2  准双向口
 *   SDA -> P3.3  准双向口
 * 外部需接 4.7kΩ 上拉电阻到 VCC
 * ------------------------------------------------------- */
sbit IIC_SCL = P3^2;
sbit IIC_SDA = P3^3;

/* 函数声明 */
void          IIC_Init(void);
void          IIC_Start(void);
void          IIC_Stop(void);
void          IIC_SendByte(unsigned char dat);
unsigned char IIC_RecvByte(unsigned char ack);   /* ack=1:发ACK  ack=0:发NACK */
unsigned char IIC_WaitAck(void);                 /* 返回0:收到ACK  1:NACK     */

#endif /* __IIC_H__ */
