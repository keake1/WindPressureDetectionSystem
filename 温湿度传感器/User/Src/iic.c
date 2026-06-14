#include <STC8H.H>
#include <intrins.h>
#include "iic.h"

/*
 * 软件模拟 I²C
 * SCL -> P3.2，SDA -> P3.3，准双向口模式
 * 参考官方 AHT2x 例程时序：SCL高持续4us，低持续2us
 */

/* 微秒级延时（@24MHz，约1us/次） */
static void iic_delay(unsigned char us)
{
    while(us--)
    {
        _nop_(); _nop_(); _nop_(); _nop_();
        _nop_(); _nop_(); _nop_(); _nop_();
        _nop_(); _nop_(); _nop_(); _nop_();
    }
}

/* IIC 初始化：P3.2/P3.3 配置为准双向口，释放总线 */
void IIC_Init(void)
{
    /* 准双向口：M1=0, M0=0（STC8H上电默认即为准双向口，此处显式配置） */
    P3M1 &= ~0x0C;
    P3M0 &= ~0x0C;

    IIC_SCL = 1;
    IIC_SDA = 1;
    iic_delay(4);
}

/* START：SCL高时，SDA下降沿 */
void IIC_Start(void)
{
    IIC_SCL = 1;
    IIC_SDA = 1;
    iic_delay(2);
    IIC_SDA = 0;   /* SDA下降沿 */
    iic_delay(4);
    IIC_SCL = 0;   /* 拉低SCL，准备发数据 */
    iic_delay(2);
}

/* STOP：SCL高时，SDA上升沿 */
void IIC_Stop(void)
{
    IIC_SCL = 0;
    IIC_SDA = 0;
    iic_delay(2);
    IIC_SCL = 1;
    iic_delay(4);
    IIC_SDA = 1;   /* SDA上升沿 */
    iic_delay(4);
}

/* 发送1字节，高位先发 */
void IIC_SendByte(unsigned char dat)
{
    unsigned char i;
    for(i = 0; i < 8; i++)
    {
        IIC_SCL = 0;
        iic_delay(2);
        if(dat & 0x80) IIC_SDA = 1;
        else           IIC_SDA = 0;
        iic_delay(2);
        IIC_SCL = 1;
        iic_delay(4);
        dat <<= 1;
    }
    IIC_SCL = 0;   /* 第9个时钟前拉低，准备接收ACK */
    iic_delay(2);
}

/* 等待从机ACK：返回0=收到ACK，返回1=NACK/超时 */
unsigned char IIC_WaitAck(void)
{
    unsigned int timeout = 200;
    IIC_SDA = 1;       /* 释放SDA，由从机驱动 */
    iic_delay(2);
    IIC_SCL = 1;
    iic_delay(4);
    while(IIC_SDA)     /* 等待SDA被从机拉低 */
    {
        if(--timeout == 0)
        {
            IIC_SCL = 0;
            iic_delay(2);
            return 1;  /* 超时，NACK */
        }
    }
    IIC_SCL = 0;
    iic_delay(2);
    return 0;          /* ACK */
}

/* 接收1字节
 * ack=1：发ACK（告知从机继续发）
 * ack=0：发NACK（告知从机停止）
 */
unsigned char IIC_RecvByte(unsigned char ack)
{
    unsigned char i, dat = 0;
    IIC_SDA = 1;       /* 释放SDA准备接收 */
    for(i = 0; i < 8; i++)
    {
        IIC_SCL = 0;
        iic_delay(2);
        IIC_SCL = 1;
        iic_delay(4);
        dat <<= 1;
        if(IIC_SDA) dat |= 0x01;
    }
    IIC_SCL = 0;
    iic_delay(2);
    /* 发送ACK/NACK */
    if(ack) IIC_SDA = 0;   /* ACK:  拉低SDA */
    else    IIC_SDA = 1;   /* NACK: 释放SDA */
    iic_delay(2);
    IIC_SCL = 1;
    iic_delay(4);
    IIC_SCL = 0;
    IIC_SDA = 1;
    iic_delay(2);
    return dat;
}
