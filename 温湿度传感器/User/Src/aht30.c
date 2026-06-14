#include <STC8H.H>
#include "aht30.h"
#include "iic.h"

AHT30_Data_t g_aht30 = {0, 0};   /* 全局温湿度数据 */

/*
 * AHT30 初始化
 * 上电后等待100ms，发送初始化命令 0xBE 0x08 0x00
 */
void AHT30_Init(void)
{
    IIC_Start();
    IIC_SendByte(AHT30_ADDR_WRITE);
    IIC_WaitAck();
    IIC_SendByte(0xBE);         /* 初始化命令 */
    IIC_WaitAck();
    IIC_SendByte(0x08);
    IIC_WaitAck();
    IIC_SendByte(0x00);
    IIC_WaitAck();
    IIC_Stop();
}

/*
 * 触发一次测量
 * 发送测量命令 0xAC 0x33 0x00
 * 返回 0：发送成功；返回 1：ACK 异常
 */
unsigned char AHT30_Measure(void)
{
    unsigned char ack;

    IIC_Start();
    IIC_SendByte(AHT30_ADDR_WRITE);
    ack = IIC_WaitAck();
    if(ack) { IIC_Stop(); return 1; }

    IIC_SendByte(0xAC);         /* 触发测量命令 */
    IIC_WaitAck();
    IIC_SendByte(0x33);
    IIC_WaitAck();
    IIC_SendByte(0x00);
    IIC_WaitAck();
    IIC_Stop();

    return 0;
}

/*
 * 读取7字节原始数据并解析（第7字节为CRC，AHT30返回7字节）
 * 需在 AHT30_Measure() 后等待约 80ms 再调用
 * 返回 0：成功；返回 1：忙/未校准/ACK异常
 *
 * 原始数据格式（7字节）：
 *   Byte0       : 状态字节，bit7=Busy，bit3~4必须=11(已校准)
 *   Byte1~Byte5 : 温湿度数据
 *   Byte6       : CRC校验
 *
 *   湿度原始值 = (Byte1<<12)|(Byte2<<4)|(Byte3>>4)        [20bit]
 *   温度原始值 = ((Byte3&0x0F)<<16)|(Byte4<<8)|Byte5      [20bit]
 *
 * 整数计算（×10倍）：
 *   湿度×10 = 原始值 × 1000 / 1048576
 *   温度×10 = 原始值 × 2000 / 1048576 - 500
 */
unsigned char AHT30_Read(void)
{
    unsigned char buf[7];
    unsigned long raw_humi, raw_temp;

    IIC_Start();
    IIC_SendByte(AHT30_ADDR_READ);
    if(IIC_WaitAck()) { IIC_Stop(); return 1; }

    buf[0] = IIC_RecvByte(1);   /* 状态字节 */
    buf[1] = IIC_RecvByte(1);
    buf[2] = IIC_RecvByte(1);
    buf[3] = IIC_RecvByte(1);
    buf[4] = IIC_RecvByte(1);
    buf[5] = IIC_RecvByte(1);
    buf[6] = IIC_RecvByte(0);   /* CRC字节，最后一字节发 NACK */
    IIC_Stop();

    /* 检查 Busy 位（bit7）和校准标志（bit3~4 必须为 11） */
    if((buf[0] & 0x98) != 0x18) return 1;

    /* 解析湿度原始值（20bit） */
    raw_humi  = (unsigned long)buf[1] << 12;
    raw_humi |= (unsigned long)buf[2] << 4;
    raw_humi |= (buf[3] >> 4);

    /* 解析温度原始值（20bit） */
    raw_temp  = (unsigned long)(buf[3] & 0x0F) << 16;
    raw_temp |= (unsigned long)buf[4] << 8;
    raw_temp |= buf[5];

    /* 转换（整数，×10倍保留1位小数，避免浮点运算） */
    g_aht30.humidity    = (unsigned int)(raw_humi * 1000 / 1048576);
    g_aht30.temperature = (int)(raw_temp * 2000 / 1048576) - 500;

    return 0;
}
