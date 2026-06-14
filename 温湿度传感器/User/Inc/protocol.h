#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <STC8H.H>

/*
 * 通信协议说明
 *
 * 【接收帧 0x03】主机 → 传感器（8字节）
 *   [0]      设备地址
 *   [1]      功能码  0x03
 *   [2~5]    固定数据 0x00 0x00 0x00 0x01
 *   [6~7]    CRC16校验（低字节在前）
 *
 * 【应答帧 0x03】传感器 → 主机（9字节）
 *   [0]      设备地址
 *   [1]      功能码  0x03
 *   [2]      数据字节数 = 4
 *   [3~4]    温度（×10，高字节在前，有符号）
 *   [5~6]    湿度（×10，高字节在前）
 *   [7~8]    CRC16校验（低字节在前）
 *
 * 【接收帧 0x06】主机 → 传感器（8字节）
 *   [0]      设备地址
 *   [1]      功能码  0x06
 *   [2~3]    寄存器地址 0x00 0x04
 *   [4~5]    写入值：0x00 0x01=点亮红灯  0x00 0x00=熄灭红灯
 *   [6~7]    CRC16校验（低字节在前）
 *
 * 【应答帧 0x06】传感器 → 主机（原帧回显，8字节）
 *   与接收帧完全相同
 */

#define PROTO_RX_LEN   8    /* 接收帧长度 */
#define PROTO_FUNC_03  0x03 /* 读温湿度功能码 */
#define PROTO_FUNC_06  0x06 /* 写控制功能码   */

/* 函数声明 */
unsigned int  CRC16(unsigned char *buf, unsigned char len); /* Modbus CRC16 */
void          Protocol_Process(void);   /* 串口数据处理，在主循环中调用 */

#endif /* __PROTOCOL_H__ */
