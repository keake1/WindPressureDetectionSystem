#include <STC8H.H>
#include "protocol.h"
#include "uart.h"
#include "gpio.h"
#include "aht30.h"

/*
 * Modbus CRC16 计算
 * 初始值 0xFFFF，多项式 0xA001（反转多项式 0x8005）
 * 结果低字节在前
 */
unsigned int CRC16(unsigned char *buf, unsigned char len)
{
    unsigned int crc = 0xFFFF;
    unsigned char i, j;
    for(i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for(j = 0; j < 8; j++)
        {
            if(crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else             crc >>= 1;
        }
    }
    return crc;   /* 低字节=crc&0xFF，高字节=crc>>8 */
}

/*
 * 串口协议处理函数
 * 在主循环中轮询调用，收到完整帧后进行解析和回复
 */
void Protocol_Process(void)
{
    unsigned int  crc_calc, crc_recv;
    unsigned char tx_buf[9];
    unsigned int  crc_tx;

    /* 未收到数据 */
    if(!g_uart_rx_flag) return;

    // /* ---- 调试回显：收到什么就原样发回去 ---- */
    // Uart1_SendBuffer(g_uart_rx_buf, g_uart_rx_len);
    // /* ---------------------------------------- */

    /* 判断帧长度是否正确
     * 注：串口采用超时判帧，g_uart_rx_flag=1时数据已全部收完
     * 若长度不对说明帧不完整或有干扰，丢弃该帧等待下一帧 */
    if(g_uart_rx_len != PROTO_RX_LEN)
    {
        g_uart_rx_flag = 0;
        g_uart_rx_len  = 0;
        return;
    }

    /* 判断地址是否匹配 */
    if(g_uart_rx_buf[0] != g_device_addr)
    {
        g_uart_rx_flag = 0;
        g_uart_rx_len  = 0;
        return;
    }

    /* 验证 CRC16（校验范围：[0]~[5]，共6字节） */
    crc_calc = CRC16(g_uart_rx_buf, 6);
    crc_recv = (unsigned int)g_uart_rx_buf[7] << 8 | g_uart_rx_buf[6]; /* 低字节在前 */
    if(crc_calc != crc_recv)
    {
        g_uart_rx_flag = 0;
        g_uart_rx_len  = 0;
        return;
    }

    /* 根据功能码处理 */
    switch(g_uart_rx_buf[1])
    {
        case PROTO_FUNC_03:
        {
            /*
             * 应答帧（9字节）：
             *   [0]    设备地址
             *   [1]    功能码 0x03
             *   [2]    数据字节数 = 4
             *   [3]    温度高字节（×10，有符号）
             *   [4]    温度低字节
             *   [5]    湿度高字节（×10）
             *   [6]    湿度低字节
             *   [7]    CRC低字节
             *   [8]    CRC高字节
             */
            tx_buf[0] = g_device_addr;
            tx_buf[1] = PROTO_FUNC_03;
            tx_buf[2] = 5;
            tx_buf[3] = 0x05;//型号
            tx_buf[4] = (unsigned char)((g_aht30.temperature >> 8) & 0xFF);
            tx_buf[5] = (unsigned char)(g_aht30.temperature & 0xFF);
            tx_buf[6] = (unsigned char)((g_aht30.humidity >> 8) & 0xFF);
            tx_buf[7] = (unsigned char)(g_aht30.humidity & 0xFF);

            crc_tx    = CRC16(tx_buf, 8);
            tx_buf[8] = (unsigned char)(crc_tx & 0xFF);        /* CRC低字节 */
            tx_buf[9] = (unsigned char)((crc_tx >> 8) & 0xFF); /* CRC高字节 */

            Uart1_SendBuffer(tx_buf, 10);
            break;
        }
        case PROTO_FUNC_06:
        {
            /*
             * 0x06 写控制命令
             * [2~3] 寄存器地址：0x00 0x04
             * [4~5] 写入值：0x00 0x01=点亮红灯  0x00 0x00=熄灭红灯
             * 应答：原帧回显（8字节）
             */
            unsigned int reg_addr = (unsigned int)g_uart_rx_buf[2] << 8 | g_uart_rx_buf[3];
            unsigned int reg_val  = (unsigned int)g_uart_rx_buf[4] << 8 | g_uart_rx_buf[5];

            if(reg_addr == 0x0004)
            {
                if(reg_val == 0x0001)       LED_RED_ON();    /* 点亮红灯 */
                else if(reg_val == 0x0000)  LED_RED_OFF();   /* 熄灭红灯 */
            }
            break;
        }
        default:
            break;
    }

    /* 清空接收缓冲区 */
    g_uart_rx_flag = 0;
    g_uart_rx_len  = 0;
}
