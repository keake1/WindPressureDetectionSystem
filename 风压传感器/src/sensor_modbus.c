#include "sensor_modbus.h"
#include "board.h"
#include "config.h"
#include "crc16.h"
#include "uart.h"

static uint8_t g_rx[8];
static uint8_t g_rx_len;

static void SensorModbus_ReplyPressure(uint8_t addr, uint16_t pressure)
{
    uint8_t tx[8];

    tx[0] = addr;
    tx[1] = 0x03U;
    tx[2] = 0x03U;
    tx[3] = SENSOR_TYPE_WIND_PRESSURE;
    tx[4] = (uint8_t)(pressure >> 8);
    tx[5] = (uint8_t)pressure;
    Crc16_Append(tx, 6U);
    Uart2_Send(tx, sizeof(tx));
}

static void SensorModbus_HandleFrame(uint8_t my_addr, uint16_t pressure)
{
    if (g_rx[0] != my_addr) {
        return;
    }

    if (Crc16_Check(g_rx, sizeof(g_rx)) == 0U) {
        return;
    }

    if ((g_rx[1] == 0x03U) &&
        (g_rx[2] == 0x00U) &&
        (g_rx[3] == 0x00U) &&
        (g_rx[4] == 0x00U) &&
        (g_rx[5] == 0x01U)) {
        SensorModbus_ReplyPressure(my_addr, pressure);
        return;
    }

    if ((g_rx[1] == 0x06U) &&
        (g_rx[2] == 0x00U) &&
        (g_rx[3] == 0x04U) &&
        (g_rx[4] == 0x00U)) {
        if (g_rx[5] == 0x01U) {
            Board_RedLedSet(1U);
        } else if (g_rx[5] == 0x00U) {
            Board_RedLedSet(0U);
        }
    }
}

void SensorModbus_Process(uint8_t my_addr, uint16_t pressure)
{
    uint8_t b;

    while (Uart2_ReadByte(&b) != 0U) {
        g_rx[g_rx_len++] = b;
        if (g_rx_len >= sizeof(g_rx)) {
            SensorModbus_HandleFrame(my_addr, pressure);
            g_rx_len = 0U;
        }
    }
}
