#include "board.h"
#include "config.h"
#include "display.h"
#include "pressure.h"
#include "sensor_modbus.h"
#include "uart.h"

void main(void)
{
    uint16_t heartbeat_tick = 0U;

    Board_Init();
    Uart_Init();
    Pressure_Init();
    Display_Init();
    Board_GreenLedToggle();

    for (;;) {
        Pressure_ProcessRx();
        Pressure_ProcessRx();
        Pressure_ProcessRx();
        SensorModbus_Process(Board_ReadAddress(), Pressure_GetValue());
        Display_SetValue(Pressure_GetValue());
        Display_ScanOnce();

        heartbeat_tick++;
        if (heartbeat_tick >= HEARTBEAT_INTERVAL_MS) {
            heartbeat_tick = 0U;
            Board_GreenLedToggle();
        }
    }
}
