#include "addr.h"

void addr_init(void)
{
    /* P02, P03, P12, P13, P14 and P15: active-low address switches. */
    P_SW2 |= 0x80;
    P0PU |= (0x01 << 2) | (0x01 << 3);
    P1PU |= (0x01 << 2) | (0x01 << 3) | (0x01 << 4) | (0x01 << 5);
    P_SW2 &= ~0x80;

    P02 = 1;
    P03 = 1;
    P12 = 1;
    P13 = 1;
    P14 = 1;
    P15 = 1;
}

unsigned char addr_read(void)
{
    unsigned char sensor_address = 0;

    sensor_address |= ((unsigned char)(!P15)) << 5;
    sensor_address |= ((unsigned char)(!P14)) << 4;
    sensor_address |= ((unsigned char)(!P13)) << 3;
    sensor_address |= ((unsigned char)(!P12)) << 2;
    sensor_address |= ((unsigned char)(!P03)) << 1;
    sensor_address |= ((unsigned char)(!P02));

    return sensor_address;
}
