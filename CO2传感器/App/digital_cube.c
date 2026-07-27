#include "digital_cube.h"

sbit P33_LED_DIG4 = P3^3;

static volatile unsigned char display_buf[4] = {0, 0, 0, 0};

static unsigned char code segment_table[] = {
    0x3F, /* 0 */
    0x06, /* 1 */
    0x5B, /* 2 */
    0x4F, /* 3 */
    0x66, /* 4 */
    0x6D, /* 5 */
    0x7D, /* 6 */
    0x07, /* 7 */
    0x7F, /* 8 */
    0x6F, /* 9 */
    0x77, /* A */
    0x7C, /* B */
    0x39, /* C */
    0x5E, /* D */
    0x79, /* E */
    0x71, /* F */
    0x76, /* H */
    0x38, /* L */
    0x37, /* n */
    0x3E, /* u */
    0x73, /* P */
    0x5C, /* o */
    0x40, /* - */
    0x00  /* blank */
};

static void Digital_Tube_AllOff(void)
{
    P23 = 1;
    P26 = 1;
    P01 = 1;
    P33_LED_DIG4 = 1;
}

void digital_cube_init(void)
{
    /* Digit-select outputs, active low. */
    P3M0 |= (0x01 << 3);
    P3M1 &= ~(0x01 << 3);
    P0M0 |= (0x01 << 1);
    P0M1 &= ~(0x01 << 1);
    P2M0 |= (0x01 << 6) | (0x01 << 3);
    P2M1 &= ~((0x01 << 6) | (0x01 << 3));

    /* Segment outputs, active high. */
    P2M0 |= (0x01 << 4) | (0x01 << 7) | (0x01 << 0) |
            (0x01 << 1) | (0x01 << 5);
    P2M1 &= ~((0x01 << 4) | (0x01 << 7) | (0x01 << 0) |
             (0x01 << 1) | (0x01 << 5));
    P3M0 |= (0x01 << 5) | (0x01 << 6) | (0x01 << 7);
    P3M1 &= ~((0x01 << 5) | (0x01 << 6) | (0x01 << 7));

    Digital_Tube_AllOff();
    digital_cube_ShowOne(23);
    P37 = 0;
}

void digital_cube_ShowOne(unsigned char num)
{
    unsigned char pattern;

    if (num >= sizeof(segment_table)) {
        num = 23;
    }
    pattern = segment_table[num];

    P24 = (pattern & 0x01) ? 1 : 0;
    P27 = (pattern & 0x02) ? 1 : 0;
    P36 = (pattern & 0x04) ? 1 : 0;
    P20 = (pattern & 0x08) ? 1 : 0;
    P21 = (pattern & 0x10) ? 1 : 0;
    P25 = (pattern & 0x20) ? 1 : 0;
    P35 = (pattern & 0x40) ? 1 : 0;
}

void Digital_Tube_Show(unsigned char num, unsigned char loc)
{
    Digital_Tube_AllOff();
    digital_cube_ShowOne(num);

    switch (loc) {
    case 0:
        P23 = 0;
        break;
    case 1:
        P26 = 0;
        break;
    case 2:
        P01 = 0;
        break;
    case 3:
        P33_LED_DIG4 = 0;
        break;
    default:
        break;
    }
}

void Digital_Tube_ShowNum(int num)
{
    unsigned char new_display[4];
    bit saved_ea;

    if (num >= 0) {
        new_display[0] = (num / 1000) % 10;
        new_display[1] = (num / 100) % 10;
        new_display[2] = (num / 10) % 10;
        new_display[3] = num % 10;
    } else {
        new_display[0] = 22;
        new_display[1] = (-num / 100) % 10;
        new_display[2] = (-num / 10) % 10;
        new_display[3] = -num % 10;
    }

    saved_ea = EA;
    EA = 0;
    display_buf[0] = new_display[0];
    display_buf[1] = new_display[1];
    display_buf[2] = new_display[2];
    display_buf[3] = new_display[3];
    EA = saved_ea;
}

void Digital_Tube_ShowDashes(void)
{
    bit saved_ea;

    saved_ea = EA;
    EA = 0;
    display_buf[0] = 22;
    display_buf[1] = 22;
    display_buf[2] = 22;
    display_buf[3] = 22;
    EA = saved_ea;
}

void Digital_Tube_flash(void)
{
    static unsigned char position = 0;

    Digital_Tube_Show(display_buf[position], position);
    position++;
    if (position >= 4) {
        position = 0;
    }
}
