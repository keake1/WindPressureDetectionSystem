#include <STC8H.H>
#include "digital_tube.h"

/* 段码 bit0~bit6 分别对应 a~g；bit7 为小数点。 */
#define SEG_A    P24
#define SEG_B    P27
#define SEG_C    P36
#define SEG_D    P20
#define SEG_E    P21
#define SEG_F    P25
#define SEG_G    P35
#define SEG_DP   P37

/* 位选低电平有效：从左到右依次为第 1、2、3 位。 */
#define DIGIT_1  P23
#define DIGIT_2  P26
#define DIGIT_3  P01

#define DIGIT_MINUS  10U
#define DIGIT_BLANK  11U

static unsigned char s_digits[3] = {DIGIT_BLANK, 0U, 0U};
static unsigned char s_scan_index = 0U;

code unsigned char s_segment_table[] =
{
    0x3FU, /* 0 */
    0x06U, /* 1 */
    0x5BU, /* 2 */
    0x4FU, /* 3 */
    0x66U, /* 4 */
    0x6DU, /* 5 */
    0x7DU, /* 6 */
    0x07U, /* 7 */
    0x7FU, /* 8 */
    0x6FU, /* 9 */
    0x40U, /* - */
    0x00U  /* blank */
};

static void DigitalTube_AllDigitsOff(void)
{
    DIGIT_1 = 1;
    DIGIT_2 = 1;
    DIGIT_3 = 1;
}

static void DigitalTube_SetSegments(unsigned char segment, unsigned char decimal_point)
{
    SEG_A  = (segment & 0x01U) ? 1 : 0;
    SEG_B  = (segment & 0x02U) ? 1 : 0;
    SEG_C  = (segment & 0x04U) ? 1 : 0;
    SEG_D  = (segment & 0x08U) ? 1 : 0;
    SEG_E  = (segment & 0x10U) ? 1 : 0;
    SEG_F  = (segment & 0x20U) ? 1 : 0;
    SEG_G  = (segment & 0x40U) ? 1 : 0;
    SEG_DP = decimal_point ? 1 : 0;
}

void DigitalTube_Init(void)
{
    /* 三个位选。 */
    P0M1 &= ~0x02U;
    P0M0 |=  0x02U;
    P2M1 &= ~0x48U;
    P2M0 |=  0x48U;

    /* 七段和小数点。 */
    P2M1 &= ~0xB3U;
    P2M0 |=  0xB3U;
    P3M1 &= ~0xE0U;
    P3M0 |=  0xE0U;

    DigitalTube_AllDigitsOff();
    DigitalTube_SetSegments(0U, 0U);
}

void DigitalTube_SetValue(int value_x10)
{
    unsigned int magnitude;

    if(value_x10 < 0)
    {
        magnitude = (unsigned int)(-value_x10);
        if(magnitude > 99U)
        {
            magnitude = 99U;       /* 三位仅可显示到 -9.9。 */
        }

        s_digits[0] = DIGIT_MINUS;
        s_digits[1] = (unsigned char)(magnitude / 10U);
        s_digits[2] = (unsigned char)(magnitude % 10U);
    }
    else
    {
        magnitude = (unsigned int)value_x10;
        if(magnitude > 999U)
        {
            magnitude = 999U;      /* 正值超量程显示为 99.9。 */
        }

        s_digits[0] = (magnitude >= 100U) ? (unsigned char)(magnitude / 100U) : DIGIT_BLANK;
        s_digits[1] = (unsigned char)((magnitude / 10U) % 10U);
        s_digits[2] = (unsigned char)(magnitude % 10U);
    }
}

void DigitalTube_Scan1ms(void)
{
    DigitalTube_AllDigitsOff();
    DigitalTube_SetSegments(s_segment_table[s_digits[s_scan_index]],
                            (s_scan_index == 1U) ? 1U : 0U);

    if(s_scan_index == 0U)
    {
        DIGIT_1 = 0;
    }
    else if(s_scan_index == 1U)
    {
        DIGIT_2 = 0;
    }
    else
    {
        DIGIT_3 = 0;
    }

    s_scan_index++;
    if(s_scan_index >= 3U)
    {
        s_scan_index = 0U;
    }
}
