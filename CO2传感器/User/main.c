#include "uart1.h"
#include "delay.h"
#include "digital_cube.h"
#include "addr.h"
#include "ModeBusRTU.h"
#include "timer0.h"
#include "config.h"

unsigned int d_CO2 = 0;

static void error_handle(void)
{
    if (Reg[0x0004] == 1) {
        P34 = 0; /* Red LED is active low. */
    } else {
        P34 = 1;
    }
}

static void run_1ms(void)
{
    Digital_Tube_flash();
    CO2_PWM_Tick1ms();
    ModBus_Tick1ms();
}

static void run_100ms(void)
{
    error_handle();
}

static void run_500ms(void)
{
    P22 = ~P22;
}

void main(void)
{
    uint16_t latest_ppm;
    uint8_t display_is_invalid = 0;

    P0M1 = 0;
    P0M0 = 0;
    P1M1 = 0;
    P1M0 = 0;
    P2M1 = 0;
    P2M0 = 0;
    P3M1 = 0;
    P3M0 = 0;
    P5M1 = 0;
    P5M0 = 0;

    /* Green LED, red LED and fourth display digit: push-pull outputs. */
    P2M0 |= (0x01 << 2);
    P2M1 &= ~(0x01 << 2);
    P3M0 |= (0x01 << 3) | (0x01 << 4);
    P3M1 &= ~((0x01 << 3) | (0x01 << 4));

    P22 = 1;
    P34 = 1;
    P33 = 1;

    addr_init();

    /* Allow the external CO2 module and power rails to settle. */
    Delay_xms(1000);
    Delay_xms(1000);

    digital_cube_init();
    Digital_Tube_ShowNum(0);

    address = addr_read();
    CO2_PWM_Init();
    UART1_config(UART1_P30P31, MAIN_Fosc, 9600);
    Timer0_Config(STC_1T, 1000);

    EA = 1;

    while (1) {
        /*
         * Timer0 measures the P3.2 PWM high time and updates the latest
         * valid value after each complete PWM pulse.
         */
        if (CO2_GetLatest(&latest_ppm)) {
            if (display_is_invalid || latest_ppm != d_CO2) {
                d_CO2 = latest_ppm;
                Digital_Tube_ShowNum(d_CO2);
                display_is_invalid = 0;
            }
        } else if (CO2_PWM_IsTimedOut() && !display_is_invalid) {
            Digital_Tube_ShowDashes();
            display_is_invalid = 1;
        }

        /* Parse complete controller requests and send replies outside ISRs. */
        ModBus_Process();
    }
}

static uint8_t count100ms;
static uint16_t count500ms;

void Timer0(void) interrupt TIMER0_VECTOR
{
    CO2_PWM_Timer0Overflow();
    run_1ms();

    count100ms++;
    count500ms++;

    if (count100ms >= 100) {
        count100ms = 0;
        run_100ms();
    }

    if (count500ms >= 500) {
        count500ms = 0;
        run_500ms();
    }
}
