#include "ModeBusRTU.h"
#include "uart1.h"
#include "config.h"

#define CO2_MAX_PPM             5000

#define TIMER0_COUNTS_PER_MS    (MAIN_Fosc / 1000UL)
#define TIMER0_RELOAD_VALUE     (65536UL - TIMER0_COUNTS_PER_MS)

#define CO2_PWM_PERIOD_MIN_MS   954UL
#define CO2_PWM_PERIOD_MAX_MS   1054UL
#define CO2_PWM_HIGH_MAX_MS     1004UL
#define CO2_PWM_TIMEOUT_MS      3000

#define MODBUS_FRAME_LENGTH     8
#define MODBUS_BUFFER_LENGTH    20
#define MODBUS_FRAME_GAP_MS     5

#define SENSOR_MODEL_CO2        0x06

typedef struct {
    uint16_t ppm;
    uint8_t valid;
} CO2_Data;

static volatile CO2_Data co2 = {0, 0};

/* P3.2 PWM parser state, sampled by Timer0 every 1 ms. */
static volatile u32 pwm_timer0_base = 0;
static volatile u32 pwm_cycle_start = 0;
static volatile u32 pwm_high_counts = 0;
static volatile uint16_t pwm_age_ms = 0;
static volatile uint8_t pwm_cycle_started = 0;
static volatile uint8_t pwm_high_captured = 0;

/* UART1 Modbus frame state. */
static volatile uint8_t ModbusReceiveBuf[MODBUS_BUFFER_LENGTH];
static volatile uint8_t modbus_rx_length = 0;
static volatile uint8_t modbus_gap_ms = 0;
static volatile uint8_t modbus_frame_ready = 0;
static volatile uint8_t modbus_rx_overflow = 0;

uint8_t address;
volatile uint16_t Reg[6] = {0};

void CO2_PWM_Init(void)
{
    /* P3.2 high-impedance input; PWM output level is 3.3 V logic. */
    P3M0 &= ~(0x01 << 2);
    P3M1 |= (0x01 << 2);
    P32 = 1;

    EX0 = 0;
    IE0 = 0;

    pwm_timer0_base = 0;
    pwm_cycle_start = 0;
    pwm_high_counts = 0;
    pwm_age_ms = 0;
    pwm_cycle_started = 0;
    pwm_high_captured = 0;
    co2.ppm = 0;
    co2.valid = 0;

    /*
     * STC8H INT0:
     *   IT0 = 0: both rising and falling edges generate an interrupt.
     *   The ISR reads P3.2 to distinguish the edge polarity.
    */
    IT0 = 0;
    PX0 = 0;
    IPH &= ~0x01;
    EX0 = 1;
}

void CO2_PWM_Timer0Overflow(void)
{
    pwm_timer0_base += TIMER0_COUNTS_PER_MS;
}

void CO2_PWM_Tick1ms(void)
{
    if (pwm_age_ms < CO2_PWM_TIMEOUT_MS) {
        pwm_age_ms++;
        if (pwm_age_ms >= CO2_PWM_TIMEOUT_MS) {
            co2.valid = 0;
            pwm_cycle_started = 0;
            pwm_high_captured = 0;
        }
    }
}

static uint16_t CO2_PWM_ReadTimer0(void)
{
    uint8_t high_first;
    uint8_t low;
    uint8_t high_second;

    do {
        high_first = TH0;
        low = TL0;
        high_second = TH0;
    } while (high_first != high_second);

    return ((uint16_t)high_first << 8) | low;
}

static u32 CO2_PWM_ReadTimestamp(void)
{
    u32 base;
    uint16_t timer_value;

    base = pwm_timer0_base;
    timer_value = CO2_PWM_ReadTimer0();

    /*
     * INT0 and Timer0 use the same priority and cannot preempt each other.
     * If Timer0 overflowed but its ISR has not run yet, include that pending
     * millisecond and reread the timer.
     */
    if (TF0) {
        base += TIMER0_COUNTS_PER_MS;
        timer_value = CO2_PWM_ReadTimer0();
    }

    return base + (uint16_t)(timer_value - TIMER0_RELOAD_VALUE);
}

static uint16_t CO2_PWM_CountsToPPM(u32 high_counts)
{
    u32 ppm;

    if (high_counts <= 2UL * TIMER0_COUNTS_PER_MS) {
        ppm = 0;
    } else {
        high_counts -= 2UL * TIMER0_COUNTS_PER_MS;
        ppm = (high_counts * 5UL + TIMER0_COUNTS_PER_MS / 2UL) /
              TIMER0_COUNTS_PER_MS;
    }

    if (ppm > CO2_MAX_PPM) {
        ppm = CO2_MAX_PPM;
    }

    return (uint16_t)ppm;
}

uint8_t CO2_GetLatest(uint16_t *ppm)
{
    bit saved_ea;
    uint8_t valid;

    saved_ea = EA;
    EA = 0;
    valid = co2.valid;
    if (valid) {
        *ppm = co2.ppm;
    }
    EA = saved_ea;

    return valid;
}

uint8_t CO2_PWM_IsTimedOut(void)
{
    bit saved_ea;
    uint8_t timed_out;

    saved_ea = EA;
    EA = 0;
    timed_out = (pwm_age_ms >= CO2_PWM_TIMEOUT_MS);
    EA = saved_ea;

    return timed_out;
}

void INT0_PWM_ISR(void) interrupt INT0_VECTOR
{
    u32 now;
    u32 period_counts;
    u32 high_counts;

    now = CO2_PWM_ReadTimestamp();

    if (P32) {
        /* Rising edge: finish the previous cycle, then start a new one. */
        if (pwm_cycle_started && pwm_high_captured) {
            period_counts = now - pwm_cycle_start;
            high_counts = pwm_high_counts;

            if (period_counts >=
                    CO2_PWM_PERIOD_MIN_MS * TIMER0_COUNTS_PER_MS &&
                period_counts <=
                    CO2_PWM_PERIOD_MAX_MS * TIMER0_COUNTS_PER_MS &&
                high_counts <=
                    CO2_PWM_HIGH_MAX_MS * TIMER0_COUNTS_PER_MS) {
                co2.ppm = CO2_PWM_CountsToPPM(high_counts);
                co2.valid = 1;
                pwm_age_ms = 0;
            }
        }

        pwm_cycle_start = now;
        pwm_high_counts = 0;
        pwm_cycle_started = 1;
        pwm_high_captured = 0;
    } else {
        /* Falling edge: latch TH for the cycle that started at rising edge. */
        if (pwm_cycle_started && !pwm_high_captured) {
            high_counts = now - pwm_cycle_start;
            if (high_counts <=
                    CO2_PWM_HIGH_MAX_MS * TIMER0_COUNTS_PER_MS) {
                pwm_high_counts = high_counts;
                pwm_high_captured = 1;
            } else {
                pwm_cycle_started = 0;
            }
        }
    }
}

uint16_t ModBus_CRC(volatile uint8_t *buffer, uint8_t length)
{
    uint8_t i;
    uint8_t bit_index;
    uint16_t crc = 0xFFFF;

    for (i = 0; i < length; i++) {
        crc ^= buffer[i];
        for (bit_index = 0; bit_index < 8; bit_index++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

void ModBus_ReceiveByte(uint8_t data_byte)
{
    if (modbus_frame_ready) {
        return;
    }

    if (modbus_rx_length < MODBUS_BUFFER_LENGTH) {
        ModbusReceiveBuf[modbus_rx_length++] = data_byte;
    } else {
        modbus_rx_overflow = 1;
    }

    modbus_gap_ms = 0;
}

void ModBus_Tick1ms(void)
{
    if (modbus_rx_length == 0 || modbus_frame_ready) {
        return;
    }

    if (modbus_gap_ms < MODBUS_FRAME_GAP_MS) {
        modbus_gap_ms++;
    }

    if (modbus_gap_ms >= MODBUS_FRAME_GAP_MS) {
        modbus_frame_ready = 1;
    }
}

static void ModBus_ResetReceive(void)
{
    bit saved_ea;

    saved_ea = EA;
    EA = 0;
    modbus_rx_length = 0;
    modbus_gap_ms = 0;
    modbus_frame_ready = 0;
    modbus_rx_overflow = 0;
    EA = saved_ea;
}

static void ModBus_WriteRegister(uint16_t register_address,
                                 uint16_t register_value)
{
    bit saved_ea;

    saved_ea = EA;
    EA = 0;
    Reg[register_address] = register_value;
    EA = saved_ea;
}

static void ModBus_SendCO2(uint8_t slave_address, uint16_t ppm)
{
    uint8_t send_buffer[8];
    uint16_t crc;

    send_buffer[0] = slave_address;
    send_buffer[1] = 0x03;
    send_buffer[2] = 0x03;
    send_buffer[3] = SENSOR_MODEL_CO2;
    send_buffer[4] = (uint8_t)(ppm >> 8);
    send_buffer[5] = (uint8_t)ppm;

    crc = ModBus_CRC(send_buffer, 6);
    send_buffer[6] = (uint8_t)crc;
    send_buffer[7] = (uint8_t)(crc >> 8);

    UART1_sendArray(sizeof(send_buffer), send_buffer);
}

void ModBus_Process(void)
{
    uint16_t crc_received;
    uint16_t register_address;
    uint16_t register_value;
    uint16_t ppm;
    uint8_t request_address;
    uint8_t function_code;
    uint8_t send_response = 0;

    if (!modbus_frame_ready) {
        return;
    }

    if (modbus_rx_overflow ||
        modbus_rx_length != MODBUS_FRAME_LENGTH) {
        ModBus_ResetReceive();
        return;
    }

    crc_received = (uint16_t)ModbusReceiveBuf[6] |
                   ((uint16_t)ModbusReceiveBuf[7] << 8);
    if (ModBus_CRC(ModbusReceiveBuf, 6) != crc_received) {
        ModBus_ResetReceive();
        return;
    }

    request_address = ModbusReceiveBuf[0];
    if (request_address != address && request_address != 0xFF) {
        ModBus_ResetReceive();
        return;
    }

    function_code = ModbusReceiveBuf[1];
    register_address = ((uint16_t)ModbusReceiveBuf[2] << 8) |
                       ModbusReceiveBuf[3];
    register_value = ((uint16_t)ModbusReceiveBuf[4] << 8) |
                     ModbusReceiveBuf[5];

    if (function_code == 0x03) {
        if (request_address != 0xFF &&
            (register_address == 0x0000 || register_address == 0x0001) &&
            register_value == 0x0001 &&
            CO2_GetLatest(&ppm)) {
            send_response = 1;
        }
    } else if (function_code == 0x06) {
        if (register_address < 6) {
            ModBus_WriteRegister(register_address, register_value);
        }
    }

    ModBus_ResetReceive();

    if (send_response) {
        ModBus_SendCO2(request_address, ppm);
    }
}

void UART1_int(void) interrupt UART1_VECTOR
{
    uint8_t data_byte;

    if (RI) {
        RI = 0;
        data_byte = SBUF;
        ModBus_ReceiveByte(data_byte);
    }

    if (TI) {
        TI = 0;
        B_TX1_Busy = 0;
    }
}
