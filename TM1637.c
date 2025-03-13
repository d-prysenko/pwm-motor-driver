#include "TM1637.h"

void __attribute__ ((noinline)) bit_delay() {
    _delay_us(5);
}


void __attribute__ ((noinline)) clk_on() {
    CLK_ON_IMMEDIATELY();
    bit_delay();
}

void __attribute__ ((noinline)) clk_off() {
    CLK_OFF_IMMEDIATELY();
    bit_delay();
}

void __attribute__ ((noinline)) dio_on() {
    DIO_ON_IMMEDIATELY();
    bit_delay();
}

void __attribute__ ((noinline)) dio_off() {
    DIO_OFF_IMMEDIATELY();
    bit_delay();
}


void send_bytes(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    start();
    send_byte(CMD_DATA_SET);
    stop();

    start();
    send_byte(CMD_ADDRESS_SET);
    send_byte(a);
    send_byte(b);
    send_byte(c);
    send_byte(d);
    stop();

    start();
    send_byte(CMD_DISPLAY_CONTROL_SET | BRIGHTNESS_8 | DISPLAY_ON);
    stop();
}

void send_byte(uint8_t a)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        CLK_OFF();

        if (a & 0x01)
            DIO_ON_IMMEDIATELY();
        else
            DIO_OFF_IMMEDIATELY();

        CLK_ON();

        a = a >> 1;
    }

    recv_ack();
}


void start(void)
{
    DIO_OFF();
}

void stop(void)
{
    CLK_ON();
    DIO_ON();
}

void recv_ack(void)
{
    CLK_OFF_IMMEDIATELY();
    DIO_INPUT();
    DIO_PULLUP();
    bit_delay();

    if (DIO_STATE()) {
        DIO_OUTPUT();
        DIO_OFF();
    }

    CLK_ON();
    CLK_OFF();

    DIO_PULLUP_OFF();
    DIO_OUTPUT();
}
