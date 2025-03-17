#include "TM1637.h"

PROGMEM const uint8_t segments[] =
{
	_0,
	_1,
	_2,
	_3,
	_4,
	_5,
	_6,
	_7,
	_8,
	_9
};

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

void send_uint16(uint16_t number, uint8_t with_leading_empty)
{
    uint8_t digits[4];
    
    for (int8_t i = 0; i < 4; i++) {
        digits[i] = 0;
    }
    
    for (int8_t i = 0; i < 4; i++) {
        digits[i] = pgm_read_byte(&segments[number % 10]);
        number /= 10;
        if (number == 0 && with_leading_empty) {
            break;
        }
    }

    send_bytes(digits[3], digits[2], digits[1], digits[0]);
}

void send_uint16_underscore(uint16_t number)
{
    uint8_t digits[4];
    
    for (int8_t i = 0; i < 4; i++) {
        digits[i] = 0;
    }
    
    for (int8_t i = 0; i < 4; i++) {
        digits[i] = pgm_read_byte(&segments[number % 10]);
        number /= 10;
        if (number == 0) {
            break;
        }
    }

    send_bytes(0x08, digits[2], digits[1], digits[0]);
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
