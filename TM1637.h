#ifndef	_TM1637_H_
#define	_TM1637_H_

#include "util.h"
#include <avr/pgmspace.h>

#define CMD_DATA_SET 0b01000000
#define CMD_DATA_SET_FIXED_ADDR 0b01000100

#define CMD_ADDRESS_SET 0b11000000

#define CMD_DISPLAY_CONTROL_SET 0b10000000
#define BRIGHTNESS_1 0b00000000
#define BRIGHTNESS_2 0b00000001
#define BRIGHTNESS_3 0b00000010
#define BRIGHTNESS_4 0b00000011
#define BRIGHTNESS_5 0b00000100
#define BRIGHTNESS_6 0b00000101
#define BRIGHTNESS_7 0b00000110
#define BRIGHTNESS_8 0b00000111
#define DISPLAY_ON  0b00001000
#define DISPLAY_OFF 0b00000000

void __attribute__ ((noinline)) bit_delay();
// bit_delay used in all 4 listed functions. noinline for decreasing size
void __attribute__ ((noinline)) clk_on();
void __attribute__ ((noinline)) clk_off();
void __attribute__ ((noinline)) dio_on();
void __attribute__ ((noinline)) dio_off();

#define CLK_INPUT() PB0_INPUT()
#define CLK_OUTPUT() PB0_OUTPUT()
#define CLK_PULLUP() PB0_PULLUP()
#define CLK_PULLUP_OFF() PB0_PULLUP_OFF()
#define CLK_ON_IMMEDIATELY() PB0_ON()
#define CLK_OFF_IMMEDIATELY() PB0_OFF()
#define CLK_ON() clk_on()
#define CLK_OFF() clk_off()
#define CLK_STATE() PB0_STATE()


#define DIO_INPUT() PD7_INPUT()
#define DIO_OUTPUT() PD7_OUTPUT()
#define DIO_PULLUP() PD7_PULLUP()
#define DIO_PULLUP_OFF() PD7_PULLUP_OFF()
#define DIO_ON_IMMEDIATELY() PD7_ON()
#define DIO_OFF_IMMEDIATELY() PD7_OFF()
#define DIO_ON() dio_on()
#define DIO_OFF() dio_off()
#define DIO_STATE() PD7_STATE()


#define _0 0x3f
#define _1 0x06
#define _2 0x5b
#define _3 0x4f
#define _4 0x66
#define _5 0x6d
#define _6 0x7d
#define _7 0x07
#define _8 0x7f
#define _9 0x6f

#define _S 0x6d
#define _L 0x38
#define _E 0x79
#define _P 0x73

#define _D 0x5e
#define _A 0x77

static inline void tm1637_init(void)
{
    CLK_OUTPUT();
    DIO_OUTPUT();

    DIO_ON();
    CLK_ON();
}

void send_uint16(uint16_t number, uint8_t with_leading_empty);
void send_uint16_underscore(uint16_t number);
void send_bytes(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
void send_byte(uint8_t a);
void start(void);
void stop(void);
void recv_ack(void);


#endif