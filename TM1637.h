#ifndef	_TM1637_H_
#define	_TM1637_H_

#include "util.h"


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

#define CLK_INPUT() PB3_INPUT()
#define CLK_OUTPUT() PB3_OUTPUT()
#define CLK_PULLUP() PB3_PULLUP()
#define CLK_PULLUP_OFF() PB3_PULLUP_OFF()
#define CLK_ON_IMMEDIATELY() PB3_ON()
#define CLK_OFF_IMMEDIATELY() PB3_OFF()
#define CLK_ON() clk_on()
#define CLK_OFF() clk_off()
#define CLK_STATE() PB3_STATE()


#define DIO_INPUT() PB2_INPUT()
#define DIO_OUTPUT() PB2_OUTPUT()
#define DIO_PULLUP() PB2_PULLUP()
#define DIO_PULLUP_OFF() PB2_PULLUP_OFF()
#define DIO_ON_IMMEDIATELY() PB2_ON()
#define DIO_OFF_IMMEDIATELY() PB2_OFF()
#define DIO_ON() dio_on()
#define DIO_OFF() dio_off()
#define DIO_STATE() PB2_STATE()


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

void send_bytes(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
void send_byte(uint8_t a);
void start(void);
void stop(void);
void recv_ack(void);


#endif