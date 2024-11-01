/**
 * Copyright (c) 2017, Łukasz Marcin Podkalicki <lpodkalicki@gmail.com>
 * The ASM code is based on Ralph Doncaster's project (https://github.com/nerdralph/nerdralph/tree/master/avr/libs/bbuart)
 * Software UART for ATtiny13
 */

#include <avr/interrupt.h>
#include "uart.h"


void
uart_putc(char c)
{
#ifdef	UART_TX_ENABLED
	uint8_t sreg;

	sreg = SREG;
	cli();
	PORTB |= 1 << UART_TX;
	DDRB |= 1 << UART_TX;
	__asm volatile(
		" cbi %[uart_port], %[uart_pin] \n\t" // start bit
		" in r0, %[uart_port] \n\t"
		" ldi r30, 3 \n\t" // stop bit + idle state
		" ldi r28, %[txdelay] \n\t"
		"TxLoop: \n\t"
		// 8 cycle loop + delay - total = 7 + 3*r22
		" mov r29, r28 \n\t"
		"TxDelay: \n\t"
		// delay (3 cycle * delayCount) - 1
		" dec r29 \n\t"
		" brne TxDelay \n\t"
		" bst %[ch], 0 \n\t"
		" bld r0, %[uart_pin] \n\t"
		" lsr r30 \n\t"
		" ror %[ch] \n\t"
		" out %[uart_port], r0 \n\t"
		" brne TxLoop \n\t"
		:
		: [uart_port] "I" (_SFR_IO_ADDR(PORTB)),
		[txdelay] "I" (TXDELAY),
		[uart_pin] "I" (UART_TX),
		[ch] "r" (c)
		: "r0","r28","r29","r30"
	);
	SREG = sreg;
#endif /* !UART_TX_ENABLED */
}

void uart_putu(uint16_t x)
{
	char buff[8] = {0};
	char *p = buff+6;
	do { *(p--) = (x % 10) + '0'; x /= 10; } while(x);
	uart_puts((const char *)(p+1));
}

void
uart_puts(const char *s)
{
	while (*s) uart_putc(*(s++));
}
