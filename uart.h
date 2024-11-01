/**
 * Copyright (c) 2017, Łukasz Marcin Podkalicki <lpodkalicki@gmail.com>
 * The ASM code is based on Ralph Doncaster's project (https://github.com/nerdralph/nerdralph/tree/master/avr/libs/bbuart)
 * Software UART for ATtiny13
 */

#ifndef	_UART_H_
#define	_UART_H_

#define	UART_TX_ENABLED		(1) // Enable UART TX

#ifndef F_CPU
# define        F_CPU           (1200000ul) // 9.6 MHz
#endif  /* !F_CPU */

#if defined(UART_TX_ENABLED) && !defined(UART_TX)
# define        UART_TX         PB2 // Use PB2 as TX pin
#endif  /* !UART_TX */

#if defined(UART_TX_ENABLED) && !defined(UART_BAUDRATE)
# define        UART_BAUDRATE   (9600)
#endif  /* !UART_BAUDRATE */

#define	TXDELAY         	(int)(((F_CPU/UART_BAUDRATE)-7 +1.5)/3)
// #define	TXDELAY         	(int)(((1200000ul/9600)-7 +1.5)/3)
// #define	TXDELAY         	(int)(63)


void uart_putc(char c);
void uart_putu(uint16_t x);
void uart_puts(const char *s);

#endif	/* !_UART_H_ */
