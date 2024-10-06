#ifndef __UTIL_H__
#define __UTIL_H__

#define F_CPU 9600000ul

#include <avr/io.h>
#include <util/delay.h>

void EEPROM_write(uint8_t ucAddress, uint8_t ucData);
uint8_t EEPROM_read(uint8_t ucAddress);

uint8_t adc_read(void);

void blink_fast(void);
void blink_slow(void);

#endif