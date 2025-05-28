#ifndef __UTIL_H__
#define __UTIL_H__

#define F_CPU 8000000ul
#define __AVR_ATmega328P__

#include <avr/io.h>
#include <util/delay.h>

#if defined (__AVR_ATtiny13__)

#define PB0_INPUT() (DDRB &= ~(1 << DDB0))
#define PB1_INPUT() (DDRB &= ~(1 << DDB1))
#define PB2_INPUT() (DDRB &= ~(1 << DDB2))
#define PB3_INPUT() (DDRB &= ~(1 << DDB3))
#define PB4_INPUT() (DDRB &= ~(1 << DDB4))
#define PB5_INPUT() (DDRB &= ~(1 << DDB5))

#define PB0_OUTPUT() (DDRB |= (1 << DDB0))
#define PB1_OUTPUT() (DDRB |= (1 << DDB1))
#define PB2_OUTPUT() (DDRB |= (1 << DDB2))
#define PB3_OUTPUT() (DDRB |= (1 << DDB3))
#define PB4_OUTPUT() (DDRB |= (1 << DDB4))
#define PB5_OUTPUT() (DDRB |= (1 << DDB5))

#define PB0_PULLUP() (PORTB |= (1 << PB0))
#define PB1_PULLUP() (PORTB |= (1 << PB1))
#define PB2_PULLUP() (PORTB |= (1 << PB2))
#define PB3_PULLUP() (PORTB |= (1 << PB3))
#define PB4_PULLUP() (PORTB |= (1 << PB4))
#define PB5_PULLUP() (PORTB |= (1 << PB5))

#define PB0_PULLUP_OFF() (PORTB &= ~(1 << PB0))
#define PB1_PULLUP_OFF() (PORTB &= ~(1 << PB1))
#define PB2_PULLUP_OFF() (PORTB &= ~(1 << PB2))
#define PB3_PULLUP_OFF() (PORTB &= ~(1 << PB3))
#define PB4_PULLUP_OFF() (PORTB &= ~(1 << PB4))
#define PB5_PULLUP_OFF() (PORTB &= ~(1 << PB5))

#define PB0_ON() PB0_PULLUP()
#define PB1_ON() PB1_PULLUP()
#define PB2_ON() PB2_PULLUP()
#define PB3_ON() PB3_PULLUP()
#define PB4_ON() PB4_PULLUP()
#define PB5_ON() PB5_PULLUP()

#define PB0_OFF() PB0_PULLUP_OFF()
#define PB1_OFF() PB1_PULLUP_OFF()
#define PB2_OFF() PB2_PULLUP_OFF()
#define PB3_OFF() PB3_PULLUP_OFF()
#define PB4_OFF() PB4_PULLUP_OFF()
#define PB5_OFF() PB5_PULLUP_OFF()

#define PB0_STATE() (PINB & (1 << PINB0))
#define PB1_STATE() (PINB & (1 << PINB1))
#define PB2_STATE() (PINB & (1 << PINB2))
#define PB3_STATE() (PINB & (1 << PINB3))
#define PB4_STATE() (PINB & (1 << PINB4))
#define PB5_STATE() (PINB & (1 << PINB5))


#define SET(where, pos) where |= (1 << pos)
#define UNSET(where, pos) where &= ~(1 << pos)

#define SLEEP_ENABLE()  SET(MCUCR, SE)
#define SLEEP_DISABLE() UNSET(MCUCR, SE)


#define IDLE_SLEEP_MODE() UNSET(MCUCR, SM0); \
    UNSET(MCUCR, SM1)

#define ADC_NOISE_REDUCTION_SLEEP_MODE() SET(MCUCR, SM0); \
    UNSET(MCUCR, SM1)

#define POWER_DOWN_SLEEP_MODE() UNSET(MCUCR, SM0); \
    SET(MCUCR, SM1)

#define ADC_NOISE_REDUCTION_SLEEP_MODE_INLINE() SET(MCUCR, SM0)
#define POWER_DOWN_SLEEP_MODE_INLINE() SET(MCUCR, SM1)

#define SET_CPU_FREQ_DIV_1() CLKPR = (1 << CLKPCE); \
    CLKPR = 0

#define SET_CPU_FREQ_DIV_2() CLKPR = (1 << CLKPCE); \
    CLKPR = (1 << CLKPS0);

#define SET_ADC_CLOCK_PRESCALE_128() ADCSRA = ADCSRA | 0b00000111

#elif defined (__AVR_ATmega328P__)

#define PB0_INPUT() (DDRB &= ~(1 << DDB0))
#define PB1_INPUT() (DDRB &= ~(1 << DDB1))
#define PB2_INPUT() (DDRB &= ~(1 << DDB2))
#define PB3_INPUT() (DDRB &= ~(1 << DDB3))
#define PB4_INPUT() (DDRB &= ~(1 << DDB4))
#define PB5_INPUT() (DDRB &= ~(1 << DDB5))

#define PD7_INPUT()  (DDRD &= ~(1 << DDD7))

#define PB0_OUTPUT() (DDRB |= (1 << DDB0))
#define PB1_OUTPUT() (DDRB |= (1 << DDB1))
#define PB2_OUTPUT() (DDRB |= (1 << DDB2))
#define PB3_OUTPUT() (DDRB |= (1 << DDB3))
#define PB4_OUTPUT() (DDRB |= (1 << DDB4))
#define PB5_OUTPUT() (DDRB |= (1 << DDB5))

#define PD6_OUTPUT() (DDRD |= (1 << DDD6))
#define PD7_OUTPUT() (DDRD |= (1 << DDD7))

#define PB0_PULLUP() (PORTB |= (1 << PB0))
#define PB1_PULLUP() (PORTB |= (1 << PB1))
#define PB2_PULLUP() (PORTB |= (1 << PB2))
#define PB3_PULLUP() (PORTB |= (1 << PB3))
#define PB4_PULLUP() (PORTB |= (1 << PB4))
#define PB5_PULLUP() (PORTB |= (1 << PB5))

#define PD7_PULLUP() (PORTD |= (1 << PD7))

#define PB0_PULLUP_OFF() (PORTB &= ~(1 << PB0))
#define PB1_PULLUP_OFF() (PORTB &= ~(1 << PB1))
#define PB2_PULLUP_OFF() (PORTB &= ~(1 << PB2))
#define PB3_PULLUP_OFF() (PORTB &= ~(1 << PB3))
#define PB4_PULLUP_OFF() (PORTB &= ~(1 << PB4))
#define PB5_PULLUP_OFF() (PORTB &= ~(1 << PB5))

#define PD7_PULLUP_OFF() (PORTD &= ~(1 << PD7))

#define PB0_ON() PB0_PULLUP()
#define PB1_ON() PB1_PULLUP()
#define PB2_ON() PB2_PULLUP()
#define PB3_ON() PB3_PULLUP()
#define PB4_ON() PB4_PULLUP()
#define PB5_ON() PB5_PULLUP()

#define PD7_ON() PD7_PULLUP()

#define PB0_OFF() PB0_PULLUP_OFF()
#define PB1_OFF() PB1_PULLUP_OFF()
#define PB2_OFF() PB2_PULLUP_OFF()
#define PB3_OFF() PB3_PULLUP_OFF()
#define PB4_OFF() PB4_PULLUP_OFF()
#define PB5_OFF() PB5_PULLUP_OFF()

#define PD7_OFF() PD7_PULLUP_OFF()

#define PB0_STATE() (PINB & (1 << PINB0))
#define PB1_STATE() (PINB & (1 << PINB1))
#define PB2_STATE() (PINB & (1 << PINB2))
#define PB3_STATE() (PINB & (1 << PINB3))
#define PB4_STATE() (PINB & (1 << PINB4))
#define PB5_STATE() (PINB & (1 << PINB5))

#define PD7_STATE() (PIND & (1 << PIND7))


#define PC5_INPUT() (DDRC &= ~(1 << DDC5))
#define PC5_OUTPUT() (DDRC |= (1 << DDC5))
#define PC5_PULLUP() (PORTC |= (1 << PC5))
#define PC5_PULLUP_OFF() (PORTC &= ~(1 << PC5))
#define PC5_ON() PC5_PULLUP()
#define PC5_OFF() PC5_PULLUP_OFF()
#define PC5_STATE() (PINC & (1 << PINC5))

#define PD4_INPUT() (DDRD &= ~(1 << DDD4))
#define PD4_OUTPUT() (DDRD |= (1 << DDD4))
#define PD4_PULLUP() (PORTD |= (1 << PD4))
#define PD4_PULLUP_OFF() (PORTD &= ~(1 << PD4))
#define PD4_ON() PD4_PULLUP()
#define PD4_OFF() PD4_PULLUP_OFF()
#define PD4_STATE() (PIND & (1 << PIND4))


#define SET(where, pos) where |= (1 << pos)
#define UNSET(where, pos) where &= ~(1 << pos)

#define SLEEP_ENABLE()  SET(SMCR, SE)
#define SLEEP_DISABLE() UNSET(SMCR, SE)


#define IDLE_SLEEP_MODE() UNSET(SMCR, SM0); UNSET(SMCR, SM1); UNSET(SMCR, SM2)
#define ADC_NOISE_REDUCTION_SLEEP_MODE() SET(SMCR, SM0); UNSET(SMCR, SM1); UNSET(SMCR, SM2)
#define POWER_DOWN_SLEEP_MODE() UNSET(SMCR, SM0); SET(SMCR, SM1); UNSET(SMCR, SM2)

#define ADC_NOISE_REDUCTION_SLEEP_MODE_INLINE() SET(SMCR, SM0)
#define POWER_DOWN_SLEEP_MODE_INLINE() SET(SMCR, SM1)

#define SET_CPU_FREQ_DIV_1() CLKPR = (1 << CLKPCE); \
    CLKPR = 0

#define SET_CPU_FREQ_DIV_2() CLKPR = (1 << CLKPCE); \
    CLKPR = (1 << CLKPS0);

#define SET_PWM_PRESCALE_1() SET(TCCR0B, CS00)

#define SET_ADC_CLOCK_PRESCALE_128() SET(ADCSRA, ADPS2); SET(ADCSRA, ADPS1); SET(ADCSRA, ADPS0)
#define SET_ADC_CLOCK_PRESCALE_64() SET(ADCSRA, ADPS2); SET(ADCSRA, ADPS1); UNSET(ADCSRA, ADPS0)

#define ENABLE_ADC() SET(ADCSRA, ADEN)
#define DISABLE_ADC() UNSET(ADCSRA, ADEN)

#define SET_ADC_1_1_REFERENCE() SET(ADMUX, REFS0); SET(ADMUX, REFS1)

#endif

void EEPROM_write(uint8_t ucAddress, uint8_t ucData);
void EEPROM_write_uint16(uint8_t ucAddress, uint16_t ucData);
uint8_t EEPROM_read(uint8_t ucAddress);
uint16_t EEPROM_read_uint16(uint8_t ucAddress);

uint16_t adc_read(void);

void blink_fast(void);
void blink_slow(void);

#endif