#define F_CPU 9600000ul

#define __AVR_ATtiny13__

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "util.h"


#define ADDR_TEMPERATURE_90 0
#define ADDR_TEMPERATURE_100 1


void setup_int(void);
void setup_pwm(void);
void setup_adc(void);

uint8_t map_temperature_to_duty(uint8_t adc_temperature, uint8_t is_ac_on);
void set_duty_smoothly(uint8_t duty);
void calc_adc_temp_borders(void);

const uint8_t temperature_duty_ac_off_map[] = {
    0x00, // <=90 0%
    0x4D, // 91   30%
    0x71, // 92   44%
    0x92, // 93   57%
    0xB3, // 94   70%
    0xD4, // 95   83%
    0xE6, // 96   90%
    0xF3, // 97   95%
    0xFF, // >=98 100%
};

const uint8_t temperature_duty_ac_on_map[] = {
    0xCD, // <=90 80%
    0xCD, // 91   80%
    0xCD, // 92   80%
    0xCD, // 93   80%
    0xE6, // 94   90%
    0xE6, // 95   90%
    0xFF, // 96   100%
    0xFF, // 97   100%
    0xFF, // >=98 100%
};

uint8_t adc_temperature_90 = 0;
uint8_t adc_temperature_100 = 0;
uint8_t adc_temperature_1_deg = 1;


volatile uint8_t is_button_pressed = 0;
volatile uint8_t calibration_number = 0;

ISR (PCINT0_vect) {
    is_button_pressed = !is_button_pressed;

    if (is_button_pressed) {
        return;
    }

    if (!calibration_number) {
        EEPROM_write(ADDR_TEMPERATURE_90, adc_read());
        blink_fast();
    } else {
        EEPROM_write(ADDR_TEMPERATURE_100, adc_read());
        blink_slow();
    }
    
    calc_adc_temp_borders();

    calibration_number = !calibration_number;
}

int main(void) {
    // set frequency divider 0 (9.6 MGHz)
    CLKPR = (1 << CLKPCE);
    CLKPR = 0;

    setup_pwm();
    setup_adc();
    setup_int();

    // pwm counter - 0, duty - 0
    TCNT0 = 0;
    OCR0A = 0;

    calc_adc_temp_borders();

    while (1)
    {
        uint8_t adc_temp = adc_read();
        uint8_t ac_on = 0;

        uint8_t duty = map_temperature_to_duty(adc_temp, ac_on);

        set_duty_smoothly(duty);
        // OCR0A = duty;

        // _delay_ms(5000);
    }

    return 0;
}

void calc_adc_temp_borders(void) {
    adc_temperature_90 = EEPROM_read(ADDR_TEMPERATURE_90);
    adc_temperature_100 = EEPROM_read(ADDR_TEMPERATURE_100);
    adc_temperature_1_deg = (adc_temperature_90 - adc_temperature_100) / 10;
}

void setup_int(void) {
    // PB3 - input
    DDRB &= ~(1 << DDB3);
    // pull-up
    PORTB |= (1 << PB3);

    // int
    GIMSK |= (1 << PCIE);
    PCMSK = (1 << PCINT3);
    SREG |= (1 << SREG_I);
}

void setup_pwm(void) {
    // PB0 - output
    DDRB |= (1 << DDB0);

    // phase correct pwm
    TCCR0A |= (1 << WGM00);
    TCCR0A &= ~(1 << WGM01);
    TCCR0B &= ~(1 << WGM01);

    // compare output mode
    TCCR0A |= (1 << COM0A1);
    TCCR0A &= ~(1 << COM0A0);

    // clock prescale 1
    TCCR0B |= (1 << CS00);
    TCCR0B &= ~(1 << CS01);
    TCCR0B &= ~(1 << CS02);
}

void setup_adc(void) {
    // PB4 - ADC
    ADMUX |= (1 << MUX1);

    // left aligned to use 8-bit ADC
    ADMUX |= (1 << ADLAR);

    // set the prescaler to clock/128
    ADCSRA |= (1 << ADPS1) | (1 << ADPS0);

    // enable ADC
    ADCSRA |= (1 << ADEN);
}

uint8_t map_temperature_to_duty(uint8_t adc_temperature, uint8_t is_ac_on) {
    uint8_t duty_index = 0;

    if (adc_temperature < adc_temperature_90) {
        duty_index = (adc_temperature_90 - adc_temperature) / adc_temperature_1_deg;
    }

    if (duty_index > 8) {
        duty_index = 8;
    }

    if (is_ac_on) {
        return temperature_duty_ac_on_map[duty_index];
    }

    return temperature_duty_ac_off_map[duty_index];
}

void set_duty_smoothly(uint8_t duty) {
    while (OCR0A > duty)
    {
        OCR0A--;
        _delay_ms(25);
    }

    while (OCR0A < duty)
    {
        OCR0A++;
        _delay_ms(25);
    }
}
