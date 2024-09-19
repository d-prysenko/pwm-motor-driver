#define F_CPU 9600000ul

#define __AVR_ATtiny13__

#include <avr/io.h>
#include <util/delay.h>
// #include <avr/iotn13.h>

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

const uint8_t adc_temperature_90 = 150;
const uint8_t adc_temperature_100 = 50;
const uint8_t adc_temperature_step = (adc_temperature_90 - adc_temperature_100) / 10;

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

uint8_t adc_read(void) {
    // Start the conversion
    ADCSRA |= (1 << ADSC);

    // Wait for it to finish - blocking
    while (ADCSRA & (1 << ADSC));

    return ADCH;
}

uint8_t map_temperature_to_duty(uint8_t adc_temperature, uint8_t is_ac_on) {
    uint8_t duty_index = 0;

    int8_t delta = adc_temperature_90 - adc_temperature;

    if (delta > 0) {
        duty_index = delta / adc_temperature_step;
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

int main(void) {
    // set frequency divider 0 (9.6 MGHz)
    CLKPR = (1 << CLKPCE);
    CLKPR = 0;

    setup_pwm();
    setup_adc();

    TCNT0 = 0; // начальное значение счётчика
    OCR0A = 5; // регистр совпадения A

    while (1)
    {
        uint8_t adc_temp = adc_read();
        uint8_t ac_on = 1;

        uint8_t duty = map_temperature_to_duty(adc_temp, ac_on);

        set_duty_smoothly(duty);

        _delay_ms(5000);
    }

    // while (1)
    // {
    //     while (OCR0A < 255)
    //     {
    //         OCR0A++;
    //         _delay_ms(25);
    //     }
    //     _delay_ms(2000);

    //     while (OCR0A > 0)
    //     {
    //         OCR0A--;
    //         _delay_ms(25);
    //     }
    //     _delay_ms(2000);
    // }

    return 0;
}