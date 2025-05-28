#define F_CPU 8000000ul

#define __AVR_ATmega328P__

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "util.h"

#include "uart.h"

#include "TM1637.h"


#define ADDR_TEMPERATURE_90 0
#define ADDR_TEMPERATURE_100 2
#define ADDR_TEMPERATURE_1_DEG 4

// #define DEBUG

#ifdef DEBUG
    #define _inline_
    #define UART_PUTU(u) uart_putu(u)
    #define UART_PUTC(c) uart_putc(c)
    #define UART_PUTS(s) uart_puts(s)
#else
    #define _inline_ static inline
    #define UART_PUTU(u)
    #define UART_PUTC(c)
    #define UART_PUTS(s)
#endif


_inline_ void setup_ignition_int(void);
_inline_ void setup_sleep_mode(void);
_inline_ void setup_pwm(void);
_inline_ void setup_adc(void);

uint8_t map_temperature_to_duty(uint16_t adc_temperature, uint8_t is_ac_on);
void set_duty_smoothly(uint8_t duty);
void calc_adc_temp_borders(void);
uint16_t read_temperature(void);

const uint8_t temperature_duty_ac_off_map[] PROGMEM = {
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

const uint8_t temperature_duty_ac_on_map[] PROGMEM = {
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

const uint8_t afterrun_delay_sec = 10;

uint16_t adc_temperature_90 = 340;
uint16_t adc_temperature_100 = 260;
uint16_t adc_temperature_1_deg = 9;

uint8_t duty = 0;
uint8_t ac_on = 0;

uint8_t ign_off = 0;


uint8_t display_counter = 0;

#define PWM OCR0A
#define IGN_IS_OFF() bit_is_clear(PIND, PIND4)

#define TEN_PERCENT 25
#define NINTY_PERCENT 230

// PD6 - pwm                 output
// PD4 - ignition            input
// PC5 - temperature         input


// uint8_t adc_temp = 0;

// ignition int
ISR (PCINT2_vect) {
    ign_off = IGN_IS_OFF();
}

// ISR (ADC_vect) {
//     uint16_t adc_temp_16 = ADC;

//     if (adc_temp_16 > 255) {
//         adc_temp = 255;
//     } else {
//         adc_temp = (uint8_t)(adc_temp_16);
//     }
// }

void __attribute__ ((noinline)) delay_25ms() {
    _delay_ms(25);
}



int main(void) {
    // set frequency divider 1 (8.0 MHz)
    SET_CPU_FREQ_DIV_1();
    // SET_CPU_FREQ_DIV_2();

    // no pull-ups
    // SET(MCUCR, PUD);

    setup_pwm();
    setup_adc();
    setup_sleep_mode();
    setup_ignition_int();

    // allow interrupts
    sei();
    // SREG |= 1 << SREG_I;
    // pwm counter - 0, duty - 0
    TCNT0 = 0;
    PWM = 0;

    ign_off = IGN_IS_OFF();

    // EEPROM_write(ADDR_TEMPERATURE_90, 225);
    // EEPROM_write(ADDR_TEMPERATURE_100, 140);
    // EEPROM_write(ADDR_TEMPERATURE_1_DEG, 10);

    // calc_adc_temp_borders();
    
    // PWM = TEN_PERCENT;
    
    tm1637_init();

    // while (1)
    // {
    //     set_duty_smoothly(NINTY_PERCENT);
    //     set_duty_smoothly(TEN_PERCENT);
    // }


    // while (1)
    // {
    //     // send_bytes(_1, _2, _3, _4);

    //     // for (int8_t i = 0; i < 25; i++)
    //     // {
    //     //     send_uint16(EEPROM_read_uint16(ADDR_TEMPERATURE_90), 0);
    //     //     _delay_ms(200);
    //     // }

    //     // for (int8_t i = 0; i < 25; i++)
    //     // {
    //     //     send_uint16(EEPROM_read_uint16(ADDR_TEMPERATURE_100), 0);
    //     //     _delay_ms(200);
    //     // }

    //     for (int8_t i = 0; i < 25; i++)
    //     {
    //         uint16_t adc_temp = read_temperature();

    //         send_uint16(adc_temp, 0);
    //         _delay_ms(200);
    //     }

    //     for (int8_t i = 0; i < 25; i++)
    //     {
    //         uint16_t adc_temp = read_temperature();
    //         uint8_t duty = map_temperature_to_duty(adc_temp, 0);

    //         send_uint16(duty, 1);
    //         _delay_ms(200);
    //     }
        
    // }


    while (1)
    {
        if (ign_off) {
            send_bytes(_D, _E, _L, _A);
            set_duty_smoothly(duty);
            
            // wait 25 ms for quicker response if ignition will be turned on when doing after run delay
            for (uint16_t i = 0; (i < afterrun_delay_sec * 40) && ign_off; i++) {
                delay_25ms();
            }

            PWM = 0;

            delay_25ms();

            if (ign_off) {
                send_bytes(0, 0, 0, 0);
                DISABLE_ADC();
                POWER_DOWN_SLEEP_MODE();
                asm("sleep");
                ENABLE_ADC();
            }
        }

        // adc_read();
        uint16_t adc_temp_16 = read_temperature();

        // if (adc_temp_16 < 50) {
        //     adc_temp_16 = 0;
        // } else {
        //     adc_temp_16 -= 50;
        // }

        // uint8_t adc_temp = (adc_temp_16 > 255) ? 255 : (uint8_t)adc_temp_16;

        // uint16_t temp = adc_read();

        if (display_counter < 5) {
            send_uint16(adc_temp_16, 0);
        }

        uint8_t ac_on = 0;

        // UART_PUTU(adc_temp);
        // UART_PUTC(' ');
    
        duty = map_temperature_to_duty(adc_temp_16, ac_on);
    
        if (display_counter >= 5) {
            send_uint16(duty, 1);
        }
    
        if (display_counter >= 10) {
            display_counter = 0;
        }
    
        display_counter++;
    
        UART_PUTC('\n');
        UART_PUTC('\r');
    
        set_duty_smoothly(duty);


        for (int8_t i = 0; i < 20; i++)
            delay_25ms();
    }

    return 0;
}

void calc_adc_temp_borders(void) {
//     adc_temperature_90 = EEPROM_read(ADDR_TEMPERATURE_90);
//     adc_temperature_100 = EEPROM_read(ADDR_TEMPERATURE_100);
//     adc_temperature_1_deg = EEPROM_read(ADDR_TEMPERATURE_1_DEG);

//     // adc_temperature_1_deg = (adc_temperature_90 - adc_temperature_100) / 10;

//     // if (adc_temperature_1_deg == 0) {
//     //     adc_temperature_1_deg = 1;
//     // }
}

void setup_ignition_int(void) {
    // PD4 - input
    PD4_INPUT();

    // int on 16-23 pins
    SET(PCICR, PCIE2);

    // only PCINT20 - PD4 pin
    SET(PCMSK2, PCINT20);
}

void setup_sleep_mode(void) {
    // Power-down sleep mode
    POWER_DOWN_SLEEP_MODE_INLINE();
    SLEEP_ENABLE();
}

void setup_pwm(void) {
    // PD6 - output
    PD6_OUTPUT();

    // phase correct pwm
    SET(TCCR0A, WGM00);

    // compare output mode
    SET(TCCR0A, COM0A1);

    // clock prescale 1
    SET_PWM_PRESCALE_1();
}

void setup_adc(void) {
    // ADC5 - ADC - PC5
    SET(ADMUX, MUX0);
    SET(ADMUX, MUX2);
    // 1.1 internal reference
    SET_ADC_1_1_REFERENCE();

    // left adjusted to use 8-bit ADC
    // SET(ADMUX, ADLAR);

    // set the prescaler to clock/128
    SET_ADC_CLOCK_PRESCALE_128();

    // ADC Interrupt Enable
    // SET(ADCSRA, ADIE);

    // disable digital input
    SET(DIDR0, ADC5D);

    // enable ADC
    ENABLE_ADC();
}

uint8_t map_temperature_to_duty(uint16_t adc_temperature, uint8_t is_ac_on) {
    // if (display_counter < 10) {
    //     send_uint16(adc_temperature, 0);
    // }

    uint8_t duty_index = 0;

    if (adc_temperature <= adc_temperature_90) {
        // analog of:
        // duty_index = (adc_temperature_90 - adc_temperature) / adc_temperature_1_deg;
        // because we have not hardware divider and don't trust pseudo int16 and __udivmodhi4
        // uint8_t temp = adc_temperature_90 - adc_temperature;
        // for (int8_t i = 0; i < 9 && temp >= adc_temperature_1_deg; i++) {
        //     temp -= adc_temperature_1_deg;
        //     duty_index++;
        // }
        duty_index = (adc_temperature_90 - adc_temperature) / adc_temperature_1_deg;
    }

    if (duty_index > 8) {
        duty_index = 8;
    }

    // if (is_ac_on) {
    //     return pgm_read_byte(&temperature_duty_ac_on_map[duty_index]);
    // }
    
    UART_PUTU(pgm_read_byte(&temperature_duty_ac_off_map[duty_index]));
    
    // if (display_counter >= 10 && display_counter < 15) {
    //     send_uint16_underscore(adc_temperature_90);
    // }
    // if (display_counter >= 15 && display_counter < 20) {
    //     send_uint16_underscore(duty_index);
    // }

    return pgm_read_byte(&temperature_duty_ac_off_map[duty_index]);
}

uint16_t read_temperature(void) {
    uint16_t sum = 0;
    uint16_t min = 1023;
    uint16_t max = 0;

    uint16_t val;

    for (int8_t i = 0; i < 10; i++)
    {
        val = adc_read();

        if (val > max) max = val;
        if (val < min) min = val;

        sum += val;

        delay_25ms();
    }

    sum = sum - min - max;

    return sum / 8;

    // uint16_t sum = adc_read();
    // sum += adc_read();
    // sum /= 2;

    // if (sum <= 150) {
    //     sum = 0;
    // } else {
    //     sum -= 150;
    // }

    // return sum;
}

void set_duty_smoothly(uint8_t duty) {
    while (PWM > duty)
    {
        // uint16_t adc_temp_16 = read_temperature();

        // send_uint16(adc_temp_16, 0);

        PWM--;
        delay_25ms();
    }

    while (PWM < duty)
    {
        // uint16_t adc_temp_16 = read_temperature();

        // send_uint16(adc_temp_16, 0);

        PWM++;
        delay_25ms();
    }
}
