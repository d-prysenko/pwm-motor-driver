#define F_CPU 1200000ul

#define __AVR_ATtiny13__

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "util.h"

#include "uart.h"


#define ADDR_TEMPERATURE_90 0
#define ADDR_TEMPERATURE_100 2

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


_inline_ void setup_calibration_int(void);
_inline_ void setup_ignition_int(void);
_inline_ void setup_sleep_mode(void);
_inline_ void setup_pwm(void);
_inline_ void setup_adc(void);

uint8_t map_temperature_to_duty(uint16_t adc_temperature, uint8_t is_ac_on);
void set_duty_smoothly(uint8_t duty);
void calc_adc_temp_borders(void);
void sleep(void);
void sleep_disable(void);

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

uint16_t adc_temperature_90 = 100;
uint16_t adc_temperature_100 = 0;
uint16_t adc_temperature_1_deg = 1;

uint8_t duty = 0;
uint8_t ac_on = 0;

volatile uint8_t calibration_number = 0;

volatile uint8_t ign_off = 0;


#define PWM OCR0A


// PB0 - pwm                 output
// PB1 - ignition            input
// PB3 - calibration button  input pull-up
// PB4 - temperature         input

// ignition int
ISR (PCINT0_vect) {
    ign_off = bit_is_clear(PINB, PINB1);
    // ign_off = 0;
}


int main(void) {
    // set frequency divider 1 (9.6 MHz)
    // SET_CPU_FREQ_DIV_1();

    // no pull-ups
    // SET(MCUCR, PUD);

    setup_pwm();
    setup_adc();
    setup_sleep_mode();
    setup_ignition_int();
    setup_calibration_int();

    // allow interrupts
    sei();

    // pwm counter - 0, duty - 0
    TCNT0 = 0;
    PWM = 0;

    ign_off = bit_is_clear(PINB, PINB1);
    // ign_off = 0;

    calc_adc_temp_borders();

    const uint8_t afterrun_delay_sec = 10;

    while (1)
    {
        if (ign_off) {
            set_duty_smoothly(duty);
            // PWM = duty;
            
            // wait 100 ms for quicker response if ignition will be turned on when doing after run delay
            for (uint16_t i = 0; (i < afterrun_delay_sec * 10) && ign_off; i++) {
                _delay_ms(100);
            }

            PWM = 0;
            _delay_ms(1);

            if (ign_off) {
                asm("sleep");
            }
        }

        if (bit_is_clear(PINB, PINB3)) {

            uint16_t temp = adc_read();

            if (!calibration_number) {
                EEPROM_write_uint16(ADDR_TEMPERATURE_90, temp);
                UART_PUTS("90 set ");
                blink_fast();
            } else {
                EEPROM_write_uint16(ADDR_TEMPERATURE_100, temp);
                UART_PUTS("100 set ");
                blink_slow();
            }

            UART_PUTU(temp);
            UART_PUTC(' ');
            
            calc_adc_temp_borders();

            calibration_number = !calibration_number;
        }

        uint16_t adc_temp = adc_read();
        uint8_t ac_on = 0;

        UART_PUTU(adc_temp);
        UART_PUTC(' ');

        duty = map_temperature_to_duty(adc_temp, ac_on);

        UART_PUTC('\n');
        UART_PUTC('\r');

        // PWM = duty;
        set_duty_smoothly(duty);
        
        _delay_ms(500);
    }

    return 0;
}

void calc_adc_temp_borders(void) {
    // adc_temperature_90 = EEPROM_read_uint16(ADDR_TEMPERATURE_90);
    // adc_temperature_100 = EEPROM_read_uint16(ADDR_TEMPERATURE_100);
    adc_temperature_90 = 600;
    adc_temperature_100 = 10;
    adc_temperature_1_deg = (adc_temperature_90 - adc_temperature_100) / 10;

    if (adc_temperature_1_deg == 0) {
        adc_temperature_1_deg = 1;
    }
}

void setup_ignition_int(void) {
    // PB1 - input
    PB1_INPUT();

    // int
    SET(GIMSK, PCIE);
    SET(PCMSK, PCINT1);
}

void setup_calibration_int(void) {
    // PB3 - input
    PB3_INPUT();
    PB3_PULLUP();

    // int0 enable
    // GIMSK |= (1 << INT0);
}

void setup_sleep_mode(void) {
    // Power-down sleep mode
    POWER_DOWN_SLEEP_MODE();
    SLEEP_ENABLE();
}

void setup_pwm(void) {
    // PB0 - output
    PB0_OUTPUT();

    // phase correct pwm
    SET(TCCR0A, WGM00);
    // UNSET(TCCR0A, WGM01);
    // UNSET(TCCR0B, WGM02);

    // compare output mode
    SET(TCCR0A, COM0A1);
    // UNSET(TCCR0A, COM0A0);

    // clock prescale 1
    SET(TCCR0B, CS00);
    // UNSET(TCCR0B, CS01);
    // UNSET(TCCR0B, CS02);
}

void setup_adc(void) {
    // PB4 - ADC
    SET(ADMUX, MUX1);
    // UNSET(ADMUX, REFS0);

    // right aligned to use 10-bit ADC
    // UNSET(ADMUX, ADLAR);

    // set the prescaler to clock/4
    // UNSET(ADCSRA, ADPS0);
    SET(ADCSRA, ADPS1);
    // UNSET(ADCSRA, ADPS2);

    // ADCSRB = 0;

    SET(DIDR0, ADC2D);

    // enable ADC
    SET(ADCSRA, ADEN);
}

uint8_t map_temperature_to_duty(uint16_t adc_temperature, uint8_t is_ac_on) {
    uint8_t duty_index = 0;

    if (adc_temperature < adc_temperature_90) {
        duty_index = (adc_temperature_90 - adc_temperature) / adc_temperature_1_deg;
    }

    if (duty_index > 8) {
        duty_index = 8;
    }

    if (is_ac_on) {
        return pgm_read_byte(&temperature_duty_ac_on_map[duty_index]);
    }
    
    UART_PUTU(pgm_read_byte(&temperature_duty_ac_off_map[duty_index]));
    
    return pgm_read_byte(&temperature_duty_ac_off_map[duty_index]);
}

void __attribute__ ((noinline)) delay_25ms() {
    _delay_ms(25);
}

void set_duty_smoothly(uint8_t duty) {
    while (PWM > duty)
    {
        PWM--;
        delay_25ms();
    }

    while (PWM < duty)
    {
        PWM++;
        delay_25ms();
    }
}
