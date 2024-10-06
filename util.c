#include "util.h"

void EEPROM_write(uint8_t ucAddress, uint8_t ucData)
{
    /* Wait for completion of previous write */
    while(EECR & (1 << EEPE));
    /* Set Programming mode */
    EECR = (0 << EEPM1) | (0 << EEPM0);
    /* Set up address and data registers */
    EEARL = ucAddress;
    EEDR = ucData;
    /* Write logical one to EEMPE */
    EECR |= (1 << EEMPE);
    /* Start eeprom write by setting EEPE */
    EECR |= (1 << EEPE);
}

uint8_t EEPROM_read(uint8_t ucAddress) {
    /* Wait for completion of previous write */
    while(EECR & (1 << EEPE));
    /* Set up address register */
    EEARL = ucAddress;
    /* Start eeprom read by writing EERE */
    EECR |= (1 << EERE);
    /* Return data from data register */
    return EEDR;
}

uint8_t adc_read(void) {
    // Start the conversion
    ADCSRA |= (1 << ADSC);

    // Wait for it to finish - blocking
    while (ADCSRA & (1 << ADSC));

    return ADCH;
}

void blink_fast(void) {
    for (uint8_t i = 0; i < 7; i++) {
        OCR0A = 255;
        _delay_ms(200);
        OCR0A = 0;
        _delay_ms(200);
    }
}

void blink_slow(void) {
    for (uint8_t i = 0; i < 5; i++) {
        OCR0A = 255;
        _delay_ms(700);
        OCR0A = 0;
        _delay_ms(400);
    }
}
