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

void EEPROM_write_uint16(uint8_t ucAddress, uint16_t ucData) {
    EEPROM_write(ucAddress, (uint8_t)(ucData >> 8));
    EEPROM_write(ucAddress + 1, (uint8_t)(ucData & 0xFF));
}

uint16_t EEPROM_read_uint16(uint8_t ucAddress) {
    return ((uint16_t)EEPROM_read(ucAddress) << 8) | EEPROM_read(ucAddress + 1);
}

uint16_t adc_read(void) {
    // Start the conversion
    ADCSRA |= (1 << ADSC);

    // Wait for it to finish - blocking
    while (ADCSRA & (1 << ADSC));

    return ADC;
}

void __attribute__ ((noinline)) delay_200(void) {
    _delay_ms(200);
}

void blink_fast(void) {
    // PB0_OUTPUT();

    for (uint8_t i = 0; i < 7; i++) {
        // PB0_ON();
        OCR0A = 255;
        delay_200();
        OCR0A = 0;
        // PB0_OFF();
        delay_200();
    }
}

void blink_slow(void) {
    // PB2_OUTPUT();

    for (uint8_t i = 0; i < 5; i++) {
        // PB0_ON();
        OCR0A = 255;
        delay_200();
        delay_200();
        delay_200();
        delay_200();
        // PB0_OFF();
        OCR0A = 0;
        delay_200();
        delay_200();
    }
}

