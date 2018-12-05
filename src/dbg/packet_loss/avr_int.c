#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

uint8_t tranceive(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1<<SPIF)));
    return SPDR;
}

ISR(SPI_STC_vect) {
    cli();
    uint8_t cmd = SPDR;
    if (cmd == 0x01) {
        uint8_t data[2];
        data[0] = tranceive(cmd);
        data[1] = tranceive(data[0]);
        tranceive(data[1]);
    }
    sei();
}

int main(void) {
    DDRB |= (1<<PB6);
    SPCR |= (1<<SPIE)|(1<<SPE);
    SPDR = 0;

    sei();
    while (1) {
    }
}
