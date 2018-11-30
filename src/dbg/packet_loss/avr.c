#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

/*
ISR(SPI_STC_vect){
    cli();
    sei();
}
*/

uint8_t tranceive(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1<<SPIF)));
    return SPDR;
}

int main(void) {
    DDRB |= (1<<PB6);
    SPCR |= (0<<SPIE)|(1<<SPE);
    SPDR = 0;

    sei();
    while (1) {
        uint8_t start = 0x9a;
        uint8_t cmd = tranceive(start);
        if (cmd != 0x01)
            continue;
        uint8_t data[2];
        data[0] = tranceive(cmd);
        data[1] = tranceive(data[0]);
        tranceive(data[1]);
    }
}
