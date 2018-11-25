#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

#include "common.h"

void spi_tranceive(uint8_t *data, int len) {
    for (int i = 0; i < len; i++) {
        SPDR = data[i];

        while(!(SPSR & (1<<SPIF)));

        data[i] = SPDR;
    }
}

ISR(SPI_STC_vect){
    cli();

    uint8_t data[100];
    /* retrieve cs and data */
    cs_t cs;
    spi_tranceive((uint8_t*)&cs, 1);
    spi_tranceive(data, DATA_LENGTH);

    /* acknowledge if valid checksum */
    uint8_t ack = ACK;
    if (!cs_check(cs, data, DATA_LENGTH))
        ack = ~ack;
    spi_tranceive(&ack, sizeof(ack));

    sei();
}

int main(void) {
    DDRB |= (1<<PB6);
    SPCR |= (1<<SPIE)|(1<<SPE);
    SPDR = 0;

    sei();
    while (1);
}
