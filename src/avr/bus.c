#include "bus.h"

#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

void spi_init_slave(){
    //Set MISO as output
    DDRB |= (1<<PB6);

    //Enable SPI Interrupt & SPI    
    SPCR |= (1<<SPIE)|(1<<SPE);

    //Clear SPI Data
    SPDR = 0;
}

void spi_tranceive(uint8_t *data, int len) {
    for (int i = 0; i < len; i++) {
        SPDR = data[i];

        while(!(SPSR & (1<<SPIF)));

        data[i] = SPDR;
    }
}
