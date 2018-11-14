#include "bus.h"

#include "const.h"

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

uint8_t spi_slave_recieve(){
    //Wait for completed
    while (!(SPSR & (1<<SPIF)));
    return SPDR;
}    

int main(int argc, char* args[]) {
    return EXIT_SUCCESS;
}