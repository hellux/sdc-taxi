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

uint8_t spi_tranceive(uint8_t data){
    // Load data into the buffer
    SPDR = data;
 
    // Wait until transmission complete
    while(!(SPSR & (1<<SPIF) ));
 
    // Return received data
    return(SPDR);
}