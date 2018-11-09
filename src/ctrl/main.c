//#include <stdlib.h>
//#include <stdio.h>

#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>


void pwm_init(){
    
    //Initialize to phase and frequency correct PWM
    TCCR1A |= (1<<WGM01)|(1<<WGM00)|(1<<COM1A1)|(1<<COM1B1);
    TCCR1B |= (1<<WGM11)|(1<<CS00);
    
    //Set PD4, PD5 to outputs
    DDRD |= (1<<PD4)|(1<<PD5);
}

void spi_init_slave(){
    //Set MISO as output
    DDRB |= (1<<PB6);

    //Enable SPI    
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
    uint8_t duty_v;
    uint8_t duty_r;

    //printf("ctrl module hej hej\n");
    pwm_init();
    spi_init_slave();

    while(1){
        OCR1A = duty_r;
        OCR1B = duty_v;
    }
    
    //return EXIT_SUCCESS;
}
