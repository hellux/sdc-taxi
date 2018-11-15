#include "bus.h"
#include "lcd.h"
#include "jtag.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#define ADC_PRESCALER_128 0x07

struct sens_values {
    uint16_t dist_front;    // distance to object (front)
    uint16_t dist_side;     // distance to object (side)
    uint8_t rotations;      // Wheel rotations since last tranceive
};

void adc_init() {
	//mux init
	ADMUX = (1 << REFS0);

	//ADC enable
	ADCSRA |= (1 << ADEN);

	/* F_ADC = F_CPU/prescaler */
	ADCSRA |= ADC_PRESCALER_128;

	//ADC interrupt enabled
	ADCSRA |= (1 << ADIE);
}

uint16_t adc_read(uint8_t channel) {
	//Set multiplexer for given channel (0-7) (front or back sensor)
	ADMUX = (ADMUX & 0xF8) | channel; //ADMUX &= 0xE0; //Clear the older channel that was read?

	// start single convertion
	// write 1 to ADSC
	ADCSRA |= (1 << ADSC);

	// wait until it is ready (=0)
	while (ADCSRA & (1 << ADSC));

	return (ADC); //ADCL-ADCH
}

//Interrupt SPI vectorwill
//Trigger after SPI-tansmission
//ISR(SPI_STC_vect){}

int main(void) {
    volatile struct sens_values values = {0};

	unsigned channel = MUX0;

	adc_init();

	//spi conf
    spi_init_slave();
    //port conf
    init_jtagport();
    init_lcdports();

	// Enable global interrupts
	sei();

	// Setup A/D-converter
	adc_init();

    while(true){
    }
    return 0;
}
