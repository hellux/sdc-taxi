#include "bus.h"
#include "lcd.h"
#include "jtag.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#define ADC_PRESCALER_128 0x07

#define DIST_FRONT_MUL 17391
#define DIST_FRONT_EXP 1.071

#define DIST_RIGHT_MUL 2680
#define DIST_RIGHT_EXP 1.018

struct sens_values {
    uint16_t dist_front;    // distance to object (front)
    uint16_t dist_side;    // distance to object (side)
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

// SPI Transmission/reception complete ISR
ISR(SPI_STC_vect)
{
    // Code to execute
    // whenever transmission/reception
    // is complete.
}

/*
	Convert voltage to distance (front sensor)
*/
float ir_front_sensor(uint16_t adc_val){
	return DIST_FRONT_EXP*pow(adc_val, -DIST_FRONT_EXP);
}

/*
	other formula:
	d = (1/a) / (ADC + B) - k, where
	d - dist in cm,
	k - corrective constant,
	ADC - digitalized value of voltage,
	a - linear member (value determined by the trend line equation),
	b - free memebr (value determined by the trend line equation)
*/



int main(void) {
    volatile struct sens_values *values = {0};

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

    while(1){
    }
    return 0;
}
