#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>


void adc_init() {

	//mux init
	ADMUX = (1 << REFS0);

	//ADC enable
	ADCSRA |= (1 << ADEN);
	//Prescaler of 128. 16000000/128 = 125000
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

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


int main(int argc, char* args[]) {
	printf("sens module hej hej\n");

	return EXIT_SUCCESS;
}
/*
int main(void) {

	//ports_init?

	unsigned channel = MUX0;

	adc_init();

	//spi conf?

	// Enable global interrupts
	sei();


	// Setup A/D-converter
	adc_init();
}
*/