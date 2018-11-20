#include "bus.h"
#include "lcd.h"
#include "jtag.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include "../spi/protocol.h"

#define ADC_PRESCALER_128 0x07

#define CNV_FRONT_MUL 17391
#define CNV_FRONT_EXP 1.071

#define CNV_SIZE_MUL 2680
#define CNV_SIZE_EXP 1.018

#define CHN_SENS_FRONT 0;
#define CHN_SENS_RIGHT 1;
//#define WHEEL_SENSOR ;


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
ISR(SPI_STC_vect) {
    // Code to execute
    // whenever transmission/reception
    // is complete.
    float data = sensors.dist_front; 
    spi_tranceive(&data, sizeof(sensors));
}

/*
	Convert voltage to distance (front sensor)
*/
float sensor_to_cm(uint16_t adc_val, uint8_t channel) {

    if(channel = CHN_SENS_FRONT) {
	return CNV_FRONT_EXP*pow(adc_val, -DIST_FRONT_EXP);

    } else {
        return CNV_SIDE_EXP*pow(adc_val, -CNV_SIDE_EXP);
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
    //volatile struct sens_values *values = {0};
    struct sens_data_frame sensors; 
    
    //lcd_init(); 
    DDRA = 0x00; 
    //lcd
    DDRD = 0xFF;

    //spi conf
    spi_init_slave();

    //port conf
    init_jtagport();

    // Enable global interrupts
    sei();

    // Setup A/D-converter
    adc_init();

    while(1){
	
	uint16_t adc_val =  adc_read(CHN_SENS_FRONT);	
	sensors.dist_front = sensor_to_cm(adc_val, CHN_SENS_FRONT);
	
	PORTD = sensors.dist_front;	   
    }
    return 0;
}
