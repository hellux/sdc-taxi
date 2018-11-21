#include "bus.h"
#include "lcd.h"
#include "jtag.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include "../spi/protocol.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define PI acos(-1.0)

#define ADC_PRESCALER_128 0x07

#define CNV_FRONT_MUL 17391
#define CNV_FRONT_EXP 1.071

#define CNV_SIDE_MUL 2680
#define CNV_SIDE_EXP 1.018

#define CHN_SENS_FRONT 0
#define CHN_SENS_SIDE 1
#define CHN_SENS_WHEEL 2

#define WHEEL_DIAMETER 0.08
#define SENS_PULS_ROTATION 5

volatile struct sens_data_frame sensors; 
volatile float wheel_sensor_cntr;


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

//INT0 -> PD2
ISR(INT0_vect) {
    cntr_sensor_wheel++;        
}
//INT1 -> PD3
ISR(INT1_vect) {
    cntr_sensor_wheel++;
}

// SPI Transmission/reception complete ISR
ISR(SPI_STC_vect) {
    cli();
    // Code to execute
    // whenever transmission/reception
    // is complete.
    struct sensors_data_frame sensors_copy = *sensors;
    uint8_t cmd;
    spi_tranceive(&cmd, sizeof(cmd));

    if (cmd == BCB_GET_SENS) {
	spi_tranceive((uint8_t*)&sensors_copy, sizeof(sensors));
    } else { // cmd == BCB_RESET
	sensors = {0};
    }
    sei();
}

/*
	Convert voltage to distance (front sensor)
*/
float sensor_to_cm(uint16_t adc_val, uint8_t channel) {

    if (channel == CHN_SENS_FRONT) {
	return CNV_FRONT_EXP*pow(adc_val, -CNV_FRONT_EXP);

    } else {
        return CNV_SIDE_EXP*pow(adc_val, -CNV_SIDE_EXP);
    }
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

/*
    Possible/temporary function to send data to the led
*/
void sensor_to_led(uint8_t channel) {

    if(channel == CHN_SENS_FRONT) {
	PORTD = sensors.dist_front;

    } else if (channel == CHN_SENS_SIDE) {
	PORTD = sensors.dist_side;		
    
    } else {
        PORTD = sensors.dist_wheel;
    }
}

void sensor_data_handler(uint8_t channel) {

    if (channel == CHN_SENS_FRONT || channel == CHN_SENS_SIDE) {
	uint16_t adc_val =  adc_read(channel);
        sensors.dist_front = sensor_to_cm(adc_val, channel);
        sensor_to_led(channel); //can be implemented if needed   

    } else {
	float wheel_dist = ((PI*WHEEL_DIAMETER) / (SENS_PULS_ROTATION*wheel_sensor_cntr));
	sensors.dist_wheel = wheel_dist;
	sensor_to_led(CHN_SENS_WHEEL);
    }   
} 


int main(void) {
     
    //lcd
    DDRD = 0xCF;

    EIMSK = 1<<INT0|1<<INT1;
    
    // Trigger on rising edge
    EICRA = 1<<ISC01|1<<ISC00|1<<ISC11|1<<ISC10;		

    //spi conf
    spi_init_slave();

    //port conf
    init_jtagport();

    // Enable global interrupts
    sei();

    // Setup A/D-converter
    adc_init();

    while(1) {
	
	cli();
	sensor_data_handler(CHN_SENS_FRONT);
	sensor_data_handler(CHN_SENS_SIDE);
	sensor_data_handler(CHN_SENS_WHEEL);
        sei();	   
    }
    return 0;
}
