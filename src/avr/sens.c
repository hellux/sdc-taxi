#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include "bus.h"
#include "../spi/protocol.h"

#define ADC_PRESCALER_128 0x07

#define CNV_FRONT_MUL 17391
#define CNV_FRONT_EXP 1.071

#define CNV_RIGHT_MUL 2680
#define CNV_RIGHT_EXP 1.018

#define CHN_SENS_FRONT 0
#define CHN_SENS_RIGHT 1

#define WHEEL_DIAM 0.08
#define WHEEL_SENS_FREQ 5
#define PI 3.14

const struct sens_data SENS_EMPTY = {0};

volatile struct sens_data sensors; 
volatile unsigned wheel_sensor_cntr;

void reset(void) {
    sensors = SENS_EMPTY;
}

void adc_init(void) {
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
    ADCSRA |= (1 << ADSC);

    // wait until it is ready (=0)
    while (ADCSRA & (1 << ADSC));

    return (ADC); //ADCL-ADCH
}

ISR(INT0_vect) {
    wheel_sensor_cntr++;        
}

ISR(INT1_vect) {
    wheel_sensor_cntr++;
}

ISR(SPI_STC_vect) {
    cli();
    uint8_t command = spi_accept(NULL, SENS_ACK);

    if (command == BBS_GET) {
        struct sens_data sensors_copy = sensors;
        spi_return(command, (uint8_t*)&sensors_copy, sizeof(sensors_copy));
    } else if (command == BBS_RST) {
        reset();
    }
    sei();
}

/*
    other conversion formula:
    d = (1/a) / (ADC + B) - k, where
    d - dist in cm,
    k - corrective constant,
    ADC - digitalized value of voltage,
    a - linear member (value determined by the trend line equation),
    b - free memebr (value determined by the trend line equation)
*/

int main(void) {
    /* enable interrupts? */
    EIMSK = (1<<INT0)|(1<<INT1);
    /* Trigger on rising edge */
    EICRA = (1<<ISC01)|(1<<ISC00)|(1<<ISC11)|(1<<ISC10);

    spi_init();
    adc_init();
    reset();

    sei();

    struct sens_data sens_local;
    while (1) {
        uint16_t adc_front = adc_read(CHN_SENS_FRONT);
        sens_local.dist_front = CNV_FRONT_EXP*pow(adc_front, -CNV_FRONT_EXP);

        uint16_t adc_right = adc_read(CHN_SENS_RIGHT);
        sens_local.dist_right = CNV_RIGHT_EXP*pow(adc_right, -CNV_RIGHT_EXP);

        /* sample wheel counter */
        cli();
        unsigned wheel_cntr = wheel_sensor_cntr;
        sei();

        sens_local.distance = PI*WHEEL_DIAM/WHEEL_SENS_FREQ*wheel_cntr;

        /* update shared struct */
        cli();
        sensors = sens_local;
        sei();
    }
    return 0;
}
