#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include "lcd.h"

#include "bus.h"
#include "protocol.h"

#define ADC_PRESCALER_128 0x07

#define CNV_FRONT_MUL 173.91
#define CNV_FRONT_EXP 1.071

#define CNV_RIGHT_MUL 26.80
#define CNV_RIGHT_EXP 1.018

#define CHN_SENS_FRONT 0
#define CHN_SENS_RIGHT 1

#define WHEEL_CIRCUM 0.265
#define WHEEL_N 10 /* number of interrupts per rotation */
#define WHEEL_N_DIST WHEEL_CIRCUM / WHEEL_N /* distance per measure */

#define WL_OCR OCR1A
#define WL_TCNT TCNT1

#define WT_PSC 64
#define WT_TOP 65535

#define SENS_SLAVE 0

const struct sens_data SENS_EMPTY = {0};

volatile struct sens_data sensors; 

void reset(void) {
    sensors = SENS_EMPTY;
}

void wheel_init(){
    /* setup timer for velocity calculation */
    TIMSK1 = (1<<OCIE1A);
    TCCR1B = (1<<WGM12)|(0<<CS12)|(1<<CS11)|(1<<CS10);
    TCCR1A = 0;
    WL_TCNT = 0;
    WL_OCR = WT_TOP;

    /* enable external interrupts */
    EIMSK = (1<<INT0)|(0<<INT1);
    /* Trigger on rising edge */
    EICRA = (1<<ISC01)|(1<<ISC00)|(1<<ISC11)|(1<<ISC10);
}

void adc_init(void) {   
    //mux init
    ADMUX = (1 << REFS0);

    //ADC enable
    ADCSRA |= (1 << ADEN);

    /* F_ADC = F_CPU/prescaler */
    ADCSRA |= ADC_PRESCALER_128;
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

/* left wheel timer expired */
ISR(TIMER1_COMPA_vect) {
    cli();
    sensors.velocity = 0;
    WL_TCNT = 0;
    sensors.updated = true;
    sei();
}

/* left wheel register */
ISR(INT0_vect) {
    cli();
    sensors.distance += WHEEL_CIRCUM/WHEEL_N;
    sensors.velocity = F_CPU*WHEEL_N_DIST/WT_PSC/WL_TCNT;
    sensors.updated = true;
    WL_TCNT = 0;
    sei();
}

ISR(SPI_STC_vect) {
    cli();
    uint8_t command = spi_accept(NULL, SPI_INSIDE_ISR, SENS_SLAVE);

    struct sens_data sensors_copy;
    switch (command) {
    case BBS_GET:
        sensors_copy = sensors;
        spi_return(command, (uint8_t*)&sensors_copy, sizeof(sensors_copy));
        sensors.updated = false;
        break;
    case BBS_RST:
        reset();
        break;
    }
    sei();
}

int main(void) {
    spi_init(SPI_ENABLE_INT);
    wheel_init();
    adc_init();
    reset();

    sei();

    while (1) {
        /* get new sensor values */
        uint16_t adc_front = adc_read(CHN_SENS_FRONT);
        uint16_t adc_right = adc_read(CHN_SENS_RIGHT);
        float dist_front = CNV_FRONT_MUL*pow(adc_front, -CNV_FRONT_EXP);
        float dist_right = CNV_RIGHT_MUL*pow(adc_right, -CNV_RIGHT_EXP);
        
        /* save local struct to global one */
        cli();
        sensors.dist_front = dist_front;
        sensors.dist_right = dist_right;
        sei();
    }

    return 0;
}
