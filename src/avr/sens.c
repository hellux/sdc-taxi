#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include "lcd.h"

#include "bus.h"
#include "../spi/protocol.h"

#define ADC_PRESCALER_128 0x07

#define CNV_FRONT_MUL 17.391
#define CNV_FRONT_EXP 1.071

#define CNV_RIGHT_MUL 2.680
#define CNV_RIGHT_EXP 1.018

#define CHN_SENS_FRONT 0
#define CHN_SENS_RIGHT 1

#define WHEEL_CIRCUM 0.265
#define WHEEL_N 10 /* number of interrupts per rotation */

const struct sens_data SENS_EMPTY = {0};

volatile struct sens_data sensors; 
volatile unsigned wheel_sensor_cntr;

void reset(void) {
    sensors = SENS_EMPTY;
}

void wheel_init(){
    /* enable interrupts? */
    EIMSK = (1<<INT0)|(1<<INT1);
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

ISR(INT0_vect) {
    cli();
    wheel_sensor_cntr++;
    sei();
}

ISR(INT1_vect) {
    cli();
    wheel_sensor_cntr++;
    sei();
}

ISR(SPI_STC_vect) {
    cli();
    uint8_t command = spi_accept(NULL, SPI_INSIDE_ISR);
    char buf[16];

    if (command == BBS_GET) {
        struct sens_data sensors_copy = sensors;
        spi_return(command, (uint8_t*)&sensors_copy, sizeof(sensors_copy));
    } else if (command == BBS_RST) {
        reset();
    }
    sei();
}

int main(void) {
    spi_init(SPI_ENABLE_INT);
    wheel_init();
    adc_init();
    reset();

#ifdef DEBUG
    DDRA = 0xFC;
    lcd_init();
    lcd_clear();
#endif

    sei();

    struct sens_data sens_local;
    while (1) {
        /* get new sensor values */
        cli();
        unsigned wheel_cntr = wheel_sensor_cntr;
        sei();
        uint16_t adc_front = adc_read(CHN_SENS_FRONT);
        uint16_t adc_right = adc_read(CHN_SENS_RIGHT);

        /* write to local struct */
        sens_local.dist_front = CNV_FRONT_MUL*pow(adc_front, -CNV_FRONT_EXP);
        sens_local.dist_right = CNV_RIGHT_MUL*pow(adc_right, -CNV_RIGHT_EXP);
        sens_local.distance = WHEEL_CIRCUM/WHEEL_N*wheel_cntr / 2;

        /* save local struct to global one */
        cli();
        sensors = sens_local;
        sei();
        
#ifdef DEBUG
        /* write values to lcd */
        /*
        char buf[16];
        lcd_set_ddram(0);
        lcd_send_string("F:");
        dtostrf(sens_local.dist_front, 5, 2, buf);
        lcd_send_string(buf);
        lcd_set_ddram(ROW2ADR);
        dtostrf(sens_local.dist_right, 5, 2, buf);
        lcd_send_string("R:");
        lcd_send_string(buf);
        lcd_send_string(" D:");
        dtostrf(sens_local.distance, 5, 2, buf);
        lcd_send_string(buf);
        */
#endif
    }
    return 0;
}
