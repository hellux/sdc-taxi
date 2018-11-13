#include "const.h"

#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

typedef struct {
    float kp;           //Constant Pro
    float kd;           //Constant Der
    float err;          //Error
    float last_err;     //Previous error
} pd_values;

void pwm_init(){
    //Initialize to phase and frequency correct PWM
    TCCR1A |= (1<<WGM10)|(1<<COM1A1)|(1<<COM1B1);
    TCCR1B |= (1<<WGM13)|(1<<CS10);
    
    //Set PD4, PD5 to outputs
    DDRD |= (1<<PD4)|(1<<PD5);
}

void spi_init_slave(){
    //Set MISO as output
    DDRB |= (1<<PB6);

    //Enable SPI Interrupt & SPI    
    SPCR |= (1<<SPIE)|(1<<SPE);

    //Clear SPI Data
    SPDR = 0;
}

uint8_t spi_slave_recieve(){
    //Wait for completed
    while (!(SPSR & (1<<SPIF)));
    return SPDR;
}    

void init_lcdports(){
    //Set ouput ports to LCD
    DDRA |= (1<<PA4)|(1<<PA5)|(1<<PA6)|(1<<PA7);
    DDRB |= (1<<PB0)|(1<<PB1);
}

void init_jtagport(){
    //Set TDO to output
    DDRC |= (1<<PC4);
}

float pd_ctrl(pd_values *v){
    float proportion;
    float derivative;
    proportion = v->err                 * v->kp;
    derivative = v->err - v->last_err   * v->kd;
    v->last_err = v->err;

    return proportion + derivative;
}

int main(int argc, char* args[]) {
    uint8_t duty_vel = 0;
    uint8_t duty_rad = 0;

    pd_values vel;
    pd_values rad;

    pwm_init();
    spi_init_slave();
    init_lcdports();

    //Enable interrupt
    sei();
    
    while(1){
        
        //vel->err = value sent from comm Velocity
        //rad->err = value sent from comm Radius
    
        duty_vel = pd_ctrl(&vel);
        duty_rad = pd_ctrl(&rad);

        OCR1A = duty_rad;
        OCR1B = duty_vel;
    }
    
    return 0;
}
